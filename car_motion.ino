// Car gate opener: the Nano powers up when the car starts, watches its built-in
// accelerometer, and - once the car actually starts moving - replays Garage.sub
// over and over for SEND_WINDOW_MS (~1 minute), then shuts itself down (stops
// transmitting) until the next power-up.
//
//   power-up -> settle -> calibrate rest -> ARMED (wait for motion)
//            -> MOVING -> transmit for 1 min -> STOP (idle)
//
// Board: Arduino Nano 33 IoT (built-in LSM6DS3 IMU on internal I2C)
// Libraries: Arduino_LSM6DS3, SmartRC-CC1101-Driver-Lib
// CC1101 wiring is the same as transmitter/transmitter.ino:
//   SCK->D13  MISO->D12  MOSI->D11  CSN->D10  GDO0->D6  VCC->3V3  GND->GND
//
// Motion: an accelerometer cannot see constant speed, only acceleration - but a
// car that starts moving always accelerates. After power-up the board averages
// the low-pass-filtered acceleration into a "rest" vector (gravity at whatever
// mounting angle), then flags motion when the deviation from rest stays above
// MOVE_THRESH_G for MOVE_HOLD_MS. A door slam is too short to count.
//
// Logs (Serial, 115200), deviations in milli-g (1000 mg = 1 g). Events are kept
// in RAM and replayed in full whenever a monitor connects (or you send a key),
// so you can plug in later and still see the boot. Watch with car_motion/logs.sh.
//
// NOTE on repeating the signal: many gate receivers treat each command as a
// toggle (open / stop / close). Firing it continuously can open the gate and
// then immediately stop or close it. SEND_GAP_MS spaces the replays out; raise
// it if the gate misbehaves, or shorten SEND_WINDOW_MS.

#include <Arduino_LSM6DS3.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <stdarg.h>
#include "garage_signal.h"

// ---------------- motion tuning ----------------
const uint32_t SETTLE_MS     = 3000;    // ignore everything right after power-up
const uint32_t CALIB_MS      = 2000;    // averaging window for the rest vector
const float    LP_ALPHA      = 0.05f;   // EMA per sample @104 Hz -> tau ~0.2 s
const float    MOVE_THRESH_G = 0.10f;   // deviation from rest that counts as moving
const uint32_t MOVE_HOLD_MS  = 600;     // must stay above threshold this long
const uint32_t DROPOUT_MS    = 150;     // short dips below threshold are tolerated
const float    DRIFT_ALPHA   = 0.0005f; // slow rest-vector tracking while still (tau ~20 s)

// ---------------- transmit behaviour ----------------
const uint32_t SEND_WINDOW_MS = 60000;  // once moving, keep transmitting this long
const uint32_t SEND_GAP_MS    = 2000;   // pause between one replay and the next

// ---------------- logging ----------------
const uint32_t STATUS_MS     = 1000;    // status line period while ARMED, 0 = off

// ---------------- CC1101 ----------------
const byte CC_SCK  = 13;
const byte CC_MISO = 12;
const byte CC_MOSI = 11;
const byte CC_GDO0 = 6;    // we drive this to key TX

const float FREQ_MHZ = 433.92;

// ======================= logger =======================
// Events go to a RAM ring buffer + Serial (if a monitor is open). On SAMD
// opening the monitor does NOT reset the board, so without the buffer the boot
// messages would be lost.
const uint8_t HIST_LINES = 48;
const uint8_t LINE_LEN   = 96;
char    hist[HIST_LINES][LINE_LEN];
uint8_t histHead = 0, histCount = 0;
uint16_t histDropped = 0;
bool    serialWasOpen = false;

static int mg(float g) { return (int)lroundf(g * 1000.0f); }

static void vlog(bool store, const char *fmt, va_list ap) {
  char line[LINE_LEN];
  uint32_t t = millis();
  int n = snprintf(line, sizeof line, "[%5lu.%03lu] ",
                   (unsigned long)(t / 1000), (unsigned long)(t % 1000));
  vsnprintf(line + n, sizeof line - n, fmt, ap);

  if (store) {
    if (histCount == HIST_LINES) histDropped++;
    strncpy(hist[histHead], line, LINE_LEN);
    histHead = (histHead + 1) % HIST_LINES;
    if (histCount < HIST_LINES) histCount++;
  }
  if (Serial) Serial.println(line);   // never block when nobody is listening
}

void logEvent(const char *fmt, ...) {   // stored + printed
  va_list ap; va_start(ap, fmt); vlog(true, fmt, ap); va_end(ap);
}
void logLive(const char *fmt, ...) {    // printed only
  if (!Serial) return;
  va_list ap; va_start(ap, fmt); vlog(false, fmt, ap); va_end(ap);
}

void dumpHistory() {
  Serial.println();
  Serial.println("===== log since power-up =====");
  if (histDropped) {
    Serial.print("(");
    Serial.print(histDropped);
    Serial.println(" older events dropped)");
  }
  uint8_t start = (histHead + HIST_LINES - histCount) % HIST_LINES;
  for (uint8_t i = 0; i < histCount; i++) Serial.println(hist[(start + i) % HIST_LINES]);
  Serial.println("===== live =====");
}

// Replay history when a monitor connects, or on any key.
void serviceSerial() {
  bool open = (bool)Serial;
  bool keyed = false;
  while (Serial.available()) { Serial.read(); keyed = true; }
  if ((open && !serialWasOpen) || keyed) dumpHistory();
  serialWasOpen = open;
}

const char *resetCause() {
#if defined(ARDUINO_ARCH_SAMD)
  uint8_t r = PM->RCAUSE.reg;
  if (r & PM_RCAUSE_POR)   return "power-on";
  if (r & PM_RCAUSE_BOD12) return "brown-out 1.2V";
  if (r & PM_RCAUSE_BOD33) return "brown-out 3.3V (supply dipped!)";
  if (r & PM_RCAUSE_EXT)   return "reset button";
  if (r & PM_RCAUSE_WDT)   return "watchdog";
  if (r & PM_RCAUSE_SYST)  return "software (upload)";
#endif
  return "unknown";
}

// ======================= radio =======================
static inline void waitUs(uint32_t us) {
  uint32_t start = micros();
  while ((uint32_t)(micros() - start) < us) { /* spin */ }
}

void configureTx() {
  ELECHOUSE_cc1101.setSpiPin(CC_SCK, CC_MISO, CC_MOSI, 10);
  ELECHOUSE_cc1101.setGDO0(CC_GDO0);

  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setCCMode(0);
  ELECHOUSE_cc1101.setModulation(2);    // ASK/OOK
  ELECHOUSE_cc1101.setMHZ(FREQ_MHZ);
  ELECHOUSE_cc1101.setPA(12);
  ELECHOUSE_cc1101.setSyncMode(0);
  ELECHOUSE_cc1101.setCrc(0);
  ELECHOUSE_cc1101.setManchester(0);
  ELECHOUSE_cc1101.setWhiteData(0);
  ELECHOUSE_cc1101.setPktFormat(3);     // async serial: data in on GDO0
}

void sendSignal() {
  pinMode(CC_GDO0, OUTPUT);
  digitalWrite(CC_GDO0, LOW);
  ELECHOUSE_cc1101.SetTx();

  for (uint16_t i = 0; i < GARAGE_SIGNAL_LEN; i++) {
    int32_t d = GARAGE_SIGNAL[i];
    digitalWrite(CC_GDO0, d > 0 ? HIGH : LOW);
    waitUs(d > 0 ? (uint32_t)d : (uint32_t)(-d));
  }

  digitalWrite(CC_GDO0, LOW);
  ELECHOUSE_cc1101.setSidle();
}

// ======================= state machine =======================
enum State { SETTLING, CALIBRATING, ARMED, SENDING, DONE };
State state = SETTLING;
void enterState(State s);     // explicit prototypes: stop the IDE from auto-
const char *stateName(State s);  // generating them above the enum declaration
const char *stateName(State s) {
  switch (s) {
    case SETTLING:    return "SETTLE";
    case CALIBRATING: return "CALIB";
    case ARMED:       return "ARMED";
    case SENDING:     return "SEND";
    default:          return "DONE";
  }
}

float lpX, lpY, lpZ;          // low-passed acceleration, g
bool  lpInit = false;
float g0X, g0Y, g0Z;          // rest vector, g
float sumX, sumY, sumZ;
uint32_t sumN = 0;

float vibAcc = 0;             // sum of |raw - lp|^2 for the vibration level
uint32_t vibN = 0;

uint32_t tStateStart = 0;
bool     above = false;
uint32_t aboveSince = 0, lastAbove = 0;
float    candPeak = 0;
uint32_t candCount = 0;
float    winMax = 0;          // peak dev in the current status window
uint32_t lastStatus = 0;

uint32_t sendStart = 0, txCount = 0;

void enterState(State s) {
  state = s;
  tStateStart = millis();
}

// Transmit the whole window, with a responsive pause between replays. Keeping
// this in one blocking call is fine: once the car is moving we only transmit.
void runSendWindow() {
  sendStart = millis();
  txCount = 0;
  logEvent("SEND   moving -> transmitting for %lu s", (unsigned long)(SEND_WINDOW_MS / 1000));

  while ((uint32_t)(millis() - sendStart) < SEND_WINDOW_MS) {
    txCount++;
    uint32_t t0 = millis();
    sendSignal();
    logEvent("TX     #%lu at +%lu.%03lu s, took %lu ms",
             (unsigned long)txCount,
             (unsigned long)((t0 - sendStart) / 1000), (unsigned long)((t0 - sendStart) % 1000),
             (unsigned long)(millis() - t0));

    uint32_t gapEnd = millis() + SEND_GAP_MS;
    while (millis() < gapEnd && (uint32_t)(millis() - sendStart) < SEND_WINDOW_MS) {
      serviceSerial();
      delay(10);
    }
  }
  logEvent("STOP   window over after %lu TX - idle until next power-up", (unsigned long)txCount);
  enterState(DONE);
}

void setup() {
  Serial.begin(115200);       // no while(!Serial): in the car nobody is listening

  logEvent("BOOT   car_motion, reset cause: %s", resetCause());
  logEvent("CFG    thresh %d mg, hold %lu ms, settle %lu ms, calib %lu ms",
           mg(MOVE_THRESH_G), (unsigned long)MOVE_HOLD_MS,
           (unsigned long)SETTLE_MS, (unsigned long)CALIB_MS);
  logEvent("CFG    send window %lu s, gap %lu ms",
           (unsigned long)(SEND_WINDOW_MS / 1000), (unsigned long)SEND_GAP_MS);

  configureTx();              // Init() starts SPI, so check the radio after it
  if (ELECHOUSE_cc1101.getCC1101()) logEvent("RADIO  CC1101 OK, %d.%02d MHz",
                                             (int)FREQ_MHZ, (int)lroundf(FREQ_MHZ * 100) % 100);
  else logEvent("RADIO  CC1101 NOT FOUND - check wiring (VCC must be 3V3)");

  uint8_t tries = 0;
  while (!IMU.begin()) {
    if (++tries == 1 || tries % 10 == 0) logEvent("IMU    init failed (try %u), retrying...", tries);
    delay(1000);
    serviceSerial();
  }
  logEvent("IMU    LSM6DS3 OK, %d Hz", (int)IMU.accelerationSampleRate());

  enterState(SETTLING);
  logEvent("SETTLE waiting %lu ms", (unsigned long)SETTLE_MS);
}

void loop() {
  serviceSerial();
  if (state == DONE) return;

  if (!IMU.accelerationAvailable()) return;
  float x, y, z;
  if (!IMU.readAcceleration(x, y, z)) return;

  if (!lpInit) {
    lpX = x; lpY = y; lpZ = z;
    lpInit = true;
  } else {
    lpX += LP_ALPHA * (x - lpX);
    lpY += LP_ALPHA * (y - lpY);
    lpZ += LP_ALPHA * (z - lpZ);
  }
  float vx = x - lpX, vy = y - lpY, vz = z - lpZ;
  vibAcc += vx * vx + vy * vy + vz * vz;
  vibN++;

  uint32_t now = millis();
  float dev = 0;

  switch (state) {
    case SETTLING:
      if (now - tStateStart >= SETTLE_MS) {
        sumX = sumY = sumZ = 0;
        sumN = 0;
        vibAcc = 0; vibN = 0;
        enterState(CALIBRATING);
        logEvent("CALIB  measuring rest vector for %lu ms - keep still", (unsigned long)CALIB_MS);
      }
      break;

    case CALIBRATING:
      sumX += lpX; sumY += lpY; sumZ += lpZ;
      sumN++;
      if (now - tStateStart >= CALIB_MS) {
        g0X = sumX / sumN; g0Y = sumY / sumN; g0Z = sumZ / sumN;
        float g = sqrtf(g0X * g0X + g0Y * g0Y + g0Z * g0Z);
        int vib = mg(sqrtf(vibAcc / (vibN ? vibN : 1)));
        logEvent("CALIB  rest (%d, %d, %d) mg, |g| %d mg, vibration %d mg, %lu samples",
                 mg(g0X), mg(g0Y), mg(g0Z), mg(g), vib, (unsigned long)sumN);
        if (g < 0.9f || g > 1.1f)
          logEvent("WARN   |g| far from 1000 mg - board was moving during calibration?");
        above = false;
        winMax = 0;
        enterState(ARMED);
        logEvent("ARMED  waiting for the car to move (thresh %d mg)", mg(MOVE_THRESH_G));
      }
      break;

    case ARMED: {
      float dx = lpX - g0X, dy = lpY - g0Y, dz = lpZ - g0Z;
      dev = sqrtf(dx * dx + dy * dy + dz * dz);
      if (dev > winMax) winMax = dev;

      if (dev > MOVE_THRESH_G) {
        if (!above) {
          above = true;
          aboveSince = now;
          candPeak = 0;
          candCount++;
          logEvent("MOTION candidate #%lu: dev %d mg", (unsigned long)candCount, mg(dev));
        }
        lastAbove = now;
        if (dev > candPeak) candPeak = dev;
        if (now - aboveSince >= MOVE_HOLD_MS) {
          logEvent("MOVING candidate #%lu held %lu ms, peak %d mg",
                   (unsigned long)candCount, (unsigned long)(now - aboveSince), mg(candPeak));
          enterState(SENDING);
          runSendWindow();      // blocks for the full minute, then -> DONE
          return;
        }
      } else {
        if (above && now - lastAbove > DROPOUT_MS) {
          above = false;
          logEvent("MOTION candidate #%lu dropped after %lu ms (need %lu), peak %d mg",
                   (unsigned long)candCount, (unsigned long)(lastAbove - aboveSince),
                   (unsigned long)MOVE_HOLD_MS, mg(candPeak));
        }
        // Clearly at rest: let the rest vector follow slow drift, but never
        // while a candidate is building.
        if (!above && dev < MOVE_THRESH_G * 0.5f) {
          g0X += DRIFT_ALPHA * (lpX - g0X);
          g0Y += DRIFT_ALPHA * (lpY - g0Y);
          g0Z += DRIFT_ALPHA * (lpZ - g0Z);
        }
      }

      if (STATUS_MS && now - lastStatus >= STATUS_MS) {
        lastStatus = now;
        int vib = mg(sqrtf(vibAcc / (vibN ? vibN : 1)));
        logLive(".      ARMED  dev %3d  max %3d / %d mg   vib %3d mg",
                mg(dev), mg(winMax), mg(MOVE_THRESH_G), vib);
        winMax = 0;
        vibAcc = 0; vibN = 0;
      }
      break;
    }

    default:    // SENDING is handled inside runSendWindow(); DONE returns above
      break;
  }
}
