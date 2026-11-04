// RAW replay of Garage.sub (Flipper SubGhz capture) on CC1101.
// Bit-bangs GDO0 in CC1101 async OOK mode -> reproduces the exact pulse
// timings Flipper sends, which (per testing) the gate accepts.
//
// Board: Arduino Nano 33 IoT (SAMD21, 3V3 logic -> CC1101 direct, no level shift)
// Wiring (same as receiver/receiver_raw):
//   CC1101 SCK  -> D13
//   CC1101 MISO -> D12
//   CC1101 MOSI -> D11
//   CC1101 CSN  -> D10
//   CC1101 GDO0 -> D6    (Arduino DRIVES this pin to key the carrier on/off)
//   CC1101 GDO2 -> D2    (unused for TX)
//   CC1101 VCC  -> 3V3   (NOT 5V!)
//   CC1101 GND  -> GND
//
// Trigger: send any line in Serial (115200), or press-to-send is also on a
// button if you wire one to D3 to GND. By default it does NOT transmit on
// its own - you must ask for it, so you don't spam the band.

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include "garage_signal.h"

const byte CC_SCK  = 13;
const byte CC_MISO = 12;
const byte CC_MOSI = 11;
const byte CC_CSN  = 10;
const byte CC_GDO0 = 6;    // we drive this to key TX
const byte CC_GDO2 = 2;

const byte BTN_PIN = 3;    // optional push button to GND (INPUT_PULLUP)

const float FREQ_MHZ = 433.92;

// Busy-wait on micros() - handles the capture's long gaps (up to 170 ms) with
// one code path, and (unlike delay()) works fine with interrupts left ON.
// Unsigned subtraction makes the ~71-min micros() rollover harmless.
static inline void waitUs(uint32_t us) {
  uint32_t start = micros();
  while ((uint32_t)(micros() - start) < us) { /* spin */ }
}

void configureTx() {
  ELECHOUSE_cc1101.setSpiPin(CC_SCK, CC_MISO, CC_MOSI, CC_CSN);
  ELECHOUSE_cc1101.setGDO0(CC_GDO0);

  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setCCMode(0);        // raw pin access, not the internal FIFO engine
  ELECHOUSE_cc1101.setModulation(2);    // 2 = ASK/OOK  (Flipper preset Ook...)
  ELECHOUSE_cc1101.setMHZ(FREQ_MHZ);    // 433.92 MHz
  ELECHOUSE_cc1101.setPA(12);           // max TX power
  ELECHOUSE_cc1101.setSyncMode(0);      // RAW: no preamble / sync
  ELECHOUSE_cc1101.setCrc(0);           // RAW: no CRC
  ELECHOUSE_cc1101.setManchester(0);    // RAW: no encoding
  ELECHOUSE_cc1101.setWhiteData(0);     // Ook...Async preset has no whitening
  ELECHOUSE_cc1101.setPktFormat(3);     // 3 = asynchronous serial: data in on GDO0
}

void sendSignal() {
  Serial.println("TX...");

  pinMode(CC_GDO0, OUTPUT);
  digitalWrite(CC_GDO0, LOW);

  ELECHOUSE_cc1101.SetTx();             // carrier enabled; GDO0 gates it on/off

  // Interrupts stay ON: micros() needs them, and the ~1 ms SysTick adds only
  // a few us of jitter, which an OOK remote (TE ~460 us) easily tolerates.
  for (uint16_t i = 0; i < GARAGE_SIGNAL_LEN; i++) {
    int32_t d = GARAGE_SIGNAL[i];
    digitalWrite(CC_GDO0, d > 0 ? HIGH : LOW);   // + = carrier on, - = off
    waitUs(d > 0 ? (uint32_t)d : (uint32_t)(-d));
  }

  digitalWrite(CC_GDO0, LOW);
  ELECHOUSE_cc1101.setSidle();         // back to idle, stop transmitting
  Serial.println("done");
}

void setup() {
  Serial.begin(115200);
  pinMode(BTN_PIN, INPUT_PULLUP);

  if (ELECHOUSE_cc1101.getCC1101()) {
    Serial.println("CC1101 OK");
  } else {
    Serial.println("CC1101 not found - check wiring (VCC must be 3V3)");
  }

  configureTx();

  Serial.print(GARAGE_SIGNAL_LEN);
  Serial.println(" edges loaded.");
  Serial.println("Press button on D3, or send any line in Serial, to transmit.");
}

void loop() {
  // Trigger over Serial: any received line sends once.
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    sendSignal();
  }

  // Trigger via button on D3 (to GND), debounced, one shot per press.
  static bool prev = HIGH;
  bool now = digitalRead(BTN_PIN);
  if (prev == HIGH && now == LOW) {
    delay(20);                         // debounce
    if (digitalRead(BTN_PIN) == LOW) sendSignal();
  }
  prev = now;
}
