// Lesen und Schreiben am LocoNet:
// - Taster an GPIO 2 (gegen GND) meldet Melder 10 belegt/frei
// - Weichenbefehle für Adresse 5 schalten die LED auf dem Pico (abzweig = an)
// - Taster an GPIO 3 (gegen GND) stellt Weiche 5 abwechselnd gerade/abzweig
#include <PicoLocoNet.h>

const uint16_t MELDER = 10;
const uint16_t WEICHE = 5;
const uint8_t PIN_MELDER = 2;
const uint8_t PIN_TASTER = 3;

PicoLocoNet ln;
bool melderAlt = false;
bool tasterAlt = false;
bool weicheGerade = true;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_MELDER, INPUT_PULLUP);
  pinMode(PIN_TASTER, INPUT_PULLUP);
  ln.begin(Serial1, 0, 1);
}

void loop() {
  // Lesen
  if (ln.receive()) {
    uint16_t addr;
    bool closed, on;
    if (PicoLocoNet::decodeSwitch(ln.frame(), addr, closed, on) && addr == WEICHE && on) {
      weicheGerade = closed;
      digitalWrite(LED_BUILTIN, closed ? LOW : HIGH);
    }
    if (ln.frame()[0] == OPC_GPON) ln.sendSensor(MELDER, melderAlt);  // Zustand nach Gleisspannung EIN melden
  }

  // Schreiben: Melder, nach jeder Änderung 20 ms Sperrzeit gegen Prellen
  static uint32_t entprell = 0;
  bool melder = digitalRead(PIN_MELDER) == LOW;
  if (melder != melderAlt && millis() - entprell > 20) {
    melderAlt = melder;
    entprell = millis();
    ln.sendSensor(MELDER, melder);
  }

  // Schreiben: Weiche umschalten, ON und nach 200 ms OFF wie ein Stellpult
  static uint32_t offZeit = 0;
  bool taster = digitalRead(PIN_TASTER) == LOW;
  if (taster && !tasterAlt) {
    weicheGerade = !weicheGerade;
    ln.sendSwitch(WEICHE, weicheGerade, true);
    offZeit = millis() + 200;
  }
  tasterAlt = taster;
  if (offZeit && (int32_t)(millis() - offZeit) >= 0) {
    ln.sendSwitch(WEICHE, weicheGerade, false);
    offZeit = 0;
  }

  ln.service();
}
