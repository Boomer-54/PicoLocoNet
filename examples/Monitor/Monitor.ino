// LocoNet-Monitor: zeigt jeden empfangenen Frame im seriellen Monitor (115200 Baud)
// und schreibt Weichenbefehle, Melder und Gleisspannung im Klartext dazu.
#include <PicoLocoNet.h>

PicoLocoNet ln;

void setup() {
  Serial.begin(115200);
  delay(2000);
  ln.begin(Serial1, 0, 1);  // TX = GPIO0, RX = GPIO1
  Serial.println("LocoNet-Monitor bereit");
}

void loop() {
  if (ln.receive()) {
    const uint8_t *m = ln.frame();
    Serial.printf("%10lu ms ", millis());
    for (uint8_t i = 0; i < ln.length(); i++) Serial.printf(" %02X", m[i]);

    uint16_t addr;
    bool closed, on, active;
    if (PicoLocoNet::decodeSwitch(m, addr, closed, on))
      Serial.printf("   Weiche %u %s %s", addr, closed ? "gerade" : "abzweig", on ? "EIN" : "AUS");
    else if (PicoLocoNet::decodeSensor(m, addr, active))
      Serial.printf("   Melder %u %s", addr, active ? "belegt" : "frei");
    else if (m[0] == OPC_GPON)
      Serial.print("   Gleisspannung EIN");
    else if (m[0] == OPC_GPOFF)
      Serial.print("   Gleisspannung AUS");
    Serial.println();
  }
  ln.service();
}
