# PicoLocoNet

LocoNet für den Raspberry Pi Pico (arduino-pico), **ohne `LocoNet.h`**.

Die Bits erledigt der Hardware-UART des Pico. Die Bibliothek kümmert sich um den Rest:

- Empfang: Frames werden am Opcode erkannt, die Länge kommt aus dem Opcode bzw. aus Byte 2, die Prüfsumme wird geprüft
- Senden über eine Warteschlange (24 Frames)
- Busruhe vor dem Senden: CD-Backoff 20 Bit + Priorität 6 Bit (1560 µs)
- Kollisionserkennung: Jedes gesendete Byte wird über das Echo geprüft
- Bis zu 4 Versuche pro Frame

Kein Timer, kein Interrupt-Bit-Banging. Die LocoNet-Schicht ist aus [pico-loconet-servo16](https://github.com/Boomer-54/pico-loconet-servo16) herausgelöst
und dort mit einem GCA50/LocoIO-Emulator für 16 Servos im Einsatz.

## Hardware

Ein Pegelwandler zwischen LocoNet (12 V) und 3,3 V, der den Pegel **nicht invertiert**:

| Signal | Pico | Verhalten |
|---|---|---|
| RX | GPIO 1 | LocoNet High → GPIO High |
| TX | GPIO 0 | GPIO Low → LocoNet wird auf Masse gezogen |

Eine passende Schaltung (Transistor-Eingangsstufe mit ≥ 47 kΩ Last, Schwelle ca. 3,4–4 V) steht in
[loconet-pico](https://github.com/Boomer-54/loconet-pico).

## Benutzung

```cpp
#include <PicoLocoNet.h>

PicoLocoNet ln;

void setup() {
  ln.begin(Serial1, 0, 1);  // TX = GPIO0, RX = GPIO1
}

void loop() {
  if (ln.receive()) {
    // ln.frame() / ln.length(): empfangener Frame mit gültiger Prüfsumme
  }
  ln.service();  // sendet wartende Frames, sobald der Bus frei ist
}
```

`receive()` und `service()` müssen in jedem `loop()`-Durchlauf aufgerufen werden. Lange `delay()` vermeiden.

### Funktionen

| Funktion | Zweck |
|---|---|
| `send(msg, len)` | beliebigen Frame senden, `len` mit Prüfsummenbyte, die Prüfsumme wird eingesetzt |
| `sendSwitch(adr, gerade, ein)` | Weichenbefehl (OPC_SW_REQ) |
| `sendSensor(adr, belegt)` | Melder (OPC_INPUT_REP) |
| `sendPower(ein)` | Gleisspannung ein/aus |
| `decodeSwitch(...)`, `decodeSensor(...)` | empfangene Weichenbefehle und Melder zerlegen |
| `txRetries()`, `txGivenUp()` | Zähler für Wiederholungen und aufgegebene Frames |

Adressen sind 1-basiert wie in Rocrail und JMRI.

### Beispiele

- **Monitor:** zeigt jeden Frame im seriellen Monitor, Weichen und Melder im Klartext
- **SchaltenUndMelden:** Taster meldet einen Melder, ein zweiter Taster stellt eine Weiche, die Pico-LED folgt einer Weichenadresse

## Installation

Ordner `PicoLocoNet` nach `Arduino/libraries/` kopieren. Board: arduino-pico (Earle Philhower), z. B. „Raspberry Pi Pico“.

## Lizenz

MIT
