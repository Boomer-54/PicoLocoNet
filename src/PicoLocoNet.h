// SPDX-License-Identifier: MIT
// Projekt: https://github.com/Boomer-54/PicoLocoNet
// LocoNet für den Raspberry Pi Pico (arduino-pico), ohne LocoNet.h.
// Die Bits macht der Hardware-UART, die Bibliothek kümmert sich um Frames,
// Prüfsumme, Busruhe, Kollisionserkennung über das Echo und Wiederholungen.
//
// Herausgelöst aus pico-loconet-servo16 (GCA50/LocoIO-Emulator).
//
// Hardware: Pegelwandler, der den Pegel NICHT invertiert:
//   RX: LocoNet High -> GPIO High
//   TX: GPIO Low     -> LocoNet wird auf Masse gezogen
#pragma once
#include <Arduino.h>

#ifndef LN_RX_MAX
#define LN_RX_MAX 32  // längster Frame, der empfangen wird
#endif
#ifndef LN_TX_MAX
#define LN_TX_MAX 16  // längster Frame, der gesendet wird
#endif
#ifndef LN_TXQ_SIZE
#define LN_TXQ_SIZE 24  // Plätze in der Sendewarteschlange
#endif

// Häufige Opcodes
#define OPC_GPOFF 0x82
#define OPC_GPON 0x83
#define OPC_IDLE 0x85
#define OPC_SW_REQ 0xB0
#define OPC_SW_REP 0xB1
#define OPC_INPUT_REP 0xB2
#define OPC_LONG_ACK 0xB4
#define OPC_PEER_XFER 0xE5

class PicoLocoNet {
public:
  // uart = Serial1 oder Serial2, Standard: TX = GPIO0, RX = GPIO1
  void begin(SerialUART &uart = Serial1, uint8_t txPin = 0, uint8_t rxPin = 1);

  // Liest empfangene Bytes. true, wenn ein kompletter Frame mit gültiger
  // Prüfsumme bereitsteht, dann gelten frame() und length() bis zum nächsten Aufruf.
  bool receive();
  const uint8_t *frame() const { return _frame; }
  uint8_t length() const { return _frameLen; }

  // Stellt einen Frame in die Warteschlange. len zählt das Prüfsummenbyte mit,
  // die Prüfsumme wird hier eingesetzt. false = Warteschlange voll oder zu lang.
  bool send(const uint8_t *msg, uint8_t len);

  // Sendet höchstens einen Frame, wenn der Bus lange genug still war.
  // Muss wie receive() in jedem loop()-Durchlauf aufgerufen werden.
  void service();

  // Hilfen für häufige Telegramme (Adressen 1-basiert wie in Rocrail/JMRI)
  bool sendSwitch(uint16_t addr, bool closed, bool on = true);  // OPC_SW_REQ
  bool sendSensor(uint16_t addr, bool active);                  // OPC_INPUT_REP
  bool sendPower(bool on);                                       // OPC_GPON / OPC_GPOFF

  // Zerlegt einen empfangenen OPC_SW_REQ bzw. OPC_INPUT_REP, false = anderer Opcode
  static bool decodeSwitch(const uint8_t *m, uint16_t &addr, bool &closed, bool &on);
  static bool decodeSensor(const uint8_t *m, uint16_t &addr, bool &active);

  static uint8_t checksum(const uint8_t *m, uint8_t n);

  // true, solange noch Frames auf das Senden warten
  bool txPending() const { return _txHead != _txTail; }
  // Zähler für die Diagnose
  uint32_t txGivenUp() const { return _txGivenUp; }
  uint32_t txRetries() const { return _txRetries; }

private:
  struct TxMsg {
    uint8_t len;
    uint8_t tries;
    uint8_t b[LN_TX_MAX];
  };

  SerialUART *_uart = nullptr;
  uint32_t _lastRxUs = 0;
  uint8_t _frame[LN_RX_MAX];
  uint8_t _frameLen = 0;
  uint8_t _rxBuf[LN_RX_MAX];
  uint8_t _rxLen = 0, _rxNeed = 0;
  TxMsg _txq[LN_TXQ_SIZE];
  uint8_t _txHead = 0, _txTail = 0;
  uint32_t _txGivenUp = 0, _txRetries = 0;
};
