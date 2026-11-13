// SPDX-License-Identifier: MIT
#include "PicoLocoNet.h"

// 1 Bit = 60 us bei 16666 Baud
#define LN_BAUD 16666
#define LN_IDLE_US 1560     // CD-Backoff 20 Bit + Prioritätsverzögerung 6 Bit
#define LN_ECHO_TIMEOUT_US 3000
#define LN_TX_TRIES 4

void PicoLocoNet::begin(SerialUART &uart, uint8_t txPin, uint8_t rxPin) {
  _uart = &uart;
  _uart->setPinout(txPin, rxPin);
  _uart->begin(LN_BAUD);
  _lastRxUs = micros();
}

uint8_t PicoLocoNet::checksum(const uint8_t *m, uint8_t n) {
  uint8_t c = 0xFF;
  for (uint8_t i = 0; i < n; i++) c ^= m[i];
  return c;
}

// ===================== Empfangen =====================
bool PicoLocoNet::receive() {
  while (_uart->available()) {
    uint8_t b = _uart->read();
    _lastRxUs = micros();
    if (b & 0x80) {
      _rxLen = 0;
      _rxNeed = 0;
    }  // Opcode: neuer Frame
    else if (_rxLen == 0)
      continue;  // Datenbyte ohne Opcode
    if (_rxLen >= sizeof(_rxBuf)) {
      _rxLen = 0;
      continue;
    }
    _rxBuf[_rxLen++] = b;
    if (_rxLen == 1) {
      switch ((b >> 5) & 3) {
        case 0: _rxNeed = 2; break;
        case 1: _rxNeed = 4; break;
        case 2: _rxNeed = 6; break;
        default: _rxNeed = 0;  // variable Länge, steht in Byte 2
      }
    } else if (_rxLen == 2 && _rxNeed == 0) {
      _rxNeed = b;
      if (_rxNeed < 3 || _rxNeed > sizeof(_rxBuf)) {
        _rxLen = 0;
        continue;
      }
    }
    if (_rxNeed && _rxLen == _rxNeed) {
      uint8_t n = _rxNeed;
      _rxLen = 0;
      if (checksum(_rxBuf, n - 1) == _rxBuf[n - 1]) {
        memcpy(_frame, _rxBuf, n);
        _frameLen = n;
        return true;
      }
    }
  }
  return false;
}

// ===================== Senden (Warteschlange) =====================
bool PicoLocoNet::send(const uint8_t *msg, uint8_t len) {
  uint8_t next = (_txHead + 1) % LN_TXQ_SIZE;
  if (next == _txTail || len < 2 || len > LN_TX_MAX) return false;
  TxMsg &m = _txq[_txHead];
  memcpy(m.b, msg, len - 1);
  m.b[len - 1] = checksum(m.b, len - 1);
  m.len = len;
  m.tries = 0;
  _txHead = next;
  return true;
}

void PicoLocoNet::service() {
  if (_txHead == _txTail) return;
  if (micros() - _lastRxUs < LN_IDLE_US) return;

  TxMsg &m = _txq[_txTail];
  bool ok = true;
  for (uint8_t i = 0; i < m.len && ok; i++) {
    _uart->write(m.b[i]);
    uint32_t t = micros();
    while (!_uart->available()) {
      if (micros() - t > LN_ECHO_TIMEOUT_US) {
        ok = false;
        break;
      }  // kein Echo
    }
    if (ok) {
      if (_uart->read() != m.b[i]) ok = false;  // Echo falsch: Kollision
      _lastRxUs = micros();
    }
  }
  if (ok || ++m.tries >= LN_TX_TRIES) {
    if (!ok) _txGivenUp++;
    _txTail = (_txTail + 1) % LN_TXQ_SIZE;
  } else {
    _txRetries++;
    while (_uart->available()) _uart->read();  // Reste verwerfen
    _rxLen = 0;
    _lastRxUs = micros();
  }
}

// ===================== Häufige Telegramme =====================
// OPC_SW_REQ: B0 sw1 sw2 chk, sw2 Bit 5 = Richtung (1 = closed), Bit 4 = ON
bool PicoLocoNet::sendSwitch(uint16_t addr, bool closed, bool on) {
  uint16_t a = addr - 1;
  uint8_t m[4] = { OPC_SW_REQ, (uint8_t)(a & 0x7F),
                   (uint8_t)(((a >> 7) & 0x0F) | (closed ? 0x20 : 0) | (on ? 0x10 : 0)), 0 };
  return send(m, 4);
}

// OPC_INPUT_REP: B2 in1 in2 chk
// in1 = A7..A1, in2 = 0 X I L A11..A8 (X = 1, I = A0, L = belegt)
bool PicoLocoNet::sendSensor(uint16_t addr, bool active) {
  uint16_t a = addr - 1;
  uint8_t m[4] = { OPC_INPUT_REP, (uint8_t)((a >> 1) & 0x7F),
                   (uint8_t)(0x40 | ((a & 1) << 5) | (active ? 0x10 : 0) | ((a >> 8) & 0x0F)), 0 };
  return send(m, 4);
}

bool PicoLocoNet::sendPower(bool on) {
  uint8_t m[2] = { (uint8_t)(on ? OPC_GPON : OPC_GPOFF), 0 };
  return send(m, 2);
}

bool PicoLocoNet::decodeSwitch(const uint8_t *m, uint16_t &addr, bool &closed, bool &on) {
  if (m[0] != OPC_SW_REQ) return false;
  addr = ((((uint16_t)m[2] & 0x0F) << 7) | (m[1] & 0x7F)) + 1;
  closed = (m[2] & 0x20) != 0;
  on = (m[2] & 0x10) != 0;
  return true;
}

bool PicoLocoNet::decodeSensor(const uint8_t *m, uint16_t &addr, bool &active) {
  if (m[0] != OPC_INPUT_REP) return false;
  addr = ((((uint16_t)m[2] & 0x0F) << 8) | ((m[1] & 0x7F) << 1) | ((m[2] >> 5) & 1)) + 1;
  active = (m[2] & 0x10) != 0;
  return true;
}
