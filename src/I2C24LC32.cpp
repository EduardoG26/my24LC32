#include "I2C24LC32.h"

I2C24LC32::I2C24LC32(uint8_t i2cAddress, size_t userDataSize)
  : _i2cAddr(i2cAddress), _wire(nullptr),
    _userSize(userDataSize),
    _slotPages(0), _slotSize(0), _numSlots(0),
    _activeRam(nullptr), _editRam(nullptr), _writeRam(nullptr),
    _activeCounter(0),
    _writePending(false),
    _writeSlotIndex(0),
    _writePageIndex(0),
    _nextCounter(0)
{
}

I2C24LC32::~I2C24LC32()
{
  delete[] _activeRam;
  delete[] _editRam;
  delete[] _writeRam;
}

bool I2C24LC32::begin(TwoWire &wire)
{
  _wire = &wire;
  _wire->begin();

  _slotPages = (_userSize + FOOTER_SIZE + PAGE_SIZE - 1) / PAGE_SIZE;
  _slotSize = _slotPages * PAGE_SIZE;
  if (_slotSize == 0 || _slotSize > EEPROM_TOTAL) return false;

  _numSlots = EEPROM_TOTAL / _slotSize;
  if (_numSlots < 2) return false;

  _activeRam = new uint8_t[_userSize];
  _editRam   = new uint8_t[_userSize];
  _writeRam  = new uint8_t[_userSize];
  if (!_activeRam || !_editRam || !_writeRam) return false;

  memset(_activeRam, 0xFF, _userSize);
  memset(_editRam, 0xFF, _userSize);
  memset(_writeRam, 0xFF, _userSize);
  _activeCounter = 0;
  _writePending = false;

  pickActiveSlotOnInit();
  memcpy(_editRam, _activeRam, _userSize);
  return true;
}

bool I2C24LC32::initFromEeprom()
{
  pickActiveSlotOnInit();
  if (_activeRam && _editRam) memcpy(_editRam, _activeRam, _userSize);
  return true;
}

uint8_t* I2C24LC32::editBuffer() { return _editRam; }
const uint8_t* I2C24LC32::activeBuffer() const { return _activeRam; }
size_t I2C24LC32::userSize() const { return _userSize; }
size_t I2C24LC32::numSlots() const { return _numSlots; }
bool I2C24LC32::isWritePending() const { return _writePending; }
uint32_t I2C24LC32::getActiveCounter() const { return _activeCounter; }

uint16_t I2C24LC32::crc16_ccitt(const uint8_t* data, size_t len, uint16_t seed) const
{
  uint16_t crc = seed;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc & 0xFFFF;
}

bool I2C24LC32::eepromReadBytes(size_t addr, uint8_t* buf, size_t len) const
{
  if (addr + len > EEPROM_TOTAL) return false;

  _wire->beginTransmission(_i2cAddr);
  _wire->write((uint8_t)((addr >> 8) & 0xFF));
  _wire->write((uint8_t)(addr & 0xFF));
  if (_wire->endTransmission(false) != 0) {
    if (_wire->endTransmission(false) != 0) return false;
  }

  size_t toRead = len;
  size_t read = 0;
  while (toRead > 0) {
    uint8_t chunk = toRead > 32 ? 32 : (uint8_t)toRead;
    size_t got = _wire->requestFrom((int)_i2cAddr, (int)chunk);
    if (got == 0) return false;
    for (size