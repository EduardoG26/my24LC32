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

uint8_t* I2C24LC32::editBuffer() { return _editRam; }
const uint8_t* I2C24LC32::activeBuffer() const { return _activeRam; }
size_t I2C24LC32::userSize() const { return _userSize; }
size_t I2C24LC32::numSlots() const { return _numSlots; }
bool I2C24LC32::isWritePending() const { return _writePending; }
uint32_t I2C24LC32::getActiveCounter() const { return _activeCounter; }

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
  // Re-scan footers and reload active buffer
  pickActiveSlotOnInit();
  if (_activeRam && _editRam) memcpy(_editRam, _activeRam, _userSize);
  return true;
}

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

  // send 16-bit address (MSB first), then request bytes
  _wire->beginTransmission(_i2cAddr);
  _wire->write((uint8_t)((addr >> 8) & 0xFF));
  _wire->write((uint8_t)(addr & 0xFF));
  if (_wire->endTransmission(false) != 0) {
    // device didn't ack the address; try once more
    if (_wire->endTransmission(false) != 0) return false;
  }

  size_t toRead = len;
  size_t read = 0;
  while (toRead > 0) {
    uint8_t chunk = toRead > 32 ? 32 : (uint8_t)toRead;
    size_t got = _wire->requestFrom((int)_i2cAddr, (int)chunk);
    if (got == 0) return false;
    for (size_t i = 0; i < got && read < len; ++i) {
      buf[read++] = _wire->read();
      toRead--;
    }
  }
  return true;
}

bool I2C24LC32::readSlotFooter(size_t slotIndex, uint32_t &outCounter, uint16_t &outCrc) const
{
  if (slotIndex >= _numSlots) return false;
  size_t base = slotBaseAddress(slotIndex);
  size_t footerOffset = _slotSize - FOOTER_SIZE;
  size_t footerAddr = base + footerOffset;

  uint8_t footer[FOOTER_SIZE];
  if (!eepromReadBytes(footerAddr, footer, FOOTER_SIZE)) return false;

  outCounter = ((uint32_t)footer[0] << 24) | ((uint32_t)footer[1] << 16) | ((uint32_t)footer[2] << 8) | ((uint32_t)footer[3]);
  outCrc = (uint16_t)((footer[4] << 8) | footer[5]);
  return true;
}

bool I2C24LC32::readSlotData(size_t slotIndex, uint8_t* dest) const
{
  if (slotIndex >= _numSlots) return false;
  size_t base = slotBaseAddress(slotIndex);
  // read exactly _userSize bytes starting at base
  return eepromReadBytes(base, dest, _userSize);
}

bool I2C24LC32::validateSlotByReading(size_t slotIndex, uint16_t expectedCrc) const
{
  size_t bytesRemaining = _userSize;
  size_t offset = 0;
  uint16_t crc = 0xFFFF;
  uint8_t buffer[PAGE_SIZE];
  while (bytesRemaining > 0) {
    size_t chunk = bytesRemaining > PAGE_SIZE ? PAGE_SIZE : bytesRemaining;
    if (!eepromReadBytes(slotBaseAddress(slotIndex) + offset, buffer, chunk)) return false;
    crc = crc16_ccitt(buffer, chunk, crc);
    offset += chunk;
    bytesRemaining -= chunk;
  }
  return crc == expectedCrc;
}

void I2C24LC32::pickActiveSlotOnInit()
{
  _activeCounter = 0;
  int bestIndex = -1;
  uint32_t bestCounter = 0;

  for (size_t i = 0; i < _numSlots; ++i) {
    uint32_t cnt;
    uint16_t crc;
    if (!readSlotFooter(i, cnt, crc)) continue;
    if (cnt == 0) continue;
    if (!validateSlotByReading(i, crc)) continue;
    if (bestIndex < 0 || cnt > bestCounter) {
      bestCounter = cnt;
      bestIndex = (int)i;
    }
  }

  if (bestIndex >= 0) {
    if (!readSlotData(bestIndex, _activeRam)) {
      memset(_activeRam, 0xFF, _userSize);
      _activeCounter = 0;
    } else {
      _activeCounter = bestCounter;
    }
  } else {
    memset(_activeRam, 0xFF, _userSize);
    _activeCounter = 0;
  }
}

bool I2C24LC32::commit()
{
  if (_writePending) return false;

  memcpy(_writeRam, _editRam, _userSize);

  // find active slot index (if any)
  int activeSlotIndex = -1;
  if (_activeCounter != 0) {
    for (size_t i = 0; i < _numSlots; ++i) {
      uint32_t cnt;
      uint16_t crc;
      if (!readSlotFooter(i, cnt, crc)) continue;
      if (cnt == _activeCounter) { activeSlotIndex = (int)i; break; }
    }
  }

  _writeSlotIndex = (activeSlotIndex >= 0) ? ((size_t)activeSlotIndex + 1) % _numSlots : 0;
  _writePageIndex = 0;
  _nextCounter = _activeCounter + 1;
  _writePending = true;
  return true;
}

bool I2C24LC32::writePageToEeprom(size_t absoluteAddr, const uint8_t* pageData, size_t len)
{
  if (absoluteAddr + len > EEPROM_TOTAL) return false;
  if (len == 0 || len > PAGE_SIZE) return false;

  _wire->beginTransmission(_i2cAddr);
  _wire->write((uint8_t)((absoluteAddr >> 8) & 0xFF));
  _wire->write((uint8_t)(absoluteAddr & 0xFF));
  _wire->write(pageData, len);
  uint8_t res = _wire->endTransmission();
  return res == 0;
}

bool I2C24LC32::writeFooterPage(size_t slotIndex, const uint8_t* dataForSlot, uint32_t counter, uint16_t crc)
{
  size_t base = slotBaseAddress(slotIndex);
  size_t footerPageOffset = (_slotPages - 1) * PAGE_SIZE;
  size_t footerAddr = base + footerPageOffset;

  uint8_t pageBuf[PAGE_SIZE];
  memset(pageBuf, 0xFF, PAGE_SIZE);

  size_t bytesBeforeLastPage = (_slotPages - 1) * PAGE_SIZE;
  if (_userSize > bytesBeforeLastPage) {
    size_t tailBytes = _userSize - bytesBeforeLastPage;
    memcpy(pageBuf, dataForSlot + bytesBeforeLastPage, tailBytes);
  }

  size_t footerOffsetInPage = PAGE_SIZE - FOOTER_SIZE;
  pageBuf[footerOffsetInPage + 0] = (uint8_t)((counter >> 24) & 0xFF);
  pageBuf[footerOffsetInPage + 1] = (uint8_t)((counter >> 16) & 0xFF);
  pageBuf[footerOffsetInPage + 2] = (uint8_t)((counter >> 8) & 0xFF);
  pageBuf[footerOffsetInPage + 3] = (uint8_t)(counter & 0xFF);
  pageBuf[footerOffsetInPage + 4] = (uint8_t)((crc >> 8) & 0xFF);
  pageBuf[footerOffsetInPage + 5] = (uint8_t)(crc & 0xFF);

  return writePageToEeprom(footerAddr, pageBuf, PAGE_SIZE);
}

void I2C24LC32::task()
{
  if (!_writePending) return;

  size_t base = slotBaseAddress(_writeSlotIndex);
  size_t pageOffset = _writePageIndex;
  size_t addr = base + pageOffset * PAGE_SIZE;

  if (pageOffset < _slotPages - 1) {
    uint8_t pageBuf[PAGE_SIZE];
    memset(pageBuf, 0xFF, PAGE_SIZE);
    size_t globalOffset = pageOffset * PAGE_SIZE;
    size_t bytesToCopy = _userSize > globalOffset ? min(PAGE_SIZE, _userSize - globalOffset) : 0;
    if (bytesToCopy > 0) memcpy(pageBuf, _writeRam + globalOffset, bytesToCopy);

    if (writePageToEeprom(addr, pageBuf, PAGE_SIZE)) {
      _writePageIndex++;
      return;
    } else {
      // device busy or error; try again later
      return;
    }
  } else {
    uint16_t crc = crc16_ccitt(_writeRam, _userSize);
    if (writeFooterPage(_writeSlotIndex, _writeRam, _nextCounter, crc)) {
      memcpy(_activeRam, _writeRam, _userSize);
      _activeCounter = _nextCounter;
      _writePending = false;
      _writeSlotIndex = (_writeSlotIndex + 1) % _numSlots;
      _writePageIndex = 0;
      _nextCounter = 0;
      return;
    } else {
      return;
    }
  }
}