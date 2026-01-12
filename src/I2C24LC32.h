#ifndef I2C24LC32_H
#define I2C24LC32_H

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <stddef.h>

class I2C24LC32 {
public:
  // Create driver instance
  // i2cAddress: base 7-bit I2C address (usually 0x50..0x57 depending on pins)
  // userDataSize: size in bytes of user data block
  I2C24LC32(uint8_t i2cAddress, size_t userDataSize);
  ~I2C24LC32();

  // Initialize Wire and scan/fill active buffer.
  // Returns true on success (found a valid slot or initialized empty buffers).
  bool begin(TwoWire &wire = Wire);

  // Re-read EEPROM and populate RAM active/edit buffers
  bool initFromEeprom();

  // Pointer to edit buffer (user may directly modify). Must call commit() to save.
  uint8_t* editBuffer();

  // Read-only pointer to currently active valid buffer
  const uint8_t* activeBuffer() const;

  // Size of user buffer
  size_t userSize() const;

  // Number of slots available in the EEPROM
  size_t numSlots() const;
  
  // Schedule a write of the edit buffer to EEPROM.
  // A snapshot is taken immediately into internal write buffer to allow
  // the user to keep editing editBuffer() while write proceeds.
  // Returns false if a write is already pending.
  bool commit();

  // Call frequently (in loop). Each call will attempt one page write (or poll)
  // to progress an ongoing EEPROM write. Must be called until commit completes.
  void task();

  // True while a commit/write is in progress
  bool isWritePending() const;

  // Counter of active slot (last updated token). 0 if none.
  uint32_t getActiveCounter() const;

private:
  // EEPROM specifics
  const uint8_t _i2cAddr;
  TwoWire* _wire;
  const size_t _userSize;
  static constexpr size_t EEPROM_TOTAL = 4096; // bytes for 24LC32
  static constexpr size_t PAGE_SIZE = 32;      // bytes
  static constexpr size_t FOOTER_SIZE = 6;     // 4 bytes counter + 2 bytes CRC16

  // Computed layout
  size_t _slotPages;   // pages per slot (including footer page)
  size_t _slotSize;    // bytes per slot = slotPages * PAGE_SIZE
  size_t _numSlots;    // number of slots that fit in EEPROM

  // RAM buffers
  uint8_t* _activeRam; // current valid data
  uint8_t* _editRam;   // user edits
  uint8_t* _writeRam;  // snapshot that's being written to EEPROM

  // Active token (counter)
  uint32_t _activeCounter;

  // Write state machine
  volatile bool _writePending;
  size_t _writeSlotIndex;   // slot index being written
  size_t _writePageIndex;   // page offset within slot (0..slotPages-1)
  uint32_t _nextCounter;    // counter to write for this slot

  // Helpers
  uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t seed = 0xFFFF) const;
  bool readSlotFooter(size_t slotIndex, uint32_t &outCounter, uint16_t &outCrc) const;
  bool readSlotData(size_t slotIndex, uint8_t* dest) const;
  bool validateSlotByReading(size_t slotIndex, uint16_t expectedCrc) const;
  void pickActiveSlotOnInit();

  bool writePageToEeprom(size_t absoluteAddr, const uint8_t* pageData, size_t len);
  bool writeFooterPage(size_t slotIndex, const uint8_t* dataForSlot, uint32_t counter, uint16_t crc);

  bool eepromReadBytes(size_t addr, uint8_t* buf, size_t len) const;
  inline size_t slotBaseAddress(size_t slotIndex) const { return slotIndex * _slotSize; }
};

#endif // I2C24LC32_H