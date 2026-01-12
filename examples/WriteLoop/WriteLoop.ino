/*
  WriteLoop example
  - Uses #define USER_BLOCK_SIZE to set block size (100 bytes)
  - Writes changing contents multiple times (exceeding wraparound)
  - Verifies the active data after each commit completes
*/

#include <Wire.h>
#include "I2C24LC32.h"

#define USER_BLOCK_SIZE 100
#define EEPROM_I2C_ADDR 0x50

I2C24LC32 eeprom(EEPROM_I2C_ADDR, USER_BLOCK_SIZE);

uint32_t writeCounter = 1; // each commit uses a new token for content pattern

// Fill the edit buffer with a "smart" changing pattern based on counter and index
void fillEditPattern(uint8_t* buf, size_t len, uint32_t token) {
  // pattern: first 4 bytes = token (LSB-first), then repeating (index ^ token)
  buf[0] = (uint8_t)(token & 0xFF);
  buf[1] = (uint8_t)((token >> 8) & 0xFF);
  buf[2] = (uint8_t)((token >> 16) & 0xFF);
  buf[3] = (uint8_t)((token >> 24) & 0xFF);
  for (size_t i = 4; i < len; ++i) {
    buf[i] = (uint8_t)((i & 0xFF) ^ (token & 0xFF));
  }
}

// Verify buffer matches expected pattern
bool verifyPattern(const uint8_t* buf, size_t len, uint32_t token) {
  if (buf[0] != (uint8_t)(token & 0xFF)) return false;
  if (buf[1] != (uint8_t)((token >> 8) & 0xFF)) return false;
  if (buf[2] != (uint8_t)((token >> 16) & 0xFF)) return false;
  if (buf[3] != (uint8_t)((token >> 24) & 0xFF)) return false;
  for (size_t i = 4; i < len; ++i) {
    if (buf[i] != (uint8_t)((i & 0xFF) ^ (token & 0xFF))) return false;
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(1);

  Serial.println("WriteLoop example starting...");

  if (!eeprom.begin()) {
    Serial.println("EEPROM: begin() failed. Check I2C wiring and block size.");
    while (1) delay(1000);
  }

  Serial.print("User block size: ");
  Serial.println(eeprom.userSize());
  Serial.print("Number of slots: ");
  Serial.println(eeprom.numSlots());
  Serial.print("Active counter: ");
  Serial.println(eeprom.getActiveCounter());

  // Force init from EEPROM (fills edit buffer)
  eeprom.initFromEeprom();
}

unsigned long lastTask = 0;
bool waitingForCompletion = false;
uint32_t lastToken = 0;
int writesToDo = 0;

// We'll do more writes than the number of slots to force wraparound
void loop() {
  // If we're idle, prepare and schedule a commit
  if (!eeprom.isWritePending() && !waitingForCompletion) {
    if (writesToDo == 0) {
      // plan number of writes: three times the number of slots (wraparound stress)
      writesToDo = (int)eeprom.numSlots() * 3;
      Serial.print("Planned writes: ");
      Serial.println(writesToDo);
    }
    uint8_t* edit = eeprom.editBuffer();
    fillEditPattern(edit, eeprom.userSize(), writeCounter);
    lastToken = writeCounter;
    writeCounter++;
    if (eeprom.commit()) {
      waitingForCompletion = true;
      Serial.print("Commit scheduled (token=");
      Serial.print(lastToken);
      Serial.println(").");
    } else {
      Serial.println("Commit failed (write pending).");
    }
  }

  // Run one small step per loop — progress EEPROM page writes
  eeprom.task();

  // Check completion
  if (waitingForCompletion && !eeprom.isWritePending()) {
    // write completed. Re-init from EEPROM and verify.
    eeprom.initFromEeprom();
    const uint8_t* active = eeprom.activeBuffer();
    bool ok = verifyPattern(active, eeprom.userSize(), lastToken);
    Serial.print("Write token ");
    Serial.print(lastToken);
    Serial.print(" complete. Verification: ");
    Serial.println(ok ? "OK" : "FAIL");

    writesToDo--;
    waitingForCompletion = false;

    if (writesToDo <= 0) {
      Serial.println("All planned writes completed.");
      // stop here
      while (1) {
        eeprom.task(); // keep servicing in case needed (not strictly necessary)
        delay(200);
      }
    }
  }

  // Keep loop quick
  if (millis() - lastTask > 5) {
    lastTask = millis();
  }
  delay(5);
}