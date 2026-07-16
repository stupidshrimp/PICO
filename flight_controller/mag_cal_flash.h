#ifndef FEATHER_MAG_CAL_FLASH_H
#define FEATHER_MAG_CAL_FLASH_H

#include <stdint.h>

// On-flash persistence layout for the runtime magnetometer calibration so a
// field calibration survives power cycles; a missing/corrupt record falls
// back to the compiled-in HARD_IRON_BIAS / SOFT_IRON_MATRIX defaults.
//
// Fixed-layout record (no padding: all members naturally aligned). Two slots
// are kept in the emulated EEPROM and saves alternate between them, writing
// only the slot that does NOT hold the newest valid record: the previous
// calibration therefore survives a save whose programming fails verification.
// (Honest limit: stm32duino EEPROM emulation is a single flash sector, so the
// flush erases the whole sector before reprogramming it from the RAM buffer.
// A power loss mid-flush can still destroy both slots; boot then falls back
// to the compiled-in constants -- the fail-safe direction.)
//
// These type definitions live in a header (rather than inline in Main.ino) so
// they are visible before the Arduino build's auto-generated function
// prototypes, which are inserted ahead of the first function definition in the
// sketch: a prototype that names one of these types must see it declared
// first.
struct MagCalFlashRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  uint32_t sequence;  // monotonically increasing; newest valid record wins
  float hardIron[3];
  float softIronDiag[3];
  uint32_t crc;  // CRC-32 over every byte above this field
};
const uint32_t MAG_CAL_FLASH_MAGIC = 0x4C43474DUL;  // "MGCL"
const uint16_t MAG_CAL_FLASH_VERSION = 1;
const uint32_t MAG_CAL_FLASH_BASE_ADDR = 0;
const uint8_t MAG_CAL_FLASH_SLOT_COUNT = 2;

enum MagCalSaveStatus {
  MAG_CAL_SAVE_OK = 0,
  // New record failed to verify but the previous stored record is intact.
  MAG_CAL_SAVE_FAILED_OLD_INTACT,
  // New record failed to verify AND no stored record survives; the next boot
  // falls back to the compiled-in constants.
  MAG_CAL_SAVE_FAILED_STORE_LOST,
};

#endif  // FEATHER_MAG_CAL_FLASH_H
