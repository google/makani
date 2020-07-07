/* Memory layout of TMS570LS1227 and similar chips. */

MEMORY {
  BOOT_FLASH (RX)  : org = 0x00000000, len = 0x00040000
  APP_FLASH (RX)   : org = 0x00040000, len = 0x00100000
  FULL_FLASH (RX)  : org = 0x00000000, len = 0x00140000
  MAIN_RAM (RW)    : org = 0x08000000, len = 0x0002F800
  DEBUG_RAM (RW)   : org = 0x0802F800, len = 0x000007F0
  BOOT_RAM (RW)    : org = 0x0802FFF0, len = 0x00000010

  /* Bank 7 EEPROM emulation flash. */
  BANK7_0 (R)      : org = 0xF0200000, len = 0x00004000
  BANK7_1 (R)      : org = 0xF0204000, len = 0x00004000
  BANK7_2 (R)      : org = 0xF0208000, len = 0x00004000
  BANK7_3 (R)      : org = 0xF020C000, len = 0x00004000

  /* Full RAM region. */
  FULL_RAM (RW)    : org = 0x08000000, len = 0x00030000
}
