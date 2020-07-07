/* Memory aliases for code running in the bootloader slot. */

REGION_ALIAS("BOOT_VECTORS", BOOT_FLASH)
REGION_ALIAS("CODE_LOAD", BOOT_FLASH)
REGION_ALIAS("CODE_RUN", MAIN_RAM)  /* Copy to RAM and run there. */
REGION_ALIAS("DATA_RAM", MAIN_RAM)
