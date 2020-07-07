/* Memory assignment for code running on the bare chip with no bootloader. */

REGION_ALIAS("BOOT_VECTORS", BOOT_FLASH)
REGION_ALIAS("CODE_LOAD", APP_FLASH)
REGION_ALIAS("CODE_RUN", APP_FLASH)
REGION_ALIAS("DATA_RAM", MAIN_RAM)
