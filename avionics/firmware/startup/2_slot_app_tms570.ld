/* Memory assignment for application code running under a bootloader. */

REGION_ALIAS("BOOT_VECTORS", APP_FLASH)
REGION_ALIAS("CODE_LOAD", APP_FLASH)
REGION_ALIAS("CODE_RUN", APP_FLASH)
REGION_ALIAS("DATA_RAM", MAIN_RAM)
