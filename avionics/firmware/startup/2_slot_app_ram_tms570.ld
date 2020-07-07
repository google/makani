/* Memory assignment for application code running under a bootloader,
 * which copies itself and runs entirely from RAM after startup.
 * This is used by the bootloader in the app slot to re-flash the boot slot.
 */

REGION_ALIAS("BOOT_VECTORS", APP_FLASH)
REGION_ALIAS("CODE_LOAD", APP_FLASH)
REGION_ALIAS("CODE_RUN", MAIN_RAM)
REGION_ALIAS("DATA_RAM", MAIN_RAM)
