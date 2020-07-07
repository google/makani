SECTIONS {
  /* Make certain that we fill all flash; uninitialized flash generates ECC
   * exceptions. See TMS570 TR 5.3.1, "SECDED Initialization". */
  .padding : {
    . = ORIGIN(CODE_LOAD) + LENGTH(CODE_LOAD) - local.kAppConfigSize;

    /* Specify default values for the AppConfig configuration block. See the
     * definition in ldscript.h. This configuration specifies a node specific
     * configuration in constant space. The bootloader calls generate_image.py
     * to write this information before programming. */

    /* AioNode value (4 bytes). Default to kAioNodeUnknown (0). */
    LONG(0);
    /* AioNode node label index (4 bytes). Default to (0). */
    LONG(0);
    /* CRC32 of application only (4 bytes). */
    LONG(0);
    /* CRC32 of application + AppConfig (all bytes preceding) (4 bytes). */
    LONG(0);
  } > CODE_LOAD = 0xFF
}
