SECTIONS {
  /* Make certain that we fill all flash; uninitialized flash generates ECC
   * exceptions. See TMS570 TR 5.3.1, "SECDED Initialization". */
  .padding : {
    . = ORIGIN(CODE_LOAD) + LENGTH(CODE_LOAD) - local.kBootConfigSize;

    /* Specify default values for the BootConfig configuration block. See the
     * definition in ldscript.h. This configuration specifies a node specific
     * configuration in constant space. The bootloader calls generate_image.py
     * to write this information before programming. */

    /* Boot block version number (4 bytes). */
    LONG(0xDE100001);
    /* IP address (4 bytes). Default to unknown node (192.168.1.254). */
    BYTE(192);
    BYTE(168);
    BYTE(1);
    BYTE(254);
    /* MAC address (6 bytes). Default to unknown node (02:00:00:00:00:FE). */
    BYTE(0x02);  /* MAC address first octet. */
    BYTE(0x00);
    BYTE(0x00);
    BYTE(0x00);
    BYTE(0x00);
    BYTE(0xFE);  /* MAC address last octet. */
    /* Hardware type (2 bytes). Default to kHardwareTypeUnknown (-1). */
    SHORT(-1);
    /* Unused (2 bytes). */
    SHORT(-1);
    /* Node index (2 bytes). Default to (0). */
    SHORT(0);
    /* CRC32 of bootloader + BootConfig (all bytes preceding) (4 bytes). */
    LONG(0);
  } > CODE_LOAD = 0xFF
}
