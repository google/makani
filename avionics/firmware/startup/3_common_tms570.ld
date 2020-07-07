/* Common linker script for TMS570 code.
 *
 * Earlier linker scripts must define MEMORY and/or REGION_ALIASES for
 * BOOT_VECTORS, CODE_LOAD, CODE_RUN, DATA_RAM, and BOOT_RAM (which may alias).
 *
 * References:
 * - TMS570LS3137_FLASH.ics in the IAR BSP
 * - Code Composer Studio/HALCoGen project linker scripts
 * - http://sourceware.org/binutils/docs-2.22/ld/Scripts.html#Scripts
 */

ENTRY(vector_table)
OUTPUT_FORMAT("elf32-bigarm")
OUTPUT_ARCH("arm")

/* Map bank 7 to flash parameters. */
REGION_ALIAS("CONFIG_PARAMS", BANK7_0)
REGION_ALIAS("CALIB_PARAMS", BANK7_1)
REGION_ALIAS("RESERVED", BANK7_2)
REGION_ALIAS("SERIAL_PARAMS", BANK7_3)

/*
 * Stack sizes.
 */

/* TODO: Use HIDDEN() once available, and remove local. prefix. */
local.kMainStackSize = 0x2000;  /* Primary C runtime stack size. */
local.kIrqProtectorSize = 64;  /* Must be power of 2 and >= 32 byte. */
local.kIrqStackSize = 0x2000;  /* Size of IRQ stack. */
local.kFiqProtectorSize = 64;  /* Must be power of 2 and >= 32 byte. */
local.kFiqStackSize = 0x2000;  /* Size of FIQ stack. */
local.kFatalAbortStackSize = 0x0400;  /* Size of stacks for fatal aborts. */
local.kContextStackSize = 0x0400;  /* Size of stack for context saving. */

/* Size of configuration blocks. */
local.kFlashParamHeaderSize = 0x14;
local.kAppConfigSize = 16;
local.kBootConfigSize = 24;

/* Segments to protect from flashing. */
PROVIDE(ldscript_protect_begin = ORIGIN(CODE_LOAD));
PROVIDE(ldscript_protect_end = ORIGIN(CODE_LOAD) + LENGTH(CODE_LOAD));

/* Store configuration block start addresses. */
PROVIDE(ldscript_app_config = ORIGIN(APP_FLASH) + LENGTH(APP_FLASH) -
        local.kAppConfigSize);
PROVIDE(ldscript_boot_config = ORIGIN(BOOT_FLASH) + LENGTH(BOOT_FLASH) -
        local.kBootConfigSize);

/* Boot loader flash segment. */
PROVIDE(ldscript_boot_flash_begin = ORIGIN(BOOT_FLASH));
PROVIDE(ldscript_boot_flash_end = ORIGIN(BOOT_FLASH) + LENGTH(BOOT_FLASH));

/* Start of primary app slot (may or may not be this program!). */
PROVIDE(ldscript_app_flash_begin = ORIGIN(APP_FLASH));
PROVIDE(ldscript_app_flash_end = ORIGIN(APP_FLASH) + LENGTH(APP_FLASH));
PROVIDE(ldscript_app_flash_crc = ORIGIN(APP_FLASH) + LENGTH(APP_FLASH) - 4);

/* Full Flash region information for bootloader. */
PROVIDE(ldscript_flash_begin = ORIGIN(FULL_FLASH));
PROVIDE(ldscript_flash_end = ORIGIN(FULL_FLASH) + LENGTH(FULL_FLASH));

/* Full RAM region so that the RAM size can be computed for MPU setup. */
PROVIDE(ldscript_ram_begin = ORIGIN(FULL_RAM));
PROVIDE(ldscript_ram_end = ORIGIN(FULL_RAM) + LENGTH(FULL_RAM));

/* Flash parameters. */
PROVIDE(ldscript_calib_param_begin = ORIGIN(CALIB_PARAMS));
PROVIDE(ldscript_calib_param_end = ORIGIN(CALIB_PARAMS) + LENGTH(CALIB_PARAMS));
PROVIDE(ldscript_config_param_begin = ORIGIN(CONFIG_PARAMS));
PROVIDE(ldscript_config_param_end =
        ORIGIN(CONFIG_PARAMS) + LENGTH(CONFIG_PARAMS));
PROVIDE(ldscript_serial_param_begin = ORIGIN(SERIAL_PARAMS));
PROVIDE(ldscript_serial_param_end =
        ORIGIN(SERIAL_PARAMS) + LENGTH(SERIAL_PARAMS));

/* Debug RAM. */
PROVIDE(ldscript_log_begin = ORIGIN(DEBUG_RAM));
PROVIDE(ldscript_log_end = ORIGIN(DEBUG_RAM) + LENGTH(DEBUG_RAM));

SECTIONS {
  /*
   * Non-volatile sections.
   */

  /* Vector table must be placed at a specific location.
   * This must come first, in case BOOT_VECTORS and CODE_LOAD are aliased. */
  .text.vectors : ALIGN(8) {
    KEEP(*(.text.vectors))

    . = ALIGN(8);  /* Avoid gap to start of next section. */
  } > BOOT_VECTORS

  /* Low level startup code is never copied. (It's what does the copying!) */
  .text.startup : ALIGN(8) {
    KEEP(*(.text.startup*))

    . = ALIGN(8);  /* Avoid gap to start of next section. */
  } > CODE_LOAD

  /* Note: Code is selected into sections using input section syntax.
   * https://sourceware.org/binutils/docs/ld/Input-Section-Basics.html
   *
   * C library (libc) functions are broken up by file, so we select by file.
   * Our code uses -ffunction-sections, so each function gets its own section.
   */

  /* Code that runs rarely and should be excluded from .text.fast. */
  .text.slow : ALIGN(8) {
    *(.text.unlikely.*)  /* GCC uses .text.unlikely for error code, etc. */

    . = ALIGN(8);  /* Avoid gap to start of next section. */
  } > CODE_RUN AT> CODE_LOAD

  /* Performance-critical code that gets copied to RAM. */
  .text.fast : ALIGN(8) {
    *(.text.memcpy)
    *lib_a-memcmp.o(.text)
    *lib_a-memset.o(.text)

    . = ALIGN(8);  /* Avoid gap to start of next section. */
  } > DATA_RAM AT> CODE_LOAD

  /* Code that runs directly from flash. */
  .text : ALIGN(8) {
    *(.text)
    *(.text.*)
    *(.text:*)
    *(.gnu.linkonce.t.*)

    . = ALIGN(8);  /* Avoid gap to start of next section. */
  } > CODE_RUN AT> CODE_LOAD

  /* Performance-critical read only initialized data that gets copied to RAM. */
  .rodata.fast : ALIGN(8) {

    . = ALIGN(8);  /* Avoid gap to start of next section. */
  } > DATA_RAM AT> CODE_LOAD

  /* Read only initialized data used directly from flash. */
  .rodata : ALIGN(8) {
    *(.rodata)
    *(.rodata.*)
    *(.gnu.linkonce.r.*)

    . = ALIGN(8);  /* Avoid gap to start of next section. */
  } > CODE_RUN AT> CODE_LOAD

  /* Used by startup code to copy data/code from ROM.
   * Note that startup code will skip no-op copies (source = target). */
  .rodata.copy_table : ALIGN(8) {
    PROVIDE(ldscript_copy_table = .);

    /*   Loaded address      Address to copy to       Size (-1 = table end) */
    LONG(LOADADDR(.text));   LONG(ADDR(.text));       LONG(SIZEOF(.text));
    LONG(LOADADDR(.text.slow));
      LONG(ADDR(.text.slow));
      LONG(SIZEOF(.text.slow));
    LONG(LOADADDR(.text.fast));
      LONG(ADDR(.text.fast));
      LONG(SIZEOF(.text.fast));
    LONG(LOADADDR(.rodata)); LONG(ADDR(.rodata));     LONG(SIZEOF(.rodata));
    LONG(LOADADDR(.rodata.fast));
      LONG(ADDR(.rodata.fast));
      LONG(SIZEOF(.rodata.fast));
    LONG(LOADADDR(.data));   LONG(ADDR(.data));       LONG(SIZEOF(.data));
    LONG(-1);                LONG(-1);                LONG(-1);

    . = ALIGN(8);  /* Avoid gap to start of next section. */
  } > CODE_LOAD  /* The copy table doesn't get copied!  That would be silly. */

  .config_param_header (NOLOAD) : ALIGN(4) {
    PROVIDE(ldscript_config_param_header = .);
    KEEP(*(.config_param_header))
  } > CONFIG_PARAMS

  .config_param_data (NOLOAD) : ALIGN(4) {
    /* Specify fixed address of kConfigParams. */
    . = ORIGIN(CONFIG_PARAMS) + local.kFlashParamHeaderSize;
    PROVIDE(ldscript_config_param_data = .);
    PROVIDE(kConfigParams = .);
    KEEP(*(.config_param_data))
  } > CONFIG_PARAMS

  .calib_param_header (NOLOAD) : ALIGN(4) {
    PROVIDE(ldscript_calib_param_header = .);
    KEEP(*(.calib_param_header))
  } > CALIB_PARAMS

  .calib_param_data (NOLOAD) : ALIGN(4) {
    /* Specify fixed address of kCalibParams. */
    . = ORIGIN(CALIB_PARAMS) + local.kFlashParamHeaderSize;
    PROVIDE(ldscript_calib_param_data = .);
    PROVIDE(kCalibParams = .);
    KEEP(*(.calib_param_data))
  } > CALIB_PARAMS

  .serial_param_header (NOLOAD) : ALIGN(4) {
    PROVIDE(ldscript_serial_param_header = .);
    KEEP(*(.serial_param_header))
  } > SERIAL_PARAMS

  .serial_param_data (NOLOAD) : ALIGN(4) {
    /* Specify fixed address of kSerialParams. */
    . = ORIGIN(SERIAL_PARAMS) + local.kFlashParamHeaderSize;
    PROVIDE(ldscript_serial_param_data = .);
    PROVIDE(kSerialParams = .);
    KEEP(*(.serial_param_data))
  } > SERIAL_PARAMS

  /*
   * Volatile sections.
   */

  /* Stacks come first in RAM so overflow runs into invalid memory.
   * This works in all link slots except slot_ram_tms570, in which case
   * a stack overflow will clobber the vector table. */
  .stacks (NOLOAD) : ALIGN(8) {
    PROVIDE(ldscript_main_stack_top = .);
    . += local.kMainStackSize; PROVIDE(ldscript_main_stack = .);

    /* Align stack protection region to allow making it an MPU region. */
    . = ALIGN(local.kIrqProtectorSize);
    /* MPU makes IRQ protector non-accessible to catch IRQ stack overflow. */
    PROVIDE(ldscript_irq_protector_begin = .); . += local.kIrqProtectorSize;
    /* IRQ stack must immediately follow IRQ protector. */
    PROVIDE(ldscript_irq_stack_top = .);
    . += local.kIrqStackSize; PROVIDE(ldscript_irq_stack = .);

    /* Align stack protection region to allow making it an MPU region. */
    . = ALIGN(local.kFiqProtectorSize);
    /* MPU makes FIQ protector non-accessible to catch FIQ stack overflow. */
    PROVIDE(ldscript_fiq_protector_begin = .); . += local.kFiqProtectorSize;
    /* FIQ stack must immediately follow FIQ protector. */
    PROVIDE(ldscript_fiq_stack_top = .);
    . += local.kFiqStackSize; PROVIDE(ldscript_fiq_stack = .);

    /* If fatal exception handlers overflow into FIQ stack, no major harm.
     * If fatal exception handlers overflow into protector, crash will occur. */
    . += local.kFatalAbortStackSize;
    PROVIDE(ldscript_svc_stack = .);
    PROVIDE(ldscript_abt_stack = .);
    PROVIDE(ldscript_und_stack = .);
    /* Context stack occurs last. */
    PROVIDE(ldscript_context_stack_top = .);
    . += local.kContextStackSize; PROVIDE(ldscript_context_stack = .);
    /* Context stack pointer. */
    PROVIDE(ldscript_context_sp = .);
    LONG(0);
  } > DATA_RAM

  /* Initialized data (via *COPY_TABLE in .rodata), must be 4-byte multiple. */
  .data : ALIGN(8) {
    *(.data)
    *(.data.*)
    *(.gnu.linkonce.d.*)
    *(.got*)

    . = ALIGN(8);  /* Avoid gap to start of next section. */
  } > DATA_RAM AT> CODE_LOAD

  /* Zero-filled data (cleared in startup code), must be 4-byte multiple. */
  .bss (NOLOAD) : ALIGN(8) {
    PROVIDE(ldscript_bss_begin = .);
    *(.bss)
    *(.bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(4);
    PROVIDE(ldscript_bss_end = .);
  } > DATA_RAM

  /* Heap for C/C++ malloc/new (used by _sbrk). */
  .heap (NOLOAD) : ALIGN(8) {
    PROVIDE(ldscript_heap_begin = .);
    /* Save 1 word for .padding if DATA_RAM and CODE_LOAD are aliases. */
    . = ORIGIN(DATA_RAM) + LENGTH(DATA_RAM) - 4;  /* consume the rest of RAM */
    PROVIDE(ldscript_heap_end = .);
  } > DATA_RAM

  /* Debug RAM. */
  .debug_log (NOLOAD) : ALIGN(4) {
    PROVIDE(ldscript_log_magic = .);
    LONG(0);
    PROVIDE(ldscript_log_size = .);
    LONG(0);
    PROVIDE(ldscript_log_head = .);
    LONG(0);
    PROVIDE(ldscript_log_tail = .);
    LONG(0);
    PROVIDE(ldscript_log_data = .);
    . = ORIGIN(DEBUG_RAM) + LENGTH(DEBUG_RAM) - 4;
    LONG(0);
  } > DEBUG_RAM

  /* Special bootloader-only reserved memory. */
  .boot_state (NOLOAD) : ALIGN(4) {
    PROVIDE(ldscript_boot_stage = .);
    LONG(0);
    LONG(0);
    LONG(0);
    LONG(0);
  } > BOOT_RAM

  /* Exception handling and other unused/discarded data. */
  /DISCARD/ : {
    *(.eh_frame*)
    *(.gcc_except_table)
    *(.ARM.exidx*)
    *(.ARM.extab*)
  }
}
