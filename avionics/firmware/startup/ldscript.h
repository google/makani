/*
 * Copyright 2020 Makani Technologies LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Symbols exposed by linker scripts.

#ifndef AVIONICS_FIRMWARE_STARTUP_LDSCRIPT_H_
#define AVIONICS_FIRMWARE_STARTUP_LDSCRIPT_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/startup/ldscript_types.h"

//
// Memory layout.
//

// Flash memory to protect (load address of running program).
extern uint8_t ldscript_protect_begin[];
extern uint8_t ldscript_protect_end[];

// Start and end of the full flash memory.
extern uint8_t ldscript_flash_begin[];
extern uint8_t ldscript_flash_end[];

// Boot loader flash segment.
extern uint8_t ldscript_boot_flash_begin[];
extern uint8_t ldscript_boot_flash_end[];

// Start and end of the main application flash (whether or not this program
// runs there).
extern uint8_t ldscript_app_flash_begin[];
extern uint8_t ldscript_app_flash_end[];
extern uint8_t ldscript_app_flash_crc[];

// The application config in the application flash.
extern AppConfig ldscript_app_config;

// The boot config in the bootloader flash (whether or not this program runs
// there).
extern BootConfig ldscript_boot_config;

// Full RAM region which can be useful for computing RAM size (like for MPU
// setup) or seeing if certain data resides in RAM (instead of Flash).
extern uint8_t ldscript_ram_begin[];
extern uint8_t ldscript_ram_end[];

// System stacks.
extern uint8_t ldscript_main_stack_top[];  // C stack overflow point (min addr).
extern uint8_t ldscript_main_stack[];  // C stack starting point (max addr).

// TMS570-specific system stacks.
extern uint8_t ldscript_irq_protector_begin[];  // IRQ stack overflow protector.
extern uint8_t ldscript_irq_stack_top[];  // IRQ stack top, end of protector.
extern uint8_t ldscript_irq_stack[];  // Stack for IRQ handlers (max addr).
extern uint8_t ldscript_fiq_protector_begin[];  // FIQ stack overflow protector.
extern uint8_t ldscript_fiq_stack_top[];  // FIQ stack top, end of protector.
extern uint8_t ldscript_fiq_stack[];  // Stack for FIQ handlers (max addr).
extern uint8_t ldscript_svc_stack[];  // Stack for SVC trap handlers (max addr).
extern uint8_t ldscript_abt_stack[];  // Stack for ABT trap handlers (max addr).
extern uint8_t ldscript_und_stack[];  // Stack for UND trap handlers (max addr).

// Zero-initialized data.
extern uint8_t ldscript_bss_begin[];
extern uint8_t ldscript_bss_end[];

// The heap, managed by brk()/sbrk() and malloc()/free().
extern uint8_t ldscript_heap_begin[];
extern uint8_t ldscript_heap_end[];

//
// Tables of data for startup code.
//

// Memory regions to copy during startup or checksum. Terminated by size=-1.
extern struct { const uint8_t *from; uint8_t *to; int32_t size;
} ldscript_copy_table[];

//
// Parameter sections.
//
extern const void ldscript_config_param_begin;
extern const void ldscript_config_param_end;
extern const void ldscript_config_param_header;
extern const void ldscript_config_param_data;
extern const void ldscript_calib_param_begin;
extern const void ldscript_calib_param_end;
extern const void ldscript_calib_param_header;
extern const void ldscript_calib_param_data;
extern const void ldscript_serial_param_begin;
extern const void ldscript_serial_param_end;
extern const void ldscript_serial_param_header;
extern const void ldscript_serial_param_data;

//
// Bootloader state.
//

// Set to magic value to enable interrupt trampolining to application.
extern uint32_t ldscript_boot_stage;

void StartupCopyData(void) __attribute__((section(".text.startup")));
void StartupClearBss(void) __attribute__((section(".text.startup")));
bool IsRunningFromBootSegment(void);

int32_t LdscriptReadInt32(int32_t *address);
int16_t LdscriptReadInt16(int16_t *address);
uint8_t LdscriptReadUint8(uint8_t *address);

#endif  // AVIONICS_FIRMWARE_STARTUP_LDSCRIPT_H_
