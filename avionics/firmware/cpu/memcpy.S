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

    .syntax unified

    /* FastCopy(int32_t n, const void *src, void *dst); */
    /* FastCopy R0=n, R1=src, R2=dst */
    /* This function utilizes load multiple (LDM) and store multiple (STM)
     * instructions to transfer 8 words per iteration. It copies as much
     * data as possible using this technique, then falls back to an unrolled,
     * 4 word copy, 1 word copy, and finally 1 byte copy. For small copies,
     * we jump to WordCopy to reduce copying to/from the stack. */
    .global FastCopy
    .thumb_func
FastCopy:
    orr     r3, r1, r2
    tst     r3, #0x03  /* Handle unaligned source address. */
    bne     WordCopy
    asrs    r3, r0, #5  /* Handle n < 32 (including negatives). */
    ble     WordCopy  /* Optimize stack push/pop cost. */
    push {r4-r11}

    /* Copy 8 words per loop iteration (requires aligned data). */
fast_copy_8_word_loop:
    ldmia   r1!, {r4-r11}  /* Copy 8 words at a time (32 bytes). */
    stmia   r2!, {r4-r11}
    subs    r3, r3, #1
    bgt     fast_copy_8_word_loop
    and     r0, r0, #0x1F  /* Uncopied data (0 <= n && n < 32). */

    /* Copy 4 words. */
    asrs    r3, r0, #4  /* Handle n < 16 (including negatives). */
    ble     fast_copy_1_word
    ldr     r4, [r1], #4
    str     r4, [r2], #4
    ldr     r5, [r1], #4
    str     r5, [r2], #4
    ldr     r6, [r1], #4
    str     r6, [r2], #4
    ldr     r7, [r1], #4
    str     r7, [r2], #4
    and     r0, r0, #0x0F  /* Uncopied data (0 <= n && n < 16). */

    /* Copy 1 word per loop iteration. */
fast_copy_1_word:
    asrs    r3, r0, #2  /* Handle n < 4 (including negatives). */
    ble     fast_copy_1_byte
fast_copy_1_word_loop:
    ldr     r4, [r1], #4
    str     r4, [r2], #4
    subs    r3, r3, #1
    bgt     fast_copy_1_word_loop
    ands    r0, r0, #0x03  /* Uncopied data (0 <= n && n < 4). */
    beq     fast_copy_done

    /* Copy 1 byte per loop iteration. */
fast_copy_1_byte:
    cmp     r0, #0  /* Handle n <= 0 (including negatives). */
    ble     fast_copy_done
fast_copy_1_byte_loop:
    ldrb    r4, [r1], #1
    strb    r4, [r2], #1
    subs    r0, r0, #1
    bgt     fast_copy_1_byte_loop

    /* Finished! */
fast_copy_done:
    pop {r4-r11}
    bx      lr


    /* WordCopy(int32_t n, const void *src, void *dst); */
    /* WordCopy R0=n, R1=src, R2=dst */
    .global WordCopy
    .thumb_func
WordCopy:
    /* Copy 4 words per loop iteration. */
    push {r4-r5}
    asrs    r3, r0, #4  /* Handle n < 16 (including negatives). */
    ble     word_copy_1_word
word_copy_4_word_loop:
    ldr     r4, [r1], #4
    str     r4, [r2], #4
    ldr     r5, [r1], #4
    str     r5, [r2], #4
    ldr     r4, [r1], #4
    str     r4, [r2], #4
    ldr     r5, [r1], #4
    str     r5, [r2], #4
    subs    r3, r3, #1
    bgt     word_copy_4_word_loop
    and     r0, r0, #0x0F  /* Uncopied data (0 <= n && n < 16). */

    /* Copy 1 word per loop iteration. */
word_copy_1_word:
    asrs    r3, r0, #2  /* Handle n < 4 (including negatives). */
    ble     word_copy_1_byte
word_copy_1_word_loop:
    ldr     r4, [r1], #4
    str     r4, [r2], #4
    subs    r3, r3, #1
    bgt     word_copy_1_word_loop
    ands    r0, r0, #0x03  /* Uncopied data (0 <= n && n < 4). */
    beq     word_copy_done

    /* Copy 1 byte per loop iteration. */
word_copy_1_byte:
    cmp     r0, #0  /* Handle n <= 0 (including negatives). */
    ble     word_copy_done
word_copy_1_byte_loop:
    ldrb    r4, [r1], #1
    strb    r4, [r2], #1
    subs    r0, r0, #1
    bgt     word_copy_1_byte_loop

    /* Finished! */
word_copy_done:
    pop {r4-r5}
    bx      lr
