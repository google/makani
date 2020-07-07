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

// Space vector pulse width modulation.  In a three phase inverter,
// shown below, exactly one of the complementary sets of switches
// (e.g. A and AN) is open at any given time.  Thus, there are eight
// possible switch configurations identified by the eight triples:
// (ABC) = (000), (001), ..., (111), that represent which set of top
// switches are open or closed.  By varying the switching times
// between these configurations, as shown in the hexagon diagram
// below, it is possible to approximately reproduce a reference
// voltage and phase angle.  The SVPWM equations here are slightly
// modified from [1] as this implementation assumes a v_ref magnitude
// from a power-variant Park Transform rather than the power-invariant
// DQ0 Transform used in [1].
//
//  i_bus  -----+------+------+
//              |      |      |
//            A  /   B  /   C  /
//              |      |      |
//              +------------------  ia
//              |      |      |
//              |      +-----------  ib
//              |      |      |
//              |      |      +----  ic
//              |      |      |
//           AN  /  BN  /  CN  /
//              |      |      |
//         -----+------+------+
//
//
//                             q-axis
//                           ^
//                           |
//             (010)         |         (110)
//                   x       |       x
//                    '      |      '
//                     '     II    '
//                      '    |    ' ------o  v_ref @ angle
//                       '   |   '       /
//                 III    '  |  '     I /  t2
//      (011)              ' | '  t1   /     (100)
//            x  .  .  .  .  +---------------x---->  d-axis
//                      (000) (111)          |
//                        ,     ,            (2/3) * v_bus
//                 IV    ,       ,    VI
//                      ,         ,
//                     ,     V     ,
//                    ,             ,
//                   x               x
//             (001)                   (101)
//
//
// [1] H. Broeck et al., Analysis and realization of a pulsewidth
//     modulator based on space vectors. IEEE Trans. Industry. App.,
//     24, 1 (1988)

#ifndef AVIONICS_MOTOR_FIRMWARE_SVPWM_H_
#define AVIONICS_MOTOR_FIRMWARE_SVPWM_H_

#include <stdbool.h>
#include <stdint.h>

// Initializes N2HET1 for SVPWM.
//
// svpwm_freq   Pulse width modulation frequency [Hz].
// pwm_per_isr  Ratio between the PWM frequency and the ISR control frequency.
//              Because only integer values are supported, this is equivalent to
//              the number of switching cycles between ISR control updates.
void SvpwmInit(float svpwm_freq, int32_t pwm_per_isr);

// Checks status of HETPINENA bit controlled by nDIS pin.
bool SvpwmCheckHetPinEna(void);

// Starts N2HET1.
void SvpwmStart(void);

// Sets SVPWM reference phase angle and voltage.
//
// angle  Phase angle of reference voltage vector [rad].
// v_ref  Magnitude of reference voltage vector [V].
// v_bus  Bus voltage [V].
void SvpwmSetReference(float angle, float v_ref, float v_bus);

extern float g_svpwm_isr_period;
extern float g_svpwm_vref_vbus_limit;

#endif  // AVIONICS_MOTOR_FIRMWARE_SVPWM_H_
