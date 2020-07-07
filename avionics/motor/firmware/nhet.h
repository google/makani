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

#ifndef AVIONICS_MOTOR_FIRMWARE_NHET_H_
#define AVIONICS_MOTOR_FIRMWARE_NHET_H_

#include <stdint.h>

typedef struct {
  uint32_t program;
  uint32_t control;
  uint32_t data;
  uint32_t reserved;
} __attribute__((__packed__)) Instruction;

// N2HET instruction set [TRM 23.5].
typedef union {
  Instruction raw;
  struct {
    unsigned int :6;
    unsigned int REQUEST_NUMBER:3;
    unsigned int BRK:1;
    unsigned int NEXT_PROGRAM_ADDRESS:9;
    unsigned int OP_CODE:4;
    unsigned int ANGLE_COUNT:1;
    unsigned int REGISTER:2;
    unsigned int COMP_SELECT:1;
    unsigned int :4;
    unsigned int INT_EN:1;

    unsigned int :3;
    unsigned int REQUEST_TYPE:2;
    unsigned int CONTROL:1;
    unsigned int :1;
    unsigned int MAX_COUNT:25;

    unsigned int DATA:25;
    unsigned int :7;

    unsigned int :32;
  } __attribute__((__packed__));
} CntInstruction;

typedef union {
  Instruction raw;
  struct {
    unsigned int :6;
    unsigned int REQUEST_NUMBER:3;
    unsigned int BRK:1;
    unsigned int NEXT_PROGRAM_ADDRESS:9;
    unsigned int OP_CODE:4;
    unsigned int HR_LR:1;
    unsigned int ANGLE_COMP:1;
    unsigned int :7;

    unsigned int :3;
    unsigned int REQUEST_TYPE:2;
    unsigned int CONTROL:1;
    unsigned int :3;
    unsigned int EN_PIN_ACTION:1;
    unsigned int CONDITIONAL_ADDRESS:9;
    unsigned int PIN_SELECT:5;
    unsigned int EXT_REG:1;
    unsigned int SUB_OPCODE:2;
    unsigned int ACTION:2;
    unsigned int REGISTER_SELECT:2;
    unsigned int INT_EN:1;

    unsigned int DATA:25;
    unsigned int HR_DATA:7;

    unsigned int :32;
  } __attribute__((__packed__));
} EcmpInstruction;

typedef union {
  Instruction raw;
  struct {
    unsigned int :6;
    unsigned int REQUEST_NUMBER:3;
    unsigned int BRK:1;
    unsigned int NEXT_PROGRAM_ADDRESS:9;
    unsigned int OP_CODE:4;
    unsigned int :1;
    unsigned int :2;
    unsigned int :6;

    unsigned int :3;
    unsigned int REQUEST_TYPE:2;
    unsigned int CONTROL:1;
    unsigned int PRV:1;
    unsigned int :3;
    unsigned int CONDITIONAL_ADDRESS:9;
    unsigned int PIN_SELECT:5;
    unsigned int EXT_REG:1;
    unsigned int EVENT:3;
    unsigned int :1;
    unsigned int REGISTER_SELECT:2;
    unsigned int INT_EN:1;

    unsigned int DATA:25;
    unsigned int :7;

    unsigned int :32;
  } __attribute__((__packed__));
} EcntInstruction;

typedef union {
  Instruction raw;
  struct {
    unsigned int :9;
    unsigned int BRK:1;
    unsigned int NEXT_PROGRAM_ADDRESS:9;
    unsigned int OP_CODE:4;
    unsigned int REMOTE_ADDRESS:9;

    unsigned int :5;
    unsigned int CONTROL:1;
    unsigned int :3;
    unsigned int Z_FL_COND:1;
    unsigned int :14;
    unsigned int EXT_REG:1;
    unsigned int INIT_FLAG:1;
    unsigned int SUB_OPCODE:1;
    unsigned int MOVE_TYPE:2;
    unsigned int REGISTER_SELECT:2;
    unsigned int :1;

    unsigned int DATA:25;
    unsigned int HR_DATA:7;

    unsigned int :32;
  } __attribute__((__packed__));
} Mov32Instruction;

#define HET1RAM (((volatile union HetRam1Frame *)0xFF460000)[0])
union HetRam1Frame {
  Instruction raw[28];
  struct {
    CntInstruction START;
    CntInstruction REAL_CNT;
    Mov32Instruction LAST_CHK;
    EcmpInstruction BRANCH;
    EcmpInstruction YA1;
    EcmpInstruction YA2;
    EcmpInstruction YAN1;
    EcmpInstruction YAN2;
    EcmpInstruction YB1;
    EcmpInstruction YB2;
    EcmpInstruction YBN1;
    EcmpInstruction YBN2;
    EcmpInstruction YC1;
    EcmpInstruction YC2;
    EcmpInstruction YCN1;
    EcmpInstruction YCN2;
    EcmpInstruction ZA1;
    EcmpInstruction ZA2;
    EcmpInstruction ZAN1;
    EcmpInstruction ZAN2;
    EcmpInstruction ZB1;
    EcmpInstruction ZB2;
    EcmpInstruction ZBN1;
    EcmpInstruction ZBN2;
    EcmpInstruction ZC1;
    EcmpInstruction ZC2;
    EcmpInstruction ZCN1;
    EcmpInstruction ZCN2;
  } __attribute__((__packed__));
};

#define HET2RAM (((volatile union HetRam2Frame *)0xFF440000)[0])
union HetRam2Frame {
  Instruction raw[10];
  struct {
    EcntInstruction THERM1;
    EcntInstruction THERM2;
    EcntInstruction THERM3;
    CntInstruction COUNT;
    Mov32Instruction LATCH1;
    Mov32Instruction CLEAN1;
    Mov32Instruction LATCH2;
    Mov32Instruction CLEAN2;
    Mov32Instruction LATCH3;
    Mov32Instruction CLEAN3;
  } __attribute__((__packed__));
};

extern const Instruction kHetInitSvpwm[28];
extern const Instruction kHetInitThermal[10];

#endif  // AVIONICS_MOTOR_FIRMWARE_NHET_H_
