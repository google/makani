/* Copyright (c) 2008-2011 Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************//**
 @File          ioctls.h

 @Description   Structures and definitions for Command Relay Ioctls
*//***************************************************************************/

#ifndef __IOCTLS_H__
#define __IOCTLS_H__

#include <asm/ioctl.h>

#include "integration_ioctls.h"


/**************************************************************************//**
 @Group         lnx_ioctl_ncsw_grp    NetCommSw Linux User-Space (IOCTL) API
 @{
*//***************************************************************************/

#define NCSW_IOC_TYPE_BASE          0xe0    /**< defines the IOCTL type for all
                                                 the NCSW Linux module commands */


/**************************************************************************//**
 @Description   IOCTL Memory allocation types.
*//***************************************************************************/
typedef enum ioc_mem_type {
    e_IOC_MEM_INVALID      = 0x00000000,  /**< Invalid memory type (error) */
    e_IOC_MEM_CACHABLE_SYS = 0x00000001,  /**< Primary DDR, cacheable segment */
    e_IOC_MEM_NOCACHE_SYS  = 0x00000004,  /**< Primary DDR, non-cacheable segment */
    e_IOC_MEM_SECONDARY    = 0x00000002,  /**< Either secondary DDR or SDRAM */
    e_IOC_MEM_PRAM         = 0x00000008   /**< Multi-user RAM identifier */
} ioc_mem_type;

/**************************************************************************//**
 @Description   Enumeration (bit flags) of communication modes (Transmit,
                receive or both).
*//***************************************************************************/
typedef enum ioc_comm_mode {
      e_IOC_COMM_MODE_NONE         = 0  /**< No transmit/receive communication */
    , e_IOC_COMM_MODE_RX           = 1  /**< Only receive communication */
    , e_IOC_COMM_MODE_TX           = 2  /**< Only transmit communication */
    , e_IOC_COMM_MODE_RX_AND_TX    = 3  /**< Both transmit and receive communication */
} ioc_comm_mode;

/**************************************************************************//**
 @Description   General Diagnostic Mode
*//***************************************************************************/
typedef enum ioc_diag_mode
{
    e_IOC_DIAG_MODE_NONE = 0,
    e_IOC_DIAG_MODE_CTRL_LOOPBACK,      /**< loopback in the controller; E.g. MAC, TDM, etc. */
    e_IOC_DIAG_MODE_CHIP_LOOPBACK,      /**< loopback in the chip but not in controller;
                                         E.g. IO-pins, SerDes, etc. */
    e_IOC_DIAG_MODE_PHY_LOOPBACK,       /**< loopback in the external PHY */
    e_IOC_DIAG_MODE_LINE_LOOPBACK,      /**< loopback in the external line */
    e_IOC_DIAG_MODE_CTRL_ECHO,          /**< */
    e_IOC_DIAG_MODE_PHY_ECHO            /**< */
} ioc_diag_mode;

/** @} */ /* end of lnx_ioctl_ncsw_grp */


#endif /* __IOCTLS_H__ */
