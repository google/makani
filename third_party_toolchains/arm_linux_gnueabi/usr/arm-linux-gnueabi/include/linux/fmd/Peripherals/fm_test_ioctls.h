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
 @File          fm_test_ioctls.h

 @Description   FM Char device ioctls
*//***************************************************************************/
#ifndef __FM_TEST_IOCTLS_H
#define __FM_TEST_IOCTLS_H

#include "ioctls.h"


/**************************************************************************//**
 @Group         lnx_ioctl_FMT_grp Frame Manager Test Linux IOCTL API

 @Description   FM-Test Linux ioctls definitions and enums

 @{
*//***************************************************************************/

#define IOC_FMT_MAX_NUM_OF_PORTS        26

/**************************************************************************//**
 @Collection    TEST Parameters
*//***************************************************************************/
/**************************************************************************//**
  @Description: Name of the FM-Test chardev
*//***************************************************************************/
#define DEV_FM_TEST_NAME                "fm-test-port"

#define DEV_FM_TEST_PORTS_MINOR_BASE    0
#define DEV_FM_TEST_MAX_MINORS          (DEV_FM_TEST_PORTS_MINOR_BASE + IOC_FMT_MAX_NUM_OF_PORTS)

#define FMT_PORT_IOC_NUM(n)             n
/* @} */

/**************************************************************************//**
 @Group         lnx_ioctl_FMT_lib_grp FM-Test library

 @Description   TODO

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Description   TODO
*//***************************************************************************/
typedef uint8_t ioc_fmt_xxx_t;

#define FM_PRS_MAX 32
#define FM_TIME_STAMP_MAX 8

/**************************************************************************//**
 @Description   FM Port buffer content description
*//***************************************************************************/
typedef struct ioc_fmt_buff_context_t {
    void            *p_user_priv;
    uint8_t         fm_prs_res[FM_PRS_MAX];
    uint8_t         fm_time_stamp[FM_TIME_STAMP_MAX];
} ioc_fmt_buff_context_t;


/**************************************************************************//**
 @Description   Buffer descriptor
*//***************************************************************************/
typedef struct ioc_fmt_buff_desc_t {
    uint32_t               qid;
    void                   *p_data;
    uint32_t               size;
    uint32_t               status;
    ioc_fmt_buff_context_t buff_context;
} ioc_fmt_buff_desc_t;


/**************************************************************************//**
 @Group         lnx_ioctl_FMT_runtime_control_grp FM-Test Runtime Control Unit

 @Description   TODO
 @{
*//***************************************************************************/

/** @} */ /* end of lnx_ioctl_FMT_runtime_control_grp group */


/**************************************************************************//**
 @Group         lnx_ioctl_FMTP_lib_grp FM-Port-Test library

 @Description   TODO

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Description   FM-Test FM port type
*//***************************************************************************/
typedef enum ioc_fmt_port_type {
    e_IOC_FMT_PORT_T_RXTX,  /**< Standard port */
    e_IOC_FMT_PORT_T_OP,    /**< Offline-parsing port */
} ioc_fmt_port_type;

/**************************************************************************//**
 @Description   TODO
*//***************************************************************************/
typedef struct ioc_fmt_port_param_t {
    uint8_t             fm_id;
    ioc_fmt_port_type   fm_port_type;
    uint8_t             fm_port_id;
    uint32_t            num_tx_queues;
} ioc_fmt_port_param_t;


/**************************************************************************//**
 @Function      FMT_PORT_IOC_INIT

 @Description   TODO

 @Param[in]     ioc_fmt_port_param_t  TODO

 @Cautions      Allowed only after the FM equivalent port is already initialized.
*//***************************************************************************/
#define FMT_PORT_IOC_INIT           _IOW(FMT_IOC_TYPE_BASE, FMT_PORT_IOC_NUM(0), ioc_fmt_port_param_t)

/**************************************************************************//**
 @Function      FMT_PORT_IOC_SET_DIAG_MODE

 @Description   TODO

 @Param[in]     ioc_diag_mode  TODO

 @Cautions      Allowed only following FMT_PORT_IOC_INIT().
*//***************************************************************************/
#define FMT_PORT_IOC_SET_DIAG_MODE  _IOW(FMT_IOC_TYPE_BASE, FMT_PORT_IOC_NUM(1), ioc_diag_mode)

/**************************************************************************//**
 @Function      FMT_PORT_IOC_SET_IP_HEADER_MANIP

 @Description   Set IP header manipulations for this port.

 @Param[in]     int     1 to enable; 0 to disable

 @Cautions      Allowed only following FMT_PORT_IOC_INIT().
*//***************************************************************************/
#define FMT_PORT_IOC_SET_IP_HEADER_MANIP  _IOW(FMT_IOC_TYPE_BASE, FMT_PORT_IOC_NUM(2), int)

/**************************************************************************//**
 @Function      FMT_PORT_IOC_SET_DPAECHO_MODE

 @Description   Set DPA in echo mode - all frame are sent back.

 @Param[in]     int     1 to enable; 0 to disable

 @Cautions      Allowed only following FMT_PORT_IOC_INIT().
*//***************************************************************************/
#define FMT_PORT_IOC_SET_DPAECHO_MODE     _IOW(FMT_IOC_TYPE_BASE, FMT_PORT_IOC_NUM(3), int)

/** @} */ /* end of lnx_ioctl_FMTP_lib_grp group */
/** @} */ /* end of lnx_ioctl_FMT_lib_grp group */
/** @} */ /* end of lnx_ioctl_FMT_grp */


#endif /* __FM_TEST_IOCTLS_H */
