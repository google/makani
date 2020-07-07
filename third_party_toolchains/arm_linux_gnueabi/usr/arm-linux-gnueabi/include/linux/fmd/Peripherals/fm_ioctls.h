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
 @File          fm_ioctls.h

 @Description   FM Char device ioctls
*//***************************************************************************/
#ifndef __FM_IOCTLS_H
#define __FM_IOCTLS_H


/**************************************************************************//**
 @Group         lnx_ioctl_FM_grp Frame Manager Linux IOCTL API

 @Description   FM Linux ioctls definitions and enums

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Collection    FM IOCTL device ('/dev') definitions
*//***************************************************************************/
#define DEV_FM_NAME                 "fm" /**< Name of the FM chardev */

#define DEV_FM_MINOR_BASE           0
#define DEV_FM_PCD_MINOR_BASE       (DEV_FM_MINOR_BASE + 1)                                 /*/dev/fmx-pcd */
#define DEV_FM_OH_PORTS_MINOR_BASE  (DEV_FM_PCD_MINOR_BASE + 1)                             /*/dev/fmx-port-ohy */
#define DEV_FM_RX_PORTS_MINOR_BASE  (DEV_FM_OH_PORTS_MINOR_BASE + FM_MAX_NUM_OF_OH_PORTS)   /*/dev/fmx-port-rxy */
#define DEV_FM_TX_PORTS_MINOR_BASE  (DEV_FM_RX_PORTS_MINOR_BASE + FM_MAX_NUM_OF_RX_PORTS)   /*/dev/fmx-port-txy */
#define DEV_FM_MAX_MINORS           (DEV_FM_TX_PORTS_MINOR_BASE + FM_MAX_NUM_OF_TX_PORTS)


#define FM_IOC_NUM(n)       n
#define FM_PCD_IOC_NUM(n)   (n+20)
#define FM_PORT_IOC_NUM(n)  (n+50)
/* @} */

#define IOC_FM_MAX_NUM_OF_PORTS         64

/**************************************************************************//**
 @Collection   FM Frame error
*//***************************************************************************/
typedef uint32_t    ioc_fm_port_frame_err_select_t;                     /**< typedef for defining Frame Descriptor errors */

#define IOC_FM_PORT_FRM_ERR_UNSUPPORTED_FORMAT              0x04000000  /**< Offline parsing only! Unsupported Format */
#define IOC_FM_PORT_FRM_ERR_LENGTH                          0x02000000  /**< Offline parsing only! Length Error */
#define IOC_FM_PORT_FRM_ERR_DMA                             0x01000000  /**< DMA Data error */
#ifdef FM_CAPWAP_SUPPORT
#define IOC_FM_PORT_FRM_ERR_NON_FM                          0x00400000  /**< non FMan error; probably come from SEC chained to FM */
#endif /* FM_CAPWAP_SUPPORT */
#define IOC_FM_PORT_FRM_ERR_PHYSICAL                        0x00080000  /**< Rx FIFO overflow, FCS error, code error, running disparity
                                                                         error (SGMII and TBI modes), FIFO parity error. PHY
                                                                         Sequence error, PHY error control character detected. */
#define IOC_FM_PORT_FRM_ERR_SIZE                            0x00040000  /**< Frame too long OR Frame size exceeds max_length_frame  */
#define IOC_FM_PORT_FRM_ERR_CLS_DISCARD                     0x00020000  /**< classification discard */
#define IOC_FM_PORT_FRM_ERR_EXTRACTION                      0x00008000  /**< Extract Out of Frame */
#define IOC_FM_PORT_FRM_ERR_NO_SCHEME                       0x00004000  /**< No Scheme Selected */
#define IOC_FM_PORT_FRM_ERR_KEYSIZE_OVERFLOW                0x00002000  /**< No Scheme Selected */
#define IOC_FM_PORT_FRM_ERR_COLOR_YELLOW                    0x00000400  /**< */
#define IOC_FM_PORT_FRM_ERR_COLOR_RED                       0x00000800  /**< */
#define IOC_FM_PORT_FRM_ERR_ILL_PLCR                        0x00000200  /**< Illegal Policer Profile selected */
#define IOC_FM_PORT_FRM_ERR_PLCR_FRAME_LEN                  0x00000100  /**< Illegal Policer Profile selected */
#define IOC_FM_PORT_FRM_ERR_PRS_TIMEOUT                     0x00000080  /**< Parser Time out Exceed */
#define IOC_FM_PORT_FRM_ERR_PRS_ILL_INSTRUCT                0x00000040  /**< Invalid Soft Parser instruction */
#define IOC_FM_PORT_FRM_ERR_PRS_HDR_ERR                     0x00000020  /**< Header error was identified during parsing */
#define IOC_FM_PORT_FRM_ERR_BLOCK_LIMIT_EXCEEDED            0x00000008  /**< Frame parsed beyind 256 first bytes */
#define IOC_FM_PORT_FRM_ERR_PROCESS_TIMEOUT                 0x00000001  /**< FPT Frame Processing Timeout Exceeded */
/* @} */


/**************************************************************************//**
 @Description   enum for defining port types
                (must match enum e_FmPortType defined in fm_ext.h)
*//***************************************************************************/
typedef enum ioc_fm_port_type {
    e_IOC_FM_PORT_TYPE_OFFLINE_PARSING, /**< Offline parsing port (id's: 0-6, share id's with
                                             host command, so must have exclusive id) */
    e_IOC_FM_PORT_TYPE_HOST_COMMAND,    /**< Host command port (id's: 0-6, share id's with
                                             offline parsing ports, so must have exclusive id) */
    e_IOC_FM_PORT_TYPE_RX,              /**< 1G Rx port (id's: 0-3) */
    e_IOC_FM_PORT_TYPE_RX_10G,          /**< 10G Rx port (id's: 0) */
    e_IOC_FM_PORT_TYPE_TX,              /**< 1G Tx port (id's: 0-3) */
    e_IOC_FM_PORT_TYPE_TX_10G,          /**< 10G Tx port (id's: 0) */
    e_IOC_FM_PORT_TYPE_DUMMY
} ioc_fm_port_type;


/**************************************************************************//**
 @Group         lnx_ioctl_FM_lib_grp FM library

 @Description   FM API functions, definitions and enums
                The FM module is the main driver module and is a mandatory module
                for FM driver users. Before any further module initialization,
                this module must be initialized.
                The FM is a "single-tone" module. It is responsible of the common
                HW modules: FPM, DMA, common QMI, common BMI initializations and
                run-time control routines. This module must be initialized always
                when working with any of the FM modules.
                NOTE - We assumes that the FML will be initialize only by core No. 0!

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Description   FM Exceptions
*//***************************************************************************/
typedef enum ioc_fm_exceptions {
    e_IOC_FM_EX_DMA_BUS_ERROR,              /**< DMA bus error. */
    e_IOC_FM_EX_DMA_READ_ECC,               /**< Read Buffer ECC error */
    e_IOC_FM_EX_DMA_SYSTEM_WRITE_ECC,       /**< Write Buffer ECC error on system side */
    e_IOC_FM_EX_DMA_FM_WRITE_ECC,           /**< Write Buffer ECC error on FM side */
    e_IOC_FM_EX_FPM_STALL_ON_TASKS ,        /**< Stall of tasks on FPM */
    e_IOC_FM_EX_FPM_SINGLE_ECC,             /**< Single ECC on FPM. */
    e_IOC_FM_EX_FPM_DOUBLE_ECC,             /**< Double ECC error on FPM ram access */
    e_IOC_FM_EX_QMI_SINGLE_ECC,             /**< Single ECC on QMI. */
    e_IOC_FM_EX_QMI_DOUBLE_ECC,             /**< Double bit ECC occured on QMI */
    e_IOC_FM_EX_QMI_DEQ_FROM_UNKNOWN_PORTID,/**< Dequeu from unknown port id */
    e_IOC_FM_EX_BMI_LIST_RAM_ECC,           /**< Linked List RAM ECC error */
    e_IOC_FM_EX_BMI_PIPELINE_ECC,           /**< Pipeline Table ECC Error */
    e_IOC_FM_EX_BMI_STATISTICS_RAM_ECC,     /**< Statistics Count RAM ECC Error Enable */
    e_IOC_FM_EX_BMI_DISPATCH_RAM_ECC,       /**< Dispatch RAM ECC Error Enable */
    e_IOC_FM_EX_IRAM_ECC,                   /**< Double bit ECC occured on IRAM*/
    e_IOC_FM_EX_MURAM_ECC                   /**< Double bit ECC occured on MURAM*/
} ioc_fm_exceptions;


/**************************************************************************//**
 @Group         lnx_ioctl_FM_runtime_control_grp FM Runtime Control Unit

 @Description   FM Runtime control unit API functions, definitions and enums.
                The FM driver provides a set of control routines for each module.
                These routines may only be called after the module was fully
                initialized (both configuration and initialization routines were
                called). They are typically used to get information from hardware
                (status, counters/statistics, revision etc.), to modify a current
                state or to force/enable a required action. Run-time control may
                be called whenever necessary and as many times as needed.
 @{
*//***************************************************************************/

/**************************************************************************//**
 @Collection   General FM defines.
 *//***************************************************************************/
#define IOC_FM_MAX_NUM_OF_VALID_PORTS (FM_MAX_NUM_OF_OH_PORTS + \
        FM_MAX_NUM_OF_1G_RX_PORTS +  \
        FM_MAX_NUM_OF_10G_RX_PORTS + \
        FM_MAX_NUM_OF_1G_TX_PORTS +  \
        FM_MAX_NUM_OF_10G_TX_PORTS)
/* @} */

/**************************************************************************//**
 @Description   Structure for Port bandwidth requirement. Port is identified
                by type and relative id.
                (must be identical to t_FmPortBandwidth defined in fm_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_port_bandwidth_t {
    ioc_fm_port_type    type;           /**< FM port type */
    uint8_t             relativePortId; /**< Type relative port id */
    uint8_t             bandwidth;      /**< bandwidth - (in term of percents) */
} ioc_fm_port_bandwidth_t;

/**************************************************************************//**
 @Description   A Structure containing an array of Port bandwidth requirements.
                The user should state the ports requiring bandwidth in terms of
                percentage - i.e. all port's bandwidths in the array must add
                up to 100.
                (must be identical to t_FmPortsBandwidthParams defined in fm_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_port_bandwidth_params {
    uint8_t                     numOfPorts;
                                /**< num of ports listed in the array below */
/*TODO:Andy64 BUG*/
    ioc_fm_port_bandwidth_t     portsBandwidths[IOC_FM_MAX_NUM_OF_VALID_PORTS];
                                /**< for each port, it's bandwidth (all port's
                                  bandwidths must add up to 100.*/
} ioc_fm_port_bandwidth_params;

/**************************************************************************//**
 @Description   enum for defining FM counters
*//***************************************************************************/
typedef enum ioc_fm_counters {
    e_IOC_FM_COUNTERS_ENQ_TOTAL_FRAME,              /**< QMI total enqueued frames counter */
    e_IOC_FM_COUNTERS_DEQ_TOTAL_FRAME,              /**< QMI total dequeued frames counter */
    e_IOC_FM_COUNTERS_DEQ_0,                        /**< QMI 0 frames from QMan counter */
    e_IOC_FM_COUNTERS_DEQ_1,                        /**< QMI 1 frames from QMan counter */
    e_IOC_FM_COUNTERS_DEQ_2,                        /**< QMI 2 frames from QMan counter */
    e_IOC_FM_COUNTERS_DEQ_3,                        /**< QMI 3 frames from QMan counter */
    e_IOC_FM_COUNTERS_DEQ_FROM_DEFAULT,             /**< QMI dequeue from default queue counter */
    e_IOC_FM_COUNTERS_DEQ_FROM_CONTEXT,             /**< QMI dequeue from FQ context counter */
    e_IOC_FM_COUNTERS_DEQ_FROM_FD,                  /**< QMI dequeue from FD command field counter */
    e_IOC_FM_COUNTERS_DEQ_CONFIRM,                  /**< QMI dequeue confirm counter */
    e_IOC_FM_COUNTERS_SEMAPHOR_ENTRY_FULL_REJECT,   /**< DMA semaphor reject due to full entry counter */
    e_IOC_FM_COUNTERS_SEMAPHOR_QUEUE_FULL_REJECT,   /**< DMA semaphor reject due to full CAM queue counter */
    e_IOC_FM_COUNTERS_SEMAPHOR_SYNC_REJECT          /**< DMA semaphor reject due to sync counter */
} ioc_fm_counters;

typedef struct ioc_fm_obj_t {
    void            *obj;
} ioc_fm_obj_t;

/**************************************************************************//**
 @Description   structure for returning revision information
*//***************************************************************************/
typedef struct ioc_fm_revision_info_t {
    uint8_t         major;               /**< Major revision */
    uint8_t         minor;               /**< Minor revision */
} ioc_fm_revision_info_t;

/**************************************************************************//**
 @Description   structure for FM counters
*//***************************************************************************/
typedef struct ioc_fm_counters_params_t {
    ioc_fm_counters cnt;                /**< The requested counter */
    uint32_t        val;                /**< The requested value to get/set from/into the counter */
} ioc_fm_counters_params_t;

/**************************************************************************//**
 @Function      FM_IOC_SET_PORTS_BANDWIDTH

 @Description   Sets relative weights between ports when accessing common resources.

 @Param[in]     ioc_fm_port_bandwidth_params    Port bandwidth percentages,
 their sum must equal 100.

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_Init().
*//***************************************************************************/
#define FM_IOC_SET_PORTS_BANDWIDTH                             _IOW(FM_IOC_TYPE_BASE, FM_IOC_NUM(2), ioc_fm_port_bandwidth_params)

/**************************************************************************//**
 @Function      FM_IOC_GET_REVISION

 @Description   Returns the FM revision

 @Param[out]    ioc_fm_revision_info_t  A structure of revision information parameters.

 @Return        None.

 @Cautions      Allowed only following FM_Init().
*//***************************************************************************/
#define FM_IOC_GET_REVISION                                    _IOR(FM_IOC_TYPE_BASE, FM_IOC_NUM(3), ioc_fm_revision_info_t)

/**************************************************************************//**
 @Function      FM_IOC_GET_COUNTER

 @Description   Reads one of the FM counters.

 @Param[in,out] ioc_fm_counters_params_t The requested counter parameters.

 @Return        Counter's current value.

 @Cautions      Allowed only following FM_Init().
                Note that it is user's responsibilty to call this routine only
                for enabled counters, and there will be no indication if a
                disabled counter is accessed.
*//***************************************************************************/
#define FM_IOC_GET_COUNTER                                    _IOWR(FM_IOC_TYPE_BASE, FM_IOC_NUM(4), ioc_fm_counters_params_t)

/**************************************************************************//**
 @Function      FM_IOC_SET_COUNTER

 @Description   Sets a value to an enabled counter. Use "0" to reset the counter.

 @Param[in]     ioc_fm_counters_params_t The requested counter parameters.

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_Init().
*//***************************************************************************/
#define FM_IOC_SET_COUNTER                                    _IOW(FM_IOC_TYPE_BASE, FM_IOC_NUM(5), ioc_fm_counters_params_t)

/**************************************************************************//**
 @Function      FM_IOC_FORCE_INTR

 @Description   Causes an interrupt event on the requested source.

 @Param[in]     ioc_fm_exceptions   An exception to be forced.

 @Return        E_OK on success; Error code if the exception is not enabled,
                or is not able to create interrupt.

 @Cautions      Allowed only following FM_Init().
*//***************************************************************************/
#define FM_IOC_FORCE_INTR                                    _IOW(FM_IOC_TYPE_BASE, FM_IOC_NUM(6), ioc_fm_exceptions)

/** @} */ /* end of lnx_ioctl_FM_runtime_control_grp group */
/** @} */ /* end of lnx_ioctl_FM_lib_grp group */
/** @} */ /* end of lnx_ioctl_FM_grp */


#endif /* __FM_IOCTLS_H */
