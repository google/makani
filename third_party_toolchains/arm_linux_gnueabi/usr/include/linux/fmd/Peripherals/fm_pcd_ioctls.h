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

/******************************************************************************
 @File          fm_pcd_ioctls.h

 @Description   FM PCD ...
*//***************************************************************************/
#ifndef __FM_PCD_IOCTLS_H
#define __FM_PCD_IOCTLS_H

#include "net_ioctls.h"
#include "fm_ioctls.h"


/**************************************************************************//**
 @Group         lnx_ioctl_FM_grp Frame Manager Linux IOCTL API

 @Description   FM Linux ioctls definitions and enums

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Group         lnx_ioctl_FM_PCD_grp FM PCD

 @Description   FM PCD API functions, definitions and enums

                The FM PCD module is responsible for the initialization of all
                global classifying FM modules. This includes the parser general and
                common registers, the key generator global and common registers,
                and the Policer global and common registers.
                In addition, the FM PCD SW module will initialize all required
                key generator schemes, coarse classification flows, and Policer
                profiles. When An FM module is configured to work with one of these
                entities, it will register to it using the FM PORT API. The PCD
                module will manage the PCD resources - i.e. resource management of
                Keygen schemes, etc.

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Collection    General PCD defines
*//***************************************************************************/
#define IOC_FM_PCD_MAX_NUM_OF_PRIVATE_HDRS              2                   /**< Number of units/headers saved for user */

#define IOC_FM_PCD_PRS_NUM_OF_HDRS                      16                  /**< Number of headers supported by HW parser */

#ifdef CONFIG_FMAN_P1023
#define IOC_FM_PCD_KG_NUM_OF_SCHEMES                    16                  /**< Total number of KG schemes */
#else
#define IOC_FM_PCD_KG_NUM_OF_SCHEMES                    32                  /**< Total number of KG schemes */
#endif
#define IOC_FM_PCD_MAX_NUM_OF_DISTINCTION_UNITS         (32 - IOC_FM_PCD_MAX_NUM_OF_PRIVATE_HDRS)
                                                                            /**< Maximum number of netenv distinction units */
#ifdef CONFIG_FMAN_P1023
#define IOC_FM_PCD_MAX_NUM_OF_OPTIONS                   7                   /**< Maximum number of netenv distinction units options */
#else
#define IOC_FM_PCD_MAX_NUM_OF_OPTIONS                   8                   /**< Maximum number of netenv distinction units options */
#endif
#define IOC_FM_PCD_MAX_NUM_OF_INTERCHANGEABLE_HDRS      4                   /**< Maximum number of interchangeable headers in a distinction unit */
#define IOC_FM_PCD_KG_NUM_OF_GENERIC_REGS               8                   /**< Total number of generic KG registers */
#define IOC_FM_PCD_KG_MAX_NUM_OF_EXTRACTS_PER_KEY       35                  /**< Max number allowed on any configuration.
                                                                                 For reason of HW implemetation, in most
                                                                                 cases less than this will be allowed. The
                                                                                 driver will return error in initialization
                                                                                 time if resource is overused. */
#ifdef CONFIG_FMAN_P1023
#define IOC_FM_PCD_MAX_NUM_OF_CLS_PLANS                 128                 /**< Number of classification plan entries. */
#else
#define IOC_FM_PCD_MAX_NUM_OF_CLS_PLANS                 256                 /**< Number of classification plan entries. */
#endif
#define IOC_FM_PCD_KG_NUM_OF_EXTRACT_MASKS              4                   /**< Total number of masks allowed on KG extractions. */
#define IOC_FM_PCD_KG_NUM_OF_DEFAULT_GROUPS             16                  /**< Number of default value logical groups */

#define IOC_FM_PCD_PRS_NUM_OF_LABELS                    32                  /**< Max number of SW parser label */
/* @} */

/**************************************************************************//**
 @Group         lnx_ioctl_FM_PCD_Runtime_grp FM PCD Runtime Unit

 @Description   FM PCD Runtime Unit

                The runtime control allows creation of PCD infrastructure modules
                such as Network Environment Characteristics, Classification Plan
                Groups and Coarse Classification Trees.
                It also allows on-the-fly initialization, modification and removal
                of PCD modules such as Keygen schemes, coarse classification nodes
                and Policer profiles.


                In order to explain the programming model of the PCD driver interface
                a few terms should be explained, and will be used below.
                  * Distinction Header - One of the 16 protocols supported by the FM parser,
                    or one of the shim headers (1-3). May be a header with a special
                    option (see below).
                  * Interchangeable Headers Group- This is a group of Headers recognized
                    by either one of them. For example, if in a specific context the user
                    chooses to treat IPv4 and IPV6 in the same way, they may create an
                    Interchangable Headers Unit consisting of these 2 headers.
                  * A Distinction Unit - a Distinction Header or an Interchangeable Headers
                    Group.
                  * Header with special option - applies to ethernet, mpls, vlan, ipv4 and
                    ipv6, includes multicast, broadcast and other protocol specific options.
                    In terms of hardware it relates to the options available in the classification
                    plan.
                  * Network Environment Characteristics - a set of Distinction Units that define
                    the total recognizable header selection for a certain environment. This is
                    NOT the list of all headers that will ever appear in a flow, but rather
                    everything that needs distinction in a flow, where distinction is made by keygen
                    schemes and coarse classification action descriptors.

                The PCD runtime modules initialization is done in stages. The first stage after
                initializing the PCD module itself is to establish a Network Flows Environment
                Definition. The application may choose to establish one or more such environments.
                Later, when needed, the application will have to state, for some of its modules,
                to which single environment it belongs.

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Description   PCD counters
                (must match enum e_FmPcdCounters defined in fm_pcd_ext.h)
*//***************************************************************************/
typedef enum ioc_fm_pcd_counters {
    e_IOC_FM_PCD_KG_COUNTERS_TOTAL,                                 /**< Policer counter */
    e_IOC_FM_PCD_PLCR_COUNTERS_YELLOW,                              /**< Policer counter */
    e_IOC_FM_PCD_PLCR_COUNTERS_RED,                                 /**< Policer counter */
    e_IOC_FM_PCD_PLCR_COUNTERS_RECOLORED_TO_RED,                    /**< Policer counter */
    e_IOC_FM_PCD_PLCR_COUNTERS_RECOLORED_TO_YELLOW,                 /**< Policer counter */
    e_IOC_FM_PCD_PLCR_COUNTERS_TOTAL,                               /**< Policer counter */
    e_IOC_FM_PCD_PLCR_COUNTERS_LENGTH_MISMATCH,                     /**< Policer counter */
    e_IOC_FM_PCD_PRS_COUNTERS_PARSE_DISPATCH,                       /**< Parser counter */
    e_IOC_FM_PCD_PRS_COUNTERS_L2_PARSE_RESULT_RETURNED,             /**< Parser counter */
    e_IOC_FM_PCD_PRS_COUNTERS_L3_PARSE_RESULT_RETURNED,             /**< Parser counter */
    e_IOC_FM_PCD_PRS_COUNTERS_L4_PARSE_RESULT_RETURNED,             /**< Parser counter */
    e_IOC_FM_PCD_PRS_COUNTERS_SHIM_PARSE_RESULT_RETURNED,           /**< Parser counter */
    e_IOC_FM_PCD_PRS_COUNTERS_L2_PARSE_RESULT_RETURNED_WITH_ERR,    /**< Parser counter */
    e_IOC_FM_PCD_PRS_COUNTERS_L3_PARSE_RESULT_RETURNED_WITH_ERR,    /**< Parser counter */
    e_IOC_FM_PCD_PRS_COUNTERS_L4_PARSE_RESULT_RETURNED_WITH_ERR,    /**< Parser counter */
    e_IOC_FM_PCD_PRS_COUNTERS_SHIM_PARSE_RESULT_RETURNED_WITH_ERR,  /**< Parser counter */
    e_IOC_FM_PCD_PRS_COUNTERS_SOFT_PRS_CYCLES,                      /**< Parser counter */
    e_IOC_FM_PCD_PRS_COUNTERS_SOFT_PRS_STALL_CYCLES,                /**< Parser counter */
    e_IOC_FM_PCD_PRS_COUNTERS_HARD_PRS_CYCLE_INCL_STALL_CYCLES,     /**< Parser counter */
    e_IOC_FM_PCD_PRS_COUNTERS_MURAM_READ_CYCLES,                    /**< MURAM counter */
    e_IOC_FM_PCD_PRS_COUNTERS_MURAM_READ_STALL_CYCLES,              /**< MURAM counter */
    e_IOC_FM_PCD_PRS_COUNTERS_MURAM_WRITE_CYCLES,                   /**< MURAM counter */
    e_IOC_FM_PCD_PRS_COUNTERS_MURAM_WRITE_STALL_CYCLES,             /**< MURAM counter */
    e_IOC_FM_PCD_PRS_COUNTERS_FPM_COMMAND_STALL_CYCLES              /**< FPM counter */
} ioc_fm_pcd_counters;

/**************************************************************************//**
 @Description   PCD interrupts
                (must match enum e_FmPcdExceptions defined in fm_pcd_ext.h)
*//***************************************************************************/
typedef enum ioc_fm_pcd_exceptions {
    e_IOC_FM_PCD_KG_EXCEPTION_DOUBLE_ECC,                   /**< Keygen ECC error */
    e_IOC_FM_PCD_PLCR_EXCEPTION_DOUBLE_ECC,                 /**< Read Buffer ECC error */
    e_IOC_FM_PCD_KG_EXCEPTION_KEYSIZE_OVERFLOW,             /**< Write Buffer ECC error on system side */
    e_IOC_FM_PCD_PLCR_EXCEPTION_INIT_ENTRY_ERROR,           /**< Write Buffer ECC error on FM side */
    e_IOC_FM_PCD_PLCR_EXCEPTION_PRAM_SELF_INIT_COMPLETE,        /**< Self init complete */
    e_IOC_FM_PCD_PLCR_EXCEPTION_ATOMIC_ACTION_COMPLETE,         /**< Atomic action complete */
    e_IOC_FM_PCD_PRS_EXCEPTION_DOUBLE_ECC,                      /**< Parser ECC error */
    e_IOC_FM_PCD_PRS_EXCEPTION_SINGLE_ECC                       /**< Parser single ECC */
} ioc_fm_pcd_exceptions;

/**************************************************************************//**
 @Description   structure for FM counters
*//***************************************************************************/
typedef struct ioc_fm_pcd_counters_params_t {
    ioc_fm_pcd_counters cnt;                /**< The requested counter */
    uint32_t            val;                /**< The requested value to get/set from/into the counter */
} ioc_fm_pcd_counters_params_t;

/**************************************************************************//**
 @Description   structure for FM exception definitios
*//***************************************************************************/
typedef struct ioc_fm_pcd_exception_params_t {
    ioc_fm_pcd_exceptions exception;        /**< The requested exception */
    bool                  enable;           /**< TRUE to enable interrupt, FALSE to mask it. */
} ioc_fm_pcd_exception_params_t;

/**************************************************************************//**
 @Description   A structure for sw parser labels
                (must be identical to struct t_FmPcdPrsLabelParams defined in fm_pcd_ext.h)
 *//***************************************************************************/
typedef struct ioc_fm_pcd_prs_label_params_t {
    uint32_t                instruction_offset;             /**< SW parser label instruction offset (2 bytes
                                                                 resolution), relative to Parser RAM. */
    ioc_net_header_type     hdr;                            /**< The existance of this header will envoke
                                                                 the sw parser code. */
    uint8_t                 index_per_hdr;                  /**< Normally 0, if more than one sw parser
                                                                 attachments for the same header, use this
                                                                 index to distinguish between them. */
} ioc_fm_pcd_prs_label_params_t;

/**************************************************************************//**
 @Description   A structure for sw parser
                (must be identical to struct t_FmPcdPrsSwParams defined in fm_pcd_ext.h)
 *//***************************************************************************/
typedef struct ioc_fm_pcd_prs_sw_params_t {
    bool                            override;           /**< FALSE to invoke a check that nothing else
                                                             was loaded to this address, including
                                                             internal patched.
                                                             TRUE to override any existing code.*/
    uint32_t                        size;               /**< SW parser code size */
    uint16_t                        base;               /**< SW parser base (in instruction counts!
                                                             muat be larger than 0x20)*/
    uint8_t                         *p_code;            /**< SW parser code */
    uint32_t                        sw_prs_data_params[IOC_FM_PCD_PRS_NUM_OF_HDRS];
                                                        /**< SW parser data (parameters) */
    uint8_t                         num_of_labels;      /**< Number of labels for SW parser. */
    ioc_fm_pcd_prs_label_params_t   labels_table[IOC_FM_PCD_PRS_NUM_OF_LABELS];
                                                        /**< SW parser labels table, containing n
                                                             umOfLabels entries */
} ioc_fm_pcd_prs_sw_params_t;

/**************************************************************************//**
 @Description   A structure to set the a KeyGen default value
 *//***************************************************************************/
typedef struct ioc_fm_pcd_kg_dflt_value_params_t {
    uint8_t                         valueId;                /**< 0,1 - one of 2 global default values */
    uint32_t                        value;                  /**< The requested default value */
} ioc_fm_pcd_kg_dflt_value_params_t;


/**************************************************************************//**
 @Function      FM_PCD_IOC_ENABLE

 @Description   This routine should be called after PCD is initialized for enabling all
                PCD engines according to their existing configuration.

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following PCD_Init() and when PCD is disabled.
*//***************************************************************************/
#define FM_PCD_IOC_ENABLE  _IO(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(1))

/**************************************************************************//**
 @Function      FM_PCD_IOC_DISABLE

 @Description   This routine may be called when PCD is enabled in order to
                disable all PCD engines. It may be called
                only when none of the ports in the system are using the PCD.

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following PCD_Init() and when PCD is enabled.
*//***************************************************************************/
#define FM_PCD_IOC_DISABLE  _IO(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(2))

 /**************************************************************************//**
 @Function      FM_PCD_IOC_PRS_LOAD_SW

 @Description   This routine may be called only when all ports in the
                system are actively using the classification plan scheme.
                In such cases it is recommended in order to save resources.
                The driver automatically saves 8 classification plans for
                ports that do NOT use the classification plan mechanism, to
                avoid this (in order to save those entries) this routine may
                be called.

 @Param[in]     ioc_fm_pcd_prs_sw_params_t  A pointer to the image of the software parser code.

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following PCD_Init() and when PCD is disabled.
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PCD_IOC_PRS_LOAD_SW_COMPAT  _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(3), ioc_compat_fm_pcd_prs_sw_params_t)
#endif
#define FM_PCD_IOC_PRS_LOAD_SW  _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(3), ioc_fm_pcd_prs_sw_params_t)

/**************************************************************************//**
 @Function      FM_PCD_IOC_KG_SET_DFLT_VALUE

 @Description   Calling this routine sets a global default value to be used
                by the keygen when parser does not recognize a required
                field/header.
                By default default values are 0.

 @Param[in]     ioc_fm_pcd_kg_dflt_value_params_t   A pointer to a structure with the relevant parameters

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following PCD_Init() and when PCD is disabled.
*//***************************************************************************/
#define FM_PCD_IOC_KG_SET_DFLT_VALUE  _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(6), ioc_fm_pcd_kg_dflt_value_params_t)

/**************************************************************************//**
 @Function      FM_PCD_IOC_KG_SET_ADDITIONAL_DATA_AFTER_PARSING

 @Description   Calling this routine allows the keygen to access data past
                the parser finishing point.

 @Param[in]     uint8_t   payload-offset; the number of bytes beyond the parser location.

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following PCD_Init() and when PCD is disabled.

*//***************************************************************************/
#define FM_PCD_IOC_KG_SET_ADDITIONAL_DATA_AFTER_PARSING  _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(7), uint8_t)

/**************************************************************************//**
 @Function      FM_PCD_IOC_SET_EXCEPTION

 @Description   Calling this routine enables/disables PCD interrupts.

 @Param[in]     h_FmPcd         FM PCD module descriptor.
 @Param[in]     ioc_fm_pcd_exception_params_t     The exception to be selected.
 @Param[in]     enable          TRUE to enable interrupt, FALSE to mask it.

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following PCD_Init().
*//***************************************************************************/
#define FM_PCD_IOC_SET_EXCEPTION _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(8), ioc_fm_pcd_exception_params_t)

/**************************************************************************//**
 @Function      FM_PCD_IOC_GET_COUNTER

 @Description   Reads one of the FM PCD counters.

 @Param[in,out] ioc_fm_pcd_counters_params_t The requested counter parameters.

 @Return        Counter's current value.

 @Cautions      Allowed only following FM_PCD_Init().
                Note that it is user's responsibilty to call this routine only
                for enabled counters, and there will be no indication if a
                disabled counter is accessed.
*//***************************************************************************/
#define FM_PCD_IOC_GET_COUNTER  _IOWR(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(9), ioc_fm_pcd_counters_params_t)

/**************************************************************************//**
 @Function      FM_PCD_IOC_SET_COUNTER

 @Description   Sets a value to an enabled counter. Use "0" to reset the counter.

 @Param[in]     ioc_fm_pcd_counters_params_t The requested counter parameters.

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following PCD_Init().
*//***************************************************************************/
#define FM_PCD_IOC_SET_COUNTER  _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(10), ioc_fm_pcd_counters_params_t)

/**************************************************************************//**
 @Function      FM_PCD_IOC_FORCE_INTR

 @Description   Causes an interrupt event on the requested source.

 @Param[in]     ioc_fm_pcd_exceptions    An exception to be forced.

 @Return        E_OK on success; Error code if the exception is not enabled,
                or is not able to create interrupt.

 @Cautions      Allowed only following PCD_Init().
*//***************************************************************************/
#define FM_PCD_IOC_FORCE_INTR _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(11), ioc_fm_pcd_exceptions)

/**************************************************************************//**
 @Collection    Definitions of coarse classification
                parameters as required by keygen (when coarse classification
                is the next engine after this scheme).
*//***************************************************************************/
#define IOC_FM_PCD_MAX_NUM_OF_CC_NODES          255
#define IOC_FM_PCD_MAX_NUM_OF_CC_TREES            8
#define IOC_FM_PCD_MAX_NUM_OF_CC_GROUPS          16
#define IOC_FM_PCD_MAX_NUM_OF_CC_UNITS            4
#define IOC_FM_PCD_MAX_NUM_OF_KEYS              256
#define IOC_FM_PCD_MAX_SIZE_OF_KEY               56
#define IOC_FM_PCD_MAX_NUM_OF_CC_ENTRIES_IN_GRP  16
/* @} */

/**************************************************************************//**
 @Collection    A set of definitions to allow protocol
                special option description.
*//***************************************************************************/
typedef uint32_t            ioc_protocol_opt_t;      /**< A general type to define a protocol option. */

typedef ioc_protocol_opt_t  ioc_eth_protocol_opt_t;  /**< Ethernet protocol options. */
#define IOC_ETH_BROADCAST               0x80000000   /**< Ethernet Broadcast. */
#define IOC_ETH_MULTICAST               0x40000000   /**< Ethernet Multicast. */

typedef ioc_protocol_opt_t  ioc_vlan_protocol_opt_t; /**< Vlan protocol options. */
#define IOC_VLAN_STACKED                0x20000000   /**< Vlan Stacked. */

typedef ioc_protocol_opt_t  ioc_mpls_protocol_opt_t; /**< MPLS protocol options. */
#define IOC_MPLS_STACKED                0x10000000   /**< MPLS Stacked. */

typedef ioc_protocol_opt_t  ioc_ipv4_protocol_opt_t; /**< IPv4 protocol options. */
#define IOC_IPV4_BROADCAST_1            0x08000000   /**< IPv4 Broadcast. */
#define IOC_IPV4_MULTICAST_1            0x04000000   /**< IPv4 Multicast. */
#define IOC_IPV4_UNICAST_2              0x02000000   /**< Tunneled IPv4 - Unicast. */
#define IOC_IPV4_MULTICAST_BROADCAST_2  0x01000000   /**< Tunneled IPv4 - Broadcast/Multicast. */

typedef ioc_protocol_opt_t  ioc_ipv6_protocol_opt_t; /**< IPv6 protocol options. */
#define IOC_IPV6_MULTICAST_1            0x00800000   /**< IPv6 Multicast. */
#define IOC_IPV6_UNICAST_2              0x00400000   /**< Tunneled IPv6 - Unicast. */
#define IOC_IPV6_MULTICAST_2            0x00200000   /**< Tunneled IPv6 - Multicast. */
/* @} */

/**************************************************************************//**
 @Description   All PCD engines
                (must match enum e_FmPcdEngine defined in fm_pcd_ext.h)
*//***************************************************************************/
typedef enum ioc_fm_pcd_engine {
    e_IOC_FM_PCD_INVALID = 0,   /**< Invalid PCD engine indicated*/
    e_IOC_FM_PCD_DONE,          /**< No PCD Engine indicated */
    e_IOC_FM_PCD_KG,            /**< Keygen indicated */
    e_IOC_FM_PCD_CC,            /**< Coarse classification indicated */
    e_IOC_FM_PCD_PLCR,          /**< Policer indicated */
    e_IOC_FM_PCD_PRS            /**< Parser indicated */
} ioc_fm_pcd_engine;

/**************************************************************************//**
 @Description   An enum for selecting extraction by header types
                (must match enum e_FmPcdExtractByHdrType defined in fm_pcd_ext.h)
*//***************************************************************************/
typedef enum ioc_fm_pcd_extract_by_hdr_type {
    e_IOC_FM_PCD_EXTRACT_FROM_HDR,      /**< Extract bytes from header */
    e_IOC_FM_PCD_EXTRACT_FROM_FIELD,    /**< Extract bytes from header field */
    e_IOC_FM_PCD_EXTRACT_FULL_FIELD     /**< Extract a full field */
} ioc_fm_pcd_extract_by_hdr_type;

/**************************************************************************//**
 @Description   An enum for selecting extraction source
                (when it is not the header)
                (must match enum e_FmPcdExtractFrom defined in fm_pcd_ext.h)
*//***************************************************************************/
typedef enum ioc_fm_pcd_extract_from {
    e_IOC_FM_PCD_EXTRACT_FROM_FRAME_START,          /**< KG & CC: Extract from beginning of frame */
    e_IOC_FM_PCD_EXTRACT_FROM_DFLT_VALUE,           /**< KG only: Extract from a default value */
    e_IOC_FM_PCD_EXTRACT_FROM_CURR_END_OF_PARSE,    /**< KG only: Extract from the point where parsing had finished */
    e_IOC_FM_PCD_EXTRACT_FROM_KEY,                  /**< CC only: Field where saved KEY */
    e_IOC_FM_PCD_EXTRACT_FROM_HASH,                 /**< CC only: Field where saved HASH */
    e_IOC_FM_PCD_EXTRACT_FROM_PARSE_RESULT,         /**< KG & CC: Extract from the parser result */
    e_IOC_FM_PCD_EXTRACT_FROM_ENQ_FQID,             /**< KG & CC: Extract from enqueue FQID */
    e_IOC_FM_PCD_EXTRACT_FROM_FLOW_ID               /**< CC only: Field where saved Dequeue FQID */
} ioc_fm_pcd_extract_from;

/**************************************************************************//**
 @Description   An enum for selecting extraction type
*//***************************************************************************/
typedef enum ioc_fm_pcd_extract_type {
    e_IOC_FM_PCD_EXTRACT_BY_HDR,                /**< Extract according to header */
    e_IOC_FM_PCD_EXTRACT_NON_HDR,               /**< Extract from data that is not the header */
    e_IOC_FM_PCD_KG_EXTRACT_PORT_PRIVATE_INFO   /**< Extract private info as specified by user */
} ioc_fm_pcd_extract_type;

/**************************************************************************//**
 @Description   An enum for selecting a default
*//***************************************************************************/
typedef enum ioc_fm_pcd_kg_extract_dflt_select {
    e_IOC_FM_PCD_KG_DFLT_GBL_0,          /**< Default selection is KG register 0 */
    e_IOC_FM_PCD_KG_DFLT_GBL_1,          /**< Default selection is KG register 1 */
    e_IOC_FM_PCD_KG_DFLT_PRIVATE_0,      /**< Default selection is a per scheme register 0 */
    e_IOC_FM_PCD_KG_DFLT_PRIVATE_1,      /**< Default selection is a per scheme register 1 */
    e_IOC_FM_PCD_KG_DFLT_ILLEGAL         /**< Illegal selection */
} ioc_fm_pcd_kg_extract_dflt_select;

/**************************************************************************//**
 @Description   An enum defining all default groups -
                each group shares a default value, one of 4 user
                initialized values.
*//***************************************************************************/
typedef enum ioc_fm_pcd_kg_known_fields_dflt_types {
    e_IOC_FM_PCD_KG_MAC_ADDR,               /**< MAC Address */
    e_IOC_FM_PCD_KG_TCI,                    /**< TCI field */
    e_IOC_FM_PCD_KG_ENET_TYPE,              /**< ENET Type */
    e_IOC_FM_PCD_KG_PPP_SESSION_ID,         /**< PPP Session id */
    e_IOC_FM_PCD_KG_PPP_PROTOCOL_ID,        /**< PPP Protocol id */
    e_IOC_FM_PCD_KG_MPLS_LABEL,             /**< MPLS label */
    e_IOC_FM_PCD_KG_IP_ADDR,                /**< IP addr */
    e_IOC_FM_PCD_KG_PROTOCOL_TYPE,          /**< Protocol type */
    e_IOC_FM_PCD_KG_IP_TOS_TC,              /**< TOS or TC */
    e_IOC_FM_PCD_KG_IPV6_FLOW_LABEL,        /**< IPV6 flow label */
    e_IOC_FM_PCD_KG_IPSEC_SPI,              /**< IPSEC SPI */
    e_IOC_FM_PCD_KG_L4_PORT,                /**< L4 Port */
    e_IOC_FM_PCD_KG_TCP_FLAG,               /**< TCP Flag */
    e_IOC_FM_PCD_KG_GENERIC_FROM_DATA,      /**< grouping implemented by sw,
                                                 any data extraction that is not the full
                                                 field described above  */
    e_IOC_FM_PCD_KG_GENERIC_FROM_DATA_NO_V, /**< grouping implemented by sw,
                                                 any data extraction without validation */
    e_IOC_FM_PCD_KG_GENERIC_NOT_FROM_DATA   /**< grouping implemented by sw,
                                                 extraction from parser result or
                                                 direct use of default value  */
} ioc_fm_pcd_kg_known_fields_dflt_types;

/**************************************************************************//**
 @Description   enum for defining header index when headers may repeat
*//***************************************************************************/
typedef enum ioc_fm_pcd_hdr_index {
    e_IOC_FM_PCD_HDR_INDEX_NONE     =   0,      /**< used when multiple headers not used, also
                                                     to specify regular IP (not tunneled). */
    e_IOC_FM_PCD_HDR_INDEX_1,                   /**< may be used for VLAN, MPLS, tunneled IP */
    e_IOC_FM_PCD_HDR_INDEX_2,                   /**< may be used for MPLS, tunneled IP */
    e_IOC_FM_PCD_HDR_INDEX_3,                   /**< may be used for MPLS */
    e_IOC_FM_PCD_HDR_INDEX_LAST     =   0xFF    /**< may be used for VLAN, MPLS */
} ioc_fm_pcd_hdr_index;

/**************************************************************************//**
 @Description   A structure for selcting the policer profile functional type
*//***************************************************************************/
typedef enum ioc_fm_pcd_profile_type_selection {
    e_IOC_FM_PCD_PLCR_PORT_PRIVATE,             /**< Port dedicated profile */
    e_IOC_FM_PCD_PLCR_SHARED                    /**< Shared profile (shared within partition) */
} ioc_fm_pcd_profile_type_selection;

/**************************************************************************//**
 @Description   A structure for selcting the policer profile algorithem
*//***************************************************************************/
typedef enum ioc_fm_pcd_plcr_algorithm_selection {
    e_IOC_FM_PCD_PLCR_PASS_THROUGH, /**< Policer pass through */
    e_IOC_FM_PCD_PLCR_RFC_2698,     /**< Policer algorythm RFC 2698 */
    e_IOC_FM_PCD_PLCR_RFC_4115      /**< Policer algorythm RFC 4115 */
} ioc_fm_pcd_plcr_algorithm_selection;

/**************************************************************************//**
 @Description   A structure for selcting the policer profile color mode
*//***************************************************************************/
typedef enum ioc_fm_pcd_plcr_color_mode {
    e_IOC_FM_PCD_PLCR_COLOR_BLIND,  /**< Color blind */
    e_IOC_FM_PCD_PLCR_COLOR_AWARE   /**< Color aware */
} ioc_fm_pcd_plcr_color_mode;

/**************************************************************************//**
 @Description   A structure for selcting the policer profile color functional mode
*//***************************************************************************/
typedef enum ioc_fm_pcd_plcr_color {
    e_IOC_FM_PCD_PLCR_GREEN,    /**< Green */
    e_IOC_FM_PCD_PLCR_YELLOW,   /**< Yellow */
    e_IOC_FM_PCD_PLCR_RED,      /**< Red */
    e_IOC_FM_PCD_PLCR_OVERRIDE  /**< Color override */
} ioc_fm_pcd_plcr_color;

/**************************************************************************//**
 @Description   A structure for selcting the policer profile packet frame length selector
*//***************************************************************************/
typedef enum ioc_fm_pcd_plcr_frame_length_select {
  e_IOC_FM_PCD_PLCR_L2_FRM_LEN,     /**< L2 frame length */
  e_IOC_FM_PCD_PLCR_L3_FRM_LEN,     /**< L3 frame length */
  e_IOC_FM_PCD_PLCR_L4_FRM_LEN,     /**< L4 frame length */
  e_IOC_FM_PCD_PLCR_FULL_FRM_LEN    /**< Full frame length */
} ioc_fm_pcd_plcr_frame_length_select;

/**************************************************************************//**
 @Description   An enum for selecting rollback frame
*//***************************************************************************/
typedef enum ioc_fm_pcd_plcr_roll_back_frame_select {
  e_IOC_FM_PCD_PLCR_ROLLBACK_L2_FRM_LEN,    /**< Rollback L2 frame length */
  e_IOC_FM_PCD_PLCR_ROLLBACK_FULL_FRM_LEN   /**< Rollback Full frame length */
} ioc_fm_pcd_plcr_roll_back_frame_select;

/**************************************************************************//**
 @Description   A structure for selcting the policer profile packet or byte mode
*//***************************************************************************/
typedef enum ioc_fm_pcd_plcr_rate_mode {
    e_IOC_FM_PCD_PLCR_BYTE_MODE,    /**< Byte mode */
    e_IOC_FM_PCD_PLCR_PACKET_MODE   /**< Packet mode */
} ioc_fm_pcd_plcr_rate_mode;

/**************************************************************************//**
 @Description   An enum for defining action of frame
*//***************************************************************************/
typedef enum ioc_fm_pcd_done_action {
    e_IOC_FM_PCD_ENQ_FRAME = 0,    /**< Enqueue frame */
    e_IOC_FM_PCD_DROP_FRAME    /**< Drop frame */
} ioc_fm_pcd_done_action;

/**************************************************************************//**
 @Description   A structure for selcting the policer counter
*//***************************************************************************/
typedef enum ioc_fm_pcd_plcr_profile_counters {
    e_IOC_FM_PCD_PLCR_PROFILE_GREEN_PACKET_TOTAL_COUNTER,               /**< Green packets counter */
    e_IOC_FM_PCD_PLCR_PROFILE_YELLOW_PACKET_TOTAL_COUNTER,              /**< Yellow packets counter */
    e_IOC_FM_PCD_PLCR_PROFILE_RED_PACKET_TOTAL_COUNTER,                 /**< Red packets counter */
    e_IOC_FM_PCD_PLCR_PROFILE_RECOLOURED_YELLOW_PACKET_TOTAL_COUNTER,   /**< Recolored yellow packets counter */
    e_IOC_FM_PCD_PLCR_PROFILE_RECOLOURED_RED_PACKET_TOTAL_COUNTER       /**< Recolored red packets counter */
} ioc_fm_pcd_plcr_profile_counters;

/**************************************************************************//**
 @Description   A structure for selecting action
*//***************************************************************************/
typedef enum ioc_fm_pcd_action {
    e_IOC_FM_PCD_ACTION_NONE,                           /**< NONE  */
    e_IOC_FM_PCD_ACTION_EXACT_MATCH,                    /**< Exact match on the selected extraction*/
    e_IOC_FM_PCD_ACTION_INDEXED_LOOKUP                  /**< Indexed lookup on the selected extraction*/
} ioc_fm_pcd_action;

/**************************************************************************//**
 @Description   A type used for returning the order of the key extraction.
                each value in this array represents the index of the extraction
                command as defined by the user in the initialization extraction array.
                The valid size of this array is the user define number of extractions
                required (also marked by the second '0' in this array).
*//***************************************************************************/
typedef    uint8_t    ioc_fm_pcd_kg_key_order_t [IOC_FM_PCD_KG_MAX_NUM_OF_EXTRACTS_PER_KEY];

/**************************************************************************//**
 @Description   A Union of protocol dependent special options
*//***************************************************************************/
typedef union ioc_fm_pcd_hdr_protocol_opt_u {
    ioc_eth_protocol_opt_t    eth_opt;     /**< Ethernet options */
    ioc_vlan_protocol_opt_t   vlan_opt;    /**< Vlan options */
    ioc_mpls_protocol_opt_t   mpls_opt;    /**< MPLS options */
    ioc_ipv4_protocol_opt_t   ipv4_opt;    /**< IPv4 options */
    ioc_ipv6_protocol_opt_t   ipv6_opt;    /**< IPv6 options */
} ioc_fm_pcd_hdr_protocol_opt_u;

/**************************************************************************//**
 @Description   A union holding all known protocol fields
*//***************************************************************************/
typedef union ioc_fm_pcd_fields_u {
    ioc_header_field_eth_t        eth;        /**< eth      */
    ioc_header_field_vlan_t       vlan;       /**< vlan     */
    ioc_header_field_llc_snap_t   llc_snap;   /**< llcSnap  */
    ioc_header_field_pppoe_t      pppoe;      /**< pppoe    */
    ioc_header_field_mpls_t       mpls;       /**< mpls     */
    ioc_header_field_ipv4_t       ipv4;       /**< ipv4     */
    ioc_header_field_ipv6_t       ipv6;       /**< ipv6     */
    ioc_header_field_udp_t        udp;        /**< udp      */
    ioc_header_field_tcp_t        tcp;        /**< tcp      */
    ioc_header_field_sctp_t       sctp;       /**< sctp     */
    ioc_header_field_dccp_t       dccp;       /**< dccp     */
    ioc_header_field_gre_t        gre;        /**< gre      */
    ioc_header_field_minencap_t   minencap;   /**< minencap */
    ioc_header_field_ipsec_ah_t   ipsec_ah;   /**< ipsecAh  */
    ioc_header_field_ipsec_esp_t  ipsec_esp;  /**< ipsecEsp */
} ioc_fm_pcd_fields_u;

/**************************************************************************//**
 @Description   structure for defining header extraction for key generation
*//***************************************************************************/
typedef struct ioc_fm_pcd_from_hdr_t {
    uint8_t             size;           /**< Size in byte */
    uint8_t             offset;         /**< Byte offset */
} ioc_fm_pcd_from_hdr_t;

/**************************************************************************//**
 @Description   structure for defining field extraction for key generation
*//***************************************************************************/
typedef struct ioc_fm_pcd_from_field_t {
    ioc_fm_pcd_fields_u field;          /**< Field selection */
    uint8_t             size;           /**< Size in byte */
    uint8_t             offset;         /**< Byte offset */
} ioc_fm_pcd_from_field_t;

/**************************************************************************//**
 @Description   A structure of parameters used to define a single network
                environment unit.
                A unit should be defined if it will later be used by one or
                more PCD engines to distinguich between flows.
*//***************************************************************************/
typedef struct ioc_fm_pcd_distinction_unit_t {
    struct {
        ioc_net_header_type             hdr;            /**< One of the headers supported by the FM */
        ioc_fm_pcd_hdr_protocol_opt_u   opt;            /**< only one option !! */
    } hdrs[IOC_FM_PCD_MAX_NUM_OF_INTERCHANGEABLE_HDRS];
} ioc_fm_pcd_distinction_unit_t;

/**************************************************************************//**
 @Description   A structure of parameters used to define the different
                units supported by a specific PCD Network Environment
                Characteristics module. Each unit represent
                a protocol or a group of protocols that may be used later
                by the different PCD engines to distinguish between flows.
                (must match struct t_FmPcdNetEnvParams defined in fm_pcd_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_pcd_net_env_params_t {
    uint8_t                         num_of_distinction_units;   /**< Number of different units to be identified */
    ioc_fm_pcd_distinction_unit_t   units[IOC_FM_PCD_MAX_NUM_OF_DISTINCTION_UNITS];
                                                                /**< An array of numOfDistinctionUnits of the
                                                                     different units to be identified */
    void                            *id;                        /**< output parameter; Returns the net-env Id to be used */
} ioc_fm_pcd_net_env_params_t;


/**************************************************************************//**
 @Description   structure for defining a single extraction action
                when creating a key
*//***************************************************************************/
typedef struct ioc_fm_pcd_extract_entry_t {
    ioc_fm_pcd_extract_type                 type;           /**< Extraction type select */
    union {
        struct {                            /**< used when type = e_IOC_FM_PCD_KG_EXTRACT_BY_HDR */
            ioc_net_header_type             hdr;            /**< Header selection */
            bool                            ignore_protocol_validation; /**< Ignore protocol validation */
            ioc_fm_pcd_hdr_index            hdr_index;       /**< Relevant only for MPLS, VLAN and tunneled
                                                                  IP. Otherwise should be cleared.*/
            ioc_fm_pcd_extract_by_hdr_type  type;            /**< Header extraction type select */
            union {
                ioc_fm_pcd_from_hdr_t       from_hdr;        /**< Extract bytes from header parameters */
                ioc_fm_pcd_from_field_t     from_field;      /**< Extract bytes from field parameters*/
                ioc_fm_pcd_fields_u         full_field;      /**< Extract full filed parameters*/
            } extract_by_hdr_type;
        } extract_by_hdr;
        struct{                       /**< used when type = e_IOC_FM_PCD_KG_EXTRACT_NON_HDR */
            ioc_fm_pcd_extract_from         src;            /**< Non-header extraction source */
            ioc_fm_pcd_action               action;         /**< Relevant for CC Only */
            uint16_t                        ic_indx_mask;   /**< Relevant only for CC where
                                                                 action=e_FM_PCD_ACTION_INDEXED_LOOKUP */
            uint8_t                         offset;         /**< Byte offset */
            uint8_t                         size;           /**< Size in byte */
        } extract_non_hdr;
    } extract_params;
} ioc_fm_pcd_extract_entry_t;

/**************************************************************************//**
 @Description   A structure for defining masks for each extracted
                field in the key.
*//***************************************************************************/
typedef struct ioc_fm_pcd_kg_extract_mask_t {
    uint8_t                extract_array_index;         /**< Index in the extraction array, as initialized by user */
    uint8_t                offset;                      /**< Byte offset */
    uint8_t                mask;                        /**< A byte mask (selected bits will be ignored) */
} ioc_fm_pcd_kg_extract_mask_t;

/**************************************************************************//**
 @Description   A structure for defining default selection per groups
                of fields
*//***************************************************************************/
typedef struct ioc_fm_pcd_kg_extract_dflt_t {
    ioc_fm_pcd_kg_known_fields_dflt_types    type;          /**< Default type select*/
    ioc_fm_pcd_kg_extract_dflt_select        dflt_select;   /**< Default register select */
} ioc_fm_pcd_kg_extract_dflt_t;

/**************************************************************************//**
 @Description   A structure for defining all parameters needed for
                generation a key and using a hash function
*//***************************************************************************/
typedef struct ioc_fm_pcd_kg_key_extract_and_hash_params_t {
    uint32_t                            private_dflt0;               /**< Scheme default register 0 */
    uint32_t                            private_dflt1;               /**< Scheme default register 1 */
    uint8_t                             num_of_used_extracts;           /**< defines the valid size of the following array */
    ioc_fm_pcd_extract_entry_t          extract_array [IOC_FM_PCD_KG_MAX_NUM_OF_EXTRACTS_PER_KEY];
    uint8_t                             num_of_used_dflt;           /**< defines the valid size of the following array */
    ioc_fm_pcd_kg_extract_dflt_t        dflts[IOC_FM_PCD_KG_NUM_OF_DEFAULT_GROUPS];
    uint8_t                             num_of_used_masks;              /**< defines the valid size of the following array */
    ioc_fm_pcd_kg_extract_mask_t        masks[IOC_FM_PCD_KG_NUM_OF_EXTRACT_MASKS];
    uint8_t                             hash_shift;                     /**< Select the 24 bits out of the 64 hash result */
    uint32_t                            hash_distribution_num_of_fqids; /**< must be > 1 and a power of 2. Represents the range
                                                                             of queues for the key and hash functionality */
    uint8_t                             hash_distribution_fqids_shift;  /**< selects the FQID bits that will be effected by the hash */
    bool                                symmetric_hash;                 /**< TRUE to generate the same hash for frames with swapped source and
                                                                  destination fields on all layers; If TRUE, driver will check that for
                                                                  all layers, if SRC extraction is selected, DST extraction must also be
                                                                  selected, and vice versa. */
} ioc_fm_pcd_kg_key_extract_and_hash_params_t;

/**************************************************************************//**
 @Description   A structure of parameters for defining a single
                Qid mask (extracted OR).
*//***************************************************************************/
typedef struct ioc_fm_pcd_kg_extracted_or_params_t {
    ioc_fm_pcd_extract_type                 type;               /**< Extraction type select */
    union{
        struct{                                                 /**< used when type = e_IOC_FM_PCD_KG_EXTRACT_BY_HDR */
            ioc_net_header_type             hdr;
            ioc_fm_pcd_hdr_index            hdr_index;          /**< Relevant only for MPLS, VLAN and tunneled
                                                                     IP. Otherwise should be cleared.*/
            bool                            ignore_protocol_validation;
        } extract_by_hdr;
        ioc_fm_pcd_extract_from             src;                /**< used when type = e_IOC_FM_PCD_KG_EXTRACT_NON_HDR */
    } extract_params;
    uint8_t                                 extraction_offset;  /**< Offset for extraction */
    ioc_fm_pcd_kg_extract_dflt_select       dflt_value;         /**< Select register from which extraction is taken if
                                                                     field not found */
    uint8_t                                 mask;               /**< Mask LSB byte of extraction (specified bits are ignored) */
    uint8_t                                 bit_offset_in_fqid; /**< out of 24 bits Qid  (max offset = 16) */
} ioc_fm_pcd_kg_extracted_or_params_t;

/**************************************************************************//**
 @Description   A structure for configuring scheme counter
*//***************************************************************************/
typedef struct ioc_fm_pcd_kg_scheme_counter_t {
    bool        update;     /**< FALSE to keep the current counter state
                                 and continue from that point, TRUE to update/reset
                                 the counter when the scheme is written. */
    uint32_t    value;      /**< If update=TRUE, this value will be written into the
                                 counter. clear this field to reset the counter. */
} ioc_fm_pcd_kg_scheme_counter_t;

/**************************************************************************//**
 @Description   A structure for defining policer profile
                parameters as required by keygen (when policer
                is the next engine after this scheme).
*//***************************************************************************/
typedef struct ioc_fm_pcd_kg_plcr_profile_t {
    bool                shared_profile;                 /**< TRUE if this profile is shared between ports
                                                             (i.e. managed by master partition) May not be TRUE
                                                             if profile is after Coarse Classification*/
    bool                direct;                         /**< if TRUE, directRelativeProfileId only selects the profile
                                                             id, if FALSE fqidOffsetRelativeProfileIdBase is used
                                                             together with fqidOffsetShift and numOfProfiles
                                                             parameters, to define a range of profiles from
                                                             which the keygen result will determine the
                                                             destination policer profile.  */
    union{
        uint16_t        direct_relative_profile_id;     /**< Used if 'direct' is TRUE, to select policer profile.
                                                             This parameter should
                                                             indicate the policer profile offset within the port's
                                                             policer profiles or SHARED window. */
        struct {
            uint8_t     fqid_offset_shift;              /**< shift of KG results without the qid base */
            uint8_t     fqid_offset_relative_profile_id_base;/**< OR of KG results without the qid base
                                                             This parameter should
                                                             indicate the policer profile offset within the port's
                                                             policer profiles windowor SHARED window depends on sharedProfile */
            uint8_t     num_of_profiles;                /**< Range of profiles starting at base */
        } indirect_profile_id;
    } profile_select;
} ioc_fm_pcd_kg_plcr_profile_t;

/**************************************************************************//**
 @Description   A structure for CC parameters if CC is the next engine after KG
*//***************************************************************************/
typedef struct ioc_fm_pcd_kg_cc_t {
    void                            *tree_id;           /**< CC Tree id */
    uint8_t                         grp_id;             /**< CC group id within the CC tree */
    bool                            plcr_next;          /**< TRUE if after CC, in case of data frame,
                                                             policing is required. */
    bool                            bypass_plcr_profile_generation;
                                                        /**< TRUE to bypass keygen policer profile
                                                             generation (profile selected is the one selected at
                                                             port initialization). */
    ioc_fm_pcd_kg_plcr_profile_t    plcr_profile;       /**< only if plcrNext=TRUE */
} ioc_fm_pcd_kg_cc_t;

/**************************************************************************//**
 @Description   A structure for initializing a keygen single scheme
                (must match struct t_FmPcdKgSchemeParams defined in fm_pcd_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_pcd_kg_scheme_params_t {
    bool                                modify;         /**< enables changing an existing scheme */
    union
    {
        uint8_t                         relative_scheme_id;
                                                         /**< if modify=FALSE:Partition relative scheme id */
        void                            *scheme_id;      /**< if modify=TRUE: a handle of the existing scheme */
    } scm_id;
    bool                                always_direct;   /**< This scheme is reached only directly, i.e.
                                                              no need for match vector. Keygen will ignore
                                                              it when matching   */
    struct                                               /**< HL Relevant only if alwaysDirect = FALSE */
    {
        void                            *net_env_id;     /**< Network environment id  */
        uint8_t                         num_of_distinction_units;
                                                         /**< Number of netenv units listed in unit_ids array */
        uint8_t                         unit_ids[IOC_FM_PCD_MAX_NUM_OF_DISTINCTION_UNITS];
                                                         /**< Indexes as passed to SetNetEnvCharacteristics array*/
    } netEnvParams;
    bool                                use_hash;        /**< use the KG Hash functionality */
    ioc_fm_pcd_kg_key_extract_and_hash_params_t key_extract_and_hash_params;
                                                         /**< used only if useHash = TRUE */
    bool                                bypass_fqid_generation;
                                                         /**< Normally - FALSE, TRUE to avoid FQID update in the IC;
                                                              In such a case FQID after KG will be the default FQID
                                                              defined for the relevant port, or the FQID defined by CC
                                                              in cases where CC was the previous engine. */
    uint32_t                            base_fqid;       /**< Base FQID */
    uint8_t                             numOfUsedExtractedOrs;
                                                         /**< Number of Fqid masks listed in extractedOrs array*/
    ioc_fm_pcd_kg_extracted_or_params_t extracted_ors[IOC_FM_PCD_KG_NUM_OF_GENERIC_REGS];
                                                         /**< IOC_FM_PCD_KG_NUM_OF_GENERIC_REGS
                                                              registers are shared between qidMasks
                                                              functionality and some of the extraction
                                                              actions; Normally only some will be used
                                                              for qidMask. Driver will return error if
                                                              resource is full at initialization time. */
    ioc_fm_pcd_engine                   next_engine;     /**< may be BMI, PLCR or CC */
    union{                                               /**< depends on nextEngine */
        ioc_fm_pcd_done_action          done_action;     /**< Used when next engine is BMI (done) */
        ioc_fm_pcd_kg_plcr_profile_t    plcr_profile;    /**< Used when next engine is PLCR */
        ioc_fm_pcd_kg_cc_t              cc;              /**< Used when next engine is CC */
    } kg_next_engine_params;
    ioc_fm_pcd_kg_scheme_counter_t      scheme_counter;  /**< A structure of parameters for updating
                                                              the scheme counter */
    void                                *id;             /**< Returns the scheme Id to be used */
} ioc_fm_pcd_kg_scheme_params_t;

/**************************************************************************//**
 @Description   A structure for defining CC params when CC is the
                next engine after a CC node
*//***************************************************************************/
typedef struct ioc_fm_pcd_cc_next_cc_params_t {
    void        *cc_node_id;                             /**< Id of the next CC node */
} ioc_fm_pcd_cc_next_cc_params_t;

/**************************************************************************//**
 @Description   A structure for defining PLCR params when PLCR is the
                next engine after a CC node
*//***************************************************************************/
typedef struct ioc_fm_pcd_cc_next_plcr_params_t {
    bool        override_params;            /**< TRUE if CC override previously decided parameters*/
    bool        shared_profile;             /**< Relevant only if overrideParams=TRUE:
                                                TRUE if this profile is shared between ports */
    uint16_t    new_relative_profileId;     /**< Relevant only if overrideParams=TRUE:
                                                (otherwise profile id
                                                is taken from keygen);
                                                This parameter should
                                                indicate the policer profile offset within the port's
                                                policer profiles or from SHARED window.*/
    uint32_t    new_fqid;                   /**< Relevant only if overrideParams=TRUE:
                                                FQID for enquing the frame;
                                                In earlier chips  if policer next engine is KEYGEN,
                                                this parameter can be 0, because the KEYGEN always decides
                                                the enqueue FQID.*/
    bool        statistics_en;               /**< In the case of TRUE Statistic counter is
                                                incremented for each received frame passed through
                                                this Coarse Classification entry.*/
} ioc_fm_pcd_cc_next_plcr_params_t;

/**************************************************************************//**
 @Description   A structure for defining enqueue params when BMI is the
                next engine after a CC node
*//***************************************************************************/
typedef struct ioc_fm_pcd_cc_next_enqueue_params_t {
    ioc_fm_pcd_done_action  action;         /**< Action - when next engine is BMI (done) */
    bool                    override_fqid;  /**< TRUE if CC override previously decided Fqid(by Keygen),
                                                 relevant if action = e_FM_PCD_ENQ_FRAME*/
    uint32_t                new_fqid;       /**< Valid if overrideFqid=TRUE, FQID for enquing the frame
                                                 (otherwise FQID is taken from keygen),
                                                 relevant if action = e_FM_PCD_ENQ_FRAME*/
    bool                    statistics_en;   /**< In the case of TRUE Statistic counter is
                                                 incremented for each received frame passed through
                                                 this Coarse Classification entry.*/
} ioc_fm_pcd_cc_next_enqueue_params_t;

/**************************************************************************//**
 @Description   A structure for defining KG params when KG is the
                next engine after a CC node
*//***************************************************************************/
typedef struct ioc_fm_pcd_cc_next_kg_params_t {
    bool       override_fqid;           /**< TRUE if CC override previously decided Fqid (by keygen),
                                          Note - this parameters unrelevant for earlier chips*/
    uint32_t   new_fqid;                 /**< Valid if overrideFqid=TRUE, FQID for enquing the frame
                                         (otherwise FQID is taken from keygen),
                                          Note - this parameters unrelevant for earlier chips*/
    void       *p_direct_scheme;        /**< Direct scheme handle to go to. */
    bool       statistics_en;           /**< In the case of TRUE Statistic counter is
                                             incremented for each received frame passed through
                                             this Coarse Classification entry.*/
} ioc_fm_pcd_cc_next_kg_params_t;

/**************************************************************************//**
 @Description   A structure for defining next engine params after a CC node.
                (must match struct t_FmPcdCcNextEngineParams defined in fm_pcd_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_pcd_cc_next_engine_params_t {
    ioc_fm_pcd_engine                           next_engine;    /**< user has to init parameters according
                                                                     to nextEngine definition */
    union {
            ioc_fm_pcd_cc_next_cc_params_t      cc_params;      /**< Parameters in case next engine is CC */
            ioc_fm_pcd_cc_next_plcr_params_t    plcr_params;    /**< Parameters in case next engine is PLCR */
            ioc_fm_pcd_cc_next_enqueue_params_t enqueue_params; /**< Parameters in case next engine is BMI */
            ioc_fm_pcd_cc_next_kg_params_t      kg_params;      /**< Parameters in case next engine is KG */
    } params;
#if defined(FM_CAPWAP_SUPPORT)
    void                                        *p_manip;       /**< Handler to headerManip.
                                                                     Relevant if next engine of the type result
                                                                     (e_FM_PCD_PLCR, e_FM_PCD_KG, e_FM_PCD_DONE) */
#endif /* defined(FM_CAPWAP_SUPPORT) */
} ioc_fm_pcd_cc_next_engine_params_t;

/**************************************************************************//**
 @Description   A structure for defining a single CC Key parameters
*//***************************************************************************/
typedef struct ioc_fm_pcd_cc_key_params_t {
    uint8_t                 *p_key; /**< pointer to the key of the size defined in keySize */
    uint8_t                 *p_mask;/**< pointer to the Mask per key  of the size defined
                                         in keySize. p_Key and p_Mask (if defined) has to be
                                         of the same size defined in the keySize */
    ioc_fm_pcd_cc_next_engine_params_t  cc_next_engine_params;
                                    /**< parameters for the next for the defined Key in
                                         the p_Key */

} ioc_fm_pcd_cc_key_params_t;

/**************************************************************************//**
 @Description   A structure for defining CC Keys parameters
*//***************************************************************************/
typedef struct ioc_keys_params_t {
    uint8_t                             num_of_keys;    /**< num Of relevant Keys  */
    uint8_t                             key_size;       /**< size of the key - in the case of the extraction of
                                                             the type FULL_FIELD keySize has to be as standard size of the relevant
                                                             key. In the another type of extraction keySize has to be as size of extraction. */
    ioc_fm_pcd_cc_key_params_t          key_params[IOC_FM_PCD_MAX_NUM_OF_KEYS];
                                                        /**< it's array with numOfKeys entries each entry in
                                                             the array of the type ioc_fm_pcd_cc_key_params_t */
    ioc_fm_pcd_cc_next_engine_params_t  cc_next_engine_params_for_miss;
                                                        /**< parameters for the next step of
                                                             unfound (or undefined) key */
} ioc_keys_params_t;

/**************************************************************************//**
 @Description   A structure for defining the CC node params
*//***************************************************************************/
typedef struct ioc_fm_pcd_cc_node_params_t {
    ioc_fm_pcd_extract_entry_t          extract_cc_params;  /**< params which defines extraction parameters */
    ioc_keys_params_t                   keys_params;        /**< params which defines Keys parameters of the
                                                                 extraction defined in  extractParams */
    void                                *id;                /**< output parameter; Returns the CC node Id to be used */
} ioc_fm_pcd_cc_node_params_t;

/**************************************************************************//**
 @Description   A structure for defining each CC tree group in term of
                NetEnv units and the action to be taken in each case.
                the unit_ids list must be in order from lower to higher indexes.

                ioc_fm_pcd_cc_next_engine_params_t is a list of 2^num_of_distinction_units
                structures where each defines the next action to be taken for
                each units combination. for example:
                num_of_distinction_units = 2
                unit_ids = {1,3}
                next_engine_per_entries_in_grp[0] = ioc_fm_pcd_cc_next_engine_params_t for the case that
                                                    unit 1 - not found; unit 3 - not found;
                next_engine_per_entries_in_grp[1] = ioc_fm_pcd_cc_next_engine_params_t for the case that
                                                    unit 1 - not found; unit 3 - found;
                next_engine_per_entries_in_grp[2] = ioc_fm_pcd_cc_next_engine_params_t for the case that
                                                    unit 1 - found; unit 3 - not found;
                next_engine_per_entries_in_grp[3] = ioc_fm_pcd_cc_next_engine_params_t for the case that
                                                    unit 1 - found; unit 3 - found;
*//***************************************************************************/
typedef struct ioc_fm_pcd_cc_grp_params_t {
    uint8_t                             num_of_distinction_units;   /**< up to 4 */
    uint8_t                             unit_ids [IOC_FM_PCD_MAX_NUM_OF_CC_UNITS];
                                                                    /**< Indexes of the units as defined in
                                                                         FM_PCD_SetNetEnvCharacteristics */
    ioc_fm_pcd_cc_next_engine_params_t  next_engine_per_entries_in_grp[IOC_FM_PCD_MAX_NUM_OF_CC_ENTRIES_IN_GRP];
                                                                    /**< Max size is 16 - if only one group used */
} ioc_fm_pcd_cc_grp_params_t;

/**************************************************************************//**
 @Description   A structure for defining the CC tree groups
                (must match struct t_FmPcdCcTreeParams defined in fm_pcd_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_pcd_cc_tree_params_t {
        void                            *net_env_id;    /**< Id of the Network environment as returned
                                                             by FM_PCD_SetNetEnvCharacteristics */
        uint8_t                         num_of_groups;  /**< Number of CC groups within the CC tree */
        ioc_fm_pcd_cc_grp_params_t      fm_pcd_cc_group_params [IOC_FM_PCD_MAX_NUM_OF_CC_GROUPS];
                                                        /**< Parameters for each group. */
        void                            *id;            /**< output parameter; Returns the tree Id to be used */
} ioc_fm_pcd_cc_tree_params_t;

/**************************************************************************//**
 @Description   A structure for defining parameters for byte rate
*//***************************************************************************/
typedef struct ioc_fm_pcd_plcr_byte_rate_mode_param_t {
    ioc_fm_pcd_plcr_frame_length_select     frame_length_selection;     /**< Frame length selection */
    ioc_fm_pcd_plcr_roll_back_frame_select  roll_back_frame_selection;  /**< relevant option only e_IOC_FM_PCD_PLCR_L2_FRM_LEN,
                                                                             e_IOC_FM_PCD_PLCR_FULL_FRM_LEN */
} ioc_fm_pcd_plcr_byte_rate_mode_param_t;

/**************************************************************************//**
 @Description   A structure for selcting the policer profile RFC 2698 or
                RFC 4115 parameters
*//***************************************************************************/
typedef struct ioc_fm_pcd_plcr_non_passthrough_alg_param_t {
    ioc_fm_pcd_plcr_rate_mode               rate_mode;                      /**< Byte / Packet */
    ioc_fm_pcd_plcr_byte_rate_mode_param_t  byte_mode_param;                /**< Valid for Byte NULL for Packet */
    uint32_t                                comitted_info_rate;             /**< KBits/Sec or Packets/Sec */
    uint32_t                                comitted_burst_size;            /**< KBits or Packets */
    uint32_t                                peak_or_accessive_info_rate;    /**< KBits/Sec or Packets/Sec */
    uint32_t                                peak_or_accessive_burst_size;   /**< KBits or Packets */
} ioc_fm_pcd_plcr_non_passthrough_alg_param_t;

/**************************************************************************//**
 @Description   A union for defining Policer next engine parameters
*//***************************************************************************/
typedef union ioc_fm_pcd_plcr_next_engine_params_u {
        ioc_fm_pcd_done_action     action;              /**< Action - when next engine is BMI (done) */
        void                       *p_profile;          /**< Policer profile handle -  used when next engine
                                                             is PLCR, must be a SHARED profile */
        void                       *p_direct_scheme;    /**< Direct scheme select - when next engine is Keygen */
} ioc_fm_pcd_plcr_next_engine_params_u;

typedef struct fm_pcd_port_params_t {
    ioc_fm_port_type                    port_type;          /**< Type of port for this profile */
    uint8_t                             port_id;            /**< FM-Port id of port for this profile */
} fm_pcd_port_params_t;

/**************************************************************************//**
 @Description   A structure for selcting the policer profile entry parameters
                (must match struct t_FmPcdPlcrProfileParams defined in fm_pcd_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_pcd_plcr_profile_params_t {
    bool                                        modify;                     /**< TRUE to change an existing profile */
    union {
        struct {
            ioc_fm_pcd_profile_type_selection   profile_type;               /**< Type of policer profile */
            void                               *p_port;                     /**< Type of policer profile */
            uint16_t                            relative_profile_id;        /**< Profile id - relative to shared group or to port */
        } new_params;
        void                                    *p_profile;                 /**< A handle to a profile - use it when modify=TRUE */
    } profile_select;
    ioc_fm_pcd_plcr_algorithm_selection         alg_selection;              /**< Profile Algoritem PASS_THROUGH, RFC_2698, RFC_4115 */
    ioc_fm_pcd_plcr_color_mode                  color_mode;                 /**< COLOR_BLIND, COLOR_AWARE */

    union {
        ioc_fm_pcd_plcr_color                   dflt_color;                 /**< For Color-Blind Pass-Through mode. the policer will re-color
                                                                                 any incoming packet with the deflt value. */
        ioc_fm_pcd_plcr_color                   override;                   /**< For Color-Aware modes. The profile response to a
                                                                                 pre-color value of 2'b11. */
    } color;

    ioc_fm_pcd_plcr_non_passthrough_alg_param_t non_passthrough_alg_param;  /**< RFC2698 or RFC4115 params */

    ioc_fm_pcd_engine                           next_engine_on_green;       /**< Green next engine type */
    ioc_fm_pcd_plcr_next_engine_params_u        params_on_green;            /**< Green next engine params */

    ioc_fm_pcd_engine                           next_engine_on_yellow;      /**< Yellow next engine type */
    ioc_fm_pcd_plcr_next_engine_params_u        params_on_yellow;           /**< Yellow next engine params */

    ioc_fm_pcd_engine                           next_engine_on_red;         /**< Red next engine type */
    ioc_fm_pcd_plcr_next_engine_params_u        params_on_red;              /**< Red next engine params */

    bool                                        trap_profile_on_flow_A;     /**< Trap on flow A */
    bool                                        trap_profile_on_flow_B;     /**< Trap on flow B */
    bool                                        trap_profile_on_flow_C;     /**< Trap on flow C */
    void                                        *id;                        /**< output parameter; Returns the profile Id to be used */
} ioc_fm_pcd_plcr_profile_params_t;

/**************************************************************************//**
 @Description   A structure for modifying CC tree next engine
*//***************************************************************************/
typedef struct ioc_fm_pcd_cc_tree_modify_next_engine_params_t {
    void                                *id;                /**< CC tree Id to be used */
    uint8_t                             grp_indx;           /**< A Group index in the tree */
    uint8_t                             indx;               /**< Entry index in the group defined by grpId */
    ioc_fm_pcd_cc_next_engine_params_t  cc_next_engine_params;
                                                            /**< parameters for the next for the defined Key in the p_Key */
} ioc_fm_pcd_cc_tree_modify_next_engine_params_t;

/**************************************************************************//**
 @Description   A structure for modifying CC node next engine
*//***************************************************************************/
typedef struct ioc_fm_pcd_cc_node_modify_next_engine_params_t {
    void                                *id;                /**< CC node Id to be used */
    uint8_t                             key_indx;           /**< Key index for Next Engine Params modifications;
                                                                 NOTE: This parameter is IGNORED for miss-key!  */
    uint8_t                             key_size;           /**< Key size of added key */
    ioc_fm_pcd_cc_next_engine_params_t  cc_next_engine_params;
                                                            /**< parameters for the next for the defined Key in the p_Key */
} ioc_fm_pcd_cc_node_modify_next_engine_params_t;

/**************************************************************************//**
 @Description   A structure for modifying CC node key and next engine
*//***************************************************************************/
typedef struct ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t {
    void                                *id;                /**< CC node Id to be used */
    uint8_t                             key_indx;           /**< Key index for Next Engine Params modifications;
                                                                 NOTE: This parameter is IGNORED for miss-key!  */
    uint8_t                             key_size;           /**< Key size of added key */
    ioc_fm_pcd_cc_key_params_t          key_params;         /**< it's array with numOfKeys entries each entry in
                                                                 the array of the type ioc_fm_pcd_cc_key_params_t */
} ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t;

/**************************************************************************//**
 @Description   A structure for remove CC node key
*//***************************************************************************/
typedef struct ioc_fm_pcd_cc_node_remove_key_params_t {
    void                                *id;                /**< CC node Id to be used */
    uint8_t                             key_indx;           /**< Key index for Next Engine Params modifications;
                                                                 NOTE: This parameter is IGNORED for miss-key!  */
} ioc_fm_pcd_cc_node_remove_key_params_t;

/**************************************************************************//**
 @Description   A structure for modifying CC node key
*//***************************************************************************/
typedef struct ioc_fm_pcd_cc_node_modify_key_params_t {
    void                                *id;                /**< CC node Id to be used */
    uint8_t                             key_indx;           /**< Key index for Next Engine Params modifications;
                                                                 NOTE: This parameter is IGNORED for miss-key!  */
    uint8_t                             key_size;           /**< Key size of added key */
    uint8_t                             *p_key;             /**< pointer to the key of the size defined in keySize */
    uint8_t                             *p_mask;            /**< pointer to the Mask per key  of the size defined
                                                                 in keySize. p_Key and p_Mask (if defined) has to be
                                                                 of the same size defined in the keySize */
} ioc_fm_pcd_cc_node_modify_key_params_t;


/**************************************************************************//**
 @Function      FM_PCD_IOC_SET_NET_ENV_CHARACTERISTICS

 @Description   Define a set of Network Environment Charecteristics.
                When setting an environment it is important to understand its
                application. It is not meant to describe the flows that will run
                on the ports using this environment, but what the user means TO DO
                with the PCD mechanisms in order to parse-classify-distribute those
                frames.
                By specifying a distinction unit, the user means it would use that option
                for distinction between frames at either a keygen scheme keygen or a coarse
                classification action descriptor. Using interchangeable headers to define a
                unit means that the user is indifferent to which of the interchangeable
                headers is present in the frame, and they want the distinction to be based
                on the presence of either one of them.
                Depending on context, there are limitations to the use of environments. A
                port using the PCD functionality is bound to an environment. Some or even
                all ports may share an environment but also an environment per port is
                possible. When initializing a scheme, a classification plan group (see below),
                or a coarse classification tree, one of the initialized environments must be
                stated and related to. When a port is bound to a scheme, a classification
                plan group, or a coarse classification tree, it MUST be bound to the same
                environment.
                The different PCD modules, may relate (for flows definition) ONLY on
                distinction units as defined by their environment. When initializing a
                scheme for example, it may not choose to select IPV4 as a match for
                recognizing flows unless it was defined in the relating environment. In
                fact, to guide the user through the configuration of the PCD, each module's
                characterization in terms of flows is not done using protocol names, but using
                environment indexes.
                In terms of HW implementation, the list of distinction units sets the LCV vectors
                and later used for match vector, classification plan vectors and coarse classification
                indexing.

 @Param[in,out] ioc_fm_pcd_net_env_params_t   An structure defining the distiction units for this configuration.

 @Return        0 on success; Error code otherwise.

 @Cautions      Allowed only following PCD_Init().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PCD_IOC_SET_NET_ENV_CHARACTERISTICS_COMPAT  _IOWR(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(20), ioc_compat_fm_pcd_net_env_params_t)
#endif
#define FM_PCD_IOC_SET_NET_ENV_CHARACTERISTICS  _IOWR(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(20), ioc_fm_pcd_net_env_params_t)

/**************************************************************************//**
 @Function      FM_PCD_IOC_DELETE_NET_ENV_CHARACTERISTICS

 @Description   Deletes a set of Network Environment Charecteristics.

 @Param[in]     ioc_fm_obj_t        An id of a Network environment object.

 @Return        E_OK on success; Error code otherwise.
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PCD_IOC_DELETE_NET_ENV_CHARACTERISTICS_COMPAT  _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(21), ioc_compat_fm_obj_t)
#endif
#define FM_PCD_IOC_DELETE_NET_ENV_CHARACTERISTICS  _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(21), ioc_fm_obj_t)

/**************************************************************************//**
 @Function      FM_PCD_IOC_KG_SET_SCHEME

 @Description   Initializing or modifying and enabling a scheme for the keygen.
                This routine should be called for adding or modifying a scheme.
                When a scheme needs modifying, the API requires that it will be
                rewritten. In such a case 'override' should be TRUE. If  the
                routine is called for a valid scheme and 'override' is FALSE,
                it will return error.

 @Param[in]     ioc_fm_pcd_kg_scheme_params_t   A structure of parameters for defining the scheme

 @Return        0 on success; Error code otherwise.

 @Cautions      Allowed only following FM_PCD_Init().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PCD_IOC_KG_SET_SCHEME_COMPAT    _IOWR(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(24), ioc_compat_fm_pcd_kg_scheme_params_t)
#endif
#define FM_PCD_IOC_KG_SET_SCHEME    _IOWR(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(24), ioc_fm_pcd_kg_scheme_params_t)

/**************************************************************************//**
 @Function      FM_PCD_IOC_KG_DEL_SCHEME

 @Description   Deleting an initialized scheme.

 @Param[in]     ioc_fm_obj_t        scheme id as initalized by application at FM_PCD_IOC_KG_SET_SCHEME

 @Return        0 on success; Error code otherwise.

 @Cautions      Allowed only following FM_PCD_Init().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PCD_IOC_KG_DEL_SCHEME_COMPAT    _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(25), ioc_compat_fm_obj_t)
#endif
#define FM_PCD_IOC_KG_DEL_SCHEME    _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(25), ioc_fm_obj_t)

/**************************************************************************//**
 @Function      FM_PCD_IOC_CC_BUILD_TREE

 @Description   This routine must be called to define a complete coarse
                classification tree. This is the way to define coarse
                classification to a certain flow - the keygen schemes
                may point only to trees defined in this way.

 @Param[in,out] ioc_fm_pcd_cc_tree_params_t     A structure of parameters to define the tree.

 @Return        0 on success; Error code if the exception is not enabled,
                or is not able to create interrupt.

 @Cautions      Allowed only following PCD_Init().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
//#define FM_PCD_IOC_CC_BUILD_TREE_COMPAT    _IOWR(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(26), ioc_compat_fm_pcd_cc_tree_params_t)
#define FM_PCD_IOC_CC_BUILD_TREE_COMPAT    _IOWR(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(26), compat_uptr_t)
#endif
//#define FM_PCD_IOC_CC_BUILD_TREE    _IOWR(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(26), ioc_fm_pcd_cc_tree_params_t)
#define FM_PCD_IOC_CC_BUILD_TREE    _IOWR(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(26), void *) /* workaround ...*/

/**************************************************************************//**
 @Function      FM_PCD_IOC_CC_DELETE_TREE

 @Description   Deleting an built tree.

 @Param[in]     ioc_fm_obj_t    An id of a CC-tree.

 @Cautions      Allowed only following FM_PCD_Init().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PCD_IOC_CC_DELETE_TREE_COMPAT    _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(27), ioc_compat_fm_obj_t)
#endif
#define FM_PCD_IOC_CC_DELETE_TREE    _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(27), ioc_fm_obj_t)

/**************************************************************************//**
 @Function      FM_PCD_IOC_CC_SET_NODE

 @Description   This routine should be called for each CC (coarse classification)
                node. The whole CC tree should be built bottom up so that each
                node points to already defined nodes. p_NodeId returns the node
                Id to be used by other nodes.

 @Param[in,out] ioc_fm_pcd_cc_node_params_t       A structure for defining the CC node params

 @Return        0 on success; Error code if the exception is not enabled,
                or is not able to create interrupt.

 @Cautions      Allowed only following PCD_Init().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PCD_IOC_CC_SET_NODE_COMPAT    _IOWR(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(28), compat_uptr_t)
#endif
#define FM_PCD_IOC_CC_SET_NODE    _IOWR(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(28), void *) /* workaround ...*/

/**************************************************************************//**
 @Function      FM_PCD_CcDeleteNode

 @Description   Deleting an built node.

 @Param[in]     ioc_fm_obj_t    An id of a CC-node.

 @Cautions      Allowed only following FM_PCD_Init().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PCD_IOC_CC_DELETE_NODE_COMPAT    _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(29), ioc_compat_fm_obj_t)
#endif
#define FM_PCD_IOC_CC_DELETE_NODE    _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(29), ioc_fm_obj_t)

/**************************************************************************//**
 @Function      FM_PCD_IOC_CC_TREE_MODIFY_NEXT_ENGINE

 @Description   Modify the Next Engine Parameters in the entry of the tree.

 @Param[in]     ioc_fm_pcd_cc_tree_modify_next_engine_params_t  A pointer to a structure with the relevant parameters

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_PCD_CcBuildTree().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PCD_IOC_CC_TREE_MODIFY_NEXT_ENGINE_COMPAT   _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(30), ioc_compat_fm_pcd_cc_tree_modify_next_engine_params_t)
#endif
#define FM_PCD_IOC_CC_TREE_MODIFY_NEXT_ENGINE   _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(30), ioc_fm_pcd_cc_tree_modify_next_engine_params_t)

/**************************************************************************//**
 @Function      FM_PCD_IOC_CC_NODE_MODIFY_NEXT_ENGINE

 @Description   Modify the Next Engine Parameters in the relevant key entry of the node.

 @Param[in]     ioc_fm_pcd_cc_node_modify_next_engine_params_t  A pointer to a structure with the relevant parameters

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_PCD_CcSetNode().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PCD_IOC_CC_NODE_MODIFY_NEXT_ENGINE_COMPAT   _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(31), ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t)
#endif
#define FM_PCD_IOC_CC_NODE_MODIFY_NEXT_ENGINE   _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(31), ioc_fm_pcd_cc_node_modify_next_engine_params_t)

/**************************************************************************//**
 @Function      FM_PCD_IOC_CC_NODE_MODIFY_MISS_NEXT_ENGINE

 @Description   Modify the Next Engine Parameters of the Miss key case of the node.

 @Param[in]     ioc_fm_pcd_cc_node_modify_next_engine_params_t  A pointer to a structure with the relevant parameters

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_PCD_CcSetNode().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PCD_IOC_CC_NODE_MODIFY_MISS_NEXT_ENGINE_COMPAT   _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(32), ioc_compat_fm_pcd_cc_node_modify_next_engine_params_t)
#endif
#define FM_PCD_IOC_CC_NODE_MODIFY_MISS_NEXT_ENGINE   _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(32), ioc_fm_pcd_cc_node_modify_next_engine_params_t)

/**************************************************************************//**
 @Function      FM_PCD_IOC_CC_NODE_REMOVE_KEY

 @Description   Remove the key (include Next Engine Parameters of this key) defined by the index of the relevant node .

 @Param[in]     ioc_fm_pcd_cc_node_remove_key_params_t  A pointer to a structure with the relevant parameters

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_PCD_CcSetNode() not only of the relevnt node but also
                the node that points to this node
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PCD_IOC_CC_NODE_REMOVE_KEY_COMPAT   _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(33), ioc_compat_fm_pcd_cc_node_remove_key_params_t)
#endif
#define FM_PCD_IOC_CC_NODE_REMOVE_KEY   _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(33), ioc_fm_pcd_cc_node_remove_key_params_t)

/**************************************************************************//**
 @Function      FM_PCD_IOC_CC_NODE_ADD_KEY

 @Description   Add the key(include Next Engine Parameters of this key)in the index defined by the keyIndex .

 @Param[in]     ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t  A pointer to a structure with the relevant parameters

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_PCD_CcSetNode() not only of the relevnt node but also
                the node that points to this node
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PCD_IOC_CC_NODE_ADD_KEY_COMPAT   _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(34), ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t)
#endif
#define FM_PCD_IOC_CC_NODE_ADD_KEY   _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(34), ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t)

/**************************************************************************//**
 @Function      FM_PCD_IOC_CC_NODE_MODIFY_KEY_AND_NEXT_ENGINE

 @Description   Modify the key and Next Engine Parameters of this key in the index defined by the keyIndex .

 @Param[in]     ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t  A pointer to a structure with the relevant parameters

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_PCD_CcSetNode() not only of the relevnt node but also
                the node that points to this node
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PCD_IOC_CC_NODE_MODIFY_KEY_AND_NEXT_ENGINE_COMPAT   _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(35), ioc_compat_fm_pcd_cc_node_modify_key_and_next_engine_params_t)
#endif
#define FM_PCD_IOC_CC_NODE_MODIFY_KEY_AND_NEXT_ENGINE   _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(35), ioc_fm_pcd_cc_node_modify_key_and_next_engine_params_t)

/**************************************************************************//**
 @Function      FM_PCD_IOC_CC_NODE_MODIFY_KEY

 @Description   Modify the key  in the index defined by the keyIndex .

 @Param[in]     ioc_fm_pcd_cc_node_modify_key_params_t  A pointer to a structure with the relevant parameters

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_PCD_CcSetNode() not only of the relevant node but also
                the node that points to this node
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PCD_IOC_CC_NODE_MODIFY_KEY_COMPAT   _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(36), ioc_compat_fm_pcd_cc_node_modify_key_params_t)
#endif
#define FM_PCD_IOC_CC_NODE_MODIFY_KEY   _IOW(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(36), ioc_fm_pcd_cc_node_modify_key_params_t)

/**************************************************************************//**
 @Function      FM_PCD_IOC_PLCR_SET_PROFILE

 @Description   Sets a profile entry in the policer profile table.
                The routine overrides any existing value.

 @Param[in,out] ioc_fm_pcd_plcr_profile_params_t    A structure of parameters for defining a
                                                    policer profile entry.

 @Return        0 on success; Error code otherwise.

 @Cautions      Allowed only following FM_PCD_Init().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PCD_IOC_PLCR_SET_PROFILE_COMPAT     _IOWR(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(37), ioc_compat_fm_pcd_plcr_profile_params_t)
#endif
#define FM_PCD_IOC_PLCR_SET_PROFILE     _IOWR(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(37), ioc_fm_pcd_plcr_profile_params_t)

/**************************************************************************//**
 @Function      FM_PCD_IOC_PLCR_DEL_PROFILE

 @Description   Delete a profile entry in the policer profile table.
                The routine set entry to invalid.

 @Param[in]     ioc_fm_obj_t        an id of a policer-profile.

 @Return        0 on success; Error code otherwise.

 @Cautions      Allowed only following FM_PCD_Init().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PCD_IOC_PLCR_DEL_PROFILE_COMPAT  _IOWR(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(38), ioc_compat_fm_obj_t)
#endif
#define FM_PCD_IOC_PLCR_DEL_PROFILE     _IOWR(FM_IOC_TYPE_BASE, FM_PCD_IOC_NUM(38), ioc_fm_obj_t)

#endif /* __FM_PCD_IOCTLS_H */
/** @} */ /* end of lnx_ioctl_FM_PCD_Runtime_grp group */
/** @} */ /* end of lnx_ioctl_FM_PCD_grp group */
/** @} */ /* end of lnx_ioctl_FM_grp group */
