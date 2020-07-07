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
 @File          fm_port_ioctls.h

 @Description   FM Port routines
*//***************************************************************************/
#ifndef __FM_PORT_IOCTLS_H
#define __FM_PORT_IOCTLS_H

#include "net_ioctls.h"
#include "fm_ioctls.h"
#include "fm_pcd_ioctls.h"


/**************************************************************************//**
 @Group         lnx_ioctl_FM_grp Frame Manager Linux IOCTL API

 @Description   FM Linux ioctls definitions and enums

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Group         lnx_ioctl_FM_PORT_grp FM Port

 @Description   FM Port API

                The FM uses a general module called "port" to represent a Tx port
                (MAC), an Rx port (MAC), offline parsing flow or host command
                flow. There may be up to 17 (may change) ports in an FM - 5 Tx
                ports (4 for the 1G MACs, 1 for the 10G MAC), 5 Rx Ports, and 7
                Host command/Offline parsing ports. The SW driver manages these
                ports as sub-modules of the FM, i.e. after an FM is initialized,
                its ports may be initialized and operated upon.
                The port is initialized aware of its type, but other functions on
                a port may be indifferent to its type. When necessary, the driver
                verifies coherency and returns error if applicable.
                On initialization, user specifies the port type and it's index
                (relative to the port's type). Host command and Offline parsing
                ports share the same id range, I.e user may not initialized host
                command port 0 and offline parsing port 0.

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Group         lnx_ioctl_FM_PORT_runtime_control_grp FM Port Runtime Control Unit

 @Description   FM Port Runtime control unit API functions, definitions and enums.

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Collection    General FM Port defines
*//***************************************************************************/
#ifdef CONFIG_FMAN_P1023
#define IOC_FM_PORT_NUM_OF_CONGESTION_GRPS       32 /**< Total number of congestion groups in QM */
#else
#define IOC_FM_PORT_NUM_OF_CONGESTION_GRPS      256 /**< Total number of congestion groups in QM */
#endif
/* @} */


/**************************************************************************//**
 @Description   struct for defining port PCD modes
                (must match enum e_FmPortPcdSupport defined in fm_port_ext.h)
*//***************************************************************************/
typedef enum ioc_fm_port_pcd_support {
    e_IOC_FM_PORT_PCD_SUPPORT_NONE = 0,             /**< BMI to BMI, PCD is not used */
    e_IOC_FM_PORT_PCD_SUPPORT_PRS_ONLY,             /**< Use only Parser */
    e_IOC_FM_PORT_PCD_SUPPORT_PLCR_ONLY,            /**< Use only Policer */
    e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_PLCR,         /**< Use Parser and Policer */
    e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_KG,           /**< Use Parser and Keygen */
    e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_CC,    /**< Use Parser, Keygen and Coarse Classification */
    e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_CC_AND_PLCR, /**< Use all PCD engines */
    e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_PLCR,  /**< Use Parser, Keygen and Policer */
#ifdef FM_CAPWAP_SUPPORT
    e_IOC_FM_PORT_PCD_SUPPORT_CC_ONLY,              /**< Use only Coarse Classification */
    e_IOC_FM_PORT_PCD_SUPPORT_CC_AND_KG,            /**< Use Coarse Classification,and Keygen */
    e_IOC_FM_PORT_PCD_SUPPORT_CC_AND_KG_AND_PLCR    /**< Use Coarse Classification, Keygen and Policer */
#endif
} ioc_fm_port_pcd_support;

/**************************************************************************//**
 @Description   enum for defining FM Port counters
                (must match enum e_FmPortCounters defined in fm_port_ext.h)
*//***************************************************************************/
typedef enum fm_port_counters {
    e_IOC_FM_PORT_COUNTERS_CYCLE,                       /**< BMI performance counter */
    e_IOC_FM_PORT_COUNTERS_TASK_UTIL,                   /**< BMI performance counter */
    e_IOC_FM_PORT_COUNTERS_QUEUE_UTIL,                  /**< BMI performance counter */
    e_IOC_FM_PORT_COUNTERS_DMA_UTIL,                    /**< BMI performance counter */
    e_IOC_FM_PORT_COUNTERS_FIFO_UTIL,                   /**< BMI performance counter */
    e_IOC_FM_PORT_COUNTERS_RX_PAUSE_ACTIVATION,         /**< BMI Rx only performance counter */
    e_IOC_FM_PORT_COUNTERS_FRAME,                       /**< BMI statistics counter */
    e_IOC_FM_PORT_COUNTERS_DISCARD_FRAME,               /**< BMI statistics counter */
    e_IOC_FM_PORT_COUNTERS_DEALLOC_BUF,                 /**< BMI deallocate buffer statistics counter */
    e_IOC_FM_PORT_COUNTERS_RX_BAD_FRAME,                /**< BMI Rx only statistics counter */
    e_IOC_FM_PORT_COUNTERS_RX_LARGE_FRAME,              /**< BMI Rx only statistics counter */
    e_IOC_FM_PORT_COUNTERS_RX_OUT_OF_BUFFERS_DISCARD,   /**< BMI Rx only statistics counter */
    e_IOC_FM_PORT_COUNTERS_RX_FILTER_FRAME,             /**< BMI Rx & OP only statistics counter */
    e_IOC_FM_PORT_COUNTERS_RX_LIST_DMA_ERR,             /**< BMI Rx, OP & HC only statistics counter */
    e_IOC_FM_PORT_COUNTERS_WRED_DISCARD,                /**< BMI OP & HC only statistics counter */
    e_IOC_FM_PORT_COUNTERS_LENGTH_ERR,                  /**< BMI non-Rx statistics counter */
    e_IOC_FM_PORT_COUNTERS_UNSUPPRTED_FORMAT,           /**< BMI non-Rx statistics counter */
    e_IOC_FM_PORT_COUNTERS_DEQ_TOTAL,                   /**< QMI counter */
    e_IOC_FM_PORT_COUNTERS_ENQ_TOTAL,                   /**< QMI counter */
    e_IOC_FM_PORT_COUNTERS_DEQ_FROM_DEFAULT,            /**< QMI counter */
    e_IOC_FM_PORT_COUNTERS_DEQ_CONFIRM                  /**< QMI counter */
} fm_port_counters;

/**************************************************************************//**
 @Description   Structure for Port id parameters.
                Fields commented 'IN' are passed by the port module to be used
                by the FM module.
                Fields commented 'OUT' will be filled by FM before returning to port.
*//***************************************************************************/
typedef struct ioc_fm_port_congestion_groups_t {
    uint16_t    num_of_congestion_grps_to_consider; /**< Size of congestion_grps_to_consider array */
    uint8_t     congestion_grps_to_consider [IOC_FM_PORT_NUM_OF_CONGESTION_GRPS];   /**< list of congestion groups */
} ioc_fm_port_congestion_groups_t;

/**************************************************************************//**
 @Description   struct for defining Dual Tx rate limiting scale
                (identical to e_FmPortDualRateLimiterScaleDown defined in fm_port_ext.h)
*//***************************************************************************/
typedef enum fm_port_dual_rate_limiter_scale_down {
    e_IOC_FM_PORT_DUAL_RATE_LIMITER_NONE = 0,           /**< Use only single rate limiter  */
    e_IOC_FM_PORT_DUAL_RATE_LIMITER_SCALE_DOWN_BY_2,    /**< Divide high rate limiter by 2 */
    e_IOC_FM_PORT_DUAL_RATE_LIMITER_SCALE_DOWN_BY_4,    /**< Divide high rate limiter by 4 */
    e_IOC_FM_PORT_DUAL_RATE_LIMITER_SCALE_DOWN_BY_8     /**< Divide high rate limiter by 8 */
} fm_port_dual_rate_limiter_scale_down;

/**************************************************************************//**
 @Description   struct for defining Tx rate limiting
                (identical to t_FmPortRateLimit defined in fm_port_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_port_rate_limit {
    uint16_t                            max_burst_size;         /**< in KBytes for Tx ports, in frames
                                                                     for offline parsing ports. (note that
                                                                     for early chips burst size is
                                                                     rounded up to a multiply of 1000 frames).*/
    uint32_t                            rate_limit;             /**< in Kb/sec for Tx ports, in frame/sec for
                                                                     offline parsing ports. Rate limit refers to
                                                                     data rate (rather than line rate). */
    fm_port_dual_rate_limiter_scale_down rate_limit_divider;    /**< For offline parsing ports only. Not-valid
                                                                     for some earlier chip revisions */
} ioc_fm_port_rate_limit_t;


/**************************************************************************//**
 @Function      FM_PORT_IOC_DISABLE

 @Description   Gracefully disable an FM port. The port will not start new tasks after all
                tasks associated with the port are terminated.

 @Return        0 on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Init().
                This is a blocking routine, it returns after port is
                gracefully stopped, i.e. the port will not except new frames,
                but it will finish all frames or tasks which were already began
*//***************************************************************************/
#define FM_PORT_IOC_DISABLE   _IO(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(1))

/**************************************************************************//**
 @Function      FM_PORT_IOC_ENABLE

 @Description   A runtime routine provided to allow disable/enable of port.

 @Return        0 on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Init().
*//***************************************************************************/
#define FM_PORT_IOC_ENABLE   _IO(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(2))

/**************************************************************************//**
 @Function      FM_PORT_IOC_SET_RATE_LIMIT

 @Description   Calling this routine enables rate limit algorithm.
                By default, this functionality is disabled.
                Note that rate-limit mechanism uses the FM time stamp.
                The selected rate limit specified here would be
                rounded to the nearest power of 2 multiplication
                (i.e. up to twice the required rate).

                May be used for Tx and offline parsing ports only

 @Param[in]     ioc_fm_port_rate_limit A structure of rate limit parameters

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Init().
*//***************************************************************************/
#define FM_PORT_IOC_SET_RATE_LIMIT _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(3), ioc_fm_port_rate_limit_t)

/**************************************************************************//**
 @Function      FM_PORT_IOC_REMOVE_RATE_LIMIT

 @Description   Calling this routine disables the previously enabled rate limit.

                May be used for Tx and offline parsing ports only

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Init().
*//***************************************************************************/
#define FM_PORT_IOC_REMOVE_RATE_LIMIT _IO(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(5))

/**************************************************************************//**
 @Function      FM_PORT_IOC_SET_ERRORS_ROUTE

 @Description   Errors selected for this routine will cause a frame with that error
                to be enqueued to error queue.
                Errors not selected for this routine will cause a frame with that error
                to be enqueued to the one of the other port queues.
                By default all errors are defined to be enqueued to error queue.
                Errors that were configured to be discarded (at initialization)
                may not be selected here.

                May be used for Rx and offline parsing ports only

 @Param[in]     ioc_fm_port_frame_err_select_t  A list of errors to enqueue to error queue

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Config() and before FM_PORT_Init().
*//***************************************************************************/
#define FM_PORT_IOC_SET_ERRORS_ROUTE   _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(4), ioc_fm_port_frame_err_select_t)

/**************************************************************************//**
 @Group         lnx_ioctl_FM_PORT_pcd_runtime_control_grp FM Port PCD Runtime Control Unit

 @Description   FM Port PCD Runtime control unit API functions, definitions and enums.

 @{
*//***************************************************************************/

/**************************************************************************//**
 @Description   A structure of scheme parameters
                (must match struct t_FmPcdKgSchemeSelect defined in fm_port_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_pcd_kg_scheme_select_t {
    bool        direct;                     /**< TRUE to use 'scheme_id' directly, FALSE to use LCV.*/
    void        *scheme_id;                 /**< Relevant for 'direct'=TRUE only.
                                                 'scheme_id' selects the scheme after parser. */
} ioc_fm_pcd_kg_scheme_select_t;

/**************************************************************************//**
 @Description    A structure for defining the Parser starting point
                (must match struct t_FmPcdPrsStart defined in fm_port_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_pcd_prs_start_t {
    uint8_t             parsing_offset; /**< Number of bytes from begining of packet to
                                             start parsing */
    ioc_net_header_type first_prs_hdr;  /**< The type of the first header axpected at
                                             'parsing_offset' */
} ioc_fm_pcd_prs_start_t;

/**************************************************************************//**
 @Description   Scheme IDs structure
                (must match struct t_FmPcdPortSchemesParams defined in fm_port_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_pcd_port_schemes_params_t {
    uint8_t     num_of_schemes;                         /**< Number of schemes for port to be bound to. */
    void        *scheme_ids [IOC_FM_PCD_KG_NUM_OF_SCHEMES];
                                                        /**< Array of 'num_of_schemes' schemes for the
                                                             port to be bound to */
} ioc_fm_pcd_port_schemes_params_t;

/**************************************************************************//**
 @Description   Union for defining port protocol parameters for parser
                (must match union u_FmPcdHdrPrsOpts defined in fm_port_ext.h)
*//***************************************************************************/
typedef union ioc_fm_pcd_hdr_prs_opts_u {
    /* MPLS */
    struct {
        bool                label_interpretation_enable;/**< When this bit is set, the last MPLS label will be
                                                             interpreted as described in HW spec table. When the bit
                                                             is cleared, the parser will advance to MPLS next parse */
        ioc_net_header_type next_parse;                 /**< must be equal or higher than IPv4 */
    } mpls_prs_options;
    /* VLAN */
    struct {
        uint16_t            tag_protocol_id1;           /**< User defined Tag Protocol Identifier, to be recognized
                                                             on VLAN TAG on top of 0x8100 and 0x88A8 */
        uint16_t            tag_protocol_id2;           /**< User defined Tag Protocol Identifier, to be recognized
                                                             on VLAN TAG on top of 0x8100 and 0x88A8 */
    } vlan_prs_options;
    /* IPV6 */
    struct {
        bool                routing_hdr_disable;        /**< Disable routing header */
    } ipv6_prs_options;

    /* UDP */
    struct {
        bool                pad_ignore_checksum;        /**< TRUE to ignore pad in checksum */
    } udp_prs_options;

    /* TCP */
    struct {
        bool                pad_ignore_checksum;        /**< TRUE to ignore pad in checksum */
    } tcp_prs_options;
} ioc_fm_pcd_hdr_prs_opts_u;


/**************************************************************************//**
 @Description   A structure for defining each header for the parser
                (must match struct t_FmPcdPrsAdditionalHdrParams defined in fm_port_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_pcd_prs_additional_hdr_params_t {
    ioc_net_header_type         hdr;                /**< Selected header */
    bool                        err_disable;        /**< TRUE to disable error indication */
    bool                        soft_prs_enable;    /**< Enable jump to SW parser when this
                                                         header is recognized by the HW parser. */
    uint8_t                     index_per_hdr;      /**< Normally 0, if more than one sw parser
                                                         attachments exists for the same header,
                                                         (in the main sw parser code) use this
                                                         index to distinguish between them. */
    bool                        use_prs_opts;       /**< TRUE to use parser options. */
    ioc_fm_pcd_hdr_prs_opts_u   prs_opts;           /**< A unuion according to header type,
                                                         defining the parser options selected.*/
} ioc_fm_pcd_prs_additional_hdr_params_t;

/**************************************************************************//**
 @Description   struct for defining port PCD parameters
                (must match t_FmPortPcdPrsParams defined in fm_port_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_port_pcd_prs_params_t {
    uint8_t                                 prs_res_priv_info;          /**< The private info provides a method of inserting
                                                                             port information into the parser result. This information
                                                                             may be extracted by Keygen and be used for frames
                                                                             distribution when a per-port distinction is required,
                                                                             it may also be used as a port logical id for analyazing
                                                                             incoming frames. */
    uint8_t                                 parsing_offset;             /**< Number of bytes from begining of packet to
                                                                             start parsing */
    ioc_net_header_type                     first_prs_hdr;              /**< The type of the first header axpected at
                                                                             'parsingOffset' */
    bool                                    includeInPrsStatistics;     /**< TRUE to include this port in the parser statistics;
                                                                              NOTE: this field is not valid when the FN is in "guest" mode. */
    uint8_t                                 num_of_hdrs_with_additional_params;
                                                        /**< Normally 0, some headers may get
                                                             special parameters */
    ioc_fm_pcd_prs_additional_hdr_params_t  additional_params[IOC_FM_PCD_PRS_NUM_OF_HDRS];
                                                        /**< A structure of additional parameters
                                                             for each header that requires them */
    bool                                    set_vlan_tpid1;             /**< TRUE to configure user selection of Ethertype to
                                                                             indicate a VLAN tag (in addition to the TPID values
                                                                             0x8100 and 0x88A8). */
    uint16_t                                vlan_tpid1;                 /**< extra tag to use if setVlanTpid1=TRUE. */
    bool                                    set_vlan_tpid2;             /**< TRUE to configure user selection of Ethertype to
                                                                             indicate a VLAN tag (in addition to the TPID values
                                                                             0x8100 and 0x88A8). */
    uint16_t                                vlan_tpid2;                 /**< extra tag to use if setVlanTpid1=TRUE. */
} ioc_fm_port_pcd_prs_params_t;

/**************************************************************************//**
 @Description   struct for defining coarse alassification parameters
                (must match t_FmPortPcdCcParams defined in fm_port_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_port_pcd_cc_params_t {
    void                *cc_tree_id; /**< CC tree id */
} ioc_fm_port_pcd_cc_params_t;

/**************************************************************************//**
 @Description   struct for defining keygen parameters
                (must match t_FmPortPcdKgParams defined in fm_port_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_port_pcd_kg_params_t {
    uint8_t             num_of_schemes;                 /**< Number of schemes for port to be bound to. */
    void                *schemes_ids[IOC_FM_PCD_KG_NUM_OF_SCHEMES];
                                                        /**< Array of 'numOfSchemes' schemes for the
                                                             port to be bound to */
    bool                direct_scheme;                  /**< TRUE for going from parser to a specific scheme,
                                                             regardless of parser result */
    void                *direct_scheme_id;              /**< relevant only if direct == TRUE */
} ioc_fm_port_pcd_kg_params_t;

/**************************************************************************//**
 @Description   struct for defining policer parameters
                (must match t_FmPortPcdPlcrParams defined in fm_port_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_port_pcd_plcr_params_t {
    void                *plcr_profile_id;               /**< relevant only if
                                                             e_IOC_FM_PCD_SUPPORT_PLCR_ONLY or
                                                             e_IOC_FM_PCD_SUPPORT_PRS_AND_PLCR were selected */
} ioc_fm_port_pcd_plcr_params_t;

/**************************************************************************//**
 @Description   struct for defining port PCD parameters
                (must match struct t_FmPortPcdParams defined in fm_port_ext.h)
*//***************************************************************************/
typedef struct ioc_fm_port_pcd_params_t {
    ioc_fm_port_pcd_support         pcd_support;    /**< Relevant for Rx and offline ports only.
                                                         Describes the active PCD engines for this port. */
    void                            *net_env_id;    /**< HL Unused in PLCR only mode */
    ioc_fm_port_pcd_prs_params_t    *p_prs_params;  /**< Parser parameters for this port */
    ioc_fm_port_pcd_cc_params_t     *p_cc_params;   /**< Coarse classification parameters for this port */
    ioc_fm_port_pcd_kg_params_t     *p_kg_params;   /**< Keygen parameters for this port */
    ioc_fm_port_pcd_plcr_params_t   *p_plcr_params; /**< Policer parameters for this port */
} ioc_fm_port_pcd_params_t;

/**************************************************************************//**
 @Description   FQID parameters structure
*//***************************************************************************/
typedef struct ioc_fm_port_pcd_fqids_params_t {
    uint32_t            num_fqids;  /**< Number of fqids to be allocated for the port */
    uint8_t             alignment;  /**< Alignment required for this port */
    uint32_t            base_fqid;  /**< output parameter - the base fqid */
} ioc_fm_port_pcd_fqids_params_t;


/**************************************************************************//**
 @Function      FM_PORT_IOC_ALLOC_PCD_FQIDS

 @Description   Allocates FQID's

                May be used for Rx and offline parsing ports only

 @Param[in,out] ioc_fm_port_pcd_fqids_params_t  Parameters for allocating FQID's

 @Return        0 on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Init().
*//***************************************************************************/
#define FM_PORT_IOC_ALLOC_PCD_FQIDS   _IOWR(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(19), ioc_fm_port_pcd_fqids_params_t)

/**************************************************************************//**
 @Function      FM_PORT_IOC_FREE_PCD_FQIDS

 @Description   Frees previously-allocated FQIDs

                May be used for Rx and offline parsing ports only

 @Param[in]		uint32_t	Base FQID of previously allocated range.

 @Return        0 on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Init().
*//***************************************************************************/
#define FM_PORT_IOC_FREE_PCD_FQIDS   _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(19), uint32_t)

/**************************************************************************//**
 @Function      FM_PORT_IOC_SET_PCD

 @Description   Calling this routine defines the port's PCD configuration.
                It changes it from its default configuration which is PCD
                disabled (BMI to BMI) and configures it according to the passed
                parameters.

                May be used for Rx and offline parsing ports only

 @Param[in]     ioc_fm_port_pcd_params_t    A Structure of parameters defining the port's PCD
                                            configuration.

 @Return        0 on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Init().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PORT_IOC_SET_PCD_COMPAT _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(20), ioc_compat_fm_port_pcd_params_t)
#endif
#define FM_PORT_IOC_SET_PCD _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(20), ioc_fm_port_pcd_params_t)

/**************************************************************************//**
 @Function      FM_PORT_IOC_DELETE_PCD

 @Description   Calling this routine releases the port's PCD configuration.
                The port returns to its default configuration which is PCD
                disabled (BMI to BMI) and all PCD configuration is removed.

                May be used for Rx and offline parsing ports which are
                in PCD mode  only

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Init().
*//***************************************************************************/
#define FM_PORT_IOC_DELETE_PCD _IO(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(21))

/**************************************************************************//**
 @Function      FM_PORT_IOC_DETACH_PCD

 @Description   Calling this routine detaches the port from its PCD functionality.
                The port returns to its default flow which is BMI to BMI.

                May be used for Rx and offline parsing ports which are
                in PCD mode only

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Init().
*//***************************************************************************/
#define FM_PORT_IOC_DETACH_PCD _IO(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(22))

/**************************************************************************//**
 @Function      FM_PORT_IOC_ATTACH_PCD

 @Description   This routine may be called after FM_PORT_DetachPCD was called,
                to return to the originally configured PCD support flow.
                The couple of routines are used to allow PCD configuration changes
                that demand that PCD will not be used while changes take place.

                May be used for Rx and offline parsing ports which are
                in PCD mode only

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Init().
*//***************************************************************************/
#define FM_PORT_IOC_ATTACH_PCD _IO(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(23))

/**************************************************************************//**
 @Function      FM_PORT_IOC_PCD_PLCR_ALLOC_PROFILES

 @Description   This routine may be called only for ports that use the Policer in
                order to allocate private policer profiles.

 @Param[in]     uint16_t       The number of required policer profiles

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Init() and FM_PCD_Init(), and before FM_PORT_SetPCD().
*//***************************************************************************/
#define FM_PORT_IOC_PCD_PLCR_ALLOC_PROFILES     _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(24), uint16_t)

/**************************************************************************//**
 @Function      FM_PORT_IOC_PCD_PLCR_FREE_PROFILES

 @Description   This routine should be called for freeing private policer profiles.

 @Return        E_OK on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Init() and FM_PCD_Init(), and before FM_PORT_SetPCD().
*//***************************************************************************/
#define FM_PORT_IOC_PCD_PLCR_FREE_PROFILES     _IO(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(25))

/**************************************************************************//**
 @Function      FM_PORT_IOC_PCD_KG_MODIFY_INITIAL_SCHEME

 @Description   This routine may be called only for ports that use the keygen in
                order to change the initial scheme frame should be routed to.
                The change may be of a scheme id (in case of direct mode),
                from direct to indirect, or from indirect to direct - specifying the scheme id.

 @Param[in]     ioc_fm_pcd_kg_scheme_select_t   A structure of parameters for defining whether
                                                a scheme is direct/indirect, and if direct - scheme id.

 @Return        0 on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Init().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PORT_IOC_PCD_KG_MODIFY_INITIAL_SCHEME_COMPAT _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(26), ioc_compat_fm_pcd_kg_scheme_select_t)
#endif
#define FM_PORT_IOC_PCD_KG_MODIFY_INITIAL_SCHEME _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(26), ioc_fm_pcd_kg_scheme_select_t)

/**************************************************************************//**
 @Function      FM_PORT_IOC_PCD_PLCR_MODIFY_INITIAL_PROFILE

 @Description   This routine may be called for ports with flows e_IOC_FM_PCD_SUPPORT_PLCR_ONLY or
                e_IOC_FM_PCD_SUPPORT_PRS_AND_PLCR  only, to change the initial Policer profile frame
                should be routed to. The change may be of a profile and/or absolute/direct mode
                selection.

 @Param[in]     ioc_fm_obj_t       Policer profile Id as returned from FM_PCD_PlcrSetProfile.

 @Return        0 on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Init().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PORT_IOC_PCD_PLCR_MODIFY_INITIAL_PROFILE_COMPAT _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(27), ioc_compat_fm_obj_t)
#endif
#define FM_PORT_IOC_PCD_PLCR_MODIFY_INITIAL_PROFILE _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(27), ioc_fm_obj_t)

/**************************************************************************//**
 @Function      FM_PORT_IOC_PCD_CC_MODIFY_TREE

 @Description   This routine may be called to change this port connection to
                a pre-initializes coarse classification Tree.

 @Param[in]     ioc_fm_obj_t    Id of new coarse classification tree selected for this port.

 @Return        0 on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Init().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PORT_IOC_PCD_CC_MODIFY_TREE_COMPAT _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(28), ioc_compat_fm_obj_t)
#endif
#define FM_PORT_IOC_PCD_CC_MODIFY_TREE _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(28), ioc_fm_obj_t)

/**************************************************************************//**
 @Function      FM_PORT_IOC_PCD_KG_BIND_SCHEMES

 @Description   These routines may be called for modifying the binding of ports
                to schemes. The scheme itself is not added,
                just this specific port starts using it.

 @Param[in]     ioc_fm_pcd_port_schemes_params_t    Schemes parameters structre

 @Return        0 on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Init().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PORT_IOC_PCD_KG_BIND_SCHEMES_COMPAT _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(30), ioc_compat_fm_pcd_port_schemes_params_t)
#endif
#define FM_PORT_IOC_PCD_KG_BIND_SCHEMES _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(30), ioc_fm_pcd_port_schemes_params_t)

/**************************************************************************//**
 @Function      FM_PORT_IOC_PCD_KG_UNBIND_SCHEMES

 @Description   These routines may be called for modifying the binding of ports
                to schemes. The scheme itself is not removed or invalidated,
                just this specific port stops using it.

 @Param[in]     ioc_fm_pcd_port_schemes_params_t    Schemes parameters structre

 @Return        0 on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Init().
*//***************************************************************************/
#if defined(CONFIG_COMPAT)
#define FM_PORT_IOC_PCD_KG_UNBIND_SCHEMES_COMPAT _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(31), ioc_compat_fm_pcd_port_schemes_params_t)
#endif
#define FM_PORT_IOC_PCD_KG_UNBIND_SCHEMES _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(31), ioc_fm_pcd_port_schemes_params_t)

/**************************************************************************//**
 @Function      FM_PORT_IOC_PCD_PRS_MODIFY_START_OFFSET

 @Description   Runtime change of the parser start offset within the header.

 @Param[in]     ioc_fm_pcd_prs_start_t  A structure of parameters for defining the
                                        start point for the parser.

 @Return        0 on success; Error code otherwise.

 @Cautions      Allowed only following FM_PORT_Init().
*//***************************************************************************/
#define      FM_PORT_IOC_PCD_PRS_MODIFY_START_OFFSET _IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(32), ioc_fm_pcd_prs_start_t)

/** @} */ /* end of lnx_ioctl_FM_PORT_pcd_runtime_control_grp group */
/** @} */ /* end of lnx_ioctl_FM_PORT_runtime_control_grp group */
/** @} */ /* end of lnx_ioctl_FM_PORT_grp group */
/** @} */ /* end of lnx_ioctl_FM_grp group */


#endif /* __FM_PORT_IOCTLS_H */
