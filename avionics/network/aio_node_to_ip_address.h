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

#ifndef AVIONICS_NETWORK_AIO_NODE_TO_IP_ADDRESS_H_
#define AVIONICS_NETWORK_AIO_NODE_TO_IP_ADDRESS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "avionics/common/network_config.h"
#include "avionics/network/aio_node.h"

// Produces the IP address for a given AIO node.
IpAddress AioNodeToIpAddress(AioNode node);

// Produces the AIO node for a given final address octet.
AioNode FinalOctetToAioNode(uint8_t final_octet);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_NETWORK_AIO_NODE_TO_IP_ADDRESS_H_
