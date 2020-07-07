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

#ifndef AVIONICS_FIRMWARE_DRIVERS_XLR_AT_H_
#define AVIONICS_FIRMWARE_DRIVERS_XLR_AT_H_

#define XLR_AT_TO_ENUM(a, b) ((a) << 8 | (b))

// See XLR PRO AT command reference tables (page 33). The enumeration value
// corresponds to the two character ASCII code.
typedef enum {
  // Special commands.
  kXlrAtCommandApplyChanges    = XLR_AT_TO_ENUM('A', 'C'),
  kXlrAtCommandSoftwareReset   = XLR_AT_TO_ENUM('F', 'R'),
  kXlrAtCommandRestoreDefaults = XLR_AT_TO_ENUM('R', 'E'),
  kXlrAtCommandWrite           = XLR_AT_TO_ENUM('W', 'R'),
  // MAC/PHY level commands.
  kXlrAtCommandNetworkId              = XLR_AT_TO_ENUM('I', 'D'),
  kXlrAtCommandRfDataRate             = XLR_AT_TO_ENUM('B', 'R'),
  kXlrAtCommandPowerLevel             = XLR_AT_TO_ENUM('P', 'L'),
  kXlrAtCommandUnicastRetries         = XLR_AT_TO_ENUM('R', 'R'),
  kXlrAtCommandBroadcastMultiTransmit = XLR_AT_TO_ENUM('M', 'T'),
  // Diagnostic commands.
  kXlrAtCommandReceivedSignalStrength   = XLR_AT_TO_ENUM('D', 'B'),
  kXlrAtCommandMacAckTimeouts           = XLR_AT_TO_ENUM('E', 'A'),
  kXlrAtCommandReceivedErrorCount       = XLR_AT_TO_ENUM('E', 'R'),
  kXlrAtCommandGoodPacketsReceived      = XLR_AT_TO_ENUM('G', 'D'),
  kXlrAtCommandTransmissionFailureCount = XLR_AT_TO_ENUM('T', 'R'),
  kXlrAtCommandUnicastsAttemptedCount   = XLR_AT_TO_ENUM('U', 'A'),
  kXlrAtCommandMacUnicastOneHopTime     = XLR_AT_TO_ENUM('%', 'H'),
  kXlrAtCommandMacBroadcastOneHopTime   = XLR_AT_TO_ENUM('%', '8'),
  kXlrAtCommandNetworkDiscoveryTimeout  = XLR_AT_TO_ENUM('N', '?'),
  // RF commands.
  kXlrAtCommandNodeMessagingOptions = XLR_AT_TO_ENUM('C', 'E'),
  kXlrAtCommandBroadcastHops        = XLR_AT_TO_ENUM('B', 'H'),
  kXlrAtCommandNetworkHops          = XLR_AT_TO_ENUM('N', 'H'),
  kXlrAtCommandNetworkDelaySlots    = XLR_AT_TO_ENUM('N', 'N'),
  // RF addressing commands.
  kXlrAtCommandSerialNumberHigh       = XLR_AT_TO_ENUM('S', 'H'),
  kXlrAtCommandSerialNumberLow        = XLR_AT_TO_ENUM('S', 'L'),
  kXlrAtCommandDestinationAddressHigh = XLR_AT_TO_ENUM('D', 'H'),
  kXlrAtCommandDestinationAddressLow  = XLR_AT_TO_ENUM('D', 'L'),
  kXlrAtCommandTransmitOptions        = XLR_AT_TO_ENUM('T', 'O'),
  kXlrAtCommandNodeIdentifier         = XLR_AT_TO_ENUM('N', 'I'),
  kXlrAtCommandNodeDiscoveryTimeout   = XLR_AT_TO_ENUM('N', 'T'),
  kXlrAtCommandNodeRecoveryOptions    = XLR_AT_TO_ENUM('N', 'O'),
  kXlrAtCommandClusterId              = XLR_AT_TO_ENUM('C', 'I'),
  kXlrAtCommandDestinationEndpoint    = XLR_AT_TO_ENUM('D', 'E'),
  kXlrAtCommandSourceEndpoint         = XLR_AT_TO_ENUM('S', 'E'),
  // Addressing discovery/configuration commands.
  kXlrAtCommandDiscoveryNode    = XLR_AT_TO_ENUM('D', 'N'),
  kXlrAtCommandNetworkDiscover  = XLR_AT_TO_ENUM('N', 'D'),
  kXlrAtCommandFindNeighbors    = XLR_AT_TO_ENUM('F', 'N'),
  kXlrAtCommandAesEncryptionKey = XLR_AT_TO_ENUM('K', 'Y'),
  // Serial interfacing commands.
  kXlrAtCommandBaudRate             = XLR_AT_TO_ENUM('B', 'D'),
  kXlrAtCommandParity               = XLR_AT_TO_ENUM('N', 'B'),
  kXlrAtCommandStopBits             = XLR_AT_TO_ENUM('S', 'B'),
  kXlrAtCommandPacketizationTimeout = XLR_AT_TO_ENUM('R', 'O'),
  kXlrAtCommandFlowControlThreshold = XLR_AT_TO_ENUM('F', 'T'),
  kXlrAtCommandApiMode              = XLR_AT_TO_ENUM('A', 'P'),
  kXlrAtCommandApiOptions           = XLR_AT_TO_ENUM('A', 'O'),
  kXlrAtCommandRtsFlowControl       = XLR_AT_TO_ENUM('D', '6'),
  kXlrAtCommandCtsFlowControl       = XLR_AT_TO_ENUM('D', '7'),
  kXlrAtCommandSerialProtocol       = XLR_AT_TO_ENUM('4', 'E'),
  kXlrAtCommandRs485Duplex          = XLR_AT_TO_ENUM('4', 'D'),
  kXlrAtCommandRs485Termination     = XLR_AT_TO_ENUM('4', 'T'),
  // Hardware diagnostic commands.
  kXlrAtCommandTemperature = XLR_AT_TO_ENUM('T', 'P'),
  kXlrAtCommandRssiTimer   = XLR_AT_TO_ENUM('R', 'P'),
  // Ethernet and IP socket mode commands.
  kXlrAtCommandIpSocketModeEnable         = XLR_AT_TO_ENUM('E', 'S'),
  kXlrAtCommandIpSocketBaudRate           = XLR_AT_TO_ENUM('I', 'B'),
  kXlrAtCommandIpProtocol                 = XLR_AT_TO_ENUM('I', 'P'),
  kXlrAtCommandDestinationIpAddress       = XLR_AT_TO_ENUM('D', 'X'),
  kXlrAtCommandSourcePort                 = XLR_AT_TO_ENUM('C', '0'),
  kXlrAtCommandDestinationPort            = XLR_AT_TO_ENUM('D', 'Y'),
  kXlrAtCommandTcpClientConnectionTimeout = XLR_AT_TO_ENUM('T', 'M'),
  kXlrAtCommandTcpServerConnectionTimeout = XLR_AT_TO_ENUM('T', 'S'),
  kXlrAtCommandIpAddressingMode           = XLR_AT_TO_ENUM('M', 'A'),
  kXlrAtCommandXlrIpAddress               = XLR_AT_TO_ENUM('M', 'Y'),
  kXlrAtCommandSubnetMask                 = XLR_AT_TO_ENUM('M', 'K'),
  kXlrAtCommandDefaultGatewayAddress      = XLR_AT_TO_ENUM('G', 'W'),
  kXlrAtCommandDnsAddress                 = XLR_AT_TO_ENUM('N', 'S'),
  kXlrAtCommandEthernetMacAddress         = XLR_AT_TO_ENUM('%', 'M'),
  // Device cloud commands.
  // Web configuration commands.
  // Ethernet RF bridging commands.
  // AT command options.
  kXlrAtCommandCommandSequenceCharacter = XLR_AT_TO_ENUM('C', 'C'),
  kXlrAtCommandExitCommandMode          = XLR_AT_TO_ENUM('C', 'N'),
  kXlrAtCommandCommandModeTimeout       = XLR_AT_TO_ENUM('C', 'T'),
  kXlrAtCommandGuardTime                = XLR_AT_TO_ENUM('G', 'T'),
  // Firmware commands.
  kXlrAtCommandFirmwareVersion           = XLR_AT_TO_ENUM('V', 'B'),
  kXlrAtCommandRfModuleFirmwareVersion   = XLR_AT_TO_ENUM('V', 'R'),
  kXlrAtCommandRfModuleHardwareVersion   = XLR_AT_TO_ENUM('H', 'V'),
  kXlrAtCommandBaseboardHardwareVersion  = XLR_AT_TO_ENUM('V', 'H'),
  kXlrAtCommandCompatibility             = XLR_AT_TO_ENUM('*', 'C'),
  kXlrAtCommandDeviceTypeIdentifier      = XLR_AT_TO_ENUM('D', 'D'),
  kXlrAtCommandPartNumber                = XLR_AT_TO_ENUM('P', 'N'),
  kXlrAtCommandMaximumPacketPayloadBytes = XLR_AT_TO_ENUM('N', 'P'),
  kXlrAtCommandConfigurationCrc          = XLR_AT_TO_ENUM('C', 'K')
} XlrAtCommand;

#endif  // AVIONICS_FIRMWARE_DRIVERS_XLR_AT_H_
