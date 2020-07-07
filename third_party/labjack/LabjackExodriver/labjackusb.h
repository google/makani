//-----------------------------------------------------------------------------
//
//  labjackusb.h
//
//  Header file for the labjackusb library.
//
//  support@labjack.com
//
//-----------------------------------------------------------------------------
//
//  Version History
//
//  0.90 - Initial release (LJUSB_AbortPipe not supported)
//
//  1.00 - Added LJUSB_SetBulkReadTimeout
//
//  1.10 - Changed the HANDLE to a void * (previously int)
//       - Added LJUSB_GetLibraryVersion
//       - Removed UE9_PIPE_EP2_OUT
//       - Changed the values of the pipes (incremented by 1)
//       - Removed function LJUSB_SetBulkReadTimeout
//       - Changed LJUSB_LINUX_DRIVER_VERSION define name to
//         LJUSB_LINUX_LIBRARY_VERSION
//
//  2.00 - Re-implemented library using libusb 1.0.  No longer requires LabJack
//         kernel module.
//       - Replaced define names U3_PIPE_EP1_IN and U3_PIPE_EP2_IN with
//         U3_PIPE_EP2_IN and U3_PIPE_EP3_IN
//       - Added U6 support
//
//  2.01 - Added U12 support
//       - Added Wireless Bridge support
//
//  2.02 - Buxfix release
//
//  2.03 - Don't print libusb timeout errors on interupt transfers
//
//  2.04 - Removed exit calls
//       - Now using unique a libusb session instead of the default
//       - Setting errno more often when libusb errors occur
//       - Changed guard define to LABJACKUSB_H_
//       - Changed LJUSB_GetDevCount return data type to unsigned int
//       - Moved LJ_VENDOR_ID to the header file so it's public
//
//  2.05 - Updated Wireless bridge support for bulk transfers
//       - Updated Wireless bridge product ID to 1000
//       - Fixed some compiler warnings
//       - Renamed LJUSB_LINUX_LIBRARY_VERSION define to LJUSB_LIBRARY_VERSION
//       - Changed LJUSB_Write/Read/Stream to return 0 on error instead of -1
//       - Changed LJUSB_GetDevCounts return data type unsigned int
//       - Changed LJUSB_GetDevCounts to return all products counted instead
//         of 0
//       - Added LJUSB_WriteTO, LJUSB_ReadTO and LJUSB_StreamTO functions
//       - Added LJUSB_GetDeviceDescriptorReleaseNumber function
//       - Added LJUSB_GetHIDReportDescriptor function for U12
//       - Now using minor versions properly in LJUSB_LIBRARY_VERSION define
//       - Renamed DEBUG define to LJ_DEBUG in source
//       - Made global variables static
//       - Replaced LJUSB_IsHandleValid checks with LJUSB_isNullHandle to
//         improve LJUSB_Write/Read/Stream speeds
//       - Initial T7 support
//       - Initial Digit support
//       - Added LJUSB_ResetConnection function.
//  2.0503 - Fixed open calls to not steal handles from other processes on
//           Linux.
//         - libusb error prints are silenced when LJ_DEBUG is not enabled.
//         - Added revision number to library version number. The float version
//           number 2.0503 is equivalent to 2.5.3 (major.minor.revision).
//-----------------------------------------------------------------------------
//

#ifndef LABJACKUSB_H_
#define LABJACKUSB_H_

#define LJUSB_LIBRARY_VERSION 2.0503f

#include <stdbool.h>

typedef void * HANDLE;
typedef unsigned int UINT;
typedef unsigned char BYTE;

//Vendor ID
#define LJ_VENDOR_ID            0x0cd5

//Product IDs
#define UE9_PRODUCT_ID         9
#define U3_PRODUCT_ID          3
#define U6_PRODUCT_ID          6
#define U12_PRODUCT_ID         1
#define BRIDGE_PRODUCT_ID      1000
#define T7_PRODUCT_ID          7
#define DIGIT_PRODUCT_ID       200
#define UNUSED_PRODUCT_ID      -1

//UE9 pipes to read/write through
#define UE9_PIPE_EP1_OUT       1
#define UE9_PIPE_EP1_IN        0x81
#define UE9_PIPE_EP2_IN        0x82  //Stream Endpoint

//U3 pipes to read/write through
#define U3_PIPE_EP1_OUT        1
#define U3_PIPE_EP2_IN         0x82
#define U3_PIPE_EP3_IN         0x83  //Stream Endpoint

//U6 pipes to read/write through
#define U6_PIPE_EP1_OUT        1
#define U6_PIPE_EP2_IN         0x82
#define U6_PIPE_EP3_IN         0x83  //Stream Endpoint

//U12 pipes to read/write through
#define U12_PIPE_EP1_IN        0x81
#define U12_PIPE_EP2_OUT       2
#define U12_PIPE_EP0           0    //Control endpoint

//Wireless bridge pipes to read/write through
#define BRIDGE_PIPE_EP1_OUT    1
#define BRIDGE_PIPE_EP2_IN     0x82
#define BRIDGE_PIPE_EP3_IN     0x83  //Spontaneous Endpoint

//T7 pipes to read/write through
#define T7_PIPE_EP1_OUT        1
#define T7_PIPE_EP2_IN         0x82
#define T7_PIPE_EP3_IN         0x83  //Stream Endpoint

//Digit pipes to read/write through
#define DIGIT_PIPE_EP1_OUT        1
#define DIGIT_PIPE_EP2_IN         0x82


#ifdef __cplusplus
extern "C"{
#endif


float LJUSB_GetLibraryVersion(void);
//Returns the labjackusb library version number.

unsigned int LJUSB_GetDevCount(unsigned long ProductID);
// Returns the total number of LabJack USB devices connected.
// ProductID = The product ID of the devices you want to get the count of.

unsigned int LJUSB_GetDevCounts(UINT *productCounts, UINT * productIds, UINT n);
// Returns the count for n products.
// productCounts = Array of size n that holds the count
// productIds = Array of size n which holds the product IDs.
// n = The size of the arrays.
// For example
//   uint productCounts[10], productIds[10];
//   r = LJUSB_GetDevCounts(productCounts, productIds, 10);
// would return arrays that may look like
//   {1, 2, 3, 4, 5, 6, 7, 0, 0, 0}
//   {3, 6, 9, 1, 1000, 7, 200, 0, 0, 0}
// which means there are
//   1 U3
//   2 U6s
//   3 UE9s
//   4 U12s
//   5 SkyMote Bridges
//   6 T7s
//   7 Digits
// connected.

int LJUSB_OpenAllDevices(HANDLE* devHandles, UINT* productIds, UINT maxDevices);
// Opens all LabJack devices up to a maximum of maxDevices.
// devHandles = An array of handles with a size of maxDevices
// productIds = An array of product IDs with a size of maxDevices
// maxDevices = Maximum number of devices to open.
// Returns the number of devices actually opened, or -1 if a tragically bad
// error occurs. The structure of the arrays is similar to that of
// LJUSB_GetDevCounts above. A simple example would be:
//   {2341234, 55343, 0, 0, ...}
//   {3, 1000, 0, 0, ...}
// where the return value is 2. 2341234 is the handle for a U3, and 55343 is the
// handle for a SkyMote Bridge.

HANDLE LJUSB_OpenDevice(UINT DevNum, unsigned int dwReserved, unsigned long ProductID);
// Obtains a handle for a LabJack USB device.  Returns NULL if there is an
// error.
// If the device is already open, NULL is returned and errno is set to EBUSY.
// DevNum = The device number of the LabJack USB device you want to open.  For
//          example, if there is one device connected, set DevNum = 1.  If you
//          have two devices connected, then set DevNum = 1, or DevNum = 2.
// dwReserved = Not used, set to 0.
// ProductID = The product ID of the LabJack USB device.  Currently the U3, U6,
//             and UE9 are supported.

bool LJUSB_ResetConnection(HANDLE hDevice);
// Performs a USB port reset to reinitialize a device.
// Returns true on success, or false on error and errno is set.
// Note that this function is experimental and currently may not work.
// If this function fails, hDevice is no longer valid (you should close it)
// and you should re-open the device.
// hDevice = The handle for your device

unsigned long LJUSB_Write(HANDLE hDevice, const BYTE *pBuff, unsigned long count);
// Writes to a device with a 1 second timeout.  If the timeout time elapses and
// no data is transferred the USB request is aborted and the call returns.
// Returns the number of bytes written, or 0 on error and errno is set.
// hDevice = The handle for your device
// pBuff = The buffer to be written to the device.
// count = The number of bytes to write.
// This function replaces the deprecated LJUSB_BulkWrite, which required the
// endpoint.

unsigned long LJUSB_Read(HANDLE hDevice, BYTE *pBuff, unsigned long count);
// Reads from a device with a 1 second timeout. If the timeout time elapses and
// no data is transferred the USB request is aborted and the call returns.
// Returns the number of bytes read, or 0 on error and errno is set.
// hDevice = The handle for your device
// pBuff = The buffer to be filled in with bytes from the device.
// count = The number of bytes expected to be read.
// This function replaces the deprecated LJUSB_BulkRead, which required the
// endpoint.

unsigned long LJUSB_Stream(HANDLE hDevice, BYTE *pBuff, unsigned long count);
// Reads from a device's stream interface with a 1 second timeout.  If the
// timeout time elapses and no data is transferred the USB request is aborted
// and the call returns.  Returns the number of bytes written, or 0 on error and
// errno is set.
// hDevice = The handle for your device
// pBuff = The buffer to be filled in with bytes from the device.
// count = The number of bytes expected to be read.
// This function replaces the deprecated LJUSB_BulkRead, which required the
// (stream) endpoint.

unsigned long LJUSB_WriteTO(HANDLE hDevice, const BYTE *pBuff, unsigned long count, unsigned int timeout);
// Writes to a device with a specified timeout.  If the timeout time elapses and
// no data is transferred the USB request is aborted and the call returns.
// Returns the number of bytes written, or 0 on error and errno is set.
// hDevice = The handle for your device
// pBuff = The buffer to be written to the device.
// count = The number of bytes to write.
// timeout = The USB communication timeout value in milliseconds.  Pass 0 for
//           an unlimited timeout.

unsigned long LJUSB_ReadTO(HANDLE hDevice, BYTE *pBuff, unsigned long count, unsigned int timeout);
// Reads from a device with a specified timeout. If the timeout time elapses and
// no data is transferred the USB request is aborted and the call returns.
// Returns the number of bytes read, or 0 on error and errno is set.
// hDevice = The handle for your device
// pBuff = The buffer to be filled in with bytes from the device.
// count = The number of bytes expected to be read.
// timeout = The USB communication timeout value in milliseconds.  Pass 0 for
//           an unlimited timeout.

unsigned long LJUSB_StreamTO(HANDLE hDevice, BYTE *pBuff, unsigned long count, unsigned int timeout);
// Reads from a device's stream interface with a specified timeout.  If the
// timeout time elapses and no data is transferred the USB request is aborted
// and the call returns.  Returns the number of bytes read, or 0 on error and
// errno is set.
// hDevice = The handle for your device
// pBuff = The buffer to be filled in with bytes from the device.
// count = The number of bytes expected to be read.
// timeout = The USB communication timeout value in milliseconds.  Pass 0 for
//           an unlimited timeout.

void LJUSB_CloseDevice(HANDLE hDevice);
// Closes the handle of a LabJack USB device.

bool LJUSB_IsHandleValid(HANDLE hDevice);
// Returns true if the handle is valid; this is, it is still connected to a
// device on the system.

unsigned short LJUSB_GetDeviceDescriptorReleaseNumber(HANDLE hDevice);
// Returns the device's release number (binary-coded decimal) stored in the
// device descriptor.
// hDevice = The handle for your device.

unsigned long LJUSB_GetHIDReportDescriptor(HANDLE hDevice, BYTE *pBuff, unsigned long count);
// Reads the HID report descriptor bytes from a device with a 1 second timeout.
// If the timeout time elapses and no data is transferred the USB request is
// aborted and the call returns.  Returns the number of bytes read, or 0 on
// error and errno is set.  Only supports the U12.
// hDevice = The handle for your device.
// pBuff = The buffer to filled in with the bytes of the report descriptor.
// count = The number of bytes expected to be read.


//Note:  For all function errors, use errno to retrieve system error numbers.

/* --------------- DEPRECATED Functions --------------- */

unsigned long LJUSB_BulkRead(HANDLE hDevice, unsigned char endpoint, BYTE *pBuff, unsigned long count);
// Reads from a bulk endpoint.  Returns the count of the number of bytes read,
// or 0 on error (and sets errno).  If there is no response within a certain
// amount of time (LJ_LIBUSB_TIMEOUT in labjackusb.c), the read will timeout.
// hDevice = Handle of the LabJack USB device.
// endpoint = The pipe you want to read your data through (xxx_PIPE_xxx_IN).
// *pBuff = Pointer a buffer that will be read from the device.
// count = The size of the buffer to be read from the device.


unsigned long LJUSB_BulkWrite(HANDLE hDevice, unsigned char endpoint, BYTE *pBuff, unsigned long count);
// Writes to a bulk endpoint.  Returns the count of the number of bytes wrote,
// or 0 on error and sets errno.
// hDevice = Handle of the LabJack USB device.
// endpoint = The pipe you want to write your data through (xxx_PIPE_xxx_OUT).
// *pBuff = Pointer to the buffer that will be written to the device.
// count = The size of the buffer to be written to the device.

bool LJUSB_AbortPipe(HANDLE hDevice, unsigned long Pipe);
// No longer supported and will return false.
// Pipes will timeout after LJ_LIBUSB_TIMEOUT, which is set by default to 1
// second.

#ifdef __cplusplus
}
#endif

#endif // LABJACKUSB_H_
