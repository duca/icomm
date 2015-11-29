/*
 * Author: Mauro Enrique de Souza Muñoz (mesmunoz@gmail.com)
 *
 * Date: 17/02/2009 (dd/mm/aaaa)
 *
 * Library for RobotDeck internal boards communication using UARTs.
 *
 * Version: 2.1.0
 * 
 */

#ifndef _IComm_
#define _IComm_

#if defined __cplusplus
extern "C" {
#endif

#include <Defs.h>


/************************************************************************/
/************************ Internal Communicator *************************/

/*************/
/*** Types ***/

/** Package States **/
//
typedef enum
{
  // Empty package.
  PACK_EMPTY,

  // Filling states.
  PACK_SYNC,     // Filled with 2 synchronism characters.
  PACK_CMD,      // Filled with package command.
  PACK_ID,       // Filled with package identificator.
  PACK_DATA,     // Filling package data.
  PACK_CHECKSUM, // Waiting for checksum.
  PACK_EOT,      // Waiting for EOT character.
  PACK_ONLY_ACK, // Buffer is full. Accepting only acknowledge packs.

  // Package complete.
  PACK_FULL,
  PACK_SENDING,
  PACK_WAIT_ACK, // Waiting for package delivery confirmation.

  // Package status.
  PACK_ERROR  // Integrity verification failed.

} PackState;


/** Acknowledge Package States **/
//
typedef enum
{
  CONF_EMPTY, // Empty package.
  CONF_SYNC,  // Filled with 2 synchronism characters.
  CONF_CMD,   // Filled with package command.
  CONF_ID,    // Filled with package identificator.
  CONF_FULL,
  CONF_SENDING
} ConfState;


/** Package structure **/

/*
 * The 'pack' field of the Package structure is code-organized as:
 *
 * + Data packages:
 * 
 *     [SYNC, SYNC, STX, id, size, D1, ..., Dsize, checksum, EOT]
 *     
 *   where:
 *     SYNC    : synchronism character constant;
 *     STX     : 'cmd' = STX (this is a data package).
 *     id      : package identificator in [1, 255];
 *     size    : number of data bytes in the range [0, 255];
 *     Di      : i-th data byte;
 *     checksum: transmited checksum.
 *     EOT     : package end character constant.
 *
 *     
 * + Acknowledge package:
 * 
 *     [SYNC, SYNC, ACK, id]
 *     
 *   where:
 *     id: identificator of the package this acknowledge refers to.
 *
 *     
 * + Negative acknowledge package:
 * 
 *     [SYNC, SYNC, NACK, id]
 *     
 *   where:
 *     id: identificator of the package this acknowledge refers to.
 */
typedef struct
{
  uint8     pack[302];
  uint8    *head;
  uint8    *tail;
  uint8     checksum;   // Package calculated checksum.
  PackState state;
} Package;


/** Confirmation package structure **/

typedef struct
{
  uint8     pack[4];
  uint8    *head;
  uint8    *tail;
  ConfState state;
} ConfPackage;


/** Callback function types **/

/*
 * Called when IComm is ready to send a package. The status of the last
 * submitted package is given as parameters. 'id' is the package
 * identificator and 'sent' is not null if the package could not be
 * sent.
 */
typedef void (*SendCallback)( void *icomm, uint8 id, uint8 sent );
typedef void (*RecvCallback)( void *icomm );


/** Internal Communicator structure **/

typedef struct
{
  Package packIn;   // Stores received data.
  Package packOut;  // Stores data to transmit.

  ConfPackage confIn;  // Stores ACK/NACK packages when _packIn is full.
  ConfPackage confOut; // Stores ACK/NACK packages for transmition.

  SendCallback sendCallback;
  RecvCallback recvCallback;

  uint8 resendCount;
  uint8 maxResend;

  uint32 timeout;  // packIn timeout.
  uint32 counter;

  uint32 timeoutAck;  // Acknowledge reception timeout.
  uint32 counterAck;

  uint8 lastReceivedId;  // ID of the last successfull received package.

} IComm;


/*****************/
/*** Functions ***/

/*
 * Client interface
 */

// Returns 'TRUE' if data can be sent through the given 'ic'.
#define icomm_canSend(ic)  ((ic)->packOut.state == PACK_EMPTY)

// Returns 'TRUE' if there is data to be gotten through the given 'ic'.
#define icomm_canRecv(ic)  ((ic)->packIn.state == PACK_FULL)
  

// Initialize the IComm communication interface.
// 
// maxResend:
//   maximum number a package can be resent;
// timeout:
//   if a package is not entirely arrived, after calling icomm_recv()
//   for 'timeout' times with no data inserted (icomm_putRxByte()),
//   data already read is discarded.
//   If 'timeout' == 0, does not use timeout. In this case IComm can lock!
// timeoutAck:
//   After sending a package, IComm waits for a confirmation ACK/NACK before
//   discard the package. If a NACK arrives, package is resent. If an ACK
//   arrives, package is discarded an user can send other package.
//   If icomm_send() is called for 'timeoutAck' times before the confirmation
//   arrives, the package is resent.
//
PUBLIC void icomm_init( IComm *ic, uint8 maxResend,
                        uint32 timeout, uint32 timeoutAck );

// Insert a package for transmition.
// Returns 'false' if sending buffer is full.
PUBLIC Bool icomm_send( IComm *ic, uint8 *data, uint8 size );

// Fills 'data' with pending received data and returns the number
// of bytes filled. If there is no pending data returns 0.
// Check for timeout.
PUBLIC uint8 icomm_recv( IComm *ic, uint8 *data );

// Check for timeout.
// If there is a package to send decrements timeout counter.
// If timeout expires, sends NACK, removes current package and returns TRUE.
// Returns FALSE otherwise.
PUBLIC uint8 icomm_timeout( IComm *ic );

// Increments acknowledge timeout.
// If timeout expires, sends NACK, removes current package and returns TRUE.
// Returns FALSE otherwise.
PUBLIC uint8 icomm_timeout_ack( IComm *ic );


// If set, when IComm is ready to send a package the given callback
// function is called.
PUBLIC void icomm_setSendCallback( IComm *ic, SendCallback callback );

// If set, when IComm has a package ready to be read the given
// callback function is called.
PUBLIC void icomm_setRecvCallback( IComm *ic, RecvCallback callback );


/*
 * UART interface
 *
 * The communicator does not know UART's software library.
 * The following functions define an interface to be used for data
 * communication between UART an communicator.
 * These functions are designed to be fast, so they can be used in
 * response to interrupt requests.
 * 
 */

// When UART is ready to send data, it must get data from communicator
// using this function and transmit the data through UART.
// Returns -1 if there is no data.
// 
PUBLIC int16 icomm_getTxByte( IComm *ic );

// When data arrives through UART, it must be inserted in communicator
// using this function. The communicator organizes data as packages.
PUBLIC Bool icomm_putRxByte( IComm *ic, uint8 byte );


#if defined __cplusplus
}
#endif

#endif // _IComm_
