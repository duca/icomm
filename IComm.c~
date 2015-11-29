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

// Debug
#ifdef JN5139
#include "..\..\..\Chip\Common\Include\Printf.h"
#endif

#include "IComm.h"
#include <stdio.h>


/***************************/
/*** Private definitions ***/

#define OFFSET_CMD   2
#define OFFSET_ID    3
#define OFFSET_SIZE  4
#define OFFSET_DATA  5



/*************************/
/*** Private Functions ***/

// Unconditionally fills '_packOut' with given data.
// Also adjusts consumption pointers.
PRIVATE void fillDataOut( IComm *ic, uint8 *data, uint8 size );

// Unconditionally read data from '_packIn'.
PRIVATE uint8 readDataIn( IComm *ic, uint8 *data );

// Informs the arrival at the peer of the package with the given id.
// 'cmd' indicates the acknowledge type: ACK or NACK.
PRIVATE void packageConfirmed( IComm *ic, uint8 cmd, uint8 id );

// Send an ACK/NACK for the current packIn package ID and discards package.
PRIVATE void sendAck( IComm *ic, uint8 cmd );

// When '_packIn' is full, IComm enters at the state PACK_ONLY_ACK and
// accepts only acknowledges packages. This function is used at that
// state to fill the '_confIn' structure.
// Returns TRUE if completes the confirmation package.
//
PRIVATE Bool fillAck( IComm *ic, uint8 byte );



/************************************************************************/
/************************ Internal Communicator *************************/

/*****************/
/*** icom init ***/
//
PUBLIC void icomm_init( IComm *ic, uint8 maxResend,
                        uint32 timeout, uint32 timeoutAck )
{
  ic->packIn.state = PACK_EMPTY;
  ic->packIn.head  = ic->packIn.tail = ic->packIn.pack;

  ic->packOut.state = PACK_EMPTY;
  ic->packOut.head  = ic->packOut.tail = ic->packOut.pack;

  ic->confIn.state = CONF_EMPTY;
  ic->confIn.head  = ic->confIn.pack;
  ic->confIn.tail  = ic->confIn.pack + 4;

  ic->confOut.state = CONF_EMPTY;
  ic->confOut.head  = ic->confOut.pack;
  ic->confOut.tail  = ic->confOut.pack + 4;

  ic->sendCallback = 0;
  ic->recvCallback = 0;

  ic->maxResend   = maxResend;
  ic->resendCount = 0;

  ic->timeout = timeout;
  ic->counter = 0;

  ic->timeoutAck = timeoutAck;
  ic->counterAck = 0;

  ic->lastReceivedId = 0;
}


/*****************/
/*** icom send ***/
//
PUBLIC Bool icomm_send( IComm *ic, uint8 *data, uint8 size )
{
  // No space to send.
  if ( ! icomm_canSend( ic ) )
  {
    icomm_timeout_ack( ic );
    return FALSE;
  }

  fillDataOut( ic, data, size );
  ic->packOut.state = PACK_FULL;
  ic->counterAck = 0;

  return TRUE;
}


/*****************/
/*** icom recv ***/
//
PUBLIC uint8 icomm_recv( IComm *ic, uint8 *data )
{
  // No data ready to be read.
  if ( ! icomm_canRecv( ic ) )
  {
    // Check for timeout.
    icomm_timeout( ic );
    return 0;
  }

  // Send ackonwledge and discards package.
  sendAck( ic, ACK );

  // If this package is already received, ignores it.
  if ( ic->lastReceivedId == ic->packIn.pack[OFFSET_ID] )
    return 0;

  // A package can be received twice if its first ACK was note received
  // by the peer. Then the package is resent, but it has already been
  // correctly received.
  ic->lastReceivedId = ic->packIn.pack[OFFSET_ID];

  // Read package data.
  return readDataIn( ic, data );
}


/********************/
/*** icom timeout ***/
//
PUBLIC uint8 icomm_timeout( IComm *ic )
{
  // Discards arrived data if this function was called for
  // 'ic->timeout' times with no bytes arriving.
  if ( ic->timeout && (ic->packIn.head != ic->packIn.tail) &&
       (++ic->counter >= ic->timeout) )
  {
    sendAck( ic, NACK );
    ic->counter = 0;
    return TRUE;
  }

  return FALSE;
}


/************************/
/*** icom timeout ack ***/
//
PUBLIC uint8 icomm_timeout_ack( IComm *ic )
{
  // Check for confirmation timeout.
  if ( ++ic->counterAck >= ic->timeoutAck )
  {
    printf( "icomm_timeout_ack()\n" );

    // Confirmation timeout. Acts like received a NACK.
    packageConfirmed( ic, NACK, ic->packOut.pack[OFFSET_ID] );
    ic->counterAck = 0;
    return TRUE;
  }

  return FALSE;
}


/*******************************/
/*** icomm set Send Callback ***/
//
PUBLIC void icomm_setSendCallback( IComm *ic, SendCallback callback )
{
  ic->sendCallback = callback;
}


/*******************************/
/*** icomm set Recv Callback ***/
//
PUBLIC void icomm_setRecvCallback( IComm *ic, RecvCallback callback )
{
  ic->recvCallback = callback;
}


/************************/
/*** icom get Tx Byte ***/
//
PUBLIC int16 icomm_getTxByte( IComm *ic )
{
  // If it is in the middle of a confirmation transmition.
  if ( ic->confOut.state == CONF_SENDING )
  {
    if ( ic->confOut.head < ic->confOut.tail )
    {
      return *ic->confOut.head++;
    }

    // Finished to send confirmation.
    ic->confOut.head  = ic->confOut.pack;
    ic->confOut.state = CONF_EMPTY;
  }

  // If it is in the middle of a package transmition.
  if ( ic->packOut.state == PACK_SENDING )
  {
    // There still has data to read.
    if ( ic->packOut.head < ic->packOut.tail )
      return *ic->packOut.head++;

    // All data had already been read.
    // Before freeing the buffer it needs to wait for package
    // delivery confirmation (acknowledge).
    ic->packOut.state = PACK_WAIT_ACK;
  }


  // If there is a pending Ack to send, start to send it.
  if ( ic->confOut.state == CONF_FULL )
  {
    ic->confOut.state = CONF_SENDING;
    return *ic->confOut.head++;
  }


  // If there is a pending package to send, start to send it.
  if ( ic->packOut.state == PACK_FULL )
  {
    ic->packOut.state = PACK_SENDING;
    return *ic->packOut.head++;
  }

  return -1;
}


/************************/
/*** icom put Rx Byte ***/
//
PUBLIC Bool icomm_putRxByte( IComm *ic, uint8 byte )
{
  uint8 cmd;

  // Restores timeout counter.
  ic->counter = 0;

  switch ( ic->packIn.state )
  {
  case PACK_FULL:
    ic->packIn.state = PACK_ONLY_ACK;

  case PACK_ONLY_ACK:
    if ( fillAck( ic, byte ) )
    {
      // Confirmation is received. Now let the client read the package
      // in the incoming package buffer.
      ic->packIn.state = PACK_FULL;
    }
    break;

  case PACK_EMPTY:  // Waiting for both synchronism characters.
    if ( *ic->packIn.head == SYNC && byte == SYNC )
    {
      *ic->packIn.head++ = 0;  // The first SYNC is zeroed for future usage.
      *ic->packIn.head++ = SYNC;
      ic->packIn.state = PACK_SYNC;
    }
    else
    {
      *ic->packIn.head = byte;
    }
    break;


  case PACK_SYNC: // Fills 'cmd'.
    *ic->packIn.head++  = byte;
    ic->packIn.checksum = 0;
    ic->packIn.state    = PACK_CMD;
    break;


  case PACK_CMD: // Fills 'id'.
    ic->packIn.checksum += (*ic->packIn.head++ = byte);

    switch ( cmd = ic->packIn.pack[OFFSET_CMD] )
    {
    case STX:
      ic->packIn.state = PACK_ID;
      break;

    case ACK:
    case NACK:
      packageConfirmed( ic, cmd, byte );  // 'byte' is the package ID!

    default:  // Unknown command.
      ic->packIn.state = PACK_EMPTY;
      ic->packIn.head = ic->packIn.tail = ic->packIn.pack;
    }
    break;


  case PACK_ID: // Fills 'size'.
    ic->packIn.checksum += (*ic->packIn.head++ = byte);
    ic->packIn.tail      = ic->packIn.head + byte;
    ic->packIn.state     = PACK_DATA;
    break;


  case PACK_DATA: // Fills all package data.
    ic->packIn.checksum += (*ic->packIn.head++ = byte);

    if ( ic->packIn.head == ic->packIn.tail )
      ic->packIn.state = PACK_CHECKSUM;
    break;


  case PACK_CHECKSUM: // Fills 'checksum'.
    if ( ic->packIn.checksum == (*ic->packIn.head++ = byte) )
    {
      ic->packIn.state = PACK_EOT;
    }
    else
    {
      // Send negative acknowledge and ignores package.
      sendAck( ic, NACK );
    }
    break;


  case PACK_EOT: // Fills 'EOT'.
    *ic->packIn.head = byte;
    if ( byte == EOT )
    {
      // Wait 'icomm_recv()' be called before send an ACK.
      ic->packIn.state = PACK_FULL;

      // Signals user about the arrived package.
      if ( ic->recvCallback )
        (*ic->recvCallback)( ic );
    }
    else
    {
      // Send negative acknowledge and ignores package.
      sendAck( ic, NACK );
    }
    break;


  default:

    // Send negative acknowledge and ignores package.
    sendAck( ic, NACK );
    return FALSE;
  }

  return TRUE;
}


/*********************/
/*** fill Data Out ***/
//
PRIVATE void fillDataOut( IComm *ic, uint8 *data, uint8 size )
{
  static uint8 pckId = 1;

  uint8 checksum = 0;
  uint8 *pack;
  uint8 *end;

  pack = ic->packOut.pack;

  // Header
  *pack++ = SYNC;
  *pack++ = SYNC;
  *pack++ = STX;
  checksum += (*pack++ = pckId++);
  checksum += (*pack++ = size);

  // Does not use package ID == 0.
  if ( ! pckId )
  {
    pckId = 1;
  }

  // Data.
  end = pack + size;
  while ( pack < end )
  {
    checksum += (*pack++ = *data++);
  }

  // Tail.
  *pack++ = checksum;
  *pack++ = EOT;

  // Consumption pointers.
  ic->packOut.head = ic->packOut.pack;
  ic->packOut.tail = pack;
}


/********************/
/*** read Data In ***/
//
PRIVATE uint8 readDataIn( IComm *ic, uint8 *data )
{
  uint8 size  = ic->packIn.pack[OFFSET_SIZE];
  uint8 *pack = ic->packIn.pack + OFFSET_DATA;
  uint8 *end  = pack + size;

  while ( pack < end )
  {
    *data++ = *pack++;
  }

  ic->packIn.head = ic->packIn.tail = ic->packIn.pack;
  return size;
}


/*************************/
/*** package Confirmed ***/
//
PRIVATE void packageConfirmed( IComm *ic, uint8 cmd, uint8 id )
{
  uint8 canResend;
  uint8 packId = ic->packOut.pack[OFFSET_ID];

  // Only full packages with the given ID can be confirmed.
  // Otherwise the confirmation is ignored.
  if ( ic->packOut.state != PACK_WAIT_ACK || packId != id )
    return;

  ic->packOut.head = ic->packOut.pack;
  canResend = (ic->resendCount++ < ic->maxResend);

  // Package was sent or cannot be sent anymore.
  if ( cmd == ACK || ! canResend )
  {
    ic->resendCount = 0;
    ic->packOut.state = PACK_EMPTY;

    // Signals user the package was sent or can not be sent anymore.
    if ( ic->sendCallback )
      (*ic->sendCallback)( ic, packId, canResend );
  }
  else
  {
    // Package will be resent.
    ic->packOut.state = PACK_FULL;
  }
}


/****************/
/*** send Ack ***/
//
PRIVATE void sendAck( IComm *ic, uint8 cmd )
{
  uint8 *conf = ic->confOut.head = ic->confOut.pack;

  *conf++ = SYNC;
  *conf++ = SYNC;
  *conf++ = cmd;
  *conf   = ic->packIn.pack[OFFSET_ID];

  ic->confOut.state = CONF_FULL;

  // Empty incomming buffer.
  ic->packIn.state = PACK_EMPTY;
  ic->packIn.head  = ic->packIn.tail = ic->packIn.pack;
}


/****************/
/*** fill Ack ***/
//
PRIVATE Bool fillAck( IComm *ic, uint8 byte )
{
  switch ( ic->confIn.state )
  {
  case CONF_EMPTY:  // Waiting for both synchronism characters.
    if ( *ic->confIn.head == SYNC && byte == SYNC )
    {
      // The first SYNC is zeroed for future usage.
      *ic->confIn.head++ = 0;
      *ic->confIn.head++ = SYNC;
      ic->confIn.state   = CONF_SYNC;
    }
    else
      *ic->confIn.head = byte;
    break;

  case CONF_SYNC: // Fills 'cmd'.
    *ic->confIn.head++ = byte;

    // Only ACK/NACK allowed.
    if ( byte == ACK || byte == NACK )
    {
      ic->confIn.state = CONF_CMD;
    }
    else
    {
      // Data packages are ignored.
      ic->confIn.state = CONF_EMPTY;
      ic->confIn.head  = ic->confIn.pack;
    }
    break;

  case CONF_CMD: // Fills 'id'. 'byte' is the package ID!
    *ic->confIn.head++ = byte;
    packageConfirmed( ic, ic->confIn.pack[OFFSET_CMD], byte );

  default:
    ic->confIn.state = CONF_EMPTY;
    ic->confIn.head  = ic->confIn.pack;
    return TRUE;
  }

  return FALSE;
}
