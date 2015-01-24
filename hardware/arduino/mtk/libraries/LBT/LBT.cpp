#include <string.h>
#include "LBT.h"
LBTRingBuffer::LBTRingBuffer( void )
{
    memset( _aucBuffer, 0, LBT_SERIAL_BUFFER_SIZE ) ;
    _iHead=0 ;
    _iTail=0 ;
}

void LBTRingBuffer::store_char( uint8_t c )
{
  int i = (uint32_t)(_iHead + 1) % LBT_SERIAL_BUFFER_SIZE ;

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if ( i != _iTail )
  {
    _aucBuffer[_iHead] = c ;
    _iHead = i ;
  }
}
