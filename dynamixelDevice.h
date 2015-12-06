/*--------------------------------------------------------------------
  This file is part of the Dynamixel Device library.

  NeoPixel is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of
  the License, or (at your option) any later version.

  NeoPixel is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with NeoPixel.  If not, see
  <http://www.gnu.org/licenses/>.
  --------------------------------------------------------------------*/

#ifndef DYNAMIXELDEVICE_H
#define DYNAMIXELDEVICE_H

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <Arduino.h>
#include <EEPROM.h>


//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------

#define DDV_USE_DEBUG_PINS
#define DDV_DEBUG_PIN1   9
#define DDV_DEBUG_PIN2  10
#define DDV_DEBUG_PIN3  11
#define DDV_DEBUG_PIN4  12

#ifndef LED_PIN
#define LED_PIN 13
#endif

#ifndef REG_COUNT
#define REG_COUNT 50
#endif

#define PACKET_TIMEOUT_MS 10

#define EEPROMREGLENGTH 23
#define REG_LOCKBIT 47
#define REG_ID 3
#define REG_BAUD 4
#define REG_DELAY 5
#define REG_LED 0x19

enum INSTRUCTION
{
  NONE,
  APING,
  READ,
  WRITE,
  REG_WRITE,
  ACTION,
  RESET,
  SYNC_WRITE=0x83
};

enum DECODE_STEP
{
  DE_HEADER1,
  DE_HEADER2,
  DE_ID,
  DE_LENGTH,
  DE_INSTRUCTION,
  DE_DATA,
  DE_CHECKSUM
};

class RegWriteClass
{
	public:
		byte RegWriteLength;
		byte RegWriteAddress;
};

//------------------------------------------------------------------------------
// Class: DynamixelDevice - Code to emulate a Dynamixel device.
//------------------------------------------------------------------------------

class DynamixelDevice
{
  protected:
  	bool diode_;
    int DecodeIndex;
    byte mID;
    byte mCount;
    byte mLength;
    byte mInstruction;
    byte mData[ 256 ];
    byte mChecksum;
    Stream* pstream_;
    uint8_t serial_in_input_mode_;
    uint32_t last_serial_input_time_;

    byte regWriteCount;
    RegWriteClass regWrites[10];

    void writeValues( byte index, byte len, byte* data );
    void flushInput();
    void processInputPacket( byte instruction, byte* data, int len );


  public:
    byte REG[ REG_COUNT ];
    byte RegWriteData[REG_COUNT];

    virtual int init( Stream* pStream, bool diode = false );
    void processInput( void );

    void setRX();
    void setTX();
    void transmitMessage( byte err, byte* data, byte len );

    // Define some virtual functions that allow some functionality
    // to be sub classed.
    virtual bool postProcessRegisterWrite(int index, int len);
    virtual void preProcessRegisterRead(int index, int len);
    virtual void processSyncWrite(int index, int len, byte* data, int data_len );
    virtual void processReset(void);
    virtual void lookAtOtherDevicePackets(int mID, byte instruction, byte* data, int len);
};

//------------------------------------------------------------------------------
// DynamixelDevice::init - Initialize the object
//------------------------------------------------------------------------------
int DynamixelDevice::init( Stream* pStream, bool diode )
{
  diode_=diode;
  int ChkGood;
  unsigned short chk = 0;
  pstream_ = pStream;
  serial_in_input_mode_ = false;
  last_serial_input_time_ = millis();    // remember when we last received any input...
#ifdef DDV_USE_DEBUG_PINS 
  pinMode(DDV_DEBUG_PIN1, OUTPUT);
  pinMode(DDV_DEBUG_PIN2, OUTPUT);
  pinMode(DDV_DEBUG_PIN3, OUTPUT);
  pinMode(DDV_DEBUG_PIN4, OUTPUT);
#endif  
  setRX();  // start off in input mode.
  regWriteCount=0;
 
  memset(REG, 0, sizeof(REG));    // clear out all registers to start
  for ( int i = 0; i < EEPROMREGLENGTH; ++i )
  {
    byte value = EEPROM.read( i );
    REG[ i ] = value;
    chk += value;
  }


  // Just to make sure we will assume that if all of the bytes we read was 0 or 0xff
  // then assume unused EEPROM so go to chkbad.
    
  
  //changed from:
  if ( (chk != 0) && (chk != (EEPROMREGLENGTH * 0xff)) && (EEPROM.read( EEPROMREGLENGTH ) == ( chk & 0xff ) + ( chk >> 8 ) ) )
  
  //if ( (chk != 0) && ( chk != ( ( ( EEPROMREGLENGTH * 0xff ) & 0xff ) + ( ( EEPROMREGLENGTH * 0xff) >> 8 ) ) ) && 
  //			(EEPROM.read( EEPROMREGLENGTH ) == ( chk & 0xff ) + ( chk >> 8 ) ) )
  {
    ChkGood = 1;
  }
  else
  {
    ChkGood = 0;
    memset(REG, 0, EEPROMREGLENGTH);
		REG[ REG_BAUD ] = 1;
    REG[ REG_ID ] = 1;
    REG[ REG_DELAY ] = 250;			//added
  }

  //int baud = (2000000 / ( REG[ REG_BAUD ] + 1 ));
	
	if(!diode_)
	{
		  // If this is for a Teensy, then we need to configure the serial port
		  #if defined(__MK20DX256__) || defined(__MKL26Z64__)
		  if (pstream_ == &Serial1)
		  {
		  	UART0_C1 |= UART_C1_LOOPS | UART_C1_RSRC;
		  }
		  else if (pstream_ == &Serial2)
		  {
		    UART1_C1 |= UART_C1_LOOPS | UART_C1_RSRC;
		  }
		  else if (pstream_ == &Serial3)
		  {
		  	UART2_C1 |= UART_C1_LOOPS | UART_C1_RSRC;
		  }
			#endif
	}
	
  DecodeIndex = DE_HEADER1;
  return ChkGood;
}

//------------------------------------------------------------------------------
// DynamixelDevice::processInput - Loops through reading and processing all
//   pending input from the serial device.  The function will make sure
//   that our half duplex serial connection is set to input
//------------------------------------------------------------------------------
void DynamixelDevice::processInput( void )
{
  // Make sure we are in input mode
  setRX();
  
  uint8_t first_char = 1;

  int ichr;
#ifdef DDV_USE_DEBUG_PINS 
    digitalWrite(DDV_DEBUG_PIN4, !digitalRead(DDV_DEBUG_PIN4));
#endif
  // loop while we have input available
  while ((ichr = pstream_->read()) != -1)
  {
#ifdef DDV_USE_DEBUG_PINS 
    digitalWrite(DDV_DEBUG_PIN3, !digitalRead(DDV_DEBUG_PIN3));
#endif
    
    if (first_char)
    {
        // If too much time has elapsed, assume we should just wait until we have a new full packet
        if ((millis() - last_serial_input_time_) > PACKET_TIMEOUT_MS)
            DecodeIndex = DE_HEADER1;
        first_char = 0;
    }
    byte input = ichr;

    switch ( DecodeIndex )
    {
      case DE_HEADER1:
        if ( input == 0xFF ) DecodeIndex = DE_HEADER2;
        break;
      case DE_HEADER2:
        if ( input == 0xFF )
        {
          DecodeIndex = DE_ID;
          mChecksum = 1;
        }
        else DecodeIndex = DE_HEADER1;
        break;
      case DE_ID:
        if ( input != 0xFF )    // we are not allowed 3 0xff's in a row, ie. id != 0xff
        {   
            mID = input;
            DecodeIndex = DE_LENGTH;
        }    
        break;
      case DE_LENGTH:
        mLength = input;
        DecodeIndex = DE_INSTRUCTION;
        break;
      case DE_INSTRUCTION:
        mInstruction = input;
        mCount = 0;
        DecodeIndex = DE_DATA;
        if ( mLength == 2 ) DecodeIndex = DE_CHECKSUM;
        break;
      case DE_DATA:
        mData[ mCount++ ] = input;
        if ( mCount >= mLength - 2 )
        {
          DecodeIndex = DE_CHECKSUM;
        }
        break;
      case DE_CHECKSUM:
        {
          DecodeIndex = DE_HEADER1;
          if (  ( ~mChecksum & 0xff ) == ( input & 0xff ) ) 
          {
            // Checksums match now see if it is for us...
            if ( ( mID == REG[ REG_ID ] || mID == 0xFE ) )
            {
#ifdef DDV_USE_DEBUG_PINS 
              digitalWrite(LED_PIN, !digitalRead(LED_PIN));
#endif 
              processInputPacket( mInstruction, mData, mLength - 2 );
            }
            else 
            {
                // Allow us to spy on other packets
                lookAtOtherDevicePackets(mID, mInstruction, mData, mLength - 2);
            }
          }
        }
        break;
    }
    mChecksum += input;
  }
  if (!first_char)
    last_serial_input_time_ = millis(); 
}


//------------------------------------------------------------------------------
// DynamixelDevice::processInputPacket - Process a complete packet that was
//    received from the Serial input for our device.
//------------------------------------------------------------------------------
void DynamixelDevice::processInputPacket( byte instruction, byte* data, int len )
{
#ifdef DDV_USE_DEBUG_PINS 
  digitalWrite(DDV_DEBUG_PIN2, !digitalRead(DDV_DEBUG_PIN2));
#endif  
  switch ( instruction )
  {
    case APING:
      {
        transmitMessage( 0, 0, 0 );
      }
      break;
    case READ:
      {
        byte index = data[ 0 ];
        byte datalen = data[ 1 ];
        preProcessRegisterRead(index, datalen);
        transmitMessage( 0, &REG[ index ], datalen );
      }
      break;
    case WRITE:
      {
        byte index = data[ 0 ];
        byte length = len - 1;

        transmitMessage( 0, 0, 0 ); // Start sending response as quick as possible
        writeValues(index, length, data + 1 );
        postProcessRegisterWrite(index, length);
      }
      break;
    case REG_WRITE:
      {
      	regWrites[regWriteCount].RegWriteAddress = data[ 0 ];
        regWrites[regWriteCount].RegWriteLength = len - 1;

        for ( int i = 0; i < regWrites[regWriteCount].RegWriteLength; ++i )
        {
          RegWriteData[ i ] = data[ i + 1 ];
        }
        regWriteCount++;
      }
      break;
    case ACTION:
      {
        if ( regWriteCount > 0 )
        {
        	for( int i = 0; i < regWriteCount; ++i )
        	{
	          writeValues( regWrites[i].RegWriteAddress, regWrites[i].RegWriteLength, RegWriteData );
	          postProcessRegisterWrite(regWrites[i].RegWriteAddress, regWrites[i].RegWriteLength);
        	}
        	regWriteCount = 0;
        }
      }
      break;
    case RESET:
      processReset();   
      break;
    case SYNC_WRITE:
      {
        byte index = data[ 0 ];
        byte datalen = data[ 1 ];
        processSyncWrite(index, datalen, data + 2, len - 3);            
      }
      break;
  }  
}

//==============================================================================
// Virtual functions that sub classes can replace and/or augment
//==============================================================================
void DynamixelDevice::processSyncWrite(int index, int len, byte* data, int data_len )
{
  // Default implementation is to look through the message to see if there
  // is a section of data for us
  for (int i = 0; i < data_len; i += len) 
  {
    if ( data[i] == REG[ REG_ID ] ) 
    {
      // Found a section for this object
      writeValues(index, len, &data[i + 1] );
      postProcessRegisterWrite(index, len);
      return; // assume only one item should match
    }
  }
}

//------------------------------------------------------------------------------
// DynamixelDevice::postProcessRegisterWrite - Update anything that we need to
//    after we receive a write to our registers. 
//------------------------------------------------------------------------------
bool DynamixelDevice::postProcessRegisterWrite(int index, int len)
{
  if ( (REG_LED >= index) && (REG_LED < (index+len)))
    digitalWrite( LED_PIN, REG[ REG_LED ] );
  return true; 
}

//------------------------------------------------------------------------------
// DynamixelDevice::preProcessRegisterRead - See if we need to get any
//    hardware states information put into logical registers, before
//    returning data to host
//------------------------------------------------------------------------------
void DynamixelDevice::preProcessRegisterRead(int index __attribute__((unused)), int len __attribute__((unused)))
{
}

//------------------------------------------------------------------------------
// DynamixelDevice::processReset - Handle the reset message. 
//------------------------------------------------------------------------------
void DynamixelDevice::processReset()
{
    // Try rebooting the processor 
    //if (mID != 0xFE)
        //asm volatile ("  jmp 0");
}

//------------------------------------------------------------------------------
// DynamixelDevice::lookAtOtherDevicePackets - Look at packets that are not
//  meant for us.  Allows subclasses to do things like turn on Neo pixels 
//  for things that are happening. 
//------------------------------------------------------------------------------
void DynamixelDevice::lookAtOtherDevicePackets(int mID __attribute__((unused)), 
    byte instruction __attribute__((unused)), byte* data __attribute__((unused)), 
    int len __attribute__((unused)))
{
  
}


//------------------------------------------------------------------------------
// DynamixelDevice::setRX - Set the Serial stream to input mode
//------------------------------------------------------------------------------
void DynamixelDevice::setRX()
{
  // There are times when the Trinket stops responding, so maybe always do the underlying code to 
  // to make sure that we are in RX Mode.    
  // Only do this if we are not in this mode now
  if (!diode_)
  {
    if (!serial_in_input_mode_)
    {
      // Wait until all TX bytes have been output - Only do this when think we should
      pstream_->flush();

      flushInput();  // remove any garbage in the input queue
    }
    
#if defined(__MK20DX256__)  || defined(__MKL26Z64__)
    // Teensy 3.1
    if (pstream_ == (Stream*)&Serial1)
      UART0_C3 &= ~UART_C3_TXDIR;
    if (pstream_ == (Stream*)&Serial2)
      UART1_C3 &= ~UART_C3_TXDIR;
    if (pstream_ == (Stream*)&Serial3)
      UART2_C3 &= ~UART_C3_TXDIR;
#else
    if (pstream_ == (Stream*)&Serial)
      UCSR0B = ((1 << RXCIE0) | (1 << RXEN0));
#ifdef UCSR1B
    if (pstream_ == (Stream*)&Serial1)
      UCSR1B = ((1 << RXCIE1) | (1 << RXEN1));
#endif
#ifdef UCSR2B
    if (pstream_ == (Stream*)&Serial2)
      UCSR2B = ((1 << RXCIE2) | (1 << RXEN2);
#endif
#ifdef UCSR3B
    if (pstream_ == (Stream*)&Serial3)
      UCSR3B = ((1 << RXCIE3) | (1 << RXEN3));
#endif
#endif
  }

  if (!serial_in_input_mode_)
  {
    serial_in_input_mode_ = true;
#ifdef DDV_USE_DEBUG_PINS 
    digitalWrite(DDV_DEBUG_PIN1, LOW);
#endif
  }
}


//------------------------------------------------------------------------------
// DynamixelDevice::setTX - Set the device stream to output
//------------------------------------------------------------------------------
void DynamixelDevice::setTX()
{

  // Only do this if we are not in this mode now
  if (!serial_in_input_mode_)
    return;
  serial_in_input_mode_ = false;

  if (!diode_)
  {
#if defined(__MK20DX256__)  || defined(__MKL26Z64__)
    // Teensy 3.1, 3.2, LC
    if (pstream_ == (Stream*)&Serial1)
      UART0_C3 |= UART_C3_TXDIR;
    if (pstream_ == (Stream*)&Serial2)
      UART1_C3 |= UART_C3_TXDIR;
    if (pstream_ == (Stream*)&Serial3)
      UART2_C3 |= UART_C3_TXDIR;
#else
    if (pstream_ == (Stream*)&Serial)
      UCSR0B = /*(1 << UDRIE1) |*/ (1 << TXEN0);
#ifdef UCSR1B
    if (pstream_ == (Stream*)&Serial1)
      UCSR1B = /*(1 << UDRIE1) |*/ (1 << TXEN1);
#endif
#ifdef UCSR2B
    if (pstream_ == (Stream*)&Serial2)
      UCSR2B = /*(1 << UDRIE3) |*/ (1 << TXEN2);
#endif
#ifdef UCSR3B
    if (pstream_ == (Stream*)&Serial3)
      UCSR3B =  /*(1 << UDRIE3) |*/ (1 << TXEN3);
#endif
#endif
  }
#ifdef DDV_USE_DEBUG_PINS 
  digitalWrite(DDV_DEBUG_PIN1, HIGH);
#endif
}

//------------------------------------------------------------------------------
// DynamixelDevice::flushInput - throw away any pending data from the stream
//    input queue
//------------------------------------------------------------------------------
void DynamixelDevice::flushInput()
{
  while (pstream_->read() != -1)
    ;
}

//------------------------------------------------------------------------------
// DynamixelDevice::writeValues - Update our internal registers and for those
//    that are in the EEPROM region write those values out to EEPROM...
//------------------------------------------------------------------------------
void DynamixelDevice::writeValues( byte index, byte len, byte* data )
{
  for ( int i = 0; i < len; ++i )
  {
    REG[ index + i ] = data[ i ];
  }

  if ( index < EEPROMREGLENGTH  )
  {
    unsigned short chk = 0; //change: needs consistancy with checksum function from begining
    for ( int i = 0; i < EEPROMREGLENGTH; ++i )
    {
      chk += REG[ i ];
      EEPROM.write( i, REG[ i ] & 0xff );
    }

    EEPROM.write( EEPROMREGLENGTH, ( chk & 0xff ) + ( chk >> 8 ) );
  }
}

//------------------------------------------------------------------------------
// DynamixelDevice::transmitMessage - Build a return packet to send back to
//    controller.
//------------------------------------------------------------------------------
void DynamixelDevice::transmitMessage( byte err, byte* data, byte len )
{
  if ( mID == 0xFE ) return;
  byte txmsg[ 1024 ];

  txmsg[ 0 ] = txmsg[ 1 ] = 0xFF;
  txmsg[ 2 ] = REG[ REG_ID ];
  txmsg[ 3 ] = len + 2;
  txmsg[ 4 ] = err;

  int i = 0;
  for ( i = 0; i < len; ++i )
  {
    txmsg[ 5 + i ] = data[ i ];
  }

  byte checksum = 0;
  for ( i = 2; i < 5 + len; ++i )
  {
    checksum += txmsg[ i ];
  }
  checksum = ~checksum;
  txmsg[ 5 + len ] = checksum;

	//delayMicroseconds( REG[ REG_DELAY ] * 2 );
  setTX();  // Set the Serial port into output mode
  pstream_->write( txmsg, 5 + len + 1 );
  setRX();  // restore
}


#endif
