
//------------------------------------------------------------------------------
// Kurt's Neo Pixel Servo code which is only slightly modified
// from Renee's LEDHand program.
//
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// Include Files
//------------------------------------------------------------------------------
#include <Adafruit_NeoPixel.h>

#include <EEPROM.h>
#define REG_COUNT 0x40
#include <dynamixelDevice.h>
#include "Other_Defines.h"
  
//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------

#define NEO_PIN 6

#define NEO_COUNT 1 // simple version
#define NEO_TYPE NEO_GRB // Through hole or RGB others GRB
//#define NEO_COUNT 7 // currently setup for ring

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)


enum RGB_MODES
{
  MANUAL,
  FADE,
  FLASH
};

enum RGB_REGISTERS
{
  // First some that we may monitor of other servos
  AX_LED = 25,
  AX_GOAL_POSITION_L = 30,
  AX_GOAL_POSITION_H,
  AX_GOAL_SPEED_L,
  AX_GOAL_SPEED_H,
  AX_PRESENT_POSITION_L,
  AX_PRESENT_POSITION_H,
  AX_PRESENT_SPEED_L,
  AX_PRESENT_SPEED_H,
  AX_PRESENT_LOAD_L,
  AX_PRESENT_LOAD_H,
  AX_PRESENT_VOLTAGE,
  AX_PRESENT_TEMPERATURE,
  
  REG_R = 0x30,   // Moved beyond some of the standard AX ones I may want to spy on... 
  REG_G,
  REG_B,
  REG_MODE, //- RGB, FADE_FLASH, PATTERN A...Z
  REG_FADETIME,
  REG_ONTIME,
  REG_OFFTIME,
  REG_REPEATTIMES,
  REG_FADEPERCENT
};

#define MAX_NO_SERVO_CHANGE_TIME 250

#if defined(__MK20DX256__) || defined(__MKL26Z64__)
#define AXSERIAL Serial1
#define DBGSerial Serial
#else
#define AXSERIAL Serial
#endif

//------------------------------------------------------------------------------
// Global Objects
//------------------------------------------------------------------------------
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEO_COUNT, NEO_PIN, NEO_TYPE + NEO_KHZ800);
#define COUNT_OF_SERVOS_TO_MONITOR 18
class NeoDynamelDevice : public DynamixelDevice
{
    int LEDSTATE;
    uint32_t next_step;
    uint32_t fade_start_time;

    // Define the servo ids that we will monitor positions from...
    // Right now I will just monitor Coxa and Femurs... 
    const static uint8_t    servo_ids_to_monitor_[];
    uint16_t                center_positions_[COUNT_OF_SERVOS_TO_MONITOR];
    uint16_t                last_positions_[COUNT_OF_SERVOS_TO_MONITOR];
    uint32_t                time_last_position_change_;
    bool                    servo_leds_showing_;
    bool                    center_values_set_;             // Should probably have constructor to setup
  public:
    // Use all of the standard functions but add our own functions to
    // process register changes and other messages.
    virtual int init( Stream* pStream, bool diode = false);
    virtual bool postProcessRegisterWrite(int index, int len);
    virtual void processSyncWrite(int index, int len, byte* data, int data_len );

    // plus some helper functions
    void updateNeoPixel( );
    void SetColor(uint8_t r, uint8_t g, uint8_t b, uint16_t wDelay = 0);
    void BlinkID(uint8_t id );

};

const uint8_t NeoDynamelDevice::servo_ids_to_monitor_[] = {
      cRRCoxaPin, cRRFemurPin, cRRTibiaPin, 
      cRMCoxaPin, cRMFemurPin, cRMTibiaPin, 
      cRFCoxaPin, cRFFemurPin, cRFTibiaPin, 
      cLFCoxaPin, cLFFemurPin, cLFTibiaPin, 
      cLMCoxaPin, cLMFemurPin, cLMTibiaPin, 
      cLRCoxaPin, cLRFemurPin, cLRTibiaPin};
 
NeoDynamelDevice neo;


//------------------------------------------------------------------------------
// Setup
//------------------------------------------------------------------------------
void setup()
{
  delay(250);
  AXSERIAL.begin( 1000000 );
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  pinMode(LED_PIN, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  neo.init( &AXSERIAL);

  // Lets see what our ID is?
#ifdef DBGSerial
  DBGSerial.begin(38400);
#endif
  delay(1000);
#ifdef DBGSerial
  DBGSerial.println("Setup completed");
#endif    
  neo.BlinkID(neo.REG[ REG_ID ]);
}



//------------------------------------------------------------------------------
// loop
//------------------------------------------------------------------------------
void loop()
{
  neo.processInput();
  neo.updateNeoPixel();
}

//------------------------------------------------------------------------------
// Virtual functionction that gets called when any logical registers change
//------------------------------------------------------------------------------
int NeoDynamelDevice::init( Stream* pStream, bool diode )
{
  return DynamixelDevice::init(pStream, diode);
}

//------------------------------------------------------------------------------
// Virtual functionction that gets called when any logical registers change
//------------------------------------------------------------------------------
bool NeoDynamelDevice::postProcessRegisterWrite(int index, int len)
{
  // Only do local process in our specific range...
  if ( !((index > REG_FADEPERCENT) || ((index + len) <= REG_R) ))
  {
    switch ( REG[ REG_MODE ] )
    {
      case MANUAL:
        {
          REG[ REG_REPEATTIMES ] = 0;
          SetColor(REG[ REG_R ], REG[ REG_G ], REG[ REG_B ] );
        }
        break;

      case FADE:
        {
          fade_start_time = millis();
          REG[REG_FADEPERCENT] = 100;
        }
        break;
    }


  }
  return DynamixelDevice::postProcessRegisterWrite(index, len);
}

//==============================================================================
// Lets spy on Syncwrites that include goal positions...
//==============================================================================
//    const static uint8_t    servo_ids_to_monitor_[];
//    uint16_t                center_positions__[COUNT_OF_SERVOS_TO_MONITOR];
//    uint16_t                last_positions_[COUNT_OF_SERVOS_TO_MONITOR];
//    uint32_t                time_last_position_change_;

void NeoDynamelDevice::processSyncWrite(int index, int len, byte* data, int data_len )
{
#if NEO_COUNT > 1  
  // See if this sync write includes goal positoin
  int index_goal_position = (index - AX_GOAL_POSITION_L) + 1; // need to skip servo id as well
  digitalWrite(3, !digitalRead(3));
  if ((index_goal_position >= 1) && (index_goal_position <= len) )
  {
    // Ok this one includes changing positions...
    bool positions_changed = false;
    for (int i = 0; i < data_len; i += len) 
    {
      for (int j = 0; j < COUNT_OF_SERVOS_TO_MONITOR; j++)
      {
        if (data[i] == servo_ids_to_monitor_[j] )
        {
          uint16_t pos = data[i + index_goal_position] + (data[i + index_goal_position + 1] << 8);

          if (pos != last_positions_[j])
          {
            last_positions_[j] = pos;
            if (!center_values_set_)
              center_positions_[j] = pos;
            else  
              positions_changed = true;
          }
          break;  // should not have duplicate ids...
        }
      }
    }
    center_values_set_ = true;    // make sure we only do once

    // If something changed remember that and output something to LEDS...
    if (positions_changed)
    {
      uint8_t iServo = 0;
      for (int iLED=1; iLED < strip.numPixels(); iLED++)
      {
        uint32_t color = 0;
        if (last_positions_[iServo] > center_positions_[iServo])
        {
          if (last_positions_[iServo+1] > center_positions_[iServo+1])
            color = 0x804000;
          else if (last_positions_[iServo+1] < center_positions_[iServo+1])
            color = 0x800040;
          else
            color = 0x800000;  
        }
        else if (last_positions_[iServo] < center_positions_[iServo])
        {
          if (last_positions_[iServo+1] > center_positions_[iServo+1])
            color = 0x004080;
          else if (last_positions_[iServo+1] < center_positions_[iServo+1])
            color = 0x008040;
          else
            color = 0x008080;  
        }
        else
        {
          if (last_positions_[iServo+1] > center_positions_[iServo+1])
            color = 0x008000;
          else if (last_positions_[iServo+1] < center_positions_[iServo+1])
            color = 0x400080;
          else
            color = 0x000080; 
        }
        strip.setPixelColor(iLED, color);
        iServo += 3;
      }
      digitalWrite(4, !digitalRead(4));
      strip.show();
      time_last_position_change_ = millis();
      servo_leds_showing_ = true;
    }
  }
#endif    
  DynamixelDevice::processSyncWrite(index, len, data, data_len );
}

//------------------------------------------------------------------------------
// SetColor - helper function to set the neopixel color
//------------------------------------------------------------------------------
void NeoDynamelDevice::SetColor(uint8_t r, uint8_t g, uint8_t b, uint16_t wDelay)
{
  strip.setPixelColor(0, r, g, b );
  strip.show();

  if (wDelay)
    delay (wDelay);

}


//------------------------------------------------------------------------------
// BlinkID - Temporary helper function.
//------------------------------------------------------------------------------
void  NeoDynamelDevice::NeoDynamelDevice::BlinkID(uint8_t id )
{
  #ifdef DBGSerial
  DBGSerial.print("ID: ");
  DBGSerial.println(id, DEC);
#endif    

  uint8_t count = id / 100;
  // Use Blue for 100's
  while (count-- > 0)
  {
    SetColor(0, 0, 255, 250);
    SetColor(0, 0, 0, 250);
  }

  // Use Green for 10's
  count = (id / 10) % 10;
  while (count-- > 0)
  {
    SetColor(0, 255, 0, 250);
    SetColor(0, 0, 0, 250);
  }
  // Use Green for 10's

  count = id % 10;
  while (count-- > 0)
  {
    SetColor(255, 0, 0, 250);
    SetColor(0, 0, 0, 250);
  }
}

//------------------------------------------------------------------------------
// updateNeoPixel - main function called by loop to update the state of the NeoPixel
//------------------------------------------------------------------------------

void NeoDynamelDevice::updateNeoPixel( )
{
  uint32_t mill = millis();

  switch ( REG[ REG_MODE ] )
  {
    case FLASH:
      {
        if ( REG[ REG_REPEATTIMES ] > 0 && next_step < mill )
        {
          if ( next_step == 0 ) next_step = mill;

          if ( LEDSTATE == 0 )
          {
            SetColor( REG[ REG_R ], REG[ REG_G ], REG[ REG_B ] );
            LEDSTATE = 1;
            next_step += ( REG[ REG_ONTIME ] * 5 );
          }
          else
          {
            SetColor( 0, 0, 0 );
            LEDSTATE = 0;
            next_step += ( REG[ REG_OFFTIME ] * 5 );
            if ( REG[ REG_REPEATTIMES ] != 0xFF ) --REG[ REG_REPEATTIMES ];
          }
        }
        else if ( REG[ REG_REPEATTIMES ] <= 0)
        {
          next_step = 0;
        }
      }
      break;
    case FADE:
      {
        // Is our fade still active?
        if ( REG[REG_FADEPERCENT] )
        {
          uint32_t delta_time = mill - fade_start_time;

          if ( delta_time >=  ( REG[ REG_FADETIME ] * 5 ))
          {
            REG[REG_FADEPERCENT] = 0;  // tell system we are done.
            SetColor(0, 0, 0);  // make sure everything is turned off
          }
          else
          {
            int percent_time = 100 - ((delta_time * 100) / ( REG[ REG_FADETIME ] * 5 ));
            if (percent_time != REG[REG_FADEPERCENT])
            {
              REG[REG_FADEPERCENT] = percent_time;
              SetColor(((int)REG[ REG_R ] * percent_time) / 100,
                       ((int)REG[ REG_G ] * percent_time) / 100,
                       ((int)REG[ REG_B ] * percent_time) / 100 );
            }
          }
        }
      }
      break;
  }
  // Also lets see if we should clear all of the LED states... 
  if (servo_leds_showing_ && ((mill - time_last_position_change_) > MAX_NO_SERVO_CHANGE_TIME ) )
  {
   digitalWrite(5, !digitalRead(5));
   // We have not received any changed positions for some time.  Assume this is the new center point... 
    int i;
    for (i = 0; i < COUNT_OF_SERVOS_TO_MONITOR; i++)
      center_positions_[i] = last_positions_[i];
    for (i = 1; i < strip.numPixels(); i++)
      strip.setPixelColor(i, 0);
    strip.show();
    servo_leds_showing_ = false;
  }
}





