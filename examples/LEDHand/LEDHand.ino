
#include <LEDFader.h>
#include <EEPROM.h>
#include <dynamixelDevice.h>
DynamixelDevice Dyna;
class LEDHand : public DynamixelDevice
{
  public:
    virtual bool postProcessRegisterWrite(int index, int len);
};

LEDHand g_ledhand;


enum RGB_MODES
{
  MANUAL,
  FADE,
  FLASH
};

enum COLOR
{
  RED=9,
  GREEN,
  BLUE
};

enum RGB_REGISTERS
{
  REG_R=0x20,
  REG_G,
  REG_B,
  REG_MODE, //- RGB, FADE_FLASH, PATTERN A...Z
  REG_FADETIME,
  REG_ONTIME,
  REG_OFFTIME,
  REG_REPEATTIMES,
  REG_FADEPERCENT

};

void setup() 
{ 
  Serial.begin( 1000000 );   
  InitRGB(); 
  g_ledhand.init(&Serial );
}

void SetColor( int red, int green, int blue )
{
  if( red >= 0 )  analogWrite( RED, 255 - red );
  if( green >= 0 )analogWrite( GREEN, 255 - green );
  if( blue >= 0 ) analogWrite( BLUE, 255 - blue );  
}

int LEDSTATE;
long long nextStep;
uint32_t fade_start_time;


void Step( long long mill )
{      
  switch( g_ledhand.REG[ REG_MODE ] )
  {
    case FLASH:
    {
      if( g_ledhand.REG[ REG_REPEATTIMES ] > 0 && nextStep < mill )
      {
        if( nextStep == 0 ) nextStep = mill;
               
        if( LEDSTATE == 0 )
        {
          SetColor( g_ledhand.REG[ REG_R ], g_ledhand.REG[ REG_G ], g_ledhand.REG[ REG_B ] );
          LEDSTATE = 1;
          nextStep += ( g_ledhand.REG[ REG_ONTIME ] * 5 );
        }
        else
        {
          SetColor( 0, 0, 0 );
          LEDSTATE = 0;
          nextStep += ( g_ledhand.REG[ REG_OFFTIME ] * 5 );
          if( g_ledhand.REG[ REG_REPEATTIMES ] != 0xFF ) --g_ledhand.REG[ REG_REPEATTIMES ];  
        }      
      }
      else if( g_ledhand.REG[ REG_REPEATTIMES ]<=0)
      {
        nextStep = 0;
      }
    }
    break;
    case FADE:
    {
      // Is our fade still active?
      if ( g_ledhand.REG[REG_FADEPERCENT] )
      {
        uint32_t delta_time = mill - fade_start_time;

        if ( delta_time >=  ( g_ledhand.REG[ REG_FADETIME ] * 5 ))
        {
          g_ledhand.REG[REG_FADEPERCENT] = 0;  // tell system we are done.
          SetColor(0, 0, 0);  // make sure everything is turned off
        }
        else
        {
          int percent_time = 100 - ((delta_time * 100) / ( g_ledhand.REG[ REG_FADETIME ] * 5 ));
          if (percent_time != g_ledhand.REG[REG_FADEPERCENT])
          {
            g_ledhand.REG[REG_FADEPERCENT] = percent_time;
            SetColor(((int)g_ledhand.REG[ REG_R ] * percent_time) / 100,
                     ((int)g_ledhand.REG[ REG_G ] * percent_time) / 100,
                     ((int)g_ledhand.REG[ REG_B ] * percent_time) / 100 );
          }
        }
      }
    }
    break;
  }      
}

  
void InitRGB()
{
  pinMode( RED, OUTPUT );
  pinMode( GREEN, OUTPUT );
  pinMode( BLUE, OUTPUT );
  analogWrite( RED, 255 );
  analogWrite( GREEN, 255 );
  analogWrite( BLUE, 255 );
  pinMode( 13, OUTPUT );
}


void loop() 
{
  g_ledhand.processInput();

  Step( millis() );
}

bool LEDHand::postProcessRegisterWrite(int index, int len)
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









