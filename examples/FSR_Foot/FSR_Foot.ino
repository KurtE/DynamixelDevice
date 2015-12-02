
#include <dynamixelDevice.h>
#include <digitalWriteFast.h>

class FSRFoot : public DynamixelDevice
{
  public:
    virtual bool postProcessRegisterWrite(int index, int len);
    virtual void preProcessRegisterRead(int index, int len);

};

FSRFoot g_fsrfoot;

int led = 13;
int footForceOffset[4];
int footForce[4];
int footSensor[4] = { A1, A2, A3, A4 };

enum EEPREG
{
  FSR_SENS1 = 0x11,
  FSR_SENS2,
  FSR_SENS3,
  FSR_SENS4
};

enum REGISTERS
{
  FSR1_L = 0X1A,
  FSR1_H,
  FSR2_L,
  FSR2_H,
  FSR3_L,
  FSR3_H,
  FSR4_L = 0x20,
  FSR4_H,
  FSR_Central_X,
  FSR_Central_Y,
  FSR_SetOffset,
  FSR_ClearOffset
};


void setup()
{
  pinMode(led, OUTPUT);
  
  digitalWrite( 9, LOW );
  digitalWrite( 10, LOW );
  digitalWrite( 11, LOW );
  digitalWrite( 12, LOW );
  
  pinMode( 9, OUTPUT );
  pinMode( 10, OUTPUT );
  pinMode( 11, OUTPUT );
  pinMode( 12, OUTPUT );
  
  pinMode( A1, INPUT );
  pinMode( A2, INPUT );
  pinMode( A3, INPUT );
  pinMode( A4, INPUT );

  pinMode( 3, OUTPUT );
  pinMode( 4, OUTPUT );
  
  Serial.begin( 1000000 );
  
  if( g_fsrfoot.init( &Serial ) == 0 )
  {
    g_fsrfoot.REG[ FSR_SENS1 ] = 0;
    g_fsrfoot.REG[ FSR_SENS2 ] = 1;
    g_fsrfoot.REG[ FSR_SENS3 ] = 2;
    g_fsrfoot.REG[ FSR_SENS4 ] = 3;
  }
}

void Store(int pin, int index, int regLow, int regHigh)
{
  int value = analogRead( pin ) + footForceOffset[ index ];
  if( value <= 0 )
  {
    value = 0;
  }
  footForce[ index ] = value;
  g_fsrfoot.REG[ regLow ] = value & 0xff;
  g_fsrfoot.REG[ regHigh ] = ( value >> 8 ) & 0xff;
}

void Average( float* previous, float next )
{
  *previous = ( *previous * .9 ) + ( next * .1 );
}

float centerY;
float centerX;
int average = 0;


#define ZEROISH 5
void CalculateOutputs()
{
  if( footForce[ 0 ] < ZEROISH && footForce[ 2 ] == ZEROISH && footForce[ 1 ] == ZEROISH && footForce[ 3 ] == ZEROISH )
  {
    g_fsrfoot.REG[ FSR_Central_X ] = 255;
    g_fsrfoot.REG[ FSR_Central_Y ] = 255;
    return;
  }
  
  float x1 = footForce[ 0 ] + footForce[ 2 ];
  float x2 = footForce[ 1 ] + footForce[ 3 ];  
  float y1 = footForce[ 0 ] + footForce[ 1 ];
  float y2 = footForce[ 2 ] + footForce[ 3 ];
   
  if( x1 + x2 > 0 )
  {
    Average( &centerX, ( 254 * x2 ) / ( x1 + x2 ) );    
  }
  else Average( &centerX, 128 );
  
  if( y1 + y2 > 0 )
  {
    Average( &centerY, ( 254 * y2 ) / ( y1 + y2 ) );
  }
  else Average( &centerY, 128 );
  
  g_fsrfoot.REG[ FSR_Central_X ] = (int)centerX;
  g_fsrfoot.REG[ FSR_Central_Y ] = (int)centerY; 
}


//right foot 3201
//left foot 3201
uint8_t loop_cycle_state = 0;

void loop() 
{

  g_fsrfoot.processInput();

  digitalWrite(3, HIGH);
  switch (loop_cycle_state++)
  {
    case 0:
      Store( footSensor[ g_fsrfoot.REG[ FSR_SENS1 ] ], 0, FSR1_L, FSR1_H );
      break;
    case 1:
      Store( footSensor[ g_fsrfoot.REG[ FSR_SENS2 ] ], 1, FSR2_L, FSR2_H );
      break;
    case 2:
      Store( footSensor[ g_fsrfoot.REG[ FSR_SENS3 ] ], 2, FSR3_L, FSR3_H );
      break;
    case 3:
      Store( footSensor[ g_fsrfoot.REG[ FSR_SENS4 ] ], 3, FSR4_L, FSR4_H );
      break;
    case 4:
      CalculateOutputs();
      loop_cycle_state = 0;
      break;
  }
  digitalWrite(3, LOW);
}

//------------------------------------------------------------------------------
// Virtual functionction that gets called when any logical registers change
//------------------------------------------------------------------------------
void FSRFoot::preProcessRegisterRead(int index, int len)
{
  DynamixelDevice::preProcessRegisterRead(index, len); 
}

//------------------------------------------------------------------------------
// Virtual functionction that gets called when any logical registers change
//------------------------------------------------------------------------------
bool FSRFoot::postProcessRegisterWrite(int index, int len)
{
  if( REG[ FSR_SetOffset ] )
  {      
    if( REG[ FSR_ClearOffset ] )
    {
      for( int i = 0; i < 4; ++i )
      {
        footForceOffset[ i ] = -( footForce[ i ] + ZEROISH );
      }
      REG[ FSR_SetOffset ] = 0;
    }
    else
    {
      REG[ FSR_ClearOffset ] = 1;
    }      
  }
  
  if( REG[ FSR_ClearOffset ] )
  {
    for( int i = 0; i < 4; ++i )
    {
      footForceOffset[ i ] = 0;
    }
    REG[ FSR_ClearOffset ] = 0;
  }
  return DynamixelDevice::postProcessRegisterWrite(index, len);
}

