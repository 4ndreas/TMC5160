//
//
//

#include "TMC5160.h"
#include <SPI.h>


// alternative driver https://github.com/tommag/TMC5160_Arduino


void TMC5160::begin(SPIClass * _mySPI, bool _default = true, int chipCS_pin = 0)
{
  mySPI = _mySPI;
  chipCS = chipCS_pin;

  pinMode(chipCS, OUTPUT);
  digitalWrite(chipCS, HIGH);

  invertMotor = false;
  if (_default)
  {
    setStealthChop();
    setEncoder(4000);
    //setCurrent(10, 30, 2);
    setPowerDown(2);

    // Values for speed and acceleration
    tmc5160_writeInt(TMC5160_VSTART, 1);
    tmc5160_writeInt(TMC5160_A1, 5000);
    tmc5160_writeInt(TMC5160_V1, 26843);
    tmc5160_writeInt(TMC5160_AMAX, 5000);  //5000
    tmc5160_writeInt(TMC5160_VMAX, 50000);
    tmc5160_writeInt(TMC5160_DMAX, 500);
    tmc5160_writeInt(TMC5160_D1, 5000);
    tmc5160_writeInt(TMC5160_VSTOP, 10);
    tmc5160_writeInt(TMC5160_RAMPMODE, TMC5160_MODE_POSITION);

    // default motor values
    motorStepps = 200;
    microStepps = 256;
    gearing = 1;
    fCLK = 120000000; // internal clk 12MHz
    // 12MHz internal clock
    timebase = 1.398101333;  // 2^24/fCLK = 2^24/12000000
  }
}

void TMC5160::setVelocityMode(void)
{
  setVMAX(0);
  tmc5160_writeInt(TMC5160_RAMPMODE, TMC5160_MODE_VELPOS);

  //setMaxSpeed(0); // There is no way to know if we should move in the positive or negative direction => set speed to 0.
	//writeRegister(TMC5160_Reg::RAMPMODE, TMC5160_Reg::VELOCITY_MODE_POS);

}

void TMC5160::initSPI(uint8_t _misoPin, uint8_t _mosiPin, uint8_t sckPin)
{
  // Setup SPI 2
  mySPI->begin(); //Initialize the SPI_2 port.
  //  mySPI->setBitOrder(MSBFIRST); // Set the SPI_2 bit order
  //  mySPI->setDataMode(SPI_MODE3); //Set the  SPI_2 data mode 3
  //  mySPI->setClockDivider(SPI_CLOCK_DIV8);  // about 5MHz for more I got problems

  //mySPI->beginTransaction(SPI_MOTOR_SETTINGS);
}

void TMC5160::setEncoder(int counts)
{
  // Encoder config
  uint32_t ENC_CONST = 0x000C1F40;     // 12.8 // 4000 enc counts
  //uint32_t ENC_CONST = 0xFFF307D0;  //-12.8
  tmc5160_writeInt(TMC5160_ENCMODE, 0x400);
  tmc5160_writeInt(TMC5160_ENC_CONST, ENC_CONST);
  tmc5160_writeInt(TMC5160_XENC, 0); // set register to 0
  //tmc5160_writeInt(TMC5160_XACTUAL, 0); // set register to 0
}


void TMC5160::invertDriver(bool invert)
{
  invertMotor = invert;

  uint32_t _GCONF = tmc5160_readInt(TMC5160_GCONF); // dummy or old data
  _GCONF = tmc5160_readInt(TMC5160_GCONF);

  if(invert){
       _GCONF |= (1<<4);
  }
  else
  {
     _GCONF &= ~(1<<4);
  }
  
  tmc5160_writeInt( TMC5160_GCONF, _GCONF);
}

void TMC5160::setVMAX(int vmax)   // max velocity
{
  tmc5160_writeInt(TMC5160_VMAX, vmax);
}

void TMC5160::setAMAX(int amax)   // max accel
{
  tmc5160_writeInt(TMC5160_AMAX, amax);  //5000
}

void TMC5160::setDMAX(int dmax)   // max de
{
  tmc5160_writeInt(TMC5160_DMAX, dmax);
}




void TMC5160::setDCMotorMode(bool torqueLimit)
{
  int mode = 0;

  uint32_t _GCONF = 0;
  uint32_t _IHOLD_IRUN = 31;  // CurrentLimit
  uint32_t _CHOPCONF = 0; // default=0x10410150; 
  uint32_t _PWMCONF = 0;  // default=0xC40C001E;
  
  // Set TOFF > 0 to enable the driver
  _CHOPCONF |= (3<<0);

  // direct_mode
  _GCONF |= (1<<TMC5160_DIRECT_MODE_SHIFT);

  if( mode == 0)  // direct current control
  { 
   _IHOLD_IRUN = 5;  // CurrentLimit
  }
  else if (mode == 1) // PWM DUTY CYCLE VELOCITY CONTROL
  {
      // PWM Duty Velocity control 
      // en_pwm_mode = 1, 
      // pwm_autoscale = 0, 
      // PWM_AMPL = 255, 
      // PWM_GRAD = 4, 
      // IHOLD = 31
      // Set TOFF > 0 to enable the driver.
      // en_pwm_mode = 1, 
      _GCONF |= (1<<TMC5160_EN_PWM_MODE_SHIFT);
      
      // pwm_autoscale = 0
      _PWMCONF &= ~(1<<TMC5160_PWM_AUTOSCALE_SHIFT);

      //PWM_OFS = 255 PWM_AMPL = 255,
      _PWMCONF |= (0xFF<<TMC5160_PWM_OFS_SHIFT);

      // PWM_GRAD = 4, 
      _PWMCONF |= (4<<TMC5160_PWM_GRAD_SHIFT);

      if(torqueLimit) // ADDITIONAL TORQUE LIMIT
      {
        // pwm_autoscale = 1, 
        _PWMCONF |= (1<<TMC5160_PWM_AUTOSCALE_SHIFT);
      }

      // freewheeling
      //  _IHOLD_IRUN = 0;
      //_PWMCONF |= (1<<TMC5160_FREEWHEEL_SHIFT);      
  }
  else // PURELY TORQUE LIMITED OPERATION
  {

  }

  tmc5160_writeInt( TMC5160_GCONF, 0);

  tmc5160_writeInt( TMC5160_CHOPCONF, _CHOPCONF);
  tmc5160_writeInt( TMC5160_PWMCONF, _PWMCONF);

  tmc5160_writeInt( TMC5160_IHOLD_IRUN, _IHOLD_IRUN);
  tmc5160_writeInt( TMC5160_GCONF, _GCONF);
}

void TMC5160::setDCMotor(int PWM_M1, int PWM_M2)
{
  PWM_M1 = constrain(PWM_M1,-255,255);
  PWM_M2 = constrain(PWM_M2,-255,255);

  if(invertMotor)
  {
    PWM_M1 *= -1;
    PWM_M2 *= -1;
  }
  
  if(( -minPWM_VAL < PWM_M1)&&(PWM_M1 < minPWM_VAL))
  {
    PWM_M1 = 0;
  }

  if(( -minPWM_VAL < PWM_M2)&&(PWM_M2 < minPWM_VAL))
  {
    PWM_M2 = 0;
  }
  
  int32_t XTARGET = 0;
  
  XTARGET = (PWM_M1 & 0xFF) + ((PWM_M2 &0xFF) << 16);

  if(PWM_M1 <0)
  {
    XTARGET += (1<<8);
  }
  if(PWM_M2 <0)
  {
    XTARGET |= (1<<24);
  }
  
  tmc5160_writeInt(TMC5160_XTARGET, XTARGET);
}



void TMC5160::setDCStep(void)
{
  // MULTISTEP_FILT=1, EN_PWM_MODE=1 enables stealthChop�
  // tmc5160_writeInt( TMC5160_GCONF, 0x0000000C);

  // TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle�)
  tmc5160_writeInt(TMC5160_CHOPCONF, 0x000100C3);

  // TPOWERDOWN=10: Delay before power down in stand still
  tmc5160_writeInt(TMC5160_TPOWERDOWN, 0x0000000A);

  // TPWMTHRS=500
  tmc5160_writeInt(TMC5160_TPWMTHRS, 0x000001F4);
}


void TMC5160::setStealthChop(void)
{
  // MULTISTEP_FILT=1, EN_PWM_MODE=1 enables stealthChop
  tmc5160_writeInt( TMC5160_GCONF, 0x0000000C);

  // TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle)
  tmc5160_writeInt(TMC5160_CHOPCONF, 0x000100C3);

  // TPOWERDOWN=10: Delay before power down in stand still
  tmc5160_writeInt(TMC5160_TPOWERDOWN, 0x0000000A);

  // TPWMTHRS=500
  tmc5160_writeInt(TMC5160_TPWMTHRS, 0x000001F4);
}

void TMC5160::setSpreadCycle(void)
{
  // MULTISTEP_FILT=1, EN_PWM_MODE=1 enables stealthChop
  tmc5160_writeInt(TMC5160_GCONF, 0x0000000C);

  // TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle)
  tmc5160_writeInt(TMC5160_CHOPCONF, 0x000100C3);

  // TPOWERDOWN=10: Delay before power down in stand still
  tmc5160_writeInt(TMC5160_TPOWERDOWN, 0x0000000A);

  // TPWMTHRS=500
  tmc5160_writeInt(TMC5160_TPWMTHRS, 0x000001F4);
}


void TMC5160::readStatus(void)
{
  uint32_t read = tmc5160_readInt(TMC5160_DRVSTATUS); // reads the last transmission all reads are delayed by one
  DRV_STAT = tmc5160_readInt(TMC5160_DRVSTATUS);

  stst	   = ((DRV_STAT >> 31) & 0x01);
  olb        = ((DRV_STAT >> 30) & 0x01);
  ola        = ((DRV_STAT >> 29) & 0x01);
  s2gb       = ((DRV_STAT >> 28) & 0x01);
  s2ga       = ((DRV_STAT >> 27) & 0x01);
  otpw       = ((DRV_STAT >> 26) & 0x01);
  ot         = ((DRV_STAT >> 25) & 0x01);
  stallGuard = ((DRV_STAT >> 24) & 0x01);
  CS_ACTUAL  = ((DRV_STAT >> 20) & 0x07);
  fsactive   = ((DRV_STAT >> 15) & 0x01);
  stealth    = ((DRV_STAT >> 14) & 0x01);
  s2vsb      = ((DRV_STAT >> 13) & 0x01);
  s2vsa      = ((DRV_STAT >> 12) & 0x01);
  SG_RESULT  = (DRV_STAT & 0x1F);
}

void TMC5160::gotoPosition(int32_t XTARGET)
{
  //  Serial.print("Motor to ");
  //  Serial.println(XTARGET);

  tmc5160_writeInt(TMC5160_XTARGET, XTARGET);
}

void TMC5160::setPosition(int32_t XTARGET)
{
  tmc5160_writeInt(TMC5160_XACTUAL, XTARGET);
  //tmc5160_writeInt(TMC5160_XTARGET, XTARGET);
}


int32_t TMC5160::getPosition(void)
{
  int32_t XACTUAL = tmc5160_readInt(TMC5160_XACTUAL);
  XACTUAL = tmc5160_readInt(TMC5160_XACTUAL);
  return XACTUAL;
}

int32_t TMC5160::getSpeed(void)
{
  int32_t VACTUAL = tmc5160_readInt(TMC5160_VACTUAL);
  VACTUAL = tmc5160_readInt(TMC5160_VACTUAL);
  return VACTUAL;
}

int32_t TMC5160::getEncoderRaw(void)
{
  int32_t VACTUAL = tmc5160_readInt(TMC5160_XENC);
  VACTUAL = tmc5160_readInt(TMC5160_XENC);
  return VACTUAL;
}


void TMC5160::setMotorStepps(int _stepps, int _microstepps, double _gearing)
{
    motorStepps = _stepps;
    microStepps = _microstepps;
    gearing = _gearing;
}
void TMC5160::setRPS(float rps)
{
  // double timebase = 1.398101;  // 2^24/fCLK = 2^24/12000000

  double vmax = (timebase * rps * gearing*motorStepps*microStepps);
  setVMAX(uint32_t(vmax));
}

void TMC5160::setRPM(float rpm)
{
  double rps = (rpm  / 60.0);
  setRPS(rps);
}

double TMC5160::getRPS(void)
{
  uint32_t vmax = getSpeed();
  double rps = vmax / ( timebase * gearing*motorStepps * microStepps);
  return rps;
}


double TMC5160::getRadPerStep(void)
{
  double rps = (TWO_PI )/ (gearing * motorStepps * microStepps);

  return rps;
}


void TMC5160::setAMax(double _amax) // acc in [µS/t²]
{
  setAMAX(uint32_t(_amax));
}

void TMC5160::setDMax(double _dmax) // acc in [µS/t²]
{
  setDMAX(uint32_t(_dmax));
}

// get acc value for time and max speed
// _time time to vmax [s]
// vmax to be reached [1/s] rotation per second
// return acc in [µS/t²]
double TMC5160::getACCfromTimetoVMax(double _time, double _vmax)
{
  // vmax [µS/t]  5000
  // time [s]     1
  // fCLK [Hz]    16 000 000
  // amax = vmax*(2^17)/time/fCLK

  double vmax = (timebase * _vmax * gearing * motorStepps * microStepps);
  double amax = vmax / _time;
  return amax;
}


/*
   IHOLD
     Standstill current (0=1/32 31=32/32)
     In combination with stealthChop mode,
     setting IHOLD=0 allows to choose freewheeling
     or coil short circuit for motor stand still.

   IRUN
     Motor run current (0=1/32 31=32/32)
     Hint: Choose sense resistors in a way,
     that normal IRUN is 16 to 31 for best microstep performance.

   HOLDDELAY
     Controls the number of clock cycles for motor power down after a motion
     as soon as standstill is detected (stst=1) and TPOWERDOWN has expired.
     The smooth transition avoids a motor jerk upon power down.
*/

void TMC5160::setCurrent(uint8_t IHOLD, uint8_t IRUN, uint8_t HOLDDELAY)
{
  // IHOLD=10, IRUN=15 (max. current), IHOLDDELAY=6

  uint32_t w_register = 0;
  w_register |= HOLDDELAY & 0x0F;
  w_register <<= 8;
  w_register |= IRUN & 0x0F;
  w_register <<= 8;
  w_register |= IHOLD & 0x0F;

  tmc5160_writeInt(TMC5160_IHOLD_IRUN, 0);
  tmc5160_writeInt(TMC5160_IHOLD_IRUN, w_register);
  //tmc5160_writeInt( TMC5160_IHOLD_IRUN, 0x00080F0A);

  //PrintHex40(TMC5160_IHOLD_IRUN, w_register);
}

/*
   TPOWERDOWN
    sets the delay time after stand still (stst)
    of the motor to motor current power down.
    Time range is about 0 to 4 seconds.
   Attention: A minimum setting of 2 is required to allow automatic tuning of stealthChop PWM_OFFS_AUTO.
   Reset Default = 10
   0�((2^8)-1) * 2^18 tCLK
   tCLK = 12Mhz
*/
void TMC5160::setPowerDown(uint8_t TPOWERDOWN)
{
  uint32_t w_register = 0;
  w_register |= TPOWERDOWN & 0xFF;
  tmc5160_writeInt(TMC5160_TPOWERDOWN, w_register);
}


/* The trinamic TMC5130 motor controller and driver operates through an
  SPI interface. Each datagram is sent to the device as an address byte
  followed by 4 data bytes. This is 40 bits (8 bit address and 32 bit word).
  Each register is specified by a one byte (MSB) address: 0 for read, 1 for
  write. The MSB is transmitted first on the rising edge of SCK.

*/

// General SPI decription
void TMC5160::tmc5160_writeInt(uint8_t address, uint32_t value)
{
  uint8_t stat;
  uint32_t datagram = value;
  uint32_t i_datagram = 0;

  digitalWrite(chipCS, LOW);
  delayMicroseconds(10);


  mySPI->beginTransaction(SPI_MOTOR_SETTINGS);
  mySPI->transfer(address | 0x80);
  mySPI->transfer((datagram >> 24) & 0xff);
  mySPI->transfer((datagram >> 16) & 0xff);
  mySPI->transfer((datagram >> 8) & 0xff);
  mySPI->transfer((datagram) & 0xff);
  mySPI->endTransaction();
  digitalWrite(chipCS, HIGH);
}

uint32_t TMC5160::tmc5160_readInt(uint8_t address)
{
  uint8_t stat;
  uint32_t i_datagram;
  uint32_t datagram = 0;

  delayMicroseconds(10);
  digitalWrite(chipCS, LOW);
  delayMicroseconds(10);

  mySPI->beginTransaction(SPI_MOTOR_SETTINGS);

  stat = mySPI->transfer(address & 0x7F);

  i_datagram |= mySPI->transfer(0);
  i_datagram <<= 8;
  i_datagram |= mySPI->transfer(0);
  i_datagram <<= 8;
  i_datagram |= mySPI->transfer(0);
  i_datagram <<= 8;
  i_datagram |= mySPI->transfer(0);

  mySPI->endTransaction();

  digitalWrite(chipCS, HIGH);
  return i_datagram;
}


// void TMC5160::readStatus()
// { 
//  char tmp[32];
 
//  int read = tmc5160_readInt(TMC5160_DRVSTATUS); // reads the last transmission all reads are delayed by one
//  motorstat.DRV_STAT = tmc5160_readInt(TMC5160_DRVSTATUS); 
//  motorstat.XACTUAL = tmc5160_readInt(TMC5160_XACTUAL);
//  motorstat.x_ENC = tmc5160_readInt(TMC5160_XENC);
//  motorstat.VACTUAL = tmc5160_readInt(TMC5160_VACTUAL); 

//  int datagram = motorstat.DRV_STAT;
 
//  Serial.println("");
//  sprintf(tmp,"stst       %1d", (datagram >> 31) & 0x01); Serial.println(tmp);
//  sprintf(tmp,"olb        %1d", (datagram >> 30) & 0x01); Serial.println(tmp);
//  sprintf(tmp,"ola        %1d", (datagram >> 29) & 0x01); Serial.println(tmp);
//  sprintf(tmp,"s2gb       %1d", (datagram >> 28) & 0x01); Serial.println(tmp);
//  sprintf(tmp,"s2ga       %1d", (datagram >> 27) & 0x01); Serial.println(tmp);
//  sprintf(tmp,"otpw       %1d", (datagram >> 26) & 0x01); Serial.println(tmp);
//  sprintf(tmp,"ot         %1d", (datagram >> 25) & 0x01); Serial.println(tmp);
//  sprintf(tmp,"stallGuard %1d", (datagram >> 24) & 0x01); Serial.println(tmp);
 
//  sprintf(tmp,"CS_ACTUAL  %d", (datagram >> 20) & 0x07);  Serial.println(tmp);
//  sprintf(tmp,"fsactive   %1d", (datagram >> 15) & 0x01); Serial.println(tmp);
//  sprintf(tmp,"stealth    %1d", (datagram >> 14) & 0x01); Serial.println(tmp);
//  sprintf(tmp,"s2vsb      %1d", (datagram >> 13) & 0x01); Serial.println(tmp);
//  sprintf(tmp,"s2vsa      %1d", (datagram >> 12) & 0x01); Serial.println(tmp);
//  sprintf(tmp,"SG_RESULT  %d", (datagram) & 0x1F); Serial.println(tmp);

 
//  sprintf(tmp,"XACTUAL    %d", motorstat.XACTUAL); Serial.print(tmp);
//  sprintf(tmp," ENCODER    %d", motorstat.x_ENC); Serial.println(tmp);
//  //sprintf(tmp," VACTUAL    %d", motorstat.VACTUAL); Serial.println(tmp);

// }