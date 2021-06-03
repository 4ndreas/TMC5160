// TMC5160.h

#ifndef _TMC5160_h
#define _TMC5160_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif
#include "TMC5160_Register.h"
#include "TMC5160_Constants.h"
#include "TMC5160_Mask_Shift.h"
#include <SPI.h>

#define SPI_MOTOR_SETTINGS SPISettings(1000000, MSBFIRST, SPI_MODE3)

class TMC5160 {
  public:
    // Initialize the SPI library
    //void begin(SPIClass * _mySPI, bool _default);
    void begin(SPIClass * _mySPI, bool _default, int chipCS_pin);
    void initSPI(uint8_t _misoPin, uint8_t _mosiPin, uint8_t sckPin);
    void readStatus(void);

    void setPowerDown(uint8_t TPOWERDOWN);
    void setCurrent(uint8_t IHOLD, uint8_t IRUN, uint8_t HOLDDELAY);

    void setVelocityMode(void);
    void setDCStep(void);
    void setStealthChop(void);
    void setSpreadCycle(void);
    void setEncoder(int counts);
    void resetDriver(void);
    void invertDriver(bool invert);

    // Values for speed and acceleration
    void setAMAX(int amax);		// max accel
    void setVMAX(int vmax);		// max velocity
    void setDMAX(int dmax);		// max de
    void setVSTOP(int vstop);

    void setDCMotorMode(bool torqueLimit);
    void setDCMotor(int PWM_M1, int PWM_M2);

    int32_t getSpeed(void);
    int32_t getPosition(void);
    int32_t getEncoderRaw(void);

    void setPosition(int32_t XTARGET);    // setactual Stepper Position
    void gotoPosition(int32_t XTARGET);   // move to Target Position
    

    void setMotorStepps(int _stepps, int _microstepps, double _gearing);
    void setRPS(float rps);
    void setRPM(float rpm);

    double getRPS(void);
    double getRadPerStep(void);


    void setAMax(double _amax);
    void setDMax(double _dmax);

    double getACCfromTimetoVMax(double _time, double _vmax);
    



    // Prints debug values over Serial 

    int XACTUAL;
    int VACTUAL;
    int stat;

    int x_ENC;
    int enc_deviation;

    bool stst;
    bool olb;
    bool ola;
    bool s2gb;
    bool s2ga;
    bool otpw;
    bool ot;
    bool stallGuard;
    int CS_ACTUAL;

    int minPWM_VAL;
    bool fsactive;
    bool stealth;
    bool s2vsb;
    bool s2vsa;
    int SG_RESULT;
    uint32_t DRV_STAT;

    uint32_t tmc5160_readInt(uint8_t address);
    void tmc5160_writeInt(uint8_t address, uint32_t value);

  protected:

  private:

    int chipCS;
    SPIClass * mySPI;
    uint8_t MISO_Pin;
    uint8_t MOSI_Pin;
    uint8_t SCK_Pin;

    bool invertMotor;

    
    

    int motorStepps;
    int microStepps;
    double gearing; 
    int fCLK;
    double timebase;
};



#endif
