/*
 * TMC5160 Dual DC Motor driver example
 * 
 * 
 */

#include <SPI.h>
#include <TMC5160.h>

TMC5160 stepper;

// HW-SPI-2
#define SPI2_MOSI_PIN  PC3
#define SPI2_SCK_PIN   PA5
#define SPI2_MISO_PIN  PC2
#define SPI2_NSS_PIN   PB12
#define SPI2_NSS2_PIN  PA4
SPIClass SPI_2(SPI2_MOSI_PIN, SPI2_MISO_PIN, SPI2_SCK_PIN);


void setup(){

   /* TMC5160 as DC motor driver */
   SPI_2.setMISO(SPI2_MISO_PIN);
   SPI_2.setMOSI(SPI2_MOSI_PIN);
   SPI_2.setSCLK(SPI2_SCK_PIN);
   SPI_2.begin();
   SPI_2.beginTransaction( SPISettings(100000, MSBFIRST, SPI_MODE3));
   
   pinMode(SPI2_NSS2_PIN, OUTPUT);
   stepper.begin(&SPI_2, false, SPI2_NSS2_PIN);   // DC0 DC1
   stepper.setDCMotorMode(false); // no torque Limit
   stepper.setDCMotor(0,0); // set DC Torque Mode
   stepper.setEncoder(0);   // enable Encode
 
}

void loop()
{
      // set Motor 1 to Max 
      stepper.setDCMotor(255,0); 

      delay(5000);
      
      // set Motor 2 to Max 
      stepper.setDCMotor(0,255);

      delay(5000);

      // set Motor 2 to -Max 
      stepper.setDCMotor(0,-255);
      
      delay(5000);

      // all off 
      stepper.setDCMotor(0,0);  
}
