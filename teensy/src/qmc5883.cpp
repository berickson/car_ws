#include "qmc5883.h"

#include <Wire.h>

void QMC5883::setAddress(uint8_t addr){
  address = addr;
}

void QMC5883::WriteReg(byte Reg,byte val){
  wire.beginTransmission(address); //start talking
  wire.write(Reg); // Tell the HMC5883 to Continuously Measure
  wire.write(val); // Set the Register
  wire.endTransmission();
}

void QMC5883::init(){
  WriteReg(0x0B,0x01);
  //Define Set/Reset period
  setMode(Mode_Continuous,ODR_200Hz,RNG_8G,OSR_512);
  /*
  Define
  OSR = 512
  Full Scale Range = 8G(Gauss)
  ODR = 200HZ
  set continuous measurement mode
  */
}

void QMC5883::setMode(uint16_t mode,uint16_t odr,uint16_t rng,uint16_t osr){
  WriteReg(0x09,mode|odr|rng|osr);
}


void QMC5883::softReset(){
  WriteReg(0x0A,0x80);
}

void QMC5883::read(int16_t* x,int16_t* y,int16_t* z){
  wire.beginTransmission(address);
  wire.write(0x00);
  wire.endTransmission();
  wire.requestFrom(address, uint8_t(6));
  *x = wire.read() | wire.read() << 8;
  *y = wire.read() | wire.read() << 8;
  *z = wire.read() | wire.read() << 8;
}

void QMC5883::read(int16_t* x,int16_t* y,int16_t* z,int* a){
  read(x,y,z);
  *a = azimuth(x, y);
}

void QMC5883::read(int16_t* x,int16_t* y,int16_t* z,float* a){
  read(x,y,z);
  *a = azimuth(x, y);
}


float QMC5883:: azimuth(int16_t x, int16_t y){
  float azimuth = atan2(y, x) * 180.0/PI;
  return azimuth < 0?360 + azimuth:azimuth;
}