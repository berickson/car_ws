#pragma once
// https://datasheet.lcsc.com/lcsc/QST-QMC5883L-TR_C192585.pdf

#include "Arduino.h"
#include "Wire.h"

#define QMC5883_ADDR 0x0D


//REG CONTROL

//0x09

#define Mode_Standby    0b00000000
#define Mode_Continuous 0b00000001

#define ODR_10Hz        0b00000000
#define ODR_50Hz        0b00000100
#define ODR_100Hz       0b00001000
#define ODR_200Hz       0b00001100

#define RNG_2G          0b00000000
#define RNG_8G          0b00010000

#define OSR_512         0b00000000
#define OSR_256         0b01000000
#define OSR_128         0b10000000
#define OSR_64          0b11000000


class QMC5883{
public:

QMC5883(TwoWire& w = Wire) : wire(w) {};
void setAddress(uint8_t addr);

void init(); //init qmc5883

void setMode(uint16_t mode,uint16_t odr,uint16_t rng,uint16_t osr); // setting

void softReset(); //soft RESET

void read(int16_t* x,int16_t* y,int16_t* z); //reading
void read(int16_t* x,int16_t* y,int16_t* z,int* a);
void read(int16_t* x,int16_t* y,int16_t* z,float* a);

float azimuth(int16_t x,int16_t y);

private:

void WriteReg(uint8_t Reg,uint8_t val);


TwoWire& wire;
uint8_t address = QMC5883_ADDR;

};
