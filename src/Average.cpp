#include <Arduino.h>
#include "Average.h"

Average::Average(Battery b1, Battery b2) {
    m_bat1 = b1;
    m_bat2 = b2;
}

uint32_t Average::Get_Volt_Average1(uint16_t mV)  {

  if (m_bat1.avg_mV < 1) m_bat1.avg_mV = mV;  // Initialise first time

  m_bat1.avg_mV = (m_bat1.avg_mV * 0.9) + (mV * 0.1);  // moving average
  Accum_Volts1(mV);  
  return m_bat1.avg_mV;
}

uint32_t Average::Get_Current_Average1(uint16_t dA)  {   // in 10*milliamperes (1 = 10 milliampere)
  
  Accum_mAh1(dA);  
  
  if (m_bat1.avg_dA < 1){
    m_bat1.avg_dA = dA;  // Initialise first time
  }

  m_bat1.avg_dA = (m_bat1.avg_dA * 0.8) + (dA * 0.2);  // moving average

  return m_bat1.avg_dA;
  }

void Average::Accum_Volts1(uint32_t mVlt) {    //  mV   milli-Volts
  m_bat1.tot_volts += (mVlt / 1000);    // Volts
  m_bat1.samples++;
}

void Average::Accum_mAh1(uint32_t dAs) {        //  dA    10 = 1A
  if (m_bat1.ft) {
    m_bat1.prv_millis = millis() -1;   // prevent divide zero
    m_bat1.ft = false;
  }
  uint32_t period = millis() - m_bat1.prv_millis;
  m_bat1.prv_millis = millis();
    
  double hrs = (float)(period / 3600000.0f);  // ms to hours

  m_bat1.mAh = dAs * hrs;   //  Tiny dAh consumed this tiny period di/dt
 // m_bat1.mAh *= 100;        //  dA to mA  
  m_bat1.mAh *= 10;        //  dA to mA ?
  
  m_bat1.tot_mAh += m_bat1.mAh;   //   Add them all in
}

float Average::Total_mAh1() {
  return m_bat1.tot_mAh;
}

float Average::Total_mWh1() {                                     // Total energy consumed bat1
  return m_bat1.tot_mAh * (m_bat1.tot_volts / m_bat1.samples);
}
//***********************************************************
uint32_t Average::Get_Volt_Average2(uint16_t mV)  {
  
  if (m_bat2.avg_mV == 0) m_bat2.avg_mV = mV;  // Initialise first time

  m_bat2.avg_mV = (m_bat2.avg_mV * 0.8) + (mV * 0.2);  // moving average
  Accum_Volts2(mV);  
  return m_bat2.avg_mV;
}
  
uint32_t Average::Get_Current_Average2(uint16_t dA)  {

  if (m_bat2.avg_dA == 0) m_bat2.avg_dA = dA;  // Initialise first time

  m_bat2.avg_dA = (m_bat2.avg_dA * 0.9) + (dA * 0.1);  // moving average

  Accum_mAh2(dA);  
  return m_bat2.avg_dA;
  }

void Average::Accum_Volts2(uint32_t mVlt) {    //  mV   milli-Volts
  m_bat2.tot_volts += (mVlt / 1000);    // Volts
  m_bat2.samples++;
}

void Average::Accum_mAh2(uint32_t dAs) {        //  dA    10 = 1A
  if (m_bat2.ft) {
    m_bat2.prv_millis = millis() -1;   // prevent divide zero
    m_bat2.ft = false;
  }
  uint32_t period = millis() - m_bat2.prv_millis;
  m_bat2.prv_millis = millis();
    
 double hrs = (float)(period / 3600000.0f);  // ms to hours

  m_bat2.mAh = dAs * hrs;   //  Tiny dAh consumed this tiny period di/dt
 // m_bat2.mAh *= 100;        //  dA to mA  
  m_bat2.mAh *= 10;        //  dA to mA ?
}

float Average::Total_mAh2() {
  return m_bat2.tot_mAh;
}

float Average::Total_mWh2() {                                     // Total energy consumed bat1
  return m_bat2.tot_mAh * (m_bat2.tot_volts / m_bat2.samples);
}
