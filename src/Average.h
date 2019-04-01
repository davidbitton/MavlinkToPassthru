#include <stdint.h>

#ifndef AVERAGE_H_
#define AVERAGE_H_

#include "Battery.h"

class Average {
  private:
    Battery m_bat1;
    Battery m_bat2;

  public:
    Average(Battery bat1, Battery bat2);

    uint32_t Get_Volt_Average1(uint16_t mV);
    uint32_t Get_Current_Average1(uint16_t dA);
    void Accum_Volts1(uint32_t mVlt);
    void Accum_mAh1(uint32_t dAs);
    float Total_mAh1();
    float Total_mWh1();
    uint32_t Get_Volt_Average2(uint16_t mV);
    uint32_t Get_Current_Average2(uint16_t dA);
    void Accum_Volts2(uint32_t mVlt);
    void Accum_mAh2(uint32_t dAs);
    float Total_mAh2();
    float Total_mWh2();
};

#endif