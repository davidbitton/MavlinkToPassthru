#ifndef BATTERY_H_
#define BATTERY_H_

struct Battery {
    float mAh;
    float tot_mAh;
    float avg_dA;
    float avg_mV;
    uint32_t prv_millis;
    uint32_t tot_volts; // sum of all samples
    uint32_t tot_mW;
    uint32_t samples;
    bool ft;
};

#endif