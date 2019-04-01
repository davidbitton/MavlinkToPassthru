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

struct Battery bat1 = {
    0, 0, 0, 0, 0, 0, 0, true};

struct Battery bat2 = {
    0, 0, 0, 0, 0, 0, 0, true};

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