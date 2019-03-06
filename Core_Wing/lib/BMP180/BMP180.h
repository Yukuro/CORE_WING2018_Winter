

#ifndef BMP180_H
#define BMP180_H

#include "mbed.h"

///  default address is 0xEF 
#define BMP180_I2C_ADDRESS 0xEF 

// Oversampling settings
#define BMP180_OSS_ULTRA_LOW_POWER 0        // 1 sample  and  4.5ms for conversion
#define BMP180_OSS_NORMAL          1        // 2 samples and  7.5ms for conversion
#define BMP180_OSS_HIGH_RESOLUTION 2        // 4 samples and 13.5ms for conversion
#define BMP180_OSS_ULTRA_HIGH_RESOLUTION 3  // 8 samples and 25.5ms for conversion

#define UNSET_BMP180_PRESSURE_VALUE 0.F
#define UNSET_BMP180_TEMPERATURE_VALUE -273.15F // absolute zero

class BMP180 
{

public:
    BMP180(PinName sda, PinName scl, int address = BMP180_I2C_ADDRESS); 
    BMP180(I2C& i2c, int address = BMP180_I2C_ADDRESS); 
    int Initialize(float altitude = 0.F, int overSamplingSetting = BMP180_OSS_NORMAL);
    int ReadData(float* pTemperature = NULL, float* pPressure = NULL);
    float GetTemperature() {return m_temperature;};
   float GetPressure() {return m_pressure;};

protected:

    int ReadRawTemperature(long* pUt);
    int ReadRawPressure(long* pUp);
    float TrueTemperature(long ut);
    float TruePressure(long up);

    int m_oss;
    float m_temperature;     
    float m_pressure;
    float m_altitude;

    I2C m_i2c;   
    int m_addr;
    char m_data[4];    

    short ac1, ac2, ac3; 
    unsigned short ac4, ac5, ac6;
    short b1, b2;
    short mb, mc, md;
    long x1, x2, x3, b3, b5, b6;
    unsigned long b4, b7;

};

#endif