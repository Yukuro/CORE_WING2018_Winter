/*
  @file BMP180.cpp

  @brief Barometric Pressure and Temperature Sensor BMP180 Breakout I2C Library

  @Author spiridion (http://mailme.spiridion.net)

  Tested on LPC1768 and FRDM-KL25Z

  Copyright (c) 2014 spiridion
  Released under the MIT License (see http://mbed.org/license/mit)

  Documentation regarding the BMP180 can be found here:
  http://mbed.org/media/uploads/spiridion/bst-bmp180-ds000-09.pdf
*/

#include "BMP180.h"
#include "mbed.h"

// Uncomment to test the documentation algorithm against the documentation example 
// Result should be 699.64 hPa and 15°C
// #define BMP180_TEST_FORMULA 

BMP180::BMP180(PinName sda, PinName scl, int address)
   : m_i2c(sda,scl), m_addr(address)
{
    m_altitude = 0;
    m_oss = BMP180_OSS_NORMAL; 
    m_temperature = UNSET_BMP180_TEMPERATURE_VALUE;
    m_pressure = UNSET_BMP180_PRESSURE_VALUE;  
}

BMP180::BMP180(I2C& i2c, int address)
   : m_i2c(i2c), m_addr(address)
{
    m_altitude = 0;
    m_oss = BMP180_OSS_NORMAL; 
    m_temperature = UNSET_BMP180_TEMPERATURE_VALUE;
    m_pressure = UNSET_BMP180_PRESSURE_VALUE;  
}

int  BMP180::Initialize(float altitude, int overSamplingSetting)
{
    char data[22];
    int errors = 0;
        
    m_altitude = altitude;
    m_oss = overSamplingSetting; 
    m_temperature = UNSET_BMP180_TEMPERATURE_VALUE;
    m_pressure = UNSET_BMP180_PRESSURE_VALUE;  
    
    // read calibration data
    data[0]=0xAA;
    errors = m_i2c.write(m_addr, data, 1);  // set the eeprom pointer position to 0xAA
    errors += m_i2c.read(m_addr, data, 22); // read 11 x 16 bits at this position 
    wait_ms(10);
    
    // store calibration data for further calculus  
    ac1 = data[0]  << 8 | data[1];
    ac2 = data[2]  << 8 | data[3];
    ac3 = data[4]  << 8 | data[5];
    ac4 = data[6]  << 8 | data[7];
    ac5 = data[8]  << 8 | data[9];
    ac6 = data[10] << 8 | data[11];
    b1  = data[12] << 8 | data[13];
    b2  = data[14] << 8 | data[15];
    mb  = data[16] << 8 | data[17];
    mc  = data[18] << 8 | data[19];
    md  = data[20] << 8 | data[21];

#ifdef BMP180_TEST_FORMULA
    ac1 = 408;
    ac2 = -72;
    ac3 = -14383;
    ac4 = 32741;
    ac5 = 32757;
    ac6 = 23153;
    b1 = 6190;
    b2 = 4;
    mb = -32768;
    mc = -8711;
    md = 2868;
    m_oss = 0;
    errors = 0;
#endif // #ifdef BMP180_TEST_FORMULA

    return errors? 0 : 1;
}

int BMP180::ReadData(float* pTemperature, float* pPressure)
{
    long t, p;

    if (!ReadRawTemperature(&t) || !ReadRawPressure(&p))
    {
        m_temperature = UNSET_BMP180_TEMPERATURE_VALUE;
        m_pressure = UNSET_BMP180_PRESSURE_VALUE;  
        return 0;
    }

    m_temperature = TrueTemperature(t);
    m_pressure = TruePressure(p);

    if (pPressure)
        *pPressure = m_pressure;
    if (pTemperature)
        *pTemperature = m_temperature;

    return 1;
}

int BMP180::ReadRawTemperature(long* pUt)
{
    int errors = 0;
    char data[2];
    
    // request temperature measurement
    data[0] = 0xF4;
    data[1] = 0x2E;
    errors = m_i2c.write(m_addr, data, 2); // write 0XF2 into reg 0XF4

    wait_ms(4.5F);

    // read raw temperature data
    data[0] = 0xF6;
    errors += m_i2c.write(m_addr, data, 2); // set eeprom pointer position to 0XF6
    errors += m_i2c.read(m_addr, data, 2);  // get 16 bits at this position 
    
#ifdef BMP180_TEST_FORMULA
    errors = 0;
#endif // #ifdef BMP180_TEST_FORMULA

    if (errors)
        return 0;
    else
        *pUt = data[0] << 8 | data[1];

#ifdef BMP180_TEST_FORMULA
    *pUt = 27898;
#endif // #ifdef BMP180_TEST_FORMULA
    
    return 1;
}

int BMP180::ReadRawPressure(long* pUp)
{
    int errors = 0;
    char data[2];
    
    // request pressure measurement
    data[0] = 0xF4;
    data[1] = 0x34 + (m_oss << 6);
    errors = m_i2c.write(m_addr, data, 2); // write 0x34 + (m_oss << 6) into reg 0XF4

    switch (m_oss)
    {
        case BMP180_OSS_ULTRA_LOW_POWER:        wait_ms(4.5); break;
        case BMP180_OSS_NORMAL:                 wait_ms(7.5); break;
        case BMP180_OSS_HIGH_RESOLUTION:        wait_ms(13.5); break;
        case BMP180_OSS_ULTRA_HIGH_RESOLUTION:  wait_ms(25.5); break;
    }

    // read raw pressure data
    data[0] = 0xF6;
    errors += m_i2c.write(m_addr, data, 1); // set eeprom pointer position to 0XF6
    errors += m_i2c.read(m_addr, data, 2);  // get 16 bits at this position     
    
#ifdef BMP180_TEST_FORMULA
    errors = 0;
#endif // #ifdef BMP180_TEST_FORMULA

    if (errors)
        return 0;
    else
        *pUp = (data[0] << 16 | data[1] << 8) >> (8 - m_oss);
#ifdef BMP180_TEST_FORMULA
        *pUp = 23843;
#endif // #ifdef BMP180_TEST_FORMULA

    return 1;
}

float BMP180::TrueTemperature(long ut)
{
    long t;
    
    // straight out from the documentation
    x1 = ((ut - ac6) * ac5) >> 15;
    x2 = ((long)mc << 11) / (x1 + md);
    b5 = x1 + x2;
    t = (b5 + 8) >> 4;

    // convert to celcius
    return t / 10.F;
}

float BMP180::TruePressure(long up)
{
    long p;
    
    // straight out from the documentation
    b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 >> 12)) >> 11;
    x2 = ac2 * b6 >> 11;
    x3 = x1 + x2;
    b3 = (((ac1 * 4 + x3) << m_oss) + 2) >> 2;
    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = ac4 * (unsigned long)(x3 + 32768) >> 15;
    b7 = ((unsigned long)up - b3)* (50000 >> m_oss);
    if (b7 < 0x80000000)
        p = (b7 << 1) / b4;
    else
        p = (b7 / b4) << 1;
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);

    // convert to hPa and, if altitude has been initialized, to sea level pressure  
    if (m_altitude == 0.F)
        return p / 100.F;
    else
        return  p / (100.F * pow((1.F - m_altitude / 44330.0L), 5.255L)); 
}