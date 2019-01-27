#include <unistd.h>
#include <wiringPiI2C.h>
#include <iostream>

#define BMP388_ADDRESS 0x77
#define BMP388_PRES_0 0x04  // 24 pressure data stored across 3 registers. This is LSB
#define BMP388_PRES_1 0x05
#define BMP388_PRES_2 0x06  // MSB
#define BMP388_TEMP_0 0x07  // 8 bit LSB
#define BMP388_TEMP_1 0x08
#define BMP388_TEMP_2 0x09  // MSB
#define BMP388_PWR_CTRL 0x1B

int bmp;

enum OSS {      // BMP-085 sampling rate
    OSS_0 = 0,  // 4.5 ms conversion time
    OSS_1,      // 7.5
    OSS_2,      // 13.5
    OSS_3       // 25.5
};

// Specify sensor parameters
uint8_t OSS = OSS_3;  // maximum pressure resolution

uint32_t readTemp() {
    wiringPiI2CWriteReg8(bmp, BMP388_PWR_CTRL, 0x2E);
    return ((uint8_t)wiringPiI2CReadReg8(bmp, BMP388_TEMP_2) << 16) + ((uint8_t)wiringPiI2CReadReg8(bmp, BMP388_TEMP_1) << 8) + (uint8_t)wiringPiI2CReadReg8(bmp, BMP388_TEMP_0);
}

uint32_t readPressure() {
    wiringPiI2CWriteReg8(bmp, BMP388_PWR_CTRL, 0x1D);  //0x34 | OSS << 6);
    usleep(100000);
    return ((uint8_t)wiringPiI2CReadReg8(bmp, BMP388_PRES_2) << 16) + ((uint8_t)wiringPiI2CReadReg8(bmp, BMP388_PRES_1) << 8) + (uint8_t)wiringPiI2CReadReg8(bmp, BMP388_PRES_0);
}

int main() {
    bmp = wiringPiI2CSetup(BMP388_ADDRESS);
    usleep(26000);
    uint8_t a = 0xff;
    uint32_t b = 0;
    uint32_t c = 0xffffff;
    uint32_t d = 0;
    b = a;
    b <<= 8;
    b += a;
    b <<= 8;
    b += a;

    d = (a << 16) + (a << 8) + a;
    //b = a << 8 | a << 8 | a << 8;
    // std::cout << b << std::endl
    //           << c << std::endl
    //           << d;
    while (1) {
        usleep(100000);
        std::cout << "Temp: " << readTemp();
        usleep(100000);
        std::cout << ", Pres: " << readPressure() << std::endl;
    }

    return 0;
}