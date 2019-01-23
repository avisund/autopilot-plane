#include <errno.h>
#include <unistd.h>
#include <wiringPiI2C.h>
#include <iostream>

using namespace std;

#define AK8963_ADDRESS 0x0C   // << 1
#define AK8963_WHO_AM_I 0x00  // should return 0x48
#define AK8963_INFO 0x01
#define AK8963_ST1 0x02     // data ready status bit 0
#define AK8963_XOUT_L 0x03  // data
#define AK8963_XOUT_H 0x04
#define AK8963_YOUT_L 0x05
#define AK8963_YOUT_H 0x06
#define AK8963_ZOUT_L 0x07
#define AK8963_ZOUT_H 0x08
#define AK8963_ST2 0x09     // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL 0x0A    // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC 0x0C    // Self test control
#define AK8963_I2CDIS 0x0F  // I2C disable
#define AK8963_ASAX 0x10    // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY 0x11    // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ 0x12    // Fuse ROM z-axis sensitivity adjustment value


#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A 0x10

#define XG_OFFSET_H 0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L 0x14
#define YG_OFFSET_H 0x15
#define YG_OFFSET_L 0x16
#define ZG_OFFSET_H 0x17
#define ZG_OFFSET_L 0x18
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D
#define LP_ACCEL_ODR 0x1E
#define WOM_THR 0x1F

#define MOT_DUR 0x20    // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR 0x21   // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR 0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN 0x23
#define I2C_MST_CTRL 0x24
#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV1_ADDR 0x28
#define I2C_SLV1_REG 0x29
#define I2C_SLV1_CTRL 0x2A
#define I2C_SLV2_ADDR 0x2B
#define I2C_SLV2_REG 0x2C
#define I2C_SLV2_CTRL 0x2D
#define I2C_SLV3_ADDR 0x2E
#define I2C_SLV3_REG 0x2F
#define I2C_SLV3_CTRL 0x30
#define I2C_SLV4_ADDR 0x31
#define I2C_SLV4_REG 0x32
#define I2C_SLV4_DO 0x33
#define I2C_SLV4_CTRL 0x34
#define I2C_SLV4_DI 0x35
#define I2C_MST_STATUS 0x36
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define DMP_INT_STATUS 0x39  // Check DMP interrupt
#define INT_STATUS 0x3A
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO 0x63
#define I2C_SLV1_DO 0x64
#define I2C_SLV2_DO 0x65
#define I2C_SLV3_DO 0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET 0x68
#define MOT_DETECT_CTRL 0x69
#define USER_CTRL 0x6A   // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1 0x6B  // Device defaults to the SLEEP mode
#define PWR_MGMT_2 0x6C
#define DMP_BANK 0x6D    // Activates a specific bank in the DMP
#define DMP_RW_PNT 0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG 0x6F     // Register in DMP from which to read or to which to write
#define DMP_REG_1 0x70
#define DMP_REG_2 0x71
#define FIFO_COUNTH 0x72
#define FIFO_COUNTL 0x73
#define FIFO_R_W 0x74
#define WHO_AM_I_MPU9250 0x75  // Should return 0x71
#define XA_OFFSET_H 0x77
#define XA_OFFSET_L 0x78
#define YA_OFFSET_H 0x7A
#define YA_OFFSET_L 0x7B
#define ZA_OFFSET_H 0x7D
#define ZA_OFFSET_L 0x7E

// Using the MSENSR-9250 breakout board, ADO is set to 0
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
//mbed uses the eight-bit device address, so shift seven-bit addresses left by one!
#define MPU9250_ADDRESS 0x68

int mpu, mag;

void readAccelData(int16_t* destination) {
    destination[0] = (int16_t)(((int16_t)wiringPiI2CReadReg8(mpu, ACCEL_XOUT_H + 0) << 8) | wiringPiI2CReadReg8(mpu, ACCEL_XOUT_H + 1));
    destination[1] = (int16_t)(((int16_t)wiringPiI2CReadReg8(mpu, ACCEL_XOUT_H + 2) << 8) | wiringPiI2CReadReg8(mpu, ACCEL_XOUT_H + 3));
    destination[2] = (int16_t)(((int16_t)wiringPiI2CReadReg8(mpu, ACCEL_XOUT_H + 4) << 8) | wiringPiI2CReadReg8(mpu, ACCEL_XOUT_H + 5));
}

void readGyroData(int16_t* destination) {
    destination[0] = (int16_t)(((int16_t)wiringPiI2CReadReg8(mpu, GYRO_XOUT_H + 0) << 8) | wiringPiI2CReadReg8(mpu, GYRO_XOUT_H + 1));
    destination[1] = (int16_t)(((int16_t)wiringPiI2CReadReg8(mpu, GYRO_XOUT_H + 2) << 8) | wiringPiI2CReadReg8(mpu, GYRO_XOUT_H + 3));
    destination[2] = (int16_t)(((int16_t)wiringPiI2CReadReg8(mpu, GYRO_XOUT_H + 4) << 8) | wiringPiI2CReadReg8(mpu, GYRO_XOUT_H + 5));
}

void readMagData(int16_t* destination) {
    if (wiringPiI2CReadReg8(mag, AK8963_ST1) & 0x01) {
        uint8_t c = wiringPiI2CReadReg8(mag, AK8963_XOUT_L + 6);
        if (!(c & 0x08)) {
            destination[0] = (int16_t)(((int16_t)wiringPiI2CReadReg8(mag, AK8963_XOUT_L + 1) << 8) | wiringPiI2CReadReg8(mag, AK8963_XOUT_L + 0));
            destination[1] = (int16_t)(((int16_t)wiringPiI2CReadReg8(mag, AK8963_XOUT_L + 3) << 8) | wiringPiI2CReadReg8(mag, AK8963_XOUT_L + 2));
            destination[2] = (int16_t)(((int16_t)wiringPiI2CReadReg8(mag, AK8963_XOUT_L + 5) << 8) | wiringPiI2CReadReg8(mag, AK8963_XOUT_L + 4));
            int useless = wiringPiI2CReadReg8(mag, AK8963_XOUT_L + 6);
        } else {
            cout << "no mag update" << endl;
        }
    } else {
        cout << "failed to read mag" << endl; 
    }
}

int main() {
    // int fd, result;
    mpu = wiringPiI2CSetup(MPU9250_ADDRESS);
    // enable mag: (https://stackoverflow.com/questions/43920137/magnetometer-data-on-mpu-9250-uclinux)
    wiringPiI2CWriteReg8(mpu, 0x37, 0x22);

    // TODO: stick this in init fn (https://github.com/kriswiner/MPU9250/blob/master/STM32F401/MPU9250.h)
    mag = wiringPiI2CSetup(AK8963_ADDRESS);
    // set mag to continuous measurement mode:
    // power down mag
    wiringPiI2CWriteReg8(mag, AK8963_CNTL, 0x00);
    usleep(100000);

    // fuse rom access mode
    wiringPiI2CWriteReg8(mag, AK8963_CNTL, 0x0F);
    usleep(100000);

    // read x y z calibration vals

    float calibVals[3];
    calibVals[0] = (float)(wiringPiI2CReadReg8(mag, AK8963_ASAX + 0) - 128) / 256.0f + 1.0f;
    calibVals[1] = (float)(wiringPiI2CReadReg8(mag, AK8963_ASAX + 1) - 128) / 256.0f + 1.0f;
    calibVals[2] = (float)(wiringPiI2CReadReg8(mag, AK8963_ASAX + 2) - 128) / 256.0f + 1.0f;
    usleep(100000);

    // power down mag
    wiringPiI2CWriteReg8(mag, AK8963_CNTL, 0x00);
    usleep(100000);

    // enable continuous mode
    enum Mscale {
        MFS_14BITS = 0,  // 0.6 mG per LSB
        MFS_16BITS       // 0.15 mG per LSB
    };
    uint8_t Mscale = MFS_16BITS;  // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
    uint8_t Mmode = 0x06;

    wiringPiI2CWriteReg8(mag, AK8963_CNTL, Mscale << 4 | Mmode);  // 00000110
    usleep(100000);

    // fd = wiringPiI2CSetup(0x68);
    // cout << "Init result: " << fd << endl;
    // while (1) {
    //     if(wiringPiI2CReadReg8(fd, 61)) {
    //     int16_t accl_x = 0;
    //     accl_x = (int16_t)(((int16_t)wiringPiI2CReadReg8(fd, 61) << 8) | wiringPiI2CReadReg8(fd, 62));
    //     cout << accl_x << endl;
    //     // usleep(100000); // 0.1 secs
    //     }
    // }

    while (1) {
        usleep(100000);
        int16_t accl[3];
        int16_t gyro[3];
        int16_t m_mag[3];
        readAccelData(accl);
        readGyroData(gyro);
        readMagData(m_mag);
        cout << "accel: x " << accl[0] << endl;  //", y "<<accl[1]<<", z "<<accl[2]<<endl;
        cout << "gyro: x " << gyro[0] << endl;   //", y "<<gyro[1]<<", z "<<gyro[2]<<endl;
        cout << "mag: x " << m_mag[0] << ", y " << m_mag[1] << ", z " << m_mag[2] << endl;
    }
}
