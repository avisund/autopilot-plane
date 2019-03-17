#include <errno.h>
#include <unistd.h>
#include <wiringPiI2C.h>
#include <iostream>
#include "MadgwickAHRS.h"

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

// Set initial input parameters
enum Ascale {
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
};

enum Gscale {
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
};

enum Mscale {
    MFS_14BITS = 0,  // 0.6 mG per LSB
    MFS_16BITS       // 0.15 mG per LSB
};

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS;  // Choose either 14-bit or 16-bit magnetometer resolution

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

void initMPU9250() {
    // https://github.com/kriswiner/MPU9250/blob/master/MPU9250BasicAHRS.ino
    wiringPiI2CWriteReg8(mpu, PWR_MGMT_1, 0x00);  // Clear sleep mode bit (6), enable all sensors
    usleep(10000);                                // Wait for all registers to reset
                                                  // get stable time source
    wiringPiI2CWriteReg8(mpu, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
    usleep(20000);

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    wiringPiI2CWriteReg8(mpu, CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    wiringPiI2CWriteReg8(mpu, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
    // determined inset in CONFIG above
    // gyro setup full range
    uint8_t c = wiringPiI2CReadReg8(mpu, GYRO_CONFIG);
    c = c & ~0x03;        // Clear Fchoice bits [1:0]
    c = c & ~0x18;        // Clear GFS bits [4:3]
    c = c | Gscale << 3;  // Set full scale range for the gyro
    wiringPiI2CWriteReg8(mpu, GYRO_CONFIG, c);

    // acc full range
    // c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG);  // get current ACCEL_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = wiringPiI2CReadReg8(mpu, ACCEL_CONFIG);
    c = c & ~0x18;        // Clear AFS bits [4:3]
    c = c | Ascale << 3;  // Set full scale range for the accelerometer
    wiringPiI2CWriteReg8(mpu, ACCEL_CONFIG, c);

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    c = wiringPiI2CReadReg8(mpu, ACCEL_CONFIG2);  // get current ACCEL_CONFIG2 register value
    c = c & ~0x0F;                                // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;                                 // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    wiringPiI2CWriteReg8(mpu, ACCEL_CONFIG2, c);  // Write new ACCEL_CONFIG2 register value
                                                  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
                                                  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the pi as master
    wiringPiI2CWriteReg8(mpu, INT_PIN_CFG, 0x22);
    wiringPiI2CWriteReg8(mpu, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    usleep(10000);
}

void calibrateMPU9250() {
    uint8_t data[12];  // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device
    wiringPiI2CWriteReg8(mpu, PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit; toggle reset device
    usleep(10000);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
    // else use the internal oscillator, bits 2:0 = 001
    wiringPiI2CWriteReg8(mpu, PWR_MGMT_1, 0x01);
    wiringPiI2CWriteReg8(mpu, PWR_MGMT_2, 0x00);
    usleep(20000);

    // Configure device for bias calculation
    wiringPiI2CWriteReg8(mpu, INT_ENABLE, 0x00);    // Disable all interrupts
    wiringPiI2CWriteReg8(mpu, FIFO_EN, 0x00);       // Disable FIFO
    wiringPiI2CWriteReg8(mpu, PWR_MGMT_1, 0x00);    // Turn on internal clock source
    wiringPiI2CWriteReg8(mpu, I2C_MST_CTRL, 0x00);  // Disable I2C master
    wiringPiI2CWriteReg8(mpu, USER_CTRL, 0x00);     // Disable FIFO and I2C master modes
    wiringPiI2CWriteReg8(mpu, USER_CTRL, 0x0C);     // Reset FIFO and DMP
    usleep(1500);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    wiringPiI2CWriteReg8(mpu, CONFIG, 0x01);        // Set low-pass filter to 188 Hz
    wiringPiI2CWriteReg8(mpu, SMPLRT_DIV, 0x00);    // Set sample rate to 1 kHz
    wiringPiI2CWriteReg8(mpu, GYRO_CONFIG, 0x00);   // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    wiringPiI2CWriteReg8(mpu, ACCEL_CONFIG, 0x00);  // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t gyrosensitivity = 131;     // = 131 LSB/degrees/sec
    uint16_t accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    wiringPiI2CWriteReg8(mpu, USER_CTRL, 0x40);  // Enable FIFO
    wiringPiI2CWriteReg8(mpu, FIFO_EN, 0x78);    // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    usleep(4000);                                // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    wiringPiI2CWriteReg8(mpu, FIFO_EN, 0x00);  // Disable gyro and accelerometer sensors for FIFO
    data[0] = wiringPiI2CReadReg8(mpu, FIFO_COUNTH);
    data[1] = wiringPiI2CReadReg8(mpu, FIFO_COUNTL);
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count / 12;  // How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        for (int tmp = 0; tmp < 12; ++tmp) {
            data[tmp] = wiringPiI2CReadReg8(mpu, FIFO_R_W + tmp);
        }
        // readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]);            // read data for averaging
        accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
        gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
        gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
        gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

        accel_bias[0] += (int32_t)accel_temp[0];  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t)accel_temp[1];
        accel_bias[2] += (int32_t)accel_temp[2];
        gyro_bias[0] += (int32_t)gyro_temp[0];
        gyro_bias[1] += (int32_t)gyro_temp[1];
        gyro_bias[2] += (int32_t)gyro_temp[2];
    }
    accel_bias[0] /= (int32_t)packet_count;  // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t)packet_count;
    accel_bias[2] /= (int32_t)packet_count;
    gyro_bias[0] /= (int32_t)packet_count;
    gyro_bias[1] /= (int32_t)packet_count;
    gyro_bias[2] /= (int32_t)packet_count;

    if (accel_bias[2] > 0L) {
        accel_bias[2] -= (int32_t)accelsensitivity;
    }  // Remove gravity from the z-axis accelerometer bias calculation
    else {
        accel_bias[2] += (int32_t)accelsensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF;  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0] / 4) & 0xFF;       // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    data[3] = (-gyro_bias[1] / 4) & 0xFF;
    data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    data[5] = (-gyro_bias[2] / 4) & 0xFF;

    // Push gyro biases to hardware registers
    wiringPiI2CWriteReg8(mpu, XG_OFFSET_H, data[0]);
    wiringPiI2CWriteReg8(mpu, XG_OFFSET_L, data[1]);
    wiringPiI2CWriteReg8(mpu, YG_OFFSET_H, data[2]);
    wiringPiI2CWriteReg8(mpu, YG_OFFSET_L, data[3]);
    wiringPiI2CWriteReg8(mpu, ZG_OFFSET_H, data[4]);
    wiringPiI2CWriteReg8(mpu, ZG_OFFSET_L, data[5]);

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0};  // A place to hold the factory accelerometer trim biases
    // readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]);  // Read factory accelerometer trim values
    data[0] = wiringPiI2CReadReg8(mpu, XA_OFFSET_H);
    data[1] = wiringPiI2CReadReg8(mpu, XA_OFFSET_L);
    accel_bias_reg[0] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
    // readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    data[0] = wiringPiI2CReadReg8(mpu, YA_OFFSET_H);
    data[1] = wiringPiI2CReadReg8(mpu, YA_OFFSET_L);
    accel_bias_reg[1] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
    // readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    data[0] = wiringPiI2CReadReg8(mpu, ZA_OFFSET_H);
    data[1] = wiringPiI2CReadReg8(mpu, ZA_OFFSET_L);
    accel_bias_reg[2] = (int32_t)(((int16_t)data[0] << 8) | data[1]);

    uint32_t mask = 1uL;              // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0};  // Define array to hold mask bit for each accelerometer bias axis

    for (ii = 0; ii < 3; ii++) {
        if ((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01;  // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0] / 8);  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0]) & 0xFF;
    data[1] = data[1] | mask_bit[0];  // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1]) & 0xFF;
    data[3] = data[3] | mask_bit[1];  // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2]) & 0xFF;
    data[5] = data[5] | mask_bit[2];  // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Apparently this is not working for the acceleration biases in the MPU-9250
    // Are we handling the temperature correction bit properly?
    // Push accelerometer biases to hardware registers
    wiringPiI2CWriteReg8(mpu, XA_OFFSET_H, data[0]);
    wiringPiI2CWriteReg8(mpu, XA_OFFSET_L, data[1]);
    wiringPiI2CWriteReg8(mpu, YA_OFFSET_H, data[2]);
    wiringPiI2CWriteReg8(mpu, YA_OFFSET_L, data[3]);
    wiringPiI2CWriteReg8(mpu, ZA_OFFSET_H, data[4]);
    wiringPiI2CWriteReg8(mpu, ZA_OFFSET_L, data[5]);
}

void toEulerAngle(double qw, double qx, double qy, double qz, double& roll, double& pitch, double& yaw) {
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (qw * qx + qy * qz);
    double cosr_cosp = +1.0 - 2.0 * (qx * qx + qy * qy);
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (qw * qz + qx * qy);
    double cosy_cosp = +1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = atan2(siny_cosp, cosy_cosp);
}

int main() {
    // int fd, result;
    mpu = wiringPiI2CSetup(MPU9250_ADDRESS);
    initMPU9250();
    calibrateMPU9250();
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
        // usleep(100000);
        int16_t accl[3];
        int16_t gyro[3];
        int16_t m_mag[3];
        readAccelData(accl);
        readGyroData(gyro);
        readMagData(m_mag);
        // cout << "accel: x " << accl[0]/1000.0 << endl;  //", y "<<accl[1]<<", z "<<accl[2]<<endl;
        // cout << "gyro: x " << gyro[0] << endl;   //", y "<<gyro[1]<<", z "<<gyro[2]<<endl;
        // cout << "mag: x " << m_mag[0] << ", y " << m_mag[1] << ", z " << m_mag[2] << endl;
        MadgwickAHRSupdate(gyro[0], gyro[1], gyro[2], accl[0], accl[1], accl[2], m_mag[0], m_mag[1], m_mag[2]);
        double roll, pitch, yaw;
        toEulerAngle(q0, q1, q2, q3, roll, pitch, yaw);
        cout << "roll: " << roll << std::endl;
        cout << "pitch: " << pitch << std::endl;
        cout << "yaw: " << yaw << std::endl
             << std::endl
             << std::endl;
    }
}
