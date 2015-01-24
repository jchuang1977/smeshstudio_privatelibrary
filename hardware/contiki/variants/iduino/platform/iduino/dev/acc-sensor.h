
#ifndef ACC_SENSOR_H_
#define ACC_SENSOR_H_

#include "lib/sensors.h"
#include "twi.h"
#define LIS3DH_ADDR 0x19
extern const struct sensors_sensor acc_sensor;

#define ACC_SENSOR "Acc"
#define ACC_X_AXIS    1
#define ACC_Y_AXIS    2
#define ACC_Z_AXIS    3

#define OFFSET_X         0x16
#define OFFSET_Y         0x17
#define OFFSET_Z         0x18
#define GAIN_X           0x19
#define GAIN_Y           0x1A
#define GAIN_Z           0x1B
#define CTRL_REG1        0x20
#define CTRL_REG2        0x21
#define STATUS_REG       0x27
#define OUTX_L           0x28
#define OUTX_H           0x29
#define OUTY_L           0x2A
#define OUTY_H           0x2B
#define OUTZ_L           0x2C
#define OUTZ_H           0x2D
#define FF_WU_CFG        0x30
#define FF_WU_SRC        0x31
#define FF_WU_ACK        0x32
#define FF_WU_THS_L      0x34
#define FF_WU_THS_H      0x35
#define FF_WU_DURATION   0x36
#define DD_CFG           0x38
#define DD_SRC           0x39
#define DD_ACK           0x3A
#define DD_THSI_L        0x3C
#define DD_THSI_H        0x3D
#define DD_THSE_L        0x3E
#define DD_THSE_H        0x3F


#endif /* ACC_SENSOR_H_ */
/** @} */
