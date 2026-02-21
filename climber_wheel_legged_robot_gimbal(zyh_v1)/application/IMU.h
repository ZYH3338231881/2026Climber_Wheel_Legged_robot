#ifndef IMU_H
#define IMU_H

#include "struct_typedef.h"
#include "macro_typedef.h"

extern inline float GetImuAngle(uint8_t axis);
extern inline float GetImuVelocity(uint8_t axis);
extern inline float GetImuAccel(uint8_t axis);
extern float GetYawBias(void);

extern float get_raw_gyro(uint8_t axis);
extern float get_raw_accel(uint8_t axis);

#endif  // IMU_H
/*------------------------------ End of File ------------------------------*/
