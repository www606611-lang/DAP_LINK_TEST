#ifndef MODULES_ICM20948_H
#define MODULES_ICM20948_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t temp;
} ICM20948_Raw_t;

typedef struct {
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
    float temp_c;
} ICM20948_Data_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} ICM20948_Angle_t;

uint8_t ICM20948_Init(void);
bool ICM20948_IsReady(void);

void ICM20948_SetDeviceAddr7bit(uint8_t addr7);
uint8_t ICM20948_GetDeviceAddr7bit(void);
uint8_t ICM20948_Probe(uint8_t addr7, uint8_t *whoami);
uint8_t ICM20948_WhoAmI(void);

uint8_t ICM20948_WriteReg(uint8_t reg, uint8_t data);
uint8_t ICM20948_ReadReg(uint8_t reg, uint8_t *data);
uint8_t ICM20948_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len);

uint8_t ICM20948_ReadRaw(ICM20948_Raw_t *raw);
uint8_t ICM20948_ReadData(ICM20948_Data_t *data);

uint8_t ICM20948_GyroCalibrate(uint16_t sample_num);
void ICM20948_ResetAngle(void);
void ICM20948_UpdateAngle(float dt);
ICM20948_Angle_t ICM20948_GetAngle(void);
ICM20948_Data_t ICM20948_GetData(void);

#ifdef __cplusplus
}
#endif

#endif
