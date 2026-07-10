#ifndef MODULES_ENCODER_H
#define MODULES_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ENCODER_SAMPLE_INTERVAL_MS
/* 编码器速度计算周期，单位ms。 */
#define ENCODER_SAMPLE_INTERVAL_MS 50U
#endif

/* 编码器编号：左电机和右电机。 */
typedef enum {
    ENCODER_LEFT = 0,
    ENCODER_RIGHT,
    ENCODER_ID_COUNT
} encoder_id_t;

/* 编码器一次读取的快照数据。 */
typedef struct {
    /* 累计计数，带方向。 */
    int32_t count;
    /* 最近一个采样周期的计数变化量。 */
    int32_t delta;
    /* 当前速度，单位为脉冲/秒。 */
    int32_t speed_pps;
    /* 当前方向：1正转，-1反转，0停止。 */
    int8_t direction;
} encoder_snapshot_t;

/* 初始化编码器计数和GPIO中断。 */
void Encoder_Init(uint32_t now_ms);

/* 周期任务：按采样周期刷新速度和方向。 */
void Encoder_Task(uint32_t now_ms);

/* 清零指定编码器计数。 */
void Encoder_Reset(encoder_id_t id);

/* 清零左右两个编码器计数。 */
void Encoder_ResetAll(void);

/* 设置指定编码器计数方向反相。 */
void Encoder_SetInverted(encoder_id_t id, bool inverted);

/* 读取指定编码器累计计数。 */
int32_t Encoder_GetCount(encoder_id_t id);

/* 读取指定编码器最近一个采样周期的计数变化量。 */
int32_t Encoder_GetDelta(encoder_id_t id);

/* 读取指定编码器速度，单位为脉冲/秒。 */
int32_t Encoder_GetSpeedPps(encoder_id_t id);

/* 读取指定编码器方向：1正转，-1反转，0停止。 */
int8_t Encoder_GetDirection(encoder_id_t id);

/* 一次性读取指定编码器的计数、速度和方向。 */
bool Encoder_GetSnapshot(encoder_id_t id, encoder_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
