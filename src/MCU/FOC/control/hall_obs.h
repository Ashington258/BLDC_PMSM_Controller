// hall_obs.h
#pragma once
#include "main.h"
#include <stdint.h>

typedef struct
{
    // 配置
    uint8_t pole_pairs; // 极对数（你的项目里是 7）
    // 原始采样
    volatile uint8_t hall_raw; // 3bit: b2=W, b1=V, b0=U
    volatile uint8_t hall_prev;
    volatile int8_t dir;     // +1 正转，-1 反转，0 未知
    volatile uint8_t sector; // 0..5
    // 时间戳/周期
    volatile uint32_t t_edge_us;     // 上一跳变时刻（us）
    volatile uint32_t last_Tedge_us; // 最近一次边沿周期
    // 速度估计
    float fe_hz;      // 电角频率估计 (Hz)
    float fe_hz_filt; // 低通后的电角频率
    // 角度估计
    float theta_e; // 电角度 [rad] 0..2π
    float theta_m; // 机械角度 [rad] 0..2π
    // 插值辅助
    volatile uint32_t t_now_us; // 在 Task 中更新用
    float alpha_lp;             // 速度低通系数 0..1（越小越平滑）
    // 失速/停转容错
    uint32_t timeout_us; // 超时判定（无边沿时用）
} HallObs;

void Hall_Init(HallObs *h, uint8_t pole_pairs, float lp_alpha, uint32_t timeout_us);
void Hall_OnEdge(HallObs *h, uint8_t hall_raw, uint32_t now_us);
void Hall_UpdateAngle(HallObs *h, uint32_t now_us);
