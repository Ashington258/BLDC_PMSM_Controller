// hall_obs.c
#include "hall_obs.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static inline float wrap_2pi(float x)
{
    while (x >= 2.0f * (float)M_PI)
        x -= 2.0f * (float)M_PI;
    while (x < 0.0f)
        x += 2.0f * (float)M_PI;
    return x;
}

// 合法霍尔状态表（U,V,W）：依你的接线可能要调整顺序
// 常见序列：001→101→100→110→010→011→001（正转）
static const uint8_t valid_states[6] = {0b001, 0b101, 0b100, 0b110, 0b010, 0b011};

static int8_t state_to_sector(uint8_t s)
{
    for (int i = 0; i < 6; ++i)
        if (valid_states[i] == s)
            return i;
    return -1; // 非法
}

static int8_t sector_step(int8_t prev, int8_t now)
{
    if (prev < 0 || now < 0)
        return 0;
    int8_t diff = now - prev;
    if (diff == 1 || diff == -5)
        return +1; // 正向+1
    if (diff == -1 || diff == 5)
        return -1; // 反向-1
    return 0;      // 跳变异常(跨多段)
}

void Hall_Init(HallObs *h, uint8_t pole_pairs, float lp_alpha, uint32_t timeout_us)
{
    h->pole_pairs = pole_pairs;
    h->hall_raw = 0;
    h->hall_prev = 0;
    h->dir = 0;
    h->sector = 0;
    h->t_edge_us = 0;
    h->last_Tedge_us = 0;
    h->fe_hz = 0.0f;
    h->fe_hz_filt = 0.0f;
    h->theta_e = 0.0f;
    h->theta_m = 0.0f;
    h->t_now_us = 0;
    h->alpha_lp = (lp_alpha > 0.f && lp_alpha < 1.f) ? lp_alpha : 0.1f;
    h->timeout_us = timeout_us ? timeout_us : 200000; // 0.2s 默认
}

// 每次边沿进入（在 HAL_TIMEx_HallSensor_Callback 调用）
void Hall_OnEdge(HallObs *h, uint8_t hall_raw, uint32_t now_us)
{
    int8_t sec_prev = state_to_sector(h->hall_prev & 0x7);
    int8_t sec_now = state_to_sector(hall_raw & 0x7);

    // 时间差
    uint32_t dt_us = (h->t_edge_us == 0) ? 0 : (now_us - h->t_edge_us);
    h->t_edge_us = now_us;

    if (sec_now < 0)
    {
        // 非法状态：忽略
        return;
    }

    if (sec_prev >= 0 && dt_us > 0)
    {
        int8_t step = sector_step(sec_prev, sec_now);
        if (step != 0)
        {
            h->dir = step; // +1 正转，-1 反转
            h->sector = (uint8_t)sec_now;
            h->last_Tedge_us = dt_us; // 最近一次 60° 电角的时间
            // 60°边沿 → 360°需要 6 倍时间
            float Telec_s = (h->last_Tedge_us * 1e-6f) * 6.0f;
            if (Telec_s > 1e-6f)
            {
                h->fe_hz = 1.0f / Telec_s; // 电角频率
                // 简单一阶低通
                h->fe_hz_filt = h->fe_hz_filt + h->alpha_lp * (h->fe_hz - h->fe_hz_filt);
            }
            // 边沿到达时，把电角度“卡”到扇区开始（或结束）
            // 我们选：扇区起点角度 = sector * 60°
            float theta_sector = (float)sec_now * (float)M_PI / 3.0f;
            h->theta_e = wrap_2pi(theta_sector);
            h->theta_m = wrap_2pi(h->theta_e / (float)h->pole_pairs);
        }
        else
        {
            // 跨多段或抖动，忽略（也可加入错误计数）
        }
    }
    else
    {
        // 第一次有效状态：仅记录
        h->sector = (uint8_t)sec_now;
        h->theta_e = wrap_2pi((float)sec_now * (float)M_PI / 3.0f);
        h->theta_m = wrap_2pi(h->theta_e / (float)h->pole_pairs);
    }

    h->hall_prev = hall_raw & 0x7;
}

// 在高速定时（比如 TIM1 20kHz 的周期回调）里做插值
void Hall_UpdateAngle(HallObs *h, uint32_t now_us)
{
    h->t_now_us = now_us;
    uint32_t since_edge = (h->t_edge_us == 0) ? 0 : (now_us - h->t_edge_us);

    // 若长期没有边沿，判定停转：保持最后角速度为 0，不再推进角度
    if (h->last_Tedge_us == 0 || since_edge > h->timeout_us)
    {
        h->fe_hz_filt = 0.0f;
        return;
    }

    // 扇区内线性插值：Δθ = (since_edge / Tedge) * 60°
    float Tedge_s = (float)h->last_Tedge_us * 1e-6f;
    float frac = (float)since_edge * 1e-6f / Tedge_s; // 0..1 附近
    if (frac < 0.f)
        frac = 0.f;
    if (frac > 1.2f)
        frac = 1.2f; // 允许一点超前以缓和抖动

    float delta = frac * (float)M_PI / 3.0f; // 60° = π/3
    if (h->dir >= 0)
    {
        h->theta_e = wrap_2pi((float)(h->sector) * (float)M_PI / 3.0f + delta);
    }
    else
    {
        // 反向则在扇区内逆向插值
        h->theta_e = wrap_2pi((float)(h->sector) * (float)M_PI / 3.0f - delta);
    }
    h->theta_m = wrap_2pi(h->theta_e / (float)h->pole_pairs);
}
