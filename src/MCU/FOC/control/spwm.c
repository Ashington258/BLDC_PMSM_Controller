#include "spwm.h"
#include <math.h>

// 如果你的编译器没有 M_PI，可以自行定义：
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 角度归一化到 [0, 2π)
static inline float wrap_angle_0_2pi(float x)
{
    const float two_pi = 2.0f * (float)M_PI;
    while (x >= two_pi)
        x -= two_pi;
    while (x < 0.0f)
        x += two_pi;
    return x;
}

// 饱和
static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi ? hi : x);
}

void SPWM_Init(SPWM_t *spwm,
               TIM_HandleTypeDef *htim,
               float update_rate_hz,
               float init_theta_rad)
{
    spwm->htim = htim;
    spwm->theta = wrap_angle_0_2pi(init_theta_rad);
    spwm->elec_freq_hz = 0.0f;
    spwm->modulation = 0.0f;
    // dt = 1 / 调用频率
    if (update_rate_hz <= 0.0f)
        update_rate_hz = 10000.0f; // fallback: 10kHz
    spwm->dt_s = 1.0f / update_rate_hz;
}

void SPWM_SetOpenloop(SPWM_t *spwm,
                      float elec_freq_hz,
                      float modulation)
{
    spwm->elec_freq_hz = elec_freq_hz;
    spwm->modulation = clampf(modulation, 0.0f, 0.98f);
}

/**
 * 正弦开环（SPWM）：
 *  Va = m * sin(θ)
 *  Vb = m * sin(θ - 2π/3)
 *  Vc = m * sin(θ + 2π/3)
 * 映射到占空比：
 *  duty = 0.5 + 0.5 * Va   (保证在[0,1]内, 当 m ≤ 1)
 * 对应 CCR = duty * ARR
 *
 * 注意：互补与死区由定时器硬件处理，这里只写 CCR1/2/3。
 */
void SPWM_Update(SPWM_t *spwm)
{
    if (!spwm || !spwm->htim)
        return;

    // 1) 推进电角度：θ(k+1) = θ(k) + 2π * f_elec * dt
    spwm->theta += 2.0f * (float)M_PI * spwm->elec_freq_hz * spwm->dt_s;
    spwm->theta = wrap_angle_0_2pi(spwm->theta);

    // 2) 生成三相正弦（电压归一化）
    const float m = spwm->modulation; // 0..0.98
    const float th = spwm->theta;
    const float th_b = th - 2.0f * (float)M_PI / 3.0f;
    const float th_c = th + 2.0f * (float)M_PI / 3.0f;

    const float va = m * sinf(th);
    const float vb = m * sinf(th_b);
    const float vc = m * sinf(th_c);

    // 3) 归一化到占空比 [0,1]
    float du_a = 0.5f + 0.5f * va;
    float du_b = 0.5f + 0.5f * vb;
    float du_c = 0.5f + 0.5f * vc;

    du_a = clampf(du_a, 0.0f, 1.0f);
    du_b = clampf(du_b, 0.0f, 1.0f);
    du_c = clampf(du_c, 0.0f, 1.0f);

    // 4) 写入 CCR（与计数模式无关；中心对齐时HAL内部会处理翻转）
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(spwm->htim); // 例如 4999
    uint32_t ccr1 = (uint32_t)(du_a * (float)arr);
    uint32_t ccr2 = (uint32_t)(du_b * (float)arr);
    uint32_t ccr3 = (uint32_t)(du_c * (float)arr);

    __HAL_TIM_SET_COMPARE(spwm->htim, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(spwm->htim, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(spwm->htim, TIM_CHANNEL_3, ccr3);
}
