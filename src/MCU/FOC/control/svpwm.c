#include "svpwm.h"
#include <math.h>

/* ---------- 数学/工具 ---------- */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
// 传入句柄用于打印调试
extern UART_HandleTypeDef huart1;
static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi ? hi : x);
}

/* min–max 零序注入：ABC → duty(0..1) */
static void svpwm_abc_to_duty(float va, float vb, float vc,
                              float *du_a, float *du_b, float *du_c)
{
    /* 轻微裁剪，避免极端输入 */
    const float va_c = clampf(va, -1.2f, 1.2f);
    const float vb_c = clampf(vb, -1.2f, 1.2f);
    const float vc_c = clampf(vc, -1.2f, 1.2f);

    float vmax = va_c, vmin = va_c;
    if (vb_c > vmax)
        vmax = vb_c;
    if (vc_c > vmax)
        vmax = vc_c;
    if (vb_c < vmin)
        vmin = vb_c;
    if (vc_c < vmin)
        vmin = vc_c;

    const float v0 = -0.5f * (vmax + vmin);

    const float ua = va_c + v0;
    const float ub = vb_c + v0;
    const float uc = vc_c + v0;

    float da = 0.5f + 0.5f * ua;
    float db = 0.5f + 0.5f * ub;
    float dc = 0.5f + 0.5f * uc;

    *du_a = clampf(da, 0.0f, 1.0f);
    *du_b = clampf(db, 0.0f, 1.0f);
    *du_c = clampf(dc, 0.0f, 1.0f);
}

/* 逆 Clarke：αβ → ABC */
static void inv_clarke(float v_alpha, float v_beta,
                       float *va, float *vb, float *vc)
{
    const float SQRT3_2 = 0.8660254037844386f; // √3/2
    *va = v_alpha;
    *vb = -0.5f * v_alpha + SQRT3_2 * v_beta;
    *vc = -0.5f * v_alpha - SQRT3_2 * v_beta;
}

/* ---------- 对外 API 实现 ---------- */

void SVPWM_Defaults(SVPWM_Handle *h)
{
    if (!h)
        return;
    h->update_rate_hz = 20000.0f;
    h->dt_s = 1.0f / h->update_rate_hz;

    h->mode = SVPWM_REF_OPENLOOP_SINE;

    h->elec_freq_hz = 100.0f;
    h->modulation = 0.7f;
    h->theta = 0.0f;

    h->v_alpha = 0.0f;
    h->v_beta = 0.0f;

    h->duty_a = 0.5f;
    h->duty_b = 0.5f;
    h->duty_c = 0.5f;

#ifndef SVPWM_NO_HAL
    h->htim = NULL;
    h->ch_a = 0;
    h->ch_b = 0;
    h->ch_c = 0;
    h->arr = 0;
    h->started = 0;
#endif
}

#ifndef SVPWM_NO_HAL
void SVPWM_AttachTimer(SVPWM_Handle *h,
                       TIM_HandleTypeDef *htim,
                       uint32_t ch_a, uint32_t ch_b, uint32_t ch_c)
{
    if (!h)
        return;
    h->htim = htim;
    h->ch_a = ch_a;
    h->ch_b = ch_b;
    h->ch_c = ch_c;
}
#endif

void SVPWM_Init(SVPWM_Handle *h, float update_rate_hz, float init_theta_rad)
{
    if (!h)
        return;
    if (update_rate_hz <= 0.0f)
        update_rate_hz = 20000.0f;

    h->update_rate_hz = update_rate_hz;
    h->dt_s = 1.0f / update_rate_hz;
    h->theta = init_theta_rad;
}

void SVPWM_Start(SVPWM_Handle *h)
{
    if (!h)
        return;

#ifndef SVPWM_NO_HAL
    if (h->htim)
    {
        /* 读取 ARR，用于 duty→CCR 映射 */
        h->arr = __HAL_TIM_GET_AUTORELOAD(h->htim);

        /* 启动三路 PWM（如需要互补/死区，请确保 TIM 已按需配置） */
        HAL_TIM_PWM_Start(h->htim, h->ch_a);
        HAL_TIM_PWM_Start(h->htim, h->ch_b);
        HAL_TIM_PWM_Start(h->htim, h->ch_c);

        h->started = 1;
    }
#endif
}

void SVPWM_SetOpenloop(SVPWM_Handle *h, float elec_freq_hz, float modulation)
{
    if (!h)
        return;
    h->mode = SVPWM_REF_OPENLOOP_SINE;
    h->elec_freq_hz = elec_freq_hz;
    h->modulation = modulation;
}

void SVPWM_Step(SVPWM_Handle *h)
{
    if (!h)
        return;

    // static uint32_t dbg_cnt = 0;  // 打印分频计数
    // const uint32_t dbg_div = 100; // 每 100 次打印一次

    float va, vb, vc;

    if (h->mode == SVPWM_REF_OPENLOOP_SINE)
    {
        h->theta += 2.0f * (float)M_PI * h->elec_freq_hz * h->dt_s;
        if (h->theta > 2.0f * (float)M_PI)
            h->theta -= 2.0f * (float)M_PI;

        const float m = h->modulation;
        const float th = h->theta;
        va = m * sinf(th);
        vb = m * sinf(th - 2.0f * (float)M_PI / 3.0f);
        vc = m * sinf(th + 2.0f * (float)M_PI / 3.0f);
    }
    else
    {
        inv_clarke(h->v_alpha, h->v_beta, &va, &vb, &vc);
    }

    svpwm_abc_to_duty(va, vb, vc, &h->duty_a, &h->duty_b, &h->duty_c);

#ifndef SVPWM_NO_HAL
    if (h->htim && h->started)
    {
        uint32_t c1 = SVPWM_DutyToCCR(h->arr, h->duty_a);
        uint32_t c2 = SVPWM_DutyToCCR(h->arr, h->duty_b);
        uint32_t c3 = SVPWM_DutyToCCR(h->arr, h->duty_c);
        __HAL_TIM_SET_COMPARE(h->htim, h->ch_a, c1);
        __HAL_TIM_SET_COMPARE(h->htim, h->ch_b, c2);
        __HAL_TIM_SET_COMPARE(h->htim, h->ch_c, c3);
    }
#endif

    // ========= 在这里增加打印 =========
    // if (++dbg_cnt >= dbg_div)
    // {
    //     dbg_cnt = 0;
    //     char buf[64];
    //     int n = snprintf(buf, sizeof(buf), "va=%.3f, vb=%.3f, vc=%.3f\r\n", va, vb, vc);
    //     if (n > 0)
    //     {
    //         // 注意：Transmit_IT 是非阻塞的，如果前一次还没发完，这里会返回 HAL_BUSY
    //         if (HAL_UART_Transmit_IT(&huart1, (uint8_t *)buf, n) != HAL_OK)
    //         {
    //             // 根据需要：丢弃、等待或排队
    //         }
    //     }
    // }
}
