#include "foc.h"
#include <math.h>

/* ============ 工具 ============ */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
static inline float clampf(float x, float lo, float hi) { return (x < lo) ? lo : ((x > hi) ? hi : x); }

/* Clarke（A 相对 α） */
static inline void clarke(float ia, float ib, float ic, float *a, float *b)
{
    (void)ic; // ia+ib+ic≈0 时，可只用 ia,ib
    *a = ia;
    *b = (ia + 2.0f * ib) * (-0.5773502691896258f); // -1/√3
}

/* Park */
static inline void park(float a, float b, float s, float c, float *d, float *q)
{
    *d = c * a + s * b;
    *q = -s * a + c * b;
}

/* 逆 Park */
static inline void inv_park(float d, float q, float s, float c, float *a, float *b)
{
    *a = c * d - s * q;
    *b = s * d + c * q;
}

/* PI（带积分抗风up） */
static inline float pi_step(float err, float kp, float ki, float *inte, float out_min, float out_max, float dt)
{
    float u = kp * err + (*inte);
    float sat = clampf(u, out_min, out_max);
    /* 仅在未饱和或有利于脱饱和方向时积分 */
    float anti = (sat == u) ? 1.0f : 0.0f;
    *inte += anti * (ki * err * dt);
    return sat;
}

/* ============ API ============ */
void FOC_Init(FOC_Ctrl *f, float fs_hz, uint8_t pole_pairs, float vdc)
{
    f->dt_s = 1.0f / ((fs_hz > 0) ? fs_hz : 20000.0f);
    f->decim_spd = 20;
    f->cnt_spd = 0; // 每 20 次电流环（1 kHz）走一次速度环
    f->pole_pairs = pole_pairs;
    f->vdc = vdc;
    f->rpm_ref = 0.0f;
    f->rpm_meas = 0.0f;

    /* 经验 PI（起步值，后续可整定） */
    f->kp_id = 0.5f;
    f->ki_id = 200.0f;
    f->kp_iq = 0.5f;
    f->ki_iq = 200.0f;
    f->kp_spd = 0.02f;
    f->ki_spd = 0.8f;

    f->int_id = f->int_iq = f->int_spd = 0.0f;
    f->id_ref = 0.0f;
    f->iq_ref = 0.0f;
    f->id_meas = f->iq_meas = 0.0f;
    f->vd_cmd = f->vq_cmd = 0.0f;

    /* 相电压限幅 ~ 0.577*Vdc（SVPWM 最大线电压→相电压）再留裕量 */
    f->v_lim = 0.5f * vdc;

    f->theta_e = 0.0f;
    f->sin_th = 0.0f;
    f->cos_th = 1.0f;
    f->v_alpha = 0.0f;
    f->v_beta = 0.0f;
}

void FOC_SetSpeedRef(FOC_Ctrl *f, float rpm) { f->rpm_ref = rpm; }
void FOC_UpdateAngle(FOC_Ctrl *f, float theta_e)
{
    f->theta_e = theta_e;
    f->sin_th = sinf(theta_e);
    f->cos_th = cosf(theta_e);
}
void FOC_UpdateVdc(FOC_Ctrl *f, float vdc)
{
    if (vdc > 1.f)
    {
        f->vdc = vdc;
        f->v_lim = 0.5f * vdc;
    }
}

int FOC_ShouldRunSpeedLoop(FOC_Ctrl *f) { return (++f->cnt_spd >= f->decim_spd); }

void FOC_StepSpeedLoop(FOC_Ctrl *f)
{
    f->cnt_spd = 0;
    /* 速度反馈（来自霍尔的电角频率→机械 rpm） */
    f->rpm_meas = SpeedFeedback_FromHall_RPM();
    float err = f->rpm_ref - f->rpm_meas;
    /* 输出 Iq_ref（电流单位安培）——注意后续你要根据电机额定电流调整限幅 */
    float iq_ref = pi_step(err, f->kp_spd, f->ki_spd, &f->int_spd, -10.0f, +10.0f, f->dt_s * f->decim_spd);
    f->iq_ref = iq_ref;
    f->id_ref = 0.0f; /* 弱磁先不做 */
}

void FOC_StepCurrentLoop(FOC_Ctrl *f)
{
    /* 1) 读取三相电流（A） */
    float ia = 0, ib = 0, ic = 0;
    PhaseCurrent_Read_ABC(&ia, &ib, &ic);

    /* 2) Clarke → Park */
    float a, b;
    clarke(ia, ib, ic, &a, &b);
    float d, q;
    park(a, b, f->sin_th, f->cos_th, &d, &q);
    f->id_meas = d;
    f->iq_meas = q;

    /* 3) Id, Iq 双 PI（单位：电压） */
    float vd = pi_step(f->id_ref - d, f->kp_id, f->ki_id, &f->int_id, -f->v_lim, +f->v_lim, f->dt_s);
    float vq = pi_step(f->iq_ref - q, f->kp_iq, f->ki_iq, &f->int_iq, -f->v_lim, +f->v_lim, f->dt_s);

    /* 4) 逆 Park → αβ，相电压限幅（圆限幅） */
    float va, vb;
    inv_park(vd, vq, f->sin_th, f->cos_th, &va, &vb);
    float mag = sqrtf(va * va + vb * vb);
    float vmax = 0.577f * f->vdc; // SVPWM 可用相电压峰值
    if (mag > vmax && mag > 1e-6f)
    {
        float k = vmax / mag;
        va *= k;
        vb *= k;
    }

    f->v_alpha = va;
    f->v_beta = vb;
}

/* ============ 弱符号：把你的 ADC/Hall 接上即可 ============ */
__attribute__((weak)) void PhaseCurrent_Read_ABC(float *ia, float *ib, float *ic)
{
    /* TODO: 换成你的 ADC 结果（单位安培，去偏置，方向一致性） */
    *ia = 0.0f;
    *ib = 0.0f;
    *ic = 0.0f;
}
__attribute__((weak)) float BusVoltage_Read(void)
{
    /* TODO: 读取母线电压（例如 ADC 量测换算） */
    return 24.0f;
}
__attribute__((weak)) float SpeedFeedback_FromHall_RPM(void)
{
    /* TODO: 可直接返回你的霍尔模块计算的机械 rpm */
    return 0.0f;
}
