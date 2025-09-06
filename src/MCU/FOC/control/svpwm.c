#include "svpwm.h"
#include <math.h>

/* ---------- 常量/工具 ---------- */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi ? hi : x);
}

/* 线性斜坡（每步增加/减少不超过 limit_per_s*dt） */
static inline float slew(float cur, float tgt, float limit_per_s, float dt)
{
    if (limit_per_s <= 0.f)
        return tgt;
    float step = limit_per_s * dt;
    if (cur < tgt - step)
        return cur + step;
    if (cur > tgt + step)
        return cur - step;
    return tgt;
}

/* min–max 零序注入：ABC → duty(0..1) */
static void svpwm_abc_to_duty(float va, float vb, float vc,
                              float *du_a, float *du_b, float *du_c)
{
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

    /* 与当前 TIM1 设置对应：中心对齐 20 kHz */
    h->update_rate_hz = 20000.0f;
    h->dt_s = 1.0f / h->update_rate_hz;

    h->mode = SVPWM_REF_OPENLOOP_SINE;

    h->elec_freq_hz = 0.0f; /* 由状态机推进 */
    h->modulation = 0.2f;   /* 启动阶段较小 */
    h->theta = 0.0f;

    h->v_alpha = 0.0f;
    h->v_beta = 0.0f;

    h->duty_a = 0.5f;
    h->duty_b = 0.5f;
    h->duty_c = 0.5f;

    /* 工程参数默认值 */
    h->pole_pairs = 2;    /* 4 极电机默认 p=2，可按需改 */
    h->v_align = 0.20f;   /* 对齐幅值（归一化） */
    h->t_align_s = 0.30f; /* 对齐时间 0.3 s */

    h->fe_target_hz = 100.0f;
    h->fe_slew_hz_per_s = 400.0f; /* 频率斜坡 400 Hz/s */
    h->m_target = 0.50f;
    h->m_slew_per_s = 1.5f; /* m 斜坡 1.5 / s */

    h->vdc = 0.0f;
    h->k_vph = 0.59f; /* 经验系数，可微调为 0.58~0.61 */

    h->state = SVPWM_STATE_STOP;
    h->t_in_state = 0.0f;

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
        update_rate_hz = 20000.0f; /* 本工程 20kHz */

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
        /* 缓存 ARR，用于 duty→CCR 映射 */
        h->arr = __HAL_TIM_GET_AUTORELOAD(h->htim);

        /* 启动三路 PWM 主通道；互补/死区请在 TIM 初始化中单独启动 N 通道 */
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
    h->fe_target_hz = elec_freq_hz;
    h->m_target = modulation;
}

void SVPWM_SetAlphaBeta(SVPWM_Handle *h, float v_alpha, float v_beta)
{
    if (!h)
        return;
    h->mode = SVPWM_REF_ALPHABETA;
    h->v_alpha = v_alpha;
    h->v_beta = v_beta;
}

void SVPWM_Step(SVPWM_Handle *h)
{
    if (!h)
        return;

    float va, vb, vc;

    if (h->mode == SVPWM_REF_OPENLOOP_SINE)
    {
        /* 推进电角度：使用当前 elec_freq_hz 和 modulation */
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
        /* αβ 直接给定 → ABC */
        inv_clarke(h->v_alpha, h->v_beta, &va, &vb, &vc);
    }

    /* ABC → 占空比 */
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
}

/* =============== 状态机/辅助 API =============== */

void SVPWM_SetSpeedRPM(SVPWM_Handle *h, float rpm)
{
    if (!h)
        return;
    /* fe = pole_pairs * rpm / 60 */
    float fe = (float)h->pole_pairs * rpm / 60.0f;
    if (fe < 0.f)
        fe = 0.f;
    h->fe_target_hz = fe;
}

/* 从目标相电压峰值与 Vdc 估算调制度（经验） */
float SVPWM_ComputeM_FromVphase(float Vdc, float Vph_peak)
{
    if (Vdc <= 0.f)
        return 0.f;
    const float k = 0.59f; /* 与零序策略/硬件死区有关，0.58~0.61 之间微调 */
    float m = Vph_peak / (k * Vdc);
    if (m < 0.f)
        m = 0.f;
    if (m > 0.95f)
        m = 0.95f;
    return m;
}

void SVPWM_StartSequence(SVPWM_Handle *h,
                         float v_align, float t_align_s,
                         float fe_target_hz, float fe_slew_hz_per_s,
                         float m_target, float m_slew_per_s)
{
    if (!h)
        return;

    h->v_align = clampf(v_align, 0.05f, 0.8f);
    h->t_align_s = (t_align_s > 0.05f) ? t_align_s : 0.05f;

    h->fe_target_hz = (fe_target_hz >= 0.f) ? fe_target_hz : 0.f;
    h->fe_slew_hz_per_s = (fe_slew_hz_per_s > 0.f) ? fe_slew_hz_per_s : 200.f;

    h->m_target = clampf(m_target, 0.f, 0.95f);
    h->m_slew_per_s = (m_slew_per_s > 0.f) ? m_slew_per_s : 1.0f;

    /* 进入 ALIGN：固定 α 轴向量；频率清零，从较小 m 起步 */
    h->mode = SVPWM_REF_ALPHABETA;
    h->v_alpha = h->v_align; /* 对齐沿 α 轴 */
    h->v_beta = 0.0f;

    h->elec_freq_hz = 0.0f;
    h->modulation = clampf(h->v_align, 0.10f, 0.30f); /* 用 v_align 近似初始 m */

    h->theta = 0.0f; /* 复位角度，便于转 RUN 时相位自然 */

    h->state = SVPWM_STATE_ALIGN;
    h->t_in_state = 0.0f;
}

void SVPWM_TaskStep(SVPWM_Handle *h)
{
    if (!h)
        return;

    /* 累计状态驻留时间 */
    h->t_in_state += h->dt_s;

    switch (h->state)
    {
    default:
    case SVPWM_STATE_STOP:
        /* 空闲：输出保持 50% */
        h->mode = SVPWM_REF_ALPHABETA;
        h->v_alpha = 0.0f;
        h->v_beta = 0.0f;
        h->modulation = 0.0f;
        break;

    case SVPWM_STATE_ALIGN:
        /* 固定 α 轴向量，吸住转子。 */
        h->mode = SVPWM_REF_ALPHABETA;
        h->v_alpha = h->v_align;
        h->v_beta = 0.0f;
        h->elec_freq_hz = 0.0f;

        /* 到时长后切 RAMP */
        if (h->t_in_state >= h->t_align_s)
        {
            h->state = SVPWM_STATE_RAMP;
            h->t_in_state = 0.0f;
            /* 切回 OPENLOOP 模式，从低 m/fe 开始爬升 */
            h->mode = SVPWM_REF_OPENLOOP_SINE;
            /* 这里保持 theta 连贯（不重置），使转场更平滑 */
        }
        break;

    case SVPWM_STATE_RAMP:
        /* 频率/调制度斜坡推进到目标 */
        h->elec_freq_hz = slew(h->elec_freq_hz, h->fe_target_hz, h->fe_slew_hz_per_s, h->dt_s);
        h->modulation = slew(h->modulation, h->m_target, h->m_slew_per_s, h->dt_s);

        /* 到达目标后进入 RUN */
        if (fabsf(h->elec_freq_hz - h->fe_target_hz) < 1e-3f &&
            fabsf(h->modulation - h->m_target) < 1e-3f)
        {
            h->state = SVPWM_STATE_RUN;
            h->t_in_state = 0.0f;
        }
        break;

    case SVPWM_STATE_RUN:
        /* 正常运行：外部可随时改 fe_target_hz / m_target，TaskStep 按斜坡跟随 */
        h->elec_freq_hz = slew(h->elec_freq_hz, h->fe_target_hz, h->fe_slew_hz_per_s, h->dt_s);
        h->modulation = slew(h->modulation, h->m_target, h->m_slew_per_s, h->dt_s);
        break;
    }

    /* 一次底层计算与写 CCR */
    SVPWM_Step(h);
}
