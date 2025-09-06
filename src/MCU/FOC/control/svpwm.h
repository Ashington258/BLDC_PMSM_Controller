#ifndef __SVPWM_H__
#define __SVPWM_H__

/**
 * SVPWM + 启动状态机（ALIGN → RAMP → RUN）
 * - Step()：底层一次性计算（推进角度/逆Clarke/零序注入 → duty → CCR）
 * - TaskStep()：上层任务（处理启动状态机与斜坡），内部会调用 Step()
 * - 计时与更新频率：请保证 TaskStep()/Step() 的调用频率 == PWM 更新频率（本工程为 20 kHz）
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ============ 是否使用 HAL ============ */
#ifndef SVPWM_NO_HAL
#include "tim.h"
#endif

    /* ============ 输入模式 ============ */
    typedef enum
    {
        SVPWM_REF_OPENLOOP_SINE = 0, /* 开环正弦：elec_freq_hz + modulation 生成 ABC 正弦 */
        SVPWM_REF_ALPHABETA = 1,     /* 直接给 v_alpha, v_beta */
    } SVPWM_InputMode;

    /* ============ 启动状态机 ============ */
    typedef enum
    {
        SVPWM_STATE_STOP = 0,  /* 停止/空闲 */
        SVPWM_STATE_ALIGN = 1, /* 定子定向（对齐） */
        SVPWM_STATE_RAMP = 2,  /* 软启斜坡 */
        SVPWM_STATE_RUN = 3,   /* 正常运行 */
    } SVPWM_RunState;

    typedef struct
    {
        /* ---- 时序/更新 ---- */
        float update_rate_hz; /* 调用频率（= PWM 更新频率），本工程 20kHz */
        float dt_s;           /* = 1 / update_rate_hz */

        /* ---- 模式与参考 ---- */
        SVPWM_InputMode mode;

        /* 开环参数（OPENLOOP 模式） */
        float elec_freq_hz; /* 目标/当前电角频率 Hz（由状态机斜坡/设置接口更新） */
        float modulation;   /* 调制度 m，建议 0..0.95 */
        float theta;        /* 电角度 rad */

        /* 直接 αβ 参考（ALPHABETA 模式） */
        float v_alpha;
        float v_beta;

        /* ---- 输出占空比（可读） ---- */
        float duty_a;
        float duty_b;
        float duty_c;

        /* ---- 工程参数 ---- */
        int pole_pairs;  /* 极对数（用于 rpm→fe 映射） */
        float v_align;   /* 对齐阶段 α 轴固定电压的“归一化幅值”，建议 0.15~0.3 */
        float t_align_s; /* 对齐时间，典型 0.2~0.5 s */

        /* 斜坡/目标（RAMP/RUN 使用） */
        float fe_target_hz;     /* 目标电角频率 Hz */
        float fe_slew_hz_per_s; /* 频率斜坡，典型 200~1000 Hz/s */
        float m_target;         /* 目标调制度 */
        float m_slew_per_s;     /* m 的斜坡，典型 1.0 / s */

        /* 观测与前馈（可选） */
        float vdc;   /* 实测母线电压（V），若不用可设 0 */
        float k_vph; /* SVPWM 下 Vph_peak ≈ k_vph * Vdc * m，经验 0.58~0.61 */

        /* ---- 状态机内部 ---- */
        SVPWM_RunState state;
        float t_in_state; /* 进入该状态后的累计时间（s） */

#ifndef SVPWM_NO_HAL
        /* ---- 定时器资源 ---- */
        TIM_HandleTypeDef *htim;
        uint32_t ch_a;
        uint32_t ch_b;
        uint32_t ch_c;
        uint32_t arr; /* 自动重装值（duty→CCR 映射） */
        uint8_t started;
#endif
    } SVPWM_Handle;

    /* ============ 对外 API ============ */

    /** 默认安全初值；不会触碰硬件 */
    void SVPWM_Defaults(SVPWM_Handle *h);

/** 绑定定时器与三相通道（使用 HAL 时） */
#ifndef SVPWM_NO_HAL
    void SVPWM_AttachTimer(SVPWM_Handle *h,
                           TIM_HandleTypeDef *htim,
                           uint32_t ch_a, uint32_t ch_b, uint32_t ch_c);
#endif

    /** 基本初始化（设定更新频率与初始角度） */
    void SVPWM_Init(SVPWM_Handle *h, float update_rate_hz, float init_theta_rad);

    /** 启动 PWM（启动三个主通道；若需要互补/死区，请在 TIM 初始化中配置并自行开启 N 通道） */
    void SVPWM_Start(SVPWM_Handle *h);

    /** 开环正弦模式：设定电角频率与调制度（立即生效为“目标值”） */
    void SVPWM_SetOpenloop(SVPWM_Handle *h, float elec_freq_hz, float modulation);

    /** 直接给定 αβ（立即覆盖为当前参考） */
    void SVPWM_SetAlphaBeta(SVPWM_Handle *h, float v_alpha, float v_beta);

    /** 仅做一次底层运算与写 CCR（不含状态机） */
    void SVPWM_Step(SVPWM_Handle *h);

    /** =========== 启动状态机相关 =========== */

    /** 设置极对数（rpm→fe 映射需要） */
    static inline void SVPWM_SetPolePairs(SVPWM_Handle *h, int pole_pairs)
    {
        if (!h)
            return;
        h->pole_pairs = (pole_pairs > 0) ? pole_pairs : 1;
    }

    /** rpm → 目标电角频率；只更新目标，不立刻跳变（由斜坡推进） */
    void SVPWM_SetSpeedRPM(SVPWM_Handle *h, float rpm);

    /** 根据 Vdc 与目标相电压峰值估算目标调制度（并限幅） */
    float SVPWM_ComputeM_FromVphase(float Vdc, float Vph_peak);

    /** 更新母线电压；用于前馈计算或遥测 */
    static inline void SVPWM_UpdateVdc(SVPWM_Handle *h, float Vdc)
    {
        if (!h)
            return;
        h->vdc = (Vdc > 0.f) ? Vdc : 0.f;
    }

    /** 进入启动序列：ALIGN→RAMP→RUN */
    void SVPWM_StartSequence(SVPWM_Handle *h,
                             float v_align, float t_align_s,
                             float fe_target_hz, float fe_slew_hz_per_s,
                             float m_target, float m_slew_per_s);

    /** 周期调用（与 PWM 同频），内部处理状态机并调用 Step() */
    void SVPWM_TaskStep(SVPWM_Handle *h);

    /** duty→CCR 映射（0..ARR） */
    static inline uint32_t SVPWM_DutyToCCR(uint32_t arr, float duty)
    {
        if (duty < 0.0f)
            duty = 0.0f;
        if (duty > 1.0f)
            duty = 1.0f;
        return (uint32_t)((float)arr * duty);
    }

#ifdef __cplusplus
}
#endif

#endif /* __SVPWM_H__ */
