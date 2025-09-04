#ifndef __SVPWM_H__
#define __SVPWM_H__

/**
 * 设计思路
 * - 统一用一个句柄 SVPWM_Handle 管理：输入模式、频率、调制度、定时器句柄/通道、ARR、运行状态等。
 * - 对上层暴露简单 API：AttachTimer / Init / Start / Step / SetOpenloop / SetAlphaBeta
 * - Step() 里自动完成：推进角度(若开环) → 参考生成 → SVPWM(min-max零序注入) → duty→CCR
 * - 参考量为归一化值，建议范围 [-1, +1]；最终 duty 严格钳位到 [0, 1]
 * - 若不想依赖 HAL：可定义 SVPWM_NO_HAL，则仅计算 duty，不写 CCR（可自行读取 duty）
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ============ 选择是否使用 HAL ============ */
/* 如需纯数学、无硬件依赖，在编译选项里加 -DSVPWM_NO_HAL */
#ifndef SVPWM_NO_HAL
/* 让工程包含 CubeMX 生成的 tim.h（其中声明了 TIM_HandleTypeDef） */
#include "tim.h"
#endif

    /* ============ 输入模式 ============ */
    typedef enum
    {
        SVPWM_REF_OPENLOOP_SINE = 0, /* 开环正弦：由 elec_freq_hz + modulation 生成 ABC 正弦 */
        SVPWM_REF_ALPHABETA = 1,     /* 闭环/外部给定：直接给 v_alpha, v_beta */
    } SVPWM_InputMode;

    /* ============ 句柄 ============ */
    typedef struct
    {
        /* ---- 时序/更新 ---- */
        float update_rate_hz; /* Step 调用频率（建议=PWM更新频率） */
        float dt_s;           /* = 1 / update_rate_hz */

        /* ---- 模式与参考 ---- */
        SVPWM_InputMode mode;

        /* 开环参数（仅在 SVPWM_REF_OPENLOOP_SINE 生效） */
        float elec_freq_hz; /* 电角频率 Hz */
        float modulation;   /* 调制度，建议 0..1，线性区 ≲ 1 */
        float theta;        /* 电角度 rad，内部推进 */

        /* αβ 参考（在 SVPWM_REF_ALPHABETA 模式下由上层更新） */
        float v_alpha;
        float v_beta;

        /* ---- 输出占空比（对外可读） ---- */
        float duty_a;
        float duty_b;
        float duty_c;

#ifndef SVPWM_NO_HAL
        /* ---- 定时器硬件资源 ---- */
        TIM_HandleTypeDef *htim;
        uint32_t ch_a;
        uint32_t ch_b;
        uint32_t ch_c;
        uint32_t arr; /* 自动重装值，用于 duty→CCR 映射 */

        /* 内部状态：是否已启动 PWM 输出 */
        uint8_t started;
#endif
    } SVPWM_Handle;

    /* ============ 对外 API ============ */

    /** 设置为默认值（安全初值），不触碰硬件 */
    void SVPWM_Defaults(SVPWM_Handle *h);

/** 绑定定时器与三相通道（需要 HAL 时使用；纯数学模式可不调用） */
#ifndef SVPWM_NO_HAL
    void SVPWM_AttachTimer(SVPWM_Handle *h,
                           TIM_HandleTypeDef *htim,
                           uint32_t ch_a, uint32_t ch_b, uint32_t ch_c);
#endif

    /** 基本初始化：设置更新频率（Step() 调用频率）与初相位 */
    void SVPWM_Init(SVPWM_Handle *h, float update_rate_hz, float init_theta_rad);

    /** 开始输出（启动 PWM 通道，并缓存 ARR）。纯数学模式下为空操作 */
    void SVPWM_Start(SVPWM_Handle *h);

    /** 设为“开环正弦”模式 */
    void SVPWM_SetOpenloop(SVPWM_Handle *h, float elec_freq_hz, float modulation);

    /** 设为“αβ直接给定”模式（闭环/上层外给），并更新一次参考 */
    void SVPWM_SetAlphaBeta(SVPWM_Handle *h, float v_alpha, float v_beta);

    /** 每次调用执行一次周期性更新：生成参考 → SVPWM → duty →(可选) 写CCR */
    void SVPWM_Step(SVPWM_Handle *h);

    /** 可选：仅做 duty→CCR 的映射（0..ARR） */
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
