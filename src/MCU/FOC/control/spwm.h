#ifndef __SPWM_H__
#define __SPWM_H__

#include "main.h"
#include <stdint.h>

//-------------调用示例-------------//
// 0. 声明结构体
// static SPWM_t spwm;

// 1. 初始化
// // 已知你的 PWM 例子是 20kHz，则让 SPWM_Update 也以 20kHz 调用
// SPWM_Init(&spwm, &htim1, /*update_rate_hz=*/20000.0f, /*init_theta=*/0.0f);
// // 设定开环：电角频率 100Hz，调制度 0.5（可根据母线电压与负载微调）
// SPWM_SetOpenloop(&spwm, 100.0f, 0.5f);

// 2. 主循环中启动更新
// SPWM_Update(&spwm);

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        TIM_HandleTypeDef *htim; // 绑定的高级定时器（已配置好3相互补PWM与死区）
        float theta;             // 电角度(弧度)
        float elec_freq_hz;      // 电角频率(Hz)，开环由外部设置
        float modulation;        // 调制度 m ∈ [0, 0.98]，正弦幅值
        float dt_s;              // 单次更新步长(秒)，= 1/调用频率(建议=PWM频率)
    } SPWM_t;

    /**
     * @brief 初始化开环SPWM对象（不启动PWM，只建模参数）
     * @param spwm      SPWM对象
     * @param htim     已配置好的TIM句柄(例如 &htim1)
     * @param update_rate_hz  SPWM_Update() 的调用频率(Hz)，建议=PWM开关频率
     * @param init_theta_rad  初始电角度(弧度)
     */
    void SPWM_Init(SPWM_t *spwm,
                   TIM_HandleTypeDef *htim,
                   float update_rate_hz,
                   float init_theta_rad);

    /**
     * @brief 设置开环参数：电角频率与调制度
     * @param spwm            SPWM对象
     * @param elec_freq_hz   电角频率(Hz)，>0 正转，<0 反转
     * @param modulation     调制度 m ∈ [0, 0.98]（正弦幅值，0=断电，~0.8起有力矩）
     */
    void SPWM_SetOpenloop(SPWM_t *spwm,
                          float elec_freq_hz,
                          float modulation);

    /**
     * @brief 开环更新：推进角度并刷新三相占空比（需以固定周期调用）
     *        建议在与PWM同频的中断/回调里调用（比如定时器更新中断）。
     *        仅更新CCR，不负责启动/停止PWM。
     */
    void SPWM_Update(SPWM_t *spwm);

#ifdef __cplusplus
}
#endif

#endif /* __SPWM_H__ */
