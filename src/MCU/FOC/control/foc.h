#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        /* 采样/时序 */
        float dt_s;         // 20 kHz：0.00005
        uint32_t decim_spd; // 速度环分频（例如 20 → 1 kHz）
        uint32_t cnt_spd;

        /* 电机/母线 */
        uint8_t pole_pairs;
        float vdc; // 实测直流母线电压（V）

        /* 目标与状态 */
        float rpm_ref;
        float rpm_meas;

        /* PI 参数（按需微调） */
        float kp_id, ki_id;   // 电流 d 轴
        float kp_iq, ki_iq;   // 电流 q 轴
        float kp_spd, ki_spd; // 速度环

        /* PI 积分器与限幅 */
        float int_id, int_iq, int_spd;
        float id_ref, iq_ref;
        float id_meas, iq_meas;
        float vd_cmd, vq_cmd;
        float v_lim; // 相电压幅值限幅（V）

        /* 观测/角度 */
        float theta_e;        // 输入：电角度（rad）
        float sin_th, cos_th; // 预存

        /* αβ → SVPWM */
        float v_alpha, v_beta;

    } FOC_Ctrl;

    /* API */
    void FOC_Init(FOC_Ctrl *f, float fs_hz, uint8_t pole_pairs, float vdc);
    void FOC_SetSpeedRef(FOC_Ctrl *f, float rpm);
    void FOC_UpdateAngle(FOC_Ctrl *f, float theta_e);
    void FOC_UpdateVdc(FOC_Ctrl *f, float vdc);
    void FOC_StepCurrentLoop(FOC_Ctrl *f);
    int FOC_ShouldRunSpeedLoop(FOC_Ctrl *f);
    void FOC_StepSpeedLoop(FOC_Ctrl *f);

    /* 由你实现（弱符号占位） */
    void PhaseCurrent_Read_ABC(float *ia, float *ib, float *ic);
    float BusVoltage_Read(void);
    float SpeedFeedback_FromHall_RPM(void);

#ifdef __cplusplus
}
#endif
