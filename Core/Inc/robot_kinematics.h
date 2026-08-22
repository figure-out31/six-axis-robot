#ifndef __ROBOT_KINEMATICS_H__
#define __ROBOT_KINEMATICS_H__

#include "robot.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOT_KINEMATICS_RESULT_NUM   (4U)   /* 逆解结果数量（4组候选解） */
#define T_ROW_COL                     (4U)   /* 变换矩阵行列数（4x4齐次矩阵） */

/**
 * @struct robot_kinematics
 * @brief 运动学解算器全局结构体
 *
 * 内部维护目标变换矩阵 T、4组候选逆解 result，
 * 以及 result_invalid_mask 标记哪些解超出关节范围无效。
 */
struct robot_kinematics {
    float T[T_ROW_COL][T_ROW_COL];                                  /* 当前目标位姿的4x4齐次变换矩阵 */
    float result[ROBOT_KINEMATICS_RESULT_NUM][ROBOT_MAX_JOINT_NUM]; /* 4组逆解结果（每组6个关节角度，单位：弧度/度） */
    uint32_t result_invalid_mask;                                   /* 逆解无效掩码，位i=1表示第i组解无效（超出关节范围） */
};

/* ============================================================
 *   运动学接口函数
 * ============================================================ */

/**
 * @brief 根据绝对位姿（位置 + RPY姿态），计算目标变换矩阵 T_out
 *
 * 将末端绝对位置 (x,y,z) 和绝对姿态 RPY固定角 (roll,pitch,yaw) 组装为
 * 4x4 齐次变换矩阵。RPY 采用 ZYX 固定角约定：R = Rz(yaw)·Ry(pitch)·Rx(roll)。
 *
 * @param x,y,z       末端在基座坐标系的绝对位置（mm）
 * @param roll         绕X轴旋转角（度）
 * @param pitch        绕Y轴旋转角（度）
 * @param yaw          绕Z轴旋转角（度）
 * @param T_out        输出的目标变换矩阵
 */
void robot_kinematics_cal_T_pose(float x, float y, float z,
                                  float roll, float pitch, float yaw,
                                  float T_out[4][4]);

/**
 * @brief 运动学正解：给定6个关节角，计算末端 4x4 齐次变换矩阵
 *
 * 使用标准 DH 变换链 T0_6 = T0_1·T1_2·T2_3·T3_4·T4_5·T5_6。
 * 约定 R = Rx(alpha)·Rz(theta)（与 MATLAB 推导一致）。
 *
 * @param theta  6个关节角（弧度，即 DH 的 theta 值）
 * @param T      输出的末端位姿 4x4 齐次变换矩阵
 */
void robot_kinematics_fk(const float theta[6], float T[4][4]);

/** @brief 打印当前所有逆解结果（调试用） */
void robot_kinematics_show_result(void);

/** @brief 打印 4x4 变换矩阵（调试用） */
void robot_kinematics_show_T(float T[4][4]);

/**
 * @brief 运动学逆解主函数
 *
 * 输入目标位姿变换矩阵 T_target，输出最优的一组关节角度 result。
 * 流程： 计算θ3→θ2→θ1→θ5→θ4→θ6 → 弧度转角度 → 关节范围映射 → 选择最优解。
 *
 * @param T_target  输入的4x4目标变换矩阵（按行优先存储为16个float）
 * @param result    输出的最优关节角度数组（6个float，单位：度）
 * @param show      是否打印调试信息（0=不打印, 非0=打印）
 * @return 成功返回 0，无有效解返回 -1
 */
int robot_kinematics_inverse(float *T_target, float *result, int show);

/**
 * @brief 按关节ID更新当前角度（用于最优解选择时作为参考）
 * @param joint_id  关节ID（0~5）
 * @param angle     当前角度（度）
 */
void robot_kinematics_joint_angle_update_by_id(uint32_t joint_id, float angle);

/**
 * @brief 批量更新当前关节角度（用于最优解选择时作为参考）
 * @param joint_angle  6个关节的当前角度数组（度）
 */
void robot_kinematics_joint_angle_update(float *joint_angle);

#ifdef __cplusplus
}
#endif

#endif /* __ROBOT_KINEMATICS_H__ */
