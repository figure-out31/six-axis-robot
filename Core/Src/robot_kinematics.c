/**
  ******************************************************************************
  * @file    robot_kinematics.c
  * @brief   6轴机械臂运动学逆解算法
  *
  *   基于 DH (Denavit-Hartenberg) 参数的解析法运动学逆解。
  *   给定末端位姿的 4x4 齐次变换矩阵 T，求解6个关节的角度。
  *
  *   求解顺序（按解析依赖关系）:
  *     θ3 → θ2 → θ1 → θ5 → θ4 → θ6
  *
  *   最多产生 4 组候选解，算法会:
  *     1. 过滤超出关节角度范围的无效解
  *     2. 选择与当前关节位置最接近的最优解（加权差值最小）
  *
  *   理论依据: robot_kinematics_sym_v3_0.m (MATLAB符号计算推导)
  ******************************************************************************
  */

#include "robot_kinematics.h"
#include "usart.h"
#include "robot.h"

/* 运动学解算器全局实例 */
static struct robot_kinematics g_robot_kinematics = {0};

/* 当前关节角度（用于最优解选择时的参考） */
static float g_current_joint_angle[ROBOT_MAX_JOINT_NUM] = {0};

/* ============================================================
 *    从 DH 参数中提取连杆常量（提高可读性）
 * ============================================================ */
#define a2 D_H[2][0]   /* 连杆2的长度（mm） */
#define a3 D_H[3][0]   /* 连杆3的长度（mm） */
#define d4 D_H[3][2]   /* 连杆4的偏距（mm） */

/* 前向声明：前 n 连杆正解（供腕部解算 R3_6 = R0_3^T·R0_6 与正解接口使用） */
static void robot_kinematics_fk_n(const float theta[6], int n, float T[4][4]);

/* ============================================================
 *    第3步: 计算 θ3（肘关节角度）
 *
 *    θ3 有两种解（对应肘关节的"上肘"和"下肘"构型），
 *    前4组候选解共享一个 θ3，后4组共享另一个 θ3。
 * ============================================================ */

/**
 * @brief 计算肘关节角度 θ3
 *
 * 从目标变换矩阵 T 中提取末端位置 p = (px, py, pz)，
 * 根据机械臂几何约束公式推导 θ3 的解析解。
 *
 * 解的结构:
 *   u_theta3 = -(2*a2*d4 ± sqrt(eq1)) / (const_eq2 + p²)
 *   theta3   = 2 * atan(u_theta3)
 *
 * 其中 eq1 包含 px,py,pz 的高次项和 DH 参数常数。
 */
static void robot_kinematics_calc_theta3(void)
{
    float px = g_robot_kinematics.T[0][3];
    float py = g_robot_kinematics.T[1][3];
    float pz = g_robot_kinematics.T[2][3];

    /* 预计算 DH 参数常数项 */
    float _2_a2_d4 = 2 * a2 * d4;
    float _2_pow_a2_2 = 2 * pow(a2, 2);
    float _2_pow_a3_2 = 2 * pow(a3, 2);
    float _2_pow_d4_2 = 2 * pow(d4, 2);

    /* 仅依赖 DH 参数的常数 */
    float const_eq1 = -pow(a2, 4) + _2_pow_a2_2 * (pow(a3, 2) + pow(d4, 2))
                - pow(a3, 4) - 2*pow(a3, 2)*pow(d4, 2) - pow(d4, 4);
    float const_eq2 = -pow(a2, 2) + 2*a2*a3 - pow(a3, 2) - pow(d4, 2);

    /* 含末端位置的项 */
    float pow_px_2 = pow(px, 2);
    float pow_py_2 = pow(py, 2);
    float pow_pz_2 = pow(pz, 2);
    float pow_distance_2 = pow_px_2 + pow_py_2 + pow_pz_2; // 末端到原点的距离平方

    /* 完整公式中的平方根内部表达式 */
    float eq1 = (const_eq1 + _2_pow_a2_2*pow_distance_2
        + _2_pow_a3_2*pow_distance_2 + _2_pow_d4_2*pow_distance_2
        - pow(px, 4) - pow(py, 4) - pow(pz, 4)
        - 2*pow_px_2*(pow_py_2 + pow_pz_2) - 2*pow_py_2*pow_pz_2);

    /* 两个解: u = -(2*a2*d4 + sqrt(eq1)) / (denominator)  和   -(2*a2*d4 - sqrt(eq1)) / (denominator) */
    float u_theta3_1 = -(_2_a2_d4 + sqrt(eq1)) / (const_eq2 + pow_distance_2);
    float u_theta3_2 = -(_2_a2_d4 - sqrt(eq1)) / (const_eq2 + pow_distance_2);

    /* u → θ: 万能公式逆变换, θ = 2*atan(u) */
    float theta3_1 = atan(u_theta3_1) * 2;
    float theta3_2 = atan(u_theta3_2) * 2;

    // 前4组(0~3)共享 theta3_1，后4组(4~7)共享 theta3_2
    for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM/2; i++) {
        g_robot_kinematics.result[i][ROBOT_JOINT_3] = theta3_1;
    }

    for (int i = ROBOT_KINEMATICS_RESULT_NUM/2; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
        g_robot_kinematics.result[i][ROBOT_JOINT_3] = theta3_2;
    }
}

/* ============================================================
 *    第2步: 计算 θ2（肩关节角度）
 *
 *    θ2 以 θ3 为输入，每个 θ3 又产生两个 θ2 分支解。
 * ============================================================ */

/**
 * @brief 根据 θ3 计算对应的两个 θ2 解
 *
 * 公式:
 *   u_theta2 = -(a2 ± sqrt(a2² + a3² + d4² + 2*a2*a3*cos(θ3) - 2*a2*d4*sin(θ3) - pz²) + a3*cos(θ3) - d4*sin(θ3))
 *            / (d4*cos(θ3) - pz + a3*sin(θ3))
 *   theta2   = 2 * atan(u_theta2)
 *
 * @param theta3    已计算的 θ3
 * @param theta2_1  输出: θ2 的第一个解
 * @param theta2_2  输出: θ2 的第二个解
 */
static void __robot_kinematics_calc_theta2(float theta3, float *theta2_1, float *theta2_2)
{
    float pz = g_robot_kinematics.T[2][3];

    float _pow_a2_2 = pow(a2, 2);
    float _pow_a3_2 = pow(a3, 2);
    float _pow_d4_2 = pow(d4, 2);
    float _2_a2_a3 = 2 * a2 * a3;
    float _2_a2_d4 = 2 * a2 * d4;

    float cos_theta3 = cos(theta3);
    float sin_theta3 = sin(theta3);

    float const_eq1 = _pow_a2_2 + _pow_a3_2 + _pow_d4_2;
    float eq1 = sqrt(const_eq1 + _2_a2_a3*cos_theta3 - _2_a2_d4*sin_theta3 - pow(pz, 2));
    float eq2 = a3*cos_theta3 - d4*sin_theta3;
    float eq3 = (d4*cos_theta3 - pz + a3*sin_theta3);

    float u_theta2_1 = -(a2 + eq1 + eq2) / eq3;
    float u_theta2_2 = -(a2 - eq1 + eq2) / eq3;

    *theta2_1 = atan(u_theta2_1) * 2;
    *theta2_2 = atan(u_theta2_2) * 2;
}

/**
 * @brief 为每组已有的 θ3 计算对应的两个 θ2
 *
 * 结果分布:
 *   result[0].θ2 = θ2 解0a, result[1].θ2 = θ2 解0b  (基于 θ3_1)
 *   result[2].θ2 = θ2 解1a, result[3].θ2 = θ2 解1b  (基于 θ3_1)
 */
static void robot_kinematics_calc_theta2(void)
{
    float theta3 = 0;
    float theta2_1 = 0;
    float theta2_2 = 0;

    theta3 = g_robot_kinematics.result[0][ROBOT_JOINT_3];
    __robot_kinematics_calc_theta2(theta3, &theta2_1, &theta2_2);
    g_robot_kinematics.result[0][ROBOT_JOINT_2] = theta2_1;
    g_robot_kinematics.result[1][ROBOT_JOINT_2] = theta2_2;

    theta3 = g_robot_kinematics.result[2][ROBOT_JOINT_3];
    __robot_kinematics_calc_theta2(theta3, &theta2_1, &theta2_2);
    g_robot_kinematics.result[2][ROBOT_JOINT_2] = theta2_1;
    g_robot_kinematics.result[3][ROBOT_JOINT_2] = theta2_2;
}

/* ============================================================
 *    第1步: 计算 θ1（基座旋转角度）
 *
 *    θ1 以 θ2、θ3 为输入。
 * ============================================================ */

/**
 * @brief 根据 θ2、θ3 计算 θ1（基座旋转角度）
 *
 * 公式:
 *   u = sqrt((-px + eq1) / (px + eq1))
 *   其中 eq1 = a2*cos(θ2) + a3*cos(θ2-θ3) + d4*sin(θ2-θ3)
 *
 *   符号由 py 方向确定。
 *
 * @return θ1（弧度）
 */
static float __robot_kinematics_calc_theta1(float theta2, float theta3)
{
    float px = g_robot_kinematics.T[0][3];
    float py = g_robot_kinematics.T[1][3];

    float diff_theta2_3 = theta2 - theta3;
    float cos_diff_theta2_3 = cos(diff_theta2_3);
    float sin_diff_theta2_3 = sin(diff_theta2_3);
    float cos_theta2 = cos(theta2);
    float sin_theta2 = sin(theta2);
    float cos_theta3 = cos(theta3);
    float sin_theta3 = sin(theta3);

    float eq1 = a2*cos_theta2 + a3*cos_diff_theta2_3 + d4*sin_diff_theta2_3;
    float u_theta1 = sqrt((-px + eq1)/(px + eq1));

    // 用 u_theta1 反算 py 验算，确认符号正确
    float eq2 = (2*u_theta1*(cos_theta2*(a2 + a3*cos_theta3 - d4*sin_theta3) + sin_theta2*(d4*cos_theta3 + a3*sin_theta3))) / (pow(u_theta1, 2) + 1);

    // 若反算的 py 与实际不符，说明符号取反了
    if (fabs(py - eq2) > ROBOT_ERROR_RANGE) {
        u_theta1 = -u_theta1;
    }

    float theta1 = atan(u_theta1) * 2;

    return theta1;
}

/** @brief 为每组解计算对应的 θ1 */
static void robot_kinematics_calc_theta1(void)
{
    for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
        float theta2 = g_robot_kinematics.result[i][ROBOT_JOINT_2];
        float theta3 = g_robot_kinematics.result[i][ROBOT_JOINT_3];
        g_robot_kinematics.result[i][ROBOT_JOINT_1] = __robot_kinematics_calc_theta1(theta2, theta3);
    }
}

/* ============================================================
 *    第5步: 计算腕部 θ4、θ5、θ6（手腕旋转角度）
 *
 *    位置逆解求出 θ1、θ2、θ3 后，R0_3 即唯一确定。
 *    由 R3_6 = R0_3^T · R0_6 一次性提取腕部三关节角度。
 *
 *    理论依据（已通过正反解闭环验证）:
 *      R3_6 = R34·R45·R56 =
 *        [ c4c5c6+s4s6,  -c4c5s6+s4c6,   c4s5 ]
 *        [ s5c6,         -s5s6,          -c5  ]
 *        [ -s4c5c6+c4s6,  s4c5s6+c4c6,  -s4s5 ]
 *    从而:
 *      θ5 = atan2(√(R[0][2]²+R[2][2]²), -R[1][2])
 *      θ4 = atan2(-R[2][2], R[0][2])
 *      θ6 = atan2(-R[1][1], R[1][0])
 * ============================================================ */

/**
 * @brief 根据 θ1、θ2、θ3 计算腕部三关节角度 θ4、θ5、θ6
 *
 * 奇异性处理: 当 θ5 ≈ 0 或 π 时出现万向节锁，θ4 与 θ6 耦合
 * （仅 θ4+θ6 唯一确定），此时取 θ4 = 0，θ6 由旋转矩阵直接确定。
 *
 * @param theta1,theta2,theta3  已解出的前三个关节角（弧度）
 * @param theta4,theta5,theta6  输出的腕部关节角（弧度）
 */
static void __robot_kinematics_calc_wrist(float theta1, float theta2, float theta3,
                                          float *theta4, float *theta5, float *theta6)
{
    /* R0_3 = 前3连杆正解 */
    float T0_3[4][4];
    float theta[6] = {theta1, theta2, theta3, 0.0f, 0.0f, 0.0f};
    robot_kinematics_fk_n(theta, 3, T0_3);

    /* R3_6 = R0_3^T · R0_6（R0_6 为目标位姿的旋转部分） */
    float R[3][3];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                sum += T0_3[k][i] * g_robot_kinematics.T[k][j];
            }
            R[i][j] = sum;
        }
    }

    float s5 = sqrt(R[0][2]*R[0][2] + R[2][2]*R[2][2]);   /* = |sin(theta5)| */
    *theta5 = atan2(s5, -R[1][2]);

    if (s5 < ROBOT_ERROR_RANGE)
    {
        /* 万向节锁: θ4 与 θ6 耦合，取 θ4=0，θ6 由 θ4+θ6 唯一确定 */
        *theta4 = 0.0f;
        *theta6 = atan2(R[2][0], R[0][0]);
    }
    else
    {
        *theta4 = atan2(-R[2][2], R[0][2]);
        *theta6 = atan2(-R[1][1], R[1][0]);
    }
}

/** @brief 为每组解计算腕部 θ4、θ5、θ6 */
static void robot_kinematics_calc_wrist(void)
{
    for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++)
    {
        float theta4 = 0.0f, theta5 = 0.0f, theta6 = 0.0f;
        __robot_kinematics_calc_wrist(g_robot_kinematics.result[i][ROBOT_JOINT_1],
                                      g_robot_kinematics.result[i][ROBOT_JOINT_2],
                                      g_robot_kinematics.result[i][ROBOT_JOINT_3],
                                      &theta4, &theta5, &theta6);
        g_robot_kinematics.result[i][ROBOT_JOINT_4] = theta4;
        g_robot_kinematics.result[i][ROBOT_JOINT_5] = theta5;
        g_robot_kinematics.result[i][ROBOT_JOINT_6] = theta6;
    }
}

/* ============================================================
 *    逆解主流程
 * ============================================================ */

/**
 * @brief 执行完整的运动学逆解
 *
 * 按解析依赖顺序依次计算：
 *   θ3 → θ2 → θ1 → θ5 → θ4 → θ6
 *
 * 计算完成后清除 result_invalid_mask（初始假设全部有效）。
 */
static void robot_kinematics_calc(void)
{
    robot_kinematics_calc_theta3();  /* 肘关节 */
    robot_kinematics_calc_theta2();  /* 肩关节 */
    robot_kinematics_calc_theta1();  /* 基座 */
    robot_kinematics_calc_wrist();   /* 腕部 θ4/θ5/θ6（通过 R3_6 = R0_3^T·R0_6 一次解出） */
    g_robot_kinematics.result_invalid_mask = 0; // 重置无效掩码
}

/* ============================================================
 *    调试输出
 * ============================================================ */

/** @brief 显示所有运动学逆解结果（通过LOG输出） */
void robot_kinematics_show_result(void)
{
    for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
        LOG("result[%d]: ", i);
        for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
            LOG("%.2f ", g_robot_kinematics.result[i][j]);
        }
        int valid = g_robot_kinematics.result_invalid_mask & (1 << i) ? 0 : 1;
        LOG(" valid:%d\n", valid);
    }
}

/* ============================================================
 *    单位转换与范围映射
 * ============================================================ */

/**
 * @brief 弧度转 0~360 度
 *
 * 将弧度值转为度，通过 fmod 和 ±360° 映射到 [0, 360)。
 * 特别注意 360.0° 映射为 0.0°。
 */
static float radians_to_degrees_0_360(float radians) {
    float degrees = radians * (180.0f / M_PI);
    degrees = fmod(degrees, 360.0f);    // 映射到 (-360, 360)
    if (degrees < 0) {
        degrees += 360.0f;              // 映射到 [0, 360)
    }

    // 处理浮点精度: 接近360时视为0
    if (fabs(degrees - 360.0f) < ROBOT_ERROR_RANGE) {
        degrees = 0.0f;
    }
    return degrees;
}

/**
 * @brief 将所有逆解结果从弧度制转换为 0~360 度制
 */
static void robot_kinematics_radians_to_degrees(void)
{
    for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
        for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
            g_robot_kinematics.result[i][j] = radians_to_degrees_0_360(g_robot_kinematics.result[i][j]);
        }
    }
}

/* ============================================================
 *    最优解选择
 * ============================================================ */

/**
 * @brief 从所有有效解中选择最优解
 *
 * 对每个有效解计算与当前关节角度的加权差值（权重数组 joint_weight），
 * 选择加权差值最小的解。
 *
 * 加权策略: 大关节（基座、肩、肘）权重更大，避免大幅旋转；
 *           末端小关节（手腕、末端夹持器）权重较小，可接受更多调整。
 *
 * @param result  输出的最优6关节角度（度）
 * @return 成功返回 0，无有效解返回 -1
 */
static int robot_kinematics_get_optimal_result(float *result)
{
    float diff = 0;
    float min_diff = 0xfffffff;
    int min_diff_result_index = -1;

    for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
        // 跳过无效解（被 mask 标记的）
        if (g_robot_kinematics.result_invalid_mask & (1 << i)) {
            continue;
        }

        diff = 0;
        for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
            diff += fabs(g_robot_kinematics.result[i][j] - g_current_joint_angle[j]) * joint_weight[j];
        }

        if (diff < min_diff) {
            min_diff = diff;
            min_diff_result_index = i;
        }
    }

    if (min_diff_result_index == -1) {
        return -1; // 没有有效解
    }

    for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
        result[j] = g_robot_kinematics.result[min_diff_result_index][j];
    }

    return 0;
}

/* ============================================================
 *    关节角度范围校验
 * ============================================================ */

/**
 * @brief 将逆解角度映射到各关节的有效范围，标记无效解
 *
 * 对每个解中的每个关节角度，尝试通过 ±360° 映射到关节允许范围。
 * 如果仍超出范围，则在 result_invalid_mask 中标记该解为无效。
 */
static void robot_kinematics_joint_angle_map(void)
{
    for (int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
        for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
            float angle = g_robot_kinematics.result[i][j];
            float min_angle = g_robot.joints[j].min_angle;
            float max_angle = g_robot.joints[j].max_angle;

            // 浮点误差处理：接近边界视为边界
            if (fabs(angle - min_angle) < ROBOT_ERROR_RANGE) {
                angle = min_angle;
            }

            if (fabs(angle - max_angle) < ROBOT_ERROR_RANGE) {
                angle = max_angle;
            }

            // 尝试 ±360° 映射到允许范围
            if (angle < min_angle) {
                angle += 360;
            } else if (angle > max_angle) {
                angle -= 360;
            }

            // 最终检查：仍不在范围内则标记此解无效
            if ((angle < min_angle) || (angle > max_angle)) {
                LOG("IK Out Of Range! Joint:%d, angle:%.2f, min:%.2f, max:%.2f\n", j, angle, min_angle, max_angle);
                g_robot_kinematics.result_invalid_mask |= (1 << i);
            }
            g_robot_kinematics.result[i][j] = angle;
        }
    }
}

/* ============================================================
 *    当前角度更新（供最优解选择使用）
 * ============================================================ */

/** @brief 批量更新当前关节角度（6个角度全部替换） */
void robot_kinematics_joint_angle_update(float *joint_angle)
{
    memcpy(g_current_joint_angle, joint_angle, sizeof(float)*ROBOT_MAX_JOINT_NUM);
}

/** @brief 按关节ID单独更新当前角度 */
void robot_kinematics_joint_angle_update_by_id(uint32_t joint_id, float angle)
{
    if (joint_id >= ROBOT_MAX_JOINT_NUM) {
        LOG("robot kinematics joint id is invalid\n");
        return;
    }
    g_current_joint_angle[joint_id] = angle;
}

/* ============================================================
 *    运动学逆解公开接口
 * ============================================================ */

/**
 * @brief 机械臂运动学逆解主函数（外部调用入口）
 *
 * 完整流程:
 *   1. 拷贝目标变换矩阵 T_target 到内部结构体
 *   2. 依次求解 θ3→θ2→θ1→θ5→θ4→θ6（弧度）
 *   3. 弧度 → 0~360 度
 *   4. 角度范围校验 + 标记无效解
 *   5. 选择与当前关节位置最接近的最优解
 *
 * @param T_target  输入的4x4目标变换矩阵（按行优先存储为16个float）
 * @param result    输出的最优关节角度（6个float，单位：度）
 * @param show      是否打印调试信息（0=不打印, 非0=打印）
 * @return 成功返回 0，无有效解返回 -1
 */
int robot_kinematics_inverse(float *T_target, float *result, int show)
{
    // 拷贝目标变换矩阵
    memcpy(g_robot_kinematics.T, T_target, sizeof(float)*16);

    // 执行逆解计算（弧度）
    robot_kinematics_calc();
    if (show) {
        LOG("target:[%f %f %f]\n", T_target[4 * 0 + 3], T_target[4 * 1 + 3], T_target[4 * 2 + 3]);
        LOG("robot kinematics result(rad):\n");
        robot_kinematics_show_result();
    }

    // 弧度 → 角度（0~360）
    robot_kinematics_radians_to_degrees();

    // 角度范围映射 + 无效解标记
    robot_kinematics_joint_angle_map();
    if (show) {
        LOG("robot kinematics result(deg):\n");
        robot_kinematics_show_result();
    }

    // 选择最优解
    int ret = robot_kinematics_get_optimal_result(result);
    if (show) {
        LOG("robot optimal result(rad):\n");
        for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
            LOG("%.2f ", result[i]);
        }
        LOG("\n");
    }
    return ret;
}

/* ============================================================
 *    正解（Forward Kinematics）与位姿矩阵组装
 * ============================================================ */

/**
 * @brief 单个连杆的 DH 齐次变换矩阵
 *
 * 采用与 MATLAB 推导一致的约定: R = Rx(alpha)·Rz(theta)，
 * 即先绕 z 轴转 theta，再绕新 x 轴转 alpha。
 *
 *    T = [ cos(theta),                 -sin(theta),                 0,        a            ]
 *        [ sin(theta)cos(alpha),        cos(theta)cos(alpha),      -sin(alpha), -d·sin(alpha) ]
 *        [ sin(theta)sin(alpha),        cos(theta)sin(alpha),       cos(alpha),  d·cos(alpha) ]
 *        [ 0,                           0,                          0,          1            ]
 *
 * @param a,alpha,d,theta  该连杆的 DH 参数（theta 单位：弧度）
 * @param T                输出的 4x4 齐次变换矩阵
 */
static void robot_kinematics_dh_transform(float a, float alpha, float d, float theta, float T[4][4])
{
    float ct = cos(theta);
    float st = sin(theta);
    float ca = cos(alpha);
    float sa = sin(alpha);

    T[0][0] = ct;
    T[0][1] = -st;
    T[0][2] = 0.0f;
    T[0][3] = a;

    T[1][0] = st * ca;
    T[1][1] = ct * ca;
    T[1][2] = -sa;
    T[1][3] = -d * sa;

    T[2][0] = st * sa;
    T[2][1] = ct * sa;
    T[2][2] = ca;
    T[2][3] = d * ca;

    T[3][0] = 0.0f;
    T[3][1] = 0.0f;
    T[3][2] = 0.0f;
    T[3][3] = 1.0f;
}

/**
 * @brief 4x4 矩阵乘法 C = A·B
 */
static void robot_kinematics_mat_mul(const float A[4][4], const float B[4][4], float C[4][4])
{
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 4; k++) {
                sum += A[i][k] * B[k][j];
            }
            C[i][j] = sum;
        }
    }
}

/**
 * @brief 前 n 个连杆的正解 T0_n = T0_1·T1_2·...·T(n-1)_n
 *
 * @param theta  6 个关节角（弧度）
 * @param n      参与正解的连杆数量（1~6）
 * @param T      输出的 4x4 齐次变换矩阵
 */
static void robot_kinematics_fk_n(const float theta[6], int n, float T[4][4])
{
    float Tn[4][4] = {
        {1.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f},
    };

    for (int i = 0; i < n; i++) {
        float Ti[4][4];
        float Tnext[4][4];
        robot_kinematics_dh_transform(D_H[i][0], D_H[i][1], D_H[i][2], theta[i], Ti);
        robot_kinematics_mat_mul(Tn, Ti, Tnext);
        memcpy(Tn, Tnext, sizeof(Tn));
    }

    memcpy(T, Tn, sizeof(Tn));
}

/**
 * @brief 运动学正解：6 个关节角 -> 末端 4x4 齐次变换矩阵
 *
 * @param theta  6 个关节角（弧度，即 DH 的 theta 值）
 * @param T      输出的末端位姿 4x4 齐次变换矩阵
 */
void robot_kinematics_fk(const float theta[6], float T[4][4])
{
    robot_kinematics_fk_n(theta, ROBOT_MAX_JOINT_NUM, T);
}

/**
 * @brief 根据绝对位姿（位置 + RPY固定角姿态）组装目标变换矩阵
 *
 * RPY 固定角（ZYX）: R = Rz(yaw)·Ry(pitch)·Rx(roll)，即先绕固定 Z 轴
 * 转 yaw，再绕固定 Y 轴转 pitch，最后绕固定 X 轴转 roll。
 *
 *    R = [ cβcγ,  sαsβcγ - cαsγ,  cαsβcγ + sαsγ ]
 *        [ cβsγ,  sαsβsγ + cαcγ,  cαsβsγ - sαcγ ]
 *        [ -sβ,   sαcβ,           cαcβ ]
 *
 * 其中 α=roll, β=pitch, γ=yaw（输入单位：度，内部转弧度）。
 *
 * @param x,y,z   末端绝对位置（mm）
 * @param roll    绕X轴旋转角（度）
 * @param pitch   绕Y轴旋转角（度）
 * @param yaw     绕Z轴旋转角（度）
 * @param T_out   输出的 4x4 齐次变换矩阵
 */
void robot_kinematics_cal_T_pose(float x, float y, float z,
                                  float roll, float pitch, float yaw,
                                  float T_out[4][4])
{
    float sa = sin(roll  * (M_PI / 180.0f));
    float ca = cos(roll  * (M_PI / 180.0f));
    float sb = sin(pitch * (M_PI / 180.0f));
    float cb = cos(pitch * (M_PI / 180.0f));
    float sg = sin(yaw   * (M_PI / 180.0f));
    float cg = cos(yaw   * (M_PI / 180.0f));

    T_out[0][0] = cb * cg;
    T_out[0][1] = sa * sb * cg - ca * sg;
    T_out[0][2] = ca * sb * cg + sa * sg;
    T_out[0][3] = x;

    T_out[1][0] = cb * sg;
    T_out[1][1] = sa * sb * sg + ca * cg;
    T_out[1][2] = ca * sb * sg - sa * cg;
    T_out[1][3] = y;

    T_out[2][0] = -sb;
    T_out[2][1] = sa * cb;
    T_out[2][2] = ca * cb;
    T_out[2][3] = z;

    T_out[3][0] = 0.0f;
    T_out[3][1] = 0.0f;
    T_out[3][2] = 0.0f;
    T_out[3][3] = 1.0f;
}

/* ============================================================
 *    调试工具
 * ============================================================ */

/** @brief 打印 4x4 变换矩阵（调试用） */
void robot_kinematics_show_T(float T[4][4])
{
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            LOG("%.2f ", T[i][j]);
        }
        LOG("\n");
    }
}
