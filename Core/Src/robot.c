/**
  ******************************************************************************
  * @file    robot.c
  * @brief   This file provides code for the robot control frame.
  ******************************************************************************
  */

#include "robot.h"
#include "task.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include "Emm_V5.h"
#include "usart.h"
#include "main.h"
#include <stdlib.h>
#include "robot_kinematics.h"


struct robot g_robot;       /* 机械臂全局控制实例（唯一） */

/* ============================================================
 *    机械臂物理参数（需根据实际结构手动标定）
 * ============================================================ */

/**
 * @brief DH参数矩阵 D_H[6][4]
 *
 * D_H[i] = {a, alpha, d, theta_home}
 *   a:          连杆长度（mm）
 *   alpha:      连杆扭转角（rad）
 *   d:          连杆偏距（mm）
 *   theta_home: 复位(home)时的关节角（rad）
 *
 * 正解/逆解中，关节角即 DH 的 theta 值本身（无需再加偏移），
 * 第4列仅为记录 home 姿态的关节角，供参考。
 */
const float D_H[6][4] = {{0,        0,          0,          M_PI/2},
                         {0,        M_PI/2,      0,         M_PI/2},
                         {200,      M_PI,        0,         -M_PI/2},
                         {47.63,    -M_PI/2,     -184.5,    0},
                         {0,        M_PI/2,      0,         M_PI/2},
                         {0,        M_PI/2,        0,         0}};

/**
 * @brief 机械臂复位状态下的末端变换矩阵 T0_6
 *
 * 即所有关节处于初始角度 {90, 90, -90, 0, 90, 0}° 时，末端相对基座标系的
 * 4x4 齐次变换矩阵（等价于 fk([90, 90, -90, 0, 90, 0]°)）。
 */
const float T_0_6_reset[4][4] = {
  {0, -1, 0, 0},
  {0, 0, -1, -47.63},
  {1, 0, 0, 15.5},
  {0, 0, 0, 1},
};

/**
 * @brief 各关节旋转权重
 *
 * 在选择逆解最优解时，用于计算当前角度与候选解之间的加权差值。
 * 权重越大，该关节在当前最优解中的"话语权"越大。
 * 通常前几个大关节（基座、肩、肘）权重更大。
 */
const float joint_weight[ROBOT_MAX_JOINT_NUM] = {5, 3, 3, 1, 1, 1};

/**
 * @brief 各关节初始状态（复位后的目标位置）
 *
 * 索引 0~5 对应关节1~6。
 * 包含: 初始角度、正方向对应的电机方向、减速比、限位开关引脚、角度范围、复位方向。
 */
static struct joint g_joint_init[ROBOT_MAX_JOINT_NUM] = 
{
  {90,    MOTOR_DIR_CW,    50,     JOINT_LIMIT_1_GPIO_Port,    JOINT_LIMIT_1_Pin,      0,      360,    DIR_NEGATIVE},  /* 关节1: 基座旋转 */
  {90,    MOTOR_DIR_CW,  50.89,  JOINT_LIMIT_2_GPIO_Port,    JOINT_LIMIT_2_Pin,       90,     180,    DIR_NEGATIVE},  /* 关节2: 大臂俯仰 */
  {-90,   MOTOR_DIR_CW,   50.89,  JOINT_LIMIT_3_GPIO_Port,    JOINT_LIMIT_3_Pin,      -90,    90 ,    DIR_NEGATIVE},  /* 关节3: 小臂俯仰 */
  {0,     MOTOR_DIR_CCW, 51,     JOINT_LIMIT_4_GPIO_Port,    JOINT_LIMIT_4_Pin,       -180,  180,    DIR_NEGATIVE},  /* 关节4: 手腕旋转 */
  {90,    MOTOR_DIR_CCW,   26.85,  JOINT_LIMIT_5_GPIO_Port,    JOINT_LIMIT_5_Pin,     0,      90,     DIR_POSITIVE}, /* 关节5: 手腕俯仰 */
  {0,     MOTOR_DIR_CCW,  51,     JOINT_LIMIT_6_GPIO_Port,    JOINT_LIMIT_6_Pin,      0,    360,    DIR_NEGATIVE},  /* 关节6: 末端旋转 */
};

/* ============================================================
 *    内部函数声明（static，仅本文件可见）
 * ============================================================ */

static void robot_joint_sync_to(float *target_angle);
static float robot_angle_normalize(float angle);
static float robot_angle_diff(float cur_angle, float target_angle);
static int robot_update_current_angle(uint8_t joint_id);
static void robot_pid_one_period(float *target_angle, float *intG_error, float *pre_error, float *total_error, int joint_num);
static void robot_joint_stop_from_isr(uint8_t joint_id)
{
  Emm_V5_Stop_Now(joint_id + 1, false);
  g_robot.joints[joint_id].velocity = 0;
}

/* ============================================================
 *    限位开关处理
 * ============================================================ */

/**
 * @brief 限位开关触发后的处理函数
 *
 * 延时200ms防抖 → 清除限位状态位，使能下一次限位检测。
 * 注意：实际的停止电机和投递事件已在 robot_joint_limit_happend() 中完成。
 */
static void robot_joint_limit_post_handle(uint8_t joint_id)
{
  vTaskDelay(200);
  taskENTER_CRITICAL();
  ROBOT_STATUS_CLEAR(g_robot.joints[joint_id].status, ROBOT_STATUS_LIMIT_HAPPENED);
  taskEXIT_CRITICAL();
}

/* ============================================================
 *    关节速度与位置控制
 * ============================================================ */

/**
 * @brief 控制关节以指定速度旋转（速度模式）
 *
 * 向电机驱动器发送速度模式CAN指令。
 * 发送后会阻塞等待CAN应答帧（最多 ROBOT_CAN_TIMEOUT ms）。
 *
 * @param joint_id      关节ID（0~5）
 * @param velocity      目标速度（rpm），正=正方向，负=反方向
 * @param acceleration  加速度
 * @return 成功返回 0，超时或ID非法返回 1
 */
uint32_t robot_joint_veloccity_to(uint32_t joint_id, float velocity, uint8_t acceleration)
{
  if (joint_id >= ROBOT_MAX_JOINT_NUM){return 1;}
  
  int start_tick = HAL_GetTick();
  struct joint *joint = &g_robot.joints[joint_id];

  // 根据速度符号和关节正方向决定电机实际方向
  uint8_t dir = (velocity > 0) ? joint->postive_direction : !(joint->postive_direction);
  ROBOT_STATUS_CLEAR(joint->status, ROBOT_STATUS_LIMIT_ENABLE);
  uint32_t addr = joint_id + 1;//can地址 = 关节ID + 1（电机ID从0开始）

  // 速度换算: rpm = velocity(度/s) * 60 * 减速比 / 360
  // 驱动器将速度值/10作为真实速度，实现0.1rpm精度，所以需 ×10
  joint->velocity = velocity;
  uint16_t _velocity = (uint16_t)(velocity * 600 *joint->reduction_ratio / 360);
  
  //挂起任务调度（can收发不能被打断）
  vTaskSuspendAll();
  can.rxFrameFlag = false;
  while (can.rxFrameFlag == false)
  {
    if (HAL_GetTick() - start_tick > ROBOT_CAN_TIMEOUT)
    {
      LOG("Error: CAN timeout;%d\n", joint_id);
      xTaskResumeAll();
      return 1;
    }
    Emm_V5_Vel_Control(addr, dir, _velocity, acceleration, false);
    HAL_Delay(1);
  }
  xTaskResumeAll();
  return 0;
}

/**
 * @brief 控制关节旋转到指定角度（位置模式）
 *
 * 将目标角度换算为步进电机脉冲数，通过CAN发送位置模式指令。
 * 支持相对运动和绝对运动两种模式。
 *
 * @param joint_id      关节ID（0~5）
 * @param dir           运动方向
 * @param angle         目标角度（度），绝对模式下为目标绝对角度，相对模式下为相对增量
 * @param velocity      运动速度（rpm）
 * @param acceleration  加速度
 * @param absolute      true=绝对运动, false=相对运动
 * @return 成功返回 0，失败返回 1
 */
static uint32_t robot_joint_rotate_to(uint32_t joint_id, enum dir dir, float angle, float velocity,
 uint32_t acceleration, bool absolute)
{
  if (joint_id >= ROBOT_MAX_JOINT_NUM)
  {
    LOG("ERROR: joint_id is out of range");
    return 1;
  }
  if (velocity < 0)
  {
    LOG("ERROR: velocity is negative");
    return 0;
  }
  
  float rel_angle = 0;
  uint8_t _dir;
  struct joint *joint = &g_robot.joints[joint_id];

  if (absolute)
  {
    /* 绝对运动: 计算从当前角度到目标角度的相对量 */
    rel_angle = angle - joint->current_angle;

    if (fabs(rel_angle) < ROBOT_JOINT_ANGLE_ERROR_RANGE)
    {
      return 0;// 已经到达目标角度，无需运动
    }
    
    /* 全范围关节(0~360)处理角度环绕，自动选择最短路径 */
    if (fabs(joint->max_angle - joint->min_angle) >= 360.0f - ROBOT_JOINT_ANGLE_ERROR_RANGE)
    {
      if (rel_angle > 180.0f)
      {
        rel_angle -= 360.0f;
      }
      else if (rel_angle < -180.0f)
      {
        rel_angle += 360.0f;
      }
    }

    /* 根据最短路径方向决定电机实际转向 */
    _dir = (rel_angle >= 0) ? joint->postive_direction : !(joint->postive_direction);

    LOG("id:%d current:%f, target:%f, rel:%f\n", joint_id, joint->current_angle, angle, rel_angle);
    joint->current_angle = angle;
  }
  else
  {
    /* 相对运动: 按指定方向旋转指定角度 */
    rel_angle = angle;
    joint->current_angle += (dir == DIR_POSITIVE) ? angle : -angle;
    _dir = (dir == DIR_POSITIVE) ? joint->postive_direction : !(joint->postive_direction);
    if (rel_angle < 0)
    {
      _dir = !_dir;
    }  
  }

  /* 将角度转换为步进脉冲数 */
  // 16细分下，3200个脉冲 = 电机转1圈
  uint32_t addr = joint_id + 1;
  uint32_t step = (uint32_t)fabs(round(rel_angle * joint->reduction_ratio * 3200 / 360));
  uint32_t _velocity =(uint16_t)fabs(velocity * 600 * joint->reduction_ratio / 360.0);
  ROBOT_STATUS_CLEAR(joint->status, ROBOT_STATUS_READY);
  Emm_V5_Pos_Control(addr, _dir, _velocity, (uint8_t)acceleration, step, false, false);
  
  return 0; 
}

static void robot_joint_set_zero(uint8_t joint_id)
{
  Emm_V5_Reset_CurPos_To_Zero(joint_id + 1);        //设置当前位置为零点
}

/* ============================================================
 *    限位开关 ISR 处理
 * ============================================================ */

/**
 * @brief 限位开关触发回调（在中断上下文中调用）
 *
 * 防抖检查后，立即停止电机，设置状态位，并投递限位事件到事件队列。
 * 注意：此函数在GPIO EXTI中断回调中调用，必须使用FromISR版本的API。
 *
 * @param joint_id  触发限位的关节ID
 */
static void robot_joint_limit_happend(uint8_t joint_id)
{
  //检查事件队列
  if(g_robot.event_queue == NULL){return;}

  //检查限位开关是否使能
  if(!ROBOT_STATUS_IS(g_robot.joints[joint_id].status, ROBOT_STATUS_LIMIT_ENABLE)){return;}

  //防止按键抖动导致重复触发事件
  if(ROBOT_STATUS_IS(g_robot.joints[joint_id].status, ROBOT_STATUS_LIMIT_HAPPENED)){return;}
  
  //ISR中直接停止电机（不能阻塞）
  robot_joint_stop_from_isr(joint_id);
  ROBOT_STATUS_SET(g_robot.joints[joint_id].status, ROBOT_STATUS_LIMIT_HAPPENED);

  //投递限位事件到事件队列（FromISR版本）
  struct robot_event event = {0};
  event.type = ROBOT_LIMIT_SWITCH_EVENT;
  event.joint_id = joint_id;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendToBackFromISR(g_robot.event_queue, &event, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ============================================================
 *    限位开关 GPIO 模式切换
 * ============================================================ */

/** @brief 将限位开关引脚设为普通输入模式（用于读取当前电平） */
static void robot_joint_limit_set_input(uint8_t joint_id)
{
  HAL_GPIO_DeInit(g_robot.joints[joint_id].limit_gpio_port, g_robot.joints[joint_id].limit_gpio_pin);
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = g_robot.joints[joint_id].limit_gpio_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(g_robot.joints[joint_id].limit_gpio_port, &GPIO_InitStruct);
}

/** @brief 将限位开关引脚设为双边沿中断模式（用于检测限位触发） */
static void robot_joint_limit_set_irq(uint8_t joint_id)
{
  HAL_GPIO_DeInit(g_robot.joints[joint_id].limit_gpio_port, g_robot.joints[joint_id].limit_gpio_pin);
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = g_robot.joints[joint_id].limit_gpio_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;  // 上升沿和下降沿都触发中断
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(g_robot.joints[joint_id].limit_gpio_port, &GPIO_InitStruct);
}

/** @brief 读取限位开关当前电平 */
static GPIO_PinState robot_get_limit_status(uint8_t joint_id)
{
  GPIO_PinState state = HAL_GPIO_ReadPin(g_robot.joints[joint_id].limit_gpio_port, g_robot.joints[joint_id].limit_gpio_pin);
  return state;
}

/* ============================================================
 *    关节复位
 * ============================================================ */

/**
 * @brief 单个关节硬件复位
 *
 * 流程: 先读取限位开关状态 → 如果已触发则直接清零位置 →
 *       否则低速向复位方向旋转直到限位开关触发 → 清零当前位置。
 *
 * @param joint_id  关节ID
 */
static void robot_joint_reset(uint8_t joint_id)
{
  GPIO_PinState state;
  int reset_dir = g_robot.joints[joint_id].reset_dir;

  robot_joint_limit_set_input(joint_id);            //设置为输入模式读取当前状态
  state = robot_get_limit_status(joint_id);         //储存当前状态
  robot_joint_limit_set_irq(joint_id);              //恢复为中断模式

  if (state == GPIO_PIN_SET)
  {
    LOG("joint %d limit switch already happend\n", joint_id);
    robot_joint_set_zero(joint_id); // 清零当前位置
    return;
  }
  
  robot_joint_rotate_to(joint_id, reset_dir, ROBOT_RESET_DEFAULT_ANGLE, ROBOT_RESET_DEFAULT_VELOCITY,
                        ROBOT_RESET_DEFAULT_ACCELERATION, false);
  while (!ROBOT_STATUS_IS(g_robot.joints[joint_id].status, ROBOT_STATUS_LIMIT_HAPPENED))
  {
    vTaskDelay(200);    //等待关节转动到限位
  }
  vTaskDelay(ROBOT_CAN_DELAY);
  robot_joint_set_zero(joint_id);        //设置当前位置为零点
}
/**
 * @brief 所有关节硬件复位
 *
 * 从末端关节开始向基座逐次复位（防止碰撞），复位完成后重置所有角度为初始值。
 */
static void robot_joint_hard_reset(void)
{
  // 从关节5到关节0依次复位（末端优先，防止碰撞）
  for (int i = ROBOT_MAX_JOINT_NUM - 1; i >= 0; i--)
  {
    ROBOT_STATUS_SET(g_robot.joints[i].status, ROBOT_STATUS_LIMIT_ENABLE);
    robot_joint_reset(i);
    vTaskDelay(100);
  }
  
  //复为完成恢复初始角度
  for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++)
  {
    g_robot.joints[i].current_angle = g_joint_init[i].current_angle;
  }
  g_robot.cur_pos.x = 0;
  g_robot.cur_pos.y = 0;
  g_robot.cur_pos.z = 0;
}
/**
 * @brief 设置零点 上电后位置当成零点 不借助限位开关
 * 
 */
static void robot_joint_set_home(void)
{
  LOG("setting home position ... \n");

  for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++)
  {
    robot_joint_set_zero(i);
    g_robot.joints[i].current_angle = g_joint_init[i].current_angle;
    osDelay(5);
  }
  
  g_robot.cur_pos.x = 0;
  g_robot.cur_pos.y = 0;
  g_robot.cur_pos.z = 0;

  LOG("home position set\n");
}

/**
 * @brief 软复位：不借助限位开关，直接运动到初始角度
 *
 * 从末端开始逐关节复位，选择最短路径运动到目标角度。
 * 运动策略：选择角度变化最小的方向（考虑360°周期性）。
 */
static void robot_joint_soft_reset(void)
{
  LOG("robot joint soft reset start\n");

  uint32_t start_tick = 0;

  /*从末端到基座，逐关节回到最开始的角度*/
  for (int i = ROBOT_MAX_JOINT_NUM - 1; i >= 0; i--)
  {
    start_tick = HAL_GetTick();

    float target = g_joint_init[i].current_angle;
    float diff = target - g_robot.joints[i].current_angle;
    enum dir go_dir = (diff >= 0) ? DIR_POSITIVE : DIR_NEGATIVE;

    //如果已经在目标位置附近 跳过
    if (fabs(g_robot.joints[i].current_angle - target) < ROBOT_JOINT_ANGLE_ERROR_RANGE)
    {
      LOG("joint %d already at target\n", i);
      continue;
    }

    //关节运动
    robot_joint_rotate_to(i, go_dir, target, ROBOT_JOINT_DEFAULT_VELOCITY, ROBOT_JOINT_DEFAULT_ACCELERATION, true);

    //等待到位
    while (!ROBOT_STATUS_IS(g_robot.joints[i].status, ROBOT_STATUS_READY))
    {
      osDelay(50);
      if ((HAL_GetTick() - start_tick) > 10000)
      {
        LOG("joint %d soft failed\n", i);
        break;
      }
    }
    osDelay(50);
  }
  LOG("robot joint soft reset done\n");
}

/* static void robot_joint_soft_reset2(void)
{
  float angle = 0;
  int ret = 0;
    for (int i = ROBOT_MAX_JOINT_NUM - 1; i >= 0; i--) {
      ROBOT_STATUS_SET(g_robot.joints[i].status, ROBOT_STATUS_LIMIT_ENABLE);
      int dir = DIR_POSITIVE;
      robot_update_current_angle(i); // 读取当前角度

      // 将当前角度映射到关节允许范围内
      ret = robot_angle_map(g_robot.joints[i].current_angle, g_joint_init[i].min_angle, g_joint_init[i].max_angle, &angle);
      if (ret != 0) {
          LOG("robot angle map failed, joint_id:%d current_angle:%.2f\n", i, g_robot.joints[i].current_angle);
          return;
      }

      // 决定旋转方向（使运动角度最小）
      if (angle > g_joint_init[i].current_angle) {
          dir = DIR_NEGATIVE;
      }

      // 对于全范围关节（0~360），选择最短路径
      if (g_joint_init[i].min_angle == 0 && g_joint_init[i].max_angle == 360) {
          if (fabs(angle - g_joint_init[i].current_angle) > 180) {
              dir = -dir;  // 绕反方向走更短
          }
      }

      LOG_FROM_ISR("[%d] current:%.2f, target:%.2f, dir:%d\n\n", i, angle, g_joint_init[i].current_angle, dir);
      g_robot.joints[i].current_angle = angle;

      robot_joint_rotate_to(i, dir, g_joint_init[i].current_angle, ROBOT_RESET_DEFAULT_VELOCITY, ROBOT_RESET_DEFAULT_ACCELERATION, true);
      vTaskDelay(100);
      g_robot.joints[i].current_angle = g_joint_init[i].current_angle;
  }

  g_robot.cur_pos.x = 0;
  g_robot.cur_pos.y = 0;
  g_robot.cur_pos.z = 0;
  
} */

/**
 * @brief 所有关节同步运动到各自的目标角度
 *
 * 先发送全部6个关节的位置指令（不等候），再统一等待到位。
 * 大步长自动分步插值（每步 ≤ 10°），超时则停止所有电机。
 */
static void robot_joint_sync_to(float *target_angle)
{
  uint32_t t0 = HAL_GetTick();

  /* 发送所有关节指令 */
  for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++)
  {
    if (fabs(g_robot.joints[i].current_angle - target_angle[i]) < ROBOT_JOINT_ANGLE_ERROR_RANGE)
    {
      ROBOT_STATUS_SET(g_robot.joints[i].status, ROBOT_STATUS_READY);
      continue;
    }
    
    float diff = target_angle[i] - g_robot.joints[i].current_angle;
    enum dir go_dir = (diff >= 0)? DIR_POSITIVE : DIR_NEGATIVE;

    robot_joint_rotate_to(i, go_dir, target_angle[i], ROBOT_JOINT_DEFAULT_VELOCITY, ROBOT_JOINT_DEFAULT_ACCELERATION, true);
    
    osDelay(2);
  }
  
  /* 等待全部电机到位 */
  for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++)
  {
    while (!ROBOT_STATUS_IS(g_robot.joints[i].status, ROBOT_STATUS_READY))
    {
      osDelay(20);
      if (HAL_GetTick() - t0 > 15000)
      {
        LOG("joints sync timeout stopping all\n");
        for (int j = 0; j < ROBOT_MAX_JOINT_NUM; j++)
        {
          if (!ROBOT_STATUS_IS(g_robot.joints[j].status, ROBOT_STATUS_READY))
          {
            Emm_V5_Stop_Now(j + 1, false);
          }
        }
        return;
      }
    }
  }
  LOG("joints sync done\n");
}

/* static void robot_joints_sync_to(struct robot_event *event)
{
    for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
        robot_joint_rotate_to(i, DIR_POSITIVE, event->param[i],
                    ROBOT_JOINT_DEFAULT_VELOCITY, ROBOT_JOINT_DEFAULT_ACCELERATION, true);
        g_robot.joints[i].current_angle = event->param[i]; // 更新当前角度
    }
} */

/* ============================================================
 *    角度映射工具函数
 * ============================================================ */

/**
 * @brief 将角度映射到指定的 [min, max] 范围内
 *
 * 利用角度的 360° 周期性，将超出范围的角度通过 ±360° 映射回来。
 * 处理浮点抖动误差（接近 min/max 时视为边界值）。
 *
 * @param angle      输入角度（度）
 * @param min_angle  最小允许角度
 * @param max_angle  最大允许角度
 * @param result     输出映射后的角度
 * @return 成功返回 0，无法映射到范围内返回 1
 */
/* static int robot_angle_map(float angle, float min_angle, float max_angle, float *result)
{
    if (result == NULL) {
        return 1;
    }

    float tmp_angle = angle;

    // 防抖动：接近边界时视为边界值
    if (fabs(angle - min_angle) < ROBOT_JOINT_ANGLE_ERROR_RANGE) {
        tmp_angle = min_angle;
    } else if (fabs(angle - max_angle) < ROBOT_JOINT_ANGLE_ERROR_RANGE) {
        tmp_angle = max_angle;
    }

    // 超出范围则通过 ±360° 映射
    if (angle < min_angle) {
        tmp_angle += 360;
    } else if (angle > max_angle) {
        tmp_angle -= 360;
    }

    // 二次防抖
    if (fabs(tmp_angle - min_angle) < ROBOT_JOINT_ANGLE_ERROR_RANGE) {
        tmp_angle = min_angle;
    } else if (fabs(tmp_angle - max_angle) < ROBOT_JOINT_ANGLE_ERROR_RANGE) {
        tmp_angle = max_angle;
    }

    // 检查最终结果是否在范围内
    if ((tmp_angle < min_angle) || (tmp_angle > max_angle)) {
        return 1;
    }

    *result = tmp_angle;
    return 0;
} */

/* ============================================================
 *    主控制任务
 * ============================================================ */

/**
 * @brief 机械臂主控制任务
 *
 * 从事件队列中阻塞取出事件，根据事件类型分发到不同的处理函数。
 * 这是整个机械臂控制系统的核心调度循环。
 *
 * 事件类型与处理：
 *   - ROBOT_JOINT_REL_ROTATE   → robot_joint_rotate_to (相对模式)
 *   - ROBOT_JOINT_ABS_ROTATE   → robot_joint_rotate_to (绝对模式)
 *   - ROBOT_LIMIT_SWITCH_EVENT → robot_joint_limit_post_handle (限位后处理)
 *   - ROBOT_AUTO_EVENT         → robot_auto_move_interpolation (自动运动+插值)
 *   - ROBOT_TIMIE_FUNC_EVENT   → robot_time_func_move (时间函数轨迹)
 *   - ROBOT_HARD_RESET_EVENT   → robot_joint_hard_reset (硬复位)
 *   - ROBOT_SOFT_RESET_EVENT   → robot_joint_soft_reset (软复位)
 *   - ROBOT_REMOTE_CONTROL_EVENT → robot_pid_remote (远程遥控)
 *   - ROBOT_JOINTS_SYNC_EVENT  → robot_joints_sync_to (关节同步)
 */

void robot_control_task(void *arg)
{
  (void)arg;
  LOG("robot control task runing!!!\n");

  struct robot_event event = {0};
  
  // 阻塞等待事件，取出后分发处理
  while (xQueueReceive(g_robot.event_queue, &event, portMAX_DELAY) == pdPASS)
  {
    switch (event.type)
    {
    case ROBOT_JOINT_REL_ROTATE :
      LOG("[jonit_id : %d] ROBOT_JOINT_REL_RTATE %f\n", event.joint_id, event.param[0]);
      robot_joint_rotate_to(event.joint_id, DIR_POSITIVE, event.param[0],
                            ROBOT_JOINT_DEFAULT_VELOCITY, ROBOT_JOINT_DEFAULT_ACCELERATION, false);
      break;
    case ROBOT_JOINT_ABS_ROTATE :
      LOG("[joint_id: %d] ROBOT_JOINT_ABS_ROTATE %f\n", event.joint_id, event.param[0]);
      robot_joint_rotate_to(event.joint_id, DIR_POSITIVE, event.param[0],
                            ROBOT_JOINT_DEFAULT_VELOCITY, ROBOT_JOINT_DEFAULT_ACCELERATION, true);
      break;
    case ROBOT_LIMIT_SWITCH_EVENT :
      LOG("[joint_id: %d] ROBOT_LIMIT_SWITCH_EVENT\n", event.joint_id);
      robot_joint_limit_post_handle(event.joint_id);
      break;
    case ROBOT_AUTO_EVENT :
      
      break;
    case ROBOT_TIMIE_FUNC_EVENT :
      
      break;
    case ROBOT_HARD_RESET_EVENT :
      LOG("ROBOT_HARD_RESET_EVENT\n");
      robot_joint_hard_reset();
      break;
    case ROBOT_SOFT_RESET_EVENT :
      LOG("ROBOT_SOFT_RESET_EVENT\n");
      robot_joint_soft_reset();
      break;
    case ROBOT_REMOTE_CONTROL_EVENT :

      break;
    case ROBOT_JOINTS_SYNC_EVENT :
      LOG("JOINTS_SYNC: %.1f %.1f %.1f %.1f %.1f %.1f\n",
          event.param[0], event.param[1], event.param[2],
          event.param[3], event.param[4], event.param[5]);
          robot_joint_sync_to(event.param);
      break;
    case ROBOT_SET_HOME_EVENT :
      LOG("ROBOT_SET_HOME_EVENT\n");  
      robot_joint_set_home();
      break;
    default:
      LOG("robot event type error\n");
    }
  }
}

/* ============================================================
 *    命令服务任务
 * ============================================================ */

/**
 * @brief 命令服务任务
 *
 * 从 cmd_queue 取出命令，根据命令来源（UART1 / MQTT）分发处理。
 * 如果 MQTT 使能，则在任务启动时初始化 ESP8266 MQTT 连接。
 */
void robot_cmd_service(void *arg)
{
  (void)arg;

  LOG("robot cmd service running !!!\n");

  struct robot_cmd cmd = {0};
  struct robot_event event = {0};

  for ( ; ; )
  {
    /*阻塞循环等待命令*/
    if (xQueueReceive(g_robot.cmd_queue, &cmd, portMAX_DELAY) !=pdPASS)
    {
      continue;
    }
    
    LOG("cmd : %s\n", cmd.cmd);

    /*-----命令解析（调试使用）-----*/
    /* j <id> <angle>          → 关节相对旋转 */
    /* j <id> a <angle>        → 关节绝对定位 */

    if (cmd.cmd[0] == 'J')
    {
      int id = 0;
      float angle = 0;
      char abs_flag = 0;

      if (sscanf(cmd.cmd, "J %d a %f", &id, &angle) == 2)
      {
        abs_flag = 1;
      }
      else if (sscanf(cmd.cmd, "J %d %f", &id, &angle) == 2)
      {
        abs_flag = 0;
      }
      else
      {
        LOG("bad format, use: j <id> <angle>  or  j <id> a <angle>\n");
        continue;
      }
      if (id < 0 || id >= ROBOT_MAX_JOINT_NUM)
      {
        LOG("joint id out");
        continue;
      }   

      event.joint_id = (uint8_t)id;
      event.param[0] = angle;
      event.type = abs_flag ? ROBOT_JOINT_ABS_ROTATE : ROBOT_JOINT_REL_ROTATE;

      xQueueSend(g_robot.event_queue, &event, 0);
      continue;
    }

    /* home -> 设置零点 */
    if (strncmp(cmd.cmd, "home", 4) == 0)
    {
      event.type = ROBOT_SET_HOME_EVENT;
      xQueueSend(g_robot.event_queue, &event, 0);
      continue;
    }
    
    /* reset 软复位（回到设置的零点） */
    if (strncmp(cmd.cmd, "reset", 5) == 0)
    {
      event.type = ROBOT_SOFT_RESET_EVENT;
      xQueueSend(g_robot.event_queue, &event, 0);
      continue;
    }

    /* G x y z [roll pitch yaw] → 笛卡尔空间移动（绝对位姿，单位 mm / 度） */
    if (cmd.cmd[0] == 'G')
    {
      float x = 0;
      float y = 0;
      float z = 0;
      float roll = 90.0f;    /* 未指定姿态时，默认保持复位/home 姿态 */
      float pitch = -90.0f;
      float yaw = 0.0f;

      int n = sscanf(cmd.cmd, "G %f %f %f %f %f %f", &x, &y, &z, &roll, &pitch, &yaw);
      if (n < 3)
      {
        LOG("bad format, use: G <X> <Y> <Z> [roll pitch yaw]\n");
        continue;
      }

      LOG("G target: X=%.1f Y=%.1f Z=%.1f R=%.1f P=%.1f Y=%.1f\n", x, y, z, roll, pitch, yaw);

      /* Step 1: 绝对位姿(xyz + RPY) → 目标变换矩阵 */
      float T_target[4][4];
      robot_kinematics_cal_T_pose(x, y, z, roll, pitch, yaw, T_target);

      /* Step 2: 更新当前角度给逆解器（选最优解用）*/
      float cur[ROBOT_MAX_JOINT_NUM];
      for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++)
      {
         cur[i] = g_robot.joints[i].current_angle;
      }
      robot_kinematics_joint_angle_update(cur);

      /* Step 3: 逆解 → 6 个关节角度 */
      float joint_angles[ROBOT_MAX_JOINT_NUM];
      int ret = robot_kinematics_inverse((float *)T_target, joint_angles, 1);
      if (ret != 0)
      {
        LOG("G: no valid ik solution, target unreachable!\n");
        continue;
      }

      /* Step 4: 投递 JOINTS_SYNC 事件 */
      event.type = ROBOT_JOINTS_SYNC_EVENT;
      for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++)
        event.param[i] = joint_angles[i];
      xQueueSend(g_robot.event_queue, &event, 0);
      continue;
    }
    osDelay(1);
  }
}

// /* NRF24L01 接收任务 */
// void nrf_receive_task(void *arg)
// {
//   (void)arg;
//   uint8_t rx_buf[32];
//   struct robot_cmd cmd = {.type = CMD_TYPE_UART3};

//   uint8_t en_aa = NRF24L01_ReadReg(NRF_EN_AA);
//   uint8_t en_rxaddr = NRF24L01_ReadReg(NRF_EN_RXADDR);
//   LOG("EN_AA: 0x%02X, EN_RXADDR: 0x%02X\n", en_aa, en_rxaddr);

//   // ===== 读取 RX_ADDR_P0 确认地址 =====
//   uint8_t rx_addr[5];
//   NRF24L01_ReadAddr(rx_addr);
//   LOG("F407 RX_ADDR_P0: %02X %02X %02X %02X %02X\n", 
//      rx_addr[0], rx_addr[1], rx_addr[2], rx_addr[3], rx_addr[4]);
//   LOG("NRF24L01 receive task started\n");

//   for (;;) {

//     if (NRF24L01_DataReceived()) 
//     {
//       NRF24L01_ReceiveData(rx_buf, 32);
//       rx_buf[31] = '\0';

//       if (rx_buf[0] == 'G' || rx_buf[0] == 'J' || rx_buf[0] == 'r' || rx_buf[0] == 'h') 
//       {

//         strncpy(cmd.cmd, (char*)rx_buf, ROBOT_CMD_LENGTH - 1);
//         cmd.cmd[ROBOT_CMD_LENGTH - 1] = '\0';

//         if (g_robot.cmd_queue != NULL) 
//         {
//           xQueueSend(g_robot.cmd_queue, &cmd, 0);
//         }

//         LOG("NRF received: %s\n", cmd.cmd);
//       }
//     }
//     osDelay(10);
//   }
// }

/* ============================================================
 *    系统初始化
 * ============================================================ */

/**
 * @brief 机械臂控制系统初始化
 *
 * 流程：
 *   1. 拷贝关节初始状态
 *   2. 创建事件队列和命令队列（FreeRTOS队列）
 *   3. 创建主控制任务 robot_control_task
 *   4. 创建命令服务任务 robot_cmd_service
 *
 * 注意：MQTT同步任务和远程控制任务目前被注释掉。
 */
void robot_init(void)
{
  /* 初始化关节数据和位姿矩阵 */
  memcpy(g_robot.joints, g_joint_init, sizeof(g_joint_init));
  memcpy(g_robot.T, T_0_6_reset, sizeof(T_0_6_reset));
  
  /* 创建事件队列（用于控制任务接收事件） */
  g_robot.event_queue = xQueueCreate(ROBOT_MAX_EVENT_NUM, sizeof(struct robot_event));
  if (g_robot.event_queue == NULL)
  {
    LOG("create robot event queue failed\n");
    return;
  }
  
  /* 创建命令队列（用于cmd_service任务接收命令） */
  g_robot.cmd_queue = xQueueCreate(ROBOT_CMD_MAX_NUM, sizeof(struct robot_cmd));
  if (g_robot.cmd_queue == NULL)
  {
    LOG("create robot cmd queue failed\n");
    return;
  }
  
  /* 创建主控制任务（最高优先级） */
  osThreadAttr_t task_attributes = 
  {
    .name = "robot_control_task",
    .stack_size = ROBOT_CONTROL_TASK_STACK_SIZE,
    .priority = ROBOT_CONTROL_TASK_PRIORITY
  };
  g_robot.control_handle = osThreadNew((osThreadFunc_t)robot_control_task, NULL, &task_attributes);
  if (g_robot.control_handle == NULL)
  {
    LOG("robot control task failed\n");
    return;
  }
  
  /* 创建命令服务任务（此高优先级） */
  task_attributes.name = "robot_cmd_service";
  task_attributes.stack_size = ROBOT_CMD_SERVICE_STACK_SIZE;
  task_attributes.priority = ROBOT_CMD_SERVICE_PRIORITY;
  g_robot.cmd_service_handle = osThreadNew((osThreadFunc_t)robot_cmd_service, NULL, &task_attributes);
  if (g_robot.cmd_service_handle == NULL)
  {
      LOG("create robot cmd service task failed\n");
      return;
  }

  // // 创建 NRF24L01 接收任务
  // osThreadAttr_t nrf_attr =
  // {
  //   .name = "nrf_receive_task",
  //   .stack_size = 2048,
  //   .priority = osPriorityRealtime1
  // };
  // g_robot.nrf_handle = osThreadNew(nrf_receive_task, NULL, &nrf_attr);
  // if (g_robot.nrf_handle == NULL)
  // {
  //     LOG("create robot g_robot.nrf_task failed\n");
  //     return;
  // }
}

/* ============================================================
 *    GPIO EXTI 回调
 * ============================================================ */

/**
 * @brief 根据GPIO引脚号反查关节ID
 * @param pin  GPIO引脚号
 * @return 关节ID（0~5），未找到返回 -1
 */
static int robot_joint_pin2id(uint16_t pin)
{
  for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++)
  {
    if (g_robot.joints[i].limit_gpio_pin == pin)
    {
      return i;
    }
  }
  return -1;
}

/**
 * @brief GPIO外部中断回调函数
 *
 * 当限位开关触发时（双边沿触发），根据引脚查找对应关节，调用限位处理。
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  int joint_id = robot_joint_pin2id(GPIO_Pin);

  if (joint_id < 0 || joint_id >= ROBOT_MAX_JOINT_NUM){return;}
  
  robot_joint_limit_happend(joint_id);
  LOG_FROM_ISR("joint limit switch happened joint id: %d\n", joint_id);
}

/* ============================================================
 *    PID 单周期控制
 * ============================================================ */

/* 调试用全局变量 */
float g_target_angle[ROBOT_MAX_JOINT_NUM] = {0};
float g_current_angle[ROBOT_MAX_JOINT_NUM] = {0};

/**
 * @brief PID 单周期控制（一个 PID_PERIOD 内）
 *
 * 对每个关节：读取当前角度 → 计算误差 → PID计算输出速度 → 发送速度指令。
 * 此函数会阻塞等待直到本周期结束（精确时控）。
 *
 * @param target_angle  各关节目标角度数组
 * @param intg_error    各关节误差积分（保持状态，跨周期累积）
 * @param pre_error     各关节上一次误差（保持状态，用于计算微分）
 * @param total_error   各关节累计误差（仅调试统计）
 * @param joint_num     关节数量
 */
static void robot_pid_one_period(float *target_angle, float *intG_error, float *pre_error, float *total_error, int joint_num)
{
  float error = 0;
  float v = 0;
  uint32_t pid_end_time = HAL_GetTick() + ROBOT_PID_PERIOD;

  for (int i = 0; i < joint_num; i++)
  {
    robot_update_current_angle(i);       //通过can读取当前角度
    g_current_angle[i] = g_robot.joints[i].current_angle;

    //计算角度误差
    error = robot_angle_diff(g_current_angle[i], target_angle[i]);
    intG_error[i] += error;
    if (total_error != NULL)
    {
      total_error[i] += fabs(error);
    }
    
    // PID控制律: v = Kp*e + Ki*∫e + Kd*Δe
    v = ROBOT_PID_KP * error + ROBOT_PID_KI * intG_error[i] + ROBOT_PID_KD * (error - pre_error[i]);
    pre_error[i] = error;
    robot_joint_veloccity_to(i, v, ROBOT_JOINT_DEFAULT_ACCELERATION);
  }

  //等待本PID周期结束（保证精确的时间控制）
  uint32_t time = HAL_GetTick();
  if (time < pid_end_time)
  {
    osDelay(pid_end_time - time);
  }
  
}

/* ============================================================
 *    读取当前关节角度（通过CAN）
 * ============================================================ */

/**
 * @brief 通过CAN总线读取电机当前位置，更新关节角度
 *
 * 发送读位置指令(S_CPOS)，阻塞等待CAN应答帧，解析角度值。
 * 角度计算：CAN返回的编码器值 → 关节角度(度)
 *
 *   angle = raw_angle * 360 / 65536 / reduction_ratio + init_angle
 *
 * @param joint_id  关节ID（0~5）
 * @return 成功返回 0，超时或数据校验失败返回 1
 */
static int robot_update_current_angle(uint8_t joint_id)
{
  struct joint *joint = &g_robot.joints[joint_id];

  vTaskSuspendAll();
  can.rxFrameFlag = false;
  uint32_t start_tick = HAL_GetTick();

  while (!can.rxFrameFlag)
  {
    if (HAL_GetTick() - start_tick > ROBOT_CAN_TIMEOUT)
    {
      LOG("joint %u update current angle timeout.\n", joint_id);
      xTaskResumeAll();
      return 1;
    }
    Emm_V5_Read_Sys_Params(joint_id + 1, S_CPOS);
    osDelay(1);
  }
  xTaskResumeAll();
  taskENTER_CRITICAL();
  
  uint8_t id = (uint8_t)(can.CAN_RxMsg.ExtId >> 8) - 1;
  
  //校验返回数据格式
  if ((can.rxData[0] != 0x36) || (can.rxData[6] != 0x6b) || (id != joint_id))
  {
    taskEXIT_CRITICAL();
    return 1;
  }
  
  //解析编码器值（大端序）
  float angle = 0;
  for (int i = 5; i >= 2; i--)
  {
    angle += (float)(((uint32_t)can.rxData[i]) << ((5 - i) <<3));
  }
  
  //处理符号位
  if (can.rxData[1] == 0x01)
  {
    angle = -angle;
  }
  
  //修正关节为正方向
  if (joint->postive_direction == MOTOR_DIR_CCW)
  {
    angle = -angle;
  }
  
  taskEXIT_CRITICAL();
  
  //编码器数值转化为角度（编码器为绝对位置，基准应为关节初始角度而非累加）
  angle = angle * 360 / 65535 /joint->reduction_ratio + g_joint_init[joint_id].current_angle;
  joint->current_angle = robot_angle_normalize(angle);
  return 0;
}

/**
 * @brief 角度归一化到 [0, 360) 范围
 *
 * @param angle  输入角度（假设在 -360 ~ 720 范围内）
 * @return 归一化后的角度
 */
static float robot_angle_normalize(float angle)
{
  /* 循环归一化到 [0, 360)，可处理任意圈数 */
  angle = fmod(angle, 360.0f);
  if (angle < 0.0f)
  {
    angle += 360.0f;
  }
  return angle;
}

/**
 * @brief 计算当前角度与目标角度之间的最小差值（考虑角度循环）
 *
 * 将差值映射到 [-180, 180] 范围，确保得到两个角度间的最短路径差。
 * 考虑了角度在 0~360 范围内循环的特性。
 *
 * @param cur_angle    当前角度（度）
 * @param target_angle 目标角度（度）
 * @return 最小角度差（度），范围 [-180, 180]
 */
static float robot_angle_diff(float cur_angle, float target_angle)
{
  float dif = target_angle - cur_angle;
  if (dif > 180)
  {
    dif -= 360;
  }
  if (dif < -180)
  {
    dif += 360;
  }
  return dif;
}
