#ifndef __ROBOT_H__
#define __ROBOT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "stdint.h"
#include "main.h"
#include "cmsis_os2.h"
#include "queue.h"
#include "string.h"
#include <math.h>
#include "stdbool.h"

/* ============================================================
 *   机械臂控制框架的头文件
 *   定义结构体、枚举、宏常量及全局接口函数
 * ============================================================ */

/* ---------- 基本常量 ---------- */
#define ROBOT_MAX_JOINT_NUM                 6       /* 机械臂关节数量（6轴） */
#define ROBOT_MAX_EVENT_NUM                 20      /* 事件队列最大长度 */

#define ROBOT_MQTT_ENABLE                   0U      /* 是否使能MQTT服务（0=关闭, 1=开启） */

/* ---------- 命令处理 ---------- */
#define ROBOT_CMD_MAX_NUM                   50      /* 命令队列最大长度 */
#define ROBOT_CMD_LENGTH                    128     /* 单条命令最大长度（字节） */
#define ROBOT_CMD_QUEUE_TIMEOUT             100     /* 发送命令到队列的超时时间（ms） */

/* ---------- 任务栈大小与优先级 ---------- */
#define ROBOT_CONTROL_TASK_STACK_SIZE       4096    /* 主控制任务栈大小（字节） */
#define ROBOT_CONTROL_TASK_PRIORITY         (osPriorityRealtime3)  /* 主控制任务优先级 */

#define ROBOT_CMD_SERVICE_STACK_SIZE        2048    /* 命令服务任务栈大小（字节） */
#define ROBOT_CMD_SERVICE_PRIORITY          (osPriorityRealtime2)  /* 命令服务任务优先级 */

#define ROBOT_MQTT_SYNC_TASK_STACK_SIZE     2048    /* MQTT同步任务栈大小（字节） */
#define ROBOT_MQTT_SYNC_TASK_PRIORITY       (osPriorityRealtime)   /* MQTT同步任务优先级 */

// #define ROBOT_REMOTE_SERVICE_STACK_SIZE     2048
// #define ROBOT_REMOTE_SERVICE_PRIORITY       (osPriorityRealtime1)
// #define ROBOT_REMOTE_RESULT_NUM             3

/* ---------- 远程控制参数 ---------- */
#define ROBOT_REMOTE_MAX_VELOCITY           (20.0f)  /* 末端最大线速度（mm/s） */
#define ROBOT_REMOTE_MAX_RPM                (5.0f)   /* 末端最大旋转角速度（rpm） */
#define ROBOT_REMOTE_TIME_RESOLUTION        (50)     /* 远程控制时间插值分辨率（ms） */

/* ---------- 关节运动默认参数 ---------- */
#define ROBOT_JOINT_DEFAULT_VELOCITY        (1.0f)  /* 关节默认速度（rpm） */
#define ROBOT_JOINT_DEFAULT_ACCELERATION    200      /* 关节默认加速度 */

/* ---------- 路径插值参数 ---------- */
#define ROBOT_INTERPOLATION_TIME_RESOLUTION (100)    /* 时间插值分辨率（ms） */
#define ROBOT_INTERPOLATION_RESOLUTION      (1.0f)   /* 空间路径插值分辨率（mm） */

/* ---------- 复位参数 ---------- */
#define ROBOT_RESET_DEFAULT_ANGLE           360      /* 复位默认旋转角度（度） */
#define ROBOT_RESET_DEFAULT_VELOCITY        (10.0f)  /* 复位默认速度（rpm） */
#define ROBOT_RESET_DEFAULT_ACCELERATION    100      /* 复位默认加速度 */

/* ---------- 机械臂状态标志位 ---------- */
#define ROBOT_STATUS_LIMIT_ENABLE           0U       /* 位0: 限位开关已使能 */
#define ROBOT_STATUS_LIMIT_HAPPENED         1U       /* 位1: 限位开关已触发 */
#define ROBOT_STATUS_READY                  2U       /* 位2: 当前运动已完成 */
#define ROBOT_STATUS_RMODE_ENABLE           3U       /* 位3: 远程控制模式已使能 */
#define ROBOT_STATUS_MQTT_CONNECTED         4U       /* 位4: MQTT已连接 */

/* 状态位操作宏 */
#define ROBOT_STATUS_IS(x, status)          (((x) & (1 << status)) != 0)           /* 检查状态位是否置位 */
#define ROBOT_STATUS_SET(x, status)         (x = ((x) | (1 << status)))            /* 置位状态位 */
#define ROBOT_STATUS_CLEAR(x, status)       (x = ((x) & ~(1 << status)))           /* 清除状态位 */

/* ---------- CAN通信 ---------- */
#define ROBOT_CAN_DELAY                     5       /* CAN发送延时（ms） */
#define ROBOT_CAN_TIMEOUT                   (10)    /* CAN通信超时（ms） */

/* ---------- PID控制参数 ---------- */
#define ROBOT_PID_KP                        (10.0f)  /* 比例系数 Kp */
#define ROBOT_PID_KI                        (0.002f) /* 积分系数 Ki */
#define ROBOT_PID_KD                        (0.0f)   /* 微分系数 Kd */
#define ROBOT_PID_PERIOD                    (20)     /* PID控制周期（ms），建议不小于10ms */

/* ---------- 误差范围 ---------- */
#define ROBOT_ERROR_RANGE                   (1e-4f)  /* 浮点数误差容限 */
#define ROBOT_JOINT_ANGLE_ERROR_RANGE       (1e-1f)  /* 关节角度误差容限（度） */

#define ROBOT_MQTT_SYNC_TIME                (100)    /* MQTT同步时间间隔（ms） */

#define M_PI		3.14159265358979323846

enum motor_dir
{
  MOTOR_DIR_CW = 0,   //电机正转
  MOTOR_DIR_CCW = 1,  //电机反转
};

enum dir
{
  DIR_POSITIVE = 1,     /* 正方向 */
  DIR_NEGATIVE = -1,     /* 负方向 */
};

/* 空间位置结构体 */
struct position
{
  float x;  /* X方向位移 mm */
  float y;  /* Y方向位移 mm */
  float z;  /* Z方向位移 mm */
};

struct joint
{
  /* ===== 初始化参数（由 g_joints_init 设置，不可运行时修改位置） ===== */
  float current_angle;                /* 关节当前角度 */
  enum motor_dir postive_direction;   /* 关节旋转正方向对应的电机旋转方向 */
  float reduction_ratio;              /*减速比 */
  GPIO_TypeDef *limit_gpio_port;      /* 限位开关GPIO端口 */
  uint16_t limit_gpio_pin;            /* 限位开关GPIO引脚 */
  float min_angle;                    /* 关节最小角度 */
  float max_angle;                    /* 关节最大角度 */
  enum dir reset_dir;                 /* 复位方向 */

  /* 上面数据用于初始化，请不要修改数据位置 */
  volatile uint32_t status;           /* 关节状态 */
  float velocity;                     /* 关节速度 */
  float acceleration;                 /* 关节加速度 */
};

/** @brief 机械臂末端的姿态（绕xyz轴的旋转量） */
struct rotate
{
  float x;
  float y;
  float z;
};

/**
 * @struct robot
 * @brief 机械臂全局控制实例
 *
 * 包含所有任务句柄、关节数组、事件/命令队列、当前末端位姿等信息。
 * 全局唯一实例为 g_robot。
 */
struct robot
{
  osThreadId_t control_handle;           /* 主控制任务句柄 */
  osThreadId_t cmd_service_handle;       /* 命令服务任务句柄 */
  osThreadId_t mqtt_sync_task_handle;    /* MQTT同步任务句柄 */
  osThreadId_t remote_service_handle;    /* 远程控制服务任务句柄 */
  float T[4][4];                         /* 当前末端位姿的4x4变换矩阵 */
  struct joint joints[ROBOT_MAX_JOINT_NUM]; /* 6个关节的状态数组 */
  uint32_t status;                       /* 机械臂整体状态标志位 */
  QueueHandle_t event_queue;             /* 事件队列句柄（FreeRTOS队列） */
  QueueHandle_t cmd_queue;               /* 命令队列句柄（FreeRTOS队列） */
  osThreadId_t nrf_handle;
  struct position cur_pos;               /* 当前末端在世界坐标系下的位置 */
  struct rotate cur_rot;                 /* 当前末端姿态 */
};

/* 关节ID枚举（便于可读性） */
enum
{
  ROBOT_JOINT_1 = 0,
  ROBOT_JOINT_2,
  ROBOT_JOINT_3,
  ROBOT_JOINT_4,
  ROBOT_JOINT_5,
  ROBOT_JOINT_6,
};

/** @brief 命令来源类型 */
enum cmd_type
{
  CMD_TYPE_UART3 = 0,  /* 来自USART3（串口调试）的命令 */
  CMD_TYPE_MQTT,       /* 来自MQTT（ESP8266 WiFi）的命令 */
};

enum robot_event_type
{
  ROBOT_JOINT_REL_ROTATE = 0,     /* 相对旋转: 指定关节相对当前位置旋转指定角度 */
  ROBOT_JOINT_ABS_ROTATE,         /* 绝对旋转: 指定关节旋转到指定角度 */
  ROBOT_LIMIT_SWITCH_EVENT,       /* 限位开关触发事件 */
  ROBOT_AUTO_EVENT,               /* 自动运动: 根据末端目标位置自动控制各关节 */
  ROBOT_TIMIE_FUNC_EVENT,         /* 时间函数运动: 末端按照时间函数P(t)轨迹运动 */
  ROBOT_HARD_RESET_EVENT,         /* 硬复位: 利用限位开关复位到初始位置 */
  ROBOT_SOFT_RESET_EVENT,         /* 软复位: 不经过限位开关，直接运动到初始位置 */
  ROBOT_TEST_EVENT,               /* 测试事件 */
  ROBOT_REMOTE_CONTROL_EVENT,     /* 远程控制事件 */
  ROBOT_JOINTS_SYNC_EVENT,        /* 关节同步事件: 所有关节同时运动到指定角度 */
  ROBOT_SET_HOME_EVENT            /*设置初始位置（零点），给软复位用*/
};
/**
 * @struct robot_event
 * @brief 机械臂事件数据包
 *
 * 通过FreeRTOS队列在任务间传递。
 */
struct robot_event
{
  enum robot_event_type type;    //事件类型
  uint8_t joint_id;              //目标关节ID
  float param[6];                 //事件参数（含义根据事件类型而异）
};

/**
 * @struct robot_cmd
 * @brief 机械臂命令数据包
 *
 * 从串口中断或MQTT回调中收到原始字符串后，
 * 封装为命令包投递到命令队列，由 cmd_service 任务处理。
 */
struct robot_cmd
{
  enum cmd_type type;                    /* 命令来源 */
  char cmd[ROBOT_CMD_LENGTH];           /* 原始命令字符串 */
};

extern const float D_H[6][4];            /* DH参数矩阵（6关节 x 4列） */
extern const float joint_weight[ROBOT_MAX_JOINT_NUM];   /* 各关节旋转权重（用于最优解选择） */
extern struct robot g_robot;              /* 机械臂全局控制实例 */

/* ============================================================
 *   全局函数声明
 * ============================================================ */

/* 初始化机械臂控制系统（创建任务和队列） */
void robot_init(void);
uint32_t robot_joint_veloccity_to(uint32_t joint_id, float velocity, uint8_t acceleration);
void robot_cmd_service(void *arg);


#ifdef __cplusplus
}
#endif

#endif /* __ROBOT_H__ */
