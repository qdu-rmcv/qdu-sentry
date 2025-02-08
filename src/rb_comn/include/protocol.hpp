/*
  视觉与电控通信协议
*/

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AI_ARM_INFANTRY (1 << 0)
#define AI_ARM_HERO (1 << 1)
#define AI_ARM_ENGINEER (1 << 2)
#define AI_ARM_DRONE (1 << 3)
#define AI_ARM_SENTRY (1 << 4)
#define AI_ARM_DART (1 << 5)
#define AI_ARM_RADAR (1 << 6)

#define AI_NOTICE_FIRE (1 << 1)
// #define AI_NOTICE_BUFF (1 << 2)     //
// #define AI_NOTICE_SNIPE (1 << 3)    //
// #define AI_NOTICE_SLOPE (1 << 4)    //上坡
// #define AI_NOTICE_OUTPOST (1 << 5)  //前哨
// #define AI_NOTICE_SENTRY (1 << 6)   //
// #define AI_NOTICE_BASE (1 << 7)     //基地

#define AI_NOTICE_NOTHING (3)
#define AI_NOTICE_AUTO_AIM (5)

#define AI_ID_MCU (0xC4)
#define AI_ID_REF (0xA8)

#define AI_TEAM_RED (1 << 0)
#define AI_TEAM_BLUE (1 << 1)

#define AI_RACE_RMUC (1 << 0)
#define AI_RACE_RMUT (1 << 1)
#define AI_RACE_RMUL1 (1 << 2)
#define AI_RACE_RMUL3 (1 << 3)

#define AI_RFID_BUFF (1 << 0)
#define AI_RFID_SNIP (1 << 1)

#define AI_GO_TO_OUTPOST (1 << 2) //4
#define AI_GO_TO_PATROL_AREA (1 << 3) //8
#define AI_GO_TO_SUPPLY (1 << 4)

typedef uint8_t Protocol_ID_t;

/* 电控 -> 视觉 MCU数据结构体*/
typedef struct __attribute__((packed)) {
  struct __attribute__((packed)) {
    float q0;
    float q1;
    float q2;
    float q3;
  } quat; /* 四元数 */

  Protocol_ID_t notice; /* 控制命令 */

  float ball_speed; /* 子弹初速度 */

  float chassis_speed; /* 底盘速度(哨兵) */
} Protocol_UpDataMCU_t;

/* 电控 -> 视觉 裁判系统数据结构体*/
typedef struct __attribute__((packed)) {
  uint16_t team;         /* 本身队伍 */
  uint16_t time;         /* 比赛开始时间 */
  uint8_t race;          /* 比赛类型 */
  uint8_t rfid;          /* 增益地点 */
  uint8_t base_hp;       /* 基地血量 */
  uint8_t sentry_hp;     /* 哨兵血量 */
  uint8_t ballet_remain; /* 剩余弹量 */
  uint8_t arm;           /* 兵种信息 */
  uint8_t x;
  uint8_t y;
  uint8_t target_x;
  uint8_t target_y;
} Protocol_UpDataReferee_t;

/* 视觉 -> 电控 数据结构体*/
typedef struct __attribute__((packed)) {
  struct __attribute__((packed)) {
    float yaw; /* 偏航角(Yaw angle) */
    float pit; /* 俯仰角(Pitch angle) */
    float rol; /* 翻滚角(Roll angle) */
  } gimbal;    /* 欧拉角 */

  Protocol_ID_t notice; /* 控制命令 */

  struct __attribute__((packed)) {
    float vx; /* x轴移动速度 */
    float vy; /* y轴移动速度、哨兵沿轨道方向(正面面向战场，右为正方向) */
    float wz; /* z轴转动速度、哨兵射界(弧度0～pi) */
  } chassis_move_vec; /* 底盘移动向量 */

  struct __attribute__((packed)) {
    float x;
    float y;
    float z;
    float pitch;
    float roll;
    float yaw;
  } extern_channel; /* 额外控制通道 */

} Protocol_DownData_t;

typedef struct __attribute__((packed)) {
  Protocol_UpDataMCU_t data;
  uint16_t crc16;
} Protocol_UpPackageMCU_t;

typedef struct __attribute__((packed)) {
  Protocol_UpDataReferee_t data;
  uint16_t crc16;
} Protocol_UpPackageReferee_t;

typedef struct __attribute__((packed)) {
  Protocol_DownData_t data;
  uint16_t crc16;
} Protocol_DownPackage_t;

#ifdef __cplusplus
}
#endif