#ifndef SURRIBOT_IPC_MSGS_H
#define SURRIBOT_IPC_MSGS_H

#include <stdint.h>

// M7 → M4: cmd_vel 명령
struct __attribute__((packed)) IpcCmdVel {
  uint8_t  msg_id;       // 0x01
  float    linear_x;     // m/s
  float    angular_z;    // rad/s
};

// M4 → M7: 오도메트리 + IMU 데이터
struct __attribute__((packed)) IpcOdomImu {
  uint8_t  msg_id;       // 0x02
  // odometry
  float    odom_x;       // m
  float    odom_y;       // m
  float    odom_theta;   // rad
  float    vel_x;        // m/s
  float    vel_theta;    // rad/s
  // imu (가속도 + 자이로)
  float    imu_ax, imu_ay, imu_az;   // m/s^2
  float    imu_gx, imu_gy, imu_gz;   // rad/s
};

#define IPC_MSG_CMD_VEL   0x01
#define IPC_MSG_ODOM_IMU  0x02

#endif
