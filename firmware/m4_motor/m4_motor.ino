// ================================================
// SurriBot M4 Core — 모터 PID + 엔코더 + IMU
// ================================================
// 현재: 하드웨어 미연결 → 시뮬레이션 오도메트리
// 추후: 실제 엔코더/IMU 읽기 + PID 제어
// ================================================

#include "RPC.h"
#include "ipc_msgs.h"

// --- PID 루프 주기 ---
#define PID_RATE_HZ    50
#define PID_DT         (1.0f / PID_RATE_HZ)

// --- 로봇 파라미터 (추후 실측) ---
#define WHEEL_BASE     0.17f    // 바퀴 간 거리 (m)
#define MAX_LINEAR     0.5f     // 최대 선속도 (m/s)
#define MAX_ANGULAR    2.0f     // 최대 각속도 (rad/s)

// --- 현재 명령 ---
volatile float cmd_linear_x  = 0.0f;
volatile float cmd_angular_z = 0.0f;

// --- 시뮬레이션 오도메트리 ---
float odom_x     = 0.0f;
float odom_y     = 0.0f;
float odom_theta = 0.0f;

// M7로부터 cmd_vel 수신 콜백
void recv_callback(const uint8_t *buf, size_t len) {
  if (len >= sizeof(IpcCmdVel) && buf[0] == IPC_MSG_CMD_VEL) {
    const IpcCmdVel *msg = (const IpcCmdVel *)buf;
    cmd_linear_x  = constrain(msg->linear_x,  -MAX_LINEAR,  MAX_LINEAR);
    cmd_angular_z = constrain(msg->angular_z, -MAX_ANGULAR, MAX_ANGULAR);
  }
}

void setup() {
  if (!RPC.begin()) {
    // RPC 실패 시 빨간 LED 깜빡
    pinMode(LEDR, OUTPUT);
    while (1) { digitalWrite(LEDR, !digitalRead(LEDR)); delay(200); }
  }
  RPC.attach(recv_callback);

  pinMode(LEDG, OUTPUT);
  digitalWrite(LEDG, LOW);  // 초록 LED on = M4 동작 중
}

void loop() {
  // === PID 루프 (50Hz) ===
  static unsigned long last_pid = 0;
  unsigned long now = millis();
  if (now - last_pid < (1000 / PID_RATE_HZ)) return;
  last_pid = now;

  // --- 시뮬레이션: diff-drive 키네매틱스 ---
  // 추후 여기에 실제 엔코더 읽기 + PID 제어 코드 교체
  float vx = cmd_linear_x;
  float vth = cmd_angular_z;

  odom_theta += vth * PID_DT;
  // theta를 -PI ~ PI로 정규화
  while (odom_theta >  3.14159f) odom_theta -= 6.28318f;
  while (odom_theta < -3.14159f) odom_theta += 6.28318f;

  odom_x += vx * cosf(odom_theta) * PID_DT;
  odom_y += vx * sinf(odom_theta) * PID_DT;

  // --- IMU 시뮬레이션 (가속도/자이로) ---
  // 추후 실제 MPU6050 I2C 읽기로 교체
  float imu_ax = 0.0f, imu_ay = 0.0f, imu_az = 9.81f;
  float imu_gx = 0.0f, imu_gy = 0.0f, imu_gz = vth;

  // --- M7으로 odom+imu 전송 ---
  IpcOdomImu msg;
  msg.msg_id     = IPC_MSG_ODOM_IMU;
  msg.odom_x     = odom_x;
  msg.odom_y     = odom_y;
  msg.odom_theta = odom_theta;
  msg.vel_x      = vx;
  msg.vel_theta  = vth;
  msg.imu_ax     = imu_ax;
  msg.imu_ay     = imu_ay;
  msg.imu_az     = imu_az;
  msg.imu_gx     = imu_gx;
  msg.imu_gy     = imu_gy;
  msg.imu_gz     = imu_gz;

  RPC.write((const uint8_t *)&msg, sizeof(msg), true);
}
