// ================================================
// SurriBot M7 Core — micro-ROS WiFi + LiDAR + RPC
// ================================================
// - /scan publish (LiDAR, 현재 시뮬레이션)
// - /cmd_vel subscribe → M4로 전달
// - M4로부터 odom/imu 수신 → /odom publish
// ================================================

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

#include "RPC.h"
#include "ipc_msgs.h"

// --- WiFi & Agent ---
#define WIFI_SSID     "your_2.4ghz_ssid"
#define WIFI_PASS     "your_password"
#define AGENT_IP      "192.168.0.27"
#define AGENT_PORT    8888

// --- LiDAR 시뮬레이션 ---
#define SCAN_NUM_READINGS  460
#define SCAN_FREQ_HZ       10.0f
#define SCAN_RANGE_MIN     0.15f
#define SCAN_RANGE_MAX     12.0f
#define ROOM_W  4.0f
#define ROOM_H  3.0f
#define OBS_X   1.5f
#define OBS_Y   1.0f
#define OBS_R   0.3f

// --- LED ---
#define LED_PIN LED_BUILTIN

// --- micro-ROS 객체 ---
rcl_publisher_t pub_scan;
rcl_publisher_t pub_odom;
rcl_subscription_t sub_cmd_vel;
sensor_msgs__msg__LaserScan msg_scan;
nav_msgs__msg__Odometry msg_odom;
geometry_msgs__msg__Twist msg_cmd_vel;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer_scan;

static float ranges_buf[SCAN_NUM_READINGS];

// M4에서 수신한 odom/imu
volatile IpcOdomImu latest_odom_imu = {};
volatile bool odom_updated = false;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
}

// --- M4로부터 odom/imu 수신 ---
void rpc_recv_callback(const uint8_t *buf, size_t len) {
  if (len >= sizeof(IpcOdomImu) && buf[0] == IPC_MSG_ODOM_IMU) {
    memcpy((void *)&latest_odom_imu, buf, sizeof(IpcOdomImu));
    odom_updated = true;
  }
}

// --- cmd_vel → M4 전달 ---
void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  IpcCmdVel ipc;
  ipc.msg_id    = IPC_MSG_CMD_VEL;
  ipc.linear_x  = (float)msg->linear.x;
  ipc.angular_z = (float)msg->angular.z;
  RPC.write((const uint8_t *)&ipc, sizeof(ipc), true);
}

// --- raycasting (LiDAR 시뮬레이션) ---
float cast_ray(float angle) {
  float dx = cosf(angle), dy = sinf(angle);
  float dist = SCAN_RANGE_MAX;

  if (dx > 1e-6f)  { float t = (ROOM_W/2.0f)/dx;  if (fabsf(t*dy) <= ROOM_H/2.0f && t < dist) dist = t; }
  if (dx < -1e-6f) { float t = (-ROOM_W/2.0f)/dx; if (fabsf(t*dy) <= ROOM_H/2.0f && t < dist) dist = t; }
  if (dy > 1e-6f)  { float t = (ROOM_H/2.0f)/dy;  if (fabsf(t*dx) <= ROOM_W/2.0f && t < dist) dist = t; }
  if (dy < -1e-6f) { float t = (-ROOM_H/2.0f)/dy; if (fabsf(t*dx) <= ROOM_W/2.0f && t < dist) dist = t; }

  float b = 2.0f * (-OBS_X*dx - OBS_Y*dy);
  float c = OBS_X*OBS_X + OBS_Y*OBS_Y - OBS_R*OBS_R;
  float det = b*b - 4.0f*c;
  if (det >= 0.0f) { float t1 = (-b - sqrtf(det)) / 2.0f; if (t1 > SCAN_RANGE_MIN && t1 < dist) dist = t1; }
  return dist;
}

// --- 10Hz scan publish + odom publish ---
void timer_scan_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;

  unsigned long ms = millis();

  // --- /scan publish ---
  msg_scan.header.stamp.sec = (int32_t)(ms / 1000);
  msg_scan.header.stamp.nanosec = (uint32_t)((ms % 1000) * 1000000UL);

  for (int i = 0; i < SCAN_NUM_READINGS; i++) {
    float angle = msg_scan.angle_min + i * msg_scan.angle_increment;
    float d = cast_ray(angle) + ((float)(random(-20, 21)) / 1000.0f);
    if (d < SCAN_RANGE_MIN) d = SCAN_RANGE_MIN;
    if (d > SCAN_RANGE_MAX) d = SCAN_RANGE_MAX;
    ranges_buf[i] = d;
  }
  RCSOFTCHECK(rcl_publish(&pub_scan, &msg_scan, NULL));

  // --- /odom publish (M4 데이터) ---
  if (odom_updated) {
    odom_updated = false;
    IpcOdomImu snap;
    memcpy(&snap, (const void *)&latest_odom_imu, sizeof(IpcOdomImu));

    msg_odom.header.stamp.sec = (int32_t)(ms / 1000);
    msg_odom.header.stamp.nanosec = (uint32_t)((ms % 1000) * 1000000UL);

    msg_odom.pose.pose.position.x = snap.odom_x;
    msg_odom.pose.pose.position.y = snap.odom_y;
    msg_odom.pose.pose.position.z = 0.0;

    // theta → quaternion (2D: qw = cos(θ/2), qz = sin(θ/2))
    float half_theta = snap.odom_theta * 0.5f;
    msg_odom.pose.pose.orientation.w = cosf(half_theta);
    msg_odom.pose.pose.orientation.x = 0.0;
    msg_odom.pose.pose.orientation.y = 0.0;
    msg_odom.pose.pose.orientation.z = sinf(half_theta);

    msg_odom.twist.twist.linear.x = snap.vel_x;
    msg_odom.twist.twist.angular.z = snap.vel_theta;

    RCSOFTCHECK(rcl_publish(&pub_odom, &msg_odom, NULL));
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // M4 부팅 + RPC 초기화
  if (!RPC.begin()) {
    Serial.println("RPC.begin() failed");
    error_loop();
  }
  RPC.attach(rpc_recv_callback);
  Serial.println("M4 booted via RPC");

  // WiFi + micro-ROS
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "portenta_node", "", &support));

  // /scan publisher (reliable)
  RCCHECK(rclc_publisher_init_default(
    &pub_scan, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan"));

  // /odom publisher (reliable)
  RCCHECK(rclc_publisher_init_default(
    &pub_odom, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));

  // /cmd_vel subscriber (best_effort)
  RCCHECK(rclc_subscription_init_best_effort(
    &sub_cmd_vel, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  // 10Hz 타이머
  RCCHECK(rclc_timer_init_default(
    &timer_scan, &support, RCL_MS_TO_NS(100), timer_scan_callback));

  // executor: timer + subscriber = 2 handles
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_scan));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &sub_cmd_vel, &msg_cmd_vel, &cmd_vel_callback, ON_NEW_DATA));

  // --- LaserScan 메시지 초기화 ---
  msg_scan.header.frame_id.data = (char *)"laser";
  msg_scan.header.frame_id.size = 5;
  msg_scan.header.frame_id.capacity = 6;
  msg_scan.angle_min = 0.0f;
  msg_scan.angle_max = 2.0f * M_PI;
  msg_scan.angle_increment = (2.0f * M_PI) / SCAN_NUM_READINGS;
  msg_scan.time_increment = (1.0f / SCAN_FREQ_HZ) / SCAN_NUM_READINGS;
  msg_scan.scan_time = 1.0f / SCAN_FREQ_HZ;
  msg_scan.range_min = SCAN_RANGE_MIN;
  msg_scan.range_max = SCAN_RANGE_MAX;
  msg_scan.ranges.data = ranges_buf;
  msg_scan.ranges.size = SCAN_NUM_READINGS;
  msg_scan.ranges.capacity = SCAN_NUM_READINGS;
  msg_scan.intensities.data = NULL;
  msg_scan.intensities.size = 0;
  msg_scan.intensities.capacity = 0;

  // --- Odometry 메시지 초기화 ---
  msg_odom.header.frame_id.data = (char *)"odom";
  msg_odom.header.frame_id.size = 4;
  msg_odom.header.frame_id.capacity = 5;
  msg_odom.child_frame_id.data = (char *)"base_link";
  msg_odom.child_frame_id.size = 9;
  msg_odom.child_frame_id.capacity = 10;

  digitalWrite(LED_PIN, LOW);
  Serial.println("=== SurriBot M7 ready ===");
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
}
