// ================================================
// SurriBot M7 Core — micro-ROS WiFi + LiDAR
// ================================================
// - /scan publish (RPLidar C1 via LDS library)
// - Breakout.UART0 → RPLidar C1 (460800 baud)
// - WiFi → micro-ROS agent (UDP)
// ================================================

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>

#include <Arduino_PortentaBreakout.h>
#include <LDS_RPLIDAR_C1.h>

// --- WiFi & Agent (wifi_config.h) ---
#include "wifi_config.h"

#if !defined(WIFI_SSID) || !defined(WIFI_PASS) || !defined(AGENT_IP) || !defined(AGENT_PORT)
  #error "wifi_config.h is missing or incomplete. Copy wifi_config.h.example to wifi_config.h and fill in your credentials."
#endif

// --- LiDAR ---
#define SCAN_NUM_READINGS  360
#define SCAN_FREQ_HZ       5.5f
#define SCAN_RANGE_MIN     0.15f
#define SCAN_RANGE_MAX     12.0f
#define LidarSerial        Breakout.UART0

LDS_RPLIDAR_C1 lidar;

// --- LED ---
#define LED_PIN LED_BUILTIN

// --- micro-ROS 객체 ---
rcl_publisher_t pub_scan;
sensor_msgs__msg__LaserScan msg_scan;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// 더블 버퍼: filling에 쓰고, ready에서 읽음
static float buf_a[SCAN_NUM_READINGS];
static float buf_b[SCAN_NUM_READINGS];
static float* filling   = buf_a;
static float* ready_buf = buf_b;
static bool   has_data  = false;
volatile bool scan_ready = false;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// --- LiDAR 콜백 ---
int lidarRead() { return LidarSerial.read(); }
size_t lidarWrite(const uint8_t* buf, size_t len) { return LidarSerial.write(buf, len); }

void lidarScanCallback(float angle_deg, float distance_mm,
    float quality, bool startOfNewScan) {
  if (startOfNewScan && has_data) {
    // 스캔 완료 → 버퍼 스왑
    float* tmp = ready_buf;
    ready_buf  = filling;
    filling    = tmp;
    scan_ready = true;
    memset(filling, 0, SCAN_NUM_READINGS * sizeof(float));
    has_data = false;
  }
  if (distance_mm <= 0) return;
  float dist_m = distance_mm / 1000.0f;
  if (dist_m < SCAN_RANGE_MIN || dist_m > SCAN_RANGE_MAX) return;
  int bin = (int)(angle_deg * SCAN_NUM_READINGS / 360.0f);
  if (bin >= 0 && bin < SCAN_NUM_READINGS) {
    filling[bin] = dist_m;
    has_data = true;
  }
}

void error_loop() {
  while (1) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // WiFi + micro-ROS (먼저 연결)
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
  delay(2000);
  Serial.println("WiFi connected");

  // LiDAR 초기화 (WiFi 이후)
  lidar.setSerialReadCallback(lidarRead);
  lidar.setSerialWriteCallback(lidarWrite);
  lidar.setScanPointCallback(lidarScanCallback);
  lidar.init();
  LidarSerial.begin(lidar.getSerialBaudRate());
  lidar.start();
  Serial.println("RPLidar C1 started");

  allocator = rcl_get_default_allocator();

  // Agent 연결 재시도 (최대 30회, 2초 간격)
  rcl_ret_t rc;
  for (int attempt = 1; attempt <= 30; attempt++) {
    rc = rclc_support_init(&support, 0, NULL, &allocator);
    if (rc == RCL_RET_OK) break;
    Serial.print("Agent retry ");
    Serial.println(attempt);
    delay(2000);
  }
  if (rc != RCL_RET_OK) error_loop();

  RCCHECK(rclc_node_init_default(&node, "portenta_node", "", &support));

  // /scan publisher (reliable — XRCE-DDS best_effort는 MTU 초과 메시지 프래그먼테이션 불가)
  RCCHECK(rclc_publisher_init_default(
    &pub_scan, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan"));

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
  msg_scan.ranges.data = ready_buf;
  msg_scan.ranges.size = SCAN_NUM_READINGS;
  msg_scan.ranges.capacity = SCAN_NUM_READINGS;
  msg_scan.intensities.data = NULL;
  msg_scan.intensities.size = 0;
  msg_scan.intensities.capacity = 0;

  digitalWrite(LED_PIN, LOW);
  Serial.println("=== SurriBot LiDAR test ready ===");
}

void loop() {
  lidar.loop();

  if (scan_ready) {
    scan_ready = false;

    // 유효 포인트 수 확인 — UART 오버플로로 비어있는 스캔 스킵
    int valid = 0;
    for (int i = 0; i < SCAN_NUM_READINGS; i++) {
      if (ready_buf[i] > 0) valid++;
    }
    if (valid < 50) return;

    unsigned long ms = millis();
    msg_scan.header.stamp.sec = (int32_t)(ms / 1000);
    msg_scan.header.stamp.nanosec = (uint32_t)((ms % 1000) * 1000000UL);
    msg_scan.ranges.data = ready_buf;  // 스왑된 버퍼 반영
    RCSOFTCHECK(rcl_publish(&pub_scan, &msg_scan, NULL));
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}
