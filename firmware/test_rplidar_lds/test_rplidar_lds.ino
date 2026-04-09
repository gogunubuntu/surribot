// ================================================
// RPLidar C1 test using LDS library
// Portenta H7 breakout board
//   Breakout.UART0 (PA_0/PI_9) → RPLidar C1 (460800)
//   Serial (USB CDC) → debug output
// ================================================

#include <Arduino_PortentaBreakout.h>
#include <LDS_RPLIDAR_C1.h>

LDS_RPLIDAR_C1 lidar;
#define LidarSerial Breakout.UART0

// 시리얼 읽기 콜백
int lidarRead() {
  return LidarSerial.read();
}

// 시리얼 쓰기 콜백
size_t lidarWrite(const uint8_t* buf, size_t len) {
  return LidarSerial.write(buf, len);
}

void scanCallback(float angle_deg, float distance_mm,
    float quality, bool startOfNewScan) {
  if (distance_mm > 0) {
    Serial.print("A:");
    Serial.print(angle_deg);
    Serial.print(" D:");
    Serial.print(distance_mm);
    Serial.print(" Q:");
    Serial.println(quality);
  }
}

void infoCallback(LDS::info_t code, String info) {
  Serial.print("[INFO] ");
  Serial.println(info);
}

void errorCallback(LDS::result_t code, String msg) {
  Serial.print("[ERROR] ");
  Serial.println(msg);
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 5000)) { delay(10); }
  Serial.println("=== RPLidar C1 LDS library test ===");

  // 콜백 등록
  lidar.setSerialReadCallback(lidarRead);
  lidar.setSerialWriteCallback(lidarWrite);
  lidar.setScanPointCallback(scanCallback);
  lidar.setInfoCallback(infoCallback);
  lidar.setErrorCallback(errorCallback);

  lidar.init();

  uint32_t baud = lidar.getSerialBaudRate();
  LidarSerial.begin(baud);
  Serial.print("LidarSerial baud: ");
  Serial.println(baud);

  LDS::result_t r = lidar.start();
  Serial.print("Start result: ");
  Serial.println(r);
}

void loop() {
  lidar.loop();
}
