#pragma once

// =============================================
// LiDAR source — uncomment exactly ONE
// =============================================
#define LIDAR_USE_SIM
// #define LIDAR_USE_RPLIDAR

// =============================================
// Scan parameters (shared by both sources)
// =============================================
#define SCAN_NUM_READINGS  360
#define SCAN_RANGE_MIN     0.15f
#define SCAN_RANGE_MAX     12.0f

// =============================================
// Source-specific settings
// =============================================
#ifdef LIDAR_USE_SIM
  #define SCAN_FREQ_HZ  10.0f
#endif

#ifdef LIDAR_USE_RPLIDAR
  #define SCAN_FREQ_HZ   5.5f
  #define RPLIDAR_BAUD   460800   // C1: 460800, A1: 115200, A2: 256000

  // Serial1 = hardware UART (production)
  // Serial  = USB CDC (PC bridge test)
  #define LIDAR_SERIAL   Serial

  // USB bridge mode: bridge handles RPLidar init,
  // firmware only parses. Comment out for direct UART (Serial1).
  #define LIDAR_USB_BRIDGE
#endif
