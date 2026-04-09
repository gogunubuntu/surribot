#include "config.h"
#ifdef LIDAR_USE_RPLIDAR

#include "lidar.h"
#include <string.h>

// RPLidar protocol constants
#define RP_SYNC       0xA5
#define RP_CMD_STOP   0x25
#define RP_CMD_SCAN   0x20
#define RP_CMD_PWM    0xF0
#define RP_SYNC2      0x5A
#define RP_DESC_LEN   7
#define RP_MOTOR_PWM  660

enum RpState { RP_WAIT_SYNC1, RP_WAIT_SYNC2, RP_WAIT_DESC, RP_SCANNING };

static RpState    state    = RP_WAIT_SYNC1;
static int        desc_cnt = 0;
static uint8_t    pkt[5];
static int        pkt_idx  = 0;

// Double-buffered scan
static float buf_a[SCAN_NUM_READINGS];
static float buf_b[SCAN_NUM_READINGS];
static float* filling   = buf_a;
static float* ready_buf = buf_b;
static bool   has_data   = false;
static volatile bool scan_ready = false;

static void process_point() {
  bool S  =  pkt[0] & 0x01;
  bool nS = (pkt[0] >> 1) & 0x01;
  if (S == nS) return;            // sync error — skip
  if (!(pkt[1] & 0x01)) return;   // check bit must be 1

  // New revolution → swap buffers
  if (S && has_data) {
    float* tmp = ready_buf;
    ready_buf  = filling;
    filling    = tmp;
    scan_ready = true;
    memset(filling, 0, SCAN_NUM_READINGS * sizeof(float));
    has_data = false;
  }

  uint16_t angle_q6 = (pkt[1] >> 1) | ((uint16_t)pkt[2] << 7);
  uint16_t dist_q2  =  pkt[3]       | ((uint16_t)pkt[4] << 8);
  if (dist_q2 == 0) return;       // invalid measurement

  float angle_deg = angle_q6 / 64.0f;
  float dist_m    = (dist_q2 / 4.0f) / 1000.0f;

  int bin = (int)(angle_deg * SCAN_NUM_READINGS / 360.0f);
  if (bin >= 0 && bin < SCAN_NUM_READINGS &&
      dist_m >= SCAN_RANGE_MIN && dist_m <= SCAN_RANGE_MAX) {
    filling[bin] = dist_m;
    has_data = true;
  }
}

void lidar_init() {
  memset(buf_a, 0, sizeof(buf_a));
  memset(buf_b, 0, sizeof(buf_b));
  state   = RP_WAIT_SYNC1;
  pkt_idx = 0;

#ifdef LIDAR_USB_BRIDGE
  // Bridge handles RPLidar init (motor + scan start).
  // Firmware only parses incoming data.
#else
  LIDAR_SERIAL.begin(RPLIDAR_BAUD);

  const uint8_t stop[] = {RP_SYNC, RP_CMD_STOP};
  LIDAR_SERIAL.write(stop, 2);
  delay(100);
  while (LIDAR_SERIAL.available()) LIDAR_SERIAL.read();

  uint8_t pwm_lo = RP_MOTOR_PWM & 0xFF;
  uint8_t pwm_hi = (RP_MOTOR_PWM >> 8) & 0xFF;
  uint8_t pwm_cmd[] = {RP_SYNC, RP_CMD_PWM, 0x02, pwm_lo, pwm_hi,
                        (uint8_t)(0 ^ RP_SYNC ^ RP_CMD_PWM ^ 0x02 ^ pwm_lo ^ pwm_hi)};
  LIDAR_SERIAL.write(pwm_cmd, sizeof(pwm_cmd));
  delay(1000);

  const uint8_t start[] = {RP_SYNC, RP_CMD_SCAN};
  LIDAR_SERIAL.write(start, 2);
#endif
}

void lidar_update() {
  while (LIDAR_SERIAL.available()) {
    uint8_t b = LIDAR_SERIAL.read();

    switch (state) {
    case RP_WAIT_SYNC1:
      if (b == RP_SYNC) state = RP_WAIT_SYNC2;
      break;

    case RP_WAIT_SYNC2:
      if (b == RP_SYNC2) { desc_cnt = 2; state = RP_WAIT_DESC; }
      else               state = RP_WAIT_SYNC1;
      break;

    case RP_WAIT_DESC:
      if (++desc_cnt >= RP_DESC_LEN) { state = RP_SCANNING; pkt_idx = 0; }
      break;

    case RP_SCANNING:
      pkt[pkt_idx++] = b;
      if (pkt_idx >= 5) { process_point(); pkt_idx = 0; }
      break;
    }
  }
}

bool lidar_get_scan(float* ranges, int num_slots) {
  if (!scan_ready) return false;
  scan_ready = false;
  int n = (num_slots < SCAN_NUM_READINGS) ? num_slots : SCAN_NUM_READINGS;
  memcpy(ranges, ready_buf, n * sizeof(float));
  return true;
}

#endif
