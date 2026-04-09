// ================================================
// RPLidar C1 direct UART test (no micro-ROS, no WiFi)
// Portenta H7 breakout board — UART0 (Serial1)
//
// Serial1 = RPLidar data (UART0, 460800 baud)
// Serial  = USB CDC debug output
// ================================================

#define RPLIDAR_BAUD   460800
#define NUM_BINS       360
#define RANGE_MIN      0.15f
#define RANGE_MAX      12.0f

#define LIDAR_SERIAL   Serial1
#define DEBUG_SERIAL   Serial

// RPLidar protocol
#define RP_SYNC     0xA5
#define RP_CMD_STOP 0x25
#define RP_CMD_SCAN 0x20
#define RP_CMD_PWM  0xF0
#define RP_SYNC2    0x5A
#define RP_MOTOR_PWM 660

enum State { WAIT_SYNC1, WAIT_SYNC2, WAIT_DESC, SCANNING };

static State    state   = WAIT_SYNC1;
static int      desc_cnt = 0;
static uint8_t  pkt[5];
static int      pkt_idx = 0;

static float bins[NUM_BINS];
static int   point_count = 0;
static int   rev_count   = 0;
static bool  has_data    = false;

void process_point() {
  bool S  =  pkt[0] & 0x01;
  bool nS = (pkt[0] >> 1) & 0x01;
  if (S == nS) return;
  if (!(pkt[1] & 0x01)) return;

  if (S && has_data) {
    // Revolution complete — print summary
    rev_count++;
    int valid = 0;
    float sum = 0;
    float mn = RANGE_MAX, mx = 0;
    for (int i = 0; i < NUM_BINS; i++) {
      if (bins[i] > 0) {
        valid++;
        sum += bins[i];
        if (bins[i] < mn) mn = bins[i];
        if (bins[i] > mx) mx = bins[i];
      }
    }
    DEBUG_SERIAL.print("[rev ");
    DEBUG_SERIAL.print(rev_count);
    DEBUG_SERIAL.print("] points=");
    DEBUG_SERIAL.print(point_count);
    DEBUG_SERIAL.print(" valid_bins=");
    DEBUG_SERIAL.print(valid);
    DEBUG_SERIAL.print("/");
    DEBUG_SERIAL.print(NUM_BINS);
    if (valid > 0) {
      DEBUG_SERIAL.print(" min=");
      DEBUG_SERIAL.print(mn, 3);
      DEBUG_SERIAL.print("m max=");
      DEBUG_SERIAL.print(mx, 3);
      DEBUG_SERIAL.print("m avg=");
      DEBUG_SERIAL.print(sum / valid, 3);
      DEBUG_SERIAL.print("m");
    }
    DEBUG_SERIAL.println();

    memset(bins, 0, sizeof(bins));
    point_count = 0;
    has_data = false;
  }

  uint16_t angle_q6 = (pkt[1] >> 1) | ((uint16_t)pkt[2] << 7);
  uint16_t dist_q2  =  pkt[3]       | ((uint16_t)pkt[4] << 8);
  if (dist_q2 == 0) return;

  float angle_deg = angle_q6 / 64.0f;
  float dist_m    = (dist_q2 / 4.0f) / 1000.0f;

  int bin = (int)(angle_deg * NUM_BINS / 360.0f);
  if (bin >= 0 && bin < NUM_BINS && dist_m >= RANGE_MIN && dist_m <= RANGE_MAX) {
    bins[bin] = dist_m;
    point_count++;
    has_data = true;
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  DEBUG_SERIAL.begin(115200);
  // USB CDC 연결 대기 (최대 5초)
  unsigned long t0 = millis();
  while (!DEBUG_SERIAL && (millis() - t0 < 5000)) { delay(10); }
  delay(500);
  DEBUG_SERIAL.println("=== RPLidar C1 direct UART test ===");
  DEBUG_SERIAL.println("Initializing Serial1 (UART0)...");

  // RPLidar UART 초기화
  LIDAR_SERIAL.begin(RPLIDAR_BAUD);
  delay(100);
  DEBUG_SERIAL.println("Serial1 OK");

  // 1) Stop (기존 상태 초기화)
  const uint8_t stop[] = {RP_SYNC, RP_CMD_STOP};
  LIDAR_SERIAL.write(stop, 2);
  delay(100);
  while (LIDAR_SERIAL.available()) LIDAR_SERIAL.read();  // flush
  DEBUG_SERIAL.println("STOP sent");

  // 2) Motor PWM 설정 (SET_PWM command with payload)
  uint8_t pwm_lo = RP_MOTOR_PWM & 0xFF;
  uint8_t pwm_hi = (RP_MOTOR_PWM >> 8) & 0xFF;
  uint8_t chk = 0 ^ RP_SYNC ^ RP_CMD_PWM ^ 0x02 ^ pwm_lo ^ pwm_hi;
  uint8_t pwm_cmd[] = {RP_SYNC, RP_CMD_PWM, 0x02, pwm_lo, pwm_hi, chk};
  LIDAR_SERIAL.write(pwm_cmd, sizeof(pwm_cmd));
  DEBUG_SERIAL.print("Motor PWM=");
  DEBUG_SERIAL.println(RP_MOTOR_PWM);
  delay(2000);  // 모터 안정화 대기
  DEBUG_SERIAL.println("Motor spin-up done");

  // 3) Start scan
  const uint8_t start[] = {RP_SYNC, RP_CMD_SCAN};
  LIDAR_SERIAL.write(start, 2);
  DEBUG_SERIAL.println("SCAN started, waiting for data...");

  memset(bins, 0, sizeof(bins));
  state   = WAIT_SYNC1;
  pkt_idx = 0;

  digitalWrite(LED_BUILTIN, LOW);
}

static unsigned long last_hb = 0;
static unsigned long byte_count = 0;

void loop() {
  // Heartbeat every 3 seconds
  if (millis() - last_hb > 3000) {
    last_hb = millis();
    DEBUG_SERIAL.print("[hb] bytes_rx=");
    DEBUG_SERIAL.print(byte_count);
    DEBUG_SERIAL.print(" state=");
    DEBUG_SERIAL.println(state);
  }

  while (LIDAR_SERIAL.available()) {
    uint8_t b = LIDAR_SERIAL.read();
    byte_count++;

    switch (state) {
    case WAIT_SYNC1:
      if (b == RP_SYNC) state = WAIT_SYNC2;
      break;
    case WAIT_SYNC2:
      if (b == RP_SYNC2) { desc_cnt = 2; state = WAIT_DESC; }
      else               state = WAIT_SYNC1;
      break;
    case WAIT_DESC:
      if (++desc_cnt >= 7) { state = SCANNING; pkt_idx = 0; }
      break;
    case SCANNING:
      pkt[pkt_idx++] = b;
      if (pkt_idx >= 5) { process_point(); pkt_idx = 0; }
      break;
    }
  }
}
