# SurriBot

MCU(Portenta H7)만으로 동작하는 무선 ROS2 모바일 로봇 플랫폼.
SBC 없이 micro-ROS WiFi로 `/scan` 토픽을 publish하고, 노트북에서 SLAM/Nav2를 처리한다.

## 시스템 구성

```
[Portenta H7]                    WiFi (2.4GHz UDP)           [노트북]
  micro-ROS  ──────────────────────────────────────>  micro-ROS agent
  /scan (LaserScan, 460pts, 10Hz)                     ROS2 Jazzy
                                                      rviz2 / slam_toolbox
```

> Portenta H7 WiFi 모듈(Murata 1DX / CYW4343W)은 **2.4GHz만 지원**한다. 5GHz 불가.

## 사전 준비

### 노트북

- Ubuntu 24.04
- ROS2 Jazzy (`/opt/ros/jazzy/`)
- arduino-cli (`~/.local/bin/arduino-cli`)
- micro-ROS agent (`~/.microros_agent_ws/`)

### 하드웨어

- Arduino Portenta H7 (USB 연결)
- WiFi 안테나 장착

### 설치 (최초 1회)

```bash
# arduino-cli
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=$HOME/.local/bin sh

# Portenta 보드 코어
arduino-cli core update-index
arduino-cli core install arduino:mbed_portenta

# micro_ros_arduino 라이브러리 (Jazzy)
arduino-cli config set library.enable_unsafe_install true
curl -L -o /tmp/micro_ros_arduino.zip \
  "https://github.com/micro-ROS/micro_ros_arduino/archive/refs/tags/v2.0.8-jazzy.zip"
arduino-cli lib install --zip-path /tmp/micro_ros_arduino.zip

# USB 권한 (udev)
sudo sh -c 'echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"2341\", MODE=\"0666\"" \
  > /etc/udev/rules.d/99-arduino.rules && udevadm control --reload-rules && udevadm trigger'
sudo usermod -aG dialout $USER

# micro-ROS agent 빌드
mkdir -p ~/microros_agent_build/src && cd ~/microros_agent_build/src
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git
cd .. && source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
# 빌드 결과물을 ~/.microros_agent_ws/로 이동
mv ~/microros_agent_build/install ~/.microros_agent_ws/install
mv ~/microros_agent_build/build ~/.microros_agent_ws/build
```

## 펌웨어 설정

`firmware/m7_main/wifi_config.h.example`을 복사하여 `wifi_config.h`를 만들고 자신의 환경에 맞게 수정:

```bash
cp firmware/m7_main/wifi_config.h.example firmware/m7_main/wifi_config.h
# wifi_config.h를 편집하여 WiFi SSID/비밀번호, Agent IP 입력
```

노트북 IP 확인:
```bash
ip -4 addr show | grep "inet " | grep -v "127.0.0.1"
```

## 빌드 & 업로드

```bash
# 1. 컴파일
arduino-cli compile --fqbn arduino:mbed_portenta:envie_m7 firmware/micro_ros_pub/

# 2. Portenta 리셋 버튼 더블탭 (부트로더 모드 진입, 초록 LED 페이딩)

# 3. 업로드
arduino-cli upload --fqbn arduino:mbed_portenta:envie_m7 -p /dev/ttyACM0 firmware/micro_ros_pub/
```

## 실행

### 1. micro-ROS agent 실행

```bash
microros-agent udp4 --port 8888
```

> alias가 없으면:
> ```bash
> source /opt/ros/jazzy/setup.bash
> source ~/.microros_agent_ws/install/setup.bash
> ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
> ```

### 2. Portenta 리셋 (1번 단일 클릭)

agent가 실행 중인 상태에서 리셋해야 한다. 순서 중요:
1. agent 먼저 실행
2. 그 다음 Portenta 리셋

LED 상태:
- **켜짐 (고정)**: WiFi 연결 중
- **꺼짐**: 정상 동작 (setup 완료, publish 중)
- **빠르게 깜빡임**: 에러 (WiFi 실패 또는 agent 미연결)

### 3. 토픽 확인

```bash
source /opt/ros/jazzy/setup.bash

# 토픽 리스트
ros2 topic list

# 주파수 확인
ros2 topic hz /scan

# 데이터 확인
ros2 topic echo /scan --once
```

### 4. rviz2로 시각화

터미널 1 — TF 발행:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map laser
```

터미널 2 — rviz2:
```bash
source /opt/ros/jazzy/setup.bash
rviz2
```

rviz2 설정:
1. 좌측 하단 **Fixed Frame** → `laser`
2. **Add** → **By topic** → `/scan` → **LaserScan**
3. LaserScan의 **Reliability Policy** → `Reliable`

## 현재 토픽

| 토픽 | 타입 | QoS | 주기 | 설명 |
|------|------|-----|------|------|
| `/scan` | `sensor_msgs/msg/LaserScan` | reliable | 10Hz | RPLIDAR C1 시뮬레이션 (460pts, 가상 방 4x3m + 원형 장애물) |

## 기술 노트

### QoS: reliable vs best_effort

- **best_effort**: 패킷 fragmentation 불가. MTU(512B) 안에 메시지가 들어가야 함 → ~100 readings 한계
- **reliable**: fragmentation 지원. 출력 버퍼(2048B) 범위에서 분할 전송 → 460+ readings 가능

RPLIDAR C1 수준의 scan 데이터는 반드시 **reliable QoS**를 사용해야 한다.

### WiFi 제약

- Portenta H7 WiFi: **2.4GHz only** (Murata 1DX / CYW4343W)
- 이미지 토픽은 micro-ROS 버퍼(2048B) 한계로 전송 불가. 카메라는 스마트폰 IP 카메라 사용 권장

### micro-ROS 버퍼 설정 (참고)

`~/Arduino/libraries/micro_ros_arduino/src/uxr/client/config.h`:
```
UXR_CONFIG_CUSTOM_TRANSPORT_MTU = 512
```

`~/Arduino/libraries/micro_ros_arduino/src/rmw_microxrcedds_c/config.h`:
```
RMW_UXRCE_STREAM_HISTORY_OUTPUT = 4
RMW_UXRCE_MAX_OUTPUT_BUFFER_SIZE = 512 * 4 = 2048
```

## 프로젝트 구조

```
surribot/
├── README.md
└── firmware/
    └── micro_ros_pub/
        └── micro_ros_pub.ino    # Portenta H7 펌웨어
```
