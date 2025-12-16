# OnRobot RG2 Gripper RViz Sync

OnRobot RG2 그리퍼의 실제 상태를 Modbus TCP로 읽어 RViz에서 실시간 시각화하는 ROS2 패키지

## 📋 개요

```
┌─────────────────┐    Modbus TCP     ┌──────────────────┐
│  ROS2 Node      │◄─────────────────►│  OnRobot RG2     │
│  (pymodbus)     │  192.168.1.1:502  │  (Compute Box)   │
└────────┬────────┘                   └──────────────────┘
         │ 
         │ /gripper_joint_states
         ▼
┌─────────────────┐
│   RViz2         │
│   (URDF 시각화)  │
└─────────────────┘
```

## 🔧 하드웨어 요구사항

- OnRobot RG2 그리퍼
- OnRobot Compute Box (IP: 192.168.1.1)
- PC와 Compute Box가 같은 네트워크에 연결

## 📦 설치

```bash
# 의존성 설치
pip3 install pymodbus

# 패키지 빌드
cd ~/ros2_ws
colcon build --packages-select gripper_rviz_sync
source install/setup.bash
```

## 🚀 사용법

### 1. 연결 테스트
```bash
# Modbus 연결 테스트
python3 ~/ros2_ws/src/gripper_rviz_sync/test_gripper.py 1

# 그리퍼 이동 테스트
python3 ~/ros2_ws/src/gripper_rviz_sync/test_gripper.py 2

# ROS2 토픽 발행 테스트
python3 ~/ros2_ws/src/gripper_rviz_sync/test_gripper.py 3
```

### 2. 전체 시스템 실행
```bash
# 그리퍼 상태 발행자 + 컨트롤러 실행
ros2 launch gripper_rviz_sync gripper_sync.launch.py
```

### 3. 개별 노드 실행
```bash
# 그리퍼 상태 발행자만 실행 (RViz 동기화)
ros2 run gripper_rviz_sync gripper_state_publisher.py

# 그리퍼 컨트롤러만 실행
ros2 run gripper_rviz_sync gripper_controller.py
```

## 📡 ROS2 토픽 & 서비스

### 발행 토픽
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/gripper_joint_states` | `sensor_msgs/JointState` | RViz 시각화용 조인트 상태 |
| `/gripper/current_width` | `std_msgs/Float32` | 현재 그리퍼 폭 (mm) |
| `/gripper/grip_detected` | `std_msgs/Bool` | 물체 감지 여부 |

### 구독 토픽
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/gripper/command/width` | `std_msgs/Float32` | 목표 폭 명령 (mm) |
| `/gripper/command/force` | `std_msgs/Float32` | 목표 힘 설정 (N) |

### 서비스
| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/gripper/open` | `std_srvs/Trigger` | 그리퍼 열기 (80mm) |
| `/gripper/close` | `std_srvs/Trigger` | 그리퍼 닫기 (0mm) |

## 🎮 그리퍼 제어 예시

```bash
# 그리퍼 열기
ros2 service call /gripper/open std_srvs/srv/Trigger

# 그리퍼 닫기
ros2 service call /gripper/close std_srvs/srv/Trigger

# 특정 폭으로 이동 (50mm)
ros2 topic pub --once /gripper/command/width std_msgs/msg/Float32 "data: 50.0"

# 힘 설정 (25N)
ros2 topic pub --once /gripper/command/force std_msgs/msg/Float32 "data: 25.0"
```

## 📊 Modbus 레지스터 맵

### 쓰기 레지스터 (Unit ID: 65)
| 주소 | 기능 | 범위 |
|------|------|------|
| 0 | Target Force | 0-400 (0.1N 단위, 0-40N) |
| 1 | Target Width | 0-1100 (0.1mm 단위, 0-110mm) |
| 2 | Control | 1 = Grip 명령 |

### 읽기 레지스터 (Unit ID: 65)
| 주소 | 기능 | 단위 |
|------|------|------|
| 267 | Current Width | 0.1mm |
| 268 | Grip Detected | 0/1 |
| 258 | Device Type | - |

## 📁 파일 구조

```
gripper_rviz_sync/
├── package.xml
├── CMakeLists.txt
├── README.md
├── test_gripper.py
├── config/
│   └── gripper_config.yaml
├── gripper_rviz_sync/
│   ├── __init__.py
│   ├── gripper_state_publisher.py  # 상태 발행 노드
│   └── gripper_controller.py       # 제어 노드
├── launch/
│   └── gripper_sync.launch.py
└── urdf/
    └── onrobot_rg2_movable.urdf.xacro  # RViz용 움직이는 URDF
```

## ⚠️ 주의사항

1. **네트워크 연결**: PC와 Compute Box가 같은 서브넷에 있어야 합니다
   - PC IP: `192.168.1.x`
   - Compute Box IP: `192.168.1.1`

2. **URDF 수정**: RViz에서 그리퍼가 움직이려면 조인트 타입이 `revolute`여야 합니다
   - 기존 URDF: `type="fixed"` → 움직이지 않음
   - 새 URDF: `type="revolute"` → 움직임 가능

3. **Joint States 결합**: 로봇 조인트와 그리퍼 조인트를 함께 시각화하려면 
   `joint_state_publisher`에서 두 토픽을 결합해야 합니다

## 🔗 참고 자료

- [OnRobot RG2 매뉴얼](https://onrobot.com/products/rg2)
- [pymodbus 문서](https://pymodbus.readthedocs.io/)
- [ROS2 JointState](http://docs.ros.org/en/humble/p/sensor_msgs/interfaces/msg/JointState.html)
