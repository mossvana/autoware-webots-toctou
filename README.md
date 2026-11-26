# Autoware-Webots TOCTOU experiment

Webots RC car와 실제 Autoware Planning, Control, Vehicle Command Gate를
연결하고, 정지 후 오래된 GO 명령 N개가 늦게 도착했을 때 다시 출발하는지
검증하는 재현용 저장소입니다. 실험 B용 안전 환경에는 아래 6개 실제
Autoware 패키지가 추가됩니다.

- `autoware_control_validator`
- `autoware_autonomous_emergency_braking`
- `autoware_operation_mode_transition_manager`
- `autoware_diagnostic_graph_aggregator`
- `autoware_mrm_handler`
- `autoware_mrm_emergency_stop_operator`

```text
Webots LiDAR / vehicle state
  -> Autoware Motion Velocity Planner (Obstacle Stop)
  -> Autoware Trajectory Follower (MPC + PID)
  -> Autoware Vehicle Command Gate
  -> Webots vehicle interface
  -> Webots Ackermann vehicle
```

두 공격 지점을 제공합니다.

- A-1 `Controller -> Gate`: Gate의 stale/out-of-order 처리 검증
- A-2 `Gate -> Vehicle Interface`: Vehicle Interface의 freshness 처리 검증

## 1. 준비할 환경

검증된 기준 환경은 다음과 같습니다.

- Ubuntu 24.04 LTS, x86_64
- ROS 2 Jazzy (`/opt/ros/jazzy`)
- Webots R2025a (`webots`가 `PATH`에 있어야 함)
- Git, vcstool, rosdep, colcon

> Ubuntu 24.02가 아니라 **Ubuntu 24.04 LTS**입니다.

필수 도구:

```bash
sudo apt update
sudo apt install -y \
  git python3-rosdep python3-colcon-common-extensions

# 이 PC에서 rosdep을 처음 사용하는 경우에만 실행
sudo rosdep init
```

Webots R2025a와 ROS 2 Jazzy 설치를 마친 뒤 환경을 확인합니다.

```bash
./scripts/check_environment.sh
```

## 2. 저장소 받기

```bash
git clone https://github.com/mossvana/autoware-webots-toctou.git
cd autoware-webots-toctou
```

## 3. 동일한 Autoware 소스 준비

`dependencies/autoware.repos`에는 최초 실험에 사용한 정확한 Git 커밋이
고정되어 있습니다. 설치 스크립트는 Autoware 전체를 받지 않습니다. 기존
Planning/Control/Gate, 위 안전 패키지 6개, 그리고 이들의 소스 의존성만
Git sparse checkout으로 받습니다.

```bash
./scripts/setup_workspace.sh
```

이 명령은 다음 작업을 수행합니다.

1. 고정된 Autoware Core/Universe/메시지 저장소에서 필요한 경로만 가져옴
2. `rosdep`으로 나머지 시스템 의존성 설치

## 4. 빌드와 단위 테스트

```bash
./scripts/build.sh
```

전체 Autoware가 아니라 이 실험 패키지까지 필요한 의존 패키지만 빌드합니다.
기준 단위 테스트 결과는 `2 tests, 0 failures`입니다.

빌드 마지막에는 6개 안전 패키지의 설치 여부와 고정 커밋 및 sparse 경로를
자동 검사합니다. 이미 빌드한 환경만 다시 검사하려면 다음을 실행합니다.

```bash
./scripts/verify_safety_environment.sh
```

## 실험 B 안전 파이프라인

환경에는 다음 연결 설정도 포함되어 있습니다.

```text
Control Validator + AEB
  -> Diagnostic Graph
  -> Operation Mode Availability
  -> MRM Handler
  -> MRM Emergency Stop Operator
  -> Vehicle Command Gate emergency input
  -> Webots vehicle interface
```

관련 launch 파일은
`ros2_ws/src/webots_autoware_demo/launch/n_command_attack_safety.launch.py`입니다.
이번 구성 작업은 패키지 설치·빌드·기동 연결까지를 범위로 하며, 실험 B의
공격 실행과 결과 판정은 포함하지 않습니다.

## 5. A-1 실행

다른 Webots와 ROS 실험을 모두 종료한 뒤 실행합니다.

```bash
./scripts/run_a1.sh
```

결과:

```text
results/generated/a1_before_gate.csv
logs/webots_before_gate.log
logs/ros_before_gate.log
```

## 6. A-2 실행

A-1이 완전히 종료된 뒤 실행합니다.

```bash
./scripts/run_a2.sh
```

결과:

```text
results/generated/a2_after_gate.csv
logs/webots_after_gate.log
logs/ros_after_gate.log
```

스크립트는 실험마다 Webots를 새로 시작하고, `/clock`을 기다린 다음 ROS 2
launch를 실행합니다. 실험 종료 시 Webots와 ROS 프로세스도 정리합니다.

## 7. 재현 성공 판정

실행 스크립트가 자동으로 다음 조건을 검사합니다.

```text
released_count == 30
min_released_age_ms > 100
out_of_order_count >= 1
stale_accepted_by_recipient == True
physical_reacceleration == True
max_forward_distance_m > 0.01
attack_success == True
```

최초 검증 결과:

| 실험 | 명령 나이 | 순서 역전 | 정지 후 최고속도 | 추가 전진 | 결과 |
| --- | ---: | ---: | ---: | ---: | --- |
| A-1 | 3744 ms | 1 | 0.2501 m/s | 0.2093 m | reproduced |
| A-2 | 3744 ms | 1 | 0.2501 m/s | 0.1703 m | reproduced |

기준 CSV는 `results/reference`에 있습니다. ROS 스케줄링과 PC 성능 때문에
이동거리는 약간 달라질 수 있습니다. 표의 숫자와 완전히 같아야 하는 것이
아니라 자동 판정 조건을 모두 통과하면 재현 성공입니다.

## 재현 범위

이 저장소는 다음을 고정합니다.

- 실험 코드와 설정
- Webots 월드와 차량 모델
- Autoware 저장소별 정확한 Git 커밋
- 공격 개수와 재생 주기
- 결과 판정 기준

Native 설치 방식이므로 Ubuntu의 모든 `.deb` 패키지를 바이트 단위로
동일하게 고정하지는 않습니다. OS 패키지까지 완전히 동일한 이미지가 필요한
경우 Docker 이미지를 별도로 제공해야 합니다. 자세한 기준은
[`ENVIRONMENT.md`](ENVIRONMENT.md)를 참고하세요.

## License

Apache License 2.0. Autoware와 Webots에는 각각의 라이선스가 적용됩니다.
