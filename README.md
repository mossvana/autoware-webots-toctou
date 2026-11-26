# Experiment B: Autoware 안전 계층 환경

이 `experiment-b` 브랜치는 실험 A의 Webots–Autoware 폐루프 환경에 실제
Autoware 안전 패키지 6개를 추가한 **실험 B용 환경 구성 브랜치**입니다.

> 현재 완료 범위는 sparse checkout, 의존성 설치, 빌드, 설정 및 launch
> 연결입니다. 실험 B의 공격 실행과 차량 영향 측정은 아직 진행하지 않았습니다.

## 브랜치와 결과 구분

| 브랜치 | 용도 | 상태 |
| --- | --- | --- |
| `main` | 실험 A: 안전 계층이 없는 N-command 지연 공격 | 실행 및 결과 검증 완료 |
| `experiment-b` | 실험 B: 안전 계층을 포함한 구성 | 환경 구성 완료, 공격 실험 미실행 |

B는 A의 주행 환경을 확장하므로 이 브랜치에도 A 코드와 기준 CSV가 포함됩니다.
하지만 `results/reference`의 결과와 `run_a1.sh`, `run_a2.sh`는 모두
**실험 A 자료이며 실험 B 결과가 아닙니다.**

실험 B에서 확인할 질문은 다음과 같습니다.

```text
Validator·AEB·MRM·Mode Manager가 포함된 안전 구성에서도
지연된 과거 GO 명령이 실제 차량에 영향을 주는가?
```

추가된 실제 Autoware 패키지는 다음과 같습니다.

- `autoware_control_validator`
- `autoware_autonomous_emergency_braking`
- `autoware_operation_mode_transition_manager`
- `autoware_diagnostic_graph_aggregator`
- `autoware_mrm_handler`
- `autoware_mrm_emergency_stop_operator`

각 패키지는 수정한 모사 코드가 아니라 고정된 Autoware 원본 소스를 그대로
빌드합니다. 이 저장소는 Webots 연결, RC 차량 파라미터, diagnostic graph와
ROS 2 토픽 remap을 제공합니다.

## 1. 준비할 환경

검증된 기준 환경은 다음과 같습니다.

- Ubuntu 24.04 LTS, x86_64
- ROS 2 Jazzy (`/opt/ros/jazzy`)
- Webots R2025a (`webots`가 `PATH`에 있어야 함)
- Git, rosdep, colcon

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
git clone -b experiment-b \
  https://github.com/mossvana/autoware-webots-toctou.git
cd autoware-webots-toctou
```

이미 저장소를 받은 팀원은 다음처럼 전환합니다.

```bash
git fetch origin
git switch experiment-b
git pull --ff-only
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

## 실험 B 상태

현재 다음 항목까지 확인했습니다.

- 필요한 Autoware 소스만 sparse checkout
- 안전 패키지 6개와 의존 패키지 빌드
- 데모 단위 테스트 `2 tests, 0 failures`
- 안전 launch 파일의 설치와 인자 파싱
- 6개 패키지의 ROS 2 패키지 인덱스 등록

차량을 움직이지 않고 launch 구성을 확인하려면 다음을 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch webots_autoware_demo n_command_attack_safety.launch.py --show-args
```

아직 B-1/B-2 공격 실행, 안전 계층의 실제 개입 여부, 차량 재가속 거리와 같은
결과는 측정하지 않았습니다. 따라서 이 브랜치에는 B 결과표나 B 기준 CSV가
없습니다.

## 상속된 실험 A 자료

다음 파일은 A 환경을 기준선으로 보존하기 위해 남아 있습니다.

```text
scripts/run_a1.sh
scripts/run_a2.sh
scripts/run_experiment.sh
results/reference/a1_before_gate.csv
results/reference/a2_after_gate.csv
```

이 파일을 실행하면 안전 패키지 6개가 포함된 B가 아니라 기존 A launch가
실행됩니다. A의 `0.2501 m/s`, `0.2093 m`, `0.1703 m` 등의 수치를 B 결과로
인용하면 안 됩니다. B용 실행 절차와 결과 CSV는 실제 B 실험 이후 별도로
추가해야 합니다.

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
