#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 || ( "$1" != "before_gate" && "$1" != "after_gate" ) ]]; then
  echo "Usage: $0 before_gate|after_gate" >&2
  exit 2
fi

attack_point="$1"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
world="$repo_root/webots/worlds/rc_car_obstacle.wbt"
result_dir="$repo_root/results/generated"
log_dir="$repo_root/logs"
mkdir -p "$result_dir" "$log_dir"

if [[ "$attack_point" == "before_gate" ]]; then
  result_csv="$result_dir/a1_before_gate.csv"
else
  result_csv="$result_dir/a2_after_gate.csv"
fi
rm -f "$result_csv"

"$repo_root/scripts/check_environment.sh"
set +u
source /opt/ros/jazzy/setup.bash
if [[ ! -r "$repo_root/ros2_ws/install/setup.bash" ]]; then
  echo "ERROR: workspace is not built; run scripts/build.sh" >&2
  exit 1
fi
source "$repo_root/ros2_ws/install/setup.bash"
set -u

webots_pid=""
launch_pid=""
cleanup() {
  set +e
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT -- "-$launch_pid" 2>/dev/null
    sleep 2
    kill -TERM -- "-$launch_pid" 2>/dev/null
  fi
  if [[ -n "$webots_pid" ]] && kill -0 "$webots_pid" 2>/dev/null; then
    kill -INT -- "-$webots_pid" 2>/dev/null
    sleep 2
    kill -TERM -- "-$webots_pid" 2>/dev/null
  fi
}
trap cleanup EXIT INT TERM

setsid webots --batch --no-rendering --mode=realtime --stdout --stderr "$world" \
  >"$log_dir/webots_${attack_point}.log" 2>&1 &
webots_pid=$!

echo "Waiting for Webots /clock..."
clock_deadline=$((SECONDS + 60))
while ! timeout 2s ros2 topic echo /clock --once >/dev/null 2>&1; do
  if ! kill -0 "$webots_pid" 2>/dev/null; then
    echo "ERROR: Webots exited before publishing /clock; see $log_dir/webots_${attack_point}.log" >&2
    exit 1
  fi
  if (( SECONDS >= clock_deadline )); then
    echo "ERROR: Webots did not publish /clock; see $log_dir/webots_${attack_point}.log" >&2
    exit 1
  fi
  sleep 1
done

setsid ros2 launch webots_autoware_demo n_command_attack.launch.py \
  attack_point:="$attack_point" \
  hold_count:=30 \
  replay_period:=0.03 \
  result_csv:="$result_csv" \
  >"$log_dir/ros_${attack_point}.log" 2>&1 &
launch_pid=$!

echo "Running $attack_point experiment..."
deadline=$((SECONDS + 90))
while [[ ! -s "$result_csv" ]]; do
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    echo "ERROR: ROS launch exited early; see $log_dir/ros_${attack_point}.log" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "ERROR: experiment timed out; see logs in $log_dir" >&2
    exit 1
  fi
  sleep 1
done

python3 "$repo_root/scripts/verify_results.py" \
  "$result_csv" --attack-point "$attack_point" --hold-count 30
echo "Result: $result_csv"
