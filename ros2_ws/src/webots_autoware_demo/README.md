# Webots LiDAR to Autoware Obstacle Stop

This demo keeps a straight `Trajectory` as the reference path and runs the
real Autoware planning/control path around it:

```text
Webots odometry + acceleration + LiDAR PointCloud2
  -> autoware_motion_velocity_planner / ObstacleStopModule
  -> autoware_trajectory_follower_node (MPC + PID)
  -> autoware_vehicle_cmd_gate
  -> Webots Ackermann vehicle
```

The observation-only `trajectory_probe` compares the first zero-velocity
point in `/planning/trajectory_raw` and `/planning/trajectory`.  A successful
detection prints `[OBSTACLE STOP] active=True` and a positive `shift`.

## Run

From the repository root, terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
webots --mode=realtime \
  webots/worlds/rc_car_obstacle.wbt
```

Terminal 2:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch webots_autoware_demo webots_obstacle_stop_closed_loop.launch.py
```

The current test world places the obstacle at `x=1.05 m`.  In the verified
run, the raw stop at `x=1.50 m` moved to `x=0.60 m`, and the vehicle stopped
with its base near `x=0.534 m`.

The files under `config/` are scaled for the 0.22-m-wheelbase RC vehicle.
They are simulation parameters, not values for a full-size vehicle.

## N-command delay experiment

`n_command_attack.launch.py` adds three experiment-only components without
replacing the Autoware planner, controller, or gate:

1. `n_command_delay`: captures N positive `Control` messages, allows the
   vehicle to reach the real LiDAR-generated stop, then replays the old batch.
2. Automatic release trigger: replay starts only after Obstacle Stop moved the
   planned stop point, the vehicle moved, and measured speed stayed below
   `0.02 m/s` for `0.5 s`.
3. `n_command_evaluator`: independently records timestamp age/order, downstream
   acceptance, measured speed, and physical forward movement to a CSV file.

Controller to Gate (A-1):

```bash
ros2 launch webots_autoware_demo n_command_attack.launch.py \
  attack_point:=before_gate hold_count:=30 \
  result_csv:=$HOME/autoware_webots_ws/results/a1_before_gate.csv
```

Gate to Vehicle Interface (A-2):

```bash
ros2 launch webots_autoware_demo n_command_attack.launch.py \
  attack_point:=after_gate hold_count:=30 \
  result_csv:=$HOME/autoware_webots_ws/results/a2_after_gate.csv
```

### Verified result (30 commands, 30 ms replay interval)

| Case | Recipient accepted | Released age | Order reversals | Peak speed after stop | Extra forward travel | Result |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| A-1 Controller -> Gate | 37 positive Gate outputs | 3744 ms | 1 | 0.2501 m/s | 0.2093 m | attack reproduced |
| A-2 Gate -> Vehicle Interface | 30 positive drive outputs | 3744 ms | 1 | 0.2501 m/s | 0.1703 m | attack reproduced |

The Gate output count can exceed 30 because the Gate publishes at its own
rate.  It does not mean that more than 30 stale inputs were injected.  In both
cases exactly 30 old commands were released.  One timestamp reversal means
that an old GO batch arrived after newer STOP commands.

These results show the behavior of this launch configuration: neither the
Gate boundary nor the demo Vehicle Interface rejected the tested commands due
to their age or timestamp order.  They do not prove that every Autoware vehicle
interface or safety configuration behaves the same way.
