# Verified environment

The reference results were produced with:

- Ubuntu 24.04.5 LTS, x86_64
- ROS 2 Jazzy
- Webots R2025a
- Autoware repositories pinned in `dependencies/autoware.repos`
- Webots world `webots/worlds/rc_car_obstacle.wbt`
- `hold_count=30`, `replay_period=0.03`

The source code and Autoware revisions are reproducible. A native installation
does not make every operating-system package byte-identical. Small differences
in ROS scheduling and physical travel distance are therefore expected. The
boolean criteria checked by `scripts/verify_results.py` define reproduction.
