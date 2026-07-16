# ROS 2 AutoRace State Machine Extension

This repository contains a customized version of the public TurtleBot3 AutoRace codebase for **ROS 2 Humble**, extended with a state-machine-based mission controller and multiple improvements across detection and mission nodes.

The main goal of this work was to improve autonomy, coordination between nodes, and overall mission reliability during the AutoRace challenge. In addition to the state machine implementation, several nodes were modified to improve sign detection, mission transitions, and communication flow between perception and control components.

## What was added

- State machine logic for mission sequencing.
- New core mission orchestration files.
- Adjustments in multiple mission and detection nodes.
- Improvements to communication between nodes for more consistent behavior.
- Launch files for running the integrated solution.

## Main modifications

The project includes changes in areas such as:

- Mission control and sequencing.
- Tunnel and construction mission behavior.
- Sign detection and traffic-light-related logic.
- Launch configuration for the complete AutoRace flow.
- Parameter tuning and calibration updates.

## Project structure

Some of the most relevant added or modified files include:

```bash
turtlebot3_autorace_mission/
├── launch/
│   ├── autorace_core.launch.py
│   └── sim_environment.launch.py
└── turtlebot3_autorace_mission/
    ├── mission_core.py
    ├── states.py
    ├── turn_logic.py
    ├── mission_tunnel.py
    └── avoid_construction.py
```

## Notes

- This repository is based on the public TurtleBot3 AutoRace project and should be understood as a modified implementation built on top of that original work.
- Some files were adapted to fit the challenge requirements and improve practical performance during testing.
- It is recommended to remove generated Python cache files (`__pycache__/`) from version control before publishing the final version of the repository.

## Suggested cleanup before publishing

Add a `.gitignore` file if you do not already have one:

```gitignore
__pycache__/
*.pyc
*.pyo
*.log
build/
install/
log/
```

Then remove cached generated files from Git tracking:

```bash
find . -type d -name "__pycache__" -exec git rm -r --cached {} +
git add .gitignore
git commit -m "Remove cache files and update gitignore"
```

## Acknowledgment

This work is a modification of the public TurtleBot3 AutoRace codebase for ROS 2 Humble, extended and adapted for challenge use with additional mission logic and node-level improvements.