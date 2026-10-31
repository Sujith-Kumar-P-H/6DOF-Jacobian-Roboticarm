# 6-DOF Robotic Order-Picking System with Jacobian-Based Closed-Loop Control

A pure-Python simulation of a 6-axis robot arm that retrieves items from two storage
shelves with a suction gripper and places them into an order tote. The arm starts at
HOME and returns to HOME when the mission is done.

**What it demonstrates**
- **Kinematics:** a data-driven DH table (standard and modified conventions), forward
  kinematics, geometric Jacobian, damped-least-squares inverse kinematics, and
  singularity monitoring through manipulability and σmin.
- **Closed-loop control (500 Hz):** an outer task-space resolved-rate loop (CLIK) with
  Jacobian feed-forward and null-space posture control, plus an inner joint PID loop
  that rejects a constant disturbance. Peak tracking error is 0.2 mm in the reference run.
- **Retrieval:** pre-grasp, insertion along the shelf axis, a suction seal rule, a vacuum
  check, retry up to 3 times, retract, transit, place and return HOME.
- **Rule-based safety:** a reachability pre-check, a payload limit (items over the limit
  are rejected), and a fault stop when the tracking error is too large.
- **Outputs:** dark-themed MP4, GIF and PNG graphs with toggleable panels, and a
  `trajectory.json` of joint angles and tool position (see the README for the layout).

**Quick start**
    pip3 install -r requirements.txt
    python3 pick_place_robot.py          # saves images, video, GIF and JSON, no window
    python3 pick_place_robot.py --show   # also opens the interactive window
    python3 pick_place_robot.py --test   # 29 self-tests
    python3 pick_place_robot.py --check-dh --robot ur5e   # validate a DH table

**Limits:** this is a simulation only. The joint model is decoupled unit-inertia, collision
checking is a floor test only, suction is a rule plus a random leak (not physics), and there
is no vision. Nothing here shows real-world safety or reliability.
