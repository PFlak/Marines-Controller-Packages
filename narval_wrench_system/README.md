# Narval wrench system

## Description 

Package that handles joystick, dualshock4 or controller and publishes wrench based on axis

## Nodes

```bash
base_node
```

### base_node

Node that subscribe to joystick topics and publishes wrenches based on them.

#### Parameters

- `use_dualshock4` (default: True) - whether to use dualshock4 controller.
- `wrench_topic_name` (default: "joy_wrench") - name of published topic with `Wrench` message
- `wrench_stamped_topic_name` (default: "joy_wrench_stmp") name of published topic with `WrenchStamped` message
- `send_stamped` (default: True) - create topic with `wrench_stamped_topic_name` and `WrenchStamped` message or default `wrench_topic_name` and `Wrench` message
- `frame_id` (default: base_link) - frame id for stamped message
- `max_force` (default: 1) - max force to emit in wrench
- `max_torque` (default: 1) - max torque to emit in wrench
- `ds4_force_x` (default: axis_left_y) - name of dualshock4 axis
- `ds4_force_y` (default: axis_left_x) - name of dualshock4 axis
- `ds4_force_z` (default: axis_right_y) - name of dualshock4 axis
- `ds4_torque_x` (default: axis_right_x) - name of dualshock4 axis
- `ds4_torque_y` - name of dualshock4 axis
- `ds4_torque_z` - name of dualshock4 axis

## Launch

```bash
base.launch.py
```

### base.launch.py

Launches `ds4_driver` node for handling Dualshock4 and `base_node` node

#### Parameters

- `config` (default: /narval_wrench_system/config/params.yaml) - path to config file
