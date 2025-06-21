Great to hear it works now! 🎉
Here's a clean and professional `README.md` for your GitHub repository:

---

```markdown
# ROS 2 Motor Controller for IWS Servo Motors

This package provides a ROS 2 node to control two IWS servo motors over UART based on velocity commands published to the `/cmd_vel` topic. It also performs basic odometry calculations based on encoder feedback.

## 🧠 Features

- Controls dual IWS servo motors over `/dev/ttyACM0` and `/dev/ttyACM1`
- Subscribes to `/cmd_vel` to receive velocity commands
- Sends motor speed via serial commands
- Initializes and enables motors automatically
- Computes and logs odometry (`x`, `y`, and `theta`)

---

## 📦 Dependencies

- ROS 2 (tested with Humble/Foxy)
- `geometry_msgs/msg/Twist`
- Python 3
- IWS servo motors with UART support

---

## 📁 Directory Structure

```

ros2\_ws/
└── src/
└── motor\_control/
├── motor\_controller.py
├── package.xml
└── setup.py

````

---

## ⚙️ Installation

1. Clone this repo into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/your_username/motor_control.git
````

2. Build the workspace:

   ```bash
   cd ~/ros2_ws
   colcon build
   ```

3. Source the workspace:

   ```bash
   source install/setup.bash
   ```

---

## 🚀 Usage

1. Connect your IWS servo motors to `/dev/ttyACM0` and `/dev/ttyACM1`

2. Run the motor controller node:

   ```bash
   ros2 run motor_control motor_controller
   ```

3. Publish velocity commands to move the robot:

   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
   ```

---

## 🛠️ Configuration

If your motor ports are different, update these lines in `motor_controller.py`:

```python
self.motor1_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
self.motor2_port = serial.Serial('/dev/ttyACM1', 115200, timeout=0.1)
```

---

## 📓 Notes

* Motors are auto-initialized on first velocity command (`/cmd_vel`)
* Odometry is computed but not yet published as a ROS topic (can be added)
* Make sure the motor power supply is connected and turned on

---

---

## 🤝 Contributing

Pull requests and improvements are welcome!

```

---

Let me know if:
- You want to include odometry publishing to `/odom`
- You’d like to add TF broadcasting (e.g., `odom → base_link`)
- You want to include a launch file and `install` section in `setup.py`

I’ll help extend this as needed.
```
