# micro-ROS ESP32 WiFi Example (PlatformIO + ROS 2 Humble)

This project demonstrates how to run a WiFi-connected [micro-ROS](https://micro.ros.org/) node on an ESP32 (e.g. AZ-Delivery DevKit C V4), publishing data to a ROS 2 Humble host.

---

## ✅ Prerequisites

- ROS 2 Humble installed on your host machine
- `micro_ros_setup` and `micro_ros_agent` already built and sourced
- ESP32 board (tested: AZ-Delivery DevKit C V4)
- PlatformIO CLI or VSCode extension
- ESP32 and ROS 2 host must be on the same WiFi network

---

## ⚙️ Setup Instructions

### 1. Clone This Repo
```
bash
git clone <this-repo-url>
cd <this-repo>
```

2. Configure PlatformIO

Ensure your platformio.ini contains:
```
[env:az-delivery-devkit-v4]
platform = espressif32
board = az-delivery-devkit-v4
framework = arduino
board_microros_distro = humble
board_microros_transport = wifi
lib_deps = https://github.com/micro-ROS/micro_ros_platformio
```

3. Set Your WiFi and Agent IP

In src/main.cpp, update:
```
char ssid[] = "YOUR_WIFI_SSID";
char password[] = "YOUR_WIFI_PASSWORD";
char agent_ip[] = "xxx.xxx.x.xxx";  // IP of your ROS 2 host
```

4. Build and Upload

Use PlatformIO as usual (GUI or CLI):
```
pio run --target upload
```

No additional setup needed — the micro_ros_platformio library will auto-install.
🛰️ Start the micro-ROS Agent on ROS 2 Host

In a terminal:
```
source /opt/ros/humble/setup.bash
cd ~/uros_ws
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
Keep this terminal open — it will display connection logs.
📡 Listen to ESP32 Messages

Open another terminal:
```
source ~/uros_ws/install/setup.bash
ros2 topic echo /esp32_hello
```
You should start seeing:

data: Hello from ESP32

📂 File Structure

.
├── src
│   └── main.cpp         # micro-ROS publisher code
├── platformio.ini       # PlatformIO environment config
└── README.md

🔗 Resources

    micro-ROS

    micro_ros_setup

    micro_ros_platformio

    PlatformIO

🧪 Tested With

    ROS 2 Humble on Ubuntu 22.04

    AZ-Delivery ESP32 Dev Kit C V4

    PlatformIO Core 6.x

    micro-ROS Agent via udp4 transport
