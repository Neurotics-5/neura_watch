<img width="1600" height="1086" alt="image" src="https://github.com/user-attachments/assets/3c773c46-a658-472c-a622-6101d8618650" />


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
```
```
ros2 topic pub --once /warning_cmd std_msgs/String "data: 'warning'"
```
```
ros2 topic pub --once /led_cmd std_msgs/String "data: 'on'"
```
```
ros2 topic pub --once /led_cmd std_msgs/String "data: 'off'"
```
```
ros2 topic echo /esp32_hello
```


---

## 🔗 Resources

**micro-ROS main site:** https://micro.ros.org/
    
**micro_ros_setup:** https://github.com/micro-ROS/micro_ros_setup
    
**micro_ros_platformio:** https://github.com/micro-ROS/micro_ros_platformio
    
**micro-ROS Agent:** https://github.com/micro-ROS/micro-ROS-Agent
    
**micro_ros_arduino:** https://github.com/micro-ROS/micro_ros_arduino
    
**ROS 2 Humble:** https://docs.ros.org/en/humble/index.html
    
**PlatformIO:** https://platformio.org/
    
**AZ-Delivery ESP32 DevKit C V4 board info:** https://docs.platformio.org/en/latest/boards/espressif32/az-delivery-devkit-v4.html
