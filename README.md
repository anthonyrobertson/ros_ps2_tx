# Wifi PS2 Controller for ROS2

Wire a PS2 controller directly to an ESP8266-like device and messages will be passed to a ROS2 project
via rosbridge_websocket. Messages are published on the `/joy` topic, of the type `sensor_msgs/msg/Joy`.

### Hardware Setup

Wiring connections
| PS2 Controller Wire | ESP8266 Pin | GPIO Number |
|---------------------|------------|-------------|
| VCC                | 3.3V        | -           |
| GND                | GND         | -           |
| Brown (D6)        | D6         | GPIO12      |
| Orange (D7)       | D7         | GPIO13      |
| Yellow (D3)       | D3         | GPIO0       |
| Blue (D5)         | D5         | GPIO14      |
| Green & Gray      | Not Connected | -       |

### Configuration:
Configure the wire variables to GPIO pins
```
#define PS2_DAT 12 //  D6  
#define PS2_CMD 13 // D7  
#define PS2_SEL 0 //  D3  this is CS  
#define PS2_CLK 14 // D5
```

TBD, more pictures


* Wifi
    * SSID
    * password
* IP of the host running the

### Build & Flash
Then once you plug it in it will look for the host with the rosbridge and start sending it messages

