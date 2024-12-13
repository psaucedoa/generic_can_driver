# generic can driver
A simple ros2 driver to get human-readable data from a J1939 sensor

## Motivation
Reading data from a J1939 sensor is kind of annoying.

You can collect logs and use some third party software to decode after-the-fact, or hardcode some
parser for real-time translation for a specific sensor. But we thought it'd be nice to load a dbc
and start reading data through ROS2. So we whipped up a simple driver for ROS2 that integrated with 
ros2_socketcan to decode incoming CAN frames live. 

## Setup

All you need is a device and a DBC! 

> The whole J1939 standard dbc will work - this takes care of using the correct messages for your device

1. Get a J1939 device (like some sensor)

2. Physically connect and power the sensor

3. Set up a CAN port on the host machine

   - For a PEAK CAN device:

       `sudo ip link set can0 up type can bitrate 250000 dbitrate 2000000 fd on fd-non-iso on `

   - For a Lawicel CANusb:

       `sudo slcand -o -c -f -s5 /dev/ttyUSB0 can0`
       
       `sudo ifconfig can0 up`

   - For a Virtual CAN network:
 
       `ip link add dev vcan0 type vcan`

       `ip link set up vcan0`

4. Modify `generic_can_driver/config/socketcan_params.yaml` for your interface (e.g. `can0`)

5. Modify `generic_can_driver/config/generic_can_params.yaml` for your device's id
   
   - This is the last two digits in the identifier of the sensor's CAN frame (e.g. E5) converted to 
     decimal

6. Launch `ros2 launch generic_can_driver generic_can.launch.py`

## Walkthrough

1. on_configure() 
    1. setupDatabase()
       1. dbc_pgns_
    2. configurePublishers()
2. on_activate()
    1. activatePublishers()
    2. subscribe to can bus 
    3. bind rxFrame()
3. rxFrame() loop

## FAQ

 1. Can you launch this node **multiple** times?
       
       > Yes. Make sure to edit the deivce name, frame, and ID in each node's config so that it
       parses the correct device

2. What if I have **multiple** CAN lines?
    > You can launch multiple instances of ros2_socketcan. Just make sure that you give each node
    the correct name for its CAN interface (e.g. can0, can1, can2, ...)

3. What if I have multiple devices on **one** CAN line?
    > You can launch multiple instances of the generic_can_driver, and leave the interface in the 
    node config (e.g can0) the same. However, make sure you update the **device names**, **frames**,
     and **IDs** in each node's config.

4. What message type does this publish out?
    > This publishes a `j1939_interfaces/msg/can_data.hpp` message. The message definition can be
    found under `ros2_j1939/j1939_interfaces`. This is a key-value message type, kinda like 
    `diagnostic_msgs/msg/DiagnosticArray.msg` but instead of being a `string-string` key-value pair,
    this is a `string-float64` key-value pair.

5. Why does it use this custom message?
    > Again, the motivation for this simple driver was to create a quick way to bring in human-
    readable data from a J1939 sensor into ROS2. For a specific implementation, which outputs a 
    "correct" message type (like an IMU sensor message), have a look at device-specific driver
    implementations.

6. How are the publishers set up?
   > On configure, the driver parses the (**user-provided**) dbc file and spins up one publisher per
   dbc message defined within that file. So if you have three dbc messages, this creates three 
   publishers, with the topics automatically named according to `/device_name/message_name`

7. The values of some of my data are constant and REALLY large, why?
   > Sometimes if a field is empty or erroring, that field ("signal") of the message will simply 
   hold the max possible value. So if your temperature is in a range of `[0,202]`, an error (such as
    no sensor, sensor malfunction, etc.) will populate that temperature field with `202`.

## Errors

 1. Socket Can refuses to change lifecycle state

       > Make sure you have a can bus enabled and that its name is correct in the launch file/params

2. `Error receiving CAN message: can0 - CAN Receive Timeout`

    > ros2_socketcan expects a certain frequency of incoming CAN frames. If this frequency is 
    missed, after a certain amount of time it alerts the user. This timeout length is configurable,
    however if your devices are active (just publishing at a slow frequency) this can also just be
    ignored.