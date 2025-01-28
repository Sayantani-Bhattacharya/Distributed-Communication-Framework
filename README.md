# Distributed low-level communication framework

Author: Sayantani Bhattacharya

## Project Objective:

 1. Enable peer-to-peer communication of large data among agents.
 2. Optimize for low latency, optimal discovery and thus network efficiency.
 3. Enable custom msg packet [point cloud, sensor and video frame data] options.
 4. Provide message queuing, user-friendly logging, and reliable delivery mechanisms.
 5. Allow cross-compilation among Ubuntu/Linux/Raspberry Pi systems. (Stretch goal)
 
## Application:
I plan to use it as the communication layer of the Unitree Quadruped Fleet, as a part of my winter project.</br>
To enable collaborative exploration and criticality detection in dangerous/disater regions.</br>
[The project repo](https://github.com/Sayantani-Bhattacharya/Multi-Hetro-Agent-Exploration-on-UnitreeGOs)


## Steps:

### Development:
1. Minimally Functional Program: the first step would be to create ROS publishers of pcl data from Jeton,integrating it with ros2-zenoh-bridge and have that transfer data to another subscriber node of the same Jetson. 
2. Then I would be adding compression, serialization and decompression features for custom msg types.
3. Filtering or any other method to enhance discovery of packets.
4. Enabling multi agent operation (should already be working, not sure though).
5. (Optional) Adding some extended features to the framework like configuring, sanity checking and handling transport layer (TCP/UDP). 
6. (Optional) Implement custom encryption for data packets before publishing over Zenoh.

### Deployment: 
Setting up the framework in Jetson and  deploying over a shared WiFi network.
### Test and Validate:
1. Single Peer Test: Verify ROS2 publishes and Zenoh bridges the point-cloud data correctly.
2. Multi-Peer Test.
3. Performance Testing: Measure latency, bandwidth usage, and packet loss. And optimize QoS and buffer settings.


## Library, Framework & Tech Stack:
1. C++
2. ROS2 humble
3. Ubuntu 22 [Docker env] / Jetson Orin Nano
4. Zenoh libraries and ros2-bridge.
5. Serialization Library: Protocol buffer/ msgpack
6. Point cloud library

## Final Goal
The ultimate goal of the project would be to develop an open source framework, that can be used by anyone working with multi agent systems (swarm, fleet, IOT). 
It would be an out of the box system, deployable for low-latency reliable communication of large sensor data.

## Minimum Viable Product
To develop the same framework without consideration on discovery filters, latency, and compression. Basically, to just have a working system, where large 
data is transfered among multiple agents with decent reliablity.
