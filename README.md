# CV Object Tracker

A ROS 2 package for real-time object detection and tracking using computer vision. This package detects colored objects in camera images and outputs control commands to track them using PID control.

## Overview

The `cv_object_tracker` package consists of two main components:

1. **Detector**: Processes camera images using HSV color space filtering to detect objects of interest
2. **Tracker**: Converts detections into robot motion commands using PID control to keep the target centered in the image frame

## Features

- **Real-time Object Detection**: HSV color-based filtering for robust object detection
- **PID Control**: Smooth tracking with configurable PID parameters
- **Lifecycle Management**: Uses ROS 2 lifecycle nodes for proper state management
- **Multi-threaded Execution**: Efficient concurrent processing of detection and tracking
- **Image Transport**: Optimized image subscription and publishing

## Architecture

### Components

#### Detector Node
- Subscribes to camera images via `image_transport`
- Applies HSV color space filtering with configurable thresholds
- Publishes detected objects as `Detection2DArray` messages
- Lifecycle-aware node with configure, activate, and cleanup states

#### Tracker Node
- Subscribes to detection messages from the Detector
- Calculates error between object position and image center
- Uses PID control to compute velocity commands
- Publishes `Twist` messages for robot motion control
- Lifecycle-aware with proper state transitions

#### PID Controller
- Implements proportional-integral-derivative control
- Configurable gains (Kp, Ki, Kd) and output limits
- Anti-windup integral clamping
- Calculates control outputs based on error and time delta

## Dependencies

- **ROS 2**: Core middleware
- **rclcpp**: ROS 2 C++ client library
- **rclcpp_lifecycle**: Lifecycle node management
- **image_transport**: Optimized image transport
- **cv_bridge**: OpenCV-ROS message conversion
- **OpenCV**: Computer vision library
- **sensor_msgs**: Camera image messages
- **vision_msgs**: Detection message types
- **geometry_msgs**: Robot motion commands

## Building

From the workspace root:

```bash
colcon build --packages-select cv_object_tracker
```

## Running

### Launch the Package

```bash
ros2 launch cv_object_tracker run.launch.py
```

This command:
- Configures and activates both detector and tracker nodes
- Starts the multi-threaded executor for concurrent processing
- Initializes all subscriptions and publishers

### Manual Node Execution

For debugging or custom configurations:

```bash
# Run the main executable
ros2 run cv_object_tracker cv_node
```

## Configuration

### Color Detection Parameters

Configure HSV color thresholds in the detector node:

- **H_MIN / H_MAX**: Hue range (0-179 in OpenCV)
- **S_MIN / S_MAX**: Saturation range (0-255)
- **V_MIN / V_MAX**: Value range (0-255)

### PID Parameters

Configure in the tracker node:

- **Kp**: Proportional gain
- **Ki**: Integral gain  
- **Kd**: Derivative gain
- **min_output**: Minimum velocity command
- **max_output**: Maximum velocity command

## Topics

### Published Topics

- `/detection_results` (`vision_msgs/Detection2DArray`): Detected objects with bounding boxes
- `/cmd_vel` (`geometry_msgs/Twist`): Robot velocity commands

### Subscribed Topics

- `/camera/image_raw` (`sensor_msgs/Image`): Input camera stream
- `/detection_results` (`vision_msgs/Detection2DArray`): Detection feed for tracker

## Lifecycle States

Both nodes follow the ROS 2 lifecycle pattern:

1. **Unconfigured**: Initial state
2. **Configured**: Resources allocated, not processing
3. **Active**: Full operation, processing and publishing
4. **Inactive**: Paused operation
5. **Finalized**: Resources released

## Usage Example

```bash
# Terminal 1: Start the simulation/camera
ros2 launch mybot launch_sim.launch.py

# Terminal 2: Run object tracker
ros2 launch cv_object_tracker run.launch.py

# Terminal 3: Monitor tracked object
ros2 topic echo /cmd_vel
```

## Node Graph

```
camera/image_raw
    |
    v
[Detector Node]
    |
    v
/detection_results
    |
    v
[Tracker Node]
    |
    v
/cmd_vel -> Robot
```

## Troubleshooting

### Object Not Detected
- Verify HSV color range parameters match the target object
- Check camera feed: `ros2 run rqt_image_view rqt_image_view`
- Adjust lighting conditions

### Unstable Tracking
- Fine-tune PID parameters (Kp, Ki, Kd)
- Reduce Kd if oscillations occur
- Increase Kp for faster response

### No Command Output
- Verify both detector and tracker nodes are active
- Check topic connections: `ros2 topic list` and `ros2 topic echo`
- Review node logs: `ros2 node list` and `rqt_graph`

## Notes

### Adding a Moving Target for Testing

To test the object tracker with an automated moving target in simulation, you can add a scripted actor to your Gazebo world file (.sdf). This blue sphere will follow a predefined square trajectory, providing a consistent test target for the tracking system.

Add the following code to your world SDF file (e.g., `tugbot_warehouse.sdf`) inside the `<world>` element:

```xml
<!-- AUTOMATIC MOVING TARGET -->
<actor name="moving_blue_target">
  <!-- Visual of the ball -->
  <link name="link">
    <visual name="visual">
      <geometry>
        <sphere>
          <radius>0.2</radius>
        </sphere>
      </geometry>
      <material>
        <ambient>0 0 1 1</ambient> <!-- Blue -->
        <diffuse>0 0 1 1</diffuse>
        <specular>0.1 0.1 0.1 1</specular>
      </material>
    </visual>
  </link>

  <!-- The Scripted Path -->
  <script>
    <loop>true</loop>
    <auto_start>true</auto_start>
    <trajectory id="0" type="square">
      <!-- Start at Center (X=2, Y=0) -->
      <waypoint>
        <time>0.0</time>
        <pose>2 0 0.25 0 0 0</pose>
      </waypoint>
      <!-- Move to Y=2 -->
      <waypoint>
        <time>5.0</time>
        <pose>2 2 0.25 0 0 0</pose>
      </waypoint>
      <!-- Move to Y=-2 -->
      <waypoint>
        <time>15.0</time>
        <pose>2 -2 0.25 0 0 0</pose>
      </waypoint>
      <!-- Return to Center -->
      <waypoint>
        <time>20.0</time>
        <pose>2 0 0.25 0 0 0</pose>
      </waypoint>
    </trajectory>
  </script>
</actor>
```

**Parameters Explanation:**

- **Radius**: 0.2 meters (adjust size as needed)
- **Color**: Blue (RGB: 0 0 1) - matches HSV detection filtering
- **Waypoints**: Define the target's position and timing:
  - X, Y, Z coordinates and orientation (roll, pitch, yaw)
  - Time in seconds for each waypoint
  - The target interpolates smoothly between waypoints
- **Loop**: Set to `true` for continuous looping motion
- **auto_start**: Set to `true` to begin motion automatically when simulation starts

**Testing Tips:**

1. Adjust waypoint coordinates to create different paths (square, circle, figure-8, etc.)
2. Modify timing values to test tracking at different speeds
3. Change the sphere radius to test object size detection
4. Experiment with HSV thresholds to optimize blue detection

## License

See LICENSE file for details.

## Maintainer

- **Email**: pritampaulwork7@gmail.com
- **Author**: ubuntu
