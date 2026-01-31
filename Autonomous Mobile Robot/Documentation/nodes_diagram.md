## Node Architecture Diagram

The following diagram represents the **ROS 2 node architecture** of the Puzzlebot system.  
It illustrates how perception, control, navigation, and odometry nodes communicate through ROS topics to achieve autonomous robot behavior.

---

## Node Descriptions

### `controller`
Main control node of the system. Manages the robot’s state machine and publishes velocity commands based on navigation and perception inputs.

### `line_state`
Processes line-following information and determines the robot’s current state relative to the track.

### `go_to_goal`
Handles navigation toward the current goal using odometry, position, and perception data.

### `path_generator`
Generates navigation goals based on detected traffic signals and track logic.

### `odometry_node`
Computes the robot’s pose using wheel encoder data and provides position feedback for navigation.

### `puzzlebot_serial_node`
Manages low-level communication between ROS 2 and the robot hardware, sending velocity commands and receiving encoder measurements.

### `traffic_light`
Detects traffic lights using a neural network and publishes the detected traffic light color.

### `traffic_signal`
Detects traffic signs using a neural network and publishes the identified signal.

### `video_source`
Captures and publishes raw camera images used by the perception nodes.

### `micro_ros_agent`
Enables communication between the Jetson platform and the microcontroller through micro-ROS.

```mermaid
flowchart LR

%% Video pipeline
VS[/video_source/video_source/] --> VR[/video_source/raw/]

%% Perception nodes
VR --> CTRL[controller]
VR --> TL[traffic_light]
VR --> TS[traffic_signal]

TL --> C[/color/]
TS --> S[/signal/]

%% Controller and states
C --> G[go_to_goal]
S --> G
LS[line_state] --> G
LS[line_state] --> ODOM
POS[/position/] --> G

CTRL[controller] --> LS
CTRL --> POS
CTRL --> CMD[/cmd_vel/]

%% Motion and odometry
CMD --> PSN[puzzlebot_serial_node]
PSN --> VELR[VelocityEncR]
PSN --> VELL[VelocityEncL]

VELR --> ODOM[odometry_node]
VELL --> ODOM

ODOM --> POSE[/pose/]
ODOM --> RO[/reset_origin/]
RO --> PG
S --> PG[path_generator]

%% Path planning
PG[path_generator] --> GOAL[/goal/]
GOAL --> G

%% Odometry finish
ODOM --> OF[odometry_finish]
OF --> CTRL

%% Pose
POSE[/pose/] --> G[go_to_goal]

