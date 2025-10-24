# COMP4034 Robotics - Minitask 1
## Group 30

### Team Members
- Mohammed Azeez
- Muhammed Mehboob

## Project Overview
This project implements a ROS2-based control system for moving a TurtleBot3 robot in a precise square pattern. The implementation includes both open-loop (time-based) and closed-loop (odometry-based) control strategies to achieve accurate movement.

### Project Structure
```
.
├── minitask1.py           # Odometry-based control implementation
├── square_open_loop.py   # Time-based control implementation
```

### Features
1. Time-based Control:
   - Precise timing calculations for movements
   - Configurable movement parameters
   - Clean state machine implementation

2. Odometry-based Control:
   - Real-time position feedback
   - Drift correction
   - Improved accuracy using sensor data

### Team Contributions

#### Mohammed Azeez
- Implemented the core state machine logic
- Developed the time-based control system
- Created the base movement algorithms
- Handled ROS2 node initialization and shutdown
- Implemented error handling and safety features

#### Muhammed Mehboob
- Implemented the odometry-based control system
- Developed position tracking algorithms
- Created the drift correction system
- Optimized movement parameters
- Added detailed logging and debugging features

### Technical Implementation
The project uses several key ROS2 concepts:
- Publishers for sending velocity commands (`cmd_vel`)
- Subscribers for receiving odometry data (`odom`)
- State machine for movement control
- Timer-based control loops
- Quaternion-based orientation tracking

### Running the Code
1. Build the package:
   ```bash
   colcon build --packages-select minitask_1
   ```

2. Source the setup file:
   ```bash
   source install/setup.bash
   ```

3. Run the main implementation:
   ```bash
   ros2 run minitask_1 minitask1
   ```

### Movement Parameters
- Linear velocity: 0.2 m/s
- Angular velocity: 0.5 rad/s
- Square side length: 1.0 meters
- Position tolerance: 0.05 meters
- Rotation tolerance: 0.05 radians

### Design Decisions
1. Combined Control Approach:
   - Uses time-based control for initial movement
   - Incorporates odometry feedback for precision
   - Implements smooth transitions between states

2. Safety Features:
   - Proper shutdown handling
   - Velocity ramping for smooth movement
   - Emergency stop capabilities
   - Error detection and recovery

3. Code Organization:
   - Clear separation of concerns
   - Well-documented functions
   - Configurable parameters
   - Reusable components

### Testing
The system was tested in various scenarios:
- Different surface types
- Various movement speeds
- Multiple consecutive square patterns
- Recovery from external disturbances

### Future Improvements
1. Dynamic parameter adjustment
2. Advanced path planning
3. Obstacle avoidance
4. More sophisticated control algorithms

### References
- ROS2 Documentation
- TurtleBot3 Manual
- Robotics Control Theory
- Quaternion Mathematics
