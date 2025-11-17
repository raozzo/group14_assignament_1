# GROUP 14 - ASSIGNMENT 

## Launch instruction 

```bash
ros2 launch group14_assignment_1 assignment1_launch.py 
```

## Exercise description 
At the launch of the simulation, the turtlebot will spawn at the Iaslab entrance. The goal is to
make the robot move to a position in the lab between two AprilTags visible from a service
camera. In that position, the robot will be able to detect three cylindrical tables placed
somewhere in the room.

- [ ] 1.detection of the apriltag
- [ ] 2.navigation to the apriltags (find a good position between the two and reach it without hitting them)
- [ ] 3.detection of tables using any available sensor
- [ ] 4.return the position of tables relative to odom reference frame

**OPTIONAL (+3 points):** Stop the navigation to goal when the turtlebot enters the corridor and implement your own
navigation method sending velocity commands to the robot and detecting the walls with the
lidar. Once at the end of it, resume the navigation to the goal.
