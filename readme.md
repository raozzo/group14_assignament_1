# GROUP 14 - ASSIGNMENT 

## LINKS

1. [Assignment 1 repo](https://github.com/raozzo/group14_assignment_1)
2. [Interfaces repo](https://github.com/raozzo/group14_interfaces)
3. [Drive Folder](https://drive.google.com/drive/u/1/folders/1HIBnHfArfs1EaReJV7I966hIEcrrfIBU)

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



### 📦 A Modular Architecture for Your Assignment

I recommend you create **two main nodes** for your package, which will work alongside the nodes that are *given* (like `apriltag_ros` and `nav2`).

#### A. The "Given" Nodes (Specialists)
* **`apriltag_ros` Node (The "Eyes"):**
    * **Responsibility:** Detects AprilTags.
    * **Output:** Publishes transforms (`tf`) of the tags relative to the camera.
* **`nav2_stack` (The "Driver"):**
    * **Responsibility:** Manages all navigation and obstacle avoidance.
    * **Input:** An **Action** goal (`/navigate_to_pose`).
    * **Output:** Action feedback and result (success/failure).

#### B. Your Custom Nodes (Your Specialists)
* **Node 1: `table_detector_node` (The "Scanner")**
    * **Responsibility:** Find cylinders (tables) in the LIDAR scan. This is *exactly* what you did in the previous exercise by finding apples. You can reuse that code.
    * **Input:** Subscribes to `/scan` (`sensor_msgs/msg/LaserScan`).
    * **Output:** Publishes a `geometry_msgs/msg/PoseArray` on a topic like `/table_poses`. The poses should be relative to the LIDAR's frame (e.g., `base_scan`).

* **Node 2: `assignment_manager_node` (The "Manager" or "Brain")**
    * **Responsibility:** This is the most important node. It manages the *state* of the mission and tells the other specialists what to do.
    * This node will be a **state machine** that looks something like this:
        1.  **State 1: FINDING GOAL:** Subscribes to `/tf`. It waits until it sees both required AprilTags.
        2.  **Calculation:** Once it sees both, it uses `tf2` to find their positions and calculates a `goal_pose` (e.g., the midpoint between them).
        3.  **State 2: NAVIGATING:** It becomes an **Action Client** for Nav2 and sends the `goal_pose` to the `/navigate_to_pose` action. It then waits for the action to complete (success or failure).
        4.  **State 3: SCANNING:** Once Nav2 reports **success** (the robot has arrived), the manager subscribes to your `/table_poses` topic.
        5.  **State 4: REPORTING:** As soon as it receives a `PoseArray` from your `table_detector_node`, it uses `tf2` (just like in the AprilTag exercise) to transform those poses from the `base_scan` frame to the `odom` frame.
        6.  **Done:** It can then print the final `odom` positions to the terminal or publish them on a final topic.
