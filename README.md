# Introductory Robotics Project
First implementations of classic robotic behavior: teleoperations, driving in a square, obstacle avoidance, and wall following.**

## High-level

There are various challenges in this project. Beyond the initial hurdles of understanding ROS nodes and topics, the physical limits of your robot, and general documentation on different "messages," there is the additional challenge of creating a system that is beautifully designed and coded. Given that this was my first time working with ROS, with some significant time constraints, I elected to get a quick-and-dirty first pass to my goals, and hope to expand from there in the future.

Let's go through each behavior individually:

### 1) Teleoperation

This one was fairly straightforward: Given code that handles keyboard input, I simply had to set specific keys for forward and backward linear velocities, and left and right angular velocities (with alternate keys to set these back to zero). These were continuously updated and passed into a Twist message, which was then published to the 'cmd_vel' topic for the Neato to interpret into action.

### 2) Driving in a square

I elected for a timing-based approach for this, rather than an odometry-based one. With timing the idea is straightforward:

- determine your desired linear and angular velocities (mine were 0.3 m/s and 0.3 rad/s)
- calculate the time required to travel your target distance (to travel 1 meter, I needed 3.33 seconds)
- calculate the time required to turn pi/2 rad (pi/.6 in my case)
- keep track of time, and have the Neato travel forward, stop, and turn for the correct amount of time.
- do this 4 times, and you've driven a square!

### 3) Obstacle Avoidance

I implemented a simple concept for obstacle avoidance. First, check for a reading of an object in front of the Neato (in a span of 10 degrees across the front axis of the robot). If that object is less than 1.25 meters away, change the linear velocity to 0 m/s and the angular velocity to 1 rad/s. The Neato will turn until it no longer detects an object in front of it, and continue moving forward at that point.

### 4) Wall Following

This one was a little trickier. The basic way I thought about this was to imagine lines pointing out from the Neato every 45 degrees around its Z axis (with straight ahead being 0 degrees), where the lines end at the first obstacle they encounter. When the Neato is parallel to a wall on its right side, the lines at 315 (right-front) and 225 (right-back) degrees will be equal. Similarly, when the Neato is parallel to a wall on its left side, the lines at 45 (left-front) and 135 (left-back) will be equal. So the goal is to achieve that state:

- First, find the distances at each of those points (45, 135, 225, and 315 degrees)
  - To account for sensor error, I took the distances in a range around those target points, and took the maximum distance of those. That counters instances of non-values.
- Compare the distances on the right-front and right-back positions. 
  - If the right-front distance is longer, the robot needs to turn right to become parallel to the wall. If the right-back distance is longer, it needs to turn left.
  - When the distances are equal (within reasonable tolerance), continue forward.
- Do the same for the left-front and left-back positions.

If all goes well, the Neato should align itself along the wall, once it has encountered one. It is also good to have detections at the front and back to avoid head-on collisions, and at the sides to maintain an even distance from the wall.


## Code Structure

## Challenges

## Improvements

## Thoughts About Person-Following and Finite-State Control

- describe at high-level. relevant diagrams. discuss strategy
- finite state controller :(
- how was code structured. detail object-oriented structure
- challenges
- improvements
- key takeaways for future robotic programming projects

