# Introductory Robotics Project
First implementations of classic robotic behavior: teleoperations, driving in a square, obstacle avoidance, and wall following.**

## High-level

There are various challenges in this project. Beyond the initial hurdles of understanding ROS nodes and topics, the physical limits of your robot, and general documentation on different "messages," there is the additional challenge of creating a system that is beautifully designed and coded. Given that this was my first time working with ROS, with some significant time constraints, I elected to get a quick-and-dirty first pass to my goals, and hope to expand from there in the future.

Let's go through each behavior individually:

### Teleoperation

This one was fairly straightforward: Given code that handles keyboard input, I simply had to set specific keys for forward and backward linear velocities, and left and right angular velocities (with alternate keys to set these back to zero). These were continuously updated and passed into a Twist message, which was then published to the 'cmd_vel' topic for the Neato to interpret into action.

### Driving in a square

I elected for a timing-based approach for this, rather than an odometry-based one. With timing the idea is straightforward:

- determine your desired linear and angular velocities (mine were 0.3 m/s and 0.3 rad/s)
- calculate the time required to travel your target distance (to travel 1 meter, I needed 3.33 seconds)
- calculate the time required to turn pi/2 rad (pi/.6 in my case)
- keep track of time, and have the Neato travel forward, stop, and turn for the correct amount of time.
- do this 4 times, and you've driven a square!

### Obstacle Avoidance

### Wall Following

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

