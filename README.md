# Introductory Robotics Project
First implementations of classic robotic behavior: teleoperations, driving in a square, obstacle avoidance, and wall following.**

## High-level Structures

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

I coded this in python with an object-oriented framework, using one Class object per behavior/node. An individual class would look something like this:

- class Behavior(Object)
  - Initialize variables
    - This is where I initialize the behavior's node, such as 'wall_following'
    - This is also where I define the topics to publish or subscribe to, such as '/scan' or '/cmd_vel'
    - It can be a very long list with different message parameters
  - Various helper functions
    - For example, ObstacleAvoidance required these functions:
      - process_scan(), to get the data from the lidar scans
      - find_max_distance(), to filter out erroneous readings
      - check_obstacle(), to check if there is an object in front of the robot
      - make_horizontal(), to compare distances and set velocities accordingly
  - Run() function
    - This calls the helper functions and continously publishes velocity updates in a Twist message to '/cmd_vel'

## Challenges

I encountered many, many challenges in this project. The deepest rabbit hole I fell down was in trying to visualize the distances coming out of the Neato in wall-following mode. I thought it would be helpful to see what the comparison looked like-- however, I couldn't figure out how to publish multiple arrow markers in a nice, clean way, and had to abandon the thought.

I also had trouble integrating different behaviors together. It would have been very nice to make use of my object-oriented structure to call one behavior class from another, and have a catch-all program running to control the Neato. Instead, I ended up running teleoperations from one terminal, and my specific behavior from another, with the latter always overriding the former. This caused some interesting hassle when things went wrong-- especially because I was also unable to call my emergency stop for the same reason. I have yet to solve this.

## Improvements

There are many improvements I would like to implement! The most appealing one is one I mentioned in the Challenges section-- integrating behavior into one executable would vastly streamline robot control and testing. Alongside that, here are some behavior-specific improvements I'd like to make:

### Driving in a Square

It would be very cool to get this odometry based! I imagine it would allow for more specific adjustments while driving the square-- the current method has some variance, and can become offset rather quickly.

### Obstacle Avoidance

A clear improvement here is to have the Neato avoiding obstacles while traveling toward a specific destination. This would also require odometry-based movement, with the robot constantly updating its surroundings to find the clearest pathway toward its destination.

### Wall Following

My implementation has some clear limits. Currently, it only works if you have it headed toward a known wall in the first place-- the code identifies an obstacle, not an actual wall structure. A more interesting (and difficult) method could involve taking in the surroundings and identifing nearby walls or corners, and specifically targeting them in order to follow. This would prevent isntances of, say, driving off into infinity at a corner, and getting caught between two non-wall obstacles unsure where to turn next.

## Going Forward

Some key things I'd like to bring forward in future robotics work is a more solid structure of code integration. Setting up essentials like visualizing and emergency stops in a way that can be quickly integrated into any behavior function would be immensely valuable.

That said, another key takeaway with this project has been to recognize when I'm stuck in a rabbit hole, and call it quits earlier in order to pursue other work. I believe I would have been able to create some form of person following or the finite-state-controller had I not spent so much time struggling with visualization code.
