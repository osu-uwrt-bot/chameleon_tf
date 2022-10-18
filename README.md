# Chameleon TF

This is a ROS2 static transform that can observe and imitate anothe transformation. It can be configured in 3 ways.
1. An inital configuration can be written into its launch config
2. A configuration can be set using a service call to command it to take on a new transform.
3. An action that can be used to observe two frames with a number of observations. The average of the samples is computed and used as the new transformation. 


## Initial Configuration
The initial configuration depends on all of the paramteres to be set. It creates the initial transform. If the intial transform is unknown, they can simply be initialized to zero. 

### Paramter list
* `source_frame` This is the parent frame name for the transform
* `target_frame` This is the child frame name for the transform
* `initial_translation` This is an array of doubles with length 3. those values represent the x, y, and z values between the parent and child frame.
* `initial_rotation` This is an array of doubles with length 3. The values represent the roll, pitch and yaw between the parent and child frame.
* `sdtddev_threshold` This is the standard deviation threshold tolerance used when configuring the transform by observation action. 

## Service Call


## Action Goal