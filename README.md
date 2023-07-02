# Chameleon TF

This is a ROS2 static transform that can observe and imitate anothe transformation. It can be configured in 3 ways.
1. An inital configuration can be written into its launch config
2. A configuration can be set using a service call to command it to take on a new transform.
3. An action that can be used to observe two frames with a number of observations. The average of the samples is computed and used as the new transformation. 


## Initial Configuration
The initial configuration depends on all of the paramteres to be set. It creates the initial transform. If the intial transform is unknown, they can simply be initialized to zero. 

### Parameter list
* `source_frame` This is the parent frame name for the transform
* `target_frame` This is the child frame name for the transform
* `initial_translation` This is an array of doubles with length 3. those values represent the x, y, and z values between the parent and child frame.
* `initial_rotation` This is an array of doubles with length 3. The values represent the roll, pitch and yaw between the parent and child frame.
* `sdtddev_threshold` This is the standard deviation threshold tolerance used when configuring the transform by observation action. 
* `transform_locks` This is an array of booleans with length 6. The values represent which of the transform axes to not allow updates to be imposed on. If an index is set to true, that axis will become locked during mimick updates and prevent its value from changing.

## Service Call
The service call can be used to hard set the transform relationship between the configured frames. the `relationship` field should contain the transform from the paretn frame to the child frame. the service should respond with the `success` flag set and the `err_msg` field as an empty string. If there was somehow an issue, the `success` field will be false, and the `err_msg` populated. 

## Action Goal
The action server functionality can be used to observe the relationship between two other frames on the TF network, and then mimick the average of the relationship.


To do this the `monitor_parent` and `monitor_child` names are used to perform a TF lookup. The action server will collect the number of samples defined in the `samples` field. this normally defaults to 10 samples. From there, the action server computes the average and standard deviation of the collected data. the standard deviation on both position and orientation is checked. If the standard deviation is low enough, the average of the transforms is used as the new relationship between the parent and child specified in the parameters of the node. 