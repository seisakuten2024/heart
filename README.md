# Run
```
$ catkin build heart
$ rosrun heart heart_node
```
# Change parameters
### heart color (RGB)
If set to 1.0 or higher, the brightness will not change.
```
$ rostopic pub /heart_color std_msgs/Float32MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0.0,0.7,0.0]" -1
```
### pulse rate
The default is 0.8. The smaller the value, the faster the beat.
```
$ rostopic pub /pulse_time std_msgs/Float32 "data: 1.0" -1
```
