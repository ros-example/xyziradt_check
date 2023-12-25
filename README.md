# 

```bash
ros2 bag play src/bag/lidar.bag --loop

ros2 topic echo /sensing/lidar/top/pointcloud_raw_ex

ros2 run xyziradt xyziradt
```
