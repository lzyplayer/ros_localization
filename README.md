#### localization

------

:)

2. set parameter in `minimalBringUp.launch` or other launch file
3. `roslaunch prm_localization minimalBringUp.launch `
4. `rosbag play PATH_TO_YOUR_BAG -r (1,2,3,4)`
5. current version only require `lidar_data`, set a low rosbag publish rate if your PC have low performance.

