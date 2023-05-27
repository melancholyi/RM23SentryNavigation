# SCURM火锅战队 23赛季哨兵导航
## 文件说明g  
# 打完比赛在写文档吧  

## 比赛应该干什么 

- 建图录制bag   
```
ros2 bag record /livox/lidar /livox/imu
```    
- 启动fastlio  
```
source install/setup.bash
ros2 launch sentry_nav2_bringup fastlio_mapping.launch.py
```
- 启动octomap
```
source install/setup.bash
ros2 launch octomap_generate octomap_server.launch.py
```
- rqtserver保存fastlio的图
- 保存octomap的图
```
ros2 launch octomap_generate octomap_saver.launch.py
```
- 压制地图
```
ros2 launch pcl_cloud run.launch.py
```
- 保存二维pgm地图
```

```
- 启动navigation2
```
source install/setup.bash
ros2 launch sentry_nav2_bringup navigation2.launch.py
```

