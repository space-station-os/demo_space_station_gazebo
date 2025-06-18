# SPACE STATION GAZEBO HARMONIC

[spacestation_full.webm](https://github.com/user-attachments/assets/39a9498a-2918-42c6-84a6-8373325f9fbe)




To launch the simulation 

```
ros2 launch ssos_new_description gazebo.launch.py
```


> Note: Please add the following line to bashrc (add the actual path) 

```bash
export GZ_SIM_RESOURCE_PATH=$HOME/ssos_ws/src/ssos_new_description/urdf:$HOME/ssos_ws/src/ssos_new_description/meshes:${GZ_SIM_RESOURCE_PATH}
```


To run the teleop 

```bash
ros2 run space_station_control teleop   
ros2 run space_station_control mux 
```

Please run both nodes in a dedicated terminal or using tmux 