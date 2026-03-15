# Dusra_code

## Setup Instructions

### 1. Clone the Repository

```
git clone --recursive https://github.com/EkHiToHallHai/Dusra_code
cd Dusra_code
```

---

### 2. Create `.env` File

Create a file called `.env` in the root directory.

Example:

```
USER=Uddipto
UID=1000
GID=1000
```

---

### 3. Disable This VS Code Setting

Turn off the following setting in VS Code:

![VS Code Setting](https://github.com/user-attachments/assets/1a783f0c-14fd-4b85-a17f-148e5bdb24a6)

---

### 4. Allow Display Access

Run this command on the host machine:

```
xhost +local:host
```

---

### 5. Reopen in Container

Click the **`><` icon** in the bottom left corner of VS Code and select:

**Reopen in Container**



---

### 6. Build the Workspace

Inside the container run:

```
colcon build
```

Then source the workspace:

```
source install/setup.bash
```

---

### 7. Set Robot Base

```
export LINOROBOT2_BASE=2wd
```

---

### 8. Launch Gazebo

```
ros2 launch linorobot2_gazebo gazebo.launch.py
```

Gazebo should now launch with the robot simulation.




------------------------
#### LAUNCH INSTRUCTIONS

export LINOROBOT2_BASE=x2
''''
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/ros_ws/src/packages/linorobot2/linorobot2_gazebo/models
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/ros_ws/install/linorobot2_description/share
'''''
ros2 launch linorobot2_gazebo gazebo.launch.py \
world_name:=baylands_world \
spawn_x:=205 \
spawn_y:=155 \
spawn_z:=2
