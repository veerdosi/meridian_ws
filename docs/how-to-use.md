# How to Use

## Prerequisites

- Docker running with the `meridian_dev` container
- (For visual mode) A VNC viewer — macOS built-in works fine

```bash
docker start meridian_dev
```

---

## Build

Run this once after pulling changes:

```bash
docker exec -it meridian_dev bash -c "
  source /opt/ros/jazzy/setup.bash &&
  cd /root/meridian_ws &&
  colcon build &&
  echo 'Build OK'
"
```

---

## Run (headless)

Runs 10 insertion episodes and prints results to the terminal:

```bash
docker exec -it meridian_dev bash -c "
  source /opt/ros/jazzy/setup.bash &&
  source /root/meridian_ws/install/setup.bash &&
  ros2 launch meridian_sim sim_sensors.launch.py
"
```

Expected output:
```
Episode  1 → SEATED  peak=42.41N dist=0.0051m dur=0.52s
Episode  2 → SEATED  peak=40.03N dist=0.0051m dur=0.77s
...
Total: 7/10 success
```

---

## Run (with visualiser)

**Step 1** — Connect VNC:
- macOS: Finder → Go → Connect to Server → `vnc://localhost:5900`
- Password: `meridian`

**Step 2** — Launch with the viewer flag:

```bash
docker exec -it meridian_dev bash -c "
  source /opt/ros/jazzy/setup.bash &&
  source /root/meridian_ws/install/setup.bash &&
  ros2 launch meridian_sim sim_sensors.launch.py enable_viewer:=true
"
```

The MuJoCo viewer opens in the VNC window and shows the UR5e arm running live insertion episodes.

---

## Stop

```bash
docker exec meridian_dev bash -c "pkill -f ros2; pkill -f mujoco_sim; pkill -f compliance_controller; pkill -f ft_sensor"
```
