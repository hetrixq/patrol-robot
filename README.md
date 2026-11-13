# Patrol RBM v5 — manual CHECK + annotated QR camera (no YOLO)

Standalone add-on to the existing `robotics-laboratory/rbm` stack, intended for
Raspberry Pi 5 8 GB + ROS 2 Docker + 2D LiDAR/SLAM + Nav2 + CSI camera through
RTSP/MediaMTX + Google Voice HAT audio + Lichtblick/Foxglove bridge.

This version was built after v4 crashed with a native `Segmentation fault`
immediately after loading/starting NCNN. **No NCNN, YOLO, torch, or Ultralytics
is imported or required by this app.** It is a simulated person detection:
a supervisor presses `CHECK`. Nav2 still handles obstacles/paths via LiDAR.

## Components

- `patrol_node.py`: *one* ROS 2 process, Nav2 action client, route storage,
  manual check trigger, 15-second QR verification, sound, publishers.
- `vision.py`: *one* background RTSP client, OpenCV QRCodeDetector, annotated
  JPEG frames at 2 FPS, closed when not needed. No model inference.
- `core.py`: pure route/QR decisions, loop or random, max 10 points.
- `lichtblick/PATROL_RBM_v5.json`: map + camera + QR status + buttons.

Existing stack components remain unchanged: `slam_toolbox`, LiDAR, Nav2,
`goal_proxy`, MediaMTX/RTSP, Foxglove bridge. We do NOT run the v3/v4 patrol
at the same time. No saved map, no second LiDAR node, no new Docker build.

## ROS 2 topics

| Topic | Type | Purpose |
|---|---|---|
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | 2–10 waypoints in `map` |
| `/robot_commands` | `std_msgs/msg/String` | `start`, `stop`, `reset`, `loop`, `random`, `undo`, `clear`, `camera_on`, `camera_off`, `check` |
| `/patrol/manual_check` | `std_msgs/msg/Bool` | Dedicated synthetic detector trigger: `data: true` |
| `/patrol/camera/image/compressed` | `sensor_msgs/msg/CompressedImage` | Annotated JPEG stream for Image panel, ~2 FPS |
| `/patrol/qr/status` | `std_msgs/msg/String` | NO QR / QR detected / GRANTED / DENIED / countdown |
| `/patrol/qr/visible` | `std_msgs/msg/Bool` | QR outline found (not necessarily decoded) |
| `/patrol/qr/value` | `std_msgs/msg/String` | Decoded QR text, blank if unreadable |
| `/patrol/state`, `/patrol/events`, `/patrol/stats`, `/patrol/markers` | as before | State, log, counters, numbered markers |

`/navigate_to_pose` is the existing Nav2 action server. The code does not
call `/goal/cancel` because the RBM proxy's service acknowledgement is not
proof of cancellation. QR checking after PATROL begins **only after Nav2
returns a canceled/succeeded action result**. This is not a physical E-stop.

## Behaviour

1. `IDLE`: RTSP camera closed; scan LiDAR map and set 2–10 points.
2. `START`: navigate between points; camera remains closed; no AI detector.
3. `CHECK` button in Lichtblick (`/patrol/manual_check` = true): cancel Nav2
   goal (if any), await Nav2 action result; no new goals during stopping.
4. `CHECKING`: connect to RTSP. Start 15-sec timeout when first frame arrives;
   publish QR-overlay JPEGs and status; play the alert sound.
5. `PERSON_001` -> `GRANTED`, camera closes. If check interrupted PATROL,
   the same waypoint is attempted again; if check began in IDLE, remain IDLE.
6. `PERSON_999` or no valid code in 15 seconds -> `DENIED`, siren loops, no
   new movement before `RESET`. `RESET` does not restart navigation.
7. `CAMERA ON` previews and highlights QR in IDLE/STOPPED without granting
   access. `CAMERA OFF` closes RTSP; preview does not start the 15-sec timer.

A camera failure is a **technical fault**, not a fake denied pass.

## Run

See [QUICKSTART_RU.md](QUICKSTART_RU.md). Inside the existing `ros` container:

```bash
cd /src/PATROL-ROBOT/rbm_patrol_v5
chmod +x *.sh
./preflight.sh
./start.sh
```

Optional tuning:

```bash
PATROL_QR_FPS=1 PATROL_QR_SCAN_WIDTH=800 PATROL_CAMERA_VIEW_WIDTH=640 ./start.sh
```

Use the included `tools/PERSON_001.png` and `tools/PERSON_999.png` to demo.

## Warning

This is classroom demonstration software. A synthetic human detection button
is **not** a safety sensor. Keep robot speed low, use a clear route, have a
human supervising movement and a physical E-stop. Never rely on a software
`STOP` button to avoid collisions with people.
