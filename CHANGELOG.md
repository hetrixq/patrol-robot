# v5 compared with v4

- Removed YOLO/NCNN/PyTorch/Ultralytics and person detection (v4 native segfault after model load).
- Replaced `STARTING` YOLO gating with immediate Nav2 patrol; camera remains OFF while moving.
- Added synthetic manual `CHECK` topic `/patrol/manual_check` (`std_msgs/msg/Bool`, `data: true`) and Lichtblick Publish button; `robot_commands: check` also supported.
- Reused robust Nav2 action cancellation logic. QR scanning starts ONLY after goal cancellation result, not proxy service ACK.
- Added single OpenCV QRCodeDetector RTSP client, annotated JPEGs via `sensor_msgs/msg/CompressedImage`.
- Added green border for decoded QR, amber for detected but undecodable; status, decoded value, visible Bool.
- Added preview controls CAMERA ON/OFF without activating access control.
- Maintained 15-second timeout, allowed PERSON_001 / denied PERSON_999, siren until RESET.
- Added 14 state-machine tests, 2 real OpenCV QR decoding tests, and 1 synthetic RTSP annotated-image test.
- Left existing RBM core services, Docker and SLAM untouched.
