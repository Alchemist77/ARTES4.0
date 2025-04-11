# ARTES4.0

## Goal

- HSV Filter to recognize objects by their color (CART4.0)  
- Volume estimation for objects (CART4.0)  
- Object recognition (CART4.0)  

---

## People Detection (Raspberry Pi 5 + Hailo + Raspberry OS)

1. Connect the Orbbec or webcam and ensure it is recognized as `/dev/video0`.  
   _If you cannot find `/dev/video0`, try reconnecting the camera or rebooting the system._

2. Open a terminal.

3. Navigate to the `hailo-rpi5-examples` directory.

4. Run the environment setup script:

   ```bash
   source setup_env.sh
   ```

5. Run the detection script:

   ```bash
   python detection.py --labels-json resources/person-labels.json --hef-path resources/yolov8s_h8l.hef --input /dev/video0
   ```

---

## Volume Detection (RealSense + ROS + Raspberry Pi with Ubuntu 20.04)

Make sure your RealSense camera is connected and recognized via ROS.

Run the volume estimation script:

```bash
python3 volume_test.py
```

---

## Color Segmentation (RealSense + ROS + Raspberry Pi with Ubuntu 20.04)

Ensure the RealSense camera is properly connected.

Run the HSV color segmentation script:

```bash
python3 color_segmentation.py
```
