<div align="center">

# AddamNet

**Autonomous defect detection and 3D mapping inside underground waste pipes.**<br>
The sensing and intelligence stack for an unmanned robot that inspects Singapore's Pneumatic
Waste Conveyance System (PWCS) pipes: embedded sensing, YOLOv8 defect detection, monocular visual
SLAM and a live dashboard, all on a Raspberry Pi 5. My B.Eng. dissertation at NUS, graded A.

**[▶ Live 3D reconstruction viewer](https://sp4ce-cowboy.github.io/AddamNet/slam-viewer/)**

[![Python](https://img.shields.io/badge/Python-3-3776AB?logo=python&logoColor=white)](https://github.com/sp4ce-cowboy/AddamNet/blob/7c47efae85a2158621a294750ad775ef63fd990b/slam/visual_slam.py)
[![OpenCV](https://img.shields.io/badge/OpenCV-4-5C3EE8?logo=opencv&logoColor=white)](https://github.com/sp4ce-cowboy/AddamNet/blob/7c47efae85a2158621a294750ad775ef63fd990b/slam/visual_slam.py#L199-L243)
[![YOLOv8](https://img.shields.io/badge/YOLOv8n-Ultralytics-111F68)](ARCHITECTURE.md#4-defect-detection-yolov8)
[![Arduino](https://img.shields.io/badge/Arduino-Mega%202560-00878F?logo=arduino&logoColor=white)](https://github.com/sp4ce-cowboy/AddamNet/blob/7c47efae85a2158621a294750ad775ef63fd990b/arduino/sensor_suite.ino)
[![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-5-A22846?logo=raspberrypi&logoColor=white)](ARCHITECTURE.md#2-system-architecture)
[![NumPy](https://img.shields.io/badge/NumPy%20%C2%B7%20SciPy-3D%20reconstruction-013243?logo=numpy&logoColor=white)](https://github.com/sp4ce-cowboy/AddamNet/blob/7c47efae85a2158621a294750ad775ef63fd990b/slam/reconstruct_viewer.py#L69-L195)
[![Three.js](https://img.shields.io/badge/Three.js-r128%20viewer-000000?logo=threedotjs&logoColor=white)](https://sp4ce-cowboy.github.io/AddamNet/slam-viewer/)
[![FastAPI](https://img.shields.io/badge/FastAPI-dashboard-009688?logo=fastapi&logoColor=white)](https://fastapi.tiangolo.com)

![Grade](https://img.shields.io/badge/dissertation-grade%20A-2ea44f?style=flat-square)
![Dataset](https://img.shields.io/badge/training%20data-22k%20labelled%20images-555?style=flat-square)
![Sensors](https://img.shields.io/badge/sensors-6-555?style=flat-square)
![Report](https://img.shields.io/badge/report-59%20pages-555?style=flat-square)

</div>

> [!NOTE]
> This consolidated README page was added by [Claude](https://docs.anthropic.com/en/docs/claude-code)
> using my [agentic-repo-analysis](https://github.com/sp4ce-cowboy/agentic-workflows) skill, from the
> project's code, git history and my dissertation. The original README, the dashboard's set-up guide,
> is in **[DOCUMENTATION.md](DOCUMENTATION.md)**. The technical write-up is in **[ARCHITECTURE.md](ARCHITECTURE.md)**.

<p align="center">
  <img width="88%" alt="The inspection dashboard: live YOLO defect detection beside monocular SLAM tracking" src="docs/images/dashboard-live.png" />
</p>

## At a glance

| | |
|---|---|
| **What** | The sensing and intelligence stack for an unmanned PWCS pipe-inspection robot: a six-sensor embedded system, AI defect detection, 3D mapping of the pipe interior, and a live dashboard, running on low-cost hardware with no internet |
| **Context** | NUS ME4101A Bachelor of Engineering Dissertation, AY2025/26, supervised by Prof. Chew Chee Meng. Graded A |
| **My role** | Individual project: requirements, hardware, Arduino firmware, dataset and model training, the SLAM and 3D-reconstruction pipeline, system integration, and the 59-page report |
| **Domains** | Embedded systems · computer vision · machine learning · robotics (SLAM) · web |
| **AI use** | As acknowledged in the dissertation, the dashboard prototype (`app/`) was generated with Claude Opus 4.6; the firmware, model, SLAM and reconstruction code are mine |

## The problem

Singapore's public housing estates move refuse through buried PWCS pipes, using high-speed air
suction. The pipes last about 30 years before costly replacement, and inspecting them today means
workers entering confined spaces, after something has already gone wrong. This project is part of
a wider effort to extend pipe life from 30 to 45 years by finding defects early, with a robot
instead of a person.

## What it does

<p align="center">
  <img width="48%" alt="The embedded system: webcam, Raspberry Pi 5, Arduino Mega and the sensor suite" src="docs/images/hardware.jpg" />
  <img width="48%" alt="System architecture from sensors to the dashboard" src="docs/images/system-architecture.png" />
</p>
<p align="center"><sub>The embedded system as built (1 webcam · 2 Raspberry Pi 5 · 3 Arduino Mega 2560 · 4 breadboard · 5 ultrasonic · 6 IMU · 7 IR obstacle · 8 temperature/humidity · 9 keypad) and its architecture</sub></p>

- **Senses the pipe.** An Arduino Mega reads six sensors: an IMU, ultrasonic and infrared distance,
  an IR obstacle sensor, gas (air quality), and temperature and humidity. It streams JSON packets to
  the Pi, including orientation, speed, distance travelled and a tether warning.
- **Finds defects.** A YOLOv8n model trained on about 22,000 labelled images detects six defect
  classes (deformation, obstacle, rupture, disconnect, misalignment, deposition) on the Pi's CPU.
- **Maps the pipe in 3D.** A monocular visual SLAM pipeline tracks the camera and builds a sparse 3D
  map. A reconstruction step turns the map into a surface mesh and an interactive 3D viewer.
- **Shows it all live.** A five-tab web dashboard shows the annotated video, telemetry, a defect log
  laid along the pipe, the 3D reconstruction, and system health.

<p align="center">
  <img width="48%" alt="Interactive SLAM 3D viewer: point cloud, mesh and camera trajectory" src="docs/images/slam-viewer.png" />
  <img width="48%" alt="Live sensor telemetry" src="docs/images/dashboard-sensors.png" />
</p>
<p align="center">
  <img width="80%" alt="Defects laid out along a linear pipeline schematic" src="docs/images/dashboard-defects.png" />
</p>
<p align="center"><sub>The 3D viewer (2,334 points, 24,025 faces, 254 camera poses; <a href="https://sp4ce-cowboy.github.io/AddamNet/slam-viewer/">try it live</a>) · live telemetry · defects along the pipe. Dashboard captures from a sample pipe video, with the sensors simulated</sub></p>

## Skills shown

| Skill | Where it shows (click the links for the code) | Docs |
|---|---|---|
| **Embedded firmware**<br><img alt="Arduino" src="https://img.shields.io/badge/Arduino-C%2B%2B-00878F?logo=arduino&logoColor=white"> | Six-sensor firmware with JSON output and per-sensor enable switches: [`sensor_suite.ino`](https://github.com/sp4ce-cowboy/AddamNet/blob/7c47efae85a2158621a294750ad775ef63fd990b/arduino/sensor_suite.ino); IMU bias calibration over 500 samples: [`mpuCalibrate`](https://github.com/sp4ce-cowboy/AddamNet/blob/7c47efae85a2158621a294750ad775ef63fd990b/arduino/sensor_suite.ino#L256-L305) | [Arduino](https://docs.arduino.cc) |
| **Visual SLAM**<br><img alt="OpenCV" src="https://img.shields.io/badge/OpenCV-4-5C3EE8?logo=opencv&logoColor=white"> | ORB/FAST features, KLT optical-flow tracking, RANSAC essential-matrix pose recovery: [`track_klt`, `estimate_pose`](https://github.com/sp4ce-cowboy/AddamNet/blob/7c47efae85a2158621a294750ad775ef63fd990b/slam/visual_slam.py#L175-L243); triangulation and reprojection filtering: [`triangulate_points`](https://github.com/sp4ce-cowboy/AddamNet/blob/7c47efae85a2158621a294750ad775ef63fd990b/slam/visual_slam.py#L86-L128) | [OpenCV calib3d](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html) |
| **3D surface reconstruction**<br><img alt="SciPy" src="https://img.shields.io/badge/SciPy-KDTree%20%C2%B7%20Delaunay-8CAAE6?logo=scipy&logoColor=white"> | Statistical outlier removal, normal estimation and alpha-shape meshing of the point cloud: [`reconstruct_viewer.py`](https://github.com/sp4ce-cowboy/AddamNet/blob/7c47efae85a2158621a294750ad775ef63fd990b/slam/reconstruct_viewer.py#L69-L195) | [scipy.spatial](https://docs.scipy.org/doc/scipy/reference/spatial.html) |
| **Object detection**<br><img alt="YOLOv8" src="https://img.shields.io/badge/YOLOv8n-Ultralytics-111F68"> | Transfer learning from COCO weights on six defect classes; the trained model: [`weights/`](https://github.com/sp4ce-cowboy/AddamNet/tree/7c47efae85a2158621a294750ad775ef63fd990b/weights) | [Ultralytics](https://docs.ultralytics.com) |
| **Edge deployment**<br><img alt="Raspberry Pi" src="https://img.shields.io/badge/Raspberry%20Pi-5-A22846?logo=raspberrypi&logoColor=white"> | CPU-only inference at 2–5 fps with no internet or accelerator; serial link at 115,200 baud: [`config.yaml`](https://github.com/sp4ce-cowboy/AddamNet/blob/7c47efae85a2158621a294750ad775ef63fd990b/config.yaml) | [Raspberry Pi](https://www.raspberrypi.com/documentation/) |
| **Interactive 3D on the web**<br><img alt="Three.js" src="https://img.shields.io/badge/Three.js-r128-000000?logo=threedotjs&logoColor=white"> | The generated viewer: point cloud, mesh, trajectory, colour modes: [live demo](https://sp4ce-cowboy.github.io/AddamNet/slam-viewer/) | [Three.js](https://threejs.org/docs/) |
| **System integration** | Firmware, model and SLAM wired into one FastAPI and WebSocket dashboard (the dashboard prototype generated with Claude): [`app/`](https://github.com/sp4ce-cowboy/AddamNet/tree/7c47efae85a2158621a294750ad775ef63fd990b/app) | [FastAPI](https://fastapi.tiangolo.com) |
| **Research & technical writing** | A 59-page dissertation: literature, design, evaluation, limitations and future work | — |

## Results

- **End to end on low-cost hardware.** Sensing, detection, mapping and the dashboard all run
  together on a Raspberry Pi 5, with no internet and no AI accelerator.
- **IMU calibration.** A start-up routine averaging 500 stationary readings removed a gyroscope bias
  of about −3.4 °/s on one axis, which would have caused over 200° of orientation error a minute.
  Residual drift came down to under 0.5 °/s.
- **Detection.** Runs at 2–5 fps on the Pi's CPU. The reported model is a single-epoch baseline
  (mAP@0.5 of 0.24), so longer training is the first next step.
- **Mapping.** Produces a camera trajectory and a sparse 3D map of the pipe interior. Monocular SLAM
  is scale-ambiguous and has no loop closure yet.

**Next steps (from the dissertation):** longer training, an RGB-D camera (Intel RealSense D405) for
metric dense mapping, a Hailo-8 or Jetson accelerator, and placing each detection on the 3D map.

## Getting started

See [DOCUMENTATION.md](DOCUMENTATION.md) for the full set-up on a Raspberry Pi 5. On a laptop, set
`video.source: "file"` in `config.yaml` with a sample video. Sensors fall back to simulated data
when no Arduino is connected. Then run `python3 run.py` and open `http://127.0.0.1:8000`.

## On my resume

- Designed and built the sensing and intelligence stack for an unmanned robot inspecting Singapore's pneumatic waste conveyance pipes (B.Eng. dissertation, graded A): a two-tier Raspberry Pi 5 and Arduino Mega platform, a six-sensor suite, YOLOv8 defect detection, monocular visual SLAM and a live web dashboard.
- Wrote Arduino firmware for six sensors (IMU, ultrasonic and infrared ranging, IR obstacle, gas, temperature and humidity) streaming JSON over a 115,200-baud serial link. A 500-sample start-up calibration cut gyroscope drift from about 3.4 °/s to under 0.5 °/s.
- Implemented a monocular visual SLAM pipeline in Python and OpenCV: ORB/FAST features with KLT optical-flow tracking, RANSAC essential-matrix pose recovery, and triangulation with reprojection filtering. It produces a camera trajectory and a sparse 3D map of the pipe interior.
- Built a surface-reconstruction step that removes outliers, estimates normals and alpha-shape meshes the SLAM point cloud (NumPy, SciPy), and exports it as an interactive Three.js viewer, live on GitHub Pages.
- Trained a YOLOv8n detector by transfer learning on about 22,000 labelled images across six defect classes, chosen for CPU-only inference on the Raspberry Pi (2–5 fps), and integrated the firmware, model and SLAM into a FastAPI and WebSocket dashboard.
