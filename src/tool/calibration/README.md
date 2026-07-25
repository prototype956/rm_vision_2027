# Calibration Tools (Scaffold v1)

## Overview

This folder contains the first usable scaffold for EX calibration workflow:

- `mv-calib-capture`: capture images and write matching quaternion files (`i.txt`, `w x y z`)
- `mv-calib-camera`: calibrate camera intrinsics (`camera_matrix`, `distort_coeffs`)
- `mv-calib-rwhandeye`: run RobotWorld-HandEye and write extrinsics (`R_camera_to_gimbal`, `t_camera_to_gimbal`)
- `mv-calib-validate`: validate calibration quality (`det(R)`, orthogonality, reprojection stats)

All write-back operations update only `calibration` fields in `src/config/vision.yaml`.

## Build

```bash
cmake -S . -B build
cmake --build build --target mv-calib-capture mv-calib-camera mv-calib-rwhandeye mv-calib-validate -j
```

## Typical Workflow

1. Capture frames + quaternions (serial realtime pose sync)

```bash
./build/bin/mv-calib-capture \
  --source-type camera \
  --camera-id 0 \
  --pose-source serial \
  --serial-config src/config/vision.yaml \
  --serial-node serial \
  --serial-max-age-ms 30 \
  --output-folder data/calib_capture \
  --pattern-type chessboard \
  --cols 10 --rows 7 --spacing-mm 40.0
```

Optional offline quaternion feed:

```bash
./build/bin/mv-calib-capture \
  --source-type video \
  --video-path video.mp4 \
  --pose-source file \
  --quat-file poses.txt \
  --output-folder data/calib_capture
```

2. Calibrate intrinsics

```bash
./build/bin/mv-calib-camera \
  data/calib_capture \
  --vision-path src/config/vision.yaml \
  --pattern-type chessboard \
  --cols 10 --rows 7 --spacing-mm 40.0 \
  --write=true
```

3. Run RobotWorld-HandEye

```bash
./build/bin/mv-calib-rwhandeye \
  data/calib_capture \
  --vision-path src/config/vision.yaml \
  --pattern-type chessboard \
  --cols 10 --rows 7 --spacing-mm 40.0 \
  --write=true
```

4. Validate calibration result

```bash
./build/bin/mv-calib-validate \
  data/calib_capture \
  --vision-path src/config/vision.yaml \
  --pattern-type chessboard \
  --cols 10 --rows 7 --spacing-mm 40.0 \
  --max-reproj-px 1.5 \
  --det-eps 0.05 \
  --ortho-eps 0.05
```

## Notes

- Quaternion file format: one sample per line, `w x y z`.
- `pose-source=serial` parses MCU up frames (`0xAA 0xFF ...`) and syncs nearest quaternion by frame timestamp.
- Translation output from robot-world-handeye is converted from mm to m before write-back.
