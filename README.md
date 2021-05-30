# panda_fixed_eye_calibration
로봇과 고정된 카메라 사이의 칼리브레이션 프로그램입니다.

# Simulation 프로그램
```bash
  python capture_calibration.py
```
space키를 누르면 이미지 데이터가 matlab_files/Images에 저장됩니다.
space키를 누르면 pose 데이터가 matlab_files/Poses에 저장됩니다.
enter키를 누르면 저장이 완료되고 프로그램이 종료됩니다.

# Matlab Calibration 프로그램
```bash
  cd matlab_files/robotcalibration.m
```
# 실제 로봇구동
moveit을 이용한 로봇구동 후
cam.py를 이용하여 이미지와 자세정보를 저장합니다.
space를 누르면 이미지와 자세정보가 Images와 Poses폴더에 저장됩니다.
esc를 누르면 프로그램이 종료됩니다.
```bash
  roslaunch panda_moveit_config demo.launch
  cd panda_fixed_eye_calibration/realsense_camera
  python cam.py
```

# 저장된 Images와 Poses으로 Calibration
다른 터미널을 키고 도커 실행
```bash
  git clone https://github.com/tjdalsckd/calibration_docker
  cd calibration_docker
  bash build.sh
  bash start.sh
```
도커 외부에서  파일을 복사 

```bash
    cd panda_fixed_eye_calibration/realsesne_camera
    docker cp  Images calibration_smc:/root/workspace/calibration_application/application/
    docker cp  Poses calibration_smc:/root/workspace/calibration_application/application/
```

도커 내부에서 calibration을 수행
```bash
./calibration_smc
```
