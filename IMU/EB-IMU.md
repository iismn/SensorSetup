IMU Serial 사용법

1. EB-IMU UART 연결
2. ROS - Serial Source Git 설치 (Catkin_Make)
3. pip install pyserial (pyserial 설치)
4. dmesg | greptty 명령창 입력후 연결된 포트 확인

* Permission Denied 시
- ls -al /dev/ttyUSB* 로 dialout 그룹 확인
- id 입력후 dialout 그룹 속해있는지 확인
- 그룹에 속해있지 않을시 sudo usermod -a -G dialout iismn
