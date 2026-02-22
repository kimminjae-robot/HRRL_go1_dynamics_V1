# HRRLab 4족 보행 로봇 동역학 보행 알고리즘 (Version 1)

본 프로젝트는 4족 보행 로봇의 동역학 기반 보행을 구현하기 위한 알고리즘 모음입니다. 

현재 기본적인 동작이 가능한 수준으로 구현되어 있으며, 실제 환경 및 특정 하드웨어 적용을 위해 다양한 파라미터 튜닝이 권장됩니다.

🛠 시스템 환경 (System Environment)
Physics Simulator: MuJoCo
QP Solver: qpSWIFT

🚀 주요 구현 기능
1. 상태 추정 및 제어 (Estimation & Control)
칼만필터 선속도 추정기
WBC (Whole Body Control)
Task Space / CTC (Computed Torque Control)
칼만필터 외란 관측기 (Disturbance Observer)

2. 지형 적응 및 안전 (Terrain Adaptation & Safety)
나이브 베이즈 발끝 슬립 확률 계산
경사 추정

3. 유틸리티 및 통신 (Utility & Communication)
동역학/기하학 계산기
TCP 데이터 통신
파이썬 시각화

## 실행 방법

### 빌드 (프로젝트 루트에서)
rm -rf build
mkdir -p build
cd build

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

### 실행
cd build 
./test ../../model/scene.xml

