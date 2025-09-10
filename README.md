# 🛗 Elevator Project – STM32F411RET6

STM32CubeIDE 기반으로 개발한 **엘리베이터 제어 시스템**입니다.  
스텝 모터, 서보 모터, 포토 커플러 센서, FND, LCD, 스위치 등을 제어하며  
층 인식 → 이동 → 도착 → 문 개폐까지의 동작을 임베디드 시스템으로 구현했습니다.

<p align="left"> 
  <a href="#-프로젝트-개요">프로젝트 개요</a> • 
  <a href="#-주요-기능">주요 기능</a> • 
  <a href="#-폴더-구조">폴더 구조</a> • 
  <a href="#-skill-stack">Skill Stack</a> • 
  <a href="#-trouble-shooting">Trouble Shooting</a> • 
  <a href="#-프로젝트-기여도">프로젝트 기여도</a> •
  <a href="#-시연-영상">시연 영상</a>
</p>

---

## 📌 프로젝트 개요
- **MCU**: STM32F411RET6 (STM32CubeIDE, HAL 라이브러리)
- **하드웨어 제어 대상**
  - 스텝 모터, 서보 모터
  - 포토 커플러 센서 (층 인식)
  - 포토레지스터 센서
  - LCD, 7-seg FND
  - 버튼 스위치
- **개발 환경**: STM32CubeIDE, .ioc 파일 기반 설정, C 언어
- **핵심 목표**
  - MCU를 중심으로 다양한 주변 장치 제어
  - 상태 기반 제어(State Machine) 학습
  - 인터럽트 기반 이벤트 처리
  - 실제 하드웨어 제약(신호 간섭, 디바운싱 등) 문제 해결 경험

---

## ✨ 주요 기능
- **층 인식** : 포토레지스터 센서를 이용해 현재 층을 감지
- **이동 로직** :
  1. 전원 ON → 초기화 → 문 닫힘 상태 확인 후 시작
  2. 포토레지스터로 현재 층 감지 및 버튼 입력 확인
  3. 입력된 층에서 정지 후 문 개폐 (Door Open / Close)
- **안전 조건** : 운행 중에는 문이 반드시 닫힌 상태 유지
- **핵심 기능** : 정확한 층 인식 및 도착 시 정지 동작 구현

---

## 📂 폴더 구조
```
elevator_project/
├─ Core/
│ ├─ Inc/
│ │ ├─ main.h
│ │ └─ stm32f4xx_hal_conf.h
│ ├─ Src/
│ │ ├─ main.c
│ │ └─ stm32f4xx_it.c
├─ Drivers/
├─ elevator.ioc # STM32CubeIDE 프로젝트 설정 파일
├─ Makefile
└─ README.md
```

---

## 🛠 Skill Stack

![STM32](https://img.shields.io/badge/STM32-03234B?style=flat&logo=stmicroelectronics&logoColor=white)
![C](https://img.shields.io/badge/C-00599C?style=flat&logo=c&logoColor=white)
![CubeIDE](https://img.shields.io/badge/STM32CubeIDE-1E90FF?style=flat&logo=stmicroelectronics&logoColor=white)
![Git](https://img.shields.io/badge/Git-F05032?style=flat&logo=git&logoColor=white)
![GitHub](https://img.shields.io/badge/GitHub-181717?style=flat&logo=github&logoColor=white)

---

## 🔧 Trouble Shooting

- **문제 1: 버튼 입력 중복 인식**
  - 빠른 입력 시 MCU가 신호를 여러 번 인식 → 중복 동작 발생
  - ✅ 해결: 소프트웨어 디바운싱 적용 → 일정 시간 내 중복 입력 무시

- **문제 2: 핀 매핑 오류**
  - 초기 PIN MAP 설계 잘못으로 일부 장치가 충돌
  - ✅ 해결: 포트 구성 재검토 및 모듈화하여 충돌 방지

- **문제 3: 전원 OFF 후 상태 손실**
  - 재시작 시 직전 층 정보 소실
  - ✅ 해결: 전원 ON 시 초기 하향 이동 후 기준 층 인식 절차 추가

---

## 📊 프로젝트 기여도
- 프로젝트 기획  : ████████████░░░ 80%  
- 코딩         : █████░░░░░░░░░░ 40%  
- PPT 제작     : ██░░░░░░░░░░░░░ 10%  
- 하드웨어 제작  : ███████████████ 100%  

---

## 🎥 시연 영상

[YouTube에서 보기](https://youtube.com/shorts/E30i3ZkvgvY?si=ayuA2xYGPYM4vr73)
