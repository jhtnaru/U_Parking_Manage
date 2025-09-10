# 지하주차장 재난대응 System 🚗🔥

**지하탈출넘버원 / 2025.09.01 / 경기인력개발원 Semicon 설계 검증**

---

## 📌 프로젝트 개요

아파트 및 대형 건물의 **지하주차장**은 구조적 특성상 사고 발생시 **인지와 대피가 어렵습니다.**\
하지만 기후 변화와 전기차 보급 확산 등으로 재난 위험성은 더욱 증가하고 있어\
지하주차장 재난, 사고에 대한 **사전 감지 → 경고 및 피난 유도 → 대응**이 필요합니다.

이에 따라 **Basys3 FPGA 보드, Verilog**를 기반으로 **지하주차장 재난 대응 시스템**을 구현하였습니다.

---

## 🎯 주요 기능

-   **다양한 센서 기반 상황 감지**
    -   침수, 화재, 지진, 폭염, 암전, 굉음, 가스누출, 동작감지
-   **상황 판단 및 등급 분류**
    -   FSM 방식으로 상황과 등급(경고, 위험) 분류
-   **재난 대응 설비 제어**
    -   차수판, 자동 사다리, 제연 및 환기
-   **경고 및 피난 유도**
    -   LCD, LED, Buzzer를 통한 상황 알림 및 대피 유도

---

## 🛠 기술 스택

-   **Hardware**: Basys3 FPGA, 각종 센서 & 모터 (Servo, Stepper, Fan 등)
-   **언어/도구**: Verilog, Vivado (Linux 환경)
-   **출력 장치**: LCD, LED, Buzzer

---

## ⚙️ 사용 부품

-   **센서류**: Water, Flame, 온습도, Photo, Sound, Gas, Motion, Ball Switch
-   **구동부**: Servo Motor, Stepper Motor, Fan Motor
-   **출력부**: LCD, LED, Buzzer

---

## 📊 시스템 구조

-   **센서 → FSM 기반 판단 → 상황 및 등급 분류 → 출력 및 제어(차수판, 사다리 등)**

---

## 📽 시연 영상 및 💻 Code

-   [시연 영상 1](https://youtu.be/Tw248NSMQMI?si=36S2efTuJ7Tr04_r)
-   [시연 영상 2](https://youtu.be/YJAre8ZWxm8?si=r-h7S7e3ir5AUFSH)
-   [Code(Notion)](https://junaru.notion.site/System-Code-25c571106f87805fb0c0c3ad1cbd0c68?source=copy_link)

---

## 🚧 문제 해결 & 개선 사항

**문제 해결**
- LCD 출력 문제 → Count & 배열 이용 순차 전송
- 전력 부족 → Transistor 회로로 Fan, Buzzer 동작 보완
- 센서 감지 불안정 → 개별 Test 및 Debugging

**개선 사항**
- 대응 장비 확장 (배수, 살수, 비상지지대 등)
- 경보 조건 세분화 (시간 초과, 중복 감지)
- UART 통신을 통한 상황 전달 및 로그 기록

---

## ✅ 결론

-   Basys3 + Verilog 기반 **FSM 설계 및 구현 경험**
-   조건 기반 상태 변화 제어 및 출력 설계
-   주제 선정 → 회로 구현 → 실물 제작 → 개선 아이디어까지 학습

---

## 📈 Flow Chart
<p align="Left">
  <img src="UParkingManage_FlowChart.png" width="70%" />
</p>
