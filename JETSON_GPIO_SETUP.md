# Jetson Orin NX GPIO 제어 가이드

## 1단계: Jetson.GPIO 라이브러리 설치

Jetson Orin NX에서 GPIO 핀을 제어하려면 `Jetson.GPIO` 라이브러리를 설치해야 합니다.

### 설치 방법

터미널(VSCode의 터미널 또는 SSH로 접속한 터미널)에서 다음 명령어를 실행하세요:

```bash
# pip 업그레이드 (선택사항이지만 권장)
pip3 install --upgrade pip

# Jetson.GPIO 라이브러리 설치
pip3 install Jetson.GPIO

# 설치 확인
python3 -c "import Jetson.GPIO as GPIO; print('Jetson.GPIO 설치 완료!')"
```

**주의사항:**
- `pip` 대신 `pip3`를 사용하세요 (Python 3)
- `sudo` 권한이 필요할 수 있습니다. 필요시 `sudo pip3 install Jetson.GPIO` 실행

---

## 2단계: GPIO 그룹 추가 (권한 설정)

GPIO를 사용하려면 현재 사용자를 `gpio` 그룹에 추가해야 합니다:

```bash
# 현재 사용자를 gpio 그룹에 추가
sudo usermod -a -G gpio $USER

# 변경사항 적용을 위해 로그아웃 후 다시 로그인하거나
# 또는 다음 명령어로 그룹 변경사항 적용
newgrp gpio
```

**주의:** 그룹 변경 후에는 VSCode를 재시작하거나 새 터미널을 열어야 합니다.

---

## 3단계: Jetson Orin NX 핀 번호 확인

Jetson Orin NX의 GPIO 핀 번호는 **BCM(Broadcom) 번호** 또는 **BOARD 번호**로 지정할 수 있습니다.

### 주요 GPIO 핀 (BCM 번호 기준)
- GPIO 18 (BCM 18) - 핀 12
- GPIO 23 (BCM 23) - 핀 16
- GPIO 24 (BCM 24) - 핀 18
- GPIO 25 (BCM 25) - 핀 22
- GPIO 12 (BCM 12) - 핀 32
- GPIO 16 (BCM 16) - 핀 36

**전체 핀맵은** `/sys/kernel/debug/gpio` 또는 Jetson GPIO 문서를 참조하세요.

---

## 4단계: 기본 사용법

Jetson.GPIO는 Raspberry Pi의 RPi.GPIO와 유사한 API를 제공합니다.

### 핵심 개념
- **BCM 모드**: Broadcom 핀 번호 사용 (논리적 번호)
- **BOARD 모드**: 물리적 핀 번호 사용 (40핀 헤더 기준)
- **IN/OUT**: 입력/출력 모드 설정
- **HIGH/LOW**: 3.3V / 0V 신호

---

## 5단계: 예제 코드 실행

프로젝트에 포함된 `gpio_example.py` 파일을 실행해보세요.

```bash
python3 gpio_example.py
```

---

## 문제 해결

### 권한 오류가 발생하는 경우
```bash
# gpio 그룹 확인
groups

# gpio 그룹이 없다면 다시 추가
sudo usermod -a -G gpio $USER
# 이후 로그아웃/재로그인 또는 newgrp gpio 실행
```

### 라이브러리를 찾을 수 없는 경우
```bash
# Python 경로 확인
which python3
python3 --version

# pip 경로 확인
which pip3

# Jetson.GPIO 재설치
pip3 uninstall Jetson.GPIO
pip3 install Jetson.GPIO
```

### GPIO 핀을 찾을 수 없는 경우
- Jetson Orin NX의 GPIO 핀은 제한적입니다
- `/sys/kernel/debug/gpio`에서 사용 가능한 GPIO 확인
- Jetson Hobbyist Wiki를 참조하세요

---

## 참고 자료
- [Jetson.GPIO GitHub](https://github.com/NVIDIA/jetson-gpio)
- [Jetson GPIO 문서](https://github.com/NVIDIA/jetson-gpio/blob/master/README.md)
- Jetson Orin NX Developer Kit 사용자 가이드

