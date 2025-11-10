
## Raspberry Pi에서 한글 입력/폰트 설정

라즈베리파이 OS 환경에서 한글 입력기와 폰트를 설치하고 설정하는 절차를 정리했습니다.

### 1. 시스템 업데이트

```bash
sudo apt update
sudo apt full-upgrade -y
```

### 2. 한글 폰트와 로케일 설치

```bash
sudo apt install -y fonts-nanum fonts-noto-cjk language-pack-ko
```

### 3. 한글 입력기(IBus Hangul) 설치

```bash
sudo apt install -y ibus ibus-hangul
```

### 4. 입력기 환경 변수 등록

`~/.xsessionrc` 또는 `~/.profile` 파일에 아래 내용을 추가합니다.

```bash
export GTK_IM_MODULE=ibus
export QT_IM_MODULE=ibus
export XMODIFIERS=@im=ibus
```

### 5. ibus 데몬 자동 실행 설정

필요하면 자동 실행 디렉터리를 생성합니다.

```bash
mkdir -p ~/.config/autostart
```

`~/.config/autostart/ibus.desktop` 파일을 만들고 아래 내용을 저장합니다.

```
[Desktop Entry]
Type=Application
Exec=ibus-daemon -drx
Hidden=false
NoDisplay=false
X-GNOME-Autostart-enabled=true
Name=IBus Daemon
```

### 6. 재부팅 후 입력기 추가

1. 시스템을 재부팅합니다.
   ```bash
   sudo reboot
   ```
2. 로그인 후 `ibus-setup`을 실행합니다.
3. **Input Method** 탭에서 `Add` → `Korean - Hangul`을 선택합니다.
4. 상단 패널(또는 트레이)에 입력기 아이콘이 생기면 `Ctrl + Space`로 한/영 전환이 가능합니다.

### 7. 폰트 적용 확인

- 한글 문서나 웹 페이지를 열어 글자가 정상적으로 표시되는지 확인합니다.
- 필요하면 `Preferences > Appearance` 메뉴에서 기본 폰트를 Nanum 계열 또는 Noto CJK로 설정합니다.

이 과정을 완료하면 라즈베리파이에서 한글 입력과 폰트를 문제없이 사용할 수 있습니다.
# waferhander-camera
