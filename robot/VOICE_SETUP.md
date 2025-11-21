# 🎤🔊 음성 제어 시스템 설정 가이드

## ✨ 기능

1. **🎤 음성 입력 (STT)**: Web Speech API (Chrome)
2. **🔊 음성 출력 (TTS)**: ElevenLabs
3. **🤖 LLM 이해**: GPT-4

→ **로봇과 대화하듯이 제어!**

---

## 🚀 빠른 시작

### 1. 의존성 설치

```bash
cd /Users/chloepark/Desktop/with-robot-4th/robot
pip install elevenlabs
```

### 2. API 키 설정

```bash
# OpenAI (이미 설정됨)
export OPENAI_API_KEY="sk-proj-..."

# ElevenLabs (새로 필요)
export ELEVENLABS_API_KEY="your-elevenlabs-key"
```

**ElevenLabs API 키 받기:**
1. https://elevenlabs.io 가입
2. Profile → API Keys
3. 키 복사

### 3. 서버 실행

```bash
# 완전한 음성 제어 버전
mjpython llm_voice_full.py

# 또는 (MuJoCo 없이)
python llm_voice_full.py
```

### 4. 브라우저 열기

**Chrome 브라우저 사용!**

```
http://localhost:8800/ui
```

---

## 🎬 사용 방법

### 음성으로 명령하기

1. **🎤 버튼 클릭**
2. **마이크 권한 허용**
3. **말하기**: "정사각형으로 움직여줘"
4. **로봇 응답 듣기**: "정사각형 경로를 시작합니다" (음성)
5. **자동 실행!**

### 대화 흐름

```
사용자: 🎤 "정사각형으로 움직여줘"
         ↓
      (음성 인식)
         ↓
로봇: 🤖 "정사각형 경로를 시작합니다" (음성)
         ↓
      (코드 생성)
         ↓
      (로봇 실행)
```

---

## 🎯 데모 시나리오

### 시나리오 1: 기본 음성 제어 (30초)

```
1. 🎤 버튼 클릭
2. "정사각형으로 움직여줘"
3. → 로봇: "정사각형 경로를 시작합니다" (음성)
4. → 화면에 코드 표시
5. → 로봇 실행
```

### 시나리오 2: 연속 대화 (1분)

```
사용자: 🎤 "원형으로 천천히 돌아줘"
로봇: 🔊 "원형 경로로 이동합니다"

사용자: 🎤 "삼각형 그려줘"
로봇: 🔊 "삼각형을 그리겠습니다"

사용자: 🎤 "제자리에서 회전해줘"
로봇: 🔊 "제자리에서 회전합니다"
```

### 시나리오 3: 완전한 데모 (2분)

```
1. 음성 입력 시연
   - 🎤 "정사각형으로 움직여줘"
   - 로봇 음성 응답 재생

2. 텍스트 입력 시연
   - 직접 타이핑: "원형으로 돌아줘"
   - 로봇 음성 응답 재생

3. 코드 생성 과정 설명
   - GPT-4가 만든 Python 코드 보여주기
   - ELLMER 논문과 연관성 설명
```

---

## 💡 기술 상세

### 음성 입력 (STT)
- **엔진**: Web Speech API
- **언어**: 한국어 (ko-KR)
- **브라우저**: Chrome만 지원

### 음성 출력 (TTS)
- **엔진**: ElevenLabs
- **모델**: eleven_multilingual_v2
- **언어**: 한국어, 영어 모두 지원
- **음성**: Rachel (clear, professional)

### 처리 흐름

```
사용자 음성 🎤
    ↓
텍스트 변환 (Web Speech API)
    ↓
GPT-4 코드 생성
    ↓
━━━━━━━━━━━━━━━━━━━━
↓                   ↓
로봇 응답 생성      코드 실행
↓                   ↓
ElevenLabs TTS     MuJoCo
↓                   ↓
음성 재생 🔊       로봇 움직임 🤖
```

---

## 🔧 문제 해결

### ElevenLabs 오류

**"ELEVENLABS_API_KEY not set"**
```bash
export ELEVENLABS_API_KEY="your-key-here"
```

**TTS는 동작하지만 음성 재생 안 됨**
- Chrome 브라우저 사용 확인
- 볼륨 확인
- 브라우저 콘솔에서 오디오 재생 권한 확인

### 음성 인식 오류

**"브라우저가 음성 인식을 지원하지 않습니다"**
- Chrome 브라우저로 변경

**"음성이 감지되지 않았습니다"**
- 마이크 권한 확인
- 시스템 마이크 설정 확인

---

## 📊 성능

- **STT 속도**: ~1-2초 (음성 → 텍스트)
- **LLM 속도**: ~3-5초 (GPT-4 코드 생성)
- **TTS 속도**: ~2-3초 (텍스트 → 음성)
- **총 응답**: ~6-10초

---

## 🎉 버전 비교

| 기능 | llm_main.py | llm_voice.py | llm_voice_full.py |
|------|-------------|--------------|-------------------|
| 텍스트 입력 | ✅ | ✅ | ✅ |
| 음성 입력 | ❌ | ✅ | ✅ |
| 음성 출력 | ❌ | ❌ | ✅ |
| LLM | ✅ | ✅ | ✅ |

**데모 추천: llm_voice_full.py** 🌟

---

## ✅ 준비 완료 체크리스트

- [ ] OpenAI API 키 설정
- [ ] ElevenLabs API 키 설정
- [ ] elevenlabs 패키지 설치
- [ ] Chrome 브라우저 준비
- [ ] 마이크 테스트 완료
- [ ] 스피커/헤드폰 테스트 완료

**모두 체크되면 시작!** 🚀
