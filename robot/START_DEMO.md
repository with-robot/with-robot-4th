# 🚀 데모 시작하기 (2분 가이드)

## ⚡ 빠른 시작 (3단계)

### 1단계: 포트 정리 (기존 프로세스 종료)
```bash
cd /Users/chloepark/Desktop/with-robot-4th/robot
./kill_port.sh
```

또는 수동으로:
```bash
lsof -ti:8800 | xargs kill -9
```

### 2단계: 서버 시작
```bash
python llm_main.py
```

**예상 출력:**
```
⚠️  WARNING: MuJoCo viewer requires mjpython on macOS
   Simulator disabled - API server running without 3D viewer

============================================================
🤖 LLM-Enabled MuJoCo Robot Simulator
============================================================
Server: http://0.0.0.0:8800
Web UI: http://0.0.0.0:8800/ui
============================================================
```

> **Note**: 3D 뷰어 경고는 정상입니다!
> 데모에서는 **코드 생성**이 중요하므로 문제없습니다.

### 3단계: 브라우저 열기
```
http://localhost:8800/ui
```

---

## 🎯 데모 모드 (시뮬레이터 없이)

**장점:**
- ✅ 코드 생성 즉시 확인
- ✅ 빠른 반응 속도
- ✅ 포트 충돌 없음
- ✅ 웹 UI 사용 가능

**데모 흐름:**
1. 웹 UI에서 명령 입력
2. GPT-4가 코드 생성 → 화면에 표시
3. 생성된 코드 설명
4. (선택) 실제 로봇 실행은 별도 데모

---

## 🔧 문제 해결

### 문제: "Address already in use"
```bash
# 포트 정리 스크립트 실행
./kill_port.sh

# 또는 수동으로 프로세스 종료
lsof -ti:8800 | xargs kill -9
```

### 문제: "mjpython required"
- **정상입니다!** 데모는 웹 UI만 사용
- 3D 뷰어 필요 시: `mjpython llm_main.py`

### 문제: OpenAI API 오류
```bash
# API 키 확인
echo $OPENAI_API_KEY

# 설정되지 않았다면
export OPENAI_API_KEY="sk-proj-..."
```

---

## 📋 데모 체크리스트

실행 전 확인:
- [ ] 포트 8800 정리됨 (`./kill_port.sh`)
- [ ] 서버 실행 중 (`python llm_main.py`)
- [ ] 브라우저에서 접속됨 (`localhost:8800/ui`)
- [ ] 예제 명령 테스트 완료

---

## 💡 데모 팁

### 강조할 포인트
1. **자연어 → 코드**: GPT-4 자동 변환 과정
2. **실시간 생성**: 생성된 코드를 즉시 확인
3. **다양한 패턴**: 정사각형, 원, 삼각형 등
4. **확장 가능**: 실제 로봇에도 적용 가능

### 예제 명령어
```
✅ "정사각형으로 움직여줘"
✅ "2미터 크기로 원형으로 돌아줘"
✅ "삼각형을 그린 다음 원점으로 복귀해줘"
✅ "Move in a square pattern"
```

---

## 🎬 발표 시나리오

### Option 1: 웹 UI 중심 (추천)
1. 웹 UI 시연
2. 자연어 입력 → 코드 생성 과정 설명
3. 생성된 코드 분석
4. ELLMER 논문과의 연관성 설명

### Option 2: 전체 시스템
1. 웹 UI로 코드 생성
2. 별도 터미널에서 실제 로봇 실행
   ```bash
   # 다른 터미널에서
   mjpython llm_main.py
   ```

---

## ✅ 준비 완료!

모든 시스템이 정상입니다. **파이팅!** 🚀
