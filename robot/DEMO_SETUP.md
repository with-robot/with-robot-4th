# 🚀 내일 데모 준비 가이드

## ⚡ 빠른 설정 (5분)

### 1. OpenAI API 키 발급
```bash
# https://platform.openai.com/api-keys 에서 API 키 발급
# 환경변수로 설정:
export OPENAI_API_KEY="your-openai-api-key-here"
```

### 2. 의존성 설치
```bash
cd /Users/chloepark/Desktop/with-robot-4th/robot
pip install -r requirements_llm.txt
```

### 3. 서버 실행
```bash
python llm_main.py
```

서버가 시작되면:
- **웹 UI**: http://localhost:8800/ui
- **API 문서**: http://localhost:8800/docs

---

## 🎬 데모 시나리오

### 시나리오 1: 기본 도형 그리기
```
입력: "정사각형으로 움직여줘"
결과: 로봇이 1x1m 정사각형 경로로 이동
```

### 시나리오 2: 크기 지정
```
입력: "2미터 크기로 원형으로 돌아줘"
결과: 반지름 2m 원형 경로 이동
```

### 시나리오 3: 복잡한 명령
```
입력: "삼각형을 그린 다음 제자리에서 한 바퀴 회전해줘"
결과: 삼각형 경로 → 360도 회전
```

### 시나리오 4: 한국어/영어 혼용
```
입력: "Move in a square pattern, then return home"
결과: 정사각형 → 원점 복귀
```

---

## 🔧 문제 해결

### OpenAI API 키 오류
```bash
# 키가 설정되었는지 확인:
echo $OPENAI_API_KEY

# 없으면 설정:
export OPENAI_API_KEY="your-key-here"
```

### 포트 충돌
```bash
# 다른 포트 사용:
# llm_main.py에서 PORT = 8801로 변경
```

### MuJoCo 창이 안 뜨는 경우
- 정상입니다! 백그라운드에서 시뮬레이터가 돌아가고 있습니다
- 별도로 실행하려면: `python -c "from simulator import MujocoSimulator; s = MujocoSimulator(); s.run()"`

---

## 📊 데모 순서 (추천)

1. **웹 UI 열기**: http://localhost:8800/ui
2. **간단한 명령 테스트**: "정사각형으로 움직여줘"
3. **생성된 코드 확인**: 화면에 표시된 Python 코드 설명
4. **복잡한 명령**: "삼각형을 그린 다음 원형으로 돌아줘"
5. **실시간 수정**: 크기나 속도 조절 요청

---

## 💡 프레젠테이션 포인트

1. **자연어 → 코드 변환**: GPT-4가 자동으로 Python 코드 생성
2. **Zero-shot Learning**: 학습 없이 다양한 패턴 실행 가능
3. **Few-shot Prompting**: 시스템 프롬프트에 예제를 포함해 RAG와 유사한 효과
4. **실시간 피드백**: 로봇 상태를 print문으로 확인
5. **확장 가능성**: Vision, Force feedback 추가 가능

---

## 🎯 핵심 메시지

"기존 로봇 제어 시스템에 LLM을 통합하여,
자연어 명령만으로 복잡한 로봇 모션을 실행할 수 있습니다.
ELLMER 논문의 핵심 아이디어를 최소 구현으로 시연합니다."
