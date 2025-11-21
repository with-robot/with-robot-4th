#!/bin/bash
# 음성 제어 데모 시작 스크립트

echo "🎤🔊 음성 제어 로봇 데모 시작"
echo "="*60

# API 키 설정
export OPENAI_API_KEY="$OPENAI_API_KEY"
export ELEVENLABS_API_KEY="sk_4a5b120d71042d94bc42b9aa5e58d9acdd2eaa6be3e6f334"

echo "✅ API Keys configured"
echo ""

# 포트 정리
echo "🔧 Cleaning port 8800..."
lsof -ti:8800 | xargs kill -9 2>/dev/null
echo "✅ Port ready"
echo ""

# static 디렉토리 생성
mkdir -p static
echo "✅ Static directory ready"
echo ""

echo "🚀 Starting voice-controlled robot server..."
echo ""
echo "Open in Chrome browser:"
echo "  → http://localhost:8800/ui"
echo ""
echo "="*60
echo ""

# 서버 시작
mjpython llm_voice_full.py
