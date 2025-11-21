#!/bin/bash
# Kill process using port 8800

echo "🔍 Checking port 8800..."

PID=$(lsof -ti:8800)

if [ -z "$PID" ]; then
    echo "✅ Port 8800 is free!"
else
    echo "🚨 Port 8800 is in use by process $PID"
    echo "🔪 Killing process..."
    kill -9 $PID
    sleep 1
    echo "✅ Port 8800 is now free!"
fi

echo ""
echo "Ready to start server!"
echo "Run: python llm_main.py"
