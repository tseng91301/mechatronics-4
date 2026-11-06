#!/bin/bash

echo "正在停止程式..."

# 優先給 ffmpeg 發送 SIGINT (相當於 Ctrl+C)，確保 output.mp4 正常封裝存檔
pkill -INT -f ffmpeg 2>/dev/null
pkill -f uvicorn 2>/dev/null
pkill -f mediamtx 2>/dev/null

sleep 1

# 檢查是否還有殘留行程
REMAINING=$(pgrep -a "ffmpeg|uvicorn|mediamtx")
if [ -n "$REMAINING" ]; then
    echo "仍有未關閉的行程，強制終止中..."
    pkill -9 -f ffmpeg 2>/dev/null
    pkill -9 -f uvicorn 2>/dev/null
    pkill -9 -f mediamtx 2>/dev/null
fi

echo "所有程式已完整關閉！"

