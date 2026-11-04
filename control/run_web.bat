@echo off
chcp 65001 >nul
echo ========================================================
echo   Mechatronics-4 機器人車輛 Web 控制台伺服器啟動中...
echo ========================================================
echo.
echo [提示] 伺服器啟動後，請在瀏覽器開啟:
echo        http://localhost:8000
echo.
echo [提示] 若需要於同網域手機連線，請輸入本機區域 IP:8000 (例如 192.168.1.xxx:8000)
echo.
echo 按 Ctrl + C 可隨時停止伺服器
echo.

python -m uvicorn server:app --host 0.0.0.0 --port 8000 --reload
pause

