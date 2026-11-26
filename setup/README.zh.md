# 系統環境初始化與攝影機設定指南 (`setup/`)

本文件說明如何在 **Raspberry Pi 4B** 上初始化共用的 Python 虛擬環境，以及設定 USB 攝影機固定裝置代號與 MediaMTX 影像串流伺服器。

---

## 一、 初始化共用 Python 虛擬環境 (`venv`)

專案根目錄的 `venv/` 與 [requirements.txt](../requirements.txt) 由全專案共用（包含 [control/](../control/)、[strategies/](../strategies/) 與 [frame_recognize/](../frame_recognize/)）。

### 1. 建立並安裝套件
請在專案**最外層根目錄**執行以下指令：

```bash
cd ~/mechatronics-4

# 1. 建立 Python 虛擬環境
python3 -m venv venv

# 2. 啟用虛擬環境
source venv/bin/activate

# 3. 升級 pip 並安裝所有共用相依套件
pip install --upgrade pip
pip install -r requirements.txt
```

### 2. 驗證安裝是否成功
```bash
venv/bin/python -c "import fastapi, uvicorn, serial, cv2, numpy; print('Python 環境檢查通過！OpenCV:', cv2.__version__)"
```

### 3. 日常使用方式
* **透過腳本自動呼叫**：[start.sh](../start.sh) 與 [control/run_web.bash](../control/run_web.bash) 已預設呼叫根目錄 `venv/bin/python`，無須手動切換。
* **直接執行單支測試腳本**：
  ```bash
  # 在專案根目錄使用虛擬環境執行 Python 程式
  venv/bin/python control/test_connection.py
  venv/bin/python frame_recognize/detect_confetti_video.py --camera 0
  ```

---

## 二、 USB 攝影機固定裝置名稱設定 (`/dev/logi_camera`)

Linux 重開機或插拔 USB 後 `/dev/video0`、`/dev/video1` 編號可能會變動，建議透過 `udev` 規則將攝影機 UUID／序號固定綁定為 `/dev/logi_camera`。

### 1. 查詢 USB 攝影機資訊
```bash
lsusb
# 範例輸出: Bus 001 Device 004: ID 046d:081b Logitech, Inc. Webcam C310
# 其中 046d 為 idVendor，081b 為 idProduct
```

查詢詳細裝置屬性與序號 (`serial`)：
```bash
udevadm info --name=/dev/video0 --attribute-walk | grep -E 'idVendor|idProduct|serial|ATTR{index}'
```

### 2. 建立 udev 規則
編輯 `/etc/udev/rules.d/99-usb-webcam.rules`：
```bash
sudo vim /etc/udev/rules.d/99-usb-webcam.rules
```

寫入以下內容（請依實際查詢到的 `idVendor`、`idProduct` 與 `serial` 替換）：
```udev
SUBSYSTEM=="video4linux", ATTR{index}=="0", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="081b", ATTRS{serial}=="62C79F10", SYMLINK+="logi_camera", MODE="0660", GROUP="video"
```

### 3. 套用規則並驗證
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/logi_camera
```
若成功看到指向 `/dev/videoX` 的符號連結即代表綁定完成。

---

## 三、 設定 MediaMTX 影像串流伺服器 (RTSP / WebRTC)

### 1. 下載與解壓縮 MediaMTX（ARM64 / Raspberry Pi）
```bash
cd ~ && mkdir -p mediamtx && cd mediamtx
wget https://github.com/bluenviron/mediamtx/releases/download/v1.21.0/mediamtx_v1.21.0_linux_arm64.tar.gz
tar -zxvf mediamtx_v1.21.0_linux_arm64.tar.gz
```

### 2. 啟動 MediaMTX 與 FFmpeg 推流
> 實務上建議直接使用專案根目錄的一鍵啟動腳本 [start.sh](../start.sh)，以下為手動測試指令：

```bash
# 終端機 1：啟動串流轉發伺服器
cd ~/mediamtx && ./mediamtx

# 終端機 2：將 /dev/logi_camera 推流至 RTSP (低延遲設定)
ffmpeg -f v4l2 -video_size 640x360 -framerate 20 -i /dev/logi_camera \
  -c:v libx264 -preset ultrafast -tune zerolatency \
  -g 20 -bf 0 -b:v 800k -maxrate 800k -bufsize 400k \
  -pix_fmt yuv420p -f rtsp -rtsp_transport tcp rtsp://localhost:8554/live/stream
```

### 3. 影像觀看方式
* **網頁 WebRTC 串流**：`http://mechatronic-4-6.local:8889/live/stream/?network-caching=0`（或使用實際 Raspberry Pi IP）
* **VLC 播放器**：開啟網路串流 → `rtsp://mechatronic-4-6.local:8554/live/stream`（建議將快取設為 `0ms`）
* **OpenCV / 策略視覺模組**：直接讀取 `rtsp://localhost:8554/live/stream`（預設已整合於 [runner.py](../strategies/runner.py) 與 [vision_align.py](../strategies/techniques/vision_align.py)）
