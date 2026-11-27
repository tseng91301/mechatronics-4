# System Setup & Camera Configuration Guide (`setup/`)

> Language: **English** | [繁體中文](README.zh.md)

This guide covers initializing the shared Python virtual environment on **Raspberry Pi 4B**, binding a persistent USB camera device symlink (`/dev/logi_camera`), and configuring the MediaMTX RTSP/WebRTC streaming server.

---

## 1. Initialize Shared Python Virtual Environment (`venv`)

The root `venv/` and [requirements.txt](../requirements.txt) are shared across all modules ([control/](../control/), [strategies/](../strategies/), and [frame_recognize/](../frame_recognize/)).

### 1.1 Create & Install Dependencies
Run from the **project root directory**:

```bash
cd ~/mechatronics-4

# 1. Create virtual environment
python3 -m venv venv

# 2. Activate environment
source venv/bin/activate

# 3. Upgrade pip and install shared packages
pip install --upgrade pip
pip install -r requirements.txt
```

### 1.2 Verify Installation
```bash
venv/bin/python -c "import fastapi, uvicorn, serial, cv2, numpy; print('Python environment OK! OpenCV:', cv2.__version__)"
```

### 1.3 Daily Usage
* **Via startup scripts**: [start.sh](../start.sh) and [control/run_web.bash](../control/run_web.bash) automatically invoke the root `venv/bin/python`.
* **Running individual scripts**:
  ```bash
  venv/bin/python control/test_connection.py
  venv/bin/python frame_recognize/detect_confetti_video.py --source 0
  ```

---

## 2. Bind Persistent USB Camera Symlink (`/dev/logi_camera`)

Linux `/dev/video*` numbers can change across reboots or USB replugs. Use a `udev` rule to map your Logitech camera serial number consistently to `/dev/logi_camera`.

### 2.1 Query USB Device Attributes
```bash
lsusb
# Example: Bus 001 Device 004: ID 046d:081b Logitech, Inc. Webcam C310
```

Inspect `idVendor`, `idProduct`, and `serial`:
```bash
udevadm info --name=/dev/video0 --attribute-walk | grep -E 'idVendor|idProduct|serial|ATTR{index}'
```

### 2.2 Create udev Rule
Edit `/etc/udev/rules.d/99-usb-webcam.rules`:
```bash
sudo vim /etc/udev/rules.d/99-usb-webcam.rules
```

Add (substituting your actual `idVendor`, `idProduct`, and `serial`):
```udev
SUBSYSTEM=="video4linux", ATTR{index}=="0", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="081b", ATTRS{serial}=="62C79F10", SYMLINK+="logi_camera", MODE="0660", GROUP="video"
```

### 2.3 Reload & Verify
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/logi_camera
```

---

## 3. Configure MediaMTX Streaming Server (RTSP / WebRTC)

### 3.1 Download MediaMTX (ARM64 / Raspberry Pi)
```bash
cd ~ && mkdir -p mediamtx && cd mediamtx
wget https://github.com/bluenviron/mediamtx/releases/download/v1.21.0/mediamtx_v1.21.0_linux_arm64.tar.gz
tar -zxvf mediamtx_v1.21.0_linux_arm64.tar.gz
```

### 3.2 Manual Streaming Commands
> In normal operation, use [start.sh](../start.sh) at the repository root. For manual testing:

```bash
# Terminal 1: Start MediaMTX
cd ~/mediamtx && ./mediamtx

# Terminal 2: Publish /dev/logi_camera via FFmpeg with low-latency flags
ffmpeg -f v4l2 -video_size 640x360 -framerate 20 -i /dev/logi_camera \
  -c:v libx264 -preset ultrafast -tune zerolatency \
  -g 20 -bf 0 -b:v 800k -maxrate 800k -bufsize 400k \
  -pix_fmt yuv420p -f rtsp -rtsp_transport tcp rtsp://localhost:8554/live/stream
```

### 3.3 Viewing the Stream
* **WebRTC Browser Stream**: `http://mechatronic-4-6.local:8889/live/stream/?network-caching=0` (or replace with Raspberry Pi IP)
* **VLC Player**: Open Network Stream → `rtsp://mechatronic-4-6.local:8554/live/stream` (`0 ms` cache recommended)
* **OpenCV / Vision Techniques**: Reads `rtsp://localhost:8554/live/stream` directly by default (integrated in [runner.py](../strategies/runner.py) and [vision_align.py](../strategies/techniques/vision_align.py))
