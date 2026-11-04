# About Camera Settings
## First Setting Camera
### Bind Camera via UUID
#### Get USB Info
```bash
lsusb
# Bus 001 Device 004: ID 046d:081b Logitech, Inc. Webcam C310
```
* `046d`: idVendor
* `0825`: idProduct
```bash
udevadm info --name=/dev/video0 --attribute-walk | grep -E 'idVendor|idProduct|serial|ATTR{index}'
# Check if idProduct and idVendor are the same.
```
```bash
sudo vim /etc/udev/rules.d/99-usb-webcam.rules

# Insert this:
# SUBSYSTEM=="video4linux", ATTR{index}=="0", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="081b", ATTRS{serial}=="62C79F10", SYMLINK+="logi_camera", MODE="0660", GROUP="video"

sudo udevadm control --reload-rules && sudo udevadm trigger

# Check if bind successfully.
ls -l /dev/logi_camera
```


## Setting RTSP Server
### Download MediaMTX Repo
```bash
cd ~ && mkdir mediamtx && cd mediamtx
wget https://github.com/bluenviron/mediamtx/releases/download/v1.21.0/mediamtx_v1.21.0_linux_arm64.tar.gz
tar -zxvf mediamtx_v1.21.0_linux_arm64.tar.gz
```

### Start Stream Publisher
```bash
cd ~/mediamtx && ./mediamtx

# New Terminal
ffmpeg -f v4l2 -i /dev/logi_camera -c:v libx264 -preset ultrafast -f rtsp rtsp://localhost:8554/live/stream

ffmpeg -f v4l2 -video_size 640x480 -framerate 20 -i /dev/logi_camera \
  -c:v libx264 -preset ultrafast -tune zerolatency \
  -g 20 -bf 0 -b:v 800k -maxrate 800k -bufsize 400k \
  -pix_fmt yuv420p -f rtsp -rtsp_transport tcp rtsp://localhost:8554/live/stream
```

### Open Viewer
* Web: http://mechatronic-4-6.local:8889/live/stream/?network-caching=0
* VLC Viewer: Open Network Streaming -> `rtsp://mechatronic-4-6.local:8554/live/stream` -> Cache 0ms