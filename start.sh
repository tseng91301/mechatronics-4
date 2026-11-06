cd ~/mediamtx && ./mediamtx &

# New Terminal
cd ~/mechatronics-4/records && ffmpeg -y -f v4l2 -video_size 640x480 -framerate 20 -i /dev/logi_camera \
  -c:v libx264 -preset ultrafast -tune zerolatency \
  -g 20 -bf 0 -b:v 800k -maxrate 800k -bufsize 400k \
  -pix_fmt yuv420p \
  -f rtsp -rtsp_transport tcp rtsp://localhost:8554/live/stream \
  -map 0:v -c:v libx264 -preset ultrafast -tune zerolatency -g 20 -bf 0 -b:v 800k -maxrate 800k -bufsize 400k -pix_fmt yuv420p -f mp4 output.mp4 &

# New Terminal
cd ~/mechatronics-4/control && venv/bin/python -m uvicorn server:app --host 0.0.0.0 --port 8000 --reload &