# qriosity
Qriosity Rover Ros Package

### Settings
Rover Ip - 192.168.1.23
Base Station Ip - 192.168.1.21

### Rover Video Stream
---
```
v4l2-ctl --device=/dev/video0 --set-fmt-video=width=800,height=600,pixelformat=1

cvlc v4l2:///dev/video1:chroma=h264:width=800:height=600 --sout#standard{access=http,mux=ts,dst=192.168.1.23:5000,name=stream,mime=video/ts}' -vvvcvlc v4l2:///dev/video1 --demux h264

```

### Rover control Software
---
You should be able to access the rover's control software from the following web address http://192.168.1.23:500/gui

### Rover gamepad example
---
The gamepad.js file in tests needs the index.html page from this repo:
https://github.com/luser/gamepadtest
