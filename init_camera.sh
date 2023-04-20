timeout 2 rostopic pub /anafi/cmd_camera olympe_bridge/CameraCommand "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
roll: 0.0
pitch: -90.0
zoom: 0.0
action: 0" 

echo "Camera message published!"