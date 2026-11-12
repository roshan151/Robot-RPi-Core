# debugging explore
vcgencmd get_throttled                 # anything other than 0x0 means under-voltage or throttling happened
last -x reboot | head                  # were there real reboots, and when?
journalctl -b -1 -e --no-pager | tail -40   # last lines of the previous boot, including the explorer's own output


## Debug camera

vcgencmd get_throttled
rpicam-hello -t 5000 --nopreview                 # camera alone
python tests/hardware/check_camera.py            # camera alone, through our code
python tests/hardware/check_camera.py head       # opens the head, then moves it

2. Check camera detection
   rpicam-hello --list-cameras
   dmesg | grep -i -E "ov5647|unicam|csi|i2c" | tail -20
   vcgencmd get_throttled

3. Did the rpicam package change
grep -E "Start-Date|Commandline|Upgrade:" /var/log/apt/history.log | tail -30
dpkg -l | grep -E "libcamera|rpicam|picamera2|raspi-firmware|linux-image" 
uname -r

4. 
rpicam-hello --list-cameras
dmesg | grep -i -E "ov5647|unicam|csi" | tail -20

5. source .venv/bin/activate
python -c "import picamera2; print(picamera2.__file__)"

## Debug voltage display

ls -l /tmp/*.sock
systemctl status pisugar-server --no-pager
python3 -c "import socket;s=socket.socket(socket.AF_UNIX);s.connect('/tmp/pisugar-server.sock');s.sendall(b'get battery\n');print(s.recv(100))"