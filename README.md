Compiled and Tested on Raspberry Pi OS Lite
following packages should be installed:
```
cmake libdbus-1-dev libbluetooth-dev
```
Download and install pigpio library http://abyz.me.uk/rpi/pigpio/download.html

For successful sdp connection it may be necessary to edit file ```/etc/systemd/system/dbus-org.bluez.service``` change line ``` ExecStart=/usr/lib/bluetooth/bluetoothd``` with ```ExecStart=/usr/lib/bluetooth/bluetoothd --compat```

to immediately apply changes issue this commands
```
sudo systemctl daemon-reload
sudo systemctl restart bluetooth
```
I2C interface should be enabled (e.g. via ```raspi-config``` tool)

The app itself also should be run as root
