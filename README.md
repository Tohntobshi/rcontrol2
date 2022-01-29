Compiled and Tested on Raspberry Pi OS Lite
following packages should be installed:
```
cmake libdbus-1-dev libbluetooth-dev libopencv-dev
```
Download and install pigpio library http://abyz.me.uk/rpi/pigpio/download.html

Camera interface should be enabled

For successful sdp connection it may be necessary to edit file ```/etc/systemd/system/dbus-org.bluez.service``` change line ``` ExecStart=/usr/lib/bluetooth/bluetoothd``` with ```ExecStart=/usr/lib/bluetooth/bluetoothd --compat```

to immediately apply changes issue this commands
```
sudo systemctl daemon-reload
sudo systemctl restart bluetooth
```
I2C interface should be enabled (e.g. via ```raspi-config``` tool)

and set to 400khz data transfer rate by updating line in ```/boot/config.txt```

from

```dtparam=i2c_arm=on```

to

```dtparam=i2c_arm=on,i2c_arm_baudrate=400000```

The app itself also should be run as root
