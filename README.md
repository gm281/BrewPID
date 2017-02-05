# BrewPID

Arduino based temperature controller for brewing beer.

## RaspberryPI setup
* use 2014-09-09-wheezy-raspbian.img (RaspberryPI is old, mark 1, so it has armv6, no longer widely supported). Also, stored on Google Drive as a backup.
* use wired network (this dhcp configures by default, acces via pi/raspberry login details)
```
scp -r RaspberryPi pi@192.168.0.101:~/
```
```
sudo apt-get install python-pip vim screen
sudo pip install bottle
sudo pip install pyserial
mkdir ~/TemperatureLogs
sudo dmesg | grep ttyACM # Figure out what serial device arduino appears as
sudo vim /boot/cmdline.txt # Remove mention of that ttyACM from boot console
vim brew_pid.py # set DEVICE to ttyACM, make sure DEBUG_MODE=False
sudo reboot
cat /proc/cmdline # Verify ttyACM gone from kernel cmdline
cd RaspberryPi/
screen -L
python brew_pid.py
```

