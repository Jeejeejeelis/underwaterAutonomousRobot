# waveshare 4G

here is the short description how to use waveshasre 4G and (GNSS module) with usb connection.
the used board seems to have usb and both modem and GNSS are visible as usb modules.
GNSS module works normally as usb GPS modulle with different /dev/USBX, and man id as below

Bus 001 Device 006: ID 1e0e:9001 
USB1

see techical details from here:

https://www.waveshare.com/wiki/SIM7600E-H_4G_HAT


```
$ ls /dev/ttyUSB*
/dev/ttyUSB0  /dev/ttyUSB1  /dev/ttyUSB2  /dev/ttyUSB3  /dev/ttyUSB4
```

## check link 

good introduction (or the best I found) how to connect modem with usb to raspberry

https://www.jeffgeerling.com/blog/2022/using-4g-lte-wireless-modems-on-raspberry-pi


```
#lsusb
Bus 001 Device 006: ID 1e0e:9001 Qualcomm / Option SimTech, Incorporated
```


https://raspberrypi.stackexchange.com/questions/104388/waveshare-sim7600e-h-no-wwan0-interface-when-connected-to-raspberry-pi-4-model-b


# install the packages

```
sudo apt-get update && sudo apt-get install libqmi-utils udhcpc
```

```
ifconfig wwan0 down
```

- Enable OS Raw IP Mode setting (not persistent) 
(actually used nano to edit raw_ip)
```
echo Y > /sys/class/net/wwan0/qmi/raw_ip
```
```
ifconfig wwan0 up
qmicli -d /dev/cdc-wdm0 --dms-set-operating-mode='online'
```

- change apn value according to the service provider information (also maybe username and password are needed)
```
qmicli -d /dev/cdc-wdm0 --device-open-net="net-raw-ip|net-no-qos-header" --wds-start-network="apn='internet',ip-type=4" --client-no-release-cid 
```
```
udhcpc -i wwan0
```

## Automatic re-connection
The simplest way to set up automatic re-connection is to add the following contents in a new file for the wwan0 interface:

### Create the file.
```
sudo nano /etc/network/interfaces.d/wwan0
```

### Paste in the following contents (replacing with your own APN).
```
iface wwan0 inet manual
     pre-up ifconfig wwan0 down
     pre-up echo Y > /sys/class/net/wwan0/qmi/raw_ip
     pre-up for _ in $(seq 1 10); do /usr/bin/test -c /dev/cdc-wdm0 && break; /bin/sleep 1; done
     pre-up for _ in $(seq 1 10); do /usr/bin/qmicli -d /dev/cdc-wdm0 --nas-get-signal-strength && break; /bin/sleep 1; done
     pre-up sudo qmicli -p -d /dev/cdc-wdm0 --device-open-net='net-raw-ip|net-no-qos-header' --wds-start-network="apn='nxtgenphone',ip-type=4" --client-no-release-cid
     pre-up udhcpc -i wwan0
     post-down /usr/bin/qmi-network /dev/cdc-wdm0 stop
```

Note: Be sure to include username and password as stated earlier, if your SIM plan requires it.

Reboot (sudo reboot) and once rebooted, run the following to manage the connection:

### ring up the connection.
```
sudo ifup wwan0
```

### Bring down the connection.
```
sudo ifdown wwan0
```

### automatically from start - modem up
If you want the interface to come up at system boot, add the line auto wwan0 above the iface line in the /etc/network/interfaces.d/wwan0 file:

```
auto wwan0
iface wwan0 inet manual
```

## ssh with iot hub device stream

- install dotnet core to pi and possibly the missing library dependencies
- get device streaming code for the Device 

     https://learn.microsoft.com/en-us/azure/iot-hub/iot-hub-device-streams-overview

     https://github.com/fzankl/iot-hub-devicestreams-sample
- device streaming should be able to connect to IOT hub and open the streaming connection to the device
- properties for the devices in properties/LaunchSettings.json should be changed to the correct values device connection string (from the IoT Hub). Same string is already in the IoT Edge config.json file (if SAS token is used)
- install the device streaming on the device (raspberry pi) and run it on the background with crontab
- crontab 
     e.g. https://phoenixnap.com/kb/crontab-reboot 
     
     @reboot 
     - add @reboot sleep 120 && /home/pi/dotnet/dotnet run --project /home/pi/Development/iot-hub-devicestreams-sample/src/DeviceProxy/DeviceProxy.csproj &
     - path depends on the location of the device streaming sw

- check if the device streaming is running 

```
pi@pi: ps aux | grep dotnet

pi         421  6.9 13.0 405680 123400 ?       Sl   08:37   0:23 /home/pi/dotnet/dotnet run --project /home/pi/Development/iot-hub-devicestreams-sample/src/DeviceProxy/DeviceProxy.csproj
edgeage+  1435  8.9  4.5 169788 43404 ?        Ssl  08:37   0:27 /usr/bin/dotnet Microsoft.Azure.Devices.Edge.Agent.Service.dll
pi        3708  0.0  0.0   7452   544 pts/0    S+   08:43   0:00 grep --color=auto dotnet
```

- for the remote connection to the device streaming you need to service proxy running on the laptop were you want to connect to the iot edge device https://github.com/fzankl/iot-hub-devicestreams-sample  ServiceProxy which connects to the IOT hub and opens the streaming connection to the device.

```	
dotnet run --project src/ServiceProxy/ServiceProxy.csproj
```

and right settings in the Properties/launchSettings.json for that IoT Hub and device

```

```
ssh pi@localhost -p 2222
```


