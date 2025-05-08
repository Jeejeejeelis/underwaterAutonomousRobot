# underwaterAutonomousRobot
## Signal Flow Diagram
<img width="842" alt="Signal flow diagram" src="https://github.com/user-attachments/assets/1a63130f-e5be-4e8b-9620-3fa6df868ee5" />


### Notes
- **test_gnss**: Not intended for the sim7600X.
- **test_ctd**: Contains TODO parts related to magic numbers and depth calculations. These relate to a file `ctd.c` which is currently missing.
- **SIM7600X-4G-HAT-Demo**: this is currently used to test GNSS on the sim7600x. Files on rpi, to be uploaded here.
- Argofloat profiling data? 

### TODO
- Address magic numbers in `test_ctd`.
- Implement depth calculations in `test_ctd`.
- Github repo with underwater simulation. https://github.com/tervoju/argofloat 


Cheatsheet:
source myenv/bin/activate



Current values from send_ctd!
python3 test_ctd.py 
cmd_string: [255, 255, 255, 255, 170, 0, 144, 0, 0, 0, 0, 0, 0, 108]
response: b'2441514354442c31382e3837312c30312e3030372c30302e3837352a37410d0a'
response: b'$AQCTD,18.871,01.007,00.875*7A\r\n'
["b'$AQCTD", '18.871', '01.007', "00.875*7A\\r\\n'"]
Git credentials stored to rpi. if not:
git config --global credential.helper store
after this push with key and credentials should be saved.

Git credentials stored to rpi. if not:
git config --global credential.helper store
after this push with key and credentials should be saved.

### Followed these instructions for GNSS!
https://www.waveshare.com/wiki/SIM7600G-H_4G_HAT_(B)
wget https://files.waveshare.com/upload/4/4e/SIM7600G-H-4G-HAT-B-Demo.zip
sudo apt-get install p7zip-full
7z x SIM7600G-H-4G-HAT-B-Demo.zip 
sudo chmod 777 -R SIM7600G-H-4G-HAT-B-Demo
cd SIM7600G-H-4G-HAT-B-Demo/Raspberry/python/PhoneCall/      
sudo apt-get install python3-pip
sudo pip3 install pyserial
sudo pip3 install RPi.GPIO
sudo python3 Phonecall.py    

# then try these, which one works?
sudo python GPS.py
sudo python3 GPS.py

# demo should print this with coordinates:
AT+CGPS=0
AT+CGPS=1
AT+CGPSINFO

Push with ubuntu server now



