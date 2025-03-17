# Copyright (c) Microsoft. All rights reserved.
# Licensed under the MIT license. See LICENSE file in the project root for
# full license information.

import time
import os
import sys
import asyncio
from six.moves import input
import threading

import json
import pprint
from datetime import datetime
from datetime import timezone
import logging
import pynmea2
import serial
import pyudev

# for example GPS vendor and device ublox GPS receiver
GPS_DEVICE_VENDOR_UBLOX = '1546'
GPS_DEVICE_ID_UBLOX = '01a8'
UBLOX_BAUDRATE = 9600

# gstar
GPS_DEVICE_VENDOR_GSTAR = '067b'
GPS_DEVICE_ID_GSTAR = '2303'
GSTAR_BAUDRATE = 4800
GSTAR_GPS_USB_PORT = "/dev/ttyUSB0"

# SIM
GPS_DEVICE_VENDOR_SIM = '1e0e'
GPS_DEVICE_ID_SIM = '9001'
GPS_SIM_BAUDRATE = 115200
GPS_SIM_DEV = "/dev/ttyUSB2"

# gps setting
GPS_DEVICE_VENDOR = GPS_DEVICE_VENDOR_SIM
GPS_DEVICE_ID = GPS_DEVICE_ID_SIM
GPS_BAUDRATE = GPS_SIM_BAUDRATE
GPS_SENDING_FREQUENCY = 600 

GPS_USB_PORT = GPS_SIM_DEV

FLOAT_SERIAL_NUMBER = ""
TWIN_CALLBACKS = 0

def is_usb_serial(device, vid=None, pid=None):
    # Checks device to see if its a USB Serial device.
    # The caller already filters on the subsystem being 'tty'.
    # If serial_num or vendor is provided, then it will further check to
    # see if the serial number and vendor of the device also matches.

    #pprint.pprint(dict(device.properties))

    # cannot be right if no vendor id
    if 'ID_VENDOR' not in device.properties:
        return False
    # searcing for right vendor
    if vid is not None:
        if device.properties['ID_VENDOR_ID'] != vid:
            logging.info(vid + ' not found  ' + device.properties['ID_VENDOR_ID'])
            return False

    if pid is not None:
        if device.properties['ID_MODEL_ID'] != pid:
            logging.info('not found')
            return False
    return True

def list_devices(vid=None, pid=None):
    devs = []
    context = pyudev.Context()
    for device in context.list_devices(subsystem='tty'):
        if is_usb_serial(device, vid= vid,  pid = pid):
            devs.append(device.device_node)
    return devs

async def main():
    global FLOAT_SERIAL_NUMBER
    global GPS_SENDING_FREQUENCY
    try:
        if not sys.version >= "3.5.3":
            raise Exception( "The module requires python 3.5.3+. Current version of Python: %s" % sys.version )
        print ( "GNSS Client  for Python" )



        # define behavior for receiving an input message on input1
        async def input1_listener(module_client):
            while True:
                input_message = await module_client.receive_message_on_input("input1")  # blocking call
                print("the data in the message received on input1 was ")
                print(input_message.data)
                print("custom properties are")
                print(input_message.custom_properties)
                print("forwarding mesage to output1")
                await module_client.send_message_to_output(input_message, "output1")

        async def receiveGPS(module_client, ser):
            # ptvsd.break_into_debugger()
            gps_message_types = ["$GNRMC", "$GPRMC", "$GLRMC", "$GARMC"]

            while True:
                try: 
                    data = ser.readline().decode('ascii', errors='replace')
                    data = data.strip()
                    #print(data)

                    if len(data) > 6 and data[0:6] in gps_message_types:
                        gps_data = pynmea2.parse(data)
                        if gps_data.latitude == 0 or gps_data.longitude == 0:
                            continue
                        msg = {
                          "timestamp": datetime.now(timezone.utc).isoformat(),
                           "GeopointTelemetry": {
                                "lat":  gps_data.latitude,
                                "lon": gps_data.longitude,
                                "alt": 0
                            },
                            "speed": gps_data.spd_over_grnd,
                            "deviceId": os.environ["IOTEDGE_DEVICEID"],
                            "machineId": FLOAT_SERIAL_NUMBER
                        }
                        # TODO: using this info

                        await asyncio.sleep(GPS_SENDING_FREQUENCY)

                except Exception as e:
                    print("GPS reader errored: %s" % e)
                    await asyncio.sleep(5)

        # define behavior for halting the application
        def stdin_listener():
            while True:
                try:
                    selection = input(".")
                    if selection == "Q" or selection == "q":
                        print("Quitting...\n")
                        break
                except:
                    time.sleep(10)

        # define port for GPS USB module
        gps_port = GPS_USB_PORT
        GPS_PORTS = list_devices(GPS_DEVICE_VENDOR, GPS_DEVICE_ID)
        print(GPS_PORTS)
        if GPS_PORTS != []:
            print('GPS DEVICE(s) FOUND')
            gps_port = GPS_USB_PORT #GPS_PORTS[0] # select first device
            print(gps_port)
        else:
            print('NO RIGHT GPS DEVICE FOUND')

        # GPS receiver
        gps_ser = serial.Serial(gps_port, baudrate=GPS_BAUDRATE, timeout=0.5)

        # Schedule task for C2D Listener
        listeners = asyncio.gather(receiveGPS(module_client, gps_ser))
        print("The GPSreader is now waiting for messages. ")

        # Run the stdin listener in the event loop
        loop = asyncio.get_event_loop()
        user_finished = loop.run_in_executor(None, stdin_listener)

        # Wait for user to indicate they are done listening for messages
        await user_finished

        # Cancel listening
        listeners.cancel()

    except Exception as e:
        print ( "Unexpected error %s " % e )
        raise

if __name__ == "__main__":
    logging.basicConfig()
    logging.getLogger().setLevel(os.environ.get("LOGLEVEL", "INFO"))
    # If using Python 3.7 or above, you can use following code instead:
    asyncio.run(main())