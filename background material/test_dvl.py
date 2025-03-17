import socket
import time
import sys
import os
import logging
import json
import csv

from flatten_json import flatten
from datetime import datetime
from datetime import timezone

from azure.iot.device.aio import IoTHubModuleClient
from azure.iot.device import MethodResponse
from azure.iot.device import Message

from six.moves import input
import threading
import asyncio
from functools import wraps, partial

from dvl2csv_file import dvl2csv_file

TCP_IP = "192.168.194.95"
TCP_PORT = 16171
deviceid = "SUB"

save_locally = True

dataJson = b''

def async_wrap(func):
    @wraps(func)
    async def run(*args, loop=None, executor=None, **kwargs):
        if loop is None:
            loop = asyncio.get_event_loop()
        pfunc = partial(func, *args, **kwargs)
        return await loop.run_in_executor(executor, pfunc)
    return run

class TCPConnection:
    def __init__(self, client=None, sock=None):
        logging.basicConfig()
        logging.getLogger().setLevel(os.environ.get("LOGLEVEL", "INFO"))
        self.module_client = client
        if sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock
        self.dvl_file = dvl2csv_file()
    @async_wrap
    def connect(self, host, port):
        try:
            self.sock.connect((host, port))
            logging.info('Successful Connection to DVL')
        except:
            logging.info('Connection Failed DVL')
    @async_wrap
    def readlines(self):
        data = self.sock.recv(1024)
        print(data)

    #@async_wrap
    async def read_dvl(self):
        global dataJson, deviceid, save_locally
        vx = 0
        vy = 0
        vz = 0
        altitude = 0
        measurement_cnt = 0
        dataJson = b''
        rawdata = b''
        data = b''
        time_delta = 0
        logging.info('Read DVL')
        while(1):
            logging.info('Read DVL - Loop')
            rawdata = b''
            while not b'\n' in rawdata:
                try:
                    data = self.sock.recv(1)
                    if len(data) == 0:
                        logging.info('dvl socket lost, no data, reconnecting')
                        #self.connect(TCP_IP, TCP_PORT)
                        continue
                except:
                    logging.info('Lost connection to DVL, reconnecting')
                    self.connect(TCP_IP, TCP_PORT)
                    continue
                rawdata = rawdata + data
        
            rawdata = dataJson + rawdata
            dataJson = b''
            strdata = rawdata.decode("utf-8").split('\n')
            dJson = strdata[1]
            strdata = strdata[0]
            logging.info("dvl message received")
            # here the dvl message handing
            jsondata = json.loads(strdata) 
           
            # DVL velocity message
            if "time" in jsondata:
                # 1 forward ?

                # 2 save local csv file
                try:
                    if save_locally == True:
                        self.dvl_file.write_csv_data(jsondata)
                except:
                    logging.info('fails to write csv')
                    
                time_delta = time_delta + jsondata["time"]/1000.0
                # 3 target to average 60 s measurements  and forward
                if jsondata["velocity_valid"] == True:
                    # average 60 s distance -> if 0,05 m/s current -> 3 m 
                    # if 80 ms between messages -> 750 messages?
                    vx = vx + jsondata["vx"]
                    vy = vy + jsondata["vy"]
                    vz = vz + jsondata["vz"]
                    altitude = altitude + jsondata["altitude"]
                    measurement_cnt += 1
                    if time_delta >= 1.0:
                        message = {
                            "deviceid": deviceid,
                            "timestamp": datetime.now(timezone.utc).isoformat(),
                            "vx": vx / measurement_cnt,
                            "vy": vy / measurement_cnt,
                            "vz": vz / measurement_cnt,
                            "altitude" : altitude /measurement_cnt,
                            "measurements": measurement_cnt
                        }
                        time_delta = 0
                        measurement_cnt = 0
                        vx = 0
                        vy = 0
                        vz = 0
                        logging.info("message collected")
                        payload = Message(json.dumps(message), content_encoding="utf-8", content_type="application/json")
                        await self.module_client.send_message_to_output(payload, "output1") 
            # IMU message with x,y,z
            if "ts" in jsondata:
                logging.info("ts as IMU message")
                    # 2 save local csv file
                try:
                    if save_locally == True:
                        self.dvl_file.write_csv_data(jsondata)
                        
                except:
                    logging.info('fails to write csv')
            # 