import json
import queue
import time
from multiprocessing import Process, Manager
from typing import Optional
import os
import requests

from communication.stm32 import STMLink


stm_link = STMLink()
stm_link.connect()
stm_link.send("FS09") #forward straight, distance and speed etc to be specified by STM board
ack1 = stm_link.recv() 
print(f"Received ACK for FS09: {ack1}")
time.sleep(2)
print("Sending 'STOP' to halt movement.")
stm_link.send("STOP")
ack2 = stm_link.recv()
print(f"Received ACK for STOP: {ack2}")
stm_link.disconnect()
print("Disconnected from STM32.")