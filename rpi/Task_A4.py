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
stm_link.send("TR00") #turn right, distance and speed etc to be specified by STM board
