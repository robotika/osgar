#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 13 20:14:48 2021

@author: linux-mint
"""

import subprocess

from osgar.node import Node


def bluetooth_scan():
    while True:
        with subprocess.Popen('hcitool scan', shell=True, stdout=subprocess.PIPE) as data:
            for line in data.stdout:
                line = line.decode()
                if "CubeArtifact" in line:
                   #print(line)
                   device = line.split()[0]
                   print(device)
                   signalStrength = bluetooth_rssi_scan(device)
                   return signalStrength 
                   

def bluetooth_rssi_scan(device):
    rssi_find = True
    while rssi_find: 
        with subprocess.Popen('bluetoothctl scan on', shell=True, stdout=subprocess.PIPE) as data:
            for line in data.stdout:
                line = line.decode()
                #print(line)
                if device in line:
                    if "RSSI: " in line:
                        mac_adress = line.split()[2]
                        if mac_adress in device:
                            rssi_find = False
                            signalStrength = line.split("RSSI: ")[1]
                            try:
                                signalStrength  = int(signalStrength)
                                print(signalStrength)
                                return signalStrength    
                            except "e":
                                pass
                                #print(e)


class Bluetooth_signal(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('bluetoothscan')
        
    def run(self):
        while self.is_bus_alive():
            signalStrength = bluetooth_scan()
            now = self.publish("bluetoothscan", signalStrength)
            
