#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2023 gr-sidekiq author.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#



import numpy as np
from gnuradio import gr
import pmt
import io

import serial
import pynmea2
import time
import threading

ser = serial.Serial('/dev/ttySKIQ_UART1', 9600, timeout=5.0)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
date = ""
output_value = ""
utc_time = ""
fix = ""
lat = ""
lat_dir = ""
lon = ""
lon_dir = ""
altitude = ""
num_sats = ""
date = ""

running = False

class thread(threading.Thread):
    def __init__(self, thread_name, thread_ID):
        threading.Thread.__init__(self)
        self.thread_name = thread_name
        self.thread_ID = thread_ID

        # helper function to execute the threads

    def run(self):
        global running, blkobject
        while running:
            blkobject.do_work()

class nmea(gr.basic_block):  # other base classes are basic_block, decim_block, interp_block
    thread_id = thread("GFG", 1000)

    def __init__(self, port = '/dev/ttySKIQ_UART1'):  # only default arguments here
        global blkobject
        
        gr.basic_block.__init__(
            self,
            name='nmea',   # will show up in GRC
            in_sig = None,
            out_sig = None)
        self.d_port = pmt.mp("out_txt")
        self.message_port_register_out(self.d_port)

        blkobject = self

        # print (textboxValue)
        # if an attribute with the same name as a parameter is found,
        # a callback is registered (properties work, too).
        self.port = port


    def get_nmea(self, portname):
        global utc_time, fix, lat, lat_dir, lon, lon_dir, altitude, num_sats, date 
        try:
            line = ser.readline()
            newline = str(line, 'utf-8', errors='ignore')
            if "GGA" in newline:
                data = pynmea2.parse(newline)

                utc_time = str(data.timestamp)

                gps_qual = data.gps_qual
                if gps_qual != 0:
                    fix = "yes"
                else:
                    fix = "no"

                lat = "{:4.6}".format(data.latitude)
                lat_dir = str(data.lat_dir)
                lon = "{:4.6}".format(data.longitude)
                lon_dir = str(data.lon_dir)
                altitude = str(data.altitude)
                num_sats = str(data.num_sats)

                print("fix(y/n): ", fix, " Timestamp(UTC):"  , utc_time,  " Lat:", "{:4.6}".format(lat), 
                        lat_dir,  " Long:", "{:4.6}".format(lon), lon_dir,  " Altitude:", altitude,  " Num Sats:", num_sats)

            if "RMC" in newline:
                data = pynmea2.parse(newline)
                date = str(data.datetime.date())
                timestamp =  data.datetime.time()
                utc_time = str(timestamp) 
            
                print("date", date, "time", utc_time)
            
            """
            if "GSV" in newline:
                data = pynmea2.parse(newline)
                sats_in_view = data.num_sv_in_view
                print("sats in view",sats_in_view)

            if "VTG" in newline:
                data = pynmea2.parse(newline)
                horizontal_speed = data.spd_over_grnd_kmph
                print("speed over ground Kmph", horizontal_speed)
            """

        except serial.SerialException as e:
            print('Device error: {}'.format(e))
            return

        except pynmea2.ParseError as e:
           print('Parse error: {}'.format(e))
           return

    def do_work(self):
        global utc_time, fix, lat, lat_dir, lon, lon_dir, altitude, num_sats, date

        self.get_nmea(self.port)
        msg = pmt.make_dict()
        valid_output = False








        if (num_sats != ""):
            key = pmt.intern("num_sats")
            value = pmt.intern(num_sats)
            msg = pmt.dict_add(msg, key, value)
            num_sats = ""
            valid_output = True

        if (altitude != ""):
            key = pmt.intern("altitude")
            value = pmt.intern(altitude)
            msg = pmt.dict_add(msg, key, value)
            altitude = ""
            valid_output = True

        if (lon_dir != ""):
            key = pmt.intern("lon_dir")
            value = pmt.intern(lon_dir)
            msg = pmt.dict_add(msg, key, value)
            lon_dir = ""
            valid_output = True

        if (lon != ""):
            key = pmt.intern("lon")
            value = pmt.intern(lon)
            msg = pmt.dict_add(msg, key, value)
            lon = ""
            valid_output = True

        if (lat_dir != ""):
            key = pmt.intern("lat_dir")
            value = pmt.intern(lat_dir)
            msg = pmt.dict_add(msg, key, value)
            lat_dir = ""
            valid_output = True

        if (lat != ""):
            key = pmt.intern("lat")
            value = pmt.intern(lat)
            msg = pmt.dict_add(msg, key, value)
            lat = ""
            valid_output = True

        if (fix != ""):
            key = pmt.intern("fix")
            value = pmt.intern(fix)
            msg = pmt.dict_add(msg, key, value)
            fix = ""
            valid_output = True

        if (utc_time != ""):
            key = pmt.intern("utc_time")
            value = pmt.intern(utc_time)
            msg = pmt.dict_add(msg, key, value)
            utc_time = ""
            valid_output = True

        if (date != ""):
            key = pmt.intern("date")
            value = pmt.intern(date)
            msg = pmt.dict_add(msg, key, value)
            date = ""
            valid_output = True

        if valid_output == True:
            self.message_port_pub(self.d_port, msg)

        return

    def start(self):
        global thread1, running

        print("in start")

        running = True
        nmea.thread_id.start()
        return

    def stop(self):
        global thread1, running

        print("in stop")
        running = False
        nmea.thread.join()

        return

        


print("main") 


print("Exit")    	


