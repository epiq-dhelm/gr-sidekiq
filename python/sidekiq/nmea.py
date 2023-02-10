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
        
        """arguments to this function show up as parameters in GRC"""
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
        global utc_time, fix
        try:
            line = ser.readline()
            newline = str(line, 'utf-8', errors='ignore')
            if "GGA" in newline:
                data = pynmea2.parse(newline)

                timestamp = data.timestamp
                utc_time = str(timestamp)
                print("utc_time", utc_time)

                lat, lon, alt = data.latitude, data.longitude, data.altitude
                lat_dir, lon_dir, num_sats = data.lat_dir, data.lon_dir, data.num_sats
                gps_qual = data.gps_qual
                if gps_qual != 0:
                    fix = "yes"
                else:
                    fix = "no"
                print("fix(y/n): ", fix, " Timestamp(UTC):"  , timestamp,  " Lat:", "{:4.6}".format(lat), lat_dir,  " Long:", "{:4.6}".format(lon), lon_dir,  " Altitude:", alt,  " Num Sats:", num_sats)

            if "RMC" in newline:
                data = pynmea2.parse(newline)
                date = data.datetime.date()
                print("date",date)


            if "GSV" in newline:
                data = pynmea2.parse(newline)
                sats_in_view = data.num_sv_in_view
                print("sats in view",sats_in_view)

            if "VTG" in newline:
                data = pynmea2.parse(newline)
                horizontal_speed = data.spd_over_grnd_kmph
                print("speed over ground Kmph", horizontal_speed)

        except serial.SerialException as e:
            print('Device error: {}'.format(e))
            return

        except pynmea2.ParseError as e:
           print('Parse error: {}'.format(e))
           return

    def do_work(self):
        global utc_time, fix

        self.get_nmea(self.port)

        # get length of string
        _len = len(utc_time)
        if (_len > 0):
            print("\n timestamp ", utc_time)
            # terminate with LF
            utc_time += "\n"
            _len += 1

            key1 = pmt.intern("utc_time")
            value1 = pmt.intern(utc_time)

            key2 = pmt.intern("lock")
            value2 = pmt.intern(fix)

            msg = pmt.make_dict()
            msg = pmt.dict_add(msg, key1, value1)
            msg = pmt.dict_add(msg, key2, value2)

            self.message_port_pub(self.d_port, msg)
            print("\n port_pub ", utc_time)
            utc_time = ""
            return (_len)
        else:
            return (0)

    def work(self):
        print("in_work")

        while 1:
            print("running")
            if (running):
                self.do_work()
        return 

    def general_work(self):
        print("in general_work")

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


