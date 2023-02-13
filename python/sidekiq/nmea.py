#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2023 gr-sidekiq author dph.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

# This module will open the UART port for NMEA traffic from the GPS module.
# This gives the time, date, fix, lat, long, and num_sats info.
# It sends it to other modules via messages.
# It has no streams, only one output message port.


import numpy as np
from gnuradio import gr
import pmt
import io

import serial
import pynmea2
import time
import threading

DEBUG_LEVEL = "warn"

# Information stored from NMEA stream
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

# The nmea thread is created by gnuradio.  In the basic block, work() is not called by gnuradio
# so start() is the only function called by gnuradio.  Thus we cannot sit in a hard loop in start, 
# so we are doing the work from a separate thread.  That thread is created in start() and killed in stop()
class get_info_thread(threading.Thread):
    def __init__(self, thread_name, thread_ID):
        threading.Thread.__init__(self)
        self.thread_name = thread_name
        self.thread_ID = thread_ID

    # sit in a hard loop getting the NEMA stream
    def run(self):
        global running, blkobject
        while running:
            blkobject.do_work()

# This is the object created by gnuradio, the start() and stop() method are called by gnuradio
class nmea(gr.basic_block):  # other base classes are basic_block, decim_block, interp_block
    thread_id = get_info_thread("GFG", 1000)

    def __init__(self, port = '/dev/ttySKIQ_UART1'):  # only default arguments here
        global blkobject, ser
        
        gr.basic_block.__init__(
            self,
            name='nmea',   # will show up in GRC
            in_sig = None,
            out_sig = None)


        # define the message port and register it
        self.d_port = pmt.mp("out_msg")
        self.message_port_register_out(self.d_port)

        # create the logger, set the level 
        self.my_log = gr.logger(self.alias())
        self.my_log.set_level(DEBUG_LEVEL)
        self.my_log.debug(f"in constructor")

        # since gnuradio instantiated this object, we don't know it, so store it for the 
        # get_info_thread
        blkobject = self

        # if an attribute with the same name as a parameter is found,
        # a callback is registered (properties work, too).
        self.port = port

        # attempt to open the serial port passed in
        self.my_log.debug(f"Setting uart port to {port}")

        try:
            self.ser = serial.Serial(port, 9600, timeout=5.0)

        except serial.SerialException as e:
            print('Device error: {}'.format(e))
            exit()

        except pynmea2.ParseError as e:
           print('Parse error: {}'.format(e))
           exit()


    # Get the nmea stream from the serial port and parse it 
    def get_nmea(self, portname):
        global utc_time, fix, lat, lat_dir, lon, lon_dir, altitude, num_sats, date 
        self.my_log.debug(f"get_nmea")

        try:
            line = self.ser.readline()
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
                
                self.my_log.info(f"utc_time {utc_time}, fix {fix}, Lat {lat}, Lat_dir {lat_dir}," 
                        "Long {lon}, Lon_dir {lon_dir}, Altitude {altitude}, Num Sats {num_sats}" )

            if "RMC" in newline:
                data = pynmea2.parse(newline)
                date = str(data.datetime.date())
                timestamp =  data.datetime.time()
                utc_time = str(timestamp) 
            
                self.my_log.info(f"date {date}, time {utc_time}")
           
            # These are not used, but are here for reference
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
            exit()

        except pynmea2.ParseError as e:
            print('Parse error: {}'.format(e))
            exit()

    # this method is called by the get_info_thread, in a loop
    def do_work(self):
        global utc_time, fix, lat, lat_dir, lon, lon_dir, altitude, num_sats, date
        self.my_log.debug("do_work")

        # Go get the current nmea stream dta
        self.get_nmea(self.port)

        # Make a dictionary to store the values received
        msg = pmt.make_dict()
        valid_output = False

        # Check each global variable to see if it needs to be reported in the next message out
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

        # If any of the parameters were added, send the message
        if valid_output == True:
            self.message_port_pub(self.d_port, msg)

        return

    # This is called by gnuradio, start the get_info_thread
    def start(self):
        global thread1, running
        self.my_log.debug("start")

        running = True
        nmea.thread_id.start()
        return

    # This is called by gnuradio, stop the get_info_thread
    def stop(self):
        global thread_id, running

        self.my_log.debug("stop")
        running = False
        nmea.thread_id.join()
        return

        




