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

ser = serial.Serial('/dev/ttySKIQ_UART1', 9600, timeout=5.0)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
date = ""
output_value = ""
utc_time = ""
fix = ""

class nema(gr.sync_block):  # other base classes are basic_block, decim_block, interp_block
    """Embedded Python Block example - a simple multiply const"""


    def __init__(self, port = '/dev/ttySKIQ_UART1'):  # only default arguments here
        """arguments to this function show up as parameters in GRC"""
        gr.sync_block.__init__(
            self,
            name='nema',   # will show up in GRC
            in_sig = None,
            out_sig = [np.byte])
        self.message_port_register_out(pmt.intern('out_txt'))

        # print (textboxValue)
        # if an attribute with the same name as a parameter is found,
        # a callback is registered (properties work, too).
        self.port = port

        self.log=gr.logger("nameOfLogger")
        self.log.set_level("DEBUG")


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




    def work(self, input_items, output_items):
        global utc_time, fix

        self.get_nmea(self.port)


        # get length of string
        _len = len(utc_time)
        if (_len > 0):
            print("\n timestamp ", utc_time)
            self.log.debug("in work {}")
            # terminate with LF
            utc_time += "\n"
            _len += 1
            # store elements in output array
            for x in range(_len):
                output_items[0][x] = ord(utc_time[x])

            key1 = pmt.intern("utc_time")
            value1 = pmt.intern(utc_time)

            key2 = pmt.intern("lock")
            value2 = pmt.intern(fix)

            msg = pmt.make_dict()
            msg = pmt.dict_add(msg, key1, value1)
            msg = pmt.dict_add(msg, key2, value2)


            self.message_port_pub(pmt.intern('out_txt'), msg)
            utc_time = ""
            return (_len)
        else:
            return (0)

