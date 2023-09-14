"""
Embedded Python Blocks:

Each time this file is saved, GRC will instantiate the first class it finds
to get ports and parameters of your block. The arguments to __init__  will
be the parameters. All of them are required to have default values!
"""

import numpy as np
from gnuradio import gr
import pmt
import ctypes
import os

#Load the C shared library

filename = 'python_telemetry_epy_block_0.py'
so_path = os.path.abspath(os.path.join( os.path.dirname(filename), 'skiq.so'))
skiq = ctypes.CDLL(so_path)



class blk(gr.sync_block):  # other base classes are basic_block, decim_block, interp_block
    """Embedded Python Block example - a simple multiply const"""

    def __init__(self, card=0):  # only default arguments here
        """arguments to this function show up as parameters in GRC"""
        gr.sync_block.__init__(
            self,
            name='Python Telemetry',   # will show up in GRC
            in_sig=[np.complex64],
            out_sig=[np.complex64]
        )
        # if an attribute with the same name as a parameter is found,
        # a callback is registered (properties work, too).
        self.card = card
        self.selectPortName = 'temp'
        self.message_port_register_in(pmt.intern(self.selectPortName))
        self.set_msg_handler(pmt.intern(self.selectPortName), self.handle_temp_msg)

        self.selectPortName = 'imu'
        self.message_port_register_in(pmt.intern(self.selectPortName))
        self.set_msg_handler(pmt.intern(self.selectPortName), self.handle_imu_msg)
        self.first = True
       

    def handle_temp_msg(self, msg):
        print("in handle_temp_msg")
        temp = ctypes.c_int()

        skiq.get_temp(ctypes.byref(temp))

        print(temp.value)

    def handle_imu_msg(self, msg):
        print("in handle_imu_msg")

    def work(self, input_items, output_items):
        if self.first == True:
            c_card = ctypes.c_byte(self.card)
            skiq.init_card(c_card)
            self.first = False
        output_items[0][:] = input_items[0] 
        return len(output_items[0])
