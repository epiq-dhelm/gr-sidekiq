"""
Embedded Python Blocks:

Each time this file is saved, GRC will instantiate the first class it finds
to get ports and parameters of your block. The arguments to __init__  will
be the parameters. All of them are required to have default values!
"""

import numpy as np
from gnuradio import gr
import pmt
import time
import threading

running = False

class thread(threading.Thread):
    def __init__(self, thread_name, thread_ID):
        threading.Thread.__init__(self)
        self.thread_name = thread_name
        self.thread_ID = thread_ID
 
        # helper function to execute the threads
        
    def run(self):
        global running, blkobject
        print(str(self.thread_name) +" "+ str(self.thread_ID));
        while running:
            print("thread running")
            blkobject.do_work()
            time.sleep(2)


class blk(gr.basic_block):  # other base classes are basic_block, decim_block, interp_block
    """Embedded Python Block example - a simple multiply const"""
    thread = thread("GFG", 1000)
    
    
    
	
    def __init__(self, example_param=1.0):  # only default arguments here
        global blkobject
        
        """arguments to this function show up as parameters in GRC"""
        gr.basic_block.__init__(
            self,
            name='test',   # will show up in GRC
            in_sig=None,
            out_sig=None
        )
        self.message_port_register_out(pmt.intern("msg_out"))
        blkobject = self
        
        # if an attribute with the same name as a parameter is found,
        # a callback is registered (properties work, too).
        self.example_param = example_param

    def work(self, input_items, output_items):
        """example: multiply with constant"""
        print("work")
        pass 
        
    def do_work(self):
        print("do work")
          
    def start(self):
        global thread, running
        print("start")
        running = True
        blk.thread.start()
        return 
    	
    def stop(self):
        global thread, running
        print("stop")
        running = False
        blk.thread.join()
    	
        return 


        


print("main") 


print("Exit")    	

