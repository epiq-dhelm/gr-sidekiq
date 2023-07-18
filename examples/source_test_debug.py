#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: source_test_debug
# GNU Radio version: v3.11.0.0git-375-ge2af6089

from packaging.version import Version as StrictVersion

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print("Warning: failed to XInitThreads()")

from PyQt5 import Qt
from gnuradio import blocks
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import sidekiq
from gnuradio.qtgui import Range, RangeWidget
from PyQt5 import QtCore
import source_test_debug_epy_block_0 as epy_block_0  # embedded python block



from gnuradio import qtgui

class source_test_debug(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "source_test_debug", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("source_test_debug")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "source_test_debug")

        try:
            if StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
                self.restoreGeometry(self.settings.value("geometry").toByteArray())
            else:
                self.restoreGeometry(self.settings.value("geometry"))
        except:
            pass

        ##################################################
        # Variables
        ##################################################
        self.sample_rate = sample_rate = 1e6
        self.samp_rate = samp_rate = 32000
        self.run_rx_calibration = run_rx_calibration = 0
        self.gain_index = gain_index = 10
        self.frequency = frequency = 1000e6
        self.bandwidth = bandwidth = sample_rate * .8

        ##################################################
        # Blocks
        ##################################################

        self._sample_rate_range = Range(1e5, 250e6, 1e6, 1e6, 200)
        self._sample_rate_win = RangeWidget(self._sample_rate_range, self.set_sample_rate, "'sample_rate'", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._sample_rate_win)
        self._gain_index_range = Range(0, 255, 1, 10, 200)
        self._gain_index_win = RangeWidget(self._gain_index_range, self.set_gain_index, "Gain Index", "counter_slider", int, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._gain_index_win)
        self._frequency_range = Range(250e6, 6000e6, 1e6, 1000e6, 200)
        self._frequency_win = RangeWidget(self._frequency_range, self.set_frequency, "'frequency'", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._frequency_win)
        self._bandwidth_range = Range(1e5, 250e6, 1e6, sample_rate * .8, 200)
        self._bandwidth_win = RangeWidget(self._bandwidth_range, self.set_bandwidth, "'bandwidth'", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._bandwidth_win)
        self.sidekiq_sidekiq_rx_0 = sidekiq.sidekiq_rx(1, 0, 100, sample_rate, bandwidth, frequency, 1, gain_index, 0, 0, 0, 0, 2)
        self.sidekiq_sidekiq_rx_0.set_max_output_buffer(32000)
        _run_rx_calibration_push_button = Qt.QPushButton('Run RX Calibration')
        _run_rx_calibration_push_button = Qt.QPushButton('Run RX Calibration')
        self._run_rx_calibration_choices = {'Pressed': 1, 'Released': 0}
        _run_rx_calibration_push_button.pressed.connect(lambda: self.set_run_rx_calibration(self._run_rx_calibration_choices['Pressed']))
        _run_rx_calibration_push_button.released.connect(lambda: self.set_run_rx_calibration(self._run_rx_calibration_choices['Released']))
        self.top_layout.addWidget(_run_rx_calibration_push_button)
        self.epy_block_0 = epy_block_0.blk(example_param=1.0)
        self.epy_block_0.set_max_output_buffer(32000)
        self.blocks_swapiq_0 = blocks.swap_iq(2, gr.sizeof_short)
        self.blocks_swapiq_0.set_max_output_buffer(32000)
        self.blocks_null_sink_0 = blocks.null_sink(gr.sizeof_short*1)
        self.blocks_complex_to_interleaved_short_0 = blocks.complex_to_interleaved_short(False,2047.0)
        self.blocks_complex_to_interleaved_short_0.set_min_output_buffer(32000)
        self.blocks_complex_to_interleaved_short_0.set_max_output_buffer(32000)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_complex_to_interleaved_short_0, 0), (self.blocks_swapiq_0, 0))
        self.connect((self.blocks_swapiq_0, 0), (self.epy_block_0, 0))
        self.connect((self.epy_block_0, 0), (self.blocks_null_sink_0, 0))
        self.connect((self.sidekiq_sidekiq_rx_0, 0), (self.blocks_complex_to_interleaved_short_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "source_test_debug")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_sample_rate(self):
        return self.sample_rate

    def set_sample_rate(self, sample_rate):
        self.sample_rate = sample_rate
        self.set_bandwidth(self.sample_rate * .8)
        self.sidekiq_sidekiq_rx_0.set_rx_sample_rate(self.sample_rate)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate

    def get_run_rx_calibration(self):
        return self.run_rx_calibration

    def set_run_rx_calibration(self, run_rx_calibration):
        self.run_rx_calibration = run_rx_calibration
        self.sidekiq_sidekiq_rx_0.run_rx_cal(self.run_rx_calibration)

    def get_gain_index(self):
        return self.gain_index

    def set_gain_index(self, gain_index):
        self.gain_index = gain_index
        self.sidekiq_sidekiq_rx_0.set_rx_gain_index(self.gain_index)

    def get_frequency(self):
        return self.frequency

    def set_frequency(self, frequency):
        self.frequency = frequency
        self.sidekiq_sidekiq_rx_0.set_rx_frequency(self.frequency)

    def get_bandwidth(self):
        return self.bandwidth

    def set_bandwidth(self, bandwidth):
        self.bandwidth = bandwidth
        self.sidekiq_sidekiq_rx_0.set_rx_bandwidth(self.bandwidth)




def main(top_block_cls=source_test_debug, options=None):

    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()

    tb.show()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    qapp.exec_()

if __name__ == '__main__':
    main()
