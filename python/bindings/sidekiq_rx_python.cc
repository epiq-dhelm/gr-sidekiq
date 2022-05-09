/*
 * Copyright 2022 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually edited  */
/* The following lines can be configured to regenerate this file during cmake      */
/* If manual edits are made, the following tags should be modified accordingly.    */
/* BINDTOOL_GEN_AUTOMATIC(0)                                                       */
/* BINDTOOL_USE_PYGCCXML(0)                                                        */
/* BINDTOOL_HEADER_FILE(sidekiq_rx.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(c5f31a8d40f4b50d13770bb135489c3c)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <sidekiq/sidekiq_rx.h>
// pydoc.h is automatically generated in the build directory
#include <sidekiq_rx_pydoc.h>

void bind_sidekiq_rx(py::module& m)
{

    using sidekiq_rx    = ::gr::sidekiq::sidekiq_rx;


    py::class_<sidekiq_rx, gr::sync_block, gr::block, gr::basic_block,
        std::shared_ptr<sidekiq_rx>>(m, "sidekiq_rx", D(sidekiq_rx))

        .def(py::init(&sidekiq_rx::make),
           py::arg("input_card_number"),
           py::arg("port1_handle"),
           py::arg("port2_handle"),
           py::arg("sample_rate"),
           py::arg("gain"),
           py::arg("gain_mode"),
           py::arg("frequency"),
           py::arg("bandwidth"),
           py::arg("sync_type"),
           D(sidekiq_rx,make)
        )
        




        
        .def("set_rx_sample_rate",&sidekiq_rx::set_rx_sample_rate,       
            py::arg("value"),
            D(sidekiq_rx,set_rx_sample_rate)
        )


        
        .def("set_rx_gain",&sidekiq_rx::set_rx_gain,       
            py::arg("value"),
            D(sidekiq_rx,set_rx_gain)
        )


        
        .def("set_rx_frequency",&sidekiq_rx::set_rx_frequency,       
            py::arg("value"),
            D(sidekiq_rx,set_rx_frequency)
        )


        
        .def("set_rx_bandwidth",&sidekiq_rx::set_rx_bandwidth,       
            py::arg("value"),
            D(sidekiq_rx,set_rx_bandwidth)
        )


        
        .def("set_rx_filter_override_taps",&sidekiq_rx::set_rx_filter_override_taps,       
            py::arg("taps"),
            D(sidekiq_rx,set_rx_filter_override_taps)
        )

        ;




}








