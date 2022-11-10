/* -*- c++ -*- */
/*
 * Copyright 2022 gr-sidekiq author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "sidekiq_tx_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace sidekiq {

#pragma message("set the following appropriately and remove this warning")
using input_type = float;
sidekiq_tx::sptr sidekiq_tx::make()
{
    return gnuradio::make_block_sptr<sidekiq_tx_impl>();
}


/*
 * The private constructor
 */
sidekiq_tx_impl::sidekiq_tx_impl()
    : gr::sync_block("sidekiq_tx",
                     gr::io_signature::make(
                         1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
                     gr::io_signature::make(0, 0, 0))
{
}

/*
 * Our virtual destructor.
 */
sidekiq_tx_impl::~sidekiq_tx_impl() {}

int sidekiq_tx_impl::work(int noutput_items,
                          gr_vector_const_void_star& input_items,
                          gr_vector_void_star& output_items)
{
    auto in = static_cast<const input_type*>(input_items[0]);

#pragma message("Implement the signal processing in your block and remove this warning")
    // Do <+signal processing+>

    // Tell runtime system how many output items we produced.
    return noutput_items;
}

} /* namespace sidekiq */
} /* namespace gr */
