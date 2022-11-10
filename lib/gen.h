/* -*- c++ -*- */
/*
 * Copyright 2022 gr-sidekiq author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIDEKIQ_SIDEKIQ_TX_IMPL_H
#define INCLUDED_SIDEKIQ_SIDEKIQ_TX_IMPL_H

#include <gnuradio/sidekiq/sidekiq_tx.h>

namespace gr {
namespace sidekiq {

class sidekiq_tx_impl : public sidekiq_tx
{
private:
    // Nothing to declare in this block.

public:
    sidekiq_tx_impl();
    ~sidekiq_tx_impl();

    // Where all the action really happens
    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);
};

} // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_SIDEKIQ_TX_IMPL_H */
