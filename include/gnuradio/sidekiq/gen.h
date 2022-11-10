/* -*- c++ -*- */
/*
 * Copyright 2022 gr-sidekiq author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIDEKIQ_SIDEKIQ_TX_H
#define INCLUDED_SIDEKIQ_SIDEKIQ_TX_H

#include <gnuradio/sidekiq/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace sidekiq {

/*!
 * \brief <+description of block+>
 * \ingroup sidekiq
 *
 */
class SIDEKIQ_API sidekiq_tx : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<sidekiq_tx> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of sidekiq::sidekiq_tx.
     *
     * To avoid accidental use of raw pointers, sidekiq::sidekiq_tx's
     * constructor is in a private implementation
     * class. sidekiq::sidekiq_tx::make is the public interface for
     * creating new instances.
     */
    static sptr make();
};

} // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_SIDEKIQ_TX_H */
