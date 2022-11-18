/* -*- c++ -*- */
/*
 * Copyright 2022 gr-sidekiq author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIDEKIQ_SIDEKIQ_TX_H
#define INCLUDED_SIDEKIQ_SIDEKIQ_TX_H

#include <pmt/pmt.h>
#include <gnuradio/sidekiq/api.h>
#include <gnuradio/sync_block.h>

using pmt::pmt_t;

namespace gr {
namespace sidekiq {

    static const pmt_t TX_TIME_KEY{pmt::string_to_symbol("tx_time")};

    static const pmt_t TX_BURST_KEY{pmt::string_to_symbol("tx_burst")};

    static const pmt_t TX_FREQ_KEY{pmt::string_to_symbol("tx_freq")};

    static const pmt_t TX_GAIN_KEY{pmt::string_to_symbol("tx_gain")};

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
    static sptr make(
                        int card,
                        int handle,
                        double sample_rate,
                        double bandwidth,
                        double frequency,
                        double attenuation,
                        int threads,
                        int buffer_size,
                        int cal_mode);

            virtual void set_tx_sample_rate(double value) = 0;

            virtual void set_tx_attenuation(double value) = 0;

            virtual void set_tx_frequency(double value) = 0;

            virtual void set_tx_bandwidth(double value) = 0;

            virtual void set_tx_cal_mode(int value) = 0;

            virtual void run_tx_cal(int value) = 0;


};

} // namespace sidekiq
} // namespace gr

#endif /* INCLUDED_SIDEKIQ_SIDEKIQ_TX_H */
