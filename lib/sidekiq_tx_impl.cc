/*
* Copyright 2018    US Naval Research Lab
* Copyright 2018    Epiq Solutions
*
*
* This is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 3, or (at your option)
* any later version.
*
* This software is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this software; see the file COPYING. If not, write to
* the Free Software Foundation, Inc., 51 Franklin Street,
* Boston, MA 02110-1301, USA.
*/

#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <pthread.h>
#include "sidekiq_tx_impl.h"

#define NUM_BLOCKS  20
static uint32_t complete_count;
// mutex to protect updates to the tx buffer
static pthread_mutex_t tx_buf_mutex;

// mutex and condition variable to signal when the tx queue may have room available
static pthread_mutex_t space_avail_mutex;
static pthread_cond_t space_avail_cond;

static void tx_complete( int32_t status, skiq_tx_block_t *p_data, void *p_user )
{
    if( status != 0 )
    {
        fprintf(stderr, "Error: packet %" PRIu32 " failed with status %d\n",
                complete_count, status);
    }

    // increment the packet completed count
    complete_count++;

    pthread_mutex_lock( &tx_buf_mutex );
    // update the in use status of the packet just completed
    if (p_user)
    {
        *(int32_t*)p_user = 0;
    }
     pthread_mutex_unlock( &tx_buf_mutex );

    // signal to the other thread that there may be space available now that a
    // packet send has completed
    pthread_mutex_lock( &space_avail_mutex );
    pthread_cond_signal(&space_avail_cond);
    pthread_mutex_unlock( &space_avail_mutex );

}

using namespace gr::sidekiq;
using pmt::pmt_t;

const pmt_t CONTROL_MESSAGE_PORT{pmt::string_to_symbol("command")};

const pmt_t TELEMETRY_MESSAGE_PORT{pmt::string_to_symbol("telemetry")};

const bool SIDEKIQ_IQ_PACK_MODE_UNPACKED{false};

static const double STATUS_UPDATE_RATE_SECONDS{1.0};

static const int16_t TX_ENABLE_REGISTER{0x0002};

static const int16_t TX_FILTER_CONFIGURATION_REGISTER{0x0065};


sidekiq_tx::sptr sidekiq_tx::make(
        int input_card_number,
        int handle,
		double sample_rate,
		double attenuation,
		double frequency,
		double bandwidth,
		int sync_type,
		bool suppress_tune_transients,
		uint8_t dataflow_mode,
		int buffer_size) {  
	return gnuradio::get_initial_sptr(
			new sidekiq_tx_impl(
                    input_card_number,
                    handle,
					sample_rate,
					attenuation,
					frequency,
					bandwidth,
					sync_type,
					suppress_tune_transients,
					dataflow_mode,
					buffer_size
			));
}

sidekiq_tx_impl::sidekiq_tx_impl(
        int input_card_number,
        int handle,
		double sample_rate,
		double attenuation,
		double frequency,
		double bandwidth,
		int sync_type,
		bool suppress_tune_transients,
		uint8_t dataflow_mode,
		int buffer_size) 
		: gr::sync_block(
		"sidekiq_tx",
		gr::io_signature::make(1, 2, sizeof(gr_complex)),
		gr::io_signature::make(0, 0, 0)),
		  sidekiq_tx_base{
                  input_card_number,
				  sync_type,
				  (skiq_tx_hdl_t)handle,
				  (skiq_tx_hdl_t)100,
				  gr::sidekiq::sidekiq_functions<skiq_tx_hdl_t>(
						  skiq_start_tx_streaming,
						  skiq_stop_tx_streaming,
						  skiq_write_tx_LO_freq,
						  skiq_read_tx_LO_freq,
						  skiq_write_tx_sample_rate_and_bandwidth,
						  skiq_read_tx_sample_rate_and_bandwidth,
						  skiq_read_curr_tx_timestamp,
						  skiq_read_rfic_tx_fir_config,
						  skiq_write_rfic_tx_fir_coeffs,
						  skiq_read_rfic_tx_fir_coeffs
				  )
		  },
		  dataflow_mode{static_cast<skiq_tx_flow_mode_t>(dataflow_mode)},
		  tx_buffer_size{static_cast<uint16_t>(buffer_size)} {

	set_tx_frequency(frequency);
	set_tx_sample_rate(sample_rate);
	set_tx_bandwidth(bandwidth);
	set_tx_suppress_tune_transients(suppress_tune_transients);
	set_tx_attenuation(attenuation);

    complete_count = 0;
    tx_buf_mutex = PTHREAD_MUTEX_INITIALIZER;
    space_avail_mutex = PTHREAD_MUTEX_INITIALIZER;
    space_avail_cond = PTHREAD_COND_INITIALIZER;

    //	get_filter_parameters();
	if (skiq_write_tx_data_flow_mode(card, hdl, this->dataflow_mode) != 0) {
		printf("Error: could not set TX dataflow mode\n");
        throw std::runtime_error("Failure: skiq_write_tx_flow_mode");
	}

    if (hdl == skiq_tx_hdl_A2 || hdl == skiq_tx_hdl_B2) {
        if (skiq_write_chan_mode(card, skiq_chan_mode_dual) != 0) {
            printf("Error: unable to configure TX channel mode\n");
            throw std::runtime_error("Failure: skiq_write_chan_mode");
        }
    } else {
        if (skiq_write_chan_mode(card, skiq_chan_mode_single) != 0) {
            printf("Error: unable to configure TX channel mode\n");
            throw std::runtime_error("Failure: skiq_write_chan_mode");
        }
    }
	if (skiq_write_tx_block_size(card, hdl, tx_buffer_size) != 0) {
		printf("Error: unable to configure TX block size: %d\n", tx_buffer_size);
        throw std::runtime_error("Failure: skiq_write_tx_block_size");
	}
	if (skiq_write_tx_transfer_mode(card, skiq_tx_hdl_A1, skiq_tx_transfer_mode_async) != 0) {
		printf("Error: unable to configure TX channel mode\n");
        throw std::runtime_error("Failure: skiq_write_tx_transfer_mode");
	}
    if( skiq_write_num_tx_threads(card, 4) != 0 ) {
		printf("Error: unable to configure TX number of threads\n");
        throw std::runtime_error("Failure: skiq_write_tx_transfer_mode");
	}
    if(skiq_register_tx_complete_callback( card, &tx_complete ) != 0){
		printf("Error: unable to configure TX number of threads\n");
        throw std::runtime_error("Failure: skiq_write_tx_transfer_mode");
	}


	if (skiq_write_iq_pack_mode(card, SIDEKIQ_IQ_PACK_MODE_UNPACKED) != 0) {
		printf("Error: unable to set iq pack mode to unpacked%d\n", SIDEKIQ_IQ_PACK_MODE_UNPACKED);
        throw std::runtime_error("Failure: skiq_write_iq_pack_mode");
	}

    p_tx_blocks = (skiq_tx_block_t **)calloc( NUM_BLOCKS, sizeof( skiq_tx_block_t * ));
    if( p_tx_blocks == NULL )
    {
        fprintf(stderr, "Error: failed to allocate memory for TX blocks.\n");
        throw std::runtime_error("Failure: calloc p_tx_blocks");
    }

    // allocate for # blocks
    p_tx_status = (int32_t *)calloc( NUM_BLOCKS, sizeof(*p_tx_status) );
    if( p_tx_status == NULL )
    {
        fprintf(stderr, "Error: failed to allocate memory for TX status.\n");
        throw std::runtime_error("Failure: calloc p_tx_status");
    }

    for (int i = 0; i < NUM_BLOCKS; i++)
    {
        /* allocate a transmit block by number of words */
        p_tx_blocks[i] = skiq_tx_block_allocate( tx_buffer_size );
        p_tx_status[i] = 0;
    }

    curr_block = 0;

	temp_buffer.resize(tx_buffer_size);


	auto alignment_multiple = static_cast<int>(volk_get_alignment() / sizeof(short));
	set_alignment(alignment_multiple);

	message_port_register_in(CONTROL_MESSAGE_PORT);
	set_msg_handler(CONTROL_MESSAGE_PORT, [this](pmt::pmt_t msg) { this->handle_control_message(msg); });
	message_port_register_out(TELEMETRY_MESSAGE_PORT);
}

bool sidekiq_tx_impl::start() {
	output_telemetry_message();
	if (skiq_start_tx_streaming(card, hdl) != 0) {
		printf("Error: could not start TX streaming\n");
        throw std::runtime_error("Failure: skiq_start_tx_streaming");
	}
	return block::start();
}

bool sidekiq_tx_impl::stop() {
	skiq_stop_tx_streaming(card, hdl);
	skiq_exit();
	return block::stop();
}

void sidekiq_tx_impl::set_tx_attenuation(double value) {
	if (skiq_write_tx_attenuation(card, hdl, static_cast<uint16_t>(value)) != 0) {
		printf("Error: could not set TX attenuation to %f\n", value);
        throw std::runtime_error("Failure: skiq_write_tx_attenuation");
	}
}

uint16_t sidekiq_tx_impl::get_tx_attenuation() {
	uint16_t result;
	if (skiq_read_tx_attenuation(card, hdl, &result) != 0) {
		printf("Error: could not get TX attenuation\n");
        throw std::runtime_error("Failure: skiq_read_tx_attenuation");
	}
	return result;
}

void sidekiq_tx_impl::set_tx_sample_rate(double value) {
	set_samplerate_bandwidth(static_cast<uint32_t>(value), bandwidth);
	status_update_rate_in_samples = static_cast<size_t >(sample_rate * STATUS_UPDATE_RATE_SECONDS);
}

void sidekiq_tx_impl::set_tx_bandwidth(double value) {
	set_samplerate_bandwidth(sample_rate, static_cast<uint32_t>(value));
}

void sidekiq_tx_impl::set_tx_suppress_tune_transients(bool value) {
	suppress_tune_transients = value;
}

void sidekiq_tx_impl::set_tx_frequency(double value) {
	if (suppress_tune_transients) {
		auto current_attenuation = get_tx_attenuation();
		set_tx_attenuation(SKIQ_MAX_TX_ATTENUATION);
		set_frequency(value);
		set_tx_attenuation(current_attenuation);
	} else {
		set_frequency(value);
	}
}

void sidekiq_tx_impl::set_tx_filter_override_taps(const std::vector<float> &taps) {
	//TODO: we need to be checking filter parameters and enforcing that current config tap length
	// is equal to tap length of custom filter we are trying to set

	float MAX_SIZE{65536.0f / 4.0f};

	filter_override_taps.clear();
	for (unsigned int count{0}; count < taps.size(); count++) {
		filter_override_taps.push_back(static_cast<int16_t>(taps[count] * MAX_SIZE));
		printf("%03d,%f,%05d\n", count, taps[count], filter_override_taps[count]);
	}
	if (!filter_override_taps.empty()) {
		set_filter_parameters(&filter_override_taps[0]);
	}
//	get_filter_parameters();
}

void sidekiq_tx_impl::output_telemetry_message() {
	message_port_pub(TELEMETRY_MESSAGE_PORT, get_telemetry_pmt());
}

//TODO: move this to pmt_helper class
double get_double_from_pmt_dict(pmt_t dict, pmt_t key, pmt_t not_found = pmt::PMT_NIL) {
	auto message_value = pmt::dict_ref(dict, key, not_found);

	return pmt::to_double(message_value);
}

void sidekiq_tx_impl::handle_control_message(pmt_t message) {
	if (pmt::dict_has_key(message, FREQ_KEY)) {
		set_tx_frequency(get_double_from_pmt_dict(message, FREQ_KEY));
	}
	if (pmt::dict_has_key(message, GAIN_KEY)) {
		set_tx_attenuation(get_double_from_pmt_dict(message, GAIN_KEY));
	}
	if (pmt::dict_has_key(message, RATE_KEY)) {
		set_tx_sample_rate(get_double_from_pmt_dict(message, RATE_KEY));
	}
	//TODO: timed freq change must be implemented before this will work
	if (pmt::dict_has_key(message, USRP_TIMED_COMMAND_KEY)) {
//		execute_timed_freq_change_command(message, true);
	}
}

void sidekiq_tx_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required) {
	(void)(noutput_items);
	ninput_items_required[0] = tx_buffer_size;
}

void sidekiq_tx_impl::update_tx_error_count() {
	uint32_t num_tx_errors;

	if (dataflow_mode == skiq_tx_immediate_data_flow_mode) {
		skiq_read_tx_num_underruns(card, hdl, &num_tx_errors);
		if (last_num_tx_errors != num_tx_errors) {
			printf("TX underrun count: %u\n", num_tx_errors);
			last_num_tx_errors = num_tx_errors;
		}
	} else {
		skiq_read_tx_num_late_timestamps(card, hdl, &num_tx_errors);
		printf("TX late burst count:  %u\n", num_tx_errors);
	}
}

void sidekiq_tx_impl::handle_tx_gain_tag(tag_t tag) {
	set_tx_attenuation(pmt::to_double(tag.value));
}

void sidekiq_tx_impl::handle_tx_freq_tag(tag_t tag) {
	set_tx_frequency(pmt::to_double(tag.value));
}

void sidekiq_tx_impl::handle_tx_time_tag(tag_t tag) {
	uint64_t seconds{pmt::to_uint64(pmt::tuple_ref(tag.value, 0))};
	double fractional{pmt::to_double(pmt::tuple_ref(tag.value, 1))};
	auto time = static_cast<double>(seconds) + fractional;
	timestamp = static_cast<uint64_t>(time * sample_rate);
}

void sidekiq_tx_impl::handle_tx_burst_length_tag(tag_t tag) {
	burst_length = pmt::to_uint64(tag.value);
	check_burst_length(tag.offset, burst_length);
	burst_samples_sent = 0;
}

void sidekiq_tx_impl::check_burst_length(size_t current_tag_offset, size_t burst_sample_length) {
	auto burst_tag_offset_delta = current_tag_offset - previous_burst_tag_offset;
	auto result = static_cast<int64_t>(burst_sample_length - burst_tag_offset_delta);

	//TODO: This can just be burst_sample_length != burst_tag_offset_delta. TEST THIS BEFORE CHANGING!!!
	if (result != 0 && previous_burst_tag_offset != 0) {
		printf(
				"Burst Tag Error - (Actual,Expected): %ld, %ld between tags  %ld:%ld\n",
				burst_tag_offset_delta,
				burst_sample_length,
				previous_burst_tag_offset,
				current_tag_offset
		);
	}
	previous_burst_tag_offset = current_tag_offset;
}



//TODO:
//1. enable gps timestamp on next pps
//2. handle message commands (done)
//3. implement timed freq change
//4. implement timed bursts	(done)
//5. test out async mode
//6. further investiate setting custom filter taps
//7. handle freq/gain/etc on tx tags (done)
//8. suppress tx tune transients (done)
//9. PID controller for txvco warp to discipline to PPS
int sidekiq_tx_impl::work(
		int noutput_items,
		gr_vector_const_void_star &input_items,
		gr_vector_void_star &output_items) {
	unsigned int input_port{};
	auto in = static_cast<const gr_complex *>(input_items[input_port]);
	int samples_written{};
	int32_t result;
	std::vector<tag_t> tags;
	(void)(output_items);
	
	//TODO: this will not work when noutput_items < tx_buffer_size, it will return 0. Fix it...
	auto ninput_items = noutput_items - (noutput_items % tx_buffer_size);
	
	if (nitems_read(input_port) - last_status_update_sample > status_update_rate_in_samples) {
		update_tx_error_count();
		last_status_update_sample = nitems_read(input_port);
	}

	get_tags_in_range(tags, input_port, nitems_read(input_port), nitems_read(input_port) + ninput_items);
	std::sort(tags.begin(), tags.end(), tag_t::offset_compare);
	BOOST_FOREACH(
			const tag_t &tag, tags) {
					if (pmt::equal(tag.key, TX_TIME_KEY)) {
						handle_tx_time_tag(tag);
					} else if (pmt::equal(tag.key, TX_BURST_LENGTH_KEY)) {
						handle_tx_burst_length_tag(tag);
					} else if (pmt::equal(tag.key, TX_FREQ_KEY)) {
						handle_tx_freq_tag(tag);
					} else if (pmt::equal(tag.key, TX_GAIN_KEY)) {
						handle_tx_gain_tag(tag);
					}
				}

	while (samples_written < ninput_items) {
		volk_32f_s32f_multiply_32f(
				reinterpret_cast<float *>(&temp_buffer[0]),
				reinterpret_cast<const float *>(in),
				dac_scaling,
				static_cast<unsigned int>(tx_buffer_size * 2));
		volk_32fc_convert_16ic(
				reinterpret_cast<lv_16sc_t *>(p_tx_blocks[curr_block]->data),
				reinterpret_cast<const lv_32fc_t*>(&temp_buffer[0]),
				tx_buffer_size);
		
		skiq_tx_set_block_timestamp(p_tx_blocks[curr_block], timestamp);

		result = skiq_transmit(card, hdl, p_tx_blocks[curr_block], &(p_tx_status[curr_block]));
        /* check to see if the TX queue is full, if so wait for a released buffer */ 
        if( result == SKIQ_TX_ASYNC_SEND_QUEUE_FULL )
        {
            // update the in use status since we didn't actually send it yet
            pthread_mutex_lock( &tx_buf_mutex );
            p_tx_status[curr_block] = 0;
            pthread_mutex_unlock( &tx_buf_mutex );

            pthread_mutex_lock( &space_avail_mutex );
            pthread_cond_wait( &space_avail_cond, &space_avail_mutex );
            pthread_mutex_unlock( &space_avail_mutex );
        }
        else if ( result != 0 ) {
			printf("Info: sidekiq transmit failed with error: %d\n", result);
		} else {
            samples_written += tx_buffer_size;
            in += tx_buffer_size;
            timestamp += tx_buffer_size;
            curr_block = (curr_block + 1) % NUM_BLOCKS;
        }
	}
	
	return samples_written;
}
