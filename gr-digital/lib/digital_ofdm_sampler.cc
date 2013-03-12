/* -*- c++ -*- */
/*
 * Copyright 2007,2008,2010 Free Software Foundation, Inc.
 * 
 * This file is part of GNU Radio
 * 
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

// george includes
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <gruel/pmt.h>
static const pmt::pmt_t TIME_KEY = pmt::pmt_string_to_symbol("rx_time");
static const pmt::pmt_t SYNC_TIME = pmt::pmt_string_to_symbol("sync_time");
#define VERBOSE 0
// Keep track of the RX timestamp
double lts_frac_of_secs;
uint64_t lts_secs;
uint64_t lts_samples_since;

uint64_t last_sync_sec;
double last_sync_frac_sec;

#include <digital_ofdm_sampler.h>
#include <gr_io_signature.h>
#include <gr_expj.h>
#include <cstdio>
#include <boost/math/special_functions/trunc.hpp>
#include <math.h>
#include <gr_message.h>

// apurv for logging the preamble //
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdexcept>
#include <stdio.h>

#ifdef HAVE_IO_H
#include <io.h>
#endif
#ifdef O_BINARY
#define OUR_O_BINARY O_BINARY
#else
#define OUR_O_BINARY 0
#endif

#ifdef O_LARGEFILE
#define OUR_O_LARGEFILE O_LARGEFILE
#else
#define OUR_O_LARGEFILE 0
#endif
// apurv for logging ends //

digital_ofdm_sampler_sptr
digital_make_ofdm_sampler (unsigned int fft_length, 
		      unsigned int symbol_length,
		      unsigned int num_preambles,
		      unsigned int timeout)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_sampler (fft_length, symbol_length, num_preambles, timeout));
}

digital_ofdm_sampler::digital_ofdm_sampler (unsigned int fft_length, 
				  unsigned int symbol_length,
				  unsigned int num_preambles,
				  unsigned int timeout)
  : gr_block ("ofdm_sampler",
	      gr_make_io_signature2 (2, 2, sizeof (gr_complex), sizeof(char)),
	      gr_make_io_signature3 (2, 3, sizeof (gr_complex)*fft_length, sizeof(char), sizeof(char)*fft_length)),
    d_state(STATE_NO_SIG), d_timeout_max(timeout), d_fft_length(fft_length), d_symbol_length(symbol_length),
    d_fp(NULL), d_fd(0), d_file_opened(false), d_num_preambles(num_preambles) 
{
  lts_samples_since=0;
  set_relative_rate(1.0/(double) fft_length);   // buffer allocator hint

  last_sync_sec = 0;
  last_sync_frac_sec = 0.0;
  d_prev_index = 0;

  std::string arg("");
  d_usrp = uhd::usrp::multi_usrp::make(arg);
  d_joint_rx_on = false; d_prev_trigger_pos = 0;
  d_phase = 0;
}

void
digital_ofdm_sampler::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
  // FIXME do we need more
  //int nreqd  = d_symbol_length + d_fft_length;
  int nreqd  = noutput_items*d_symbol_length;
  unsigned ninputs = ninput_items_required.size ();
  for (unsigned i = 0; i < ninputs; i++)
    ninput_items_required[i] = nreqd;
}

#if 0
int
digital_ofdm_sampler::general_work (int noutput_items,
			       gr_vector_int &ninput_items,
			       gr_vector_const_void_star &input_items,
			       gr_vector_void_star &output_items)
{
  gr_complex *iptr = (gr_complex *) input_items[0];
  const char *trigger = (const char *) input_items[1];

  gr_complex *optr = (gr_complex *) output_items[0];
  char *outsig = (char *) output_items[1];
  outsig[0] = 0;


  // 3rd output , char*fft //
  char *outsig2 = NULL;
  if(output_items.size() >= 3) 
     outsig2 = (char*) output_items[2];

  //FIXME: we only process a single OFDM symbol at a time; after the preamble, we can 
  // process a few at a time as long as we always look out for the next preamble.

  unsigned int index=d_fft_length;  // start one fft length into the input so we can always look back this far

  outsig2[0] = 0; // set output to no signal by default

  unsigned int delta = 0;

  // Search for a preamble trigger signal during the next symbol length
  while((d_state != STATE_PREAMBLE) && (index <= (d_symbol_length+d_fft_length))) {
    if(trigger[index] && !d_joint_rx_on) {

      unsigned int allowed_misalignment = 4;
      unsigned int gap = (((sizeof(MULTIHOP_HDR_TYPE) * 8)/MAX_DATA_CARRIERS)+d_num_preambles) * (d_symbol_length);		// 2 preambles
      unsigned left_boundary = gap - allowed_misalignment;
      unsigned right_boundary = gap + allowed_misalignment;

      uint64_t samples_passed = lts_samples_since + index;
      unsigned int obs_gap = samples_passed - d_prev_index;

      if(obs_gap > left_boundary && obs_gap < right_boundary) {
	  samples_passed = d_prev_index + gap;
	  index += (gap - obs_gap);
	  delta = gap - obs_gap; 
	  printf("index: %u, gap: %d, obs_gap: %d, delta: %d\n", index, gap, obs_gap, delta); fflush(stdout);
	  d_joint_rx_on = true;						// hack: to disallow the false spikes when multiple senders are involved in this tx //
      }


      d_prev_index = samples_passed;


      outsig[0] = 1;
      outsig2[0] = 1; // tell the next block there is a preamble coming
      d_state = STATE_PREAMBLE;
    }
    else
      index++;
  }

  unsigned int i, pos, ret;
  switch(d_state) {
  case(STATE_PREAMBLE):
    //printf("Set SYMBOL BOUNDARY here .. freq_offset: %.3f\n", freq_offset[index]); fflush(stdout);
    for(i = (index - d_fft_length + 1); i <= index; i++) {
      *optr++ = iptr[i];
    }
   
    d_timeout = d_timeout_max; // tell the system to expect at least this many symbols for a frame
    d_state = STATE_FRAME;
 
    consume_each(index - d_fft_length + 1); // consume up to one fft_length away to keep the history
    lts_samples_since += index - d_fft_length + 1;
    ret = 1;
    break;
    
  case(STATE_FRAME):
    // skip over fft length history and cyclic prefix
    pos = d_symbol_length;         // keeps track of where we are in the input buffer
 
    while(pos < d_symbol_length + d_fft_length) {
      *optr++ = iptr[pos++];
    }

    if(d_timeout-- == 0) {
      printf("TIMEOUT\n");
      d_state = STATE_NO_SIG;
    }

    consume_each(d_symbol_length); // jump up by 1 fft length and the cyclic prefix length
    lts_samples_since += d_symbol_length;
    ret = 1;
    break;

  case(STATE_NO_SIG):
  default:
    consume_each(index-d_fft_length); // consume everything we've gone through so far leaving the fft length history
    lts_samples_since += index-d_fft_length;
    ret = 0;
    d_joint_rx_on = false;
    break;
  }

  return ret;
}

#else
int
digital_ofdm_sampler::general_work (int noutput_items,
                                gr_vector_int &ninput_items,
                                gr_vector_const_void_star &input_items,
                                gr_vector_void_star &output_items)
{
  const gr_complex *in = (const gr_complex *) input_items[0];
  const char *trigger = (const char *) input_items[1];

  gr_complex *out = (gr_complex *) output_items[0];
  char *outsig = (char *) output_items[1];

#if 0
  gr_complex *outsig2 = NULL;
  if(output_items.size() >= 3) 
     outsig2 = (char*) output_items[2];
#endif

  int cp_length = d_symbol_length - d_fft_length;

// trigger[t] != 0 indicates that the input should be sampled here.
// Next input should be sampled d_symbol_length later.

  int nin = 0;
  int nout = 0;

  outsig[nout] == 0;

  while((nin < ninput_items[0] - d_symbol_length)
     && (nout < noutput_items)) {

    if (d_state == STATE_SIGNAL) {
      if(--d_timeout == 0) {
#if 1 
        std::cerr << "TIMEOUT" << std::endl;
#endif
        d_state = STATE_NO_SIG;
	d_joint_rx_on = false;
      } else {
        // copy symbol to output
        // (it could be a spurious last symbol, but we'd better be safe and include it)
        memcpy(out + nout * d_fft_length,
               in + nin,
               d_fft_length*sizeof(gr_complex));
        ++nout;
        outsig[nout] = 0;
      }
    }

    nin += d_symbol_length;
    uint64_t samples_passed = d_symbol_length;

    // look for trigger
    for(int j = nin - d_symbol_length, index=0; j < nin; ++j, index++) {
      if(trigger[j] && !d_joint_rx_on) {
        if (j + d_symbol_length <= nin + cp_length) {
          // we don't allow symbols shorter than cp_length
          if (nout) {
            if (outsig[nout-1]) // haven't we just copied it out?
              continue; // FIXME: this is edge programming!! :-(
            --nout; // overwrite the most recent output symbol
          }
        }

#if 1
	uint64_t curr_trigger_pos = lts_samples_since + index;			
	uint64_t obs_gap = curr_trigger_pos - d_prev_trigger_pos;

	int allowed_misalignment = cp_length+5;
        int gap = (((sizeof(MULTIHOP_HDR_TYPE) * 8)/MAX_DATA_CARRIERS)+d_num_preambles) * (d_symbol_length);             // 2 preambles
        int left = gap - allowed_misalignment;
        int right = gap + allowed_misalignment;

	if(obs_gap > left && obs_gap < right) {
           j += (gap - obs_gap);
	   index += (gap - obs_gap);

           d_joint_rx_on = true; 
	   curr_trigger_pos = lts_samples_since+index;				// adjust trigger pos
	   printf("j: %d, nin: %d, true_gap: %d, obs_gap: %llu, delta: %d\n", j, nin, gap, obs_gap, gap-obs_gap); fflush(stdout);
	}

	samples_passed = index;
	printf("joint_rx: %d, obs_gap: %llu, gap: %d, num_preambles: %d, prev_trigger: %llu, curr_trigger: %llu, index: %d\n", d_joint_rx_on, obs_gap, gap, d_num_preambles, d_prev_trigger_pos, curr_trigger_pos, index); fflush(stdout);
	d_prev_trigger_pos = curr_trigger_pos; 
#endif

        // set new sampling offset
        nin = j;
        outsig[nout] = 1; // indicate preamble
	printf("nout: %d\n", nout); fflush(stdout);
        d_state = STATE_SIGNAL;
        d_timeout = d_timeout_max;
        break;
      }
    }
    lts_samples_since += samples_passed;
  }
  consume_each(nin);
  return nout;
}
#endif

void
digital_ofdm_sampler::log_preamble(gr_complex *iptr, long index) {
  if(!d_file_opened)
  {
      d_file_opened = open_log();
      assert(d_file_opened);
      fprintf(stderr, "preambles file opened!\n");
  }
  assert(d_fp != NULL);
  int count = ftell(d_fp);
  count = fwrite_unlocked(iptr + index, sizeof(gr_complex), d_fft_length, d_fp);	// to log preamble
  //count = fwrite_unlocked(iptr + index, sizeof(gr_complex), d_symbol_length, d_fp);	// log anything ;)
}

bool
digital_ofdm_sampler::open_log()
{
  printf("open_log called\n"); fflush(stdout);

  // open the preamble log file //
  char *filename = "preamble.dat";
  if ((d_fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename);
     return false;
  }
  else {
      if((d_fp = fdopen (d_fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "preambles file cannot be opened\n");
            close(d_fd);
            return false;
      }
  }
  return true;
}

inline void
digital_ofdm_sampler::correct_freq_offset(gr_complex *out, int noutput_items, float phase) {
  float sensitivity = -2.0/((float) d_fft_length);
  for (int i = 0; i < noutput_items; i++){
    d_phase = d_phase + sensitivity * phase;
    float oi, oq;
    gr_sincosf (d_phase, &oq, &oi);
    out[i] *= gr_complex (oi, oq);
  }

  // Limit the phase accumulator to [-16*pi,16*pi]
  // to avoid loss of precision in the addition above.

  if (fabs (d_phase) > 16 * M_PI){
    double ii = boost::math::trunc (d_phase / (2 * M_PI));
    d_phase = d_phase - (ii * 2 * M_PI);
  }
}
