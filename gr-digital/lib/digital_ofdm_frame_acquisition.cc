  /* -*- c++ -*- */
/*
 * Copyright 2006,2007,2008,2010 Free Software Foundation, Inc.
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

#include <digital_ofdm_frame_acquisition.h>
#include <gr_io_signature.h>
#include <gr_expj.h>
#include <gr_math.h>
#include <cstdio>

#include <gr_sincos.h>
#include <math.h>
#include <boost/math/special_functions/trunc.hpp>

// apurv++ starts - for logging //
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
// apurv++ ends - for logging //

#define VERBOSE 0
#define M_TWOPI (2*M_PI)
#define MAX_NUM_SYMBOLS 1000


static const pmt::pmt_t SYNC_TIME = pmt::pmt_string_to_symbol("sync_time");

digital_ofdm_frame_acquisition_sptr
digital_make_ofdm_frame_acquisition (unsigned int occupied_carriers, unsigned int fft_length, 
				unsigned int cplen,
				const std::vector<std::vector<gr_complex> > &known_symbol,
				//const std::vector<gr_complex> &known_symbol,
				unsigned int max_fft_shift_len)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_frame_acquisition (occupied_carriers, fft_length, cplen,
									known_symbol, max_fft_shift_len));
}

digital_ofdm_frame_acquisition::digital_ofdm_frame_acquisition (unsigned occupied_carriers, unsigned int fft_length, 
						      unsigned int cplen,
						      const std::vector<std::vector<gr_complex> > &known_symbol,
						      //const std::vector<gr_complex> &known_symbol,
						      unsigned int max_fft_shift_len)
  : gr_block ("ofdm_frame_acquisition",
	      gr_make_io_signature2 (2, 2, sizeof(gr_complex)*fft_length, sizeof(char)),   // apurv--: input
	      gr_make_io_signature3 (2, 3, sizeof(gr_complex)*occupied_carriers, sizeof(char), sizeof(gr_complex)*occupied_carriers)),	// apurv++: output
    d_occupied_carriers(occupied_carriers),
    d_fft_length(fft_length),
    d_cplen(cplen),
    d_freq_shift_len(max_fft_shift_len),
    d_known_symbol(known_symbol),
    d_coarse_freq(0),
    d_phase_count(0),
    d_known_norm(0)
{
  d_known_diff.resize(d_occupied_carriers-2);
  d_symbol_diff.resize(d_fft_length-2);
  d_hestimate = (gr_complex*) malloc(sizeof(gr_complex) * d_occupied_carriers);
  memset(d_hestimate, 0, sizeof(gr_complex) * d_occupied_carriers);

//raw	
#if 0
  const std::vector<gr_complex> &first = d_known_symbol[0];
  for(unsigned i = (pad() & 1); i < d_occupied_carriers-2; i+= 2) {
    // NOTE: only even frequencies matter
    // check pad to figure out which are odd
    d_known_diff[i] = first[i] * conj(first[i+2]);
    d_known_norm += norm(d_known_diff[i]);
  }
#else
  d_symbol_phase_diff.resize(d_fft_length);
  d_known_phase_diff.resize(d_occupied_carriers);
  unsigned int i = 0, j = 0;

  const std::vector<gr_complex> &first = d_known_symbol[0];
  std::fill(d_known_phase_diff.begin(), d_known_phase_diff.end(), 0);
  for(i = 0; i < first.size()-2; i+=2) {
    d_known_phase_diff[i] = norm(first[i] - first[i+2]);

    d_known_diff[i] = first[i] * conj(first[i+2]);
    d_known_norm += norm(d_known_diff[i]);
  }
#endif
}

digital_ofdm_frame_acquisition::~digital_ofdm_frame_acquisition(void)
{
  free(d_hestimate);
}

static const int LOOKAHEAD = 3;
void
digital_ofdm_frame_acquisition::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
  unsigned ninputs = ninput_items_required.size ();
  for (unsigned i = 0; i < ninputs; i++)
    ninput_items_required[i] = LOOKAHEAD;		// raw
}

gr_complex
digital_ofdm_frame_acquisition::coarse_freq_comp(float freq_delta, int symbol_count)
{
  return gr_expj((float)(-M_TWOPI*freq_delta*d_cplen)/d_fft_length*symbol_count);
}

inline gr_complex
digital_ofdm_frame_acquisition::compensate() const {
  double carrier = (M_TWOPI * d_coarse_freq * d_cur_symbol * d_cplen) / d_fft_length;
  return gr_expj(-carrier);
}


float
digital_ofdm_frame_acquisition::correlate(const gr_complex *symbol, int zeros_on_left)
{
#if 1 
  unsigned int i,j;
  
  std::fill(d_symbol_phase_diff.begin(), d_symbol_phase_diff.end(), 0);
  for(i = 0; i < d_fft_length-2; i++) {
    d_symbol_phase_diff[i] = norm(symbol[i] - symbol[i+2]);
    d_symbol_diff[i] = conj(symbol[i]) * symbol[i+2];					// from raw //
  }

  // sweep through all possible/allowed frequency offsets and select the best
  int index = 0;
  float max = 0, sum=0;
  for(i =  zeros_on_left - d_freq_shift_len; i < zeros_on_left + d_freq_shift_len; i++) {
    sum = 0;
    for(j = 0; j < d_occupied_carriers; j++) {
      sum += (d_known_phase_diff[j] * d_symbol_phase_diff[i+j]);
    }
    if(sum > max) {
      max = sum;
      index = i;
    }
  }
  
  // set the coarse frequency offset relative to the edge of the occupied tones
  d_coarse_freq = index - zeros_on_left;
  int d_old = d_coarse_freq;
  d_coarse_freq = 0; //hack apurv++
  printf("old: %d, new: %d\n", d_old, d_coarse_freq); fflush(stdout);
// raw
#else
  for(unsigned i = 0; i < d_fft_length-2; i++) {
    d_symbol_diff[i] = conj(symbol[i]) * symbol[i+2];
  }

  unsigned int pad = (d_fft_length - d_occupied_carriers + 1)/2;
  // sweep through all possible/allowed frequency offsets and select the best
  double best_sum = 0.0;
  unsigned int best_d = 0;

  for(unsigned d = 0; d < 2*pad-1; ++d) {
    gr_complex sum = 0.0;
    for(unsigned j = pad & 1; j < d_occupied_carriers-2; j+= 2) {
      sum += (d_known_diff[j] * d_symbol_diff[d+j]);
    }
    //std::cerr << "d = " << d << "\tM = " << abs(sum) << std::endl;
    double asum = abs(sum);
    if(asum > best_sum) {
      best_sum = asum;
      best_d = d;
    }
  }
  d_coarse_freq = best_d - pad;


  double norm_sum = 0.0;
  for(unsigned j = pad & 1; j < d_occupied_carriers-2; j+= 2) {
    norm_sum += norm(d_symbol_diff[best_d+j]);
  }
  float correlation = best_sum / sqrt(d_known_norm * norm_sum);
  return correlation;
#endif
}

void
digital_ofdm_frame_acquisition::calculate_equalizer(const gr_complex *symbol, int zeros_on_left)
{
  unsigned int i=0;
  const std::vector<gr_complex> &known_symbol = d_known_symbol[d_cur_symbol];
  gr_complex comp = coarse_freq_comp(d_coarse_freq, d_cur_symbol);
  //printf("calculate_equalizer, cur_symbol: %d, comp (%f, %f)\n", d_cur_symbol, comp.real(), comp.imag()); fflush(stdout);
 
  if(d_cur_symbol == 0) {
     // Set first tap of equalizer
     d_hestimate[0] = known_symbol[0] / 
        (coarse_freq_comp(d_coarse_freq,1)*symbol[zeros_on_left+d_coarse_freq]);

    // set every even tap based on known symbol
    // linearly interpolate between set carriers to set zero-filled carriers
    // FIXME: is this the best way to set this?
    for(i = 2; i < d_occupied_carriers; i+=2) {
       d_hestimate[i] = known_symbol[i] / 
         (coarse_freq_comp(d_coarse_freq,1)*(symbol[i+zeros_on_left+d_coarse_freq]));
       d_hestimate[i-1] = (d_hestimate[i] + d_hestimate[i-2]) / gr_complex(2.0, 0.0);    
    }

    // with even number of carriers; last equalizer tap is wrong
    if(!(d_occupied_carriers & 1)) {
       d_hestimate[d_occupied_carriers-1] = d_hestimate[d_occupied_carriers-2];
    }
  }
  else {
    for(i = 0; i < d_occupied_carriers; ++i) {
       gr_complex tx = known_symbol[i];
       gr_complex rx = symbol[i+zeros_on_left+d_coarse_freq];
       d_hestimate[i] = tx/(rx*comp);
    }
  }

  if(VERBOSE) {
    fprintf(stderr, "Equalizer setting:\n");
    for(i = 0; i < d_occupied_carriers; i++) {
      gr_complex sym = coarse_freq_comp(d_coarse_freq,d_cur_symbol)*symbol[i+zeros_on_left+d_coarse_freq];
      gr_complex output = sym * d_hestimate[i];
      fprintf(stderr, "sym: %+.4f + j%+.4f  ks: %+.4f + j%+.4f  eq: %+.4f + j%+.4f  ==>  %+.4f + j%+.4f\n", 
	      sym .real(), sym.imag(),
	      d_known_symbol[d_cur_symbol][i].real(), d_known_symbol[d_cur_symbol][i].imag(),
	      d_hestimate[i].real(), d_hestimate[i].imag(),
	      output.real(), output.imag());
    }
    fprintf(stderr, "\n");
  }
}

inline int
digital_ofdm_frame_acquisition::pad() const {
  // amount of FFT padding on the left
  return (d_fft_length - d_occupied_carriers + 1)/2 + d_coarse_freq;
}

void
digital_ofdm_frame_acquisition::init_estimate(const gr_complex *symbol) {
  for(unsigned int i = 0; i < d_occupied_carriers; ++i) {
    d_hestimate[i] = 0;
  }
}

void
digital_ofdm_frame_acquisition::update_estimate(const gr_complex *symbol)
{
  int p = pad();
  // take coarse frequency offset into account
  gr_complex comp = compensate();

  const std::vector<gr_complex> &known_symbol = d_known_symbol[d_cur_symbol];

  // set every even tap based on known symbol
  for(unsigned int i = 0; i < d_occupied_carriers; ++i) {
    gr_complex tx = known_symbol[i];
    gr_complex rx = symbol[i+p];
    d_hestimate[i] += tx / (rx * comp);
  }
}

void
digital_ofdm_frame_acquisition::finish_estimate()
{
  // just normalize
  unsigned int num_symbols = d_known_symbol.size() - 1; // the first one does not count
  for(unsigned int i = 0; i < d_occupied_carriers; ++i) {
    d_hestimate[i] /= num_symbols;;
  }
  float h_abs = 0.0f;
  float h_abs2 = 0.0f;
  float h_arg = 0.0f;
  float h_max = 0.0f;
  for(unsigned int i = 0; i < d_occupied_carriers; ++i) {
    if (i == (d_occupied_carriers+1)/2) {
      // skip the DC
      continue;
    }
    float aa = std::abs(d_hestimate[i]);
    h_abs += aa;
    h_abs2 += aa*aa;
    h_arg += std::arg(d_hestimate[i]); // FIXME: unwrap
    h_max = std::max(h_max, aa);
  }
  h_abs /= d_occupied_carriers-1;
  h_abs2 /= d_occupied_carriers-1;
  h_arg /= d_occupied_carriers-1;
  std::cerr << "H: phase = " << h_arg
            << "\tavg = " << h_abs
            << "\tmax = " << h_max
            << "\tSD = " << h_abs2 - h_abs*h_abs
            << std::endl;
}

int
digital_ofdm_frame_acquisition::general_work(int noutput_items,
					gr_vector_int &ninput_items,
					gr_vector_const_void_star &input_items,
					gr_vector_void_star &output_items)
{
  gr_complex *symbol = (gr_complex *)input_items[0];
  const char *signal_in = (const char *)input_items[1];

  gr_complex *out = (gr_complex *) output_items[0];
  char *signal_out = (char *) output_items[1];
  
  int unoccupied_carriers = d_fft_length - d_occupied_carriers;
  int zeros_on_left = (int)ceil(unoccupied_carriers/2.0);

  /* apurv++: log hestimates */
  gr_complex *hout = NULL;
  if(output_items.size() >= 3)
      hout = (gr_complex *) output_items[2];

// copied from raw version // 
#if 0
  int ninput = ninput_items[0];
  int nproduced = 0;
  int nconsumed = 0;

  while((nconsumed < ninput) && (nproduced < noutput_items)) {
    // first, try to determine if new frame
    bool newframe = false;
    float corr;
    *signal_out = 0;

    // regular mode
    if(*signal_in) {
       corr = correlate(symbol, zeros_on_left);
       //printf("corr: %f\n", corr); fflush(stdout);
       newframe = (corr > 0.9f);
       std::cerr << "correlation = " << corr << std::endl;
       if(newframe) {
          std::cerr << "correlation = " << corr << " " << "coarse_freq: " << d_coarse_freq << std::endl;
          printf("--- nconsumed: %d, ninput: %d, nproduced: %d, noutput_items: %d\n", nconsumed, ninput, nproduced, noutput_items); fflush(stdout);
       }
    }

    if(newframe) {
       printf("consumed: %d, d_cur_symbol: %d, ks.size: %d\n", nconsumed, d_cur_symbol, d_known_symbol.size()); fflush(stdout);
       d_cur_symbol=0;
       init_estimate(symbol);

       ++nproduced;
       ++nconsumed;
       ++signal_in;	

       *signal_out = 1;	++signal_out;
       memcpy(out, &symbol[pad()], sizeof(gr_complex) * d_occupied_carriers); out+= d_occupied_carriers;
       hout+=d_occupied_carriers;
       symbol+= d_fft_length;
       continue;
    }

    ++d_cur_symbol;
    if(d_cur_symbol < d_known_symbol.size()) {
      // use for equalization
      update_estimate(symbol);

      ++nproduced;
      ++nconsumed;
      ++signal_in;
      ++signal_out;
      memcpy(out, &symbol[pad()], sizeof(gr_complex) * d_occupied_carriers);
      out+= d_occupied_carriers;
      hout+= d_occupied_carriers;
      symbol+= d_fft_length;
      continue;
    }

    // time to produce

    if (d_cur_symbol == d_known_symbol.size()) {
      finish_estimate();
    }

    *signal_out = 0;

    int p = pad();
    gr_complex comp = compensate();
    memcpy(hout, d_hestimate, sizeof(gr_complex) * d_occupied_carriers);
    memcpy(out, &symbol[p], sizeof(gr_complex) * d_occupied_carriers);
    //assert(d_coarse_freq == 0);

    hout+= d_occupied_carriers;
    out+= d_occupied_carriers;
    ++signal_out;
    ++nproduced;

    ++nconsumed;
    ++signal_in;
    symbol+= d_fft_length;
  }
  consume_each(nconsumed);
  return nproduced;  
#else
  // existing version // 
  if(signal_in[0]) {
    d_cur_symbol = 0;
    d_phase_count = 1;
    correlate(symbol, zeros_on_left);
    //calculate_equalizer(symbol, zeros_on_left);
    signal_out[0] = 1;
  }
  else {
    signal_out[0] = 0;
  } 

  if(d_cur_symbol < d_known_symbol.size()) {
     calculate_equalizer(symbol, zeros_on_left);
  }
  d_cur_symbol++;
 

  if(output_items.size() >= 3) {
     memcpy(hout, d_hestimate, sizeof(gr_complex) * d_occupied_carriers);
     memcpy(out, &symbol[zeros_on_left], sizeof(gr_complex) * d_occupied_carriers);
     assert(d_coarse_freq == 0);
  }

  d_phase_count++;
  if(d_phase_count == MAX_NUM_SYMBOLS) {
    d_phase_count = 1;
  }

  //test_timestamp(1);				// enable to track tag propagation
  consume_each(1);
  return 1;
#endif
}

#if 0
/* just for debugging - tracking propagation of tags from sampler to sink */
inline void
digital_ofdm_frame_acquisition::test_timestamp(int output_items) {
  unsigned int tag_port = 1;
  std::vector<gr_tag_t> rx_sync_tags1;
  const uint64_t nread1 = nitems_read(tag_port);
  get_tags_in_range(rx_sync_tags1, tag_port, nread1, nread1+output_items, SYNC_TIME);

  printf("(ACQ) nread1: %llu, output_items: %d\n", nread1, output_items); fflush(stdout);
  if(rx_sync_tags1.size()>0) {
     size_t t = rx_sync_tags1.size()-1;
     uint64_t offset = rx_sync_tags1[t].offset;

     const pmt::pmt_t &value = rx_sync_tags1[t].value;
     uint64_t sync_secs = pmt::pmt_to_uint64(pmt_tuple_ref(value, 0));
     double sync_frac_of_secs = pmt::pmt_to_double(pmt_tuple_ref(value,1));
     printf("test_timestamp1 (ACQ):: found %d tags, offset: %llu, output_items: %d, nread1: %llu value[%llu, %f]\n", rx_sync_tags1.size(), rx_sync_tags1[t].offset, output_items, nread1, sync_secs, sync_frac_of_secs); fflush(stdout);
  } else {
     //std::cerr << "ACQ---- Header received, with no sync timestamp1?\n";
  }
}
#endif
