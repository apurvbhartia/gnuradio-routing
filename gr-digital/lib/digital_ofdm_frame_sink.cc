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

#include <digital_ofdm_frame_sink.h>
#include <gr_io_signature.h>
#include <gr_expj.h>
#include <gr_math.h>
#include <math.h>
#include <cstdio>
#include <stdexcept>
#include <iostream>
#include <string.h>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>

#include <digital_crc32.h>
#include <gr_uhd_usrp_source.h>

using namespace std;

#define SEND_ACK_ETHERNET 1
#define UNASSIGNED 100

#define VERBOSE 0
#define LOG_H 1

//#define SCALE 1e3

static const pmt::pmt_t SYNC_TIME = pmt::pmt_string_to_symbol("sync_time");
int last_noutput_items;

//uhd_usrp_source_sptr 
static boost::shared_ptr<uhd_usrp_source> d_usrp_rx; //uhd_make_usrp_source(arg1, arg2);

inline void
digital_ofdm_frame_sink::enter_search()
{
  if (VERBOSE)
    fprintf(stderr, "@ enter_search\n");

  d_state = STATE_SYNC_SEARCH;
  d_curr_ofdm_symbol_index = 0;
  d_preamble_cnt = 0;
}

/* called everytime a preamble sync is received
   to allow multiple senders: do not reset the phase, slope, etc since they are used during demodulation */
inline void
digital_ofdm_frame_sink::enter_have_sync()
{
  if (VERBOSE)
    fprintf(stderr, "@ enter_have_sync\n");

  d_state = STATE_HAVE_SYNC;

  // clear state of demapper
  d_byte_offset = 0;
  d_partial_byte = 0;

  // apurv++ clear the header details //
  d_hdr_byte_offset = 0;
  memset(d_header_bytes, 0, HEADERBYTELEN);
  memset(&d_header, 0, HEADERBYTELEN);

  d_hdr_ofdm_index = 0;
  fill(d_dfe.begin(), d_dfe.end(), gr_complex(1.0,0.0));

}

inline void
digital_ofdm_frame_sink::enter_have_header() {
  assert(d_fwd || d_dst);
  d_state = STATE_HAVE_HEADER;
  d_curr_ofdm_symbol_index = 0;
  d_num_ofdm_symbols = ceil(((float) (d_packetlen * 8))/(d_data_carriers.size() * d_data_nbits));
  assert(d_num_ofdm_symbols < MAX_OFDM_SYMBOLS);                                // FIXME - arbitrary '70'.. 
  printf("d_num_ofdm_symbols: %d, actual sc size: %d, d_data_nbits: %d\n", d_num_ofdm_symbols, d_data_carriers.size(), d_data_nbits);
  fflush(stdout);

  d_pktInfo = createPktInfo();
  d_pktInfo->symbols = (gr_complex*) malloc(sizeof(gr_complex) * d_num_ofdm_symbols * d_occupied_carriers);
  memset(d_pktInfo->symbols, 0, sizeof(gr_complex) * d_num_ofdm_symbols * d_occupied_carriers);
  printf("allocated pktInfo rxSymbols\n"); fflush(stdout);
}

inline void
digital_ofdm_frame_sink::reset_demapper() {
  printf("reset demapper\n"); fflush(stdout);
  // clear state of demapper
  d_byte_offset = 0;
  d_partial_byte = 0;

  memset(d_freq, 0, sizeof(float) * MAX_SENDERS);
  memset(d_phase, 0, sizeof(float) * MAX_SENDERS);

  fill(d_dfe.begin(), d_dfe.end(), gr_complex(1.0,0.0));
}

/* extract the header info
   the header for now includes: 
   - pkt-type           (1 bit)
   - pktlen             (12 bits)
   - n_senders          (3 bits)
   - batch-number       (8 bits)
   - rx-id              (4 bits)
   - src-id             (4 bits)
   - coeffs             (4*batchsize)
   P.S: CRC has already been extracted by now and verified!

 --- 1 ---  ---------- 12 ---- ----- 3 ----- ------- 8 ------------- 4 ------   ------- 4 -------
|| pkt_type ||     pkt_len    || n_senders ||   batch_number ||    rx-id      ||     src-id     ||
---------------------------------------------------------------
*/
inline void
digital_ofdm_frame_sink::extract_header()
{
  d_packet_whitener_offset = 0;                 // apurv++: hardcoded!

  d_pkt_type = d_header.pkt_type;
  d_flow = d_header.flow_id;

  d_packetlen = d_header.packetlen;
  d_batch_number = d_header.batch_number;

  d_pkt_num = d_header.pkt_num;
  d_prevLinkId = d_header.link_id;
  printf("\tpkt_num: %d \t\t\t\t batch_num: %d \t\t\t\t len: %d src: %d prev-link: %d\n", d_header.pkt_num, d_header.batch_number, d_packetlen, d_header.src_id, d_prevLinkId); fflush(stdout);

  if (VERBOSE || 1)
    fprintf(stderr, " hdr details: (src: %d), (rx: %d), (batch_num: %d), (d_nsenders: %d), (d_packetlen: %d), (d_pkt_type: %d), (prev_hop: %d)\n",
                    d_header.src_id, d_header.dst_id, d_batch_number, d_nsenders, d_packetlen, d_header.pkt_type, d_header.prev_hop_id);

  // do appropriate conversions
  d_dst_id = ((int) d_header.dst_id) + '0';
  d_src_id = ((int) d_header.src_id) + '0';
  d_prev_hop_id = ((int) d_header.prev_hop_id) + '0';

  // extract coeffs if PRO //
  if(d_proto == PRO) {
     FlowInfo *fInfo = getFlowInfo(false, d_flow);
     for(int i = 0; i < d_batch_size; i++) {
	fInfo->coeffs[i] = d_header.coeffs[i];
	printf("coeff[%d] : %d\n", i, fInfo->coeffs[i]); fflush(stdout);
     }
  }
  
  printf("prev_hop: %c, src: %c, dst: %c\n", d_prev_hop_id, d_src_id, d_dst_id); fflush(stdout);
}

unsigned char digital_ofdm_frame_sink::slicer(const gr_complex x, const std::vector<gr_complex> &sym_position,
                                      		  const std::vector<unsigned char> &sym_value_out)
{
  unsigned int table_size = sym_value_out.size();
  unsigned int min_index = 0;
  float min_euclid_dist = norm(x - sym_position[0]);
  float euclid_dist = 0;

  for (unsigned int j = 1; j < table_size; j++){
    euclid_dist = norm(x - sym_position[j]);
    if (euclid_dist < min_euclid_dist){
      min_euclid_dist = euclid_dist;
      min_index = j;
    }
  }
#if 0
  gr_complex closest = d_sym_position[min_index];
  printf("x: (%f, %f), closest: (%f, %f), evm: (%f, %f)\n", x.real(), x.imag(), closest.real(), closest.imag(), min_euclid_dist); fflush(stdout);
#endif
  return sym_value_out[min_index];
}

/* uses pilots - from rawofdm */
void digital_ofdm_frame_sink::equalize_interpolate_dfe(const gr_complex *in, gr_complex *out) 
{
  gr_complex phase_error = 0.0;

  float cur_pilot = 1.0;
  for (unsigned int i = 0; i < d_pilot_carriers.size(); i++) {
    gr_complex pilot_sym(cur_pilot, 0.0);
    cur_pilot = -cur_pilot;
    int di = d_pilot_carriers[i];
    phase_error += (in[di] * conj(pilot_sym));
  } 

  int sender_index = 0;
  // update phase equalizer
  float angle = arg(phase_error);
  d_freq[sender_index] = d_freq[sender_index] - d_freq_gain*angle;
  d_phase[sender_index] = d_phase[sender_index] + d_freq[sender_index] - d_phase_gain*angle;
  if (d_phase[sender_index] >= 2*M_PI) d_phase[sender_index] -= 2*M_PI;
  else if (d_phase[sender_index] <0) d_phase[sender_index] += 2*M_PI;

  gr_complex carrier = gr_expj(-angle);

  // update DFE based on pilots
  cur_pilot = 1.0;
  for (unsigned int i = 0; i < d_pilot_carriers.size(); i++) {
    gr_complex pilot_sym(cur_pilot, 0.0);
    cur_pilot = -cur_pilot;
    int di = d_pilot_carriers[i];
    gr_complex sigeq = in[di] * carrier * d_dfe[i];
    // FIX THE FOLLOWING STATEMENT
    if (norm(sigeq)> 0.001)
      d_dfe[i] += d_eq_gain * (pilot_sym/sigeq - d_dfe[i]);
  }

  // equalize all data using interpolated dfe and demap into bytes
  unsigned int pilot_index = 0;
  int pilot_carrier_lo = 0;
  int pilot_carrier_hi = d_pilot_carriers[0];
  gr_complex pilot_dfe_lo = d_dfe[0];
  gr_complex pilot_dfe_hi = d_dfe[0];

  /* alternate implementation of lerp */
  float denom = 1.0/(pilot_carrier_hi - pilot_carrier_lo);					// 1.0/(x_b - x_a)

  for (unsigned int i = 0; i < d_data_carriers.size(); i++) {
      int di = d_data_carriers[i];
      if(di > pilot_carrier_hi) {					// 'di' is beyond the current pilot_hi
	  pilot_index++;						// move to the next pilot

	  pilot_carrier_lo = pilot_carrier_hi;				// the new pilot_lo
	  pilot_dfe_lo = pilot_dfe_hi;					// the new pilot_dfe_lo

	 if (pilot_index < d_pilot_carriers.size()) {
	     pilot_carrier_hi = d_pilot_carriers[pilot_index];
	     pilot_dfe_hi = d_dfe[pilot_index];
	 }
	 else {
	     pilot_carrier_hi = d_occupied_carriers;			// cater to the di's which are beyond the last pilot_carrier
	 }
	 denom = 1.0/(pilot_carrier_hi - pilot_carrier_lo);
      }

      float alpha = float(di - pilot_carrier_lo) * denom;		// (x - x_a)/(x_b - x_a)
      gr_complex dfe = pilot_dfe_lo + alpha * (pilot_dfe_hi - pilot_dfe_lo);	// y = y_a + (y_b - y_a) * (x - x_a)/(x_b - x_a) 
      gr_complex sigeq = in[di] * carrier * dfe;
      out[i] = sigeq;
  }
}

// apurv++ comment: demaps an entire OFDM symbol in one go (with pilots) //
unsigned int digital_ofdm_frame_sink::demap(const gr_complex *in,
                                            unsigned char *out, bool is_header)
{
  int nbits = d_data_nbits;
  if(is_header) 
     nbits = d_hdr_nbits;

  gr_complex *rot_out = (gr_complex*) malloc(sizeof(gr_complex) * d_data_carriers.size());		// only the data carriers //
  memset(rot_out, 0, sizeof(gr_complex) * d_data_carriers.size());
  equalize_interpolate_dfe(in, rot_out);


  unsigned int i=0, bytes_produced=0;

  while(i < d_data_carriers.size()) {
    if(d_nresid > 0) {
      d_partial_byte |= d_resid;
      d_byte_offset += d_nresid;
      d_nresid = 0;
      d_resid = 0;
    }

    while((d_byte_offset < 8) && (i < d_data_carriers.size())) {

      unsigned char bits;
      if(is_header) 
	 bits = slicer(rot_out[i], d_hdr_sym_position, d_hdr_sym_value_out);
      else
	 bits = slicer(rot_out[i], d_data_sym_position, d_data_sym_value_out);

      i++;
      if((8 - d_byte_offset) >= nbits) {
        d_partial_byte |= bits << (d_byte_offset);
        d_byte_offset += nbits;
      }
      else {
        d_nresid = nbits-(8-d_byte_offset);
        int mask = ((1<<(8-d_byte_offset))-1);
        d_partial_byte |= (bits & mask) << d_byte_offset;
        d_resid = bits >> (8-d_byte_offset);
        d_byte_offset += (nbits - d_nresid);
      }
      //printf("demod symbol: %.4f + j%.4f   bits: %x   partial_byte: %x   byte_offset: %d   resid: %x   nresid: %d\n", 
       //    in[i-1].real(), in[i-1].imag(), bits, d_partial_byte, d_byte_offset, d_resid, d_nresid);
    }

    if(d_byte_offset == 8) {
      //printf("demod byte: %x \n\n", d_partial_byte);
      out[bytes_produced++] = d_partial_byte;
      d_byte_offset = 0;
      d_partial_byte = 0;
    }
  }

  // cleanup //
  free(rot_out);
  if(is_header) 
     d_hdr_nbits = nbits;
  else
     d_data_nbits = nbits;

  return bytes_produced;
}

digital_ofdm_frame_sink_sptr
digital_make_ofdm_frame_sink(const std::vector<gr_complex> &hdr_sym_position,
                        const std::vector<unsigned char> &hdr_sym_value_out,
			const std::vector<gr_complex> &data_sym_position,
                        const std::vector<unsigned char> &data_sym_value_out,
			const std::vector<std::vector<gr_complex> > &preamble,
                        gr_msg_queue_sptr target_queue, gr_msg_queue_sptr fwd_queue, 
			unsigned int occupied_carriers, unsigned int fft_length,
			unsigned int proto,
                        float phase_gain, float freq_gain, unsigned int id, 
			unsigned int batch_size, int exp_size, int fec_n, int fec_k)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_frame_sink(hdr_sym_position, hdr_sym_value_out,
							data_sym_position, data_sym_value_out,
							preamble,
                                                        target_queue, fwd_queue,
							occupied_carriers, fft_length, proto,
                                                        phase_gain, freq_gain, id,
							batch_size, 
							exp_size, fec_n, fec_k));
}


digital_ofdm_frame_sink::digital_ofdm_frame_sink(const std::vector<gr_complex> &hdr_sym_position,
                                       const std::vector<unsigned char> &hdr_sym_value_out,
		                       const std::vector<gr_complex> &data_sym_position,
                		       const std::vector<unsigned char> &data_sym_value_out,
				       const std::vector<std::vector<gr_complex> > &preamble,
                                       gr_msg_queue_sptr target_queue, gr_msg_queue_sptr fwd_queue, 
				       unsigned int occupied_carriers, unsigned int fft_length,
				       unsigned int proto,
                                       float phase_gain, float freq_gain, unsigned int id,
				       unsigned int batch_size, 
				       int exp_size, int fec_n, int fec_k)
  : gr_sync_block ("ofdm_frame_sink",
                   //gr_make_io_signature2 (2, 2, sizeof(gr_complex)*occupied_carriers, sizeof(char)),  // apurv--
                   gr_make_io_signature4 (2, 4, sizeof(gr_complex)*occupied_carriers, sizeof(char), sizeof(gr_complex)*occupied_carriers, sizeof(gr_complex)*fft_length), //apurv++
                   gr_make_io_signature (0, 1, sizeof(gr_complex)*occupied_carriers)),
    d_target_queue(target_queue), d_occupied_carriers(occupied_carriers),
    d_byte_offset(0), d_partial_byte(0),
    d_resid(0), d_nresid(0),d_phase_gain(phase_gain),d_freq_gain(freq_gain),
    d_eq_gain(0.05), 
    d_batch_size(batch_size),
    d_pkt_num(0),
    d_id(id+'0'),
    d_fft_length(fft_length),
    d_out_queue(fwd_queue),
    d_expected_size(exp_size),
    d_fec_n(fec_n),
    d_fec_k(fec_k),
    d_preamble(preamble),
    d_preamble_cnt(0),
    d_proto(proto)
{

   assign_subcarriers();
   d_dfe.resize(d_pilot_carriers.size());
   fill(d_dfe.begin(), d_dfe.end(), gr_complex(1.0,0.0));


  // make sure we stay in the limit currently imposed by the occupied_carriers
  if(d_data_carriers.size() > d_occupied_carriers) {
    throw std::invalid_argument("digital_ofdm_mapper_bcv: subcarriers allocated exceeds size of occupied carriers");
  }

  d_bytes_out = new unsigned char[d_occupied_carriers];
  set_hdr_sym_value_out(hdr_sym_position, hdr_sym_value_out);
  set_data_sym_value_out(data_sym_position, data_sym_value_out);

  enter_search();

  //apurv check //
  d_num_hdr_ofdm_symbols = (HEADERBYTELEN * 8) / d_data_carriers.size();
  if((HEADERBYTELEN * 8) % d_data_carriers.size() != 0) {	       // assuming bpsk
     printf("d_data_carriers: %d, HEADERBYTELEN: %d\n", d_data_carriers.size(), HEADERBYTELEN); fflush(stdout);
     assert(false);
  }


  printf("HEADERBYTELEN: %d hdr_ofdm_symbols: %d, d_data_carriers.size(): %d\n", HEADERBYTELEN, d_num_hdr_ofdm_symbols, d_data_carriers.size());
  fflush(stdout);

  // apurv- for offline analysis/logging //
  d_fp_hestimates = NULL;
  open_hestimates_log();

  d_in_estimates = (gr_complex*) malloc(sizeof(gr_complex) * occupied_carriers);
  memset(d_in_estimates, 0, sizeof(gr_complex) * occupied_carriers);  

  memset(d_phase, 0, sizeof(float) * MAX_BATCH_SIZE);
  memset(d_freq, 0, sizeof(float) * MAX_BATCH_SIZE);

  int n_entries = pow(double (d_data_sym_position.size()), double(d_batch_size)); //pow(2.0, double(d_batch_size));
  std::string arg("");
  d_usrp = uhd::usrp::multi_usrp::make(arg);
  d_flow = -1;

  d_usrp_rx = get_usrp_source_instance();
  assert(d_usrp_rx);
  printf("sample rate: %f\n", d_usrp_rx->get_samp_rate()); 
  reset_demapper();

  fill_all_carriers_map();   

  d_out_pkt_time = uhd::time_spec_t(0.0);
  d_last_pkt_time = uhd::time_spec_t(0.0);

  populateEthernetAddress();
  populateFlowInfo();

  if(d_proto == SPP) {
    /* spp has per-hop ACKs and the route is determined through route.txt */
    populateRouteInfo();
    create_per_hop_ack_socks();
  }
  else if(d_proto == PRO) {
    /* pro has only dst-source ACKs and route is through credit.txt */
    populateCreditInfo();
    create_e2e_ack_socks();
  }
}

void
digital_ofdm_frame_sink::assign_subcarriers() {
  int dc_tones = 8;
  int num_pilots = 8;
  int pilot_gap = 11;

  int half1_end = (d_occupied_carriers-dc_tones)/2;	//40
  int half2_start = half1_end+dc_tones;			//48

  // first half
  for(int i = 0; i < half1_end; i++) {
     if(i%pilot_gap == 0) 
	d_pilot_carriers.push_back(i);
     else
	d_data_carriers.push_back(i);
  }

  // second half
  for(int i = half2_start, j = 0; i < d_occupied_carriers; i++, j++) {
     if(j%pilot_gap == 0) 
	d_pilot_carriers.push_back(i);
     else
	d_data_carriers.push_back(i);
  }

  /* debug carriers */
  printf("pilot carriers: \n");
  for(int i = 0; i < d_pilot_carriers.size(); i++) {
     printf("%d ", d_pilot_carriers[i]); fflush(stdout);
  }
  printf("\n");

  printf("data carriers: \n");
  for(int i = 0; i < d_data_carriers.size(); i++) {
     printf("%d ", d_data_carriers[i]); fflush(stdout);
  }
  printf("\n");
}

void
digital_ofdm_frame_sink::fill_all_carriers_map() {
  d_all_carriers.resize(d_occupied_carriers);

  unsigned int p = 0, d = 0, dc = 0;

  for(unsigned int i = 0; i < d_occupied_carriers; i++) {
      int carrier_index = i;

      if(d_data_carriers[d] == carrier_index) {
	 d_all_carriers[i] = 0;
	 d++;
      } 
      else if(d_pilot_carriers[p] == carrier_index) {
	 d_all_carriers[i] = 1;
	 p++;
      }
      else {
	 d_all_carriers[i] = 2;
	 dc++;
      }      
  }

  printf("d: %d, p: %d, dc: %d\n", d, p, dc); fflush(stdout);

  assert(d == d_data_carriers.size());
  assert(p == d_pilot_carriers.size());
  assert(dc == d_occupied_carriers - d_data_carriers.size() - d_pilot_carriers.size());
}

digital_ofdm_frame_sink::~digital_ofdm_frame_sink ()
{
  delete [] d_bytes_out;
}

bool
digital_ofdm_frame_sink::set_hdr_sym_value_out(const std::vector<gr_complex> &sym_position,
                                      const std::vector<unsigned char> &sym_value_out)
{
  if (sym_position.size() != sym_value_out.size())
    return false;

  if (sym_position.size()<1)
    return false;

  d_hdr_sym_position  = sym_position;
  d_hdr_sym_value_out = sym_value_out;
  d_hdr_nbits = (unsigned int)ceil(log10(float(d_hdr_sym_value_out.size())) / log10((float)2.0));		// I've got no idea!!!!!
  printf("size: %d, d_hdr_nbits: %d\n", d_hdr_sym_value_out.size(), d_hdr_nbits); fflush(stdout);

  return true;
}

bool
digital_ofdm_frame_sink::set_data_sym_value_out(const std::vector<gr_complex> &sym_position,
                                      const std::vector<unsigned char> &sym_value_out)
{
  if (sym_position.size() != sym_value_out.size())
    return false;

  if (sym_position.size()<1)
    return false;

  d_data_sym_position  = sym_position;
  d_data_sym_value_out = sym_value_out;
  d_data_nbits = (unsigned int)ceil(log10(float(d_data_sym_value_out.size())) / log10((float)2.0));               // I've got no idea!!!!!
  printf("size: %d, d_data_nbits: %d\n", d_data_sym_value_out.size(), d_data_nbits); fflush(stdout);

  return true;
}

int
digital_ofdm_frame_sink::work (int noutput_items,
                          gr_vector_const_void_star &input_items,
                          gr_vector_void_star &output_items)
{
  gr_complex *in = (gr_complex *) input_items[0];
  const char *sig = (const char *) input_items[1];

  last_noutput_items=noutput_items;

  /* channel estimates if applicable */
  gr_complex *in_estimates = NULL;
  bool use_estimates = false;
  if(input_items.size() >= 3) {
       in_estimates = (gr_complex *) input_items[2];
       use_estimates = true;
  }

  if (VERBOSE)
    fprintf(stderr,">>> Entering state machine, d_state: %d\n", d_state);

  FlowInfo *flowInfo = NULL;
  unsigned int bytes=0;
  unsigned int n_data_carriers = d_data_carriers.size();
  int count = 0;
  switch(d_state) {

  case STATE_SYNC_SEARCH:    // Look for flag indicating beginning of pkt
    if (VERBOSE)
      fprintf(stderr,"SYNC Search, noutput=%d\n", noutput_items);

    if (sig[0]) {  // Found it, set up for header decode
      enter_have_sync();
      test_timestamp(1);
      memcpy(d_in_estimates, in_estimates, sizeof(gr_complex) * d_occupied_carriers);  // use in HAVE_HEADER state //
      d_preamble_cnt++;
#if LOG_H
      count = ftell(d_fp_hestimates);
      count = fwrite_unlocked(in_estimates, sizeof(gr_complex), d_occupied_carriers, d_fp_hestimates);
#endif
    }
    break;      // don't demodulate the preamble, so break!

  case STATE_HAVE_SYNC:
    if(d_preamble_cnt < d_preamble.size()) {
	if(d_preamble_cnt == d_preamble.size()-1) {
	   /* last preamble - calculate SNR */
	   equalizeSymbols(&in[0], d_in_estimates);
	   calculate_snr(in);
	}
	else {
	   /* log hestimates (**disable if calculate_equalizer is not being called for this preamble**) */
#if LOG_H
	   memcpy(d_in_estimates, in_estimates, sizeof(gr_complex) * d_occupied_carriers);
           count = ftell(d_fp_hestimates);
           count = fwrite_unlocked(in_estimates, sizeof(gr_complex), d_occupied_carriers, d_fp_hestimates);
#endif
	}
	d_preamble_cnt++;
	break;
    }

    if (sig[0]) {
       printf("ERROR -- Found SYNC in HAVE_SYNC\n");
       reset_demapper();
       enter_search();
       break;
    }
    equalizeSymbols(&in[0], d_in_estimates);

    // only demod after getting the preamble signal; otherwise, the 
    // equalizer taps will screw with the PLL performance
    bytes = demap(&in[0], d_bytes_out, true);
    d_hdr_ofdm_index++;

    /* add the demodulated bytes to header_bytes */
    memcpy(d_header_bytes+d_hdr_byte_offset, d_bytes_out, bytes);
    d_hdr_byte_offset += bytes;

    //printf("hdrbytelen: %d, hdrdatalen: %d, d_hdr_byte_offset: %d, bytes: %d\n", HEADERBYTELEN, HEADERDATALEN, d_hdr_byte_offset, bytes); fflush(stdout);

    /* header processing */

    if (d_hdr_byte_offset == HEADERBYTELEN) {
        if(header_ok()) {						 // full header received
          printf("HEADER OK prev_hop: %d pkt_num: %d batch_num: %d dst_id: %d\n", d_header.prev_hop_id, d_header.pkt_num, d_header.batch_number, d_header.dst_id); fflush(stdout);
          extract_header();						// fills local fields with header info

	  flowInfo = getFlowInfo(false, d_flow);
	  if(!shouldProcess(flowInfo)) {
	      enter_search();
	      break;
          }

	  enter_have_header();
	  prepareForNewBatch(flowInfo);
        }
        else {
          printf("HEADER NOT OK --\n"); fflush(stdout);
          enter_search();           					// bad header
	  reset_demapper();
        }
    }
    break;

  case STATE_HAVE_HEADER:						// process the frame after header
    if (sig[0]) {
        printf("ERROR -- Found SYNC in HAVE_HEADER, length of %d\n", d_packetlen);
	enter_search();
	reset_demapper();
	resetPktInfo(d_pktInfo);
	break;
    }

    if(d_curr_ofdm_symbol_index >= d_num_ofdm_symbols) {
	printf("d_curr_ofdm_symbol_index: %d, d_num_ofdm_symbols: %d\n", d_curr_ofdm_symbol_index, d_num_ofdm_symbols); fflush(stdout);
	assert(false);
    }

    equalizeSymbols(&in[0], &d_in_estimates[0]);
    storePayload(&in[0]);
    d_curr_ofdm_symbol_index++;

    if(d_curr_ofdm_symbol_index == d_num_ofdm_symbols) {		// last ofdm symbol in pkt

       std::string decoded_msg;
       flowInfo = getFlowInfo(false, d_flow);
       bool tx_ack = demodulate_packet(decoded_msg, flowInfo);
       if(d_proto == SPP) 
	  send_ack(tx_ack, flowInfo);					// per-hop ack
       else if(d_dst && d_proto == PRO) {
	  bool e2e_ack_pro = decodePacket(decoded_msg, flowInfo);
	  if(e2e_ack_pro) {
	     send_ack(true, flowInfo);
	  }
       }

       if(tx_ack && d_fwd) {
	  //updateCredit?
	  makePacket(decoded_msg, flowInfo);
       }

       resetPktInfo(d_pktInfo);
       enter_search();     						// current pkt done, next packet
       reset_demapper();
       break;
    }

    break;
 
  default:
    assert(0);
  } // switch

  return 1;
}

/* equalization that was supposed to be done by the acquisition block! */
inline void
digital_ofdm_frame_sink::equalizeSymbols(gr_complex *in, gr_complex *in_estimates)
{
   for(unsigned int i = 0; i < d_occupied_carriers; i++) 
        in[i] = in[i] * in_estimates[i];
}

/*
   - extract header crc
   - calculate header crc
   - verify if both are equal
*/
bool
digital_ofdm_frame_sink::header_ok()
{
  //printf("header_ok start\n"); fflush(stdout);
  dewhiten(d_header_bytes, HEADERBYTELEN);
  memcpy(&d_header, d_header_bytes, d_hdr_byte_offset);
  unsigned int rx_crc = d_header.hdr_crc;
 
  unsigned char *header_data_bytes = (unsigned char*) malloc(HEADERDATALEN);
  memcpy(header_data_bytes, d_header_bytes, HEADERDATALEN);
  unsigned int calc_crc = digital_crc32(header_data_bytes, HEADERDATALEN);
  free(header_data_bytes);
 
  if(rx_crc != calc_crc) {
	printf("\nrx_crc: %u, calc_crc: %u, \t\t\t\t\tpkt_num: %d \tbatch_num: %d\n", rx_crc, calc_crc, d_header.pkt_num, d_header.batch_number); fflush(stdout);
  }

  char prev_hop = ((int) d_header.prev_hop_id) + '0';
  char next_hop = ((int) d_header.next_hop_id) + '0';
  printf("prevHop: %c, nextHop: %c\n", prev_hop, next_hop); fflush(stdout);
 
  /* DEBUG
  printf("\nrx_crc: %u, calc_crc: %u, \t\t\t\t\tpkt_num: %d \tbatch_num: %d\n", rx_crc, calc_crc, d_header.pkt_num, d_header.batch_number); fflush(stdout);
  printf("hdr details: (src: %d), (rx: %d), (batch_num: %d), (d_nsenders: %d), (d_packetlen: %d), (d_pkt_type: %d)\n", d_header.src_id, d_header.dst_id, d_header.batch_number, d_header.nsenders, d_header.packetlen, d_header.pkt_type);

  printf("header_ok end\n"); fflush(stdout);
  */

  return (rx_crc == calc_crc);
}

void
digital_ofdm_frame_sink::dewhiten(unsigned char *bytes, const int len)
{
  for(int i = 0; i < len; i++)
      bytes[i] = bytes[i] ^ random_mask_tuple[i];
}

/* stores 1 OFDM symbol */
void
digital_ofdm_frame_sink::storePayload(gr_complex *in)
{
  unsigned int offset = d_curr_ofdm_symbol_index * d_occupied_carriers;
  memcpy(d_pktInfo->symbols + offset, in, sizeof(gr_complex) * d_occupied_carriers);
}

bool
digital_ofdm_frame_sink::demodulate_packet(std::string& decoded_msg, FlowInfo *fInfo)
{
  printf("demodulate\n"); fflush(stdout);

  // run the demapper now: demap each packet within the batch //
  unsigned char *bytes_out = (unsigned char*) malloc(sizeof(unsigned char) * d_occupied_carriers);

  gr_complex *out_symbols = d_pktInfo->symbols;
  int packetlen_cnt = 0;
  unsigned char packet[MAX_PKT_LEN];
  bool crc_valid = false;

  //printf("d_partial_byte: %d, d_byte_offset: %d\n", d_partial_byte, d_byte_offset); 
  for(unsigned int o = 0; o < d_num_ofdm_symbols; o++) {
      unsigned int offset = o * d_occupied_carriers;

      memset(bytes_out, 0, sizeof(unsigned char) * d_occupied_carriers);
      unsigned int bytes_decoded = demap(&out_symbols[offset], bytes_out, false);

      unsigned int jj = 0;
      while(jj < bytes_decoded) {
           packet[packetlen_cnt++] = bytes_out[jj++];

           if (packetlen_cnt == d_packetlen){              // packet is filled
		
               gr_message_sptr msg = gr_make_message(0, d_packet_whitener_offset, 0, packetlen_cnt);
	       memcpy(msg->msg(), packet, packetlen_cnt);

	       crc_valid = crc_check(msg->to_string(), decoded_msg);
	       printf("crc valid: %d\n", crc_valid); fflush(stdout);		

               msg.reset();                            // free it up
           }
      }
  }

  free(bytes_out);

  fInfo->total_pkts_rcvd++;

  if(d_proto == SPP) {
     if(crc_valid) {
        fInfo->num_pkts_correct++;
        printf("crc valid: %d\n", crc_valid); fflush(stdout); 
        fInfo->last_batch_acked = fInfo->active_batch;
     }
     printf("--------------------------------------------------------------------\n");
     printf("Flow: %c, Batch OK: %d, Batch Id: %d, Total Pkts: %d, Correct: %d\n", fInfo->flowId, crc_valid, fInfo->active_batch, fInfo->total_pkts_rcvd, fInfo->num_pkts_correct); 
     printf("---------------------------------------------------------------------\n"); fflush(stdout);
  } 
  else if(d_proto == PRO) {
     printf("Flow: %c, Pkt OK: %d, Batch Id: %d\n", fInfo->flowId, crc_valid, fInfo->active_batch); fflush(stdout);
  }
  print_msg(decoded_msg);  
  return crc_valid;
}

bool
digital_ofdm_frame_sink::open_hestimates_log()
{
  // hestimates - only for those for which the header is OK //
  const char *filename = "ofdm_hestimates.dat";
  int fd;
  if ((fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
     perror(filename);
     return false;
  }
  else {
      if((d_fp_hestimates = fdopen (fd, true ? "wb" : "w")) == NULL) {
            fprintf(stderr, "hestimates file cannot be opened\n");
            close(fd);
            return false;
      }
  }
  return true;
}

/* node can be either of 3: 
   - final destination of the flow
   - a forwarder
   - a triggered forwarder
*/
bool
digital_ofdm_frame_sink::shouldProcess(FlowInfo *fInfo) {

  char hdr_next_hop = ((int) d_header.next_hop_id) + '0';
  char hdr_flow = ((int) d_header.flow_id) + '0';
  char hdr_dst = ((int) d_header.dst_id) + '0'; 

  printf("shouldProcess - hdr_next_hop: %c, hdr_dst: %c, d_id: %c, hdr_flow: %c, flow: %c\n",
			hdr_next_hop, hdr_dst, d_id, hdr_flow, fInfo->flowId); fflush(stdout);
  if(hdr_next_hop == d_id && hdr_flow == fInfo->flowId) {
     if(hdr_dst == d_id) 
        d_dst = true;
     else
	d_fwd = true;
  }
  else return false;

  if(fInfo->last_batch_acked == -1)
     return true;

  if(fInfo->last_batch_acked >= d_batch_number) {
    printf("--shouldProcess: false, batch: %d, last_acked_batch: %d\n", d_batch_number, fInfo->last_batch_acked);
    fflush(stdout);
    return false;
  }

  printf("--shouldProcess: true, batch: %d, last_acked_batch: %d\n", d_batch_number, fInfo->last_batch_acked);
  return true;
}

void
digital_ofdm_frame_sink::prepareForNewBatch(FlowInfo *fInfo) {
  printf("prepareForNewBatch, batch: %d\n", d_header.batch_number); fflush(stdout);
  fInfo->active_batch = d_batch_number;
  fInfo->pkts_fwded = 0;
}


inline PktInfo*
digital_ofdm_frame_sink::createPktInfo() {
  FlowInfo *flowInfo = getFlowInfo(false, d_flow);
  assert(flowInfo);

  PktInfo *pktInfo = (PktInfo*) malloc(sizeof(PktInfo));
  memset(pktInfo, 0, sizeof(PktInfo));

  pktInfo->n_senders = d_nsenders;
  pktInfo->symbols = NULL;
  return pktInfo;
}

inline void
digital_ofdm_frame_sink::resetPktInfo(PktInfo* pktInfo) {
  if(pktInfo->symbols != NULL)
     free(pktInfo->symbols);
}

/* add the pkts to the innovative-store of the corresponding flow-id - all the packets belong to the same batch */
inline FlowInfo*
digital_ofdm_frame_sink::getFlowInfo(bool create, unsigned char flowId)
{ 
  vector<FlowInfo*>::iterator it = d_flowInfoVector.begin();
  FlowInfo *flow_info = NULL;
  while(it != d_flowInfoVector.end()) {
     flow_info = *it;
     if(flow_info->flowId == flowId)
	return flow_info;
     it++;
  }
  
  if(flow_info == NULL && create)
  {
     flow_info = (FlowInfo*) malloc(sizeof(FlowInfo));
     memset(flow_info, 0, sizeof(FlowInfo));
     flow_info->pkts_fwded = 0;
     flow_info->last_batch_acked = -1;
     flow_info->flowId = flowId;
     d_flowInfoVector.push_back(flow_info);
  }

  return flow_info; 
}

/* handle the 'lead sender' case */
void
digital_ofdm_frame_sink::makeHeader(MULTIHOP_HDR_TYPE &header, unsigned char *header_bytes, FlowInfo *flowInfo)
{
   memset(header_bytes, 0, HEADERBYTELEN);

   memset(&header, 0, sizeof(header));

   header.flow_id = flowInfo->flowId;
   header.src_id = flowInfo->src; 
   header.dst_id = flowInfo->dst;       
   header.prev_hop_id = d_id;	
   header.next_hop_id = flowInfo->nextHopId;

   assert(header.next_hop_id != UNASSIGNED);

   printf("-- makeHeader b: %d, pkt_num: %d, len: %d\n", flowInfo->active_batch, d_pkt_num, d_packetlen); fflush(stdout);

   header.packetlen = d_packetlen;
   header.batch_number = flowInfo->active_batch;
   header.pkt_type = DATA_TYPE;
   header.pkt_num = d_pkt_num;
   header.link_id = 0;				// dumb field - might use later

   flowInfo->pkts_fwded++;

   unsigned char header_data_bytes[HEADERDATALEN];
   memcpy(header_data_bytes, &header, HEADERDATALEN);                         // copy e'thing but the crc

   unsigned int calc_crc = digital_crc32(header_data_bytes, HEADERDATALEN);
   header.hdr_crc = calc_crc;

   memcpy(header_bytes, header_data_bytes, HEADERDATALEN);                            // copy header data
   memcpy(header_bytes+HEADERDATALEN, &calc_crc, sizeof(int));                	      // copy header crc

   printf("len: %d, crc: %u\n", d_packetlen, calc_crc);

   whiten(header_bytes, HEADERBYTELEN);
   debugHeader(header_bytes);
}

void
digital_ofdm_frame_sink::debugHeader(unsigned char *header_bytes)
{
   printf("debugHeader\n"); fflush(stdout);
   whiten(header_bytes, HEADERBYTELEN);    // dewhitening effect !
   MULTIHOP_HDR_TYPE header;
   memset(&header, 0, HEADERBYTELEN);
   assert(HEADERBYTELEN == (HEADERDATALEN + sizeof(int) + PADDING_SIZE));
   memcpy(&header, header_bytes, HEADERBYTELEN);

   printf("batch#: %d, debug crc: %u\n", header.batch_number, header.hdr_crc);
   whiten(header_bytes, HEADERBYTELEN);
}

#if 0
inline CreditInfo*
digital_ofdm_frame_sink::findCreditInfo(unsigned char flowId) {
  CreditInfo *creditInfo = NULL;
  if(d_creditInfoVector.size() == 0)
	return creditInfo;

  CreditInfoVector::iterator it = d_creditInfoVector.begin();
  
  while(it != d_creditInfoVector.end()) {
        creditInfo = *it;
	if(creditInfo->flowId == flowId)
	    return creditInfo;	
        it++;
  } 

  return NULL;
}
#endif

/* sends the ACK on the backend ethernet */
void 
digital_ofdm_frame_sink::send_ack(bool ack, FlowInfo *flowInfo) {
  MULTIHOP_ACK_HDR_TYPE ack_header;
  memset(&ack_header, 0, sizeof(ack_header));
 
  ack_header.src_id = d_id;

  if(d_proto == SPP)
     ack_header.dst_id = flowInfo->prevHopId;				// per-hop ack for SPP
  else if(d_proto == PRO)
     ack_header.dst_id = flowInfo->src;					// e2e ack for PRO

  ack_header.batch_number = flowInfo->active_batch;                     // active-batch got ACKed
  if(ack) ack_header.pkt_type = ACK_TYPE;
  else ack_header.pkt_type = NACK_TYPE;

  ack_header.flow_id = flowInfo->flowId;

  for(int i = 0; i < PADDING_SIZE; i++)
       ack_header.pad[i] = 0;

  char ack_buf[ACK_HEADERDATALEN];
  memcpy(ack_buf, &ack_header, ACK_HEADERDATALEN);

  // transmit an ACK on all sockets (FIXME) - right now I cannot get the socket for a specific client //
  for(int i = 0; i < d_ack_tx_socks.size(); i++) {
     if (send(d_ack_tx_socks[i], (char *)&ack_buf[0], ACK_HEADERDATALEN, 0) < 0) {
         assert(false);
     } 
  }
  printf("@@@@@@@@@@@@@@@@ sent ACK (%d bytes), to node: %c for batch %d @@@@@@@@@@@@@@@@@@@@\n", ACK_HEADERDATALEN, ack_header.dst_id, flowInfo->active_batch); fflush(stdout);
}

#if 0
void
digital_ofdm_frame_sink::makePacket(std::string msg, FlowInfo *flowInfo) {
  printf("makePacket start\n"); fflush(stdout);
  gr_message_sptr out_msg = gr_make_message(DATA_TYPE, 0, 0, HEADERBYTELEN+d_packetlen);
  unsigned char header_bytes[HEADERBYTELEN];
  makeHeader(header_bytes, flowInfo);		// header is whitened in this function 

  unsigned char *data_bytes = (unsigned char*) malloc(d_packetlen);
  memcpy(data_bytes, (unsigned char*) msg.data(), d_packetlen);

  whiten(data_bytes, d_packetlen);		// whitening effect!
  printf("whitened.. \n"); fflush(stdout);
  for(int i = 0; i < d_packetlen; i++) {
     printf("%02x", (unsigned char) data_bytes[i]);
  }
  printf("\n"); fflush(stdout);

  memcpy(out_msg->msg(), header_bytes, HEADERBYTELEN);
  memcpy(out_msg->msg()+HEADERBYTELEN, data_bytes, d_packetlen);  
  d_out_queue->insert_tail(out_msg);

  out_msg.reset();
  free(data_bytes);
  printf("makePacket end\n"); fflush(stdout);
}
#endif

void
digital_ofdm_frame_sink::makePacket(std::string msg, FlowInfo *flowInfo) {
  printf("makePacket start\n"); fflush(stdout);
  gr_message_sptr out_msg = gr_make_message(DATA_TYPE, 0, 0, HEADERBYTELEN+d_packetlen);
  unsigned char header_bytes[HEADERBYTELEN];
  MULTIHOP_HDR_TYPE header;
  makeHeader(header, header_bytes, flowInfo);           // header is whitened in this function 
  out_msg->set_header(header);


  unsigned char *data_bytes = (unsigned char*) malloc(d_packetlen);
  memcpy(data_bytes, (unsigned char*) msg.data(), d_packetlen);

  whiten(data_bytes, d_packetlen);              // whitening effect!
  printf("whitened.. \n"); fflush(stdout);
  for(int i = 0; i < d_packetlen; i++) {
     printf("%02x", (unsigned char) data_bytes[i]);
  }
  printf("\n"); fflush(stdout);

  memcpy(out_msg->msg(), data_bytes, d_packetlen);  
  d_out_queue->insert_tail(out_msg);

  out_msg.reset();
  free(data_bytes);
  printf("makePacket end\n"); fflush(stdout);
}

void
digital_ofdm_frame_sink::whiten(unsigned char *bytes, const int len)
{
   for(int i = 0; i < len; i++)
        bytes[i] = bytes[i] ^ random_mask_tuple[i];
}

inline void
digital_ofdm_frame_sink::set_msg_timestamp(gr_message_sptr msg) {
  unsigned int tag_port = 1;
  std::vector<gr_tag_t> rx_sync_tags;
  const uint64_t nread = this->nitems_read(tag_port);
  this->get_tags_in_range(rx_sync_tags, tag_port, nread, nread+last_noutput_items, SYNC_TIME);
  if(rx_sync_tags.size()>0) {
     size_t t = rx_sync_tags.size()-1;
     printf("set_timestamp (SINK):: found %d tags, offset: %ld, nread: %ld, output_items: %ld\n", rx_sync_tags.size(), rx_sync_tags[t].offset, nread, last_noutput_items); fflush(stdout);
     const pmt::pmt_t &value = rx_sync_tags[t].value;
     uint64_t sync_secs = pmt::pmt_to_uint64(pmt_tuple_ref(value, 0));
     double sync_frac_of_secs = pmt::pmt_to_double(pmt_tuple_ref(value,1));
     msg->set_timestamp(sync_secs, sync_frac_of_secs);
  } else {
     std::cerr << "SINK ---- Header received, with no sync timestamp?\n";
  }

  std::vector<gr_tag_t> rx_sync_tags2;
  this->get_tags_in_range(rx_sync_tags2, tag_port, 0, nread, SYNC_TIME);
  if(rx_sync_tags2.size()>0) {
     size_t t = rx_sync_tags2.size()-1;
     printf("test_timestamp2 (SINK):: found %d tags, offset: %ld, nread: %ld\n", rx_sync_tags2.size(), rx_sync_tags2[t].offset, nread); fflush(stdout);
     const pmt::pmt_t &value = rx_sync_tags2[t].value;
     uint64_t sync_secs = pmt::pmt_to_uint64(pmt_tuple_ref(value, 0));
     double sync_frac_of_secs = pmt::pmt_to_double(pmt_tuple_ref(value,1));
  } else {
     std::cerr << "SINK ---- Header received, with no sync timestamp2?\n";
  }
}

/* crc related stuff - so that ofdm_packet_utils.py can be removed completely! */
bool
digital_ofdm_frame_sink::crc_check(std::string msg, std::string& decoded_str)
{
  int fec_n = d_fec_n, fec_k = d_fec_k;
  int exp_len = d_expected_size;
  
  int len = msg.length();
  unsigned char *msg_data = (unsigned char*) malloc(len+1);
  memset(msg_data, 0, len+1);
  memcpy(msg_data, msg.data(), len); 

  /* dewhiten the msg first */
  printf("crc_check begin! len: %d\n", len); fflush(stdout);
  dewhiten(msg_data, len);
  std::string dewhitened_msg((const char*) msg_data, len);

  /*
  for(int i = 0; i < len; i++) {
     printf("%02x ", (unsigned char) dewhitened_msg[i]); fflush(stdout);
  } 
  printf("\n"); fflush(stdout);
  */

  /* perform any FEC if required */
  if(fec_n > 0 && fec_k > 0) {
	decoded_str = digital_rx_wrapper(dewhitened_msg, fec_n, fec_k, 1, exp_len);
  } else {
	decoded_str = dewhitened_msg;
  }

  if(len < 4) {
	free(msg_data);
	return false;
  }

  /* now, calculate the crc based on the received msg */
  std::string msg_wo_crc = decoded_str.substr(0, exp_len-4); 
  unsigned int calculated_crc = digital_crc32(msg_wo_crc);
  char hex_crc[15];
  snprintf(hex_crc, sizeof(hex_crc), "%08x", calculated_crc);
  printf("hex calculated crc: %s  int calculated crc: %u\n", hex_crc, calculated_crc); 

 
  /* get the msg crc from the end of the msg */
  std::string expected_crc = decoded_str.substr(exp_len-4, 4);
  std::string hex_exp_crc = "";

  for(int i = expected_crc.size()-1; i >= 0; i--) {			// f-ing endianess!!
        char tmp[3];
        snprintf(tmp, sizeof(tmp), "%02x", (unsigned char) expected_crc[i]);
	//printf("%s", test); fflush(stdout);
	hex_exp_crc.append(tmp);
  }

  //printf("hex exp crc (%d): %s\n", sizeof(hex_exp_crc), hex_exp_crc.c_str()); fflush(stdout);
  free(msg_data);
  bool res =  (hex_exp_crc.compare(hex_crc) == 0);

  printf("crc_check end: %d\n", res); fflush(stdout);
  return res;
  //return (hex_exp_crc.compare(hex_crc) == 0);
}

inline void
digital_ofdm_frame_sink::print_msg(std::string msg) {
  int len = msg.length();
  for(int i = 0; i < len; i++) {
     printf("%02x", (unsigned char) msg[i]); 
  }  
  printf("\n"); fflush(stdout);
}

/* returns true if a tag was found */
inline void
digital_ofdm_frame_sink::test_timestamp(int output_items) {
  //output_items = last_noutput_items;
  unsigned int tag_port = 1;
  std::vector<gr_tag_t> rx_sync_tags;
  const uint64_t nread1 = nitems_read(tag_port);
//get_tags_in_range(rx_sync_tags, tag_port, nread1, nread1+output_items, SYNC_TIME);
  get_tags_in_range(rx_sync_tags, tag_port, nread1, nread1+17, SYNC_TIME); 

  printf("(SINK) nread1: %llu, output_items: %d\n", nread1, output_items); fflush(stdout);
  if(rx_sync_tags.size()>0) {
     size_t t = rx_sync_tags.size()-1;
     uint64_t offset = rx_sync_tags[t].offset;

     printf("test_timestamp1 (SINK):: found %d tags, offset: %llu, output_items: %d, nread1: %llu\n", rx_sync_tags.size(), rx_sync_tags[t].offset, output_items, nread1); fflush(stdout);

     const pmt::pmt_t &value = rx_sync_tags[t].value;
     uint64_t sync_secs = pmt::pmt_to_uint64(pmt_tuple_ref(value, 0));
     double sync_frac_of_secs = pmt::pmt_to_double(pmt_tuple_ref(value,1));

     d_sync_tag.value = rx_sync_tags[t].value;
     d_sync_tag.offset = offset;
  }
 
  //std::cerr << "SINK---- Header received, with no sync timestamp1?\n";
  //assert(false);
}

inline void
digital_ofdm_frame_sink::populateEthernetAddress()
{
   // node-id ethernet-addrees port-on-which-it-is-listening //
   printf("populateEthernetAddress\n"); fflush(stdout);
   FILE *fl = fopen ( "eth_add.txt" , "r+" );
   if (fl==NULL) {
        fprintf (stderr, "File error\n");
        exit (1);
   }

   char line[128];
   const char *delim = " ";
   while ( fgets ( line, sizeof(line), fl ) != NULL ) {
        char *strtok_res = strtok(line, delim);
        vector<char*> token_vec;
        while (strtok_res != NULL) {
           token_vec.push_back(strtok_res);
           //printf ("%s\n", strtok_res);
           strtok_res = strtok (NULL, delim);
        }

	EthInfo *ethInfo = (EthInfo*) malloc(sizeof(EthInfo));
	memset(ethInfo, 0, sizeof(EthInfo));
        NodeId node_id = (NodeId) atoi(token_vec[0]) + '0';
        memcpy(ethInfo->addr, token_vec[1], 16);
	unsigned int port = atoi(token_vec[2]);
	ethInfo->port = port;

        d_ethInfoMap.insert(pair<NodeId, EthInfo*> (node_id, ethInfo));

        std::cout<<"Inserting in the map: node " << node_id << " addr: " << ethInfo->addr << endl;
   }

   fclose (fl);

#if 0
   // debug
   EthInfoMap::iterator it = d_ethInfoMap.begin();
   while(it != d_ethInfoMap.end()) {
      std::cout << " key: " << it->first << " d_id: " << d_id << endl;
      it++;
   } 
#endif
}

inline void
digital_ofdm_frame_sink::calculate_snr(gr_complex *in) {
  const std::vector<gr_complex> &ks = d_preamble[d_preamble_cnt];

  float err = 0.0;
  float pow = 0.0;

  for(int i=0; i<d_occupied_carriers;i++) {
     //printf("ks: (%f, %f), in: (%f, %f)\n", ks[i].real(), ks[i].imag(), in[i].real(), in[i].imag());

     err += norm(ks[i]-in[i]);
     pow += norm(ks[i]);
  }

  printf("calculate_snr: %f\n", 10*log10(pow/err)); fflush(stdout);
}

/* util function */
inline void
digital_ofdm_frame_sink::open_server_sock(int sock_port, vector<unsigned int>& connected_clients, int num_clients) {
  printf("open_server_sock start, #clients: %d\n", num_clients); fflush(stdout);
  int sockfd, _sock;
  struct sockaddr_in dest;

  /* create socket */
  sockfd = socket(PF_INET, SOCK_STREAM, 0);
  fcntl(sockfd, F_SETFL, O_NONBLOCK);

  /* ensure the socket address can be reused, even after CTRL-C the application */
  int optval = 1;
  int ret = setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

  if(ret == -1) {
        printf("@ digital_ofdm_mapper_bcv::setsockopt ERROR\n"); fflush(stdout);
  }

  /* initialize structure dest */
  bzero(&dest, sizeof(dest));
  dest.sin_family = AF_INET;

  /* assign a port number to socket */
  dest.sin_port = htons(sock_port);
  dest.sin_addr.s_addr = INADDR_ANY;

  bind(sockfd, (struct sockaddr*)&dest, sizeof(dest));

  /* make it listen to socket with max 20 connections */
  listen(sockfd, 1);
  assert(sockfd != -1);

  struct sockaddr_in client_addr;
  int addrlen = sizeof(client_addr);

  int conn = 0;
  while(conn != num_clients) {
    _sock = accept(sockfd, (struct sockaddr*)&client_addr, (socklen_t*)&addrlen);
    if(_sock != -1) {
      printf(" -- connected client: %d\n", conn); fflush(stdout);
      connected_clients.push_back(_sock);
      conn++;
    }
  }

  printf("open_server_sock done.. #clients: %d\n", num_clients);
  assert(connected_clients.size() == num_clients);
}

/* util function */
inline int
digital_ofdm_frame_sink::open_client_sock(int port, const char *addr, bool blocking) {
  int sockfd;
  struct sockaddr_in dest;

  /* create socket */
  sockfd = socket(PF_INET, SOCK_STREAM, 0);
  if(!blocking)
     fcntl(sockfd, F_SETFL, O_NONBLOCK);

  assert(sockfd != -1);

  /* initialize value in dest */
  bzero(&dest, sizeof(struct sockaddr_in));
  dest.sin_family = PF_INET;
  dest.sin_port = htons(port);
  dest.sin_addr.s_addr = inet_addr(addr);
  /* Connecting to server */
  connect(sockfd, (struct sockaddr*)&dest, sizeof(struct sockaddr_in));
  return sockfd;
}

/* d_ack_tx_socks (clients to which I will send an ACK)
   all clients/servers must be unique 
*/
inline void
digital_ofdm_frame_sink::create_per_hop_ack_socks() {
  printf("(sink):: create_per_hop_ack_socks\n"); fflush(stdout);
  // clients to which I will send an ACK // 
  int num_clients = d_prevHopIds.size();
  EthInfoMap::iterator e_it = d_ethInfoMap.find(d_id);
  assert(e_it != d_ethInfoMap.end());
  EthInfo *ethInfo = (EthInfo*) e_it->second;
  open_server_sock(ethInfo->port, d_ack_tx_socks, num_clients);
  printf("(sink):: create_per_hop_ack_socks done\n"); fflush(stdout);
}

/* read shortest path info */
inline void
digital_ofdm_frame_sink::populateRouteInfo() {
   // flow	num_nodes	src	fwd..	dst //
   FILE *fl = fopen ( "route.txt" , "r+" );
   if (fl==NULL) {
        fprintf (stderr, "File error\n");
        exit (1);
   }

   char line[128];
   const char *delim = " ";
   while ( fgets ( line, sizeof(line), fl ) != NULL ) {
        char *strtok_res = strtok(line, delim);
        vector<char*> token_vec;
        while (strtok_res != NULL) {
           token_vec.push_back(strtok_res);
           //printf ("%s\n", strtok_res);
           strtok_res = strtok (NULL, delim);
        }
	int num_nodes = atoi(token_vec[1]);
        printf("num_nodes: %d\n", num_nodes); fflush(stdout);

        printf("size: %d\n", token_vec.size()); fflush(stdout);

	unsigned char flowId = atoi(token_vec[0])+'0';
	printf("flow: %c\n", flowId); fflush(stdout);

	FlowInfo *flowInfo = getFlowInfo(false, flowId);

	NodeId src = atoi(token_vec[2]) + '0';
	NodeId dst = atoi(token_vec[token_vec.size()-1]) + '0';
	assert(flowInfo->src == src && flowInfo->dst == dst); 			// sanity

        for(int i = 0; i < num_nodes; i++) {
           NodeId nodeId = atoi(token_vec[i+2]) + '0';
	   if(nodeId == d_id) {
	      if(i == 0) {
		 flowInfo->nextHopId = atoi(token_vec[i+3]) + '0';		// next entry
		 flowInfo->prevHopId = UNASSIGNED;				// unassigned
		 d_nextHopIds.insert(std::pair<unsigned char, bool> (flowInfo->nextHopId, true));
	      }
	      else if(i == num_nodes-1) {
		 flowInfo->nextHopId = UNASSIGNED;					// unassigned
		 flowInfo->prevHopId = atoi(token_vec[i+1]) + '0';		// previous entry
		 d_prevHopIds.insert(std::pair<unsigned char, bool> (flowInfo->prevHopId, true));
	      }
	      else {
		 flowInfo->nextHopId = atoi(token_vec[i+3]) + '0';             // next entry
		 flowInfo->prevHopId = atoi(token_vec[i+1]) + '0';             // previous entry
		 d_nextHopIds.insert(std::pair<unsigned char, bool> (flowInfo->nextHopId, true));
		 d_prevHopIds.insert(std::pair<unsigned char, bool> (flowInfo->prevHopId, true));
	      }
	      break;
	   }
        }

        printf("\n");
   }
   fclose (fl);
}

inline void
digital_ofdm_frame_sink::populateFlowInfo() {
    // flowId   srcId   dstId //
   printf("populateFlowInfo\n"); fflush(stdout);
   FILE *fl = fopen ( "flow.txt" , "r+" );
   if (fl==NULL) {
        fprintf (stderr, "File error\n");
        assert(false);
   }

   char line[128];
   const char *delim = " ";
   while ( fgets ( line, sizeof(line), fl ) != NULL ) {
        char *strtok_res = strtok(line, delim);
        vector<char*> token;
        while (strtok_res != NULL) {
           token.push_back(strtok_res);
           strtok_res = strtok (NULL, delim);
        }

        FlowInfo *fInfo = getFlowInfo(true, atoi(token[0])+'0');
        fInfo->src = atoi(token[1])+'0';
        fInfo->dst = atoi(token[2])+'0';
   }

   fclose (fl);
}

inline void
digital_ofdm_frame_sink::populateCreditInfo() {
    // flow  nodeId  prevNodeId   nextNodeId   credit  //
   printf("populateCreditInfo\n"); fflush(stdout);
   FILE *fl = fopen ( "credit.txt" , "r+" );
   if (fl==NULL) {
        fprintf (stderr, "File error\n");
        assert(false);
   }

   char line[128];
   const char *delim = " ";
   while ( fgets ( line, sizeof(line), fl ) != NULL ) {
        char *strtok_res = strtok(line, delim);
        vector<char*> token;
        while (strtok_res != NULL) {
           token.push_back(strtok_res);
           strtok_res = strtok (NULL, delim);
        }
	
	char flowId = atoi(token[0])+'0';
	char nodeId = atoi(token[1])+'0';
	if(nodeId == d_id) {
	   FlowInfo *fInfo = getFlowInfo(false, flowId); assert(fInfo != NULL);
           CreditInfo* cInfo = (CreditInfo*) malloc(sizeof(CreditInfo));
           memset(cInfo, 0, sizeof(CreditInfo));

           cInfo->flowId = flowId;
	   cInfo->prevHopId = atoi(token[2])+'0';
	   cInfo->nextHopId = atoi(token[3])+'0';
           cInfo->delta_credit = atof(token[4]);

	   d_creditInfoVector.push_back(cInfo);
        }
   }
   fclose (fl);
}

inline void
digital_ofdm_frame_sink::create_e2e_ack_socks() {
   printf("(sink):: create_e2e_ack_socks\n"); fflush(stdout);

   EthInfoMap::iterator e_it = d_ethInfoMap.find(d_id);
   assert(e_it != d_ethInfoMap.end());
   EthInfo *ethInfo = (EthInfo*) e_it->second;

   for(int i = 0; i < d_flowInfoVector.size(); i++) {
      FlowInfo *fInfo = d_flowInfoVector[i];
      if(fInfo->dst == d_id) {
	 open_server_sock(ethInfo->port, d_ack_tx_socks, 1);
	 int index = d_ack_tx_socks.size()-1;
	 d_sock_map.insert(std::pair<unsigned char, unsigned int>(fInfo->flowId, d_ack_tx_socks[index]));
      }
   }
   printf("(sink):: create_e2e_ack_socks done\n"); fflush(stdout);
}

inline NetCoder*
digital_ofdm_frame_sink::getNetCoder(FlowId flowId, int data_len) {
   FlowNetCoderMap::iterator it = d_NetCoderMap.find(flowId);
   NetCoder *netcoder = NULL;
   if(it == d_NetCoderMap.end()) {
      netcoder = new NetCoder(d_batch_size, data_len);
      d_NetCoderMap.insert(std::pair<FlowId, NetCoder*> (flowId, netcoder));
   }
   else
      netcoder = it->second;

   return netcoder;
}

/* PRO decoding - adds the packet to the NetCoder and if rank==batch_size, then 
   decodes the packets */
inline bool
digital_ofdm_frame_sink::decodePacket(std::string encoded_msg, FlowInfo *fInfo) {
   printf("decodePacket_PRO - encoded_len: %d\n", encoded_msg.length()); fflush(stdout);
   // strip the crc //
   std::string msg = encoded_msg.substr(0, encoded_msg.length()-4); 
   int len = msg.length();  

   NetCoder *decoder = getNetCoder(fInfo->flowId, len);
   assert(decoder != NULL);

   int cur_rank = decoder->num_pkts;
   unsigned char *pkt = (unsigned char*) malloc(d_batch_size+len);
   for(int i = 0; i < d_batch_size; i++) 
      pkt[i] = fInfo->coeffs[i];
   memcpy(pkt+d_batch_size, msg.data(), len); 
  

   printf("encoded content - \n"); fflush(stdout);
   for(int i = 0; i < d_batch_size+len; i++)
       printf("%02X ", pkt[i]);
   printf("\n");

   int new_rank = decoder->AddPacket(pkt);
   assert(new_rank >= cur_rank);

   printf("old_rank: %d, new_rank: %d\n", cur_rank, new_rank);
   if(new_rank == cur_rank || new_rank < d_batch_size) {
      printf("cannot decode yet!\n"); fflush(stdout);
      free(pkt);
      return false;
   }

   printf("Decoding possible! \n"); fflush(stdout);
   unsigned char **res = decoder->Decode();
   for(int i = 0; i < d_batch_size; i++) {
       printf("%3d ", i); 
       for(int j = 0; j < len; j++) 
	   printf("%02X ", int(res[i][d_batch_size+j]));
       printf("\n");
   }

   decoder->num_pkts = 0;
   fInfo->num_pkts_correct += d_batch_size;
   fInfo->last_batch_acked = fInfo->active_batch;  
   free(pkt);

   printf("-------------------------------------------------------------\n");
   printf("Flow: %c, Batch Id: %d, Total Pkts Rx: %d, Correct Pkts: %d\n", fInfo->flowId, fInfo->active_batch, fInfo->total_pkts_rcvd, fInfo->num_pkts_correct);
   printf("-------------------------------------------------------------\n");
   fflush(stdout);

   return true;
}
