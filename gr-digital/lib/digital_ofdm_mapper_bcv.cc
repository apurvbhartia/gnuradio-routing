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

#include <digital_ofdm_mapper_bcv.h>
#include <gr_io_signature.h>
#include <stdexcept>
#include <string.h>
#include <cstdio> 
#include <gr_expj.h>
#include <time.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>

digital_ofdm_mapper_bcv_sptr
digital_make_ofdm_mapper_bcv (const std::vector<gr_complex> &hdr_constellation, 
			 const std::vector<gr_complex> &data_constellation,
			 const std::vector<std::vector<gr_complex> > &preamble,
			 unsigned int msgq_limit, 
                         unsigned int occupied_carriers, unsigned int fft_length, 
			 unsigned int tdma, unsigned int id,
			 unsigned int source,
			 unsigned int batch_size,
			 unsigned int dst_id)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_mapper_bcv (hdr_constellation, data_constellation, preamble, msgq_limit, occupied_carriers, fft_length, tdma, id, source, batch_size, dst_id));
}

// Consumes 1 packet and produces as many OFDM symbols of fft_length to hold the full packet
digital_ofdm_mapper_bcv::digital_ofdm_mapper_bcv (const std::vector<gr_complex> &hdr_constellation, 
					const std::vector<gr_complex> &data_constellation,
					const std::vector<std::vector<gr_complex> > &preamble,
					unsigned int msgq_limit, 
					unsigned int occupied_carriers, unsigned int fft_length, 
					unsigned int tdma, unsigned int id,
					unsigned int source,
					unsigned int batch_size,
					unsigned int dst_id) 
  : gr_sync_block ("ofdm_mapper_bcv",
		   gr_make_io_signature (0, 0, 0),
		   gr_make_io_signature4 (1, 4, sizeof(gr_complex)*fft_length, sizeof(short), sizeof(char), sizeof(char)*fft_length)),
    d_hdr_constellation(hdr_constellation),
    d_data_constellation(data_constellation),
    d_msgq(gr_make_msg_queue(msgq_limit)), d_eof(false),
    d_occupied_carriers(occupied_carriers),
    d_fft_length(fft_length),
    d_pending_flag(0),
    d_batch_size(batch_size),
    d_source(source),
    d_modulated(true),	// start afresh //
    d_send_null(false),
    d_pkt_num(0),
    d_id(id + '0'),
    d_dst_id(dst_id+'0'),
    d_preambles_sent(0),
    d_preamble(preamble),
    d_flow(0),
    d_tdma(tdma)
{
  if (!(d_occupied_carriers <= d_fft_length))
    throw std::invalid_argument("digital_ofdm_mapper_bcv: occupied carriers must be <= fft_length");

  // sanity check preamble symbols
  for (size_t i = 0; i < d_preamble.size(); i++){
    if (d_preamble[i].size() != (size_t) d_fft_length)
      throw std::invalid_argument("digital_ofdm_mapper_bcv: invalid length for preamble symbol");
  }

  std::string arg("");
  d_usrp = uhd::usrp::multi_usrp::make(arg);
  d_out_pkt_time = uhd::time_spec_t(0.0);
  d_last_pkt_time = uhd::time_spec_t(0.0);

  assign_subcarriers();
  // make sure we stay in the limit currently imposed by the occupied_carriers
  if(d_data_carriers.size() > d_occupied_carriers) {
    throw std::invalid_argument("digital_ofdm_mapper_bcv: subcarriers allocated exceeds size of occupied carriers");
  }
  fill_all_carriers_map();  

  d_hdr_nbits = (unsigned long)ceil(log10(float(d_hdr_constellation.size())) / log10((float) 2.0));
  d_data_nbits = (unsigned long)ceil(log10(float(d_data_constellation.size())) / log10((float) 2.0));

  printf("d_data_nbits: %d, const size: %d, d_hdr_bits: %d, const size: %d\n", d_data_nbits, d_data_constellation.size(), d_hdr_nbits, d_hdr_constellation.size()); fflush(stdout);

  d_num_ofdm_symbols = 0;
  d_ofdm_symbol_index = 0;
  d_num_hdr_symbols = (HEADERBYTELEN*8)/d_data_carriers.size();

  d_time_pkt_sent = 0;

  populateFlowInfo();
  populateRouteInfo();
  populateEthernetAddress();
  create_ack_socks();

  if(d_tdma) {
    create_scheduler_sock();
    d_request_id = 0;
  }
}

digital_ofdm_mapper_bcv::~digital_ofdm_mapper_bcv(void)
{
}

/* builds a single OFDM symbol */
void
digital_ofdm_mapper_bcv::generateOFDMSymbol(gr_complex* out, int len)
{
  unsigned int i = 0;
  unsigned char bits = 0;
  while((d_msg_offset < len) && (i < d_data_carriers.size())) {

    // need new data to process
    if(d_bit_offset == 0) {
      d_msgbytes = d_msg->msg()[d_msg_offset];
      //printf("mod message byte: %x\n", d_msgbytes);
    }

    if(d_nresid > 0) {
      // take the residual bits, fill out nbits with info from the new byte, and put them in the symbol
      d_resid |= (((1 << d_nresid)-1) & d_msgbytes) << (d_hdr_nbits - d_nresid);
      bits = d_resid;

      out[d_data_carriers[i]] = d_hdr_constellation[bits];
      i++;

      d_bit_offset += d_nresid;
      d_nresid = 0;
      d_resid = 0;
    }
    else {
      if((8 - d_bit_offset) >= d_hdr_nbits) {  // test to make sure we can fit nbits
        // take the nbits number of bits at a time from the byte to add to the symbol
        bits = ((1 << d_hdr_nbits)-1) & (d_msgbytes >> d_bit_offset);
        d_bit_offset += d_hdr_nbits;

        out[d_data_carriers[i]] = d_hdr_constellation[bits];
        i++;
      }
      else {  // if we can't fit nbits, store them for the next 
        // saves d_nresid[0] bits of this message where d_nresid[0] < d_nbits
        unsigned int extra = 8-d_bit_offset;
        d_resid = ((1 << extra)-1) & (d_msgbytes >> d_bit_offset);
        d_bit_offset += extra;
        d_nresid = d_hdr_nbits - extra;
      }

    }

    //printf("d_bit_offset[0]: %d, d_msg_offset[0]: %d\n", d_bit_offset[0], d_msg_offset[0]); fflush(stdout);

    if(d_bit_offset == 8) {
      d_bit_offset = 0;
      d_msg_offset++;
    }
  }

  // Ran out of data to put in symbol
  if (d_msg_offset == len) {
    if(d_nresid > 0) {
      d_resid |= 0x00;
      bits = d_resid;
      d_nresid = 0;
      d_resid = 0;
    }

    while(i < d_data_carriers.size() && (d_ack || d_default)) {   // finish filling out the symbol
      out[d_data_carriers[i]] = d_hdr_constellation[randsym()];

      i++;
    }

    assert(d_bit_offset == 0);
  }
}


/***************************** integrating SOURCE related functionality to this mapper **************************/
int digital_ofdm_mapper_bcv::randsym()
{
  return (rand() % d_hdr_constellation.size());
}

/* non-lossy synchronous ACKs/NACKs, since they are sent over the ethernet. 
Either will receive an ACK or a NACK */
inline void
digital_ofdm_mapper_bcv::check_for_ack(FlowInfo *fInfo) {
  char ack_buf[ACK_HEADERDATALEN];
  int buf_size = ACK_HEADERDATALEN;
  memset(ack_buf, 0, buf_size);

  unsigned char flowId = fInfo->flowId;
  int n_extracted = 0;
  int rx_sock = d_ack_rx_socks[0];
  printf("check_for_ack, rx_sock: %d\n", rx_sock); fflush(stdout);
  bool found_ack = false;
  int nbytes = recv(rx_sock, ack_buf, buf_size, MSG_PEEK);
  if(nbytes > 0) {
     int offset = 0;
     while(1) {
         nbytes = recv(rx_sock, ack_buf+offset, buf_size-offset, 0);
         if(nbytes > 0) offset += nbytes;
         if(offset == buf_size) {

	     found_ack = true;
	     MULTIHOP_ACK_HDR_TYPE ack;
       	     memcpy(&ack, ack_buf, buf_size);

	     unsigned char ack_flow_id = ((int) ack.flow_id) + '0';
	     int ack_pkt_type = ((int) ack.pkt_type);	

	     printf("ACK received - pkt_type: %d, dst: %c, flow: %c\n", ack_pkt_type, ack.dst_id, ack_flow_id); fflush(stdout);
	     if(ack_pkt_type == ACK_TYPE && ack_flow_id == flowId && ack.dst_id == d_id) {
		assert(fInfo->active_batch == ack.batch_number);
	        if(fInfo->active_batch == ack.batch_number) {
		   fInfo->last_batch_acked = ack.batch_number;
       		}
		printf("#################### ACK details, batch: %d, flow: %c ##################################\n",
						    ack.batch_number, ack_flow_id); fflush(stdout);
	     }
	     else if(ack_pkt_type == NACK_TYPE && ack_flow_id == flowId && ack.dst_id == d_id) {
		printf("#################### NACK details, batch: %d, flow: %c ##################################\n",
                                                    ack.batch_number, ack_flow_id); fflush(stdout);
             }

            n_extracted++;
#if 0
	    // see if there's another packet //
	    memset(ack_buf, 0, buf_size);
	    nbytes = recv(rx_sock, ack_buf, buf_size, MSG_PEEK);
	    if(nbytes > 0) {
		offset = 0; 
		continue;
	    }
#endif
            break;
         }
     }
     assert(offset == buf_size);
   }
   printf("check_for_ack - flowId: %c, n_extracted: %d\n", flowId, n_extracted); 
}

bool
digital_ofdm_mapper_bcv::have_packet_to_send() {
   if(d_source) 
      return true;		// source always has a packet to send right now

   if(d_msgq->empty_p())
      return false;
   return true;
}

void
digital_ofdm_mapper_bcv::print_msg(gr_message_sptr msg) {
   int len = msg->length();
   unsigned char *data = msg->msg();

   for(int i = 0; i < len; i++) {
     printf("%02x", (unsigned char) data[i]);
   }
   printf("\n");
}

int
digital_ofdm_mapper_bcv::work(int noutput_items,
                              gr_vector_const_void_star &input_items,
                              gr_vector_void_star &output_items)
{
  FlowInfo *flowInfo = getFlowInfo(false, d_flow);
  if(d_modulated && flowInfo->pkts_fwded > 0) {
     check_for_ack(flowInfo);
  }

  if(d_modulated) {
     /*
     while(!have_packet_to_send()) {
	sleep(0.5);
     }*/

     make_packet(flowInfo);				// blocking if no msg to send //
     print_msg(d_msg);
     if(d_tdma) {
        send_scheduler_msg(REQUEST_INIT_MSG);
        while(!check_scheduler_reply()) {
           sleep(0.65);
        }
     }
     make_time_tag();
  }

  return modulate_and_send(noutput_items, input_items, output_items, flowInfo);
}

void
digital_ofdm_mapper_bcv::make_packet(FlowInfo *flowInfo)
{
  assert(d_modulated);
  // if last batch was acked, then dequeue a fresh one //
  if(flowInfo->last_batch_acked == flowInfo->active_batch) {
     flowInfo->active_batch++;
     printf("start active_batch: %d, last_batch_acked: %d\n", flowInfo->active_batch, flowInfo->last_batch_acked); fflush(stdout);
     for(unsigned int b = 0; b < d_batch_size; b++) {
	 d_msg.reset();
	 d_msg = d_msgq->delete_head();		// dequeue the pkts from the queue //
	 assert(d_msg->length() > 0);	
	 d_packetlen = d_msg->length();		// assume: all pkts in batch are of same length //
	 d_num_ofdm_symbols = ceil(((float) ((d_packetlen) * 8))/(d_data_carriers.size() * d_data_nbits)); // include 'x55'
	 printf("d_num_ofdm_symbols: %d, d_num_hdr_symbols: %d\n", d_num_ofdm_symbols, d_num_hdr_symbols); fflush(stdout);

	 makeHeader(flowInfo);
     }
  }
  else {
     // retransmission! //
     assert(flowInfo->last_batch_acked < flowInfo->active_batch);
     printf("retransmit batch: %d, last_batch_acked: %d, pkts_sent: %d\n", 
		flowInfo->active_batch, flowInfo->last_batch_acked, flowInfo->pkts_fwded); fflush(stdout);
  }

  d_modulated = false;
  initializeBatchParams();
  d_pending_flag = 1;                             // start of packet flag //

  if(((HEADERBYTELEN*8) % d_data_carriers.size()) != 0) {	// assuming bpsk //
     printf("HEADERBYTELEN: %d, size: %d\n", HEADERBYTELEN, d_data_carriers.size()); fflush(stdout);
     assert(false);
  }

  initHeaderParams();

  d_preambles_sent = d_ofdm_symbol_index = d_null_symbol_cnt = d_hdr_ofdm_index = 0; 
  d_pkt_num++;
}

int
digital_ofdm_mapper_bcv::modulate_and_send(int noutput_items,
	  		      gr_vector_const_void_star &input_items,
  			      gr_vector_void_star &output_items,
			      FlowInfo *flowInfo) {
  bool tx_pilot = false;

  // the final result will be in 'out' //
  gr_complex *out = (gr_complex *)output_items[0];
  memset(out, 0, sizeof(gr_complex) * d_fft_length);

  /* for burst tagger trigger */
  unsigned short *burst_trigger = NULL;
  if(output_items.size() >= 2) {
     burst_trigger = (unsigned short*) output_items[1];
     memset(burst_trigger, 0, sizeof(short));
  }
  burst_trigger[0] = 1;
  /* end */

  /* optional - for out timing signal required only for debugging */
  unsigned char *out_signal = NULL;
  if (output_items.size() >= 3){
    out_signal = (unsigned char*) output_items[3];
    memset(out_signal, 0, sizeof(char) * d_fft_length);
  }
  /* end */

  if(d_preambles_sent < d_preamble.size()) {
     generatePreamble(out);
     d_preambles_sent++;
     tx_pilot = false;
     if(d_preambles_sent == 1 && output_items.size() >= 3) out_signal[0]=1;
  }
  else if(d_hdr_byte_offset < HEADERBYTELEN) {
      generateOFDMSymbolHeader(out); 					// send the header symbols out first //
      tx_pilot = true;
  }
  else if(!d_send_null) {

      assert(d_pending_flag == 0);
	// send the data symbols now //
      generateOFDMSymbolData(out); 

 	// offline, timekeeping, etc //
      assert(d_ofdm_symbol_index < d_num_ofdm_symbols);
      d_ofdm_symbol_index++;

      tx_pilot = true;
  }
  else {
      // send a NULL symbol at the end //
      d_pending_flag = 2;
      flowInfo->pkts_fwded++;
      assert(d_ofdm_symbol_index == d_num_ofdm_symbols);
      d_send_null = false;
      d_modulated = true;
      flowInfo->pkts_fwded++;
      if(d_tdma) send_scheduler_msg(REQUEST_COMPLETE_MSG);
      d_request_id++;
  }

  // now add the pilot symbols //
  if(tx_pilot) {
      double cur_pilot = 1.0;
      for(int i = 0; i < d_pilot_carriers.size(); i++) {
         out[d_pilot_carriers[i]] = gr_complex(cur_pilot, 0.0);
         cur_pilot = -cur_pilot;
      }
  }
  else if(d_ofdm_symbol_index > 0) {
      for(int i = 0; i < d_pilot_carriers.size(); i++) {
	out[d_pilot_carriers[i]] = gr_complex(0.0, 0.0);
      }
  }

  char *out_flag = 0;
  if(output_items.size() >= 3)
    out_flag = (char *) output_items[2];

  if (out_flag)
    out_flag[0] = d_pending_flag;

  if(d_pending_flag == 2) {
    burst_trigger[0] = 0;
  }

  d_pending_flag = 0;  
  return 1;
}

void
digital_ofdm_mapper_bcv::assign_subcarriers() {
  int dc_tones = 8;
  int num_pilots = 8;
  int pilot_gap = 11;

  int half1_end = (d_occupied_carriers-dc_tones)/2;     //40
  int half2_start = half1_end+dc_tones;                 //48

  int off = (d_fft_length-d_occupied_carriers)/2;	//4
  
  // first half
  for(int i = 0; i < half1_end; i++) {
     if(i%pilot_gap == 0)
        d_pilot_carriers.push_back(i+off);
     else
        d_data_carriers.push_back(i+off);
  }

  // second half
  for(int i = half2_start, j = 0; i < d_occupied_carriers; i++, j++) {
     if(j%pilot_gap == 0)
        d_pilot_carriers.push_back(i+off);
     else
        d_data_carriers.push_back(i+off);
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
digital_ofdm_mapper_bcv::fill_all_carriers_map() {
  d_all_carriers.resize(d_occupied_carriers);

  unsigned int left_half_fft_guard = (d_fft_length - d_occupied_carriers)/2;

  unsigned int p = 0, d = 0, dc = 0;

  for(unsigned int i = 0; i < d_occupied_carriers; i++) {
      int carrier_index = left_half_fft_guard + i;

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

#ifdef DEBUG
  printf("fill_all_carriers: --- -\n"); fflush(stdout);
  for(int i = 0; i < d_all_carriers.size(); i++) {
	printf("i: %d ----- val: %d\n", i, d_all_carriers[i]); fflush(stdout);
  }
  printf(" ---------------------------------------------------- \n"); fflush(stdout);
#endif

  assert(d == d_data_carriers.size());
  assert(p == d_pilot_carriers.size());
  assert(dc == d_occupied_carriers - d_data_carriers.size() - d_pilot_carriers.size());
}

inline void
digital_ofdm_mapper_bcv::initializeBatchParams()
{
  d_msg_offset = d_bit_offset = d_nresid = d_resid = 0;
}

// generate an OFDM symbol *each* for the packets involved //
void
digital_ofdm_mapper_bcv::generateOFDMSymbolData(gr_complex* out)
{
  // Build a single symbol: P.S: this depends on the all msgs being equal in length ! 
  unsigned int i = 0;
  unsigned char bits = 0;
  while((d_msg_offset < d_msg->length()) && (i < d_data_carriers.size())) {

    // need new data to process
    if(d_bit_offset == 0) {
      d_msgbytes = d_msg->msg()[d_msg_offset];
    }

    if(d_nresid > 0) {
      // take the residual bits, fill out nbits with info from the new byte, and put them in the symbol
      d_resid |= (((1 << d_nresid)-1) & d_msgbytes) << (d_data_nbits - d_nresid);
      bits = d_resid;

      out[d_data_carriers[i]] = d_data_constellation[bits];
      i++;

      d_bit_offset += d_nresid;
      d_nresid = 0;
      d_resid = 0;
    }
    else {
      if((8 - d_bit_offset) >= d_data_nbits) {  // test to make sure we can fit nbits
        // take the nbits number of bits at a time from the byte to add to the symbol
        bits = ((1 << d_data_nbits)-1) & (d_msgbytes >> d_bit_offset);
        d_bit_offset += d_data_nbits;

        out[d_data_carriers[i]] = d_data_constellation[bits];
        i++;
      }
      else {  // if we can't fit nbits, store them for the next 
        // saves d_nresid bits of this message where d_nresid < d_data_nbits
        unsigned int extra = 8-d_bit_offset;
        d_resid = ((1 << extra)-1) & (d_msgbytes >> d_bit_offset);
        d_bit_offset += extra;
        d_nresid = d_data_nbits - extra;
      }

    }

    if(d_bit_offset == 8) {
      d_bit_offset = 0;
      d_msg_offset++;
    }
  }

  // Ran out of data to put in symbol
  if (d_msg_offset == d_msg->length()) {
    if(d_nresid > 0) {
      d_resid |= 0x00;
      bits = d_resid;
      d_nresid = 0;
      d_resid = 0;
    }

#if 0
    while(i < d_data_carriers.size()) {   // finish filling out the symbol
      out[d_data_carriers[i]] = d_constellation[randsym()];
      i++;
    }
#endif

    //printf("complete pkt modulated\n"); fflush(stdout);
    //d_modulated = true;		// complete msg has been demodulated //
    d_send_null = true;			// send a null symbol at the end //

    assert(d_bit_offset == 0);

  }
}

void
digital_ofdm_mapper_bcv::initHeaderParams()
{
  d_hdr_resid = 0;
  d_hdr_nresid = 0;
  d_hdr_byte_offset = 0;
  d_hdr_byte = 0;
  d_hdr_bit_offset = 0;
}

/* this is a little dumb right now to have an almost duplicate 
   implementation. Keep it till I figure out a better way 
    generate an OFDM symbol *each* for the packets involved 
*/
void
digital_ofdm_mapper_bcv::generateOFDMSymbolHeader(gr_complex* out)
{
  unsigned int i = 0;
  unsigned char bits = 0;
  while((d_hdr_byte_offset < HEADERBYTELEN) && (i < d_data_carriers.size())) {

    // need new data to process
    if(d_hdr_bit_offset == 0) {
      d_hdr_byte = d_header_bytes[d_hdr_byte_offset];
    }

    if(d_hdr_nresid > 0) {
      // take the residual bits, fill out nbits with info from the new byte, and put them in the symbol
      d_hdr_resid |= (((1 << d_hdr_nresid)-1) & d_hdr_byte) << (d_hdr_nbits - d_hdr_nresid);
      bits = d_hdr_resid;

      out[d_data_carriers[i]] = d_hdr_constellation[bits];
      i++;

      d_hdr_bit_offset += d_hdr_nresid;
      d_hdr_nresid = 0;
      d_hdr_resid = 0;
    }
    else {
      if((8 - d_hdr_bit_offset) >= d_hdr_nbits) {  // test to make sure we can fit nbits
        // take the nbits number of bits at a time from the byte to add to the symbol
        bits = ((1 << d_hdr_nbits)-1) & (d_hdr_byte >> d_hdr_bit_offset);
        d_hdr_bit_offset += d_hdr_nbits;

        out[d_data_carriers[i]] = d_hdr_constellation[bits];
	//printf("fill sc: %d\n", d_data_carriers[i]); fflush(stdout);
        i++;
      }
      else {  // if we can't fit nbits, store them for the next 
        // saves d_hdr_nresid bits of this message where d_hdr_nresid < d_nbits
        unsigned int extra = 8-d_hdr_bit_offset;
        d_hdr_resid = ((1 << extra)-1) & (d_hdr_byte >> d_hdr_bit_offset);
        d_hdr_bit_offset += extra;
        d_hdr_nresid = d_hdr_nbits - extra;
      }

    }

    if(d_hdr_bit_offset == 8) {
      d_hdr_bit_offset = 0;
      d_hdr_byte_offset++;
    }
  }

  // Ran out of data to put in symbol
  if (d_hdr_byte_offset == HEADERBYTELEN) {
    printf("hdr done!\n"); fflush(stdout);
    if(d_hdr_nresid > 0) {
      d_hdr_resid |= 0x00;
      bits = d_hdr_resid;
      d_hdr_nresid = 0;
      d_hdr_resid = 0;
    }

    while(i < d_data_carriers.size()) {   // finish filling out the symbol
      out[d_data_carriers[i]] = d_hdr_constellation[randsym()];

      i++;
    }

    assert(d_hdr_bit_offset == 0);
  }
}

/* populate header_data_bytes and also d_header 
   add all the header details:
	src_id
	rx_id
	packet_len
	batch_number
	n_senders
	pkt_type
	hdr crc
*/
void
digital_ofdm_mapper_bcv::makeHeader(FlowInfo *fInfo)
{
   memset(d_header_bytes, 0, HEADERBYTELEN);
   if(d_source) {
      memset(&d_header, 0, sizeof(d_header));

      d_header.flow_id = fInfo->flowId;
      d_header.src_id = d_id;
      d_header.dst_id = d_dst_id;
      d_header.prev_hop_id = d_id;
      d_header.next_hop_id = fInfo->nextHopId;

      d_header.packetlen = d_packetlen - 1;		// -1 for '55' appended (ref ofdm_packet_utils)
      d_header.batch_number = fInfo->active_batch;
      d_header.pkt_type = DATA_TYPE;
      d_header.pkt_num = d_pkt_num;			// unique number across all flows
      d_header.link_id = 0;
   }
   else {
      d_header = d_msg->header();
   }

   printf("makeHeader for batch: %d, pkt: %d, len: %d\n", d_header.batch_number, d_header.pkt_num, d_header.packetlen); 
   fflush(stdout);

   d_last_pkt_time = d_out_pkt_time;  

   unsigned char header_data_bytes[HEADERDATALEN];
   memcpy(header_data_bytes, &d_header, HEADERDATALEN);				// copy e'thing but the crc

   unsigned int calc_crc = digital_crc32(header_data_bytes, HEADERDATALEN);
   d_header.hdr_crc = calc_crc;

   memcpy(d_header_bytes, header_data_bytes, HEADERDATALEN);				// copy header data
   memcpy(d_header_bytes+HEADERDATALEN, &calc_crc, sizeof(int));		// copy header crc

   printf("len: %d, crc: %u\n", d_packetlen, calc_crc);

   //debugHeader();  
   whiten();
   //debugHeader();
}

void
digital_ofdm_mapper_bcv::debugHeader()
{
   whiten();	// dewhitening effect !
   memset(&d_header, 0, HEADERBYTELEN);
   assert(HEADERBYTELEN == (HEADERDATALEN + sizeof(int) + PADDING_SIZE));
   memcpy(&d_header, d_header_bytes, HEADERBYTELEN);

   printf("debug crc: %u\n", d_header.hdr_crc); 
   printf("\n"); 
   whiten();
}

void
digital_ofdm_mapper_bcv::whiten()
{
   for(int i = 0; i < HEADERBYTELEN; i++)
   {
	d_header_bytes[i] = d_header_bytes[i] ^ random_mask_tuple1[i];
   }
}

/* ack over ethernet */
int
digital_ofdm_mapper_bcv::openACKSocket() {
  int sock_port = 9000;
  int sockfd;
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
  listen(sockfd, 20);
  if(sockfd != -1) {
    printf("ACK Socket OPEN success\n"); fflush(stdout);
  }
  else {
    printf("ACK Socket OPEN error\n"); fflush(stdout);
  }

  return sockfd;
}

int
digital_ofdm_mapper_bcv::isACKSocketOpen() {
  if(!d_sock_opened) 
    return 0;

  return 1;
}

/* in the work_source() mode to just test if trigger on ethernet works */
inline void
digital_ofdm_mapper_bcv::make_time_tag() {

  /* calculate the time reqd to send the preamble+header */
  int decimation = 128;
  double rate = 1.0/decimation;
 
  int num_ofdm_symbols_to_wait = 4000; //400; //3000;

  int cp_length = d_fft_length/4;
  int symbol_length = d_fft_length + cp_length;
  uint32_t num_samples = num_ofdm_symbols_to_wait * symbol_length;
  double time_per_sample = 1 / 100000000.0 * (int)(1/rate);
  double duration = num_samples * time_per_sample;

  uint64_t sync_secs = (uint64_t) duration;
  double sync_frac_of_secs = duration - (uint64_t) duration;
  uhd::time_spec_t duration_time = uhd::time_spec_t(sync_secs, sync_frac_of_secs);

  uhd::time_spec_t c_time = d_usrp->get_time_now();           // current time //
  uhd::time_spec_t out_time = c_time + duration_time;         // scheduled out time //

  /* check if the interval from the last packet sent is atleast the duration */
  uhd::time_spec_t interval_time = out_time - d_last_pkt_time;    // interval between the transmissions //
  if(interval_time < duration_time) {
     uhd::time_spec_t extra_gap_time = duration_time - interval_time;
     out_time += extra_gap_time;
     interval_time += extra_gap_time;
  }

  uint64_t interval_samples = (interval_time.get_frac_secs() * 1e8)/decimation;

  printf("timestamp: curr (%llu, %f), out (%llu, %f) , last_time (%llu, %f), interval(%lld, %f), interval_samples(%llu), duration(%llu, %f)\n", (uint64_t) c_time.get_full_secs(), c_time.get_frac_secs(), (uint64_t) out_time.get_full_secs(), out_time.get_frac_secs(), (uint64_t) d_last_pkt_time.get_full_secs(), d_last_pkt_time.get_frac_secs(), (int64_t) interval_time.get_full_secs(), interval_time.get_frac_secs(), interval_samples, sync_secs, sync_frac_of_secs); fflush(stdout);

  assert(out_time > c_time);
  d_out_pkt_time = out_time;

  const pmt::pmt_t key = pmt::pmt_string_to_symbol("tx_time");
  const pmt::pmt_t value = pmt::pmt_make_tuple(
		  pmt::pmt_from_uint64((uint64_t) out_time.get_full_secs()),
		  pmt::pmt_from_double(out_time.get_frac_secs())
		  );
  const pmt::pmt_t srcid = pmt::pmt_string_to_symbol(this->name());
  add_item_tag(1/*chan0*/, nitems_written(1), key, value, srcid);
  printf("(MAPPER) make_time_tag, at offset: %llu\n", nitems_written(1)); fflush(stdout);
}

inline void
digital_ofdm_mapper_bcv::generatePreamble(gr_complex *out) {
    memcpy(out, &d_preamble[d_preambles_sent][0], d_fft_length*sizeof(gr_complex));
}

/* utility functions start */
inline void
digital_ofdm_mapper_bcv::open_server_sock(int sock_port, vector<unsigned int>& connected_clients, int num_clients) {
  int sockfd, _sock;
  struct sockaddr_in dest;

  printf("open_server_sock, #clients: %d\n", num_clients); fflush(stdout);
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
      connected_clients.push_back(_sock);
      conn++;
    }
  }
  
  printf("open_server_sock done, #clients: %d\n", num_clients); fflush(stdout);
  assert(connected_clients.size() == num_clients);
}

inline int
digital_ofdm_mapper_bcv::open_client_sock(int port, const char *addr, bool blocking) {
  int sockfd;
  struct sockaddr_in dest;

  struct timeval tv;
  tv.tv_sec = 1;
  tv.tv_usec = 5e5;

  /* create socket */
  sockfd = socket(PF_INET, SOCK_STREAM, 0);
  if(!blocking)
     fcntl(sockfd, F_SETFL, O_NONBLOCK);

  assert(setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval))==0);
  assert(sockfd != -1);

  /* initialize value in dest */
  bzero(&dest, sizeof(struct sockaddr_in));
  dest.sin_family = PF_INET;
  dest.sin_port = htons(port);
  dest.sin_addr.s_addr = inet_addr(addr);
  /* Connecting to server */
  while(connect(sockfd, (struct sockaddr*)&dest, sizeof(struct sockaddr_in)) == -1)
	continue;
  printf("sockfd: %d, errno: %d\n", sockfd, errno); fflush(stdout);
  return sockfd;
}

inline void
digital_ofdm_mapper_bcv::populateEthernetAddress()
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

        //std::cout<<"Inserting in the map: node " << node_id << " addr: " << ethInfo->addr << endl;
   }

    fclose (fl);
}

/* d_ack_tx_socks (clients to which I will send an ACK)
   d_ack_rx_socks (servers from which I will receive an ACK)
   all clients/servers must be unique 
*/
inline void
digital_ofdm_mapper_bcv::create_ack_socks() {
  printf("(mapper):: create_ack_socks\n"); fflush(stdout);
  EthInfoMap::iterator e_it;
  EthInfo *ethInfo = NULL;
#if 0
  // clients to which I will send an ACK // 
  int num_clients = d_prevHopIds.size();
  EthInfoMap::iterator e_it = d_ethInfoMap.find(d_id);
  assert(e_it != d_ethInfoMap.end());
  EthInfo *ethInfo = (EthInfo*) e_it->second;
  open_server_sock(ethInfo->port, d_ack_tx_socks, num_clients);
#endif

  // connect to servers from which I will receive an ACK //
  map<unsigned char, bool>:: iterator it = d_nextHopIds.begin();

  //struct timeval tv;
  //tv.tv_sec = 1;
  //tv.tv_usec = 5e5;

  while(it != d_nextHopIds.end()) {
     NodeId nodeId = it->first;
     e_it = d_ethInfoMap.find(nodeId); 
     assert(e_it != d_ethInfoMap.end());
     ethInfo = (EthInfo*) e_it->second;
     int rx_sock = open_client_sock(ethInfo->port, ethInfo->addr, true);

     //assert(setsockopt(rx_sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval))==0);
     d_ack_rx_socks.push_back(rx_sock);
     it++;
  }
  assert(d_ack_rx_socks.size() == d_nextHopIds.size());
  printf("(mapper): create_ack_socks num_socks: %d done!\n", d_ack_rx_socks.size()); fflush(stdout);
}

/* read shortest path info */
inline void
digital_ofdm_mapper_bcv::populateRouteInfo() {
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

	printf("src: %c, dst: %c, flowInfo.src: %c, flowInfo.dst: %c\n", src, dst, flowInfo->src, flowInfo->dst); fflush(stdout);
	assert(flowInfo->src == src && flowInfo->dst == dst); 			// sanity

        for(int i = 0; i < num_nodes; i++) {
           NodeId nodeId = atoi(token_vec[i+2]) + '0';
	   printf("nodeId: %c\n", nodeId); fflush(stdout);
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
	 	 printf("here!\n"); fflush(stdout);
	      }
	      break;
	   }
        }

        printf("num_prevHops: %d, num_nextHops: %d\n", d_prevHopIds.size(), d_nextHopIds.size()); 
        printf("\n");
   }
   fclose (fl);
}

inline FlowInfo*
digital_ofdm_mapper_bcv::getFlowInfo(bool create, unsigned char flowId)
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
     flow_info->flowId = flowId;
     d_flowInfoVector.push_back(flow_info);
  }

  return flow_info;
}

inline void
digital_ofdm_mapper_bcv::populateFlowInfo() {
    // flowId	srcId	dstId //
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
        vector<char*> token_vec;
        while (strtok_res != NULL) {
           token_vec.push_back(strtok_res);
	   //printf ("%s\n", strtok_res);
           strtok_res = strtok (NULL, delim);
        }
	
	unsigned char flowId = atoi(token_vec[0])+'0';
	FlowInfo *fInfo = getFlowInfo(true, flowId);
      
	fInfo->src = atoi(token_vec[1])+'0';
 	fInfo->dst = atoi(token_vec[2])+'0';
	printf("flowId: %c, src: %c, dst: %c\n", flowId, fInfo->src, fInfo->dst); 
   }

   fclose (fl);
}

inline void
digital_ofdm_mapper_bcv::send_scheduler_msg(int type) {
   printf("send_scheduler_msg type: %d, request_id: %d\n", type, d_request_id); fflush(stdout);
   SchedulerMsg request;
   request.request_id = d_request_id;
   request.nodeId = d_id;
   request.type = type;

   int buf_size = sizeof(SchedulerMsg);
   char *buf = (char*) malloc(buf_size);
   memcpy(buf, &request, buf_size);

   if(send(d_scheduler_sock, buf, buf_size, 0) < 0) { 
      printf("errno: %d\n", errno); fflush(stdout);
      assert(false);
   }

   free(buf);
}

inline bool
digital_ofdm_mapper_bcv::check_scheduler_reply() {
   int buf_size = sizeof(SchedulerMsg);
   char *buf = (char*) malloc(buf_size);
   memset(buf, 0, buf_size);

   SchedulerMsg reply;   
   reply.request_id = -1;

   int nbytes = recv(d_scheduler_sock, buf, buf_size, MSG_PEEK);
   if(nbytes > 0) {
      int offset = 0;
      while(1) {
	nbytes = recv(d_scheduler_sock, buf+offset, buf_size-offset, 0);
	if(nbytes > 0) offset += nbytes;
	if(offset == buf_size) {
	   memcpy(&reply, buf, buf_size);
	   assert(reply.type == REPLY_MSG);

	   printf("check_scheduler_reply, type: %d, request_id: %d, nodeId: %c\n", 
				reply.type, reply.request_id, reply.nodeId); fflush(stdout);
	   break;
	}
      }
   }
   free(buf);

   bool success = (reply.request_id == d_request_id && reply.nodeId == d_id);
   return success;
}

inline void
digital_ofdm_mapper_bcv::create_scheduler_sock() {
   printf("(mapper): create_scheduler_sock\n"); fflush(stdout);
   d_scheduler_sock = open_client_sock(9000, "128.83.120.84", true);
}
