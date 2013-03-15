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
#include <bitset>

#include "GaloisField.h"
#include "NetCoder.h"

digital_ofdm_mapper_bcv_sptr
digital_make_ofdm_mapper_bcv (const std::vector<gr_complex> &hdr_constellation, 
			 const std::vector<gr_complex> &data_constellation,
			 const std::vector<std::vector<gr_complex> > &preamble,
			 unsigned int msgq_limit, 
                         unsigned int occupied_carriers, unsigned int fft_length, 
			 unsigned int tdma, unsigned int proto, unsigned int ack_mode,
			 unsigned int id,
			 unsigned int source,
			 unsigned int batch_size,
			 unsigned int dst_id,
			 unsigned int fec_n, unsigned int fec_k)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_mapper_bcv (hdr_constellation, data_constellation, preamble, msgq_limit, occupied_carriers, fft_length, tdma, proto, ack_mode, id, source, batch_size, dst_id, 
								 fec_n, fec_k));
}

// Consumes 1 packet and produces as many OFDM symbols of fft_length to hold the full packet
digital_ofdm_mapper_bcv::digital_ofdm_mapper_bcv (const std::vector<gr_complex> &hdr_constellation, 
					const std::vector<gr_complex> &data_constellation,
					const std::vector<std::vector<gr_complex> > &preamble,
					unsigned int msgq_limit, 
					unsigned int occupied_carriers, unsigned int fft_length, 
					unsigned int tdma, unsigned int proto, unsigned int ack_mode,
					unsigned int id,
					unsigned int source,
					unsigned int batch_size,
					unsigned int dst_id,
					unsigned int fec_n, unsigned int fec_k) 
  : gr_sync_block ("ofdm_mapper_bcv",
		   gr_make_io_signature (0, 0, 0),
		   gr_make_io_signature5 (1, 5, sizeof(gr_complex)*fft_length, sizeof(short), sizeof(short), sizeof(char), sizeof(char)*fft_length)),
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
    d_flow('0'),
    d_tdma(tdma),
    d_proto(proto),
    d_ack(ack_mode),
    fec_N(fec_n), fec_K(fec_k)
{

  srand((int) d_id);
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

  if(((HEADERBYTELEN*8) % d_data_carriers.size()) != 0) {       // assuming bpsk //
     printf("HEADERBYTELEN: %d, size: %d\n", HEADERBYTELEN, d_data_carriers.size()); fflush(stdout);
     assert(false);
  }

  d_hdr_nbits = (unsigned long)ceil(log10(float(d_hdr_constellation.size())) / log10((float) 2.0));
  d_data_nbits = (unsigned long)ceil(log10(float(d_data_constellation.size())) / log10((float) 2.0));

  printf("d_data_nbits: %d, const size: %d, d_hdr_bits: %d, const size: %d\n", d_data_nbits, d_data_constellation.size(), d_hdr_nbits, d_hdr_constellation.size()); fflush(stdout);

  d_num_ofdm_symbols = 0;
  d_ofdm_symbol_index = 0;
  d_num_hdr_symbols = (HEADERBYTELEN*8)/d_data_carriers.size();

  d_time_pkt_sent = 0;

  BitInterleave bi(d_data_carriers.size(), d_data_nbits);
  bi.fill(d_interleave_map, false);

  populateFlowInfo();
  populateEthernetAddress();

  if(d_proto == SPP) {
     populateRouteInfo();
     if(d_ack) create_ack_socks();
  } 
  else if(d_proto == PRO) {
     if(d_ack) create_e2e_ack_socks();
     populateCreditInfo();			// both the weight and red. values //
  }
  else if(d_proto == CF) {
     populateCreditInfo_CF();
     if(d_ack) {
	create_e2e_ack_socks();
	open_trigger_sock_CF();
     }
  }

  if(d_tdma) {
    create_scheduler_sock();
    d_request_id = 0;
  }

  d_pktInfo.symbols = NULL;			// used only for CF+alamouti //
  d_fp = NULL;
}

digital_ofdm_mapper_bcv::~digital_ofdm_mapper_bcv(void)
{
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

  map<FlowId, SockId>::iterator mit = d_sock_map.find(flowId);
  assert(mit != d_sock_map.end());
  SockId rx_sock = mit->second;

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

	     FlowId ack_flow_id = ((int) ack.flow_id) + '0';
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
  /* branch out completely if CF */
  if(d_proto == CF) {
     return work_CF(noutput_items, input_items, output_items);
  }

  FlowInfo *flowInfo = getFlowInfo(false, d_flow);

  /* branch out completely if PRO forwarder */
  if(!d_source) {
     if(d_proto == PRO) 
        return work_forwarder_PRO(noutput_items, input_items, output_items);
  }

  if(d_modulated && flowInfo->pkts_fwded && d_ack) {
     check_for_ack(flowInfo);					// non-blocking //
  }

  if(d_modulated) {
     if(d_proto == SPP)
        make_packet_SPP(flowInfo);				// blocking if no msg to send //
     else if(d_proto == PRO) {
	make_packet_PRO_source(flowInfo);
     }

     print_msg(d_msg);
     if(d_tdma) {
        send_scheduler_msg(REQUEST_INIT_MSG, d_flow);
	SchedulerMsg reply;
        while(!check_scheduler_reply(reply)) {
           sleep(0.65);
        }
     }
     make_time_tag(uhd::time_spec_t(0, 0), false);		// dumb params here
  }

  return modulate_and_send(noutput_items, input_items, output_items, flowInfo);
}

/* this should work for both SPP and CF (unless we decide to send in 'batches') */
void
digital_ofdm_mapper_bcv::make_packet_SPP(FlowInfo *flowInfo)
{
  assert((d_proto==CF && d_source) || (d_proto==SPP));
  assert(d_modulated);
  assert(d_batch_size == 1);
  // if last batch was acked, then dequeue a fresh one //
  if(flowInfo->last_batch_acked == flowInfo->active_batch) {
     flowInfo->active_batch++;
     printf("start active_batch: %d, last_batch_acked: %d\n", flowInfo->active_batch, flowInfo->last_batch_acked); fflush(stdout);
     d_msg.reset();
     gr_message_sptr msg = d_msgq->delete_head();         // dequeue the pkts from the queue //
     assert(msg->length() > 0);

     add_crc_and_fec(msg);	     // creates the final d_msg
     if(!d_source) { 
	assert(d_proto == SPP);
	d_msg->set_header(msg->header());
     }

     d_packetlen = d_msg->length();         // assume: all pkts in batch are of same length //
     d_num_ofdm_symbols = ceil(((float) ((d_packetlen) * 8))/(d_data_carriers.size() * d_data_nbits)); // include 'x55'
     printf("d_num_ofdm_symbols: %d, d_num_hdr_symbols: %d\n", d_num_ofdm_symbols, d_num_hdr_symbols); fflush(stdout);
     makeHeader(flowInfo);
     msg.reset();
  }
  else {
     // retransmission! //
     makeHeader(flowInfo);
     assert(flowInfo->last_batch_acked < flowInfo->active_batch);
     printf("retransmit batch: %d, last_batch_acked: %d, pkts_sent: %d\n", 
		flowInfo->active_batch, flowInfo->last_batch_acked, flowInfo->pkts_fwded); fflush(stdout);
  }

  d_modulated = false;
  initializeBatchParams();
  d_pending_flag = 1;                             // start of packet flag //

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

  /* for CDD */
  unsigned short *cdd = NULL;
  if(output_items.size() >= 3) {
     cdd = (unsigned short*) output_items[2];
     memset(cdd, 0, sizeof(short));
  }
  cdd[0] = 0;
#if 0
  if(!flowInfo->isLead || 1)
     cdd[0] = 2;
#endif
  /* end */

  /* optional - for out timing signal required only for debugging */
  unsigned char *out_signal = NULL;
  if (output_items.size() >= 5){
    out_signal = (unsigned char*) output_items[4];
    memset(out_signal, 0, sizeof(char) * d_fft_length);
  }
  /* end */

#if 1
  if(d_proto == CF && flowInfo->num_tx > 1 && !flowInfo->isLead && d_null_symbol_cnt < (d_preamble.size()+d_num_hdr_symbols)) {
     d_null_symbol_cnt++;	
  } else
#endif
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
#if 1
  else if(d_proto == CF && flowInfo->num_tx > 1 && flowInfo->isLead && d_null_symbol_cnt < (d_preamble.size()+d_num_hdr_symbols)) {
      d_null_symbol_cnt++;
  }
#endif
  else if(d_ofdm_symbol_index < d_num_ofdm_symbols) {

      tx_pilot = true;
      assert(d_pending_flag == 0);

	// send the data symbols now //
      if(d_proto == CF && flowInfo->num_tx > 1) {
	 assert(d_num_ofdm_symbols%2 == 0);
	 generateOFDMSymbolData_alamouti(out, flowInfo);
	 if(flowInfo->isLead) tx_pilot = false; 
      } 
      else if(d_proto == CF && !d_source) {
	 copyOFDMSymbolData_CF(out, d_ofdm_symbol_index);
      }
      else 
	 generateOFDMSymbolData(out); 

 	// offline, timekeeping, etc //
      d_ofdm_symbol_index++;
  }
  else {
      // send a NULL symbol at the end //
       int ii=0;
#if 0
       while(ii < d_data_carriers.size()) {   // finish filling out the symbol
         out[d_data_carriers[ii]] = d_data_constellation[randsym()];
         ii++;
      }
#endif
      d_pending_flag = 2;
      flowInfo->pkts_fwded++;
      assert(d_ofdm_symbol_index == d_num_ofdm_symbols);
      d_send_null = false;
      d_modulated = true;
      flowInfo->pkts_fwded++;
      if(d_tdma && ((d_proto==CF && !d_source && flowInfo->isLead) || (d_proto==CF && d_source) || (d_proto != CF))) 
	 send_scheduler_msg(REQUEST_COMPLETE_MSG, flowInfo->flowId);
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
  if(output_items.size() >= 4)
    out_flag = (char *) output_items[3];

  if (out_flag)
    out_flag[0] = d_pending_flag;

  if(d_pending_flag == 2) {
    burst_trigger[0] = 0;
  }

  d_pending_flag = 0;  
  return 1;
}

void
digital_ofdm_mapper_bcv::logDataSymbols(gr_complex *out)
{
  //printf("digital_ofdm_mapper_bcv::logDataSymbols\n"); fflush(stdout);
  if(d_fp == NULL) {
      printf("opening file\n"); fflush(stdout);
      int fd;
      const char *filename = "known_data.dat";
      if ((fd = open (filename, O_WRONLY|O_CREAT|O_TRUNC|OUR_O_LARGEFILE|OUR_O_BINARY|O_APPEND, 0664)) < 0) {
         perror(filename);
         assert(false);
      }
      else {
         if((d_fp = fdopen (fd, true ? "wb" : "w")) == NULL) {
              fprintf(stderr, "log file cannot be opened\n");
              close(fd);
              assert(false);
         }
      }
  }

  int nc = d_data_carriers.size();
  gr_complex *log_symbols = (gr_complex*) malloc(sizeof(gr_complex) * nc);

  for(int i = 0; i < nc; i++) {
     memcpy(log_symbols+i, out+d_data_carriers[i], sizeof(gr_complex));
  }

  int count = fwrite_unlocked(log_symbols, sizeof(gr_complex), nc, d_fp);
  assert(count == nc);
  //printf("count: %d written to known_data.dat, total: %d \n", count, ftell(d_fp)); fflush(stdout);
  free(log_symbols);
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

   
    if(i<d_data_carriers.size()) printf("finish filling out the symbol.. \n");
    while(i < d_data_carriers.size()) {   // finish filling out the symbol
      out[d_data_carriers[i]] = d_data_constellation[0];
      //printf("out: (%.2f, %.2f)\n", out[d_data_carriers[i]].real(), out[d_data_carriers[i]].imag());
      i++;
    }

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

      d_header.packetlen = d_packetlen;		// -1 for '55' appended (ref ofdm_packet_utils)
      d_header.batch_number = fInfo->active_batch;
      d_header.pkt_type = DATA_TYPE;
      d_header.pkt_num = d_pkt_num;			// unique number across all flows
      d_header.link_id = '0';
      d_header.lead_sender = int(fInfo->isLead) + '0';
      d_header.nsenders = fInfo->num_tx+'0';
      d_header.lead_sender = '1';			// src is always the lead sender

      printf("lead_sender: %c, nsenders: %c\n", d_header.lead_sender, d_header.nsenders); fflush(stdout);     
 	
      if(d_proto == PRO) 
	 memcpy(d_header.coeffs, fInfo->coeffs, d_batch_size);
   }
   else {
      if(d_proto == SPP) 
         d_header = d_msg->header();
      else if(d_proto == PRO || d_proto == CF) {
	 memset(&d_header, 0, sizeof(d_header));
	 d_header.flow_id = fInfo->flowId;
	 d_header.src_id = fInfo->src;
	 d_header.dst_id = fInfo->dst;
	 d_header.batch_number = fInfo->active_batch;
	 d_header.pkt_num = d_pkt_num;

	 if(d_proto == PRO) {
	    d_header.prev_hop_id = d_id;
	    d_header.packetlen = d_packetlen;
	    d_header.next_hop_id = fInfo->nextHopId;		// useless field for PRO
	    memcpy(d_header.coeffs, fInfo->coeffs, d_batch_size);
	 }
	 else if(d_proto == CF) {
	    d_header.prev_hop_id = d_id;			// useless for CF, only link matters!
	    d_header.packetlen = fInfo->packetlen;
	    d_header.link_id = fInfo->nextLinkId;
	    d_header.pkt_num = fInfo->seqNo;			// in case of CF, just use the unique seqNo, which will be consistent even for multiple tx
	    d_header.nsenders = fInfo->num_tx + '0';
	    d_header.lead_sender = int(fInfo->isLead) + '0';
            printf("(MAPPER) makeHeader batch: %d, pkt: %d, len: %d, src: %c, dst: %c, flow: %c, next_hop: %c, nextLink: %c, lead: %c\n",           
                    fInfo->active_batch, d_pkt_num, fInfo->packetlen, fInfo->src, fInfo->dst, fInfo->flowId,         
                    fInfo->nextHopId, fInfo->nextLinkId, d_header.lead_sender);
            fflush(stdout);
	 }
      }
   }

   printf("(MAPPER), makeHeader, batch:% d, pkt: %d, len: %d, src: %d, dst: %d, flow: %d, next_hop: %d, nextLink: %c\n", 
	d_header.batch_number, d_header.pkt_num, d_header.packetlen, d_header.src_id, d_header.dst_id, d_header.flow_id, 
	d_header.next_hop_id, d_header.link_id); fflush(stdout);

   d_last_pkt_time = d_out_pkt_time;  

   unsigned char header_data_bytes[HEADERDATALEN];
   memcpy(header_data_bytes, &d_header, HEADERDATALEN);				// copy e'thing but the crc

   unsigned int calc_crc = digital_crc32(header_data_bytes, HEADERDATALEN);
   d_header.hdr_crc = calc_crc;

   memcpy(d_header_bytes, header_data_bytes, HEADERDATALEN);				// copy header data
   memcpy(d_header_bytes+HEADERDATALEN, &calc_crc, sizeof(int));		// copy header crc

   printf("len: %d, crc: %u\n", d_header.packetlen, calc_crc);

   //debugHeader();  
   whiten(d_header_bytes, HEADERBYTELEN);
   //debugHeader();
}

void
digital_ofdm_mapper_bcv::debugHeader()
{
   whiten(d_header_bytes, HEADERBYTELEN);	// dewhitening effect !
   memset(&d_header, 0, HEADERBYTELEN);
   assert(HEADERBYTELEN == (HEADERDATALEN + sizeof(int) + PADDING_SIZE));
   memcpy(&d_header, d_header_bytes, HEADERBYTELEN);

   printf("debug crc: %u\n", d_header.hdr_crc); 
   printf("\n"); 
   whiten(d_header_bytes, HEADERBYTELEN);
}

void
digital_ofdm_mapper_bcv::whiten(unsigned char *bytes, unsigned int len)
{
   for(int i = 0; i < len; i++)
	bytes[i] = bytes[i] ^ random_mask_tuple1[i];
}

/* uses 'msg' to create a 'd_msg' with appropriate length. 'msg' can be freed after 
   this call */
void
digital_ofdm_mapper_bcv::add_crc_and_fec(gr_message_sptr msg) {
   printf("add_crc_and_fec, fec_N: %d, fec_K: %d\n", fec_N, fec_K); fflush(stdout);
   unsigned int calc_crc = 0;
   int len = msg->length();
   printf("len: %d\n", len); fflush(stdout); 

   unsigned char *msg_data = (unsigned char*) malloc(len+sizeof(calc_crc));
   memset(msg_data, 0, len+sizeof(calc_crc));
   memcpy(msg_data, msg->msg(), len);

   printf("before crc: \n");
   for(int i = 0; i < len; i++) {
     printf("%02x", (unsigned char) msg_data[i]);
   }
   printf("\n");


   calc_crc = digital_crc32(msg_data, len); 
   memcpy(msg_data+len, &calc_crc, sizeof(calc_crc));
   len += sizeof(calc_crc);

   printf("after crc: \n"); fflush(stdout);
   for(int i = 0; i < len; i++) {
     printf("%02x", (unsigned char) msg_data[i]);
   }
   printf("\n");

   int pad_len = 0;
   if(fec_N == 0 && fec_K == 0) {
      pad_len = getPadBytes(len); 
      printf("padded_len: %d\n", pad_len); fflush(stdout);
      d_msg = gr_make_message(0, 0, 0, len+pad_len);
      memcpy(d_msg->msg(), msg_data, len);
      memcpy(d_msg->msg()+len, msg_data, pad_len);		// just treat them as random bytes!
   }
   else {
      std::string coded_msg = digital_tx_wrapper(std::string((const char*) msg_data, len), fec_N, fec_K, 0);
      len = coded_msg.length();
      pad_len = getPadBytes(len);
      printf("padded_len: %d\n", pad_len); fflush(stdout);
      d_msg = gr_make_message(0, 0, 0, len+pad_len);
      memcpy(d_msg->msg(), coded_msg.data(), len);
      memcpy(d_msg->msg()+len, coded_msg.data(), pad_len);	// just treat them as random bytes
   }

   len += pad_len;
   printf("len:%d\n", len); fflush(stdout);
   interleave(d_msg->msg(), len);
   whiten(d_msg->msg(), len);

   free(msg_data);
}

/* required for interleaving, since it expects a full integer # of OFDM symbols */
inline int
digital_ofdm_mapper_bcv::getPadBytes(int len) {
   int num_ofdm_symbols = ceil(((float) ((len) * 8))/(d_data_carriers.size() * d_data_nbits));
   int pad_bits = (num_ofdm_symbols*d_data_carriers.size()*d_data_nbits) - (len*8);
   printf("num_ofdm_symbols: %d, pad_bits: %d, len: %d\n", num_ofdm_symbols, pad_bits, len); fflush(stdout);
   assert(pad_bits%8 == 0);		
   return pad_bits/8;
}

inline void
digital_ofdm_mapper_bcv::make_time_tag(uhd::time_spec_t out_time, bool enforce) {

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

  if(!enforce) {
     out_time = c_time + duration_time;         // scheduled out time //

     /* check if the interval from the last packet sent is atleast the duration */
     uhd::time_spec_t interval_time = out_time - d_last_pkt_time;    // interval between the transmissions //
     if(interval_time < duration_time) {
        uhd::time_spec_t extra_gap_time = duration_time - interval_time;
        out_time += extra_gap_time;
        interval_time += extra_gap_time;
     }

     uint64_t interval_samples = (interval_time.get_frac_secs() * 1e8)/decimation;

     printf("timestamp: curr (%llu, %f), out (%llu, %f) , last_time (%llu, %f), interval(%lld, %f), interval_samples(%llu), duration(%llu, %f)\n", (uint64_t) c_time.get_full_secs(), c_time.get_frac_secs(), (uint64_t) out_time.get_full_secs(), out_time.get_frac_secs(), (uint64_t) d_last_pkt_time.get_full_secs(), d_last_pkt_time.get_frac_secs(), (int64_t) interval_time.get_full_secs(), interval_time.get_frac_secs(), interval_samples, sync_secs, sync_frac_of_secs); fflush(stdout);
  }
#if 0
  else {
        // disable (put here only to test synchronization) //
      num_samples = (d_preamble.size()+d_num_hdr_symbols) * (d_fft_length+cp_length);
      duration =  num_samples*time_per_sample; 
      sync_secs = (uint64_t) duration;
      sync_frac_of_secs = duration - (uint64_t) duration;
      out_time += uhd::time_spec_t(sync_secs, sync_frac_of_secs);
  }
#endif

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

inline SockId
digital_ofdm_mapper_bcv::open_client_sock(int port, const char *addr, bool blocking) {
  SockId sockfd;
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

/* all clients/servers must be unique */
inline void
digital_ofdm_mapper_bcv::create_ack_socks() {
  printf("(mapper):: create_ack_socks\n"); fflush(stdout);
  EthInfoMap::iterator e_it;
  EthInfo *ethInfo = NULL;

  FlowInfoVector::iterator f_it = d_flowInfoVector.begin();
  while(f_it != d_flowInfoVector.end()) {
      FlowInfo *fInfo = *f_it;
      if(fInfo->nextHopId != UNASSIGNED) {
	 e_it = d_ethInfoMap.find(fInfo->nextHopId);
	 assert(e_it != d_ethInfoMap.end());
	 ethInfo = (EthInfo*) e_it->second;
	 SockId rx_sock = open_client_sock(ethInfo->port, ethInfo->addr, true);
	 d_sock_map.insert(std::pair<FlowId, unsigned int>(fInfo->flowId, rx_sock));
      }
      f_it++;
  }
  
  printf("(mapper): create_ack_socks num_socks: %d done!\n", d_sock_map.size()); fflush(stdout);
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
     flow_info->active_batch = -1;
     flow_info->last_batch_acked = -1;
     flow_info->nextLinkId = '0';
     flow_info->num_tx = 1;
     flow_info->isLead = 1;	

     d_flowInfoVector.push_back(flow_info);

     if(d_proto == CF) {
	PktSymbolsVector *pv = (PktSymbolsVector*) malloc(sizeof(PktSymbolsVector));
        memset(pv, 0, sizeof(PktSymbolsVector));
	d_flowPktVector.insert(std::pair<FlowId, PktSymbolsVector*> (flowId, pv));
     }	
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
digital_ofdm_mapper_bcv::send_scheduler_msg(int type, FlowId flowId) {
   FlowInfo *fInfo = getFlowInfo(false, flowId);
   if(d_source) fInfo->nextLinkId = '0';
   printf("send_scheduler_msg type: %d, request_id: %d, flowId: %c, batch: %d, link: %c\n", type, d_request_id, flowId, fInfo->active_batch, fInfo->nextLinkId); fflush(stdout);
   SchedulerMsg request;
   request.request_id = d_request_id;
   request.nodeId = d_id;
   request.type = type;
   request.flow = flowId;
   request.batch_num = fInfo->active_batch;

   request.proto = d_proto;
   request.linkId = fInfo->nextLinkId;

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
digital_ofdm_mapper_bcv::check_scheduler_reply(SchedulerMsg& reply) {
   int buf_size = sizeof(SchedulerMsg);
   char *buf = (char*) malloc(buf_size);
   memset(buf, 0, buf_size);

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

	   printf("check_scheduler_reply, type: %d, request_id: %d, nodeId: %c, linkId: %c\n", 
				reply.type, reply.request_id, reply.nodeId, reply.linkId); fflush(stdout);
	   break;
	}
      }
   }
   free(buf);

   if(d_proto == CF) {
      FlowInfo *fInfo = getFlowInfo(false, reply.flow);
      if(reply.nodeId == d_id) 
	 return true;
      else if(reply.linkId == fInfo->nextLinkId) {
	 for(int i = 0; i < reply.num_tx; i++) {
	     if(reply.senders[i] == d_id)
	        return true;
	 }
      }
      return false;
   }
   else 
      return (reply.nodeId == d_id);
}

inline void
digital_ofdm_mapper_bcv::create_scheduler_sock() {
   printf("(mapper): create_scheduler_sock\n"); fflush(stdout);
   d_scheduler_sock = open_client_sock(9000, "128.83.120.84", true);
}

/* for PRO */
inline void
digital_ofdm_mapper_bcv::create_e2e_ack_socks() {
  printf("(mapper):: create_e2e_ack_socks\n"); fflush(stdout);
  EthInfoMap::iterator e_it;
  EthInfo *ethInfo = NULL;

  FlowInfoVector::iterator it = d_flowInfoVector.begin();
  while(it != d_flowInfoVector.end()) {
      FlowInfo *fInfo = *it;
      if(fInfo->src == d_id) {
	 e_it = d_ethInfoMap.find(fInfo->dst);
	 assert(e_it != d_ethInfoMap.end());
	 ethInfo = (EthInfo*) e_it->second;
	 SockId rx_sock = open_client_sock(ethInfo->port, ethInfo->addr, true);
	 d_sock_map.insert(std::pair<FlowId, unsigned int>(fInfo->flowId, rx_sock));
      }
      it++;
  } 
  printf("(mapper): create_e2e_ack_socks num_socks: %d done!\n", d_sock_map.size()); fflush(stdout);
}

/* populates both the weight and the redundancy values for each node */
inline void
digital_ofdm_mapper_bcv::populateCreditInfo() {
   printf("populateCreditInfo\n"); fflush(stdout);
   // weight //
   FILE *fl = fopen ( "credit_W.txt" , "r+" );
   if (fl==NULL) {
        fprintf (stderr, "File error\n");
        assert(false);
   }

   char line[128];
   const char *delim = " ";
	 // flow  nodeId  prevNodeId  weight  //
   while ( fgets ( line, sizeof(line), fl ) != NULL ) {
        char *strtok_res = strtok(line, delim);
        vector<char*> token;
        while (strtok_res != NULL) {
           token.push_back(strtok_res);
           strtok_res = strtok (NULL, delim);
        }

        FlowId flowId = atoi(token[0])+'0';
        char nodeId = atoi(token[1])+'0';
        if(nodeId == d_id) {
	   CreditWInfo wInfo; 
           wInfo.flowId = flowId;
           wInfo.prevHopId = atoi(token[2])+'0';
           wInfo.weight = atof(token[3]);
	   printf("flow: %c, prevHop: %c, weight: %f\n", flowId, wInfo.prevHopId, wInfo.weight); 
	   d_creditWInfoVector.push_back(wInfo);
	   d_flowCreditMap.insert(std::pair<FlowId, float> (flowId, 0.0));
        }
   }
   fclose (fl);   

   // redundancy //
   printf("populateCreditInfo\n"); fflush(stdout);
   fl = fopen ( "credit_R.txt" , "r+" );
   if (fl==NULL) {
        fprintf (stderr, "File error\n");
        assert(false);
   }
	// flow  nodeId  redundancy  //
   while ( fgets ( line, sizeof(line), fl ) != NULL ) {
        char *strtok_res = strtok(line, delim);
        vector<char*> token;
        while (strtok_res != NULL) {
           token.push_back(strtok_res);
           strtok_res = strtok (NULL, delim);
        }

        FlowId flowId = atoi(token[0])+'0';
        char nodeId = atoi(token[1])+'0';
        if(nodeId == d_id) {
	   float r_val = atof(token[2]); 
	   d_creditRInfoMap.insert(std::pair<FlowId, float> (flowId, r_val));
        }
   }
   fclose (fl);
}

inline NetCoder*
digital_ofdm_mapper_bcv::getNetCoder(FlowId flowId, int data_len) {
   FlowNetCoderMap::iterator it = d_NetCoderMap.find(flowId);

   if(data_len == 0)
     assert(it != d_NetCoderMap.end());

   NetCoder *netcoder = NULL;
   if(it == d_NetCoderMap.end()) {
      netcoder = new NetCoder(d_batch_size, data_len);
      d_NetCoderMap.insert(std::pair<FlowId, NetCoder*> (flowId, netcoder));
   }
   else
      netcoder = it->second;

   return netcoder;
}

inline void
digital_ofdm_mapper_bcv::make_packet_PRO_source(FlowInfo *flowInfo)
{
  source_PRO(flowInfo);
  makeHeader(flowInfo);
  initializeBatchParams();
  d_pending_flag = 1;                             // start of packet flag //

  initHeaderParams();

  d_preambles_sent = d_ofdm_symbol_index = d_null_symbol_cnt = d_hdr_ofdm_index = 0;
  d_pkt_num++;
  d_modulated = false;
}

inline void
digital_ofdm_mapper_bcv::source_PRO(FlowInfo *flowInfo) {
  gr_message_sptr msg[MAX_BATCH_SIZE];
  int data_len = 0;

  if(flowInfo->last_batch_acked == flowInfo->active_batch) {
     // if last batch was acked, then dequeue a fresh one //
     flowInfo->active_batch++;
     printf("start active_batch: %d, last_batch_acked: %d\n", flowInfo->active_batch, flowInfo->last_batch_acked); fflush(stdout);
     d_msg.reset();
     for(unsigned int b = 0; b < d_batch_size; b++) {
         msg[b] = d_msgq->delete_head();
         assert(msg[b]->length() > 0);
         data_len = msg[b]->length();
	 AddPacketToNetCoder(msg[b], flowInfo, !b, false);
	 msg[b].reset();
     }
  }
  else {
     // retransmission! //
     assert(flowInfo->last_batch_acked < flowInfo->active_batch);
     printf("retransmit batch: %d, last_batch_acked: %d, pkts_sent: %d\n",
                flowInfo->active_batch, flowInfo->last_batch_acked, flowInfo->pkts_fwded); fflush(stdout);

     NetCoder *encoder = getNetCoder(flowInfo->flowId, 0);
     data_len = encoder->data_len;
  }

  /* get a fresh encoded pkt from NetCoder */
  uint8_t *pkt = (uint8_t*) malloc(d_batch_size+data_len);
  GetEncodedPacket(pkt, flowInfo);

   printf("debug encoded content - \n"); fflush(stdout);
   for(int i = 0; i < d_batch_size+data_len; i++)
       printf("%02X ", pkt[i]);
   printf("\n");


  /* add crc and fec */
  gr_message_sptr e_msg = gr_make_message(0, 0, 0, data_len);
  memcpy(e_msg->msg(), &(pkt[d_batch_size]), data_len);
  add_crc_and_fec(e_msg);

  /* clean up */
  free(pkt); e_msg.reset();

  print_msg(d_msg); 
  d_packetlen = d_msg->length();
  
  d_num_ofdm_symbols = ceil(((float) ((d_packetlen) * 8))/(d_data_carriers.size() * d_data_nbits));
  printf("d_num_ofdm_symbols: %d, d_num_hdr_symbols: %d\n", d_num_ofdm_symbols, d_num_hdr_symbols); fflush(stdout);
}

/* adds the packet to NetCoder. Ensure the netcoder is flushed for a new batch. It also creates
   the packet in the format reqd by the NetCoder
*/
inline int
digital_ofdm_mapper_bcv::AddPacketToNetCoder(gr_message_sptr msg, FlowInfo *fInfo, bool new_batch, bool fwder) {
   printf("AddPacketToNetCoder flow: %c, new_batch: %d, is_fwder: %d\n", 
				fInfo->flowId, new_batch, fwder); fflush(stdout);
   int data_len = msg->length();
   NetCoder *encoder = getNetCoder(fInfo->flowId, data_len);
   if(new_batch) encoder->num_pkts = 0;                         // flush //

   int rank = encoder->num_pkts;
   uint8_t *pkt = (uint8_t*) malloc(d_batch_size+data_len);
   memset(pkt, 0, d_batch_size+data_len);

   if(!fwder)
      pkt[rank] = 1;
   else {
      for(int i = 0; i < d_batch_size; i++) 
	 pkt[i] = fInfo->coeffs[i];
   }
   memcpy(pkt+d_batch_size, msg->msg(), data_len);

   printf("encoded pkt added to NetCoder, before adding rank: %d\n", rank); fflush(stdout);
   for(int k = 0; k < data_len+d_batch_size; k++)
       printf("%02X ", pkt[k]);
   printf("\n");

   encoder->AddPacket(pkt);
   printf("encoded pkt added to NetCoder, after adding rank: %d\n", encoder->num_pkts); fflush(stdout);
   free(pkt);

   return encoder->num_pkts;
}

inline void
digital_ofdm_mapper_bcv::GetEncodedPacket(uint8_t *pkt, FlowInfo *fInfo) {
   NetCoder *encoder = getNetCoder(fInfo->flowId, 0);
   int data_len = encoder->data_len;

   printf("GetEncodedPacket, flow: %c, data_len: %d\n", fInfo->flowId, data_len); fflush(stdout);
   uint8_t *tmp_pkt = (uint8_t*) malloc(d_batch_size+data_len);
   memset(tmp_pkt, 0, d_batch_size+data_len);

   printf("just before encode..\n"); fflush(stdout);
   tmp_pkt = encoder->Encode();
   printf("just after encode.. rank: %d\n", encoder->num_pkts); fflush(stdout);
   for(int i = 0; i < d_batch_size+data_len; i++)
       printf("%02X ", tmp_pkt[i]);
   printf("\n");

   memcpy(pkt, tmp_pkt, d_batch_size+data_len);
   printf("encoded content - \n"); fflush(stdout);
   for(int i = 0; i < d_batch_size+data_len; i++)
       printf("%02X ", pkt[i]);
   printf("\n"); 

   /* copy the coeffs */
   for(int i = 0; i < d_batch_size; i++) {
       fInfo->coeffs[i] = pkt[i] & 0xff;
       printf("coeff[%d]: %d\n", i, fInfo->coeffs[i]); fflush(stdout);
   }

   free(tmp_pkt);
}

/* check the credit, if credit >= 1.0, then send a scheduler request. 
   Get the scheduler reply, then again quickly check if any credit updates arrived
   in the meantime. Ensure some credit>=1.0, and do the transmission. If
   the credit<1.0 in the 2nd phase, then send a scheduler complete message.
*/
inline int
digital_ofdm_mapper_bcv::work_forwarder_PRO(int noutput_items,
                              gr_vector_const_void_star &input_items,
                              gr_vector_void_star &output_items) {
   if(d_modulated) {
      bool request_sent = false;
      FlowId flowId;
      while(1) {
         bool tx_ok = check_credit_PRO(true, flowId);		// blocking!
         if(!tx_ok) continue;

         if(d_tdma) {
	    if(!request_sent) 
	       send_scheduler_msg(REQUEST_INIT_MSG, flowId);
	    request_sent = true;

	    SchedulerMsg reply;
            while(!check_scheduler_reply(reply)) {
	       sleep(0.65);
	    }

	    tx_ok = check_credit_PRO(false, flowId);	 	// non-blocking!
	    if(!tx_ok) { 
	       send_scheduler_msg(REQUEST_COMPLETE_MSG, flowId);
	       request_sent = false;
	       continue;
	    }
         }
         break;
      } // while
      make_time_tag(uhd::time_spec_t(0, 0), false);
      make_packet_PRO_fwd();				// make the packet
   }

   assert(d_flow > 0);					// must be set in make_packet_PRO_fwd()
   FlowInfo *fInfo = getFlowInfo(false, d_flow);
   return modulate_and_send(noutput_items, input_items, output_items, fInfo);   
}

/* creates a packet for the forwarder */
inline void
digital_ofdm_mapper_bcv::make_packet_PRO_fwd() {
   assert(d_modulated);
   assert(d_flow_q.size() >= 1);
   FlowId flowId = d_flow_q.front();
   d_flow_q.erase(d_flow_q.begin());

   FlowInfo *fInfo = getFlowInfo(false, flowId);
   d_flow = flowId;

   NetCoder *encoder = getNetCoder(flowId, 0); 
   int data_len = encoder->data_len;

    printf("make_packet_PRO_fwd .. flowId: %c, data_len: %d\n", flowId, data_len); fflush(stdout);
   uint8_t *pkt = (uint8_t*) malloc(d_batch_size+data_len);
   GetEncodedPacket(pkt, fInfo); 

   /* add crc and fec */
   gr_message_sptr e_msg = gr_make_message(0, 0, 0, data_len);
   memcpy(e_msg->msg(), &(pkt[d_batch_size]), data_len);
   add_crc_and_fec(e_msg);

   /* clean up */
   free(pkt); e_msg.reset();

   print_msg(d_msg);
   d_packetlen = d_msg->length();
   d_num_ofdm_symbols = ceil(((float) ((d_packetlen) * 8))/(d_data_carriers.size() * d_data_nbits));
   printf("d_num_ofdm_symbols: %d, d_num_hdr_symbols: %d\n", d_num_ofdm_symbols, d_num_hdr_symbols); fflush(stdout);

   makeHeader(fInfo);
   initializeBatchParams();
   d_pending_flag = 1;                             // start of packet flag //

   initHeaderParams();

   d_preambles_sent = d_ofdm_symbol_index = d_null_symbol_cnt = d_hdr_ofdm_index = 0;
   d_pkt_num++;
   d_modulated = false;
}

/* updates (increases) the credit for (flowId, prevHopId) for all nextHopId entries 
   works both for PRO and CF 
   for PRO - the prevId is prevHopId
*/
inline void
digital_ofdm_mapper_bcv::updateCredit(FlowInfo *fInfo, unsigned char prevId) {

   assert(d_proto == PRO); 
   // get the weight //
   CreditWInfo wInfo;
   for(int i = 0; i < d_creditWInfoVector.size(); i++) {
       wInfo = d_creditWInfoVector[i];
       if(wInfo.flowId == fInfo->flowId && wInfo.prevHopId == prevId) 
	  break;	  
   }

   // get the redundancy //
   CreditRInfoMap::iterator it = d_creditRInfoMap.find(fInfo->flowId);
   assert(it != d_creditRInfoMap.end());
   float red_val = it->second;

   // update the credit //
   FlowCreditMap::iterator fit = d_flowCreditMap.find(fInfo->flowId);
   assert(fit != d_flowCreditMap.end());
   float credit = fit->second;
   credit += (wInfo.weight*red_val);

   printf("updateCredit: flow: %c, prevId: %c, old_credit: %f, new_credit: %f\n", 
			fInfo->flowId, prevId, fit->second, credit); fflush(stdout);

   // enqueue the pkt placeholders //
   while(credit >= 1.0) {
      updateFwdQ_PRO(fInfo->flowId, true);
      credit -= 1.0;
   }

   d_flowCreditMap.erase(fit);
   d_flowCreditMap.insert(std::pair<FlowId, float> (fInfo->flowId, credit));
}

/* reset the credit for the flowId */
inline void
digital_ofdm_mapper_bcv::resetCredit(FlowInfo *fInfo) {
   if(fInfo->active_batch == -1)
      return;

   printf("resetCredit, flow: %c\n", fInfo->flowId); fflush(stdout);
   FlowCreditMap::iterator fit = d_flowCreditMap.find(fInfo->flowId);
   assert(fit != d_flowCreditMap.end());
   d_flowCreditMap.erase(fit);
   d_flowCreditMap.insert(std::pair<FlowId, float> (fInfo->flowId, 0.0));   

   updateFwdQ_PRO(fInfo->flowId, false);			// remove all entries of this flow from flow_Q //

   NetCoder *encoder = getNetCoder(fInfo->flowId, 0);
   encoder->num_pkts = 0; 
}

/* check the queue if any more packets received from the sink - 
   - if yes, then update the credit 
   - update the fwd_Q if required 
*/
inline bool
digital_ofdm_mapper_bcv::check_credit_PRO(bool blocking, FlowId &flowId) {
   printf("check_credit_PRO, blocking :%d\n", blocking); fflush(stdout);
   gr_message_sptr msg;
   if(blocking) 
      msg = d_msgq->delete_head();
   else 
      msg = d_msgq->delete_head_nowait();

   if(msg != 0) {
      MULTIHOP_HDR_TYPE hdr = msg->header();

      unsigned short batchId = hdr.batch_number;
      flowId = ((int) hdr.flow_id) + '0';
      NodeId prevHopId = ((int)hdr.prev_hop_id) + '0';
      FlowInfo *fInfo = getFlowInfo(false, flowId);

        // copy the coeffs //
      for(int i = 0; i < d_batch_size; i++)
	 fInfo->coeffs[i] = hdr.coeffs[i];
	
      assert(fInfo != NULL);

      if((batchId < fInfo->active_batch) && (fInfo->active_batch != -1)) {
	 // stale batch - complete ignore //
	 printf("Stale batch, batchId: %d, active_batch: %d\n", batchId, fInfo->active_batch); fflush(stdout);
	 msg.reset();
	 return false;
      } else if((batchId > fInfo->active_batch) || (fInfo->active_batch == -1)) {
	 // new batch - reset encoder, credit, flow_Q, batch details, etc //
	 printf("Jumping to batch, batchId: %d, active_batch: %d\n", batchId, fInfo->active_batch);
	 resetCredit(fInfo);
         fInfo->active_batch = batchId;
	 fInfo->last_batch_acked = batchId-1;
      }

      printf("flow: %c, batch: %d, prevHop: %c, active_batch: %d\n", flowId, batchId, prevHopId, fInfo->active_batch);
      fflush(stdout);

	// now update the credit //
      updateCredit(fInfo, prevHopId);

	// add the msg to the netcoder (after innovative check) //      
      NetCoder *encoder = getNetCoder(flowId, msg->length());
      int rank = encoder->num_pkts;
      assert(rank <= d_batch_size);	

      if(rank == d_batch_size) { 
	 printf("non-innovative packet, rank: %d\n", rank); fflush(stdout);
      }
      else 
         AddPacketToNetCoder(msg, fInfo, false, true);

      msg.reset();
   }

   return (d_flow_q.size() > 0);
}

/* this Q contains the flows for which credit >= 1.0 (time order is maintained) 
   1. update - add the flow if its credit now is >= 1.0
   2. delete (!update) - if the flow was ACKed before it could be sent out
*/
inline void
digital_ofdm_mapper_bcv::updateFwdQ_PRO(FlowId flowId, bool update) {
   bool _delete = !update;

   if(_delete) { 
      vector<FlowId>::iterator fit = d_flow_q.begin();
      while(fit != d_flow_q.end()) {
         if(*fit == flowId) {
	    d_flow_q.erase(fit);
	    fit = d_flow_q.begin();
	    continue;
         }
         fit++;	
      }
      return;
   }
   d_flow_q.push_back(flowId);
}

/* ---------------------------------------------------- CF -----------------------------------------
   -------------------------------------------------------------------------------------------------
*/
/* check the credit, if credit >= 1.0, then send a scheduler request. 
   Get the scheduler reply, then again quickly check if any credit updates arrived
   in the meantime. Ensure some credit>=1.0, and do the transmission. If
   the credit<1.0 in the 2nd phase, then send a scheduler complete message.
*/
inline int
digital_ofdm_mapper_bcv::work_forwarder_CF(int noutput_items,
                              gr_vector_const_void_star &input_items,
                              gr_vector_void_star &output_items) {
   if(d_modulated) {
      bool request_sent = false;
      SchedulerMsg reply;
      FlowId flowId;
      bool isLead = false;
      while(1) {
         bool tx_ok = check_credit_CF(true, flowId);           // blocking!
         if(!tx_ok) continue;

         if(d_tdma) {
            if(!request_sent)
               send_scheduler_msg(REQUEST_INIT_MSG, flowId);
            request_sent = true;

            while(!check_scheduler_reply(reply)) {	// also set the d_lead_sender flag
               sleep(0.65);
            }

            tx_ok = check_credit_CF(false, flowId);            // non-blocking!
	    FlowInfo *fInfo = getFlowInfo(false, flowId);
            if(!tx_ok || (reply.batch_num != fInfo->active_batch)) {
	       if(reply.lead_sender == d_id) {
		  printf("Sending ABORT Trigger.. tx_ok: %d, reply.batch: %d, active_batch: %d\n",
			  tx_ok, reply.batch_num, fInfo->active_batch); fflush(stdout);
		  send_scheduler_msg(REQUEST_ABORT_MSG, flowId);   // abort to the scheduler
		  send_trigger_abort_CF(reply);         // abort to the slaves
                   request_sent = false;
                   continue;				// slightly inefficient: only the lead can abort the trigger
	       }
            }
         }
	
	 if(reply.lead_sender == d_id) {
	    isLead = true;
            make_time_tag(d_out_pkt_time, false);           // do not enforce 'd_out_pkt_time'
	    if(reply.num_tx > 1) 
               send_trigger_CF(reply);
         }
	 else {
	    isLead = false;
	    TriggerMsg rx_trigger;
	    get_trigger_CF(rx_trigger, reply);			    // blocking!
	
	    assert(rx_trigger.lead_id == reply.lead_sender);
	    assert(rx_trigger.flow == reply.flow);
	    assert(rx_trigger.batch_num == reply.batch_num);

	    FlowInfo *fInfo = getFlowInfo(false, reply.flow);
	    if(rx_trigger.type == TRIGGER_ABORT || (fInfo->active_batch != reply.batch_num)) {
		printf("RX Trigger ABORT.. reply.batch: %d, active_batch: %d\n",
                          reply.batch_num, fInfo->active_batch); fflush(stdout);
		continue;
	    }
	    d_out_pkt_time = uhd::time_spec_t(rx_trigger.secs, rx_trigger.frac_secs);
	    make_time_tag(d_out_pkt_time, true);
	 }
         break;
      } // while
      prepare_packet_CF_fwd(reply);
      FlowInfo *fInfo = getFlowInfo(false, d_flow);
      if(fInfo->num_tx > 1)
         generateModulatedData_CF();                             // pregenerate the modulated data, do alamouti later //
   }

   /* the payload is already modulated, and only the header needs to be modulated */
   FlowInfo *fInfo = getFlowInfo(false, d_flow);
   return modulate_and_send(noutput_items, input_items, output_items, fInfo);
}

/* just prepare to fwd a CF_packet - the param is passed to ensure only the lead replies 
   to the scheduler with the COMPLETE message */
void
digital_ofdm_mapper_bcv::prepare_packet_CF_fwd(SchedulerMsg reply)
{
   /* get the flow that needs to be serviced */
   assert(d_flow_q_CF.size() >= 1);
   std::pair<FlowId, LinkId> flowLink = d_flow_q_CF.front();
   d_flow = flowLink.first;
   LinkId nextLinkId = flowLink.second;
   
   assert(d_flow >= 0);
   FlowInfo *fInfo = getFlowInfo(false, d_flow);
   fInfo->nextLinkId = nextLinkId;
   fInfo->isLead = (reply.lead_sender == d_id)?true:false;
   fInfo->leadId = reply.lead_sender;
   fInfo->seqNo = reply.seqNo;
   fInfo->num_tx = reply.num_tx;
   d_flow_q_CF.erase(d_flow_q_CF.begin());

   printf("prepare_packet_CF_fwd -- flow: %c, nextLinkId: %c, isLead: %d\n", d_flow, nextLinkId, fInfo->isLead); fflush(stdout);

   /* init other params */
   assert(d_modulated);
   assert(d_batch_size == 1);
   d_num_ofdm_symbols = ceil(((float) ((fInfo->packetlen) * 8))/(d_data_carriers.size() * d_data_nbits));
   printf("d_num_ofdm_symbols: %d, d_num_hdr_symbols: %d\n", d_num_ofdm_symbols, d_num_hdr_symbols); fflush(stdout);

   makeHeader(fInfo);
   initializeBatchParams();
   d_pending_flag = 1;                             // start of packet flag //

   initHeaderParams();

   d_preambles_sent = d_ofdm_symbol_index = d_null_symbol_cnt = d_hdr_ofdm_index = 0;
   d_pkt_num++;
   d_modulated = false;
}

/* in the unlikely event, when 1) send scheduler request, 2) get the reply 
   3) ensure credit >= 1.0, and the credit fails, and I happen to be the lead sender, 
   then 4) send the abort trigger on others who might be waiting on me! */
inline void
digital_ofdm_mapper_bcv::send_trigger_abort_CF(SchedulerMsg reply) {
   TriggerMsg trigger;
   trigger.lead_id = d_id;
   trigger.flow = reply.flow;
   trigger.batch_num = reply.batch_num;
   trigger.type = TRIGGER_ABORT;

   short num_tx = trigger.num_tx;
   assert(num_tx < MAX_SENDERS);

   for(int i = 0; i < num_tx; i++)
      trigger.senders[i] = reply.senders[i];

   int buf_size = sizeof(TriggerMsg);
   char *buf = (char*) malloc(buf_size);
   memcpy(buf, &trigger, buf_size);

   assert(send(d_trigger_sock, buf, buf_size, 0) >= 0);
   free(buf);
}

/* since anyone can be a triggerer, every node in a composite link needs a broadcast 
   socket to communicate with others. The lead-sender does the trigger broadcast, 
   while all others will wait on this broadcast
*/
inline void
digital_ofdm_mapper_bcv::open_trigger_sock_CF() {
   int on = 1;
   struct sockaddr_in sin;

   if((d_trigger_sock = socket(PF_INET, SOCK_DGRAM, 0)) < 0) {
      perror("socket");
      exit(1);
   }

   if(setsockopt(d_trigger_sock, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on)) < 0) {
      perror("setsockopt");
      exit(1);
   }

   sin.sin_family = AF_INET;
   sin.sin_addr.s_addr = INADDR_BROADCAST;
   sin.sin_port = htons(5555);
   if(bind(d_trigger_sock, (struct sockaddr *)&sin, sizeof(sin)) < 0) {
      perror("bind");
      exit(1);
   } 

   printf("errno: %d\n", errno); fflush(stdout);
}

/* trigger broadcast to everyone listening */
inline void
digital_ofdm_mapper_bcv::send_trigger_CF(SchedulerMsg reply) {
   TriggerMsg trigger;
   trigger.lead_id = d_id;
   trigger.seqNo = reply.seqNo;
   trigger.num_tx = reply.num_tx;
   trigger.flow = reply.flow;
   trigger.batch_num = reply.batch_num;
   trigger.secs = d_out_pkt_time.get_full_secs();
   trigger.frac_secs = d_out_pkt_time.get_frac_secs();
   trigger.type = TRIGGER_START;

   short num_tx = trigger.num_tx;
   assert(num_tx < MAX_SENDERS);

   printf("send_trigger_CF, seqNo: %d, lead: %c, num_tx: %d, flow: %c, batch: %d\n", 
			    trigger.seqNo, d_id, num_tx, trigger.flow, trigger.batch_num); fflush(stdout);

   for(int i = 0; i < num_tx; i++) 
      trigger.senders[i] = reply.senders[i];

   struct sockaddr_in sendaddr;
   memset(&sendaddr, 0, sizeof sendaddr);
   sendaddr.sin_family = AF_INET;
   sendaddr.sin_port = htons(5555);
   sendaddr.sin_addr.s_addr = INADDR_BROADCAST;
   
   int buf_size = sizeof(TriggerMsg);
   char *buf = (char*) malloc(buf_size);
   memcpy(buf, &trigger, buf_size);

#if 1
   if(sendto(d_trigger_sock, buf, buf_size, 0, (struct sockaddr *)& sendaddr, sizeof(sendaddr)) < 0) {
      printf("errno: %d\n", errno); fflush(stdout);
      assert(false);
   }
#else
   if(send(d_trigger_sock, buf, buf_size, 0) < 0) {
      printf("errno: %d\n", errno); fflush(stdout);
      assert(false);
   }
#endif

   free(buf);
}

/* block waiting for the trigger from the lead sender */
inline void
digital_ofdm_mapper_bcv::get_trigger_CF(TriggerMsg& trigger, SchedulerMsg reply) {
   printf("get_trigger_CF\n"); fflush(stdout);
   uhd::time_spec_t out_time;
   int buf_size = sizeof(TriggerMsg);
   char *eth_buf = (char*) malloc(buf_size);

   bool done = false;
   while(!done) {  
      int nbytes = recv(d_trigger_sock, eth_buf, buf_size, MSG_PEEK);
      if(nbytes > 0) {
         int offset = 0;
         while(1) {
            nbytes = recv(d_trigger_sock, eth_buf+offset, buf_size-offset, 0);
            if(nbytes > 0) offset += nbytes;
            if(offset == buf_size) {
	       memcpy(&trigger, eth_buf, buf_size);
	       if(trigger.seqNo != reply.seqNo) {
		  break;
	       }
	       done = true;
               break;
            }
         }
      }
   }
   printf("Trigger received for from lead: %c, batch: %d\n", trigger.lead_id, trigger.batch_num); fflush(stdout);
   free(eth_buf);
}

/* check the queue if any more packets received from the sink - 
   - if yes, then update the credit 
   - update the fwd_Q if required 
*/
inline bool
digital_ofdm_mapper_bcv::check_credit_CF(bool blocking, FlowId &flowId) {

   printf("check_credit_CF, blocking :%d\n", blocking); fflush(stdout);
   gr_message_sptr msg;

   while(1) {
      if(blocking)
	 msg = d_msgq->delete_head();
      else
	 msg = d_msgq->delete_head_nowait();

      if(msg != 0) {
         MULTIHOP_HDR_TYPE hdr = msg->header();

	 unsigned short batchId = hdr.batch_number;
	 flowId = ((int) hdr.flow_id) + '0';
	 LinkId prevLinkId = hdr.link_id;
	 FlowInfo *fInfo = getFlowInfo(false, flowId);
	 assert(fInfo != NULL);

	 if((batchId < fInfo->active_batch) && (fInfo->active_batch != -1)) {
            // stale batch - complete ignore //
            printf("Stale batch, batchId: %d, active_batch: %d\n", batchId, fInfo->active_batch); fflush(stdout);
            msg.reset();
            return false;
         } 
         else if((batchId > fInfo->active_batch) || (fInfo->active_batch == -1)) {
            // new batch - reset encoder, credit, flow_Q, batch details, etc //
            printf("Jumping to batch, batchId: %d, active_batch: %d, prevLinkId: %c\n", batchId, fInfo->active_batch, prevLinkId);
	    resetCredit_CF(fInfo);
	    fInfo->active_batch = batchId;
	    fInfo->last_batch_acked = batchId-1;
	    fInfo->prevLinkId = prevLinkId;
	    fInfo->packetlen = hdr.packetlen;
	 }

	 printf("flow: %c, batch: %d, prevLink: %c, active_batch: %d\n", flowId, batchId, prevLinkId, fInfo->active_batch);
	 fflush(stdout);

	 // now update the credit //
	 updateCredit_CF(fInfo);
	 processPacket_CF(msg, fInfo);
	 msg.reset();
      }

      if(d_msgq->count() == 0)
	 break;
   }
   return (d_flow_q_CF.size() > 0);
}

/* every incoming packet from the queue is processed here. The packets for the same batch 
   are combined to do either (i) mrc or (ii) equal gain combining and only the resulting 
   complex packet is transmitted when the credit >= 1.0. This ensures that the latest combined 
   packet is the one that is transmitted */
void
digital_ofdm_mapper_bcv::processPacket_CF(gr_message_sptr msg, FlowInfo *fInfo) {
   printf("processPacket_CF, batch: %d, prevLinkId: %c, num_symbols: %d\n", 
	   fInfo->active_batch, fInfo->prevLinkId, msg->length()/sizeof(gr_complex));

   gr_complex *symbols = (gr_complex*) malloc(msg->length());
   memcpy(symbols, msg->msg(), msg->length());
  
   FlowPktVector::iterator fit = d_flowPktVector.find(fInfo->flowId);
   assert(fit != d_flowPktVector.end());
   PktSymbolsVector *pv = fit->second;
   pv->push_back(symbols);

   /* MRC or Equal-Gain-Combinining will be done when you have to send the packet */
}

int
digital_ofdm_mapper_bcv::work_CF(int noutput_items,
                              gr_vector_const_void_star &input_items,
                              gr_vector_void_star &output_items) {
  FlowInfo *flowInfo = getFlowInfo(false, d_flow);

  /* branch out completely if forwarder */
  if(!d_source) {
     return work_forwarder_CF(noutput_items, input_items, output_items);
  }

  if(d_modulated && flowInfo->pkts_fwded && d_ack) {
     check_for_ack(flowInfo);                                   // non-blocking //
  }

  if(d_modulated) {
     make_packet_SPP(flowInfo);                              // blocking if no msg to send //

     print_msg(d_msg);
     if(d_tdma) {
        send_scheduler_msg(REQUEST_INIT_MSG, d_flow);
	SchedulerMsg reply;
        while(!check_scheduler_reply(reply)) {
           sleep(0.65);
        }
     }
     make_time_tag(uhd::time_spec_t(0, 0), false);

     if(flowInfo->num_tx > 1) 
        generateModulatedData_CF();				// generates all modulated data at once! //
  }

  return modulate_and_send(noutput_items, input_items, output_items, flowInfo);
}

/* does the actual mrc/equal gain combining of all the versions of this packet to 
   get a single packet */
void
digital_ofdm_mapper_bcv::copyOFDMSymbolData_CF(gr_complex *out, int ofdm_symbol_index) {

  int start_offset = d_data_carriers[0];
  if(d_data_carriers[0] > d_pilot_carriers[0])
      start_offset = d_pilot_carriers[0];

  long offset = ofdm_symbol_index*d_occupied_carriers;
  FlowPktVector::iterator fit = d_flowPktVector.find(d_flow);
  assert(fit != d_flowPktVector.end());

  float eq_factor = 1/((float) d_flowPktVector.size());

  printf("copyOfdmSymbolData_CF, eq_factor: %f, ofdm_index: %d\n", eq_factor, d_ofdm_symbol_index); fflush(stdout);
  PktSymbolsVector *pv = fit->second;
  PktSymbolsVector::iterator pit = pv->begin();
  while(pit != pv->end()) {
     gr_complex *symbols = *pit;
     for(int i = 0; i < d_occupied_carriers; i++) {
         out[start_offset+i] += (eq_factor*symbols[offset+i]);
     }
     pit++;
  }

  d_msg_offset += (d_occupied_carriers);

  if(d_msg_offset == d_num_ofdm_symbols*d_occupied_carriers)
     d_send_null = true;
}

inline void
digital_ofdm_mapper_bcv::resetCredit_CF(FlowInfo *fInfo) {
   if(fInfo->active_batch == -1)
      return;

   printf("resetCredit_CF, flow: %c\n", fInfo->flowId); fflush(stdout);
   CreditInfo_CF cInfo;
   cInfo.flowId = fInfo->flowId;
   updateFwdQ_CF(&cInfo, false);                        // remove all entries of this flow from flow_Q //

   /* remove all the packets for this batch from the flow */
   FlowPktVector::iterator fit = d_flowPktVector.find(fInfo->flowId);
   assert(fit != d_flowPktVector.end());
   PktSymbolsVector *pv = fit->second;
   PktSymbolsVector::iterator pit = pv->begin();
   while(pit != pv->end()) {
      gr_complex *symbols = *pit;
      free(symbols);
      pit++;
   }
   pv->clear();
}

/* updates (increases) the credit for (flowId, prevLinkId) for all nextLinkId entries 
   CF only!
*/
inline void
digital_ofdm_mapper_bcv::updateCredit_CF(FlowInfo *fInfo) {
   printf("updatCredit_CF: flow: %c, prevLink: %c\n", fInfo->flowId, fInfo->prevLinkId); fflush(stdout);
   assert(d_proto == CF);
  
   CreditInfoVector_CF::iterator it = d_creditInfoVector_CF.begin();
   while(it != d_creditInfoVector_CF.end()) {
      CreditInfo_CF *cInfo = *it;
      if(cInfo->flowId == fInfo->flowId && cInfo->prevLinkId == fInfo->prevLinkId) {
	 cInfo->curr_credit += cInfo->delta_credit;
	 printf("updateCredit_CF: flow: %c, prevLinkId: %c, new_credit: %f\n",
                        fInfo->flowId, fInfo->prevLinkId, cInfo->curr_credit); fflush(stdout);
	 while(cInfo->curr_credit >= 1.0) {
	    updateFwdQ_CF(cInfo, true);
	    fInfo->nextLinkId = cInfo->nextLinkId;
	    cInfo->curr_credit -= 1.0;
	 }
      }
      it++;
   }
}

/* this Q contains the flows for which credit >= 1.0 (time order is maintained) 
   1. update - add the flow if its credit now is >= 1.0
   2. delete (!update) - if the flow was ACKed before it could be sent out
*/
inline void
digital_ofdm_mapper_bcv::updateFwdQ_CF(CreditInfo_CF *cInfo, bool update) {
   bool _delete = !update;

   /* if delete - then remove all the flowId entries from the queue (irr. of the nextLinkId) */
   if(_delete) {
      vector<std::pair<FlowId, LinkId> >::iterator fit = d_flow_q_CF.begin();
      while(fit != d_flow_q_CF.end()) {
	 std::pair<FlowId, LinkId> pf = *fit;
	 if(pf.first == cInfo->flowId)
            d_flow_q_CF.erase(fit);
            fit = d_flow_q_CF.begin();
            continue;
         fit++;
      }
      return;
   }

   /* update - then insert it */
   d_flow_q_CF.push_back(std::pair<FlowId, LinkId> (cInfo->flowId, cInfo->nextLinkId));
}

/* one single file that contains both the weight and red. values */
inline void
digital_ofdm_mapper_bcv::populateCreditInfo_CF() {
   printf("populateCreditInfo_CF\n"); fflush(stdout);
   // flow node prevLink nextLink weight redundancy //
   FILE *fl = fopen ( "credit_CF.txt" , "r+" );
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

        FlowId flowId = atoi(token[0])+'0';
        NodeId nodeId = atoi(token[1])+'0';
        if(nodeId == d_id) {
	   CreditInfo_CF *cInfo = (CreditInfo_CF*) malloc(sizeof(CreditInfo_CF));
           cInfo->flowId = flowId;
           cInfo->prevLinkId = atoi(token[2])+'0';
	   cInfo->nextLinkId = atoi(token[3])+'0';
	   float wt = atof(token[4]);
	   float red = atof(token[5]);
	   cInfo->delta_credit = wt*red;
	   cInfo->curr_credit = 0.0;

           printf("flow: %c, prevLink: %c, nextLink: %c, wt: %f, red: %f, delta_credit: %f\n", 
		   flowId, cInfo->prevLinkId, cInfo->nextLinkId, wt, red, cInfo->delta_credit);
           d_creditInfoVector_CF.push_back(cInfo);
        }
   }
   fclose (fl);
}

inline void
digital_ofdm_mapper_bcv::interleave(unsigned char *data, int len) {

   /* convert to bits */
   std::string orig_s;
   for(int i = 0; i < len; i++) {
       std::bitset<sizeof(unsigned char) * 8> s_bits(data[i]);
       std::string tmp_string = s_bits.to_string<char,std::char_traits<char>,std::allocator<char> >();
       orig_s.append(tmp_string);
   }
   int len_bits = orig_s.length();
   printf("len_bits: %d, len: %d\n", len_bits, len); fflush(stdout);
   assert(len_bits == len*8);

   unsigned char *out = (unsigned char*) malloc(len_bits);
   for(unsigned i = 0; i < len_bits; i += d_interleave_map.size()) {
       for(unsigned j = 0; j < d_interleave_map.size(); ++j) {
           out[i+d_interleave_map[j]] = (orig_s.data())[i+j];
       }
   }

   /*convert back to data */
   string tmp_s((const char*) out, len_bits);
   for(int i = 0, k = 0; i < len_bits; i=i+8)
   {
      bitset<sizeof(unsigned char)*8> s_bits(tmp_s.substr(i, 8));
      data[k++] = (unsigned char) s_bits.to_ulong();
   }
   free(out);
}

/* symbols have been pre-modulated, only ensure 'out' is filled using alamouti codes

 	         T1    T2
   Sender1:: 	 s1   -s2*
   ----------------------
   Sender2::	 s2    s1*
*/
inline void
digital_ofdm_mapper_bcv::generateOFDMSymbolData_alamouti(gr_complex* out, FlowInfo *fInfo) {
  gr_complex *symbols = d_pktInfo.symbols;
  int offset = d_ofdm_symbol_index*d_fft_length;

  // if the lead sender //
  if(fInfo->isLead) {
     for(int i=0; i<d_data_carriers.size(); i++) {
        if(d_ofdm_symbol_index%2==0)
            out[d_data_carriers[i]] = symbols[offset+d_data_carriers[i]];
         else
	    out[d_data_carriers[i]] = -conj(symbols[offset+d_data_carriers[i]]);
     }
  }
  else {
     // if not the lead //
     for(int i=0; i<d_data_carriers.size(); i++) {
         if(d_ofdm_symbol_index%2==0)
            out[d_data_carriers[i]] = symbols[offset+d_fft_length+d_data_carriers[i]];
         else
	    out[d_data_carriers[i]] = conj(symbols[offset-d_fft_length+d_data_carriers[i]]);
     }
  }
}

/* used when alamouti codes are used, pre-generates all the ofdm symbols at once, 
   so in modulate_and_send, the alamouti codes can be used on these symbols */
inline void
digital_ofdm_mapper_bcv::generateModulatedData_CF() {
  bool extra_symbol = (d_num_ofdm_symbols%2==0)?false:true;		// add 1 OFDM symbol, if odd to work with alamouti (2x1)
  if(extra_symbol) 
     d_num_ofdm_symbols++;

  if(d_pktInfo.symbols == NULL) {
     d_pktInfo.symbols = (gr_complex*) malloc(sizeof(gr_complex)*d_num_ofdm_symbols*d_fft_length);
  }

  memset(d_pktInfo.symbols, 0, sizeof(gr_complex)*d_num_ofdm_symbols*d_fft_length);
  for(int o=0; o<d_num_ofdm_symbols; o++) {
      if(d_source)
         generateOFDMSymbolData(&(d_pktInfo.symbols[o*d_fft_length]));
      else {
	 if(extra_symbol && o==d_num_ofdm_symbols-1) {
	    // just add the dumb symbol 
	    memcpy(&(d_pktInfo.symbols[o*d_fft_length]), &(d_pktInfo.symbols[(o-1)*d_fft_length]), sizeof(gr_complex)*d_fft_length);
	 } else {
 	    copyOFDMSymbolData_CF(&(d_pktInfo.symbols[o*d_fft_length]), o);
	 }
      }
      logDataSymbols(&(d_pktInfo.symbols[o*d_fft_length]));
  }
  d_send_null = false;
  printf("generateModulatedData_CF.. extra: %d, d_num_ofdm_symbols: %d\n", extra_symbol, d_num_ofdm_symbols); fflush(stdout);
}
