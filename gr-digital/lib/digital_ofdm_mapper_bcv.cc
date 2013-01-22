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
                         unsigned int occupied_carriers, unsigned int fft_length, unsigned int id,
			 unsigned int source,
			 unsigned int batch_size,
			 unsigned int dst_id)
{
  return gnuradio::get_initial_sptr(new digital_ofdm_mapper_bcv (hdr_constellation, data_constellation, preamble, msgq_limit, occupied_carriers, fft_length, id, source, batch_size, dst_id));
}

// Consumes 1 packet and produces as many OFDM symbols of fft_length to hold the full packet
digital_ofdm_mapper_bcv::digital_ofdm_mapper_bcv (const std::vector<gr_complex> &hdr_constellation, 
					const std::vector<gr_complex> &data_constellation,
					const std::vector<std::vector<gr_complex> > &preamble,
					unsigned int msgq_limit, 
					unsigned int occupied_carriers, unsigned int fft_length, unsigned int id,
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
    d_batch_q_num(-1),
    d_batch_size(batch_size),
    d_batch_to_send(0),
    d_packets_sent_for_batch(0),	// for testing only //
    d_source(source),
    d_modulated(true),	// start afresh //
    d_send_null(false),
    d_pkt_num(0),
    d_id(id + '0'),
    d_dst_id(dst_id),
    d_preambles_sent(0),
    d_preamble(preamble)
{
  if (!(d_occupied_carriers <= d_fft_length))
    throw std::invalid_argument("digital_ofdm_mapper_bcv: occupied carriers must be <= fft_length");

  // sanity check preamble symbols
  for (size_t i = 0; i < d_preamble.size(); i++){
    if (d_preamble[i].size() != (size_t) d_fft_length)
      throw std::invalid_argument("digital_ofdm_mapper_bcv: invalid length for preamble symbol");
  }

#ifdef ACK_ON_ETHERNET
  d_sock_fd = openACKSocket();
#endif

  std::string arg("");
  d_usrp = uhd::usrp::multi_usrp::make(arg);
  d_out_pkt_time = uhd::time_spec_t(0.0);
  d_last_pkt_time = uhd::time_spec_t(0.0);

  // this is not the final form of this solution since we still use the occupied_tones concept,
  // which would get us into trouble if the number of carriers we seek is greater than the occupied carriers.
  // Eventually, we will get rid of the occupied_carriers concept.
  std::string carriers = "F00F";		// apurv++,  8 DC subcarriers
 
  // A bit hacky to fill out carriers to occupied_carriers length
  int diff = (d_occupied_carriers - 4*carriers.length()); 
  while(diff > 7) {
    carriers.insert(0, "f");
    carriers.insert(carriers.length(), "f");
    diff -= 8;
  }

  // if there's extras left to be processed
  // divide remaining to put on either side of current map
  // all of this is done to stick with the concept of a carrier map string that
  // can be later passed by the user, even though it'd be cleaner to just do this
  // on the carrier map itself
  int diff_left=0;
  int diff_right=0;

  // dictionary to convert from integers to ascii hex representation
  char abc[16] = {'0', '1', '2', '3', '4', '5', '6', '7', 
		  '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
  if(diff > 0) {
    char c[2] = {0,0};

    diff_left = (int)ceil((float)diff/2.0f);   // number of carriers to put on the left side
    c[0] = abc[(1 << diff_left) - 1];          // convert to bits and move to ASCI integer
    carriers.insert(0, c);
    
    diff_right = diff - diff_left;	       // number of carriers to put on the right side
    c[0] = abc[0xF^((1 << diff_right) - 1)];   // convert to bits and move to ASCI integer
        carriers.insert(carriers.length(), c);
  }
  
  // find out how many zeros to pad on the sides; the difference between the fft length and the subcarrier
  // mapping size in chunks of four. This is the number to pack on the left and this number plus any 
  // residual nulls (if odd) will be packed on the right. 
  diff = (d_fft_length/4 - carriers.length())/2; 

  /* pilot configuration */ 
  int num_pilots = 8; //12; //4;
  unsigned int pilot_index = 0;			     // tracks the # of pilots carriers added   
  unsigned int data_index = 0;			     // tracks the # of data carriers added
  unsigned int count = 0;			     // tracks the total # of carriers added
  unsigned int pilot_gap = 11; //7; //18;
  unsigned int start_offset = 0; //8;

  unsigned int i,j,k;
  //for(i = 0; i < (d_occupied_carriers/4)+diff_left; i++) {
  for(i = 0; i < carriers.length(); i++) {
    char c = carriers[i];                            // get the current hex character from the string
    for(j = 0; j < 4; j++) {                         // walk through all four bits
      k = (strtol(&c, NULL, 16) >> (3-j)) & 0x1;     // convert to int and extract next bit
      if(k) {                                        // if bit is a 1, 
	int carrier_index = 4*(i+diff) + j - diff_left;

	// check if it should be pilot, else add as data //
	if(count == (start_offset + (pilot_index * pilot_gap))) {		// check notes below !
	   d_pilot_carriers.push_back(carrier_index);
	   pilot_index++;
	}
	else {
   	   d_data_carriers.push_back(carrier_index);  // use this subcarrier
	   data_index++;
	}
	count++;
      }
    }
  }

  assert(pilot_index + data_index == count);

  /* debug carriers */ 
  printf("pilot carriers (%d): \n", d_pilot_carriers.size()); 
  for(int i = 0; i < d_pilot_carriers.size(); i++) {
     printf("%d ", d_pilot_carriers[i]); fflush(stdout);
  }
  printf("\n");
  assert(d_pilot_carriers.size() == pilot_index);
  assert(d_data_carriers.size() == data_index); 
  
  // hack the above to populate the d_pilots_carriers map and remove the corresponding entries from d_data_carriers //
  /* if # of data carriers = 72
	# of pilots        = 6
	gap b/w pilots     = 72/6 = 12
 	start pilot	   = 5
	other pilots	   = 5, 17, 29, .. so on
	P.S: DC tones should not be pilots!
  */  
  printf("data carriers (%d): \n", d_data_carriers.size());
  for(int i = 0; i < d_data_carriers.size(); i++) {
     printf("%d ", d_data_carriers[i]); fflush(stdout);
  }
  printf("\n");

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

int
digital_ofdm_mapper_bcv::work(int noutput_items,
	  		      gr_vector_const_void_star &input_items,
  			      gr_vector_void_star &output_items)
{
  /* ack over ethernet */
  struct sockaddr_in client_addr;
  int addrlen = sizeof(client_addr);
  
#ifdef ACK_ON_ETHERNET 
  while (!d_sock_opened) {
    d_client_fd = accept(d_sock_fd, (struct sockaddr*)&client_addr, (socklen_t*)&addrlen);
    if (d_client_fd != -1) {
      printf("@ attached rcvr with ACK socket!\n"); fflush(stdout);
      d_sock_opened = true;

      /* make the socket non-blocking */
      //int flags = fcntl(d_client_fd, F_GETFL, 0);
      //fcntl(d_client_fd, F_SETFL, flags|O_NONBLOCK);

      struct timeval tv;
      tv.tv_sec = 1;
      tv.tv_usec = 5e5; 
      assert(setsockopt(d_client_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval))==0);
      break;
    }
  }

  /* once a packet has been *completely* modulated, check for ACK on the backend ethernet socket */
  if(d_modulated) {
    memset(d_ack_buf, 0, sizeof(d_ack_buf));
    int nbytes = recv(d_client_fd, d_ack_buf, sizeof(d_ack_buf), 0);
    printf("n_bytes: %d\n", nbytes); fflush(stdout);
#if 1
    if(nbytes >= 0) {
#if 0
       if(nbytes == ACK_HEADERBYTELEN) {
    	  nbytes = recv(d_client_fd, d_ack_buf, sizeof(d_ack_buf), 0);
	  fflush(stdout);
       }
       else {
 	  nbytes = recv(d_client_fd, d_ack_buf, sizeof(d_ack_buf), MSG_WAITALL); 
       }
#endif
       printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ received nbytes: %d as ACK @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ \n", nbytes);

       MULTIHOP_ACK_HDR_TYPE ack_header;
       memcpy(&ack_header, d_ack_buf, sizeof(d_ack_buf));
       printf("ACK details, batch: %d, flow: %d\n", ack_header.batch_number, ack_header.flow_id); fflush(stdout);


       printf("batch: %d ACKed, packets sent for batch: %d, \n", ack_header.batch_number, d_packets_sent_for_batch); fflush(stdout);
       if(d_batch_to_send <= ack_header.batch_number) {
	  d_batch_to_send = ack_header.batch_number + 1;
	  d_packets_sent_for_batch = 0;
       }
    }
#endif
  }
#endif	// ACK_ON_ETHERNET

  //int packetlen = 0;		//only used for make_header//

#ifndef ACK_ON_ETHERNET
  if(d_packets_sent_for_batch == (d_batch_size+1))
  {
	d_packets_sent_for_batch = 0;
	d_batch_to_send += 1;
  }
#endif


  /* if trigged batch_to_send is different from current batch_q_num, 
     then new batch needs to be sent out. 
     0. flush the current batchQ, reset the msgs
     1. dequeue the fresh K pkts from the msgQ
     2. reset the flags
  */
  if(d_batch_q_num != d_batch_to_send)
  {
	printf("start batch: %d, d_batch_q_num: %d\n", d_batch_to_send, d_batch_q_num); fflush(stdout);
	flushBatchQ();
	for(unsigned int b = 0; b < d_batch_size; b++)
	{
	    d_msg.reset();
	    d_msg = d_msgq->delete_head();		// dequeue the pkts from the queue //
	    assert(d_msg->length() > 0);	
	    d_packetlen = d_msg->length();		// assume: all pkts in batch are of same length //
	    d_num_ofdm_symbols = ceil(((float) ((d_packetlen) * 8))/(d_data_carriers.size() * d_data_nbits)); // include 'x55'
	    printf("d_num_ofdm_symbols: %d, d_num_hdr_symbols: %d\n", d_num_ofdm_symbols, d_num_hdr_symbols); fflush(stdout);
	}

	initializeBatchParams();
	d_pending_flag = 1;				// start of packet flag //
	d_batch_q_num = d_batch_to_send;
  }
  else if(d_modulated)
  {
	printf("continue batch: %d, pkts sent: %d\n", d_batch_to_send, d_packets_sent_for_batch); fflush(stdout);
	// no change in the batch number, but this pkt has been sent out completely (retx) //
	initializeBatchParams();
	d_pending_flag = 1;				// start of packet flag //
  }

  d_modulated = false;

  // if start of a packet: then add a header and whiten the entire packet //
  if(d_pending_flag == 1)
  {
     printf("d_packets_sent_for_batch: %d, d_batch_to_send: %d, d_batch_q_num: %d\n", d_packets_sent_for_batch, d_batch_to_send, d_batch_q_num); fflush(stdout);

     fflush(stdout);
     if(((HEADERBYTELEN*8) % d_data_carriers.size()) != 0) {	// assuming bpsk //
	  printf("HEADERBYTELEN: %d, size: %d\n", HEADERBYTELEN, d_data_carriers.size()); fflush(stdout);
	  assert(false);
     }

     makeHeader();
     initHeaderParams();

     d_preambles_sent = d_ofdm_symbol_index = d_null_symbol_cnt = d_hdr_ofdm_index = 0; 
     make_time_tag_source();
     d_pkt_num++;
  }

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
      d_packets_sent_for_batch += 1;
      assert(d_ofdm_symbol_index == d_num_ofdm_symbols);
      d_send_null = false;
      d_modulated = true;
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
  return 1;  // produced symbol
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

/***********************************************************************
 apurv++ starts here, implementing MORE kind of an extension

#### ANALOG domain #####
send(batch_num, K)
  var batch_q[K]        #queue containing modulated symbols of K pkts
  var batch_q_num       #batch num of the batch_queue

  if(batch_q_num != batch_num):
         flush(batch_q)         #releases the batch queue contents
         dequeue(msg_q, K)      #dequeues K new packets from digital queue
         foreach msg dequeued:
                signal = modulate(msg)  #digital to analog modulation
                insert(signal, batch_q) #insert analog signal (of symbols)
                batch_q_num = batch_num #update batch_q number

  encoded_signal = empty        #encoding starts
  foreach signal in batch_q:
         choose random coefficient C
         foreach symbol in signal:
                symbol' = symbol * C

         encoded_signal = encoded_signal + signal
*************************************************************************/

inline void
digital_ofdm_mapper_bcv::initializeBatchParams()
{
  d_msg_offset = d_bit_offset = d_nresid = d_resid = 0;
}

void
digital_ofdm_mapper_bcv::flushBatchQ()
{

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
digital_ofdm_mapper_bcv::makeHeader()
{
   memset(&d_header, 0, sizeof(d_header));
   d_header.dst_id = d_dst_id;
   d_header.flow_id = 0;

   d_header.src_id = d_id;
   d_header.prev_hop_id = d_id;

   d_header.packetlen = d_packetlen - 1;		// -1 for '55' appended (ref ofdm_packet_utils)
   d_header.batch_number = d_batch_to_send;
   d_header.pkt_type = DATA_TYPE;

   d_header.pkt_num = d_pkt_num;
   d_header.link_id = 0;
 
   printf("makeHeader for batch: %d, pkt: %d, len: %d\n", d_batch_to_send, d_pkt_num, d_packetlen); fflush(stdout);
   d_last_pkt_time = d_out_pkt_time;

   for(int i = 0; i < PADDING_SIZE; i++)
	d_header.pad[i] = 0; 
 
   unsigned char header_data_bytes[HEADERDATALEN];
   memcpy(header_data_bytes, &d_header, HEADERDATALEN);				// copy e'thing but the crc

   unsigned int calc_crc = digital_crc32(header_data_bytes, HEADERDATALEN);
   d_header.hdr_crc = calc_crc;

   memcpy(d_header_bytes, header_data_bytes, HEADERDATALEN);				// copy header data
   memcpy(d_header_bytes+HEADERDATALEN, &calc_crc, sizeof(int));		// copy header crc

   memcpy(d_header_bytes+HEADERDATALEN+sizeof(int), d_header.pad, PADDING_SIZE);
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

/* ack processing (at the flow-source) */
void
digital_ofdm_mapper_bcv::processACK(gr_message_sptr ackMsg) {
  printf("mapper: processACK\n"); 

  assert(ackMsg->type() == ACK_TYPE);
  MULTIHOP_ACK_HDR_TYPE ack_header;
  memcpy(&ack_header, ackMsg->msg(), ACK_HEADERBYTELEN);
  
  assert(ack_header.dst_id == d_id);
  if(d_batch_to_send == ack_header.batch_number) {
     d_batch_to_send += 1;
     d_packets_sent_for_batch = 0;
  }

  ackMsg.reset();
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

  /* allocate d_ack_buf */
  d_ack_buf = (char*) malloc(sizeof(char) * sizeof(ACK_HEADERDATALEN));
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
digital_ofdm_mapper_bcv::make_time_tag_source() {

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
