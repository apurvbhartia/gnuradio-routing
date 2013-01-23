/* -*- c++ -*- */
/*
 * Copyright 2005 Free Software Foundation, Inc.
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
#ifndef INCLUDED_GR_MESSAGE_H
#define INCLUDED_GR_MESSAGE_H

#include <map>
#include <list>
#include <gr_types.h>
#include <string>
#include <uhd/usrp/multi_usrp.hpp>

#define MAX_FFT_LENGTH 96
#define MAX_DATA_CARRIERS 72

/* apurv++ define header type */
#define MAX_BATCH_SIZE     2 
#define PADDING_SIZE       6
#define UNASSIGNED 100

#define ACK_PADDING_SIZE 6
#define SRC_PILOT 0 				// enable if only the source pilots are relayed throughout //

#define DATA_TYPE       1 //0
#define ACK_TYPE        2 //1
#define NACK_TYPE    3

#define MAX_RX 3
//#define H_PRECODING 1

typedef struct coeff_str {
  unsigned short phase;				// scaled
  unsigned short amplitude;			// scaled	
} COEFF; 

#pragma pack(1)
typedef struct multihop_hdr_type {

  // 1
  unsigned char src_id : 3;
  unsigned char dst_id  : 3;
  unsigned char flow_id : 2;

  // 2 bytes 
  unsigned short batch_number: 13;
  unsigned char prev_hop_id : 3;

  // 2 bytes
  unsigned short packetlen: 15;
  unsigned char pkt_type : 1;

  // 2 bytes
  unsigned short pkt_num;

  // 1
  unsigned char link_id;
   
  // 4
  unsigned int hdr_crc;

  // 6
  unsigned char pad[PADDING_SIZE];                 // to ensure size % (occupied_carriers-dc_carriers) = 0

} MULTIHOP_HDR_TYPE;

#pragma pack(1)
typedef struct ack_multihop_hdr_type {

  // 3 bytes
  unsigned char pkt_type : 1;
  unsigned char flow_id : 4;
  unsigned short batch_number : 11;
  unsigned char src_id : 4;
  unsigned char dst_id  : 4;

  // 6 bytes 
  unsigned char pad[ACK_PADDING_SIZE];                 // to ensure size % (occupied_carriers-dc_carriers) = 0

} MULTIHOP_ACK_HDR_TYPE;

typedef unsigned char NodeId;
typedef std::vector<unsigned char> NodeIds;

typedef struct eth_info {
  char addr[16];
  int port;
} EthInfo;

typedef std::map<NodeId, EthInfo*> EthInfoMap;                // to store ethernet add

typedef struct pkt_str {
  unsigned int n_senders;
  gr_complex* symbols;
} PktInfo;

typedef struct flow_info_str {
  NodeId src, dst, prevNodeId, nextNodeId;
  unsigned char flowId;
  unsigned int active_batch;
  unsigned int last_batch_acked;
  unsigned int pkts_fwded;
} FlowInfo;

typedef std::vector<FlowInfo*> FlowInfoVector;

class gr_message;
typedef boost::shared_ptr<gr_message> gr_message_sptr;

/*!
 * \brief public constructor for gr_message
 */
gr_message_sptr 
gr_make_message(long type = 0, double arg1 = 0, double arg2 = 0, size_t length = 0);

gr_message_sptr
gr_make_message_from_string(const std::string s, long type = 0, double arg1 = 0, double arg2 = 0);

/*!
 * \brief Message class.
 *
 * \ingroup misc
 * The ideas and method names for adjustable message length were
 * lifted from the click modular router "Packet" class.
 */
class gr_message {
  gr_message_sptr d_next;	// link field for msg queue
  long		  d_type;	// type of the message
  double	  d_arg1;	// optional arg1
  double 	  d_arg2;	// optional arg2
  char		  d_arg3;
  
  bool 			d_timestamp_valid;		// whether the timestamp is valid
  uint64_t 	d_preamble_sec;				// the preamble sync time in seconds
  double 		d_preamble_frac_sec;	// the preamble sync time's fractional seconds
  float			d_timing_offset;

  unsigned char	 *d_buf_start;	// start of allocated buffer
  unsigned char  *d_msg_start;	// where the msg starts
  unsigned char  *d_msg_end;	// one beyond end of msg
  unsigned char  *d_buf_end;	// one beyond end of allocated buffer
  
  gr_message (long type, double arg1, double arg2, size_t length);

  friend gr_message_sptr
    gr_make_message (long type, double arg1, double arg2, size_t length);

  friend gr_message_sptr
    gr_make_message_from_string (const std::string s, long type, double arg1, double arg2);

  friend class gr_msg_queue;

  unsigned char *buf_data() const  { return d_buf_start; }
  size_t buf_len() const 	   { return d_buf_end - d_buf_start; }

public:
  ~gr_message ();
  void set_timestamp(uint64_t ps, double pfs);

  long type() const   { return d_type; }
  double arg1() const { return d_arg1; }
  double arg2() const { return d_arg2; }
  char 	 arg3() const { return d_arg3; }

  void set_type(long type)   { d_type = type; }
  void set_arg1(double arg1) { d_arg1 = arg1; }
  void set_arg2(double arg2) { d_arg2 = arg2; }
  void set_arg3(double arg3) { d_arg3 = arg3; }

  bool timestamp_valid() const { return d_timestamp_valid; }
  long preamble_sec() const { return (long)d_preamble_sec; }
  double preamble_frac_sec() const { return d_preamble_frac_sec; }

  void set_timing_offset(float timing_offset) { d_timing_offset = timing_offset; }
  float timing_offset() const { return d_timing_offset; }

  unsigned char *msg() const { return d_msg_start; }
  size_t length() const      { return d_msg_end - d_msg_start; }
  std::string to_string() const;

};

long gr_message_ncurrently_allocated ();

#endif /* INCLUDED_GR_MESSAGE_H */
