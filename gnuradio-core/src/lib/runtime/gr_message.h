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
#define MAX_OCCUPIED_CARRIERS 80
#define MAX_DATA_CARRIERS 64
#define MAX_OFDM_SYMBOLS 70

/* apurv++ define header type */
#define MAX_BATCH_SIZE     2 
#define PADDING_SIZE       0
#define UNASSIGNED 100
#define ACK_PADDING_SIZE 6

/* packet types */
#define DATA_TYPE       1 //0
#define ACK_TYPE        2 //1
#define NACK_TYPE    3

/* scheduler msgs */
#define REQUEST_INIT_MSG 1
#define REPLY_MSG 2
#define REQUEST_COMPLETE_MSG 3
#define REQUEST_ABORT_MSG 4

/* trigger msgs */
#define TRIGGER_START 1
#define TRIGGER_ABORT 2

/* protocols */
#define SPP 0
#define PRO 1
#define CF 2

#define MAX_SENDERS 4

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
  unsigned char next_hop_id: 3;
  unsigned short packetlen: 12;
  unsigned char pkt_type : 1;

  // 2 bytes
  unsigned short pkt_num;

  // 1
  unsigned char link_id;

  // 1
  unsigned char nsenders;

  // 1
  unsigned char lead_sender;

  // 2 bytes
  unsigned char coeffs[MAX_BATCH_SIZE];		   // only used for PRO; empty (0) when not used
   
  // 4
  unsigned int hdr_crc;

  // 2
  unsigned char pad[PADDING_SIZE];                 // to ensure size % (occupied_carriers-dc_carriers) = 0

} MULTIHOP_HDR_TYPE;

#pragma pack(1)
typedef struct ack_multihop_hdr_type {
  
  unsigned char src_id, dst_id;
  unsigned short batch_number;

  // 3 bytes
  unsigned char pkt_type : 4;
  unsigned char flow_id : 4;

  // 6 bytes 
  unsigned char pad[ACK_PADDING_SIZE];                 // to ensure size % (occupied_carriers-dc_carriers) = 0

} MULTIHOP_ACK_HDR_TYPE;

typedef unsigned char LinkId;				// composite link (CF) 
typedef unsigned char NodeId;
typedef unsigned char FlowId;
typedef int SockId;

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
  NodeId src, dst, prevHopId, nextHopId;
  FlowId flowId;
  LinkId prevLinkId, nextLinkId;		// used only by CF //
  int active_batch;
  int last_batch_acked;				// can be -1 //
  unsigned int pkts_fwded;			// for the active_batch //
  int packetlen;

  unsigned int total_pkts_rcvd;
  unsigned int num_pkts_correct;
 
  /* for pro */
  unsigned char coeffs[MAX_BATCH_SIZE];		// only holds the coeffs for the current packet (working copy) - others are held in NetCoder pkt_lst //

  /* for cf */
  NodeId leadId;
  bool isLead;  
} FlowInfo;

typedef std::vector<FlowInfo*> FlowInfoVector;

/* medium availability */
#pragma pack(1)
typedef struct scheduler_msg {
  int request_id;
  int seqNo;
  NodeId nodeId;
  int type;

  /* these fields used only for CF */
  short num_tx;
  NodeId lead_sender;
  FlowId flow;
  int batch_num;			// even if batch_size = 1, batch_num is not pkt_num, since multiple retx might happen for same batch
  short proto;
  LinkId linkId;
  NodeId senders[MAX_SENDERS];
} SchedulerMsg;

typedef struct trigger_msg {
  NodeId lead_id;
  int seqNo;
  short num_tx;
  FlowId flow;
  unsigned int batch_num;
  unsigned char senders[MAX_SENDERS];
  uint64_t secs;
  double frac_secs;
  short type;					// START, ABORT
} TriggerMsg;


/* credits */
typedef struct credit_info {
  FlowId flowId;
  NodeId prevHopId;
  LinkId prevLinkId;
  float weight;
} CreditWInfo;

typedef std::vector<CreditWInfo> CreditWInfoVector;          // redundancy
typedef std::map<FlowId, float> CreditRInfoMap;              // weight
typedef std::map<FlowId, float> FlowCreditMap;               // current credit

/* for CF */
typedef struct credit {
  FlowId flowId;
  LinkId prevLinkId, nextLinkId;
  float delta_credit;
  float curr_credit;
} CreditInfo_CF;
typedef std::vector<CreditInfo_CF*> CreditInfoVector_CF;

/* for CF */
typedef struct composite_link_str {
  LinkId linkId;
  NodeIds srcIds;
  NodeIds dstIds;
} CompositeLink;
typedef std::vector<CompositeLink*> CompositeLinkVector;

/* for CF 
  used to store the signals of the same batch which can be combined 
  using equal gain or mrc */
typedef std::vector<gr_complex*> PktSymbolsVector;			// vector of packets //
typedef std::map<FlowId, PktSymbolsVector*> FlowPktVector;		// "above" .. for a flow //

/* interleaving */
struct BitInterleave {
  unsigned d_bpsc; // coded bits per subcarrier
  unsigned d_cbps; // coded bits per symbol
  static const int d_num_chunks = 16;

  BitInterleave(int ncarriers, int nbits)
    : d_bpsc(nbits), d_cbps(nbits * ncarriers) {}

  unsigned index(unsigned k) {
    // see 17.3.5.6 in 802.11a-1999, floor is implicit
    assert (k < d_cbps);
    unsigned s = std::max(d_bpsc / 2, (unsigned)1);
    unsigned i = (d_cbps / d_num_chunks) * (k % d_num_chunks) + (k / d_num_chunks);
    unsigned j = s * (i / s) + (i + d_cbps - (d_num_chunks * i / d_cbps)) % s;
    assert (j < d_cbps);
    return j;
  }

  void fill(std::vector<unsigned> &v, bool inverse) {
    v.resize(d_cbps);
    if (!inverse) {
      for (unsigned i = 0; i < d_cbps; ++i) {
        v[i] = index(i);
      }
    } else {

      for (unsigned i = 0; i < d_cbps; ++i) {
        v[index(i)] = i;
      }
    }
  }
};


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

  unsigned char	 *d_buf_start;	// start of allocated buffer
  unsigned char  *d_msg_start;	// where the msg starts
  unsigned char  *d_msg_end;	// one beyond end of msg
  unsigned char  *d_buf_end;	// one beyond end of allocated buffer

  MULTIHOP_HDR_TYPE d_header;
  
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

  void set_header(MULTIHOP_HDR_TYPE header) { d_header = header; }
  MULTIHOP_HDR_TYPE header() const { return d_header; }

  unsigned char *msg() const { return d_msg_start; }
  size_t length() const      { return d_msg_end - d_msg_start; }
  std::string to_string() const;

};

long gr_message_ncurrently_allocated ();

#endif /* INCLUDED_GR_MESSAGE_H */
