/* -*- c++ -*- */
/*
 * Copyright 2005,2010 Free Software Foundation, Inc.
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

#include <gr_stream_to_streams.h>
#include <gr_io_signature.h>
#include <string.h>

gr_stream_to_streams_sptr
gr_make_stream_to_streams (size_t item_size, size_t nstreams)
{
  return gnuradio::get_initial_sptr(new gr_stream_to_streams (item_size, nstreams));
}

gr_stream_to_streams::gr_stream_to_streams (size_t item_size, size_t nstreams)
  : gr_sync_decimator ("stream_to_streams",
		       gr_make_io_signature (1, 1, item_size),
		       gr_make_io_signature (nstreams,
					     nstreams, item_size),
		       nstreams)
{
}

int
gr_stream_to_streams::work (int noutput_items,
			    gr_vector_const_void_star &input_items,
			    gr_vector_void_star &output_items)
{
  size_t item_size = output_signature()->sizeof_stream_item (0);

  const char *in = (const char *) input_items[0];
  char **outv = (char **) &output_items[0];
  int nstreams = output_items.size();

  for (int i = 0; i < noutput_items; i++){
    for (int j = 0; j < nstreams; j++){
      memcpy(outv[j], in, item_size);
      outv[j] += item_size;
      in += item_size;
    }
  }

  return noutput_items;
}
