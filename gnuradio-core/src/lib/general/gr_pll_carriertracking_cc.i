/* -*- c++ -*- */
/*
 * Copyright 2005,2011 Free Software Foundation, Inc.
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

GR_SWIG_BLOCK_MAGIC(gr,pll_carriertracking_cc);

gr_pll_carriertracking_cc_sptr
gr_make_pll_carriertracking_cc (float loop_bw, 
				float max_freq,
				float min_freq);

class gr_pll_carriertracking_cc : public gr_sync_block, public gri_control_loop
{
 private:
  gr_pll_carriertracking_cc (float loop_bw, float max_freq, float min_freq);
 public:
  bool lock_detector(void);
  bool  squelch_enable(bool);
  float set_lock_threshold(float);

};
