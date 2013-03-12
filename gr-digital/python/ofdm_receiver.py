#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2010 Szymon Jakubczak
#

import math
from gnuradio import gr
from numpy import fft

import digital_swig
from ofdm_sync_pn import ofdm_sync_pn

# /////////////////////////////////////////////////////////////////////////////
#                   OFDM receiver
# /////////////////////////////////////////////////////////////////////////////

class ofdm_receiver(gr.hier_block2):
  """
  Performs receiver synchronization on OFDM symbols.

  The receiver performs channel filtering as well as symbol, frequency, and phase synchronization.
  """

  def __init__(self, fft_length, cp_length, occupied_tones, snr, ks, threshold, options, logging=False):	# apurv++: added num_symbols, use_chan_filt
    """
    Hierarchical block for receiving OFDM symbols.

    The input is the complex modulated signal at baseband.
    Synchronized packets are sent back to the demodulator.

    @param params: Raw OFDM parameters
    @type  params: ofdm_params
    @param logging: turn file logging on or off
    @type  logging: bool
    """

    gr.hier_block2.__init__(self, "ofdm_receiver",
      gr.io_signature(1, 1, gr.sizeof_gr_complex), # Input signature
      gr.io_signature3(3, 3, gr.sizeof_gr_complex*occupied_tones, gr.sizeof_char, gr.sizeof_gr_complex*occupied_tones)) # Output signature

    # low-pass filter the input channel
    bw = (float(occupied_tones) / float(fft_length)) / 2.0
    tb = bw*0.08
    lpf_coeffs = gr.firdes.low_pass (1.0,                     # gain
                                     1.0,                     # sampling rate
                                     bw+tb,                   # midpoint of trans. band
                                     tb,                      # width of trans. band
                                     gr.firdes.WIN_HAMMING)   # filter type
    self.chan_filt = gr.fft_filter_ccc(1, lpf_coeffs)
    self.connect(self, self.chan_filt)
    self.connect(self.chan_filt, gr.file_sink(gr.sizeof_gr_complex, "rx-filt.dat"))

    zeros_on_left = int(math.ceil((fft_length - occupied_tones)/2.0))
    ks0 = fft_length*[0,]
    ks0[zeros_on_left : zeros_on_left + occupied_tones] = ks[0]
    ks0 = fft.ifftshift(ks0)
    ks0time = fft.ifft(ks0)
    ks0time = ks0time.tolist()

    #sync = ofdm_sync_pn(fft_length, cp_length, True, logging)		# raw
    sync = ofdm_sync_pn(fft_length, cp_length, True, ks0time, threshold, logging)	# crosscorr version

    use_chan_filt=1
    if use_chan_filt == 0:
       self.connect(gr.file_source(gr.sizeof_gr_complex, "chan-filt.dat"), sync)
    else:
       self.connect(self.chan_filt, sync)

    # correct for fine frequency offset computed in sync (up to +-pi/fft_length)
    nco_sensitivity = 2.0/fft_length

    nco = gr.frequency_modulator_fc(nco_sensitivity)
    sigmix = gr.multiply_cc()


    # sample at symbol boundaries
    # NOTE: (sync,2) indicates the first sample of the symbol!
    sampler = digital_swig.ofdm_sampler(fft_length, fft_length+cp_length, len(ks)+1, timeout=100)		# apurv--

    # frequency offset correction #
    self.connect((sync,0), (sigmix,0))
    self.connect((sync,1), nco, (sigmix,1))
    self.connect(sigmix, (sampler,0))
    self.connect((sync,2), (sampler,1))

    """
    self.connect((sync,0), (sampler,0))
    self.connect((sync,2), (sampler,1))  # timing signal to sample at
    self.connect((sync,1), gr.file_sink(gr.sizeof_float, "offset.dat"))
    """

    self.connect((sampler, 1), gr.file_sink(gr.sizeof_char, "sampler_timing.dat"))
    #self.connect((sampler, 2), gr.file_sink(gr.sizeof_char*fft_length, "sampler_timing_fft.dat"))

    # fft on the symbols
    win = [1 for i in range(fft_length)]
    # see gr_fft_vcc_fftw that it works differently if win = []
    fft1 = gr.fft_vcc(fft_length, True, win, True)
    self.connect((sampler,0), fft1)

    # use the preamble to correct the coarse frequency offset and initial equalizer
    ###frame_acq = raw.ofdm_frame_acquisition(fft_length, cp_length, preambles_raw, carriers)
    frame_acq = digital_swig.ofdm_frame_acquisition(occupied_tones, fft_length,
                                                    cp_length, ks)

    self.frame_acq = frame_acq

    self.connect(fft1, (frame_acq,0))
    self.connect((sampler,1), (frame_acq,1))

    #self.connect(fft, gr.null_sink(gr.sizeof_gr_complex*options.fft_length))
    #self.connect((sampler, 1), gr.null_sink(gr.sizeof_char))
    #self.connect(gr.file_source(gr.sizeof_gr_complex*options.fft_length, "symbols_src.dat"), (frame_acq, 0))
    #self.connect(gr.file_source(gr.sizeof_char, "timing.dat"), (frame_acq, 1))

    self.connect((frame_acq,0), (self,0))  # finished with fine/coarse freq correction
    self.connect((frame_acq,1), (self,1))  # frame and symbol timing
    self.connect((frame_acq,2), (self,2))  # hestimates

    self.connect((frame_acq,0), gr.file_sink(gr.sizeof_gr_complex*occupied_tones, "rx-acq.dat"))
    self.connect((frame_acq,1), gr.file_sink(gr.sizeof_char, "timing-acq.dat"))

    if logging:
      self.connect(self.chan_filt,
                   gr.file_sink(gr.sizeof_gr_complex, "rx-filt.dat"))
      self.connect(fft1,
                   gr.file_sink(gr.sizeof_gr_complex*fft_length, "rx-fft.dat"))
      self.connect((frame_acq,0),
                   gr.file_sink(gr.sizeof_gr_complex*occupied_tones, "rx-acq.dat"))
      self.connect((frame_acq,1),
                   gr.file_sink(1, "rx-detect.datb"))
      self.connect(sampler,
                   gr.file_sink(gr.sizeof_gr_complex*fft_length, "rx-sampler.dat"))
      self.connect(sigmix,
                   gr.file_sink(gr.sizeof_gr_complex, "rx-sigmix.dat"))
      self.connect(nco,
                   gr.file_sink(gr.sizeof_gr_complex, "rx-nco.dat"))

