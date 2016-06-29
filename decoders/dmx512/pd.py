##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2016 Fabian J. Stumpf <sigrok@fabianstumpf.de>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, write to the Free Software
## Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
##

import sigrokdecode as srd

class Decoder(srd.Decoder):
    api_version = 2
    id = 'dmx512'
    name = 'DMX512'
    longname = 'Digital MultipleX 512'
    desc = 'Professional lighting control.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = ['dmx512']
    channels = (
        {'id': 'dmx', 'name': 'DMX data', 'desc': 'Any DMX data line'},
    )
    annotations = (
        ('bit', 'Bit'),
        ('break', 'Break'),
        ('mab', 'Mark after break'),
        ('startbit', 'Start bit'),
        ('stopbits', 'Stop bit'),
        ('startcode', 'Start code'),
        ('channel', 'Channel'),
        ('interframe', 'Interframe'),
        ('interpacket', 'Interpacket'),
        ('data', 'Data'),
        ('error', 'Error')
    )
    annotation_rows = (
        ('name', 'Logical', (1, 2, 5, 6, 7, 8)),
        ('data', 'Data', (9, )),
        ('bits', 'Bits', (0, 3, 4)),
        ('errors', 'Errors', (10, ))
    )

    def __init__(self, **kwargs):
        self.samplerate = None
        self.sample_usec = None
        self.samplenum = -1
        self.run_start = -1
        self.run_bit = 0
        self.state = 'FIND_BREAK'

    def start(self):
        self.out_bin = self.register(srd.OUTPUT_BINARY)
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value
            self.sample_usec = 1 / value * 1000000
            self.skip_per_bit = int(4 / self.sample_usec)

    def decode(self, ss, es, data):
        if self.samplerate is None:
            raise Exception("Cannot decode without samplerate.")
        for (self.samplenum, pins) in data:
            # seek for an interval with no state change with a length between 88 and 180 us (BREAK)
            if self.state == 'FIND_BREAK':
                if self.run_bit != pins[0]:
                    runlen = (self.samplenum - self.run_start) * self.sample_usec
                    if runlen > 88 and runlen < 180:
                        self.put(self.run_start, self.samplenum, self.out_ann, [1, ['Break']])
                        self.bit_break = self.run_bit
                        self.state = 'MARK_MAB'
                        self.channel = 0
                    self.run_bit = pins[0]
                    self.run_start = self.samplenum
            # directly following the BREAK is the MARK AFTER BREAK
            elif self.state == 'MARK_MAB':
                if self.run_bit != pins[0]:
                        self.put(self.run_start, self.samplenum, self.out_ann, [2, ['MAB']])
                        self.state = 'READ_BYTE'
                        self.channel = 0
                        self.bit = 0
                        self.aggreg = pins[0]
                        self.run_start = self.samplenum
            # mark and read a single transmitted byte (start bit, 8 data bits, 2 stop bits)
            elif self.state == 'READ_BYTE':
                # This is kinda ugly but hey
                self.next_sample = self.run_start + (self.bit + 1) * self.skip_per_bit
                self.aggreg += pins[0]
                if self.samplenum == self.next_sample:
                    bit_value = 0 if round(self.aggreg/self.skip_per_bit) == self.bit_break else 1
                    self.aggreg = pins[0]
                    if self.bit == 0:
                        self.byte = 0
                        self.put(self.run_start, self.samplenum, self.out_ann, [3, ['Start bit']])
                        if bit_value != 0:
                            # invalid start bit, seek new break
                            self.put(self.samplenum, self.samplenum, self.out_ann, [10, ['Invalid start bit']])
                            self.run_bit = pins[0]
                            self.state = 'FIND_BREAK'
                    elif self.bit >= 9:
                        self.put(self.samplenum - self.skip_per_bit, self.samplenum, self.out_ann, [4, ['Stop bit']])
                        if bit_value != 1:
                            # invalid stop bit, seek new break
                            self.put(self.samplenum, self.samplenum, self.out_ann, [10, ['Invalid stop bit']])
                            self.run_bit = pins[0]
                            self.state = 'FIND_BREAK'
                    else:
                        # label and process one bit
                        self.put(self.samplenum - self.skip_per_bit, self.samplenum, self.out_ann, [0, [str(bit_value)]])
                        self.byte |= bit_value << (self.bit - 1)

                    # label a complete byte
                    if self.bit == 10:
                        if self.channel == 0:
                            self.put(self.run_start, self.next_sample, self.out_ann, [5, ['Start code']])
                        else:
                            self.put(self.run_start, self.next_sample, self.out_ann, [6, ['Channel ' + str(self.channel)]])
                        self.put(self.run_start + self.skip_per_bit, self.next_sample - 2 * self.skip_per_bit, self.out_ann, [9, [str(self.byte) + " / " + str(hex(self.byte))]])
                        # continue by scanning the IFT
                        self.channel += 1
                        self.run_start = self.samplenum
                        self.run_bit = pins[0]
                        self.state = 'MARK_IFT'


                    self.bit += 1
            # mark the INTERFRAME-TIME between bytes / INTERPACKET-TIME between packets
            elif self.state == 'MARK_IFT':
                if self.run_bit != pins[0]:
                    if self.channel > 512:
                        self.put(self.run_start, self.samplenum, self.out_ann, [8, ['Interpacket']])
                        self.state = 'FIND_BREAK'
                        self.run_bit = pins[0]
                        self.run_start = self.samplenum
                    else:
                        self.put(self.run_start, self.samplenum, self.out_ann, [7, ['Interframe']])
                        self.state = 'READ_BYTE'
                        self.bit = 0
                        self.run_start = self.samplenum
               
