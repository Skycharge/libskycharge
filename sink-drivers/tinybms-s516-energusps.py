#!/usr/bin/env python3
#
# Copyright 2022 Skycharge GmbH, Roman Penyaev
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met: 1. Redistributions of source code must retain the above
# copyright notice, this list of conditions and the following
# disclaimer.
#
# 2. Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.
#

#
# This partially implements the TinyBMS UART protocol.
# Please see the spec here:
# https://www.energusps.com/web/binary/saveas?filename_field=datas_fname&field=datas&model=ir.attachment&id=21208
#

"""Skycharge TinyBMS s516 driver

Usage:
  tinybms-s516-energusps.py get-state [--device <uart-dev>] [--json] [--cells] [--events] [--additional-cmd <cmds>] [--verbose]
  tinybms-s516-energusps.py get-settings [--device <uart-dev>] [--json] [--additional-cmd <cmds>] [--verbose]
  tinybms-s516-energusps.py set-fully-charged-voltage-mv <mv> [--device <uart-dev>] [--json] [--additional-cmd <cmds>] [--verbose]
  tinybms-s516-energusps.py set-fully-discharged-voltage-mv <mv> [--device <uart-dev>] [--json] [--additional-cmd <cmds>] [--verbose]
  tinybms-s516-energusps.py set-early-balancing-voltage-threshold-mv <mv> [--device <uart-dev>] [--json] [--additional-cmd <cmds>] [--verbose]
  tinybms-s516-energusps.py set-charge-finished-current-ma <ma> [--device <uart-dev>] [--json] [--additional-cmd <cmds>] [--verbose]
  tinybms-s516-energusps.py set-capacity-ah <ah> [--device <uart-dev>] [--json] [--additional-cmd <cmds>] [--verbose]
  tinybms-s516-energusps.py set-nr-cells <nr> [--device <uart-dev>] [--json] [--additional-cmd <cmds>] [--verbose]
  tinybms-s516-energusps.py set-allowed-disbalance-voltage-mv <mv> [--device <uart-dev>] [--json] [--additional-cmd <cmds>] [--verbose]
  tinybms-s516-energusps.py set-overvoltage-cutoff-mv <mv> [--device <uart-dev>] [--json] [--additional-cmd <cmds>] [--verbose]
  tinybms-s516-energusps.py set-undervoltage-cutoff-mv <mv> [--device <uart-dev>] [--json] [--additional-cmd <cmds>] [--verbose]
  tinybms-s516-energusps.py set-discharge-overcurrent-cutoff-a <a> [--device <uart-dev>] [--json] [--additional-cmd <cmds>] [--verbose]
  tinybms-s516-energusps.py set-charge-overcurrent-cutoff-a <a> [--device <uart-dev>] [--json] [--additional-cmd <cmds>] [--verbose]
  tinybms-s516-energusps.py set-overheat-cutoff-c <c> [--device <uart-dev>] [--json] [--additional-cmd <cmds>] [--verbose]
  tinybms-s516-energusps.py set-soc <soc> [--device <uart-dev>] [--json] [--additional-cmd <cmds>] [--verbose]

Options:
  -h --help                Show this screen
  --device=<uart-dev>      Path to the UART device. By default Skycharge Sink device is accessed
  --cells                  Get cells state
  --events                 Get BMS events
  --json                   Output data in JSON
  --verbose                Show bytes send and received
  --additional-cmd <cmds>  Additional command line for the 'skycharge-cli', for example '--conffile' can be specified
"""

import sys
import time
import serial
import struct
from docopt import docopt
import json
import subprocess
from enum import Enum

args = {}

# MODBUS CRC tables
CRC_HI = [
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
]

CRC_LOW = [
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
    0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
    0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
    0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
]

RX_TIMEOUT = 5

def crc16(data):
    """
    brief Calculate MODBUS crc
    :param data: Data
    :return: List CRC high Byte, CRC low Byte
    """
    assert type(data) == list

    crc_high = 0xFF
    crc_low = 0xFF

    for i in range(0, len(data)):
        index = crc_high ^ data[i]
        crc_high = crc_low ^ CRC_HI[index]
        crc_low = CRC_LOW[index]

    return [crc_high, crc_low]

def run(command, indata=None):
    process = subprocess.Popen(command,
                               stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE)
    if indata and isinstance(indata, list):
        indata = bytearray(indata)
    (out, err) = process.communicate(input=indata)
    process.wait()

    return out

class passthru_port:
    def write(self, data):
        additional = args['--additional-cmd']
        additional = additional.split(' ') if additional is not None else []
        command = [ "skycharge-cli", "sink-passthru-send"] + additional
        run(command, data)

    def read(self):
        additional = args['--additional-cmd']
        additional = additional.split(' ') if additional is not None else []
        command = [ "skycharge-cli", "sink-passthru-recv"] + additional
        return run(command)

    def flush(self):
        self.read()

class bms_status(Enum):
    UNKNOWN       = 0x00
    CHARGING      = 0x91
    FULLY_CHARGED = 0x92
    DISCHARGING   = 0x93
    REGENERATION  = 0x96
    IDLE          = 0x97
    FAULT         = 0x9b

class bms_event(Enum):
    NO_EVENT                             = 0x00
    UNKNOWN                              = 0xff

    #
    # Fault events
    #
    UNDER_VOLTAGE_CUTOFF                 = 0x02
    OVER_VOLTAGE_CUTOFF                  = 0x03
    OVER_TEMPERATURE_CUTOFF              = 0x04
    DISCHARGING_OVER_CURRENT_CUTOFF      = 0x05
    CHARGING_OVER_CURRENT_CUTOFF         = 0x06
    REGENERATION_OVER_CURRENT_CUTOFF     = 0x07
    LOW_TEMP_CUTOFF                      = 0x0a
    CHARGER_SWITCH_ERROR                 = 0x0b
    LOAD_SWITCH_ERROR                    = 0x0c
    SINGLE_PORT_SWITCH_ERROR             = 0x0d
    EXTERNAL_CURRENT_SENSOR_DISCONNECTED_FAULT = 0x0e  # BMS restart required
    EXTERNAL_CURRENT_SENSOR_CONNECTED_FAULT    = 0x0f  # BMS restart required

    #
    # Warning events
    #
    FULLY_DISCHARGED_CUTOFF              = 0x31
    LOW_TEMP_CHARGING_CUTOFF             = 0x37
    VOLTAGE_TOO_HIGH_CHARGING_DONE       = 0x38
    VOLTAGE_TOO_LOW_CHARGING_DONE        = 0x39

    #
    # Info events
    #
    SYSTEM_STARTED                       = 0x61
    CHARGING_STARTED                     = 0x62
    CHARGING_DONE                        = 0x63
    CHARGER_CONNECTED                    = 0x64
    CHARGER_DISCONNECTED                 = 0x65
    DUAL_PORT_OP_MODE_ACTIVATED          = 0x66
    SINGLE_PORT_OP_MODE_ACTIVATED        = 0x67
    RECOVERED_FROM_OVER_TEMP_FAULT       = 0x73
    RECOVERED_FROM_LOW_TEMP_WARN         = 0x74
    RECOVERED_FROM_LOW_TEMP_FAULT        = 0x75
    RECOVERED_FROM_CHG_OVER_CURRENT_FAULT        = 0x76
    RECOVERED_FROM_DISCHG_OVER_CURRENT_FAULT     = 0x77
    RECOVERED_FROM_REGEN_OVER_CURRENT_FAULT      = 0x78
    RECOVERED_FROM_OVER_VOLTAGE_FAULT            = 0x79
    RECOVERED_FROM_FULLY_DISCHARGED_VOLTAGE_WARN = 0x7a
    RECOVERED_FROM_UNDER_VOLTAGE_FAULT           = 0x7b
    EXTERNAL_CURRENT_SENSOR_CONNECTED            = 0x7c
    EXTERNAL_CURRENT_SENSOR_DISCONNECTED         = 0x7d

class bms:
    buf = []

    def __init__(self, port):
        self.port = port

    def read_byte(self, timeout):
        start = time.time()

        while not self.buf and time.time() - start < timeout:
            self.buf = list(self.port.read())

        if not self.buf:
            return None

        return self.buf.pop(0)

    def read_exact(self, size):
        data = []
        for i in range(0, size):
            b = self.read_byte(RX_TIMEOUT)
            if b is None:
                return None
            data += [b]

        return data

    def tuple_with_N_nones(self, fmt):
        nr_entries = len(struct.unpack(fmt, b'\0' * struct.calcsize(fmt)))
        return (None) if nr_entries == 1 else (None,) * nr_entries

    def exec_cmd(self, fmt, cmd, cmd_resp=None, size_from_payload_byte=False):
        # Flush possible cached input before starting transmission
        self.buf = []
        self.port.flush()

        if cmd_resp is None:
            cmd_resp = cmd[0]

        crc_fmt = "BB"
        fmt = '<' + fmt
        cmd = [0xaa] + cmd
        cmd += crc16(cmd)
        if args['--verbose']:
            print("SEND",  [hex(b) for b in cmd])
        self.port.write(cmd)
        hdr = []
        while True:
            data = self.read_exact(1)
            if not data:
                return self.tuple_with_N_nones(fmt)
            if data[0] != cmd[0]:
                continue
            break
        hdr += data

        data = self.read_exact(1)
        if not data:
            return self.tuple_with_N_nones(fmt)
        if data[0] != cmd_resp:
            # Presumably error, which has 6bytes packet, read the rest
            self.read_exact(4)
            return self.tuple_with_N_nones(fmt)
        hdr += data

        pl_byte = None
        if not size_from_payload_byte:
            size = struct.calcsize(fmt)
        else:
            pl_byte = self.read_exact(1)
            if not pl_byte:
                return self.tuple_with_N_nones(fmt)
            size = pl_byte[0]

        size = size + struct.calcsize(crc_fmt)
        data = self.read_exact(size)
        if not data:
            return self.tuple_with_N_nones(fmt)
        if size_from_payload_byte:
            # Prepend payload byte to the list
            data = pl_byte + data

        if args['--verbose']:
            print("RECV", [hex(b) for b in hdr + data])

        bdata = bytearray(data)
        unpacked = struct.unpack_from(fmt, bdata)

        crc = crc16(hdr + list(bdata[:-2]))
        crc_lsb, crc_msb = data[-2:]
        if crc_lsb != crc[0] or crc_msb != crc[1]:
            return self.tuple_with_N_nones(fmt)

        # All the rest unparsed except 2 crc bytes
        rest_bytes = data[struct.calcsize(fmt):-2]

        to_ret = list(unpacked) + rest_bytes

        return to_ret[0] if len(to_ret) == 1 else to_ret

    def flatten(self, t):
        return [item for sublist in t for item in sublist]

    def read_registers(self, addrs):
        assert(addrs and len(addrs) >= 1)
        fmt = 'B' + 'HH' * len(addrs)
        cmd = [0x09, len(addrs) * 2]
        addrs_bytes = list([addr.to_bytes(2, 'little') for addr in addrs])
        cmd += self.flatten(addrs_bytes)
        pl, *pairs = self.exec_cmd(fmt, cmd)
        return pairs

    def write_registers(self, addr_val_pairs):
        assert(addr_val_pairs and len(addr_val_pairs) >= 1)
        fmt = 'B'
        cmd = [0x0d, len(addr_val_pairs) * 2]
        addr_val_bytes = list([addr.to_bytes(2, 'little') for addr in addr_val_pairs])
        cmd += self.flatten(addr_val_bytes)
        byte = self.exec_cmd(fmt, cmd, cmd_resp=0x01)
        return byte and byte == cmd[0]

    def get_fully_charged_voltage_mv(self):
        pair = self.read_registers([300])
        return pair[1]

    def set_fully_charged_voltage_mv(self, mv):
        return self.write_registers([300, mv])

    def get_fully_discharged_voltage_mv(self):
        pair = self.read_registers([301])
        return pair[1]

    def set_fully_discharged_voltage_mv(self, mv):
        return self.write_registers([301, mv])

    def get_early_balancing_voltage_threshold_mv(self):
        pair = self.read_registers([303])
        return pair[1]

    def set_early_balancing_voltage_threshold_mv(self, mv):
        return self.write_registers([303, mv])

    def get_charge_finished_current_ma(self):
        pair = self.read_registers([304])
        return pair[1]

    def set_charge_finished_current_ma(self, ma):
        return self.write_registers([304, ma])

    def get_battery_capacity_ah(self):
        pair = self.read_registers([306])
        return pair[1]

    def set_battery_capacity_ah(self, ah):
        return self.write_registers([306, ah])

    def get_nr_cells(self):
        pair = self.read_registers([307])
        return pair[1]

    def set_nr_cells(self, nr):
        return self.write_registers([307, nr])

    def get_allowed_disbalance_voltage_mv(self):
        pair = self.read_registers([308])
        return pair[1]

    def set_allowed_disbalance_voltage_mv(self, mv):
        return self.write_registers([308, mv])

    def get_overvoltage_cutoff_mv(self):
        pair = self.read_registers([315])
        return pair[1]

    def set_overvoltage_cutoff_mv(self, mv):
        return self.write_registers([315, mv])

    def get_undervoltage_cutoff_mv(self):
        pair = self.read_registers([316])
        return pair[1]

    def set_undervoltage_cutoff_mv(self, mv):
        return self.write_registers([316, mv])

    def get_discharge_overcurrent_cutoff_a(self):
        pair = self.read_registers([317])
        return pair[1]

    def set_discharge_overcurrent_cutoff_a(self, a):
        return self.write_registers([317, a])

    def get_charge_overcurrent_cutoff_a(self):
        pair = self.read_registers([318])
        return pair[1]

    def set_charge_overcurrent_cutoff_a(self, a):
        return self.write_registers([318, a])

    def get_overheat_cutoff_c(self):
        pair = self.read_registers([319])
        return pair[1]

    def set_overheat_cutoff_c(self, c):
        return self.write_registers([319, c])

    def get_soc(self):
        pair = self.read_registers([328])
        return pair[1]

    def set_soc(self, soc):
        return self.write_registers([328, soc])

    def battery_status(self):
        pair = self.read_registers([50])
        try:
            status = bms_status(pair[1])
        except:
            status = bms_status.UNKNOWN
        return status

    def battery_soc(self):
        fmt = 'I'
        cmd = [0x1a]
        soc = self.exec_cmd(fmt, cmd)
        return soc / 1000000 if soc is not None else 0

    def battery_voltage(self):
        fmt = 'f'
        cmd = [0x14]
        v = self.exec_cmd(fmt, cmd)
        return v if v is not None else 0.0

    def battery_current(self):
        fmt = 'f'
        cmd = [0x15]
        i =  self.exec_cmd(fmt, cmd)
        return i if i is not None else 0.0

    def temperature(self):
        fmt = 'BHHH'
        cmd = [0x1b]
        pl, *temp = self.exec_cmd(fmt, cmd)
        return [t / 10 if t != 0x8000 else 0 for t in temp] if pl is not None else [0, 0, 0]

    def clear_events(self):
        fmt = 'B'
        cmd = [0x02, 0x01]
        res = self.exec_cmd(fmt, cmd, cmd_resp=0x01)
        return res is not None and res == cmd[0]

    def newest_events(self):
        fmt = 'BBBBB'
        cmd = [0x11]
        b = self.exec_cmd(fmt, cmd, size_from_payload_byte=True)
        if not b:
            return b
        pl = b.pop(0)
        events  = []
        for i in range(0, pl, 4):
            ts = b[i] | (b[i+1] << 8) | (b[i+2] << 16)
            id = b[i+3]

            try:
                ev = bms_event(id)
            except:
                ev = bms_event.UNKNOWN

            if ev == bms_event.UNKNOWN or ev == bms_event.NO_EVENT:
                continue

            events.append({'event': ev, 'ts': ts})
        events.reverse()
        return events

    def cells_voltages(self):
        nr_cells = self.get_nr_cells()
        if nr_cells is None:
            return []
        fmt = 'B' + 'H' * nr_cells
        cmd = [0x1c]
        pl, *voltages = self.exec_cmd(fmt, cmd)
        return [v / 10000 for v in voltages] if pl is not None else []

    def cells_balancing_need(self):
        balancing = self.read_registers([51])
        if balancing[1] is None:
            return 0
        return balancing[1]

    def cells_balancing_real(self):
        balancing = self.read_registers([52])
        if balancing[1] is None:
            return 0
        return balancing[1]

def get_state(bms):
    cells_v = []
    cells_b_need = 0
    cells_b_real = 0
    events = []
    temp = []

    status = bms.battery_status()
    temp = bms.temperature()
    soc = bms.battery_soc()
    v = bms.battery_voltage()
    i = bms.battery_current()

    if args['--events']:
        events = bms.newest_events()

    if args['--cells']:
        cells_v = bms.cells_voltages()
        cells_b_need = bms.cells_balancing_need()
        cells_b_real = bms.cells_balancing_real()

    if args['--json']:
        dic = {
            'status': status.name,
            'soc': soc,
            'battery-current': i,
            'battery-voltage': v,
            'temp': temp,

        }
        if args['--cells']:
            dic['cells-voltages'] = cells_v
            dic['cells-balancing-need'] = hex(cells_b_need)
            dic['cells-balancing-real'] = hex(cells_b_real)
        if args['--events']:
            events = [{'event': e['event'].name, 'ts': e['ts']} for e in events]
            dic['bms-events'] = events

        print(json.dumps(dic))
    else:
        spaces = "              " if args['--cells'] else ""
        nr_cells = len(cells_v)
        b_status = " (BALANCING)" if cells_b_real else ""
        print("%sStatus:  %s" % (spaces, status.name + b_status))
        print("%s  Temp:  %.2fC" % (spaces, temp[0]))
        print("%s   SoC:  %.2f%% " % (spaces, soc))
        print("%s     I:  %.2fA " % (spaces, i))
        print("%s     V:  %.2fV " % (spaces, v))
        if args['--cells']:
            print("             Cells V:  %s" % (" ".join(["%.2fV" % v for v in cells_v])))
            print("     Cells balancing:  %s" % (" ".join(["  ^  " if cells_b_real & (1<<(15-i)) else "     " for i in range(0, nr_cells)])))
            print("Cells need balancing:  %s" % (" ".join(["  ^  " if cells_b_need & (1<<(15-i)) else "     " for i in range(0, nr_cells)])))
        if args['--events']:
            print("%sEvents:  %s" % (spaces, ", ".join([e['event'].name for e in events])))

def get_settings(bms):
    fully_charged_mv = bms.get_fully_charged_voltage_mv()
    fully_discharged_mv = bms.get_fully_discharged_voltage_mv()
    early_balancing_thr_mv = bms.get_early_balancing_voltage_threshold_mv()
    charge_finished_current_ma = bms.get_charge_finished_current_ma()
    capacity_ah = bms.get_battery_capacity_ah()
    nr_cells = bms.get_nr_cells()
    allowed_disbalance_mv = bms.get_allowed_disbalance_voltage_mv()
    overvoltage_cutoff_mv = bms.get_overvoltage_cutoff_mv()
    undervoltage_cutoff_mv = bms.get_undervoltage_cutoff_mv()
    discharge_overcurrent_cutoff_a = bms.get_discharge_overcurrent_cutoff_a()
    charge_overcurrent_cutoff_a = bms.get_charge_overcurrent_cutoff_a()
    overheat_cutoff_c = bms.get_overheat_cutoff_c()
    soc = bms.get_soc()

    if args['--json']:
        dic = {
            'fully-charged-voltage-mv': fully_charged_mv,
            'fully-discharged-voltage-mv': fully_discharged_mv,
            'early-balancing-voltage-threshold-mv': early_balancing_thr_mv,
            'charge-finished-current-ma': charge_finished_current_ma,
            'capacity-ah': capacity_ah,
            'nr-cells': nr_cells,
            'allowed-disbalance-voltage-mv': allowed_disbalance_mv,
            'overvoltage-cutoff-mv': overvoltage_cutoff_mv,
            'undervoltage-cutoff-mv': undervoltage_cutoff_mv,
            'discharge-overcurrent-cutoff-a': discharge_overcurrent_cutoff_a,
            'charge-overcurrent-cutoff-a': charge_overcurrent_cutoff_a,
            'overheat-cutoff-c': overheat_cutoff_c,
            'soc': soc
        }
        print(json.dumps(dic))
    else:
        print("Fully charged voltage:               %umV" % fully_charged_mv)
        print("Fully discharged voltage:            %umV" % fully_discharged_mv)
        print("Early balancing voltage threshold:   %umV" % early_balancing_thr_mv)
        print("Charge finished current:             %umA" % charge_finished_current_ma)
        print("Capacity:                            %uAh" % capacity_ah)
        print("Number of cells:                     %u"   % nr_cells)
        print("Allowed disbalance voltage:          %umV" % allowed_disbalance_mv)
        print("Over-voltage cutoff:                 %umV" % overvoltage_cutoff_mv)
        print("Under-voltage cutoff:                %umV" % undervoltage_cutoff_mv)
        print("Discharge over-current cutoff:       %uA"  % discharge_overcurrent_cutoff_a)
        print("Charge over-current cutoff:          %uA"  % charge_overcurrent_cutoff_a)
        print("Over-heat temperature cutoff:        %uC"  % overheat_cutoff_c)
        print("SoC:                                 %u"   % soc)

if __name__ == '__main__':
    args = docopt(__doc__)

    if args['--device']:
        port = serial.Serial(args['--device'], 115200, timeout=0)
    else:
        port = passthru_port()
    bms = bms(port)

    if args['get-state']:
        get_state(bms)
    elif args['get-settings']:
        get_settings(bms)
    elif args['set-fully-charged-voltage-mv']:
        bms.set_fully_charged_voltage_mv(int(args['<mv>']))
    elif args['set-fully-discharged-voltage-mv']:
        bms.set_fully_discharged_voltage_mv(int(args['<mv>']))
    elif args['set-early-balancing-voltage-threshold-mv']:
        bms.set_early_balancing_voltage_threshold_mv(int(args['<mv>']))
    elif args['set-charge-finished-current-ma']:
        bms.set_charge_finished_current_ma(int(args['<ma>']))
    elif args['set-capacity-ah']:
        bms.set_battery_capacity_ah(int(args['<ah>']))
    elif args['set-nr-cells']:
        bms.set_nr_cells(int(args['<nr>']))
    elif args['set-allowed-disbalance-voltage-mv']:
        bms.set_allowed_disbalance_voltage_mv(int(args['<mv>']))
    elif args['set-overvoltage-cutoff-mv']:
        bms.set_overvoltage_cutoff_mv(int(args['<mv>']))
    elif args['set-undervoltage-cutoff-mv']:
        bms.set_undervoltage_cutoff_mv(int(args['<mv>']))
    elif args['set-discharge-overcurrent-cutoff-a']:
        bms.set_discharge_overcurrent_cutoff_a(int(args['<a>']))
    elif args['set-charge-overcurrent-cutoff-a']:
        bms.set_charge_overcurrent_cutoff_a(int(args['<a>']))
    elif args['set-overheat-cutoff-c']:
        bms.set_overheat_cutoff_c(int(args['<c>']))
    elif args['set-soc']:
        bms.set_soc(int(args['<soc>']))
