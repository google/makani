# Copyright 2020 Makani Technologies LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Inverter fault strings, each group corresponds to 16 bits in a fault word.

These strings are defined by @robertsonp.
"""

INVERTER_FAULTS = [
    [  # D0
        "DC Input Not Ready",
        "Voltage Mismatch",
        "Command Stop",
        "Precharge Error",
        "E Stop",
        "Low Power",
        "Low Input Current",
        "Low Voltage Ridethrough Failure",
        "Door Open",
        "DC Breaker Not Closed",
        "AC Breaker Not Closed",
        "AC Fuse 1",
        "Manual Stop",
        "AC Fuse 2",
        "Reserved",
        "Change in Operating Mode"
    ],
    [  # D1
        "DC Input Over Voltage",
        "DC Input Under Voltage",
        "DC Bus Over Voltage",
        "DC Bus Under Voltage",
        "DC Ground Fault",
        "Grid OverVolt S",
        "Grid OverVolt F",
        "Grid UnderVolt F",
        "Grid UnderVolt S",
        "VoltUnbalanced F",
        "Grid OverFreq",
        "Grid UnderFreq",
        "Neutral OverCurrent S",
        "Neutral OverCurrent F",
        "Line Transient Overvoltage",
        "Drive Signals Not Synchronized"
    ],
    [  # D2
        "Program Check Error",
        "FPGA Version Error",
        "Power History Error",
        "History Checksum Error",
        "Parameter Checksum Error",
        "PLL Unlock Error",
        "U AC Ready Error",
        "PV Reverse Error",
        "U Fb Scaling",
        "I Fb Scaling",
        "I Diff",
        "Parameter Change",
        "Over Modulated",
        "ADC",
        "NVRAM",
        "FPGA"
    ],
    [  # D3
        "Board Power",
        "Reserved",
        "GFCI",
        "Grid OF Fast",
        "FPGA Watchdog",
        "AC Surge",
        "Inv Fuse 1",
        "Inv Fuse 2",
        "Inv1 Over Temperature",
        "Inv2 Over Temperature",
        "TSFM Over Temperature",
        "L1 OT Over Temperature",
        "L2 OT Over Temperature",
        "Z GFDI",
        "I GFDI",
        "U GFDI"
    ],
    [  # D4
        "Fb A1",
        "Fb B1",
        "Fb C1",
        "Fb A2",
        "Fb B2",
        "Fb C2",
        "DC Input Overcurrent",
        "DC Input Overcurrent Instant",
        "DC Undervolt Instant",
        "DC Overvolt Instant",
        "Inv SW OC",
        "Inv HW OC 1",
        "Inv HW OC 2",
        "Grid Overcurrent",
        "Grid Current Unblanced",
        "Inverter Undervolts"
    ],
    [  # D5
        "Over Temperature",
        "Ambient Over Temperature",
        "Heat Sink 1 Over Temperature",
        "Heat Sink 2 Over Temperature",
        "Heat Sink 3 Over Temperture",
        "Heat Sink 4 Over Temperature",
        "Heat Sink 5 Over Temperature",
        "Heat Sink 6 Over Temperature",
        "Grid Seq",
        "MD5 Lock",
        "Inv HW OC 3",
        "DSP Inv OC Inst",
        "Hw Inv OC Inst",
        "Reserved",
        "Reserved",
        "Reserved"
    ],
    [  # D6
        "Fan 1",
        "Fan 2",
        "DC Contactor Open",
        "DC Contactor Closed",
        "AC Contactor Open",
        "AC Contactor Closed",
        "DC Surge",
        "Grid Undervolts Inst",
        "System Halted",
        "Cal Data Checksum",
        "Gate Fb A3",
        "Gate Fb B3",
        "Gate Fb C3",
        "Gate Fb A4",
        "Gate Fb B4",
        "Gate Fb C4"
    ],
    [  # D7
        "Fan 1 Warning",
        "Fan 2 Warning",
        "Fan 3 Warning",
        "Low Power",
        "HT Downrating",
        "Reg Warning",
        "Reserved",
        "Reserved",
        "Reserved",
        "Reserved",
        "Reserved",
        "Reserved",
        "Reserved",
        "Reserved",
        "Reserved",
        "Reserved"
    ]
]


