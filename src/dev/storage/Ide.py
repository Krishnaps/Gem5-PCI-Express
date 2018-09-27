# Copyright (c) 2005-2007 The Regents of The University of Michigan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Nathan Binkert

from m5.SimObject import SimObject
from m5.params import *
from PciDevice import PciDevice

class IdeID(Enum): vals = ['master', 'slave']

class IdeDisk(SimObject):
    type = 'IdeDisk'
    cxx_header = "dev/storage/ide_disk.hh"
    delay = Param.Latency('1us', "Fixed disk delay in microseconds")
    driveID = Param.IdeID('master', "Drive ID")
    image = Param.DiskImage("Disk image")

class IdeController(PciDevice):
    type = 'IdeController'
    cxx_header = "dev/storage/ide_ctrl.hh"
    disks = VectorParam.IdeDisk("IDE disks attached to this controller")

    VendorID = 0x8086
    DeviceID = 0x7111
    Command = 0x0
    Status = 0x280
    Revision = 0x0
    ClassCode = 0x01
    SubClassCode = 0x01
    ProgIF = 0x85
    BAR0 = 0x00000001
    BAR1 = 0x00000001
    BAR2 = 0x00000001
    BAR3 = 0x00000001
    BAR4 = 0x00000001
    BAR5 = 0x00000001
    CapabilityPtr = 0xC8
    InterruptLine = 0x1f
    InterruptPin = 0x01
    BAR0Size = '8B'
    BAR1Size = '4B'
    BAR2Size = '8B'
    BAR3Size = '4B'
    BAR4Size = '16B'
    PMCAPBaseOffset = 0xC8
    PMCAPNextCapability= 0xD0
    PMCAPCapId = 0x01
    PMCAPCapabilities= 0x0022
    PMCAPCtrlStatus= 0x0000
    MSICAPBaseOffset = 0xD0
    MSICAPCapId = 0x05
    MSICAPNextCapability= 0xE0
    MSICAPMsgCtrl = 0x0080
    MSICAPMsgAddr = 0x00000000
    MSICAPMsgUpperAddr= 0x00000000
    MSICAPMsgData = 0x0000
    MSIXCAPBaseOffset = 0xA0
    MSIXCAPNextCapability= 0x00
    MSIXCAPCapId = 0x11
    MSIXMsgCtrl = 0x0001
    MSIXTableOffset = 0x00000003
    MSIXPbaOffset = 0x00004003
    PXCAPBaseOffset = 0xE0
    PXCAPNextCapability= 0xA0 #No need MSI-X or MSI Capability. Looks like 82574 supports legacy PCI interrupts
    PXCAPCapId = 0x10
    PXCAPCapabilities=0x0001
    PXCAPDevCapabilities = 0x00008CC1
    PXCAPDevCtrl= 0x2810
    PXCAPDevStatus=0x0000
    PXCAPLinkCap = 0x01031011 #00031C11 , disabled aspm, set pcie port number of 1. 
    PXCAPLinkCtrl= 0x0000
    PXCAPLinkStatus= 0x1011
    io_shift = Param.UInt32(0x0, "IO port shift");
    ctrl_offset = Param.UInt32(0x0, "IDE disk control offset")
