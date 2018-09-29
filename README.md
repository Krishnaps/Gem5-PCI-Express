# Gem5-PCI-Express


The pertinent PCI Express files are pcie_link.hh, pcie_link.cc, PciBridge.py, 
PciBridge.cc and PciBridge.hh in the src/mem folder.

pcie_link.hh and pcie_link.cc contain class definitions and declarations for the 
PCIE link implementation. The PCIE links implement a simplified version
of the Ack/NAK protocol for reliable transmission of packets. Various overheads
are taken into account too. 

PciBridge.py contains python configuration for the Gem5 Root Complex and 
PCI Express switch. The Root Complex has 3 root ports, while the switch has 3
downstream and one upstream port. Each "port" mentioned is further subdivided 
into a master/slave port pair for sending PIO requests to PCIE devices and
receiving DMA requests from PCI Express devices respectively. 

The Gem5 topology with PCIE links, PCI Express Root Complex, Switch and IDE controller is in configs/common/FSConfig.py

m5out contains a checkpoint file that can be used, and vmlinux is the kernel image with PCI Express support. 


