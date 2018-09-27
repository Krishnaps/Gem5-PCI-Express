/*
 * Copyright (c) 2011-2013 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Ali Saidi
 *          Steve Reinhardt
 *          Andreas Hansson
 */

/**
 * @file
 * Declaration of a memory-mapped bridge that connects a master
 * and a slave through a request and response queue.
 */

#ifndef __MEM_PCIBRIDGE_HH__
#define __MEM_PCIBRIDGE_HH__


#include <deque>

#include "base/types.hh"
#include "mem/mem_object.hh"
#include "params/PciBridge.hh"
#include "dev/pci/types.hh"
#include "dev/pci/host.hh"
#include "dev/pci/pcireg.h"


#define PCIE_CONFIG_SIZE 0xFFF  // PCi express has an extended configuration space also. Total config space is 4096 B. Range of config addresses is from 0-4095. 
#define PCIE_HEADER_SIZE 64    // The standard PCIe/PCI header is just 64 B. Other config space areas are occupied by extended capability registers and extended config space registers(only in PCIe). 
#define PXCAPSIZE 36 // Pcie capability occupies 9 Dwords for PCi-Pci bridge.
#define SLOT_REG_BASE 20
#define SLOT_REG_LIMIT 27
#define IO_MASK 0x00000001
#define MEM_MASK 0x0000000F
#define IO_BASE_MASK 0x0F
#define IO_LIMIT_MASK 0x0F
#define PREFETCH_MEM_BASE_MASK 0x000
#define PREFETCH_MEM_LIMIT_MASK 0x000F
#define MMIO_BASE_MASK 0x000F
#define MMIO_LIMIT_MASK 0x000F
#define IO_BASE_SHIFT 4096
#define PREFETCH_BASE_SHIFT 1048576
#define ADDR_MAX 0xFFFFFFFFFFFFFFFF // 2^64 -1 

class PciBridge ;  

     
      
  
class config_class
{
  public:
     union config_storage
     {
       uint8_t data[64] ;  // 64 B data to hold bridge configuration header. 
       struct
       {
          uint16_t VendorId ; 
          uint16_t DeviceId ;
          uint16_t Command ; 
          uint16_t Status ;  
          uint8_t Revision ;
          uint8_t ProgIF ; 
          uint8_t SubClassCode ;
          uint8_t ClassCode ; 
          uint8_t CacheLineSize ; 
          uint8_t LatencyTimer ; 
          uint8_t HeaderType ; 
          uint8_t BIST ;
          uint32_t Bar0 ; 
          uint32_t Bar1 ; 
          uint8_t  PrimaryBusNumber ; 
          uint8_t  SecondaryBusNumber ; 
          uint8_t  SubordinateBusNumber ; 
          uint8_t  SecondaryLatencyTimer ;
          uint8_t  IOBase ; 
          uint8_t  IOLimit ; 
          uint16_t  SecondaryStatus ; 
          uint16_t  MemoryBase ;
          uint16_t  MemoryLimit ; 
          uint16_t  PrefetchableMemoryBase ; 
          uint16_t  PrefetchableMemoryLimit ;
          uint32_t  PrefetchableMemoryBaseUpper ; 
          uint32_t  PrefetchableMemoryLimitUpper ; 
          uint16_t  IOBaseUpper ;
          uint16_t  IOLimitUpper ;
          uint8_t   CapabilityPointer ;
          uint8_t reserved[3] ;           // initialize reserved fields to 0 
          uint32_t ExpansionROMBaseAddress ;
          uint8_t InterruptLine ; 
          uint8_t InterruptPin ; 
          uint16_t BridgeControl ; 
    };
  } storage1 ;
  
  union pcie_capability_storage
  {
    uint8_t data[36] ; // pcie capability structure occupies 36 B . 
    struct
    {
      uint8_t PXCAPCapId ;
      uint8_t PXCAPNextCapability ; 
      uint16_t PXCAPCapabilities;
      uint32_t PXCAPDevCapabilities ;
      uint16_t PXCAPDevStatus ; 
      uint16_t PXCAPDevCtrl ; 
      uint32_t PXCAPLinkCap; 
      uint16_t PXCAPLinkCtrl ; 
      uint16_t PXCAPLinkStatus ; 
      uint32_t PXCAPSlotCapabilities ; // set to 0 , assume that corresponding PCIe device is not in slot
      uint16_t PXCAPSlotControl ; // set to 0 , assume that corresponding PCIe device is not in add -in card slot
      uint16_t PXCAPSlotStatus ; // set to 0 for above reason. 
      uint32_t PXCAPRootControl ; 
      uint32_t PXCAPRootStatus ;
    } ;
  } storage2 ; 

 uint32_t barsize[2] ; // size of BARs if implementeed for the bridge
 uint32_t barflags[2] ; // whether the BARs have been written to with the base address. 
 int pci_bus, pci_dev, pci_func ; // Bus , device and Function numbers for the PCI bridge
 Tick readConfig(PacketPtr pkt) ;  // Functions so configuration software can read and write values to the bridge
 Tick writeConfig(PacketPtr pkt) ;
 bool isWritable(int offset) ; // function to see if config address being written to has a R/W property. Return False if that address is read-only. 
 uint32_t BarSize[2] ; // To store sizes of BARs , if implemented. 
 // Any need to implement get AddrRange function ??? Seems to be needed only for PIO port, which bridge doesn't have. No need for DMA related fns either 
 Tick configDelay ; // Time to be returned when a configuration access is made. Since no PIO port , no need for PIO latency either.
 PciBusAddr BridgeAddr ;
 uint8_t PXCAPBaseOffset ; // offset of the PCIe capability structure in config space. All PCIe devices have to implement this . 
 config_class(int pci_bus , int pci_dev, int pci_func , uint8_t root_port_number):BridgeAddr((uint8_t)pci_bus , (uint8_t)pci_dev , (uint8_t)pci_func , root_port_number) {}  // constructor just initalizes the BridgeAddr object, which is used by the PCI host for config accesses
 
  PciBridge * bridge ; 
int is_switch ; 

 Addr getBar0() ;
 Addr getBar1() ;
 Addr getIOBase() ;
 Addr getIOLimit() ;
 Addr getMemoryBase() ; 
 Addr getMemoryLimit() ; 
 Addr getPrefetchableMemoryBase() ; 
 Addr getPrefetchableMemoryLimit() ; 

 uint8_t id ; 
 uint8_t is_valid ; 

 bool valid_io_base ; 
 bool valid_io_limit ; 
 bool valid_prefetchable_memory_base ;
 bool valid_prefetchable_memory_limit ; 
 bool valid_memory_base ; 
 bool valid_memory_limit ;  
 
 
} ; 
 
      
         

  
     

/**
 * A bridge is used to interface two different crossbars (or in general a
 * memory-mapped master and slave), with buffering for requests and
 * responses. The bridge has a fixed delay for packets passing through
 * it and responds to a fixed set of address ranges.
 *
 * The bridge comprises a slave port and a master port, that buffer
 * outgoing responses and requests respectively. Buffer space is
 * reserved when a request arrives, also reserving response space
 * before forwarding the request. If there is no space present, then
 * the bridge will delay accepting the packet until space becomes
 * available.
 */
class PciBridge : public MemObject
{
  protected:

    /**
     * A deferred packet stores a packet along with its scheduled
     * transmission time
     */
    class DeferredPacket
    {

      public:

        const Tick tick;
        const PacketPtr pkt;

        DeferredPacket(PacketPtr _pkt, Tick _tick) : tick(_tick), pkt(_pkt)
        { }
    };

    // Forward declaration to allow the slave port to have a pointer
    class PciBridgeMasterPort;

    /**
     * The port on the side that receives requests and sends
     * responses. The slave port has a set of address ranges that it
     * is responsible for. The slave port also has a buffer for the
     * responses not yet sent.
     */
    class PciBridgeSlavePort : public SlavePort
    {

      private:

        /** The bridge to which this port belongs. */
        PciBridge& bridge;

        /**
         * Master port on the other side of the bridge.
         */
        //PciBridgeMasterPort& masterPort;

        /** Minimum request delay though this bridge. */
        const Cycles delay;

        // Removed 'Ranges' object since the PCI Bridge slaveport accepts ranges created by the BARs as well as the I/O and memory bases and limits.
         

        /**
         * Response packet queue. Response packets are held in this
         * queue for a specified delay to model the processing delay
         * of the bridge. We use a deque as we need to iterate over
         * the items for functional accesses.
         */
        std::deque<DeferredPacket> transmitList;

        /** Counter to track the outstanding responses. */
        unsigned int outstandingResponses;

        /** If we should send a retry when space becomes available. */
        bool retryReq;
        bool is_upstream ; 

        /** Max queue size for reserved responses. */
        unsigned int respQueueLimit;

        /**
         * Upstream caches need this packet until true is returned, so
         * hold it for deletion until a subsequent call
         */
        std::unique_ptr<Packet> pendingDelete;

        /**
         * Is this side blocked from accepting new response packets.
         *
         * @return true if the reserved space has reached the set limit
         */
        bool respQueueFull() const;

        /**
         * Handle send event, scheduled when the packet at the head of
         * the response queue is ready to transmit (for timing
         * accesses only).
         */
        void trySendTiming();

        /** Send event for the response queue. */
        EventFunctionWrapper sendEvent;

      public:

        /**
         * Constructor for the BridgeSlavePort.
         *
         * @param _name the port name including the owner
         * @param _bridge the structural owner
         * @param _masterPort the master port on the other side of the bridge
         * @param _delay the delay in cycles from receiving to sending
         * @param _resp_limit the size of the response queue
         * @param _ranges a number of address ranges to forward
         */
        PciBridgeSlavePort(const std::string& _name, PciBridge& _bridge, bool upstream, uint8_t slaveID,uint8_t rcid , 
                        Cycles _delay,
                        int _resp_limit);
        uint8_t slaveID ; // slave port id
        uint8_t rcid ; // root complex id 
         void fill_ranges(AddrRangeList & ranges , config_class * storage_ptr) const ; 
        /**
         * Queue a response packet to be sent out later and also schedule
         * a send if necessary.
         *
         * @param pkt a response to send out after a delay
         * @param when tick when response packet should be sent
         */
        void schedTimingResp(PacketPtr pkt, Tick when);
        void public_sendRangeChange()
        {
            sendRangeChange() ;
        }

        /**
         * Retry any stalled request that we have failed to accept at
         * an earlier point in time. This call will do nothing if no
         * request is waiting.
         */
        void retryStalledReq();

      protected:

        /** When receiving a timing request from the peer port,
            pass it to the bridge. */
        bool recvTimingReq(PacketPtr pkt);

        /** When receiving a retry request from the peer port,
            pass it to the bridge. */
        void recvRespRetry();

        /** When receiving a Atomic requestfrom the peer port,
            pass it to the bridge. */
        Tick recvAtomic(PacketPtr pkt);

        /** When receiving a Functional request from the peer port,
            pass it to the bridge. */
        void recvFunctional(PacketPtr pkt);

        /** When receiving a address range request the peer port,
            pass it to the bridge. */
        AddrRangeList getAddrRanges() const;
    };


    /**
     * Port on the side that forwards requests and receives
     * responses. The master port has a buffer for the requests not
     * yet sent.
     */
    class PciBridgeMasterPort : public MasterPort
    {

      private:

        /** The bridge to which this port belongs. */
        PciBridge& bridge;
        uint64_t totalCount ;         
        /**
         * The slave port on the other side of the bridge.
         */
       // PciBridgeSlavePort& slavePort;

        /** Minimum delay though this bridge. */
        const Cycles delay;

        /**
         * Request packet queue. Request packets are held in this
         * queue for a specified delay to model the processing delay
         * of the bridge.  We use a deque as we need to iterate over
         * the items for functional accesses.
         */
        std::deque<DeferredPacket> transmitList;

        /** Max queue size for request packets */
        const unsigned int reqQueueLimit;

        /**
         * Handle send event, scheduled when the packet at the head of
         * the outbound queue is ready to transmit (for timing
         * accesses only).
         */
        void trySendTiming();
        void incCount() ; 

        /** Send event for the request queue. */
        EventFunctionWrapper sendEvent;
        EventFunctionWrapper countEvent ; 

      public:

        /**
         * Constructor for the BridgeMasterPort.
         *
         * @param _name the port name including the owner
         * @param _bridge the structural owner
         * @param _slavePort the slave port on the other side of the bridge
         * @param _delay the delay in cycles from receiving to sending
         * @param _req_limit the size of the request queue
         */
        PciBridgeMasterPort(const std::string& _name, PciBridge& _bridge,uint8_t rcid , 
                         Cycles _delay,
                         int _req_limit);

        /**
         * Is this side blocked from accepting new request packets.
         *
         * @return true if the occupied space has reached the set limit
         */
        bool reqQueueFull() const;

        /**
         * Queue a request packet to be sent out later and also schedule
         * a send if necessary.
         *
         * @param pkt a request to send out after a delay
         * @param when tick when response packet should be sent
         */
        void schedTimingReq(PacketPtr pkt, Tick when);

        /**
         * Check a functional request against the packets in our
         * request queue.
         *
         * @param pkt packet to check against
         *
         * @return true if we find a match
         */
        bool checkFunctional(PacketPtr pkt);
        uint8_t rcid ; 

      protected:

        /** When receiving a timing request from the peer port,
            pass it to the bridge. */
        bool recvTimingResp(PacketPtr pkt);

        /** When receiving a retry request from the peer port,
            pass it to the bridge. */
        void recvReqRetry();
    };


   

  public:
        /** Slave port of the bridge. */
    PciBridgeSlavePort slavePort;
    PciBridgeSlavePort slavePort_DMA1 ;
    PciBridgeSlavePort slavePort_DMA2 ; 
    PciBridgeSlavePort slavePort_DMA3 ;  

    /** Master port of the bridge. */
    PciBridgeMasterPort masterPort_DMA ; 
    PciBridgeMasterPort masterPort1;
    PciBridgeMasterPort masterPort2 ; 
    PciBridgeMasterPort masterPort3 ; 
    
 
    PciBridgeMasterPort * getMasterPort(Addr address) ; // Given a particular address which Master Port to forward it to ?  A slaveport calls this function when it receives a request. 
    PciBridgeSlavePort *  getSlavePort(int bus_num) ; 
    virtual BaseMasterPort& getMasterPort(const std::string& if_name,
                                          PortID idx = InvalidPortID);
    virtual BaseSlavePort& getSlavePort(const std::string& if_name,
                                        PortID idx = InvalidPortID);

    virtual void init();

    typedef PciBridgeParams Params;
    config_class * storage_ptr1 ; // create a new config_class when the bridge corresponding to a root port is created. 
    config_class * storage_ptr2 ; // Configuration for root port 2 
    config_class * storage_ptr3 ; // config structure for R.P. 3
    config_class * storage_ptr4 ; 
    void initialize_ports (config_class * storage_ptr , Params * p ) ; 
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
    uint8_t rc_id ; // root complex id 
    int is_switch ;
    int is_transmit ;  
    
    PciBridge(Params *p);
    ~PciBridge() ; 
};

#endif //__MEM_PCIBRIDGE_HH__
