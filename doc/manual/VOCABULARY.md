# Vocabulary of the LiteNVMe manual

Project-specific writing rules, completing the generic `STYLE.md` of the manual kit.

## Names of the parts

- *the core* is the `LiteNVMe` class and its sub-blocks (`litenvme/`); *the project* is LiteNVMe as
  a whole, with the bench SoC, the firmware, the host tools and the generator.
- *the host* is the FPGA side: the design that drives the SSD (root port, core, firmware). It is
  never the PC on the bench; that one is *the bench PC*.
- *the root port* is the LitePCIe root port of the FPGA (`LitePCIeRootPort` on a PHY in
  `mode="RootPort"`); its own configuration space is reached through *cfg_mgmt*, the
  configuration management port of the hard IP.
- *the SSD* is the drive; *the controller* is its NVMe function, the PCIe function whose BAR0
  holds the NVMe registers. Say *the SSD* for the device and the link, *the controller* for its
  registers and queues.
- *accessors*: the *configuration accessor* (`LiteNVMePCIeCfgAccessor`), the *MMIO accessor*
  (`LiteNVMePCIeMmioAccessor`), the *doorbell accessor* (the second MMIO accessor, `mmio_db`) and
  the *root configuration accessor* (`LiteNVMeRootCfgMgmt`).
- *the host-memory window* is the memory the SSD reads and writes through PCIe
  (`LiteNVMeHostMemResponder` and its backend). Never "hostmem" in prose; `hostmem` only as a
  code identifier.
- *the I/O engine* (`LiteNVMeIOEngine`, `LiteNVMeIOEngineAXI`), *the block streamer*
  (`LiteNVMeBlockStreamer`), *the request generator* (`LiteNVMeRequestGen`), *the init
  sequencer* (`LiteNVMeInitSequencer`). The block streamer and the request generator are the
  *front-ends* of the I/O engine.
- *the staging region* is the part of the host-memory window the block streamer copies data
  through; *the PRP-list region* holds the PRP-list pages of the I/O engine.
- *bring-up* is the whole sequence from the release of PERST# to `init_done`; *the firmware* is
  `bench/firmware/main.c` on the soft CPU; *the generator* is `litenvme_gen` (`litenvme/gen.py`),
  *the standalone core* its Verilog output.
- *target* is a bench script of `bench/` (`alibaba_xcku3p.py`); *the bench SoC* is the design it
  builds; *harness* scripts are the `bench/hw_*.sh` shell scripts.

## Protocol names

- NVMe objects in capitals, as the NVM Express Base Specification writes them: SQ, CQ, SQE, CQE,
  CID, SQHD, PRP, PRP1, PRP2, SGL, LBA, NLB, NSID, MDTS. First use spelled out: *submission
  queue (SQ)*.
- Controller registers by name in capitals without code font: CAP, VS, CC, CSTS, AQA, ASQ, ACQ;
  fields dotted: CC.EN, CSTS.RDY, CAP.MQES, CAP.DSTRD, CAP.TO.
- *admin queue* (queue 0, admin commands) and *I/O queue* (queue 1 in LiteNVMe); *queue size*
  is the number of entries of a ring, *queue depth* the number of commands in flight.
- *doorbell*: a controller register the host writes with a new SQ tail or CQ head; *doorbell
  stride* from CAP.DSTRD.
- *phase bit*: the P bit of a CQE; *phase*: the value the host expects in it for this
  pass over the ring.
- *sector* is a 512-byte logical block (the only LBA size the core uses); *namespace* is an NVMe
  namespace (LiteNVMe uses namespace 1).
- *MPS* is the PCIe Max Payload Size and *MRRS* the Max Read Request Size. The NVMe *memory page
  size* (field CC.MPS, 4 KiB in LiteNVMe) is always written in words, never as "MPS".
- PCIe packet names as in the LitePCIe manual: TLP, MemRd, MemWr, CplD, CfgRd0, CfgWr0, BAR,
  BDF, BME. PCIe topics are not explained again: link to the LitePCIe manual.
- Generations as Gen2, Gen3; link widths as x4.
- Cite the specification by name, "NVM Express Base Specification", without a version or a
  section number unless the section is checked.

## Placeholders

- `<config>` is a generator YAML file (`examples/alibaba_xcku3p.yml`); `<bar0>` the bus address
  given to BAR0 (`0xe0000000` on the bench); `<lba>` a starting sector number.

## Hardware names

- Board: *Alibaba XCKU3P* (the Alibaba Cloud KU3P card, FPGA XCKU3P); *M.2 adapter* for the
  PCIe-slot-to-M.2 adapter that carries the SSD.
- FPGA families: Kintex UltraScale+, Xilinx 7-Series.
- SSDs are named only as the logs name them (vendor and device ID); prose says *the SSD*.
