```{=latex}
\frontmatter
```

# About this document {.unnumbered}

This manual is the reference of the LiteNVMe project: an NVMe host core that lets FPGA logic read
and write an off-the-shelf NVMe SSD over a LitePCIe root port, with no host computer and no
operating system in the data path.

It is written for FPGA engineers who need mass storage in a design, and it teaches NVMe along the
way.

No prior knowledge of NVMe is assumed. Part I explains how a host drives an NVMe SSD over PCI
Express: the controller registers, the queues in host memory, the commands and the data pointers.
PCI Express itself is not explained again: the text links to the LitePCIe manual for the link, the
packets and the configuration space.

Part II describes the core: its architecture, the register accessors, the host-memory window, the
I/O engine, its front-ends and the two ways to bring the SSD up. Part III is the user guide: the
first run, the integration in a design, the bench and its tools, the simulation and the debugging.

The appendices hold the register reference, the build options, the glossary, the performance
figures and the maintenance notes of this manual.

| If you want to | Read |
|------------------------------------------|----------------------------------------------|
| Know what the project does and does not | [Introduction](#introduction) |
| Learn how a host drives an NVMe SSD | [NVMe over PCIe in a nutshell](#nvme-primer) |
| See how the core is organised | [Architecture](#architecture) |
| Run the simulations, then a first SSD | [Getting started](#getting-started) |
| Add NVMe storage to your own design | [Integration](#integration) |
| Understand an SSD that does not answer | [Debugging](#debugging) |

The manual follows the repository: every name and default is taken from the source code, every
measured number from a log of the repository, and the manual is generated from it on the date
printed on the title page. When the code and the manual disagree, the code is right and the
manual has a bug.

## Conventions {.unnumbered}

- Numbers: `0x` prefix for hexadecimal, `0b` for binary. Bit ranges are written high to low,
  `[31:16]`.
- Sizes in bytes, KiB and MiB. Throughputs in MB/s (10^6 bytes per second), as the firmware
  prints them. Link rates as Gen2 and Gen3, link widths as x4.
- NVMe names follow the NVM Express Base Specification, cited by name. Queue and command objects
  are written in capitals (SQ, CQ, SQE, CQE, CID, PRP, LBA); controller registers by name (CAP,
  CC, CSTS) and their fields dotted (CC.EN, CSTS.RDY).
- PCI Express names follow the LitePCIe manual: TLP, MemRd, MemWr, CplD, BAR, BDF. *MPS* is the
  PCIe Max Payload Size; the NVMe memory page size is always written in words.
- *The host* is the FPGA side that drives the SSD: the root port, the core and the firmware or the
  init sequencer. *The bench PC* is the computer that builds and loads the bitstream and runs the
  host tools.
- *The SSD* is the drive; *the controller* is its NVMe function, the registers and queues the
  host talks to. *The core* is the `LiteNVMe` class and its sub-blocks; *the project* is LiteNVMe
  as a whole, with the bench, the firmware, the tools and the generator.
- *A sector* is a 512-byte logical block, the only block size the core uses.
- `<config>` stands for a YAML configuration file of the standalone generator `litenvme_gen`, such
  as `examples/alibaba_xcku3p.yml`.
- Paths are relative to the repository root. Shell commands are given for a Linux PC.
