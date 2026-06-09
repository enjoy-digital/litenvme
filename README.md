```
                               __   _ __      _  ___   ____  ___
                              / /  (_) /____ / |/ / | / /  |/  /__
                             / /__/ / __/ -_)    /| |/ / /|_/ / -_)
                            /____/_/\__/\__/_/|_/ |___/_/  /_/\__/

                                 Copyright 2025-2026 / EnjoyDigital

                             A small footprint and configurable NVMe host core
                                      powered by Migen & LiteX
```

[![](https://github.com/enjoy-digital/litenvme/workflows/ci/badge.svg)](https://github.com/enjoy-digital/litenvme/actions) ![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg) [![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/enjoy-digital/litenvme)


[> Intro
--------
LiteNVMe provides a small footprint and configurable NVMe host core.

LiteNVMe is part of LiteX libraries whose aims are to lower the entry level of
complex FPGA cores by providing simple, elegant and efficient implementations
of components used in today's SoC such as Ethernet, SATA, PCIe, SDRAM Controller...

Using Migen to describe the HDL allows the core to be highly and easily configurable.

LiteNVMe can be used as LiteX library on top of a LitePCIe RootPort or can be
integrated with your standard design flow by generating the Verilog RTL that you
will use as a standard core.

LiteNVMe is an NVMe host: it brings up an off-the-shelf NVMe SSD through PCIe,
configures the controller and I/O queues, then exposes block read/write transfers
to FPGA logic.

<p align="center"><img src="doc/images/litenvme_setup.jpg" width="800"></p>

[> Features
-----------
PHY:
  - LitePCIe RootPort based PCIe access.
  - Xilinx Ultrascale+ PCIe Gen3 x4 validated on Alibaba XCKU3P.
  - 256-bit datapath on the validated design.

Core:
  - PCIe CFG and BAR0 MMIO accessors.
  - NVMe controller enable, Identify and I/O queue creation.
  - Host-memory DMA window for NVMe command/data buffers.
  - Hardware I/O engine with queue-depth support.
  - Optional pure-RTL NVMe init sequencer for CPU-less bring-up.

Frontend:
  - AXI-Stream block read/write interface.
  - CSR/software control for LiteX SoCs.
  - Standalone Verilog generator with YAML configuration.
  - Reference Alibaba KU3P test SoC and bench scripts.

Software:
  - Firmware bring-up helpers.
  - Etherbone host tools for hardware validation.
  - Pytest simulation and generator test suite.

[> FPGA Proven
--------------
LiteNVMe has been validated end-to-end on real hardware:

- Alibaba Cloud KU3P FPGA card with a commercial NVMe SSD.
- PCIe Gen3 x4 RootPort link-up, BAR0 discovery and controller enable.
- Admin Identify, I/O queue creation and DMA read/write transfers.
- Firmware bring-up and pure-RTL init bring-up.
- 8 KiB QD32 transfers at around 2.7 GB/s reads and writes on the validated setup.
- Write/read-back data integrity checks.

Detailed validation, resource and performance information is available in:

- `doc/VALIDATION.md`
- `doc/STANDALONE_CORE.md`
- `doc/NVME_PERFORMANCE.md`
- `doc/NVME_CORE_COMPARISON.md`

[> Possible improvements
------------------------
- add PCIe Gen4/Gen5 and wider lane configurations.
- extend validation to more FPGA families and NVMe SSDs.
- widen pure-RTL init compatibility by decoding more controller/PCIe parameters.
- add larger/chained transfers beyond the staging window.
- add DDR/LiteDRAM backed host-memory window support.
- add more documentation and integration examples.
- ... See below Support and consulting :)

If you want to support these features, please contact us at florent [AT]
enjoy-digital.fr.

[> Getting started
------------------
1. Install Python 3.8+ and FPGA vendor's development tools.
2. Install LiteX and the cores by following the LiteX's wiki [installation guide](https://github.com/enjoy-digital/litex/wiki/Installation).
3. You can use LiteNVMe directly as a LiteX library or generate a standalone Verilog core:

```sh
$ litenvme_gen examples/alibaba_xcku3p.yml --output-dir build --name litenvme_core
```

The generated core exposes PCIe pads, status signals and an AXI-Stream block
interface. See `doc/STANDALONE_CORE.md` for the complete integration flow.

The reference Alibaba KU3P design can also be built and tested over Etherbone:

```sh
$ ./bench/alibaba_xcku3p.py --with-cpu --cpu-boot=bios --with-etherbone \
    --with-io-engine --with-block-streamer --csr-csv=csr.csv --build --load
$ ./bench/hw_block.sh
```

For CPU-less bring-up, select the RTL init path:

```sh
$ ./bench/alibaba_xcku3p.py --with-etherbone --with-io-engine --with-rtl-init \
    --csr-csv=csr.csv --build --load
$ ./bench/hw_rtlinit.sh
```

[> Documentation
----------------
- `doc/DIAGRAMS.md`: Visual architecture diagrams.
- `doc/ARCHITECTURE.md`: As-built Gen3-256b architecture.
- `doc/STANDALONE_CORE.md`: Standalone generation, interface, resources and usage.
- `doc/NVME_PERFORMANCE.md`: Throughput study and measured limits.
- `doc/VALIDATION.md`: Simulation, generator and hardware-validation evidence.
- `doc/NVME_CORE_COMPARISON.md`: Comparison with other public NVMe cores.
- `doc/archive/PROGRESS.md`: Archived development log.

[> Tests
--------
Unit tests are available in ./test/.
To run all the unit tests:

```sh
$ python3 -m pytest -q
```

Tests can also be run individually:

```sh
$ python3 -m pytest -q test/test_gen_config.py
```

[> License
----------
LiteNVMe is released under the very permissive two-clause BSD license. Under
the terms of this license, you are authorized to use LiteNVMe for closed-source
proprietary designs.
Even though we do not require you to do so, those things are awesome, so please
do them if possible:
 - tell us that you are using LiteNVMe
 - cite LiteNVMe in publications related to research it has helped
 - send us feedback and suggestions for improvements
 - send us bug reports when something goes wrong
 - send us the modifications and improvements you have done to LiteNVMe.

[> Support and consulting
-------------------------
We love open-source hardware and like sharing our designs with others.

LiteNVMe is developed and maintained by EnjoyDigital.

If you would like to know more about LiteNVMe or if you are already a happy
user and would like to extend it for your needs, EnjoyDigital can provide standard
commercial support as well as consulting services.

So feel free to contact us, we'd love to work with you! (and eventually shorten
the list of the possible improvements :)

[> Contact
----------
E-mail: florent [AT] enjoy-digital.fr
