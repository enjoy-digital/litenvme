```
                               __   _ __      _  ___   ____  ___
                              / /  (_) /____ / |/ / | / /  |/  /__
                             / /__/ / __/ -_)    /| |/ / /|_/ / -_)
                            /____/_/\__/\__/_/|_/ |___/_/  /_/\__/

                           Small footprint LiteX based NVMe core.
                           Copyright (c) 2025-2026 Enjoy-Digital.
```

![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)

> **Status: WIP**
>
> LiteNVMe’s goal is a **full NVMe core** (controller/driver-side logic) that can integrate cleanly into LiteX SoCs. Development is currently split into small, verifiable milestones to de-risk the project: first prove PCIe + NVMe protocol correctness end-to-end, then replace temporary components with the final high-performance architecture.


[> Intro
--------

LiteNVMe aims to provide a small footprint and configurable **NVMe core** for LiteX.

The end goal is a complete NVMe solution (Admin + I/O queues, PRP/SGL data paths,
DMA to/from system memory, software integration). Development is done through
incremental, verifiable milestones to validate PCIe/NVMe behavior early and keep
the final core robust.

Today, LiteNVMe uses a bring-up oriented setup (LitePCIe RootPort + a small
AXI-mapped BRAM “host memory window”) to validate the protocol and data movement
end-to-end on real hardware before moving to the final architecture (AXI/DDR
backend, full DMA, higher throughput).

[> Current status (WIP)
-----------------------

Hardware validated (end-to-end with a real NVMe SSD):

* PCIe link up (RootPort mode) and BAR0 discovery.
* NVMe BAR0 MMIO access (CAP/VS/CC/CSTS/AQA/ASQ/ACQ + doorbells).
* Admin queue init (AQA/ASQ/ACQ, CC.EN, CSTS.RDY).
* Admin Identify Controller (CNS=1) with decoded fields.
* Set Features: Number of Queues (FID=0x07).
* Create IO CQ/SQ (PC=1).
* Successful I/O Reads with DMA into the host memory window.
* Successful I/O Writes with readback verification.
* Bring-up validated both from Python scripts and from an integrated SoftCPU running firmware.

Bring-up components currently used:

* LitePCIe in RootPort mode.
* AXI-mapped BRAM host memory responder (stands in for system DRAM during bring-up).
* UARTBone-driven Python test scripts for reproducible sequences and debug.
* SoftCPU firmware in the target design to run Identify/Read/Write directly on hardware.

SoftCPU flow (current vs. target usage):

* **Current bring-up**: the VexRiscv runs a small test firmware that exposes
  manual NVMe commands over UART (Identify/Read/Write/Verify) for validation.
* **Target usage**: the SoftCPU will be driven by user logic via control/status
  registers (e.g. AXI-Lite), and will handle the internal NVMe sequencing
  without requiring a human-operated UART shell.

[> Validation steps (bring-up)
-------------------------------

1. Enable PCIe MEM/BME and assign/discover BAR0.
2. Read NVMe CAP/VS and sanity-check controller presence.
3. Program AQA/ASQ/ACQ and enable controller (CC.EN), wait for CSTS.RDY.
4. Submit Identify Controller and verify:

   * CQE status success.
   * Identify buffer DMA contents decoded correctly.
5. Set Features: Number of Queues.
6. Create IO CQ1 then IO SQ1.
7. Identify Namespace List (CNS=2) and select active NSID.
8. Issue Read (opcode 0x02) to PRP1 buffer and verify:

   * CQE status success.
   * Buffer changed from prefill pattern.
   * Expected on-disk signatures (GPT: `"EFI PART"` at LBA1, MBR sig 0xAA55).

[> Possible improvements
------------------------

* Replace BRAM host window with real system memory backend (AXI/DDR).
* Add a proper DMA path (PRP/SGL) and scale performance (multi-queue, deeper rings).
* Make MMIO/CFG accessors fully robust (sticky completion/clear semantics).
* Extend command coverage (Identify Namespace, larger reads/writes, admin log pages, etc.).
* Add higher-level software integration (driver/userspace tooling).

If you want to support these features, please contact us at florent [AT]
enjoy-digital.fr.

[> Getting started
------------------

1. Install Python 3.8+ and FPGA vendor's development tools.
2. Install LiteX and the cores by following the LiteX wiki installation guide.
3. Build a LiteNVMe RootPort test SoC for your platform and connect an NVMe SSD.
4. Run the bring-up scripts to execute the validation steps above.

[> Tests
--------

Unit tests are available in ./test/ and cover the BRAM-backed host memory responder
and NVMe request/response sequencing in simulation.

To run all unit tests:

```sh
$ pytest -v
```

Or run individually:

```sh
$ python3 -m unittest test.test_name
```

[> License
----------

LiteNVMe is released under the very permissive two-clause BSD license. Under
the terms of this license, you are authorized to use LiteNVMe for closed-source
proprietary designs.

Even though we do not require you to do so, those things are awesome, so please
do them if possible:

* tell us that you are using LiteNVMe
* cite LiteNVMe in publications related to research it has helped
* send feedback and suggestions for improvements
* send bug reports when something goes wrong
* send modifications and improvements you have done to LiteNVMe.

[> Support and consulting
-------------------------

LiteNVMe is developed and maintained by EnjoyDigital.

If you would like to know more about LiteNVMe or extend it for your needs,
EnjoyDigital can provide standard commercial support as well as consulting services.

So feel free to contact us, we'd love to work with you!

[> Contact
----------

E-mail: florent [AT] enjoy-digital.fr
