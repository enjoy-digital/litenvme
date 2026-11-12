# Simulation and verification {#simulation}

This chapter describes how LiteNVMe is verified without hardware: the test suite, the models it
uses and what they cannot show, the checks of the continuous integration, and how to add a test
or look at waveforms. Read it before changing the gateware; [Getting started](#getting-started)
shows a first run of the suite.

## At a glance {#sim-glance}

The suite is in `test/`: 13 files, 46 test cases, all Migen simulations run with
`run_simulation`, plus the configuration checks of the generator. There is no Verilator
simulation and no SSD model beyond small Python processes.

| Test file | Block under test | Cases | Around it |
|------------------------------------|-------------------|-------|--------------------------|
| `test_cfg_accessor.py` | Configuration accessor | 3 | Port model: read, write, timeout |
| `test_mem_accessor.py` | MMIO accessor | 5 | Port model: read, multi-beat read, write, bad `wsel`, timeout |
| `test_hostmem.py` | Host-memory window | 7 | MemRd of an SQE with stalls; sub-beat writes |
| `test_nvme_sequence.py` | Host-memory window | 3 | Identify-like sequence of MemRd and MemWr, 64, 128 and 256 bits |
| `test_io_engine.py` | I/O engine | 3 | Dword memory, doorbell and SSD models; ring wrap |
| `test_io_engine_prp.py` | I/O engine, PRP | 5 | One, two, three and many pages, mixed sizes |
| `test_io_engine_axi.py` | Engine AXI bridge | 3 | Block RAM backend, 64, 128 and 256 bits |
| `test_io_engine_` `integration.py` | Engine, bridge, arbiter | 3 | Block RAM backend, SSD model on a second AXI master |
| `test_request_gen.py` | Request generator | 3 | Same path as above, driven by the generator |
| `test_block_streamer.py` | Block streamer | 4 | Write, read, error status, `done` timing |
| `test_init_sequencer.py` | Init sequencer | 1 | Configuration, MMIO, root and memory models, admin CQEs |
| `test_gen_config.py` | Generator configuration | 6 | YAML loading and checks, no elaboration |
| `models.py` | Shared models | none | Dword memory port, doorbell accessor |

`test/conftest.py` puts `test/` on the Python path so that the test files share `models.py`.

## Running the tests {#sim-run}

The suite runs from the root of the repository, as the continuous integration runs it:

```sh
python3 -m pytest -q test
# One file, or one case.
python3 -m pytest -q test/test_io_engine_prp.py
python3 -m pytest -q test/test_io_engine_prp.py -k list_many
```

The whole suite takes several minutes in one process, most of it in the engine and block streamer
simulations.

`pytest.ini` turns off the assertion rewriting of pytest (`addopts = --assert=plain`): Migen names
anonymous signals from the source line that creates them, and rewritten test modules can confuse
that tracer when several HDL test files are imported together. The tests are `unittest` test cases:
`python3 -m unittest discover -s test -t test` runs them too. A plain `python3 -m unittest` from the
root finds none, since `test/` is not a package.

## What the models cover {#sim-models}

Each test replaces the parts around the block under test by Python processes that follow the
protocol of the real neighbour. The real gateware is used wherever it is part of the data path:
the block RAM backend, the AXI arbiter and the bridge of the engine run in the bridge,
integration, request generator and block streamer tests.

| Model | Stands for | Used by |
|------------------------------|---------------------------------|---------------------------------------|
| Port model | LitePCIe crossbar port: requests out, completions in | Accessor and window tests |
| Dword memory (`make_mem_model`) | Host-memory window seen by the engine | `test_io_engine_prp.py` (`test_io_engine.py` has its own) |
| Doorbell model (`make_mmio_model`) | Doorbell accessor: records each doorbell value | `test_io_engine_prp.py`, `test_request_gen.py`; other tests inline their own |
| SSD model | Controller: reads the new SQE, writes a CQE with the right CID, phase and SQHD | Engine, generator, streamer, init tests |

The SSD models answer each command with success, unless the test asks for an error status. They
do not move the data of a Read or a Write; the block streamer tests check the staging region and
the output stream instead.

## Limits of the models {#sim-limits}

Some behaviours of the hardware do not appear in these simulations. Two of them caused failures
on the bench that the suite had passed, and the code carries the fix.

- **Torn CQE.** The SSD models never update a CQE while the engine reads it. A real SSD writes it
  as a TLP at any time, so the engine could read the old dwords of an entry and the new phase
  bit. The engine therefore reads dword 3 (phase, status, CID) first ([The I/O engine](#io-engine)).
- **Synthesis of variable shifts.** A shift by a signal value simulates correctly in Migen but
  was implemented as no shift on hardware in the sub-beat write path of the window. The window
  uses a `Case` of constant shifts instead ([The host-memory window](#host-memory)).
- **No PCIe link and no root port.** The accessors are tested on a port model, not on the
  LitePCIe crossbar; MPS, ordering and completion timeouts of a real link are not modelled.
- **No elaboration of the integrated core.** No test builds the `LiteNVMe` class or the generator
  SoC, so register naming and the wiring of `add_csrs()` are not checked by the suite. The
  generator is elaborated only by the dry run of the continuous integration
  ([Continuous integration](#sim-ci)) and by the register appendix
  ([Register reference](#registers)).

Hardware runs remain the reference for these points: the [status table](#intro-status) says which
features have run on a board.

## Continuous integration {#sim-ci}

`.github/workflows/ci.yml` runs on every push and pull request, on Ubuntu with Python 3.9, after
installing LiteX with `litex_setup.py`. Its steps are in order:

| Step | Check |
|------------------------------|----------------------------------------------------------|
| Package Discovery | `litenvme` is found by `setuptools` |
| Import Smoke Test | The core modules and the generator import |
| Generator Config Check | `examples/alibaba_xcku3p.yml` loads, `init_mode` is `firmware` |
| Generator Dry Run | The example in `rtl` mode, without a CPU, is generated (no RISC-V toolchain) |
| Collect Tests | pytest collects the suite |
| Check Register Appendix | The register snapshots and `A_registers.md` match the gateware |
| Run Tests | `python3 -m pytest -q test` |

The register step runs `doc/manual/tools/csr_doc.py dump --check` and `generate --check`; a
change of a register therefore needs the appendix to be regenerated
([Register appendix](#sim-registers)). The manual itself is checked by
`.github/workflows/doc.yml` ([Maintaining this manual](#maintenance)).

### Register appendix {#sim-registers}

[Register reference](#registers) is generated: never edit `A_registers.md` by hand.
`doc/manual/registers.py` says which design to elaborate (the generator on
`examples/alibaba_xcku3p.yml`, in its request generator variant) and which register banks to
document, with the descriptions the gateware lacks; `tools/csr_doc.py` does the rest. The
elaboration runs no FPGA toolchain and compiles no firmware.

```sh
# Elaborate the design and write the snapshot doc/manual/csr/xcku3p_request_gen.json.
python3 doc/manual/tools/csr_doc.py dump
# Write A_registers.md from the snapshot.
python3 doc/manual/tools/csr_doc.py generate
# What the continuous integration runs: fail if either is stale.
python3 doc/manual/tools/csr_doc.py dump --check
python3 doc/manual/tools/csr_doc.py generate --check
```

A description belongs in the gateware (`description=` of the `CSRStorage`, `CSRStatus` or
`CSRField`); `DESCRIPTIONS` of `registers.py` only completes the registers declared without one.

Commit the snapshot and the appendix with the change of the register. The snapshot also holds the
registers of LiteX and LitePCIe that the design uses, so a register change in LiteX or LitePCIe also
needs the snapshot regenerated.

## Adding a test {#sim-add}

A new test follows the files above: a `unittest.TestCase` in `test/`, a small DUT module that
instantiates the block, and generator processes that drive it and model its neighbours.

1. Build the DUT with the parameters under test; loop over the data widths 64, 128 and 256 when
   the block depends on them.
2. Reuse `make_mem_model` and `make_mmio_model` from `models.py` for the engine side; write the
   SSD model in the test, since each test captures different things.
3. Mark the model processes `@passive`, so that the simulation ends when the stimulus ends.
4. Record results in Python lists or dictionaries from the processes, then assert after
   `run_simulation` returns: a generator process cannot return a value.
5. Keep assertions plain (`assert` or the `unittest` methods); run the new file alone, then the
   whole suite.

A test that reproduces a hardware failure is the most useful kind: [Debugging](#debugging)
describes how to narrow a failure on the bench first.

## Waveforms {#sim-vcd}

Three tests write a VCD file in the current directory: `test_nvme_sequence.py`
(`nvme_sequence.vcd`), `test_io_engine.py` (`io_engine.vcd`) and `test_io_engine_axi.py`
(`io_engine_axi.vcd`).

The cases of a file share one name, so the file holds the last case run; run one case with `-k` to
keep its waveform. The other tests pass `vcd_name=None`; give a name there to record one. VCD files
are ignored by git (`.gitignore`).

```sh
python3 -m pytest -q test/test_io_engine.py -k wrap
gtkwave io_engine.vcd
```

In the engine waveform, the doorbell model and the SSD model are the passive processes; the
signals to look at first are the request and completion streams of the engine, its memory port
and its doorbell start and address.
