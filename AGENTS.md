# AGENTS.md: LiteNVMe

NVMe host core for LiteX: brings up an NVMe SSD from a LitePCIe root port (controller enable, Identify,
I/O queues) and exposes a block interface, with no host CPU or OS in the data path.

Read the LiteX agent page first: https://github.com/enjoy-digital/litex/wiki/LiteX-for-AI-Agents (framework map, conventions, how to
verify a change, contribute back). Index of the whole wiki for agents: https://github.com/enjoy-digital/litex/wiki/llms.txt.

## Where things are

- `litenvme/core.py`, `init.py`, `cfg.py`, `request_gen.py`, `io_engine.py`, `hostmem.py`, `block.py`, `mem.py`.
- `litenvme/gen.py` (`litenvme_gen`): standalone core; `bench/`: hardware benches (XCKU3P) and host tools; `doc/`; `test/`.

## Verify a change

```sh
python3 -m pytest -q test                                           # what CI runs
```

- Variable shifts/indexes that simulate fine can synthesize wrongly: prefer constant-shift `Case`
  structures in data paths, and check on hardware.
- Max payload size stays at the reset value unless both the root port and the device are
  programmed (root first).
- Report only measured numbers: one reader on the console, results valid only with a complete
  capture.

## Rules

- Style: https://github.com/enjoy-digital/litex/blob/master/doc/coding_style.md (match the
  surrounding file; `module: Description of the change.` commit titles; one change per commit).
- Never edit generated files (`build/`, `csr.h`, `csr.csv`, the Verilog): change the Python.
- One topic per pull request, from the default development branch, with what was tested and on
  which hardware; say that an agent wrote it and who supervised. Do not rename options or
  reformat unrelated code.
- Hardware: never flash a board or write its SPI flash unless the user asked; `--load` (volatile)
  is the default for experiments.

## Debug on hardware

Method: https://github.com/enjoy-digital/litex/wiki/Debug-A-SoC-On-Hardware (check the link and `--ident` first, registers before
traces, narrow LiteScope captures). `litex_mcp` gives the same loop as MCP tools (CSRs by name,
LiteScope captures, builds with parsed timing, simulation): https://github.com/enjoy-digital/litex/wiki/LiteX-for-AI-Agents#drive-a-soc-through-mcp.
