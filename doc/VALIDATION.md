# LiteNVMe Validation

This page is the curated validation entry point. Raw bring-up notes and intermediate debug
captures are preserved under `doc/archive/` and `bench/results/archive/`, but the evidence below is
the current reader-facing baseline.

## Simulation

Run:

```sh
python3 -m pytest -q
```

Current expected result:

```text
46 passed
```

The suite covers the PCIe CFG/MMIO accessors, host-memory responder, I/O engine, PRP-list
generation, request generator, RTL init sequencer, block streamer and end-to-end protocol models.

## Generator Checks

The standalone generator has two validated bring-up modes:

| Mode | Config | Expected path |
|------|--------|---------------|
| Firmware | `init_mode: "firmware"` | Embedded CPU runs bring-up firmware and raises `status_init_done`. |
| RTL | `init_mode: "rtl"` | CPU-less init sequencer auto-starts after PCIe PERST# release and raises `status_init_done`. |

Smoke commands:

```sh
python3 -c 'from litenvme.gen import load_config; load_config("examples/alibaba_xcku3p.yml")'
litenvme_gen examples/alibaba_xcku3p.yml --output-dir build --name litenvme_core
```

For RTL-init generator checks, copy the example YAML, set `init_mode` to `"rtl"`, keep
`firmware` at `"none"`, remove the CPU fields or set `cpu` to `None`, and keep the block streamer
as the exposed front-end.

## Hardware Baseline

Reference setup:

- Alibaba KU3P (`xcku3p-ffvb676-2-e`).
- Crucial CT500P310SSD8 NVMe SSD.
- PCIe Gen3 x4, 256-bit datapath at 125 MHz.
- I/O engine QD32.

Curated result files:

| File | Contents |
|------|----------|
| `bench/results/gen3_256b_2026-06-01/SUMMARY.md` | Main hardware summary. |
| `bench/results/gen3_256b_2026-06-01/throughput_8kib_default.log` | Default 8 KiB throughput run. |
| `bench/results/gen3_256b_2026-06-01/integrity.log` | Write/read-back integrity proof. |
| `bench/results/gen3_256b_2026-06-01/nvme_init.log` | NVMe initialization log. |

Current headline numbers:

| Workload | Result |
|----------|--------|
| 8 KiB reads, QD32 | ~2.69 GB/s, errors = 0 |
| 8 KiB writes, QD32 | ~2.74 GB/s, errors = 0 |
| Integrity | write -> read-back exact |

## Archive Policy

Keep final, reproducible evidence in the main docs and `bench/results/gen3_256b_2026-06-01/`.
Move exploratory captures, failed hypotheses and long development logs to archive paths. This
preserves traceability without making users and evaluators read through bring-up history first.
