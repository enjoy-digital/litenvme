# Front-ends: block streamer and request generator {#frontends}

This chapter describes the two front-ends of the I/O engine and the third option, driving the
engine streams from your own logic. The block streamer gives user logic a sector interface with
data streams, like a disk. The request generator issues sequential commands for benchmarks.
Read it to choose a front-end and to drive it correctly.

## At a glance {#fe-glance}

| Front-end | Interface | Data path | Use |
|---------------------|---------------------------------|-------------------------------|----------------------|
| Block streamer | Sector, count; data streams | Copied through a staging region | Reading and writing data |
| Request generator | Registers: count, LBA, step | Stays in the window | Throughput measurements |
| Engine streams | Request and completion streams | Your buffers in the window | Custom designs |

`LiteNVMe` builds the block streamer with `with_block_streamer=True` and the request generator
with `with_request_gen=True`. [Front-end ownership](#arch-frontend) gives the rule that connects
one of them to the engine.

## Block streamer {#fe-streamer}

The I/O engine works with addresses: the SSD reads and writes data buffers in the host-memory
window. The block streamer (`LiteNVMeBlockStreamer`) hides this. It copies the data of a write
from a stream into a staging region of the window, and the data of a read from the staging region
into a stream, as the figure below shows.

![Block streamer. Look at the two paths: the command FSM exchanges one request and one completion
with the I/O engine per transfer, and the staging DMA copies the data between the user streams and
the staging region of the window.](fe_streamer.svg)

| Signal | Width | Meaning |
|-------------|-------|----------------------------------------------------------------|
| `start` | 1 | Starts a transfer when the streamer is idle |
| `write` | 1 | 1 for a write to the SSD, 0 for a read |
| `sector` | 64 | First sector |
| `count` | 32 | Number of sectors of 512 bytes |
| `nsid` | 32 | Namespace, 1 at reset |
| `done` | 1 | 1 when idle or finished, 0 during a transfer |
| `busy` | 1 | 1 during a transfer |
| `error` | 1 | The completion of the last transfer reported an error |
| `wr_sink` | data | Data of a write, one PCIe beat per word |
| `rd_source` | data, last | Data of a read, `last` on the final word |

The data streams are as wide as the window: at 256 bits, a sector is 16 words. `start` is
sampled while the streamer is idle, so a one-cycle pulse starts one transfer; a `start` held high
starts a new one as soon as the previous one ends.

### Write and read {#fe-sequence}

A write runs in three steps. The streamer copies `count` sectors from `wr_sink` into the staging
region, sends one Write request to the engine with the staging region as its buffer, and waits for
the completion. `done` rises after the completion; `error` is set if its SC or SCT is not 0.

A read runs the other way, as the figure below shows. The streamer sends one Read request to the
engine and waits for the completion, then copies the sectors from the staging region to
`rd_source`, with `last` on the final word. `done` rises when the copy ends.

![A block streamer read. Read it top to bottom: the SSD writes the data into the staging region
before it posts the CQE, and the streamer starts the copy to `rd_source` only after the
completion.](fe_timing.svg)

The copy and the NVMe command run one after the other: a write does not start before all its data
has arrived, and a read sends no data before the SSD has finished. The staging DMA moves one
single-beat AXI access at a time and waits for each to finish, so it needs several cycles per
word.

### Limits {#fe-limits}

- One NVMe command per transfer: the transfer must fit in the staging region. The hardware does
  not check that `count * 512` is at most `staging_size`.
- `count` from 1 to 65 535. The request to the engine carries the count in 16 bits, so a larger
  count is truncated; a count of 0 makes the staging DMA run through 2^32 words.
- `wr_sink` must deliver exactly `count` sectors of data. The streamer counts words and ignores
  `last` on `wr_sink`.
- The staging region must start on a 4 KiB boundary, for the PRP rules of the engine. A transfer
  of more than 8 KiB uses the PRP-list page of its slot, which must not lie inside the staging
  region; the generator checks it, the bench SoC does not. See [Window layouts](#mem-layout).

### Registers {#fe-streamer-registers}

With `with_csr=True` (the default), the block streamer gets registers instead of top-level signals.

| Register | Meaning |
|---------------------|-------------------------------------------------------------|
| `sector_lo`, `_hi` | First sector |
| `count` | Number of sectors, 1 at reset |
| `nsid` | Namespace, 1 at reset |
| `ctrl` | `start` (bit 0, pulse), `write` (bit 1) |
| `status` | `done` (bit 0), `busy` (bit 1), `error` (bit 2) |
| `cycles` | Cycles from `start` to `done` |

The data streams have no register access: in a SoC with registers, the bench SoC connects them to a
pattern generator and checker. In a `LiteNVMe` with registers, user logic must still connect both
streams; left unconnected, a transfer never ends. The standalone core exposes the control signals
and the two streams as top-level ports instead; see [Integration](#integration).

## Request generator {#fe-reqgen}

The request generator (`LiteNVMeRequestGen`) issues a series of commands as fast as the engine
takes them, with no CPU involved, and measures the time until the last completion. It moves no
data in or out of the window: it measures the SSD, the link and the engine.

For the command `k` of a run, counted from 0, the generator sends:

| Field | Value |
|--------|-----------------------------------------------------|
| `op` | `op` of the run (0 read, 1 write) |
| `nsid` | `nsid` of the run |
| `lba` | `base_lba + k * lba_step` |
| `nlb` | `nlb` of the run |
| `buf` | `buf_base + (k mod qmod) * buf_stride` |

With `qmod` equal to the queue size of the engine and `buf_stride` at least `nlb * 512`, each
command in flight gets its own buffer. The firmware benchmark keeps 4 KiB, so its 8 KiB commands
share buffer halves: fine for throughput, not for data checks.

The run ends when `count` completions have arrived. The generator counts the completions with an
error status and keeps the status, CID and index of the first one.

| Register | Meaning |
|---------------------|---------------------------------------------------------------|
| `op`, `nsid`, `nlb` | Command fields (`nlb` 1 at reset) |
| `base_lba_lo`, `_hi` | First sector |
| `lba_step` | Sectors between two commands |
| `count` | Commands of the run, 1 at reset |
| `buf_base_lo`, `_hi` | Bus address of the first buffer |
| `buf_stride` | Bytes between two buffers, `0x1000` at reset |
| `qmod` | Number of buffers, 64 at reset |
| `ctrl` | `start` (pulse) |
| `status` | `done` (bit 0) |
| `completed` | Completions of the run |
| `errors` | Completions with SC or SCT not 0 |
| `cycles` | Cycles from `start` to the last completion |
| `last_status` | Status field of the last completion |
| `first_err_status` | Status of the first failed completion |
| `first_err_cid` | CID of the first failed completion |
| `first_err_idx` | Index of the first failed completion in the run |

The throughput of a run is the number of bytes, `count * nlb * 512`, times the clock frequency,
divided by `cycles`. This is what the firmware command `nvme_engine_bench` prints; see
[Examples](#examples) and [Performance](#performance).

## Driving the engine from your own logic {#fe-custom}

Built with neither front-end, or with both, `LiteNVMe` exports the engine streams as `request` and
`completion`. User logic then follows the rules of the engine:

1. Place each data buffer in the host-memory window, on a 4 KiB boundary, and give each command in
   flight its own buffer.
2. For a write, put the data into the buffer before the request. The parameter
   `extra_axi_masters` adds your AXI masters to the arbiter of the window for this.
3. Send the request on `request`, then wait for its completion on `completion`. Match
   completions by CID: the CID of the `n`-th request since reset is `n` modulo `qsize`.
4. Check the status of each completion, and for a read, use the buffer only after it.

Keep `completion` ready: the engine waits on a completion that is not taken and submits nothing
meanwhile.

## In the code {#fe-code}

`LiteNVMeBlockStreamer` and its staging DMA `LiteNVMeHostMemStreamDMA` are in `litenvme/block.py`;
`LiteNVMeRequestGen` is in `litenvme/request_gen.py`. The front-end multiplexer and the data
pattern logic of the bench SoC are in `bench/alibaba_xcku3p.py`.

## Where to go next {#fe-next}

[Bring-up](#bringup) prepares the engine that the front-ends drive; [Integration](#integration)
connects a front-end in a SoC or a standalone core.
