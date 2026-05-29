import sys, time
from litex import RemoteClient

op    = sys.argv[1]
lba   = int(sys.argv[2])
nlb   = int(sys.argv[3])
count = int(sys.argv[4])
outf  = sys.argv[5]

b = RemoteClient(csr_csv="build/alibaba_xcku3p/csr.csv"); b.open()

# drain crossover rx
while not b.regs.uart_xover_rxempty.read():
    b.regs.uart_xover_rxtx.read()

cmd = "nvme_engine_bench %s 0xe0000000 1 %d %d %d 16" % (op, lba, nlb, count)
for ch in cmd + "\r":
    while b.regs.uart_xover_txfull.read():
        pass
    b.regs.uart_xover_rxtx.write(ord(ch))

# Wait for the run to finish: gen done bit set AND completed reached count (or timeout).
t0 = time.time()
done = False
while time.time() - t0 < 75:
    st = b.regs.nvme_gen_status.read()
    comp = b.regs.nvme_gen_completed.read()
    if (st & 1) and comp >= count:
        done = True
        break
    time.sleep(0.05)

comp = b.regs.nvme_gen_completed.read()
err  = b.regs.nvme_gen_errors.read()
cyc  = b.regs.nvme_gen_cycles.read()
b.close()

payload = count * nlb * 512
tp = (payload * 125e6 / cyc / 1e6) if cyc else 0.0
percmd = (cyc // count) if count else 0
line = ("MARK_A op=%s lba=%d nlb=%d count=%d done=%s completed=%d errors=%d "
        "cycles=%d payload=%d tp_MBps=%.2f percmd_cyc=%d MARK_END\n" % (
        op, lba, nlb, count, done, comp, err, cyc, payload, tp, percmd))
open(outf, "w").write(line)
