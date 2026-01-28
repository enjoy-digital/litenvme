// This file is part of LiteNVMe.
//
// Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
// Developed with LLM assistance.
// SPDX-License-Identifier: BSD-2-Clause

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>

#include <irq.h>
#include <libbase/uart.h>
#include <libbase/console.h>
#include <generated/csr.h>

/*-----------------------------------------------------------------------*/
/* NVMe constants/layout                                                  */
/*-----------------------------------------------------------------------*/

#define HOSTMEM_BASE       0x10000000u
#define ASQ_ADDR           (HOSTMEM_BASE + 0x0000u)
#define ACQ_ADDR           (HOSTMEM_BASE + 0x1000u)
#define ID_BUF_ADDR        (HOSTMEM_BASE + 0x2000u)
#define ADMIN_Q_ENTRIES    2

#define NVME_CAP           0x0000u
#define NVME_VS            0x0008u
#define NVME_CC            0x0014u
#define NVME_CSTS          0x001cu
#define NVME_AQA           0x0024u
#define NVME_ASQ           0x0028u
#define NVME_ACQ           0x0030u

/*-----------------------------------------------------------------------*/
/* UART                                                                  */
/*-----------------------------------------------------------------------*/

static char *readstr(void)
{
	char c[2];
	static char s[64];
	static int ptr = 0;

	if (readchar_nonblock()) {
		c[0] = getchar();
		c[1] = 0;
		if ((unsigned char)c[0] == 0x00 || (unsigned char)c[0] == 0xff)
			return NULL;
		switch (c[0]) {
		case 0x7f:
		case 0x08:
			if (ptr > 0) {
				ptr--;
				fputs("\x08 \x08", stdout);
			}
			break;
		case 0x07:
			break;
		case '\r':
		case '\n':
			s[ptr] = 0x00;
			fputs("\n", stdout);
			ptr = 0;
			return s;
		default:
			if (ptr >= (int)(sizeof(s) - 1))
				break;
			fputs(c, stdout);
			s[ptr] = c[0];
			ptr++;
			break;
		}
	}

	return NULL;
}

static char *get_token(char **str)
{
	char *c, *d;

	c = (char *)strchr(*str, ' ');
	if (c == NULL) {
		d = *str;
		*str = *str + strlen(*str);
		return d;
	}
	*c = 0;
	d = *str;
	*str = c + 1;
	return d;
}

static void prompt(void)
{
	printf("\e[92;1mlitenvme\e[0m> ");
}

/*-----------------------------------------------------------------------*/
/* Help                                                                  */
/*-----------------------------------------------------------------------*/

static void help(void)
{
	puts("\nLiteNVMe soft-CPU firmware\n");
	puts("Available commands:");
	puts("help               - Show this command");
	puts("reboot             - Reboot CPU");
	puts("status             - Print PCIe link + hostmem counters");
	puts("bdf <b> <d> <f>    - Set PCIe BDF (bus/dev/fn)");
	puts("cfg_rd <reg>       - Read CFG dword (reg index)");
	puts("cfg_wr <reg> <v>   - Write CFG dword (reg index)");
	puts("cmd_enable         - Set Command.MEM + Command.BME");
	puts("cmd_disable        - Clear Command.MEM + Command.BME");
	puts("bar0 <addr>        - Set BAR0 base address");
	puts("bar0_rd <off>      - Read BAR0 dword at offset");
	puts("bar0_wr <off> <v>  - Write BAR0 dword at offset");
	puts("bar0_dump <len> [s]- Dump BAR0 space (bytes), optional stride");
	puts("mmio_rd <addr>     - MMIO read dword at absolute address");
	puts("mmio_wr <addr> <v> - MMIO write dword at absolute address");
	puts("mmio_dump <addr> <len> [s] - Dump MMIO space (bytes), optional stride");
	puts("bar0_info          - Read NVMe CAP/VS/CSTS at BAR0");
	puts("nvme_identify [cid]- Run Admin Identify (controller) and decode");
	puts("nvme_identify_auto [bar0] [cid] - Auto BAR0 assign + enable MEM/BME/INTx off + Identify");
	puts("todo               - Placeholder for NVMe bring-up");
}

/*-----------------------------------------------------------------------*/
/* Commands                                                              */
/*-----------------------------------------------------------------------*/

static void reboot_cmd(void)
{
	ctrl_reset_write(1);
}

static void status_cmd(void)
{
#ifdef CSR_PCIE_PHY_PHY_LINK_STATUS_ADDR
	printf("pcie_link: %08" PRIx32 "\n", (uint32_t)pcie_phy_phy_link_status_read());
#else
	printf("pcie_link: (no CSR)\n");
#endif
#ifdef CSR_HOSTMEM_CSR_DMA_WR_COUNT_ADDR
	printf("hostmem_wr_count: %08" PRIx32 "\n", (uint32_t)hostmem_csr_dma_wr_count_read());
	printf("hostmem_rd_count: %08" PRIx32 "\n", (uint32_t)hostmem_csr_dma_rd_count_read());
#else
	printf("hostmem_count: (no CSR)\n");
#endif
}

/*-----------------------------------------------------------------------*/
/* MMIO Helpers                                                          */
/*-----------------------------------------------------------------------*/

static uint64_t bar0_base = 0;

static void mmio_set_addr(uint64_t addr)
{
	mmio_mem_adr_l_write((uint32_t)(addr & 0xffffffff));
	mmio_mem_adr_h_write((uint32_t)((addr >> 32) & 0xffffffff));
}

static void mmio_start(int we, uint8_t wsel, uint16_t length)
{
	uint32_t ctrl = 0;
	ctrl |= 1 << 0;
	ctrl |= (we ? 1 : 0) << 1;
	ctrl |= (wsel & 0xf) << 4;
	ctrl |= (length & 0x3ff) << 8;
	mmio_mem_ctrl_write(ctrl);
	mmio_mem_ctrl_write(0);
}

static int mmio_wait_done(unsigned int timeout)
{
	while (timeout--) {
		uint32_t stat = mmio_mem_stat_read();
		if (stat & 0x1)
			return (stat >> 1) & 0x1;
	}
	return 1;
}

static int mmio_rd32(uint64_t addr, uint32_t *val)
{
	mmio_set_addr(addr);
	mmio_start(0, 0xf, 1);
	if (mmio_wait_done(1000000))
		return 1;
	*val = mmio_mem_rdata_read();
	return 0;
}

static int mmio_wr32(uint64_t addr, uint32_t val)
{
	mmio_set_addr(addr);
	mmio_mem_wdata_write(val);
	mmio_start(1, 0xf, 1);
	return mmio_wait_done(1000000);
}

static int mmio_rd64(uint64_t addr, uint64_t *val)
{
	uint32_t lo = 0, hi = 0;
	int err0 = mmio_rd32(addr + 0, &lo);
	int err1 = mmio_rd32(addr + 4, &hi);
	*val = ((uint64_t)hi << 32) | lo;
	return err0 | err1;
}

static int mmio_wr64(uint64_t addr, uint64_t val)
{
	uint32_t lo = (uint32_t)(val & 0xffffffffu);
	uint32_t hi = (uint32_t)((val >> 32) & 0xffffffffu);
	int err0 = mmio_wr32(addr + 0, lo);
	int err1 = mmio_wr32(addr + 4, hi);
	return err0 | err1;
}

static void bar0_cmd(char *str)
{
	if (str == NULL || strlen(str) == 0) {
		printf("bar0 = 0x%016" PRIx64 "\n", bar0_base);
		return;
	}
	bar0_base = strtoull(str, NULL, 0);
	printf("bar0 = 0x%016" PRIx64 "\n", bar0_base);
}

static void bar0_rd_cmd(char *str)
{
	uint32_t off = (uint32_t)strtoul(str, NULL, 0);
	uint32_t v = 0;
	if (mmio_rd32(bar0_base + off, &v))
		printf("ERR\n");
	else
		printf("0x%08" PRIx32 "\n", v);
}

static void bar0_wr_cmd(char *str)
{
	char *a = get_token(&str);
	char *b = get_token(&str);
	uint32_t off = (uint32_t)strtoul(a, NULL, 0);
	uint32_t v   = (uint32_t)strtoul(b, NULL, 0);
	if (mmio_wr32(bar0_base + off, v))
		printf("ERR\n");
}

static void bar0_dump_cmd(char *str)
{
	char *a = get_token(&str);
	char *b = get_token(&str);
	uint32_t len = (uint32_t)strtoul(a, NULL, 0);
	uint32_t stride = b ? (uint32_t)strtoul(b, NULL, 0) : 4;
	if (stride == 0)
		stride = 4;
	for (uint32_t off = 0; off < len; off += stride) {
		uint32_t v = 0;
		mmio_rd32(bar0_base + off, &v);
		printf("0x%08" PRIx32 ": 0x%08" PRIx32 "\n", off, v);
	}
}

static void mmio_rd_cmd(char *str)
{
	uint64_t addr = strtoull(str, NULL, 0);
	uint32_t v = 0;
	if (mmio_rd32(addr, &v))
		printf("ERR\n");
	else
		printf("0x%08" PRIx32 "\n", v);
}

static void mmio_wr_cmd(char *str)
{
	char *a = get_token(&str);
	char *b = get_token(&str);
	uint64_t addr = strtoull(a, NULL, 0);
	uint32_t v    = (uint32_t)strtoul(b, NULL, 0);
	if (mmio_wr32(addr, v))
		printf("ERR\n");
}

static void mmio_dump_cmd(char *str)
{
	char *a = get_token(&str);
	char *b = get_token(&str);
	char *c = get_token(&str);
	uint64_t addr = strtoull(a, NULL, 0);
	uint32_t len  = (uint32_t)strtoul(b, NULL, 0);
	uint32_t stride = c ? (uint32_t)strtoul(c, NULL, 0) : 4;
	if (stride == 0)
		stride = 4;
	for (uint32_t off = 0; off < len; off += stride) {
		uint32_t v = 0;
		mmio_rd32(addr + off, &v);
		printf("0x%08" PRIx32 ": 0x%08" PRIx32 "\n", off, v);
	}
}

static void bar0_info_cmd(void)
{
	uint32_t cap0 = 0, cap1 = 0, vs = 0, csts = 0;
	mmio_rd32(bar0_base + 0x0000, &cap0);
	mmio_rd32(bar0_base + 0x0004, &cap1);
	mmio_rd32(bar0_base + 0x0008, &vs);
	mmio_rd32(bar0_base + 0x001c, &csts);
	printf("CAP  = 0x%08" PRIx32 "%08" PRIx32 "\n", cap1, cap0);
	printf("VS   = 0x%08" PRIx32 "\n", vs);
	printf("CSTS = 0x%08" PRIx32 "\n", csts);
}

/*-----------------------------------------------------------------------*/
/* HostMem helpers (CSR debug port)                                       */
/*-----------------------------------------------------------------------*/

static void hostmem_set_adr(uint32_t dword_adr)
{
	hostmem_csr_csr_adr_write(dword_adr);
}

static void hostmem_wr32(uint32_t addr, uint32_t data)
{
	uint32_t dword = (addr - HOSTMEM_BASE) >> 2;
	hostmem_set_adr(dword);
	hostmem_csr_csr_wdata_write(data);
	hostmem_csr_csr_we_write(1);
	hostmem_csr_csr_we_write(0);
}

static uint32_t hostmem_rd32(uint32_t addr)
{
	uint32_t dword = (addr - HOSTMEM_BASE) >> 2;
	hostmem_set_adr(dword);
	return hostmem_csr_csr_rdata_read();
}

static void hostmem_fill(uint32_t addr, uint32_t length, uint32_t value)
{
	for (uint32_t off = 0; off < length; off += 4)
		hostmem_wr32(addr + off, value);
}

static void hostmem_read_dwords(uint32_t addr, uint32_t *dst, uint32_t count)
{
	for (uint32_t i = 0; i < count; i++)
		dst[i] = hostmem_rd32(addr + 4 * i);
}

/*-----------------------------------------------------------------------*/
/* CFG Helpers                                                           */
/*-----------------------------------------------------------------------*/

static uint8_t cfg_bus = 0;
static uint8_t cfg_dev = 0;
static uint8_t cfg_fun = 0;

static uint32_t cfg_bdf_pack(uint8_t bus, uint8_t dev, uint8_t fun, uint8_t reg, uint8_t ext)
{
	uint32_t v = 0;
	v |= ((uint32_t)bus & 0xff) << 0;
	v |= ((uint32_t)dev & 0x1f) << 8;
	v |= ((uint32_t)fun & 0x07) << 13;
	v |= ((uint32_t)reg & 0x3f) << 16;
	v |= ((uint32_t)ext & 0x07) << 22;
	return v;
}

static int cfg_wait_done(unsigned int timeout)
{
	while (timeout--) {
		uint32_t stat = cfg_cfg_stat_read();
		if (stat & 0x1)
			return (stat >> 1) & 0x1;
	}
	return 1;
}

static int cfg_rd32(uint8_t reg, uint32_t *val)
{
	cfg_cfg_bdf_write(cfg_bdf_pack(cfg_bus, cfg_dev, cfg_fun, reg, 0));
	cfg_cfg_ctrl_write(1);
	cfg_cfg_ctrl_write(0);
	if (cfg_wait_done(1000000))
		return 1;
	*val = cfg_cfg_rdata_read();
	return 0;
}

static int cfg_wr32(uint8_t reg, uint32_t val)
{
	cfg_cfg_bdf_write(cfg_bdf_pack(cfg_bus, cfg_dev, cfg_fun, reg, 0));
	cfg_cfg_wdata_write(val);
	cfg_cfg_ctrl_write(1 | (1 << 1));
	cfg_cfg_ctrl_write(0);
	return cfg_wait_done(1000000);
}

static void bdf_cmd(char *str)
{
	char *a = get_token(&str);
	char *b = get_token(&str);
	char *c = get_token(&str);
	if (a && b && c) {
		cfg_bus = (uint8_t)strtoul(a, NULL, 0);
		cfg_dev = (uint8_t)strtoul(b, NULL, 0);
		cfg_fun = (uint8_t)strtoul(c, NULL, 0);
	}
	printf("BDF = %u:%u:%u\n", cfg_bus, cfg_dev, cfg_fun);
}

static void cfg_rd_cmd(char *str)
{
	uint8_t reg = (uint8_t)strtoul(str, NULL, 0);
	uint32_t v = 0;
	if (cfg_rd32(reg, &v))
		printf("ERR\n");
	else
		printf("0x%08" PRIx32 "\n", v);
}

static void cfg_wr_cmd(char *str)
{
	char *a = get_token(&str);
	char *b = get_token(&str);
	uint8_t reg = (uint8_t)strtoul(a, NULL, 0);
	uint32_t v  = (uint32_t)strtoul(b, NULL, 0);
	if (cfg_wr32(reg, v))
		printf("ERR\n");
}

static void cmd_set_bits(int mem, int bme, int intdis)
{
	uint32_t v = 0;
	if (cfg_rd32(1, &v)) {
		printf("ERR\n");
		return;
	}
	uint32_t cmd = v & 0xffffu;
	if (mem >= 0)    cmd = (cmd & ~(1u << 1))  | ((mem ? 1u : 0u) << 1);
	if (bme >= 0)    cmd = (cmd & ~(1u << 2))  | ((bme ? 1u : 0u) << 2);
	if (intdis >= 0) cmd = (cmd & ~(1u << 10)) | ((intdis ? 1u : 0u) << 10);
	v = (v & 0xffff0000u) | (cmd & 0xffffu);
	if (cfg_wr32(1, v))
		printf("ERR\n");
}

static void cmd_enable_cmd(void)
{
	cmd_set_bits(1, 1, -1);
}

static void cmd_disable_cmd(void)
{
	cmd_set_bits(0, 0, -1);
}

/*-----------------------------------------------------------------------*/
/* NVMe helpers                                                          */
/*-----------------------------------------------------------------------*/

static uint32_t bit(uint32_t v, uint32_t n) { return (v >> n) & 1u; }

static uint32_t csts_rdy(uint32_t csts) { return bit(csts, 0); }
static uint32_t cc_en(uint32_t cc) { return bit(cc, 0); }

static uint32_t doorbell_stride_bytes(uint32_t dstrd)
{
	return (1u << dstrd) * 4u;
}

static uint64_t nvme_db_addr(uint64_t bar0, uint64_t cap, uint16_t qid, int is_cq)
{
	uint32_t dstrd = (uint32_t)((cap >> 32) & 0xf);
	uint32_t stride = doorbell_stride_bytes(dstrd);
	uint32_t off = 0x1000u + (qid * 2u + (is_cq ? 1u : 0u)) * stride;
	return bar0 + off;
}

static uint32_t cc_make_en(uint32_t iocqes, uint32_t iosqes, uint32_t mps)
{
	uint32_t cc = 0;
	cc |= (1u & 0x1u) << 0;
	cc |= (mps & 0xfu) << 7;
	cc |= (iosqes & 0xfu) << 16;
	cc |= (iocqes & 0xfu) << 20;
	return cc;
}

static uint64_t bar_size_from_mask(uint64_t mask)
{
	return (~mask + 1u) & 0xffffffffffffffffull;
}

static int nvme_bar0_assign(uint64_t base_addr)
{
	uint32_t bar0 = 0, bar1 = 0;
	if (cfg_rd32(4, &bar0)) {
		puts("ERR: CFG read BAR0 failed.");
		return 1;
	}
	if (bar0 & 1) {
		puts("ERR: BAR0 is I/O, unexpected for NVMe.");
		return 1;
	}
	uint32_t bar_type = (bar0 >> 1) & 0x3;
	int is_64 = (bar_type == 0x2);

	printf("BAR0 orig    = 0x%08" PRIx32 " (64b=%d)\n", bar0, is_64);

	cfg_wr32(4, 0xffffffff);
	if (cfg_rd32(4, &bar0)) {
		puts("ERR: CFG read BAR0 mask failed.");
		return 1;
	}
	uint64_t mask = (uint64_t)(bar0 & 0xfffffff0u);
	if (is_64) {
		cfg_wr32(5, 0xffffffff);
		if (cfg_rd32(5, &bar1)) {
			puts("ERR: CFG read BAR1 mask failed.");
			return 1;
		}
		mask |= ((uint64_t)bar1 << 32);
	}

	uint64_t size = bar_size_from_mask(mask);
	printf("BAR0 size    = 0x%llx\n", (unsigned long long)size);

	if (size == 0 || (size & (size - 1)) != 0) {
		puts("ERR: BAR size invalid.");
		return 1;
	}
	if (base_addr & (size - 1)) {
		puts("ERR: BAR0 base not aligned to size.");
		return 1;
	}

	cfg_wr32(4, (uint32_t)(base_addr & 0xffffffffu) | (bar0 & 0xfu));
	if (is_64)
		cfg_wr32(5, (uint32_t)((base_addr >> 32) & 0xffffffffu));

	cfg_rd32(4, &bar0);
	if (is_64) {
		cfg_rd32(5, &bar1);
		printf("BAR0/BAR1    = 0x%08" PRIx32 " / 0x%08" PRIx32 "\n", bar0, bar1);
	} else {
		printf("BAR0         = 0x%08" PRIx32 "\n", bar0);
	}

	bar0_base = base_addr;
	return 0;
}

static int nvme_wait_rdy(uint32_t want_rdy, uint32_t loops)
{
	while (loops--) {
		uint32_t csts = 0;
		if (!mmio_rd32((uint64_t)bar0_base + NVME_CSTS, &csts)) {
			if (csts_rdy(csts) == want_rdy)
				return 1;
		}
	}
	return 0;
}

static void nvme_cmd_identify_controller(uint16_t cid, uint64_t prp1, uint32_t *cmd)
{
	for (int i = 0; i < 16; i++)
		cmd[i] = 0;
	cmd[0]  = (0x06u & 0xffu) | (((uint32_t)cid & 0xffffu) << 16);
	cmd[1]  = 0;
	cmd[6]  = (uint32_t)(prp1 & 0xffffffffu);
	cmd[7]  = (uint32_t)((prp1 >> 32) & 0xffffffffu);
	cmd[10] = 0x00000001u;
}

static void decode_cqe(uint32_t d0, uint32_t d1, uint32_t d2, uint32_t d3)
{
	uint32_t sq_head = d2 & 0xffffu;
	uint32_t sq_id   = (d2 >> 16) & 0xffffu;
	uint32_t cid     = d3 & 0xffffu;
	uint32_t sts     = (d3 >> 16) & 0xffffu;
	uint32_t p       = sts & 0x1u;
	uint32_t sc      = (sts >> 1) & 0xffu;
	uint32_t sct     = (sts >> 9) & 0x7u;
	uint32_t m       = (sts >> 14) & 0x1u;
	uint32_t dnr     = (sts >> 15) & 0x1u;
	printf("CQE:\n");
	printf("  SQ Head : %" PRIu32 "\n", sq_head);
	printf("  SQ ID   : %" PRIu32 "\n", sq_id);
	printf("  CID     : %" PRIu32 "\n", cid);
	printf("  Status  : P=%" PRIu32 " SCT=%" PRIu32 " SC=%" PRIu32 " M=%" PRIu32 " DNR=%" PRIu32 "\n",
	       p, sct, sc, m, dnr);
	printf("  DW0/DW1 : 0x%08" PRIx32 " 0x%08" PRIx32 "\n", d0, d1);
}

static void identify_decode(const uint32_t *dws, uint32_t dword_count)
{
	uint8_t b[256];
	uint32_t bytes = dword_count * 4;
	if (bytes > sizeof(b))
		bytes = sizeof(b);
	for (uint32_t i = 0; i < bytes / 4; i++) {
		uint32_t v = dws[i];
		b[i * 4 + 0] = (uint8_t)(v & 0xff);
		b[i * 4 + 1] = (uint8_t)((v >> 8) & 0xff);
		b[i * 4 + 2] = (uint8_t)((v >> 16) & 0xff);
		b[i * 4 + 3] = (uint8_t)((v >> 24) & 0xff);
	}

	uint16_t vid   = (uint16_t)(b[0] | (b[1] << 8));
	uint16_t ssvid = (uint16_t)(b[2] | (b[3] << 8));
	uint16_t cntlid = (uint16_t)(b[0x4e] | (b[0x4f] << 8));
	uint32_t ver = (uint32_t)(b[0x50] | (b[0x51] << 8) | (b[0x52] << 16) | (b[0x53] << 24));
	uint32_t oaes = (uint32_t)(b[0x58] | (b[0x59] << 8) | (b[0x5a] << 16) | (b[0x5b] << 24));

	char sn[21];
	char mn[41];
	char fr[9];
	memcpy(sn, &b[0x04], 20); sn[20] = 0;
	memcpy(mn, &b[0x18], 40); mn[40] = 0;
	memcpy(fr, &b[0x40], 8);  fr[8]  = 0;

	printf("Identify Controller (decoded):\n");
	printf("  VID      : 0x%04" PRIx16 "\n", vid);
	printf("  SSVID    : 0x%04" PRIx16 "\n", ssvid);
	printf("  SN       : %s\n", sn);
	printf("  MN       : %s\n", mn);
	printf("  FR       : %s\n", fr);
	printf("  VER      : %" PRIu32 ".%" PRIu32 ".%" PRIu32 " (0x%08" PRIx32 ")\n",
	       (ver >> 16) & 0xffffu, (ver >> 8) & 0xffu, ver & 0xffu, ver);
	printf("  CNTLID   : 0x%04" PRIx16 "\n", cntlid);
	printf("  IEEE OUI : %02x-%02x-%02x\n", b[0x49], b[0x4a], b[0x4b]);
	printf("  RAB      : %u\n", b[0x48]);
	printf("  CMIC     : 0x%02x\n", b[0x4c]);
	printf("  MDTS     : %u\n", b[0x4d]);
	printf("  OAES     : 0x%08" PRIx32 "\n", oaes);
}

static void nvme_identify_run(uint16_t cid)
{
	if (bar0_base == 0) {
		puts("ERR: BAR0 base is 0. Use bar0 <addr> first.");
		return;
	}

	cmd_enable_cmd();

	uint64_t cap = 0;
	uint32_t cc = 0;
	mmio_rd64((uint64_t)bar0_base + NVME_CAP, &cap);
	mmio_rd32((uint64_t)bar0_base + NVME_CC, &cc);
	if (cc_en(cc)) {
		mmio_wr32((uint64_t)bar0_base + NVME_CC, cc & ~1u);
		nvme_wait_rdy(0, 1000000);
	}

	uint32_t aqa = ((ADMIN_Q_ENTRIES - 1) & 0xfffu) | (((ADMIN_Q_ENTRIES - 1) & 0xfffu) << 16);
	mmio_wr32((uint64_t)bar0_base + NVME_AQA, aqa);
	mmio_wr64((uint64_t)bar0_base + NVME_ASQ, (uint64_t)ASQ_ADDR);
	mmio_wr64((uint64_t)bar0_base + NVME_ACQ, (uint64_t)ACQ_ADDR);

	cc = cc_make_en(4, 6, 0);
	mmio_wr32((uint64_t)bar0_base + NVME_CC, cc);

	if (!nvme_wait_rdy(1, 2000000)) {
		puts("WARN: CSTS.RDY did not assert.");
	}

	hostmem_fill(ACQ_ADDR, 16, 0);
	hostmem_fill(ID_BUF_ADDR, 0x100, 0);

	uint32_t cmd[16];
	nvme_cmd_identify_controller(cid, (uint64_t)ID_BUF_ADDR, cmd);
	for (int i = 0; i < 16; i++)
		hostmem_wr32(ASQ_ADDR + i * 4, cmd[i]);

	uint64_t db = nvme_db_addr((uint64_t)bar0_base, cap, 0, 0);
	mmio_wr32(db, 1);

	uint32_t d0 = 0, d1 = 0, d2 = 0, d3 = 0;
	for (uint32_t loops = 0; loops < 2000000; loops++) {
		d0 = hostmem_rd32(ACQ_ADDR + 0x0);
		d1 = hostmem_rd32(ACQ_ADDR + 0x4);
		d2 = hostmem_rd32(ACQ_ADDR + 0x8);
		d3 = hostmem_rd32(ACQ_ADDR + 0xc);
		if (((d3 >> 16) & 0x1) == 1)
			break;
	}

	printf("ACQ[0] raw: %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 "\n", d0, d1, d2, d3);
	decode_cqe(d0, d1, d2, d3);

	uint64_t db_cq = nvme_db_addr((uint64_t)bar0_base, cap, 0, 1);
	mmio_wr32(db_cq, 1);

	uint32_t id_dws[64];
	hostmem_read_dwords(ID_BUF_ADDR, id_dws, 64);
	identify_decode(id_dws, 64);
}

static void nvme_identify_cmd(char *str)
{
	uint16_t cid = 1;
	if (str && strlen(str))
		cid = (uint16_t)strtoul(str, NULL, 0);
	nvme_identify_run(cid);
}

static void nvme_identify_auto_cmd(char *str)
{
	uint64_t base_addr = 0xe0000000ull;
	uint16_t cid = 1;
	if (str && strlen(str)) {
		char *a = get_token(&str);
		char *b = get_token(&str);
		if (a && strlen(a))
			base_addr = strtoull(a, NULL, 0);
		if (b && strlen(b))
			cid = (uint16_t)strtoul(b, NULL, 0);
	}

	if (nvme_bar0_assign(base_addr))
		return;

	cmd_set_bits(1, 1, 1);
	nvme_identify_run(cid);
}

static void todo_cmd(void)
{
	puts("TODO: implement NVMe bring-up + IO commands in firmware.");
}

/*-----------------------------------------------------------------------*/
/* Console service / Main                                                */
/*-----------------------------------------------------------------------*/

static void console_service(void)
{
	char *str;
	char *token;

	str = readstr();
	if (str == NULL)
		return;
	token = get_token(&str);
	if (strcmp(token, "help") == 0)
		help();
	else if (strcmp(token, "reboot") == 0)
		reboot_cmd();
	else if (strcmp(token, "status") == 0)
		status_cmd();
	else if (strcmp(token, "bdf") == 0)
		bdf_cmd(str);
	else if (strcmp(token, "cfg_rd") == 0)
		cfg_rd_cmd(str);
	else if (strcmp(token, "cfg_wr") == 0)
		cfg_wr_cmd(str);
	else if (strcmp(token, "cmd_enable") == 0)
		cmd_enable_cmd();
	else if (strcmp(token, "cmd_disable") == 0)
		cmd_disable_cmd();
	else if (strcmp(token, "bar0") == 0)
		bar0_cmd(str);
	else if (strcmp(token, "bar0_rd") == 0)
		bar0_rd_cmd(str);
	else if (strcmp(token, "bar0_wr") == 0)
		bar0_wr_cmd(str);
	else if (strcmp(token, "bar0_dump") == 0)
		bar0_dump_cmd(str);
	else if (strcmp(token, "mmio_rd") == 0)
		mmio_rd_cmd(str);
	else if (strcmp(token, "mmio_wr") == 0)
		mmio_wr_cmd(str);
	else if (strcmp(token, "mmio_dump") == 0)
		mmio_dump_cmd(str);
	else if (strcmp(token, "bar0_info") == 0)
		bar0_info_cmd();
	else if (strcmp(token, "nvme_identify") == 0)
		nvme_identify_cmd(str);
	else if (strcmp(token, "nvme_identify_auto") == 0)
		nvme_identify_auto_cmd(str);
	else if (strcmp(token, "todo") == 0)
		todo_cmd();
	prompt();
}

int main(void)
{
#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setmask(0);
	irq_setie(1);
#endif
	uart_init();

	help();
	prompt();

	while (1) {
		console_service();
	}

	return 0;
}
