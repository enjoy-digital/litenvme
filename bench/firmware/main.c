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
/* CSR feature flags                                                      */
/*-----------------------------------------------------------------------*/

#ifdef CSR_PCIE_MMIO_MEM_CTRL_ADDR
#define PCIE_MMIO_AVAILABLE 1
#else
#define PCIE_MMIO_AVAILABLE 0
#endif

/*-----------------------------------------------------------------------*/
/* NVMe constants/layout                                                  */
/*-----------------------------------------------------------------------*/

#define HOSTMEM_BASE       0x10000000u
#define ASQ_ADDR           (HOSTMEM_BASE + 0x0000u)
#define ACQ_ADDR           (HOSTMEM_BASE + 0x1000u)
#define ID_BUF_ADDR        (HOSTMEM_BASE + 0x2000u)
#define IO_CQ_ADDR         (HOSTMEM_BASE + 0x3000u)
#define IO_SQ_ADDR         (HOSTMEM_BASE + 0x4000u)
#define IO_RD_BUF_ADDR     (HOSTMEM_BASE + 0x5000u)
#define IO_WR_BUF_ADDR     (HOSTMEM_BASE + 0x6000u)
#define ADMIN_Q_ENTRIES    2
#define IO_Q_ENTRIES       4

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
	puts("cfg_rd <reg>       - Read CFG dword (reg index)");
	puts("cfg_wr <reg> <v>   - Write CFG dword (reg index)");
	puts("cmd_enable         - Set Command.MEM + Command.BME");
	puts("cmd_disable        - Clear Command.MEM + Command.BME");
	puts("mmio_rd <addr>     - MMIO read dword at absolute address");
	puts("mmio_wr <addr> <v> - MMIO write dword at absolute address");
	puts("mmio_dump <addr> <len> [s] - Dump MMIO space (bytes), optional stride");
	puts("nvme_identify [bar0] [cid] - Auto BAR0 assign + enable MEM/BME/INTx off + Identify");
	puts("nvme_read [bar0] [nsid] [slba] [nlb]  - Read NLB blocks into hostmem");
	puts("nvme_write [bar0] [nsid] [slba] [nlb] - Write NLB blocks from hostmem");
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
static int mmio_last_err = 0;

static void delay_cycles(unsigned int cycles)
{
	volatile unsigned int i;
	for (i = 0; i < cycles; i++)
		__asm__ __volatile__("");
}

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

static int mmio_wait_done(unsigned int timeout, uint32_t *stat_out)
{
	while (timeout--) {
		uint32_t stat = mmio_mem_stat_read();
		if (stat & 0x1) {
			if (stat_out)
				*stat_out = stat;
			mmio_last_err = (stat >> 1) & 0x1;
			return 0;
		}
	}
	mmio_last_err = 1;
	return 1;
}

static int mmio_rd32(uint64_t addr, uint32_t *val)
{
	uint32_t stat = 0;
	mmio_set_addr(addr);
	mmio_start(0, 0xf, 1);
	if (mmio_wait_done(1000000, &stat))
		return 1;
	*val = mmio_mem_rdata_read();
	return 0;
}

static int mmio_wr32(uint64_t addr, uint32_t val)
{
	uint32_t stat = 0;
	mmio_set_addr(addr);
	mmio_mem_wdata_write(val);
	mmio_start(1, 0xf, 1);
	if (mmio_wait_done(1000000, &stat))
		return 1;
	return 0;
}

static int mmio_rd64(uint64_t addr, uint64_t *val)
{
	uint32_t lo = 0, hi = 0;
	int err0 = mmio_rd32(addr + 0, &lo);
	int mmio_err0 = mmio_last_err;
	if (err0)
		return 1;
	int err1 = mmio_rd32(addr + 4, &hi);
	int mmio_err1 = mmio_last_err;
	if (err1)
		return 1;
	*val = ((uint64_t)hi << 32) | lo;
	mmio_last_err = mmio_err0 | mmio_err1;
	return 0;
}

static int mmio_wr64(uint64_t addr, uint64_t val)
{
	uint32_t lo = (uint32_t)(val & 0xffffffffu);
	uint32_t hi = (uint32_t)((val >> 32) & 0xffffffffu);
	int err0 = mmio_wr32(addr + 0, lo);
	int mmio_err0 = mmio_last_err;
	if (err0)
		return 1;
	int err1 = mmio_wr32(addr + 4, hi);
	int mmio_err1 = mmio_last_err;
	if (err1)
		return 1;
	mmio_last_err = mmio_err0 | mmio_err1;
	return 0;
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

static const uint8_t cfg_bus = 0;
static const uint8_t cfg_dev = 1;
static const uint8_t cfg_fun = 0;
static int cfg_last_err = 0;

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

static void cfg_ctrl_write(uint32_t v)
{
	cfg_cfg_ctrl_write(v);
}

static void cfg_bdf_write(uint32_t v)
{
	cfg_cfg_bdf_write(v);
}

static void cfg_wdata_write(uint32_t v)
{
	cfg_cfg_wdata_write(v);
}

static uint32_t cfg_stat_read(void)
{
	return cfg_cfg_stat_read();
}

static uint32_t cfg_rdata_read(void)
{
	return cfg_cfg_rdata_read();
}

static int cfg_wait_done(unsigned int timeout)
{
	while (timeout--) {
		uint32_t stat = cfg_stat_read();
		if (stat & 0x1)
			{
				cfg_last_err = (stat >> 1) & 0x1;
				return 0;
			}
	}
	cfg_last_err = 1;
	return 1;
}

static int cfg_rd32(uint8_t reg, uint32_t *val)
{
	cfg_bdf_write(cfg_bdf_pack(cfg_bus, cfg_dev, cfg_fun, reg, 0));
	cfg_ctrl_write(1);
	cfg_ctrl_write(0);
	if (cfg_wait_done(1000000))
		return 1;
	if (cfg_last_err)
		return 1;
	*val = cfg_rdata_read();
	return 0;
}

static int cfg_wr32(uint8_t reg, uint32_t val)
{
	cfg_bdf_write(cfg_bdf_pack(cfg_bus, cfg_dev, cfg_fun, reg, 0));
	cfg_wdata_write(val);
	cfg_ctrl_write(1 | (1 << 1));
	cfg_ctrl_write(0);
	return cfg_wait_done(1000000);
}

static void cfg_rd_cmd(char *str)
{
	uint8_t reg = (uint8_t)strtoul(str, NULL, 0);
	uint32_t v = 0;
	if (cfg_rd32(reg, &v))
		printf("ERR\n");
	else
		printf("0x%08" PRIx32 "\n", v);
	if (cfg_last_err)
		puts("WARN: cfg_rd err=1 (read may still be valid).");
}

static void cfg_wr_cmd(char *str)
{
	char *a = get_token(&str);
	char *b = get_token(&str);
	uint8_t reg = (uint8_t)strtoul(a, NULL, 0);
	uint32_t v  = (uint32_t)strtoul(b, NULL, 0);
	if (cfg_wr32(reg, v))
		printf("WARN: cfg_wr err=1 (write may still be accepted)\n");
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
		printf("WARN: cfg_wr err=1 (write may still be accepted)\n");
}

static void cmd_print_cmdsts(void)
{
	uint32_t v = 0;
	if (cfg_rd32(1, &v)) {
		puts("ERR: CFG read CMD/STS failed.");
		return;
	}
	printf("CMD/STS = 0x%08" PRIx32 "\n", v);
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
static uint32_t cqe_ok(uint32_t d3)
{
	uint32_t sts = (d3 >> 16) & 0xffffu;
	uint32_t sc  = (sts >> 1) & 0xffu;
	uint32_t sct = (sts >> 9) & 0x7u;
	return (sct == 0) && (sc == 0);
}

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

static void nvme_cmd_set_features_num_queues(uint16_t cid, uint16_t nsqr, uint16_t ncqr, uint32_t *cmd)
{
	for (int i = 0; i < 16; i++)
		cmd[i] = 0;
	cmd[0]  = (0x09u & 0xffu) | (((uint32_t)cid & 0xffffu) << 16);
	uint32_t cdw10 = 0x07;
	cdw10 |= ((nsqr - 1) & 0xffffu) << 0;
	cdw10 |= ((ncqr - 1) & 0xffffu) << 16;
	cmd[10] = cdw10;
}

static void nvme_cmd_create_iocq(uint16_t cid, uint64_t prp1_cq, uint16_t qid,
                                 uint16_t qsize_minus1, uint16_t iv, uint16_t ien, uint16_t pc,
                                 uint32_t *cmd)
{
	for (int i = 0; i < 16; i++)
		cmd[i] = 0;
	cmd[0]  = (0x05u & 0xffu) | (((uint32_t)cid & 0xffffu) << 16);
	cmd[6]  = (uint32_t)(prp1_cq & 0xffffffffu);
	cmd[7]  = (uint32_t)((prp1_cq >> 32) & 0xffffffffu);
	cmd[10] = (qid & 0xffffu) | ((qsize_minus1 & 0xffffu) << 16);
	uint32_t cq_flags = ((pc & 0x1u) << 0) | ((ien & 0x1u) << 1);
	cmd[11] = ((iv & 0xffffu) << 16) | (cq_flags & 0xffffu);
}

static void nvme_cmd_create_iosq(uint16_t cid, uint64_t prp1_sq, uint16_t qid,
                                 uint16_t qsize_minus1, uint16_t cqid, uint16_t qprio, uint16_t pc,
                                 uint32_t *cmd)
{
	for (int i = 0; i < 16; i++)
		cmd[i] = 0;
	cmd[0]  = (0x01u & 0xffu) | (((uint32_t)cid & 0xffffu) << 16);
	cmd[6]  = (uint32_t)(prp1_sq & 0xffffffffu);
	cmd[7]  = (uint32_t)((prp1_sq >> 32) & 0xffffffffu);
	cmd[10] = (qid & 0xffffu) | ((qsize_minus1 & 0xffffu) << 16);
	uint32_t sq_flags = ((pc & 0x1u) << 0) | ((qprio & 0x3u) << 1);
	cmd[11] = ((cqid & 0xffffu) << 16) | (sq_flags & 0xffffu);
}

static void nvme_cmd_read(uint16_t cid, uint32_t nsid, uint64_t prp1_data, uint64_t slba, uint16_t nlb_minus1, uint32_t *cmd)
{
	for (int i = 0; i < 16; i++)
		cmd[i] = 0;
	cmd[0]  = (0x02u & 0xffu) | (((uint32_t)cid & 0xffffu) << 16);
	cmd[1]  = nsid;
	cmd[6]  = (uint32_t)(prp1_data & 0xffffffffu);
	cmd[7]  = (uint32_t)((prp1_data >> 32) & 0xffffffffu);
	cmd[10] = (uint32_t)(slba & 0xffffffffu);
	cmd[11] = (uint32_t)((slba >> 32) & 0xffffffffu);
	cmd[12] = (nlb_minus1 & 0xffffu);
}

static void nvme_cmd_write(uint16_t cid, uint32_t nsid, uint64_t prp1_data, uint64_t slba, uint16_t nlb_minus1, uint32_t *cmd)
{
	for (int i = 0; i < 16; i++)
		cmd[i] = 0;
	cmd[0]  = (0x01u & 0xffu) | (((uint32_t)cid & 0xffffu) << 16);
	cmd[1]  = nsid;
	cmd[6]  = (uint32_t)(prp1_data & 0xffffffffu);
	cmd[7]  = (uint32_t)((prp1_data >> 32) & 0xffffffffu);
	cmd[10] = (uint32_t)(slba & 0xffffffffu);
	cmd[11] = (uint32_t)((slba >> 32) & 0xffffffffu);
	cmd[12] = (nlb_minus1 & 0xffffu);
}

static uint64_t bar_size_from_mask(uint64_t mask)
{
	return (~mask + 1u) & 0xffffffffffffffffull;
}

static int nvme_bar0_assign(uint64_t base_addr)
{
	uint32_t bar0 = 0, bar1 = 0;
	uint32_t bar0_orig = 0;
	uint32_t id = 0;

	if (cfg_rd32(0, &id)) {
		puts("ERR: CFG read ID failed.");
		return 1;
	}
	if (id == 0xffffffffu || id == 0x00000000u) {
		puts("ERR: No device at current BDF (read VID/DID invalid).");
		return 1;
	}
	printf("VID/DID      = 0x%08" PRIx32 "\n", id);

	if (cfg_rd32(4, &bar0_orig)) {
		puts("ERR: CFG read BAR0 failed.");
		return 1;
	}
	if (bar0_orig & 1) {
		puts("ERR: BAR0 is I/O, unexpected for NVMe.");
		return 1;
	}
	uint32_t bar_type = (bar0_orig >> 1) & 0x3;
	int is_64 = (bar_type == 0x2);

	printf("BAR0 orig    = 0x%08" PRIx32 " (64b=%d)\n", bar0_orig, is_64);

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

	cfg_wr32(4, (uint32_t)(base_addr & 0xffffffffu) | (bar0_orig & 0xfu));
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
		if (!mmio_rd32(bar0_base + NVME_CSTS, &csts)) {
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

static uint16_t admin_sq_tail = 0;
static uint16_t admin_cq_head = 0;
static uint16_t admin_cq_phase = 1;
static uint16_t io_sq_tail = 0;
static uint16_t io_cq_head = 0;
static uint16_t io_cq_phase = 1;

static void nvme_admin_reset_queues(void);
static void nvme_io_reset_queues(void);
static int nvme_admin_submit(uint64_t cap, const uint32_t *cmd, uint32_t *cqe);
static int nvme_io_submit(uint64_t cap, const uint32_t *cmd, uint32_t *cqe);


static int nvme_admin_init(uint64_t *cap_out)
{
	if (bar0_base == 0) {
		puts("ERR: BAR0 base is 0. Use bar0 <addr> first.");
		return 1;
	}
	cmd_enable_cmd();

	uint64_t cap = 0;
	uint32_t cc = 0;
	for (int tries = 0; tries < 5; tries++) {
		if (mmio_rd64(bar0_base + NVME_CAP, &cap)) {
			puts("ERR: CAP read timeout.");
		} else if (mmio_last_err) {
			puts("WARN: CAP read err=1 (read may still be valid).");
		}
		if (mmio_rd32(bar0_base + NVME_CC, &cc)) {
			puts("ERR: CC read timeout.");
		} else if (mmio_last_err) {
			puts("WARN: CC read err=1 (read may still be valid).");
		}
		if (cap != 0)
			break;
		delay_cycles(50000);
	}
	if (cap == 0) {
		puts("ERR: CAP is 0 (BAR0 not responding).");
		return 1;
	}
	if (cc_en(cc)) {
		if (mmio_wr32(bar0_base + NVME_CC, cc & ~1u))
			puts("ERR: CC write timeout.");
		else if (mmio_last_err)
			puts("WARN: CC write err=1 (write may still be accepted).");
		if (!nvme_wait_rdy(0, 1000000))
			puts("WARN: CSTS.RDY did not clear.");
	}

	uint32_t aqa = ((ADMIN_Q_ENTRIES - 1) & 0xfffu) | (((ADMIN_Q_ENTRIES - 1) & 0xfffu) << 16);
	if (mmio_wr32(bar0_base + NVME_AQA, aqa))
		puts("ERR: AQA write timeout.");
	else if (mmio_last_err)
		puts("WARN: AQA write err=1 (write may still be accepted).");
	if (mmio_wr64(bar0_base + NVME_ASQ, (uint64_t)ASQ_ADDR))
		puts("ERR: ASQ write timeout.");
	else if (mmio_last_err)
		puts("WARN: ASQ write err=1 (write may still be accepted).");
	if (mmio_wr64(bar0_base + NVME_ACQ, (uint64_t)ACQ_ADDR))
		puts("ERR: ACQ write timeout.");
	else if (mmio_last_err)
		puts("WARN: ACQ write err=1 (write may still be accepted).");

	cc = cc_make_en(4, 6, 0);
	if (mmio_wr32(bar0_base + NVME_CC, cc))
		puts("ERR: CC write timeout.");
	else if (mmio_last_err)
		puts("WARN: CC write err=1 (write may still be accepted).");

	if (!nvme_wait_rdy(1, 2000000)) {
		puts("WARN: CSTS.RDY did not assert.");
	}

	nvme_admin_reset_queues();
	hostmem_fill(ASQ_ADDR, ADMIN_Q_ENTRIES * 64, 0);
	hostmem_fill(ACQ_ADDR, ADMIN_Q_ENTRIES * 16, 0);
	if (cap_out)
		*cap_out = cap;
	return 0;
}

static int nvme_identify_run(uint16_t cid)
{
	uint64_t cap = 0;
	if (nvme_admin_init(&cap))
		return 1;
	hostmem_fill(ACQ_ADDR, ADMIN_Q_ENTRIES * 16, 0);
	hostmem_fill(ID_BUF_ADDR, 0x100, 0);

	uint32_t cmd[16];
	uint32_t cqe[4];
	nvme_cmd_identify_controller(cid, (uint64_t)ID_BUF_ADDR, cmd);
	if (nvme_admin_submit(cap, cmd, cqe)) {
		puts("ERR: Identify command failed.");
		decode_cqe(cqe[0], cqe[1], cqe[2], cqe[3]);
		return 1;
	}

	uint32_t id_dws[64];
	hostmem_read_dwords(ID_BUF_ADDR, id_dws, 64);
	identify_decode(id_dws, 64);
	return 0;
}

static void nvme_admin_reset_queues(void)
{
	admin_sq_tail = 0;
	admin_cq_head = 0;
	admin_cq_phase = 1;
}

static void nvme_io_reset_queues(void)
{
	io_sq_tail = 0;
	io_cq_head = 0;
	io_cq_phase = 1;
}

static int nvme_admin_submit(uint64_t cap, const uint32_t *cmd, uint32_t *cqe)
{
	uint32_t sqe_addr = ASQ_ADDR + admin_sq_tail * 64;
	for (int i = 0; i < 16; i++)
		hostmem_wr32(sqe_addr + i * 4, cmd[i]);

	admin_sq_tail = (admin_sq_tail + 1) % ADMIN_Q_ENTRIES;
	uint64_t db = nvme_db_addr((uint64_t)bar0_base, cap, 0, 0);
	if (mmio_wr32(db, admin_sq_tail))
		puts("ERR: SQ doorbell write timeout.");
	else if (mmio_last_err)
		puts("WARN: SQ doorbell write err=1 (write may still be accepted).");

	uint32_t d0 = 0, d1 = 0, d2 = 0, d3 = 0;
	for (uint32_t loops = 0; loops < 2000000; loops++) {
		d0 = hostmem_rd32(ACQ_ADDR + admin_cq_head * 16 + 0);
		d1 = hostmem_rd32(ACQ_ADDR + admin_cq_head * 16 + 4);
		d2 = hostmem_rd32(ACQ_ADDR + admin_cq_head * 16 + 8);
		d3 = hostmem_rd32(ACQ_ADDR + admin_cq_head * 16 + 12);
		if (((d3 >> 16) & 0x1u) == admin_cq_phase)
			break;
	}

	cqe[0] = d0; cqe[1] = d1; cqe[2] = d2; cqe[3] = d3;

	admin_cq_head = (admin_cq_head + 1) % ADMIN_Q_ENTRIES;
	if (admin_cq_head == 0)
		admin_cq_phase ^= 1u;
	uint64_t db_cq = nvme_db_addr((uint64_t)bar0_base, cap, 0, 1);
	if (mmio_wr32(db_cq, admin_cq_head))
		puts("ERR: CQ doorbell write timeout.");
	else if (mmio_last_err)
		puts("WARN: CQ doorbell write err=1 (write may still be accepted).");

	return cqe_ok(cqe[3]) ? 0 : 1;
}

static int nvme_io_submit(uint64_t cap, const uint32_t *cmd, uint32_t *cqe)
{
	uint32_t sqe_addr = IO_SQ_ADDR + io_sq_tail * 64;
	for (int i = 0; i < 16; i++)
		hostmem_wr32(sqe_addr + i * 4, cmd[i]);

	io_sq_tail = (io_sq_tail + 1) % IO_Q_ENTRIES;
	uint64_t db = nvme_db_addr((uint64_t)bar0_base, cap, 1, 0);
	if (mmio_wr32(db, io_sq_tail))
		puts("ERR: IO SQ doorbell write timeout.");
	else if (mmio_last_err)
		puts("WARN: IO SQ doorbell write err=1 (write may still be accepted).");

	uint32_t d0 = 0, d1 = 0, d2 = 0, d3 = 0;
	for (uint32_t loops = 0; loops < 2000000; loops++) {
		d0 = hostmem_rd32(IO_CQ_ADDR + io_cq_head * 16 + 0);
		d1 = hostmem_rd32(IO_CQ_ADDR + io_cq_head * 16 + 4);
		d2 = hostmem_rd32(IO_CQ_ADDR + io_cq_head * 16 + 8);
		d3 = hostmem_rd32(IO_CQ_ADDR + io_cq_head * 16 + 12);
		if (((d3 >> 16) & 0x1u) == io_cq_phase)
			break;
	}

	cqe[0] = d0; cqe[1] = d1; cqe[2] = d2; cqe[3] = d3;

	io_cq_head = (io_cq_head + 1) % IO_Q_ENTRIES;
	if (io_cq_head == 0)
		io_cq_phase ^= 1u;
	uint64_t db_cq = nvme_db_addr((uint64_t)bar0_base, cap, 1, 1);
	if (mmio_wr32(db_cq, io_cq_head))
		puts("ERR: IO CQ doorbell write timeout.");
	else if (mmio_last_err)
		puts("WARN: IO CQ doorbell write err=1 (write may still be accepted).");

	return cqe_ok(cqe[3]) ? 0 : 1;
}

static void nvme_identify_cmd(char *str)
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
	cmd_print_cmdsts();
	nvme_identify_run(cid);
}

static int nvme_io_setup(uint64_t cap)
{
	nvme_io_reset_queues();
	hostmem_fill(IO_CQ_ADDR, IO_Q_ENTRIES * 16, 0);
	hostmem_fill(IO_SQ_ADDR, IO_Q_ENTRIES * 64, 0);

	uint32_t cmd[16];
	uint32_t cqe[4];

	nvme_cmd_set_features_num_queues(0x20, 1, 1, cmd);
	if (nvme_admin_submit(cap, cmd, cqe)) {
		puts("ERR: Set Features (Number of Queues) failed.");
		decode_cqe(cqe[0], cqe[1], cqe[2], cqe[3]);
		return 1;
	}

	nvme_cmd_create_iocq(0x21, (uint64_t)IO_CQ_ADDR, 1, IO_Q_ENTRIES - 1, 0, 0, 1, cmd);
	if (nvme_admin_submit(cap, cmd, cqe)) {
		puts("ERR: Create IO CQ failed.");
		decode_cqe(cqe[0], cqe[1], cqe[2], cqe[3]);
		return 1;
	}

	nvme_cmd_create_iosq(0x22, (uint64_t)IO_SQ_ADDR, 1, IO_Q_ENTRIES - 1, 1, 0, 1, cmd);
	if (nvme_admin_submit(cap, cmd, cqe)) {
		puts("ERR: Create IO SQ failed.");
		decode_cqe(cqe[0], cqe[1], cqe[2], cqe[3]);
		return 1;
	}

	return 0;
}

static void nvme_read_cmd(char *str)
{
	uint64_t base_addr = 0xe0000000ull;
	uint32_t nsid = 1;
	uint64_t slba = 0;
	uint32_t nlb  = 1;

	if (str && strlen(str)) {
		char *a = get_token(&str);
		char *b = get_token(&str);
		char *c = get_token(&str);
		char *d = get_token(&str);
		if (a && strlen(a)) base_addr = strtoull(a, NULL, 0);
		if (b && strlen(b)) nsid = (uint32_t)strtoul(b, NULL, 0);
		if (c && strlen(c)) slba = strtoull(c, NULL, 0);
		if (d && strlen(d)) nlb  = (uint32_t)strtoul(d, NULL, 0);
	}

	if (nlb == 0) {
		puts("ERR: nlb must be >= 1.");
		return;
	}
	if (nlb > 8) {
		puts("ERR: nlb too large for PRP1-only (max 8 blocks @ 512B).");
		return;
	}

	if (nvme_bar0_assign(base_addr))
		return;

	uint64_t cap = 0;
	if (nvme_admin_init(&cap))
		return;
	if (nvme_io_setup(cap))
		return;

	hostmem_fill(IO_RD_BUF_ADDR, 0x1000, 0);

	uint32_t cmd[16];
	uint32_t cqe[4];
	nvme_cmd_read(0x30, nsid, (uint64_t)IO_RD_BUF_ADDR, slba, (uint16_t)(nlb - 1), cmd);
	if (nvme_io_submit(cap, cmd, cqe)) {
		puts("ERR: Read command failed.");
		decode_cqe(cqe[0], cqe[1], cqe[2], cqe[3]);
		return;
	}

	uint32_t data[16];
	hostmem_read_dwords(IO_RD_BUF_ADDR, data, 16);
	printf("Read data (first 16 dwords):\n  ");
	for (int i = 0; i < 16; i++)
		printf("%08" PRIx32 "%s", data[i], (i == 15) ? "\n" : " ");
}

static void nvme_write_cmd(char *str)
{
	uint64_t base_addr = 0xe0000000ull;
	uint32_t nsid = 1;
	uint64_t slba = 0;
	uint32_t nlb  = 1;

	if (str && strlen(str)) {
		char *a = get_token(&str);
		char *b = get_token(&str);
		char *c = get_token(&str);
		char *d = get_token(&str);
		if (a && strlen(a)) base_addr = strtoull(a, NULL, 0);
		if (b && strlen(b)) nsid = (uint32_t)strtoul(b, NULL, 0);
		if (c && strlen(c)) slba = strtoull(c, NULL, 0);
		if (d && strlen(d)) nlb  = (uint32_t)strtoul(d, NULL, 0);
	}

	if (nlb == 0) {
		puts("ERR: nlb must be >= 1.");
		return;
	}
	if (nlb > 8) {
		puts("ERR: nlb too large for PRP1-only (max 8 blocks @ 512B).");
		return;
	}

	if (nvme_bar0_assign(base_addr))
		return;

	uint64_t cap = 0;
	if (nvme_admin_init(&cap))
		return;
	if (nvme_io_setup(cap))
		return;

	hostmem_fill(IO_WR_BUF_ADDR, 0x1000, 0xA5A5A5A5);

	uint32_t cmd[16];
	uint32_t cqe[4];
	nvme_cmd_write(0x31, nsid, (uint64_t)IO_WR_BUF_ADDR, slba, (uint16_t)(nlb - 1), cmd);
	if (nvme_io_submit(cap, cmd, cqe)) {
		puts("ERR: Write command failed.");
		decode_cqe(cqe[0], cqe[1], cqe[2], cqe[3]);
		return;
	}

	puts("Write completed.");
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
	else if (strcmp(token, "cfg_rd") == 0)
		cfg_rd_cmd(str);
	else if (strcmp(token, "cfg_wr") == 0)
		cfg_wr_cmd(str);
	else if (strcmp(token, "cmd_enable") == 0)
		cmd_enable_cmd();
	else if (strcmp(token, "cmd_disable") == 0)
		cmd_disable_cmd();
	else if (strcmp(token, "mmio_rd") == 0)
		mmio_rd_cmd(str);
	else if (strcmp(token, "mmio_wr") == 0)
		mmio_wr_cmd(str);
	else if (strcmp(token, "mmio_dump") == 0)
		mmio_dump_cmd(str);
	else if (strcmp(token, "nvme_identify") == 0)
		nvme_identify_cmd(str);
	else if (strcmp(token, "nvme_read") == 0)
		nvme_read_cmd(str);
	else if (strcmp(token, "nvme_write") == 0)
		nvme_write_cmd(str);
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
