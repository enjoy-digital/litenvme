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

static uint32_t bar0_base = 0;

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

static void bar0_cmd(char *str)
{
	if (str == NULL || strlen(str) == 0) {
		printf("bar0 = 0x%08" PRIx32 "\n", bar0_base);
		return;
	}
	bar0_base = (uint32_t)strtoul(str, NULL, 0);
	printf("bar0 = 0x%08" PRIx32 "\n", bar0_base);
}

static void bar0_rd_cmd(char *str)
{
	uint32_t off = (uint32_t)strtoul(str, NULL, 0);
	uint32_t v = 0;
	if (mmio_rd32((uint64_t)bar0_base + off, &v))
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
	if (mmio_wr32((uint64_t)bar0_base + off, v))
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
		mmio_rd32((uint64_t)bar0_base + off, &v);
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
	mmio_rd32((uint64_t)bar0_base + 0x0000, &cap0);
	mmio_rd32((uint64_t)bar0_base + 0x0004, &cap1);
	mmio_rd32((uint64_t)bar0_base + 0x0008, &vs);
	mmio_rd32((uint64_t)bar0_base + 0x001c, &csts);
	printf("CAP  = 0x%08" PRIx32 "%08" PRIx32 "\n", cap1, cap0);
	printf("VS   = 0x%08" PRIx32 "\n", vs);
	printf("CSTS = 0x%08" PRIx32 "\n", csts);
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

static void cmd_enable_cmd(void)
{
	uint32_t v = 0;
	if (cfg_rd32(1, &v)) {
		printf("ERR\n");
		return;
	}
	v |= (1 << 1); /* MEM */
	v |= (1 << 2); /* BME */
	if (cfg_wr32(1, v))
		printf("ERR\n");
}

static void cmd_disable_cmd(void)
{
	uint32_t v = 0;
	if (cfg_rd32(1, &v)) {
		printf("ERR\n");
		return;
	}
	v &= ~(1 << 1);
	v &= ~(1 << 2);
	if (cfg_wr32(1, v))
		printf("ERR\n");
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
