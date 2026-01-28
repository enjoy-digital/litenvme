// This file is part of LiteNVMe.
//
// Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
// Developed with LLM assistance.
// SPDX-License-Identifier: BSD-2-Clause

#include <stdio.h>
#include <string.h>

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
	printf("pcie_link: %08x\n", pcie_phy_phy_link_status_read());
#else
	printf("pcie_link: (no CSR)\n");
#endif
#ifdef CSR_HOSTMEM_CSR_DMA_WR_COUNT_ADDR
	printf("hostmem_wr_count: %08x\n", hostmem_csr_dma_wr_count_read());
	printf("hostmem_rd_count: %08x\n", hostmem_csr_dma_rd_count_read());
#else
	printf("hostmem_count: (no CSR)\n");
#endif
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
