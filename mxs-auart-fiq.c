/*
 * Freescale STMP37XX/STMP378X Application UART driver with polling and FIQ
 *
 * Author: Jonathan Ben-Avraham <yba@tkos.co.il>
 * Based on mxs-auart.c by Dmitry Pervushin <dimka@embeddedalley.com>
 *
 * Portions of this code are based on work by:
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 * (https://github.com/crystalfontz/cfa_10036_kernel/blob/cfa-3.9-10-10049-fiq/drivers/misc/cfa10049_fiq.c)
 * pascal@pabr.org
 * (http://www.pabr.org/pxarc/doc/pxarc.en.html)
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 * Copyright 2013 FriskyDSP Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

// Todo - make module parameters
// #define UART_IRQ
// #define TIMER_FIQ
#define UART_FIQ

#define ONE_SHOT_RX
#define DEBUG

#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_device.h>
#include <asm/cacheflush.h>
#include <linux/of_irq.h>
#include <asm/fiq.h>
#include <asm/pgtable.h>
#include <linux/fs.h>
#include <linux/of_address.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>

#ifndef DEBUG
#define printk(...)
#endif

#define MXS_AUART_PORTS 5

#define AUART_CTRL0			0x00000000
#define AUART_CTRL0_SET			0x00000004
#define AUART_CTRL0_CLR			0x00000008
#define AUART_CTRL0_TOG			0x0000000c
#define AUART_CTRL1			0x00000010
#define AUART_CTRL1_SET			0x00000014
#define AUART_CTRL1_CLR			0x00000018
#define AUART_CTRL1_TOG			0x0000001c
#define AUART_CTRL2			0x00000020
#define AUART_CTRL2_SET			0x00000024
#define AUART_CTRL2_CLR			0x00000028
#define AUART_CTRL2_TOG			0x0000002c
#define AUART_LINECTRL			0x00000030
#define AUART_LINECTRL_SET		0x00000034
#define AUART_LINECTRL_CLR		0x00000038
#define AUART_LINECTRL_TOG		0x0000003c
#define AUART_LINECTRL2			0x00000040
#define AUART_LINECTRL2_SET		0x00000044
#define AUART_LINECTRL2_CLR		0x00000048
#define AUART_LINECTRL2_TOG		0x0000004c
#define AUART_INTR			0x00000050
#define AUART_INTR_SET			0x00000054
#define AUART_INTR_CLR			0x00000058
#define AUART_INTR_TOG			0x0000005c
#define AUART_DATA			0x00000060
#define AUART_STAT			0x00000070
#define AUART_DEBUG			0x00000080
#define AUART_VERSION			0x00000090
#define AUART_AUTOBAUD			0x000000a0

#define AUART_CTRL0_SFTRST			(1 << 31)
#define AUART_CTRL0_CLKGATE			(1 << 30)
#define AUART_CTRL0_RXTO_ENABLE			(1 << 27)
#define AUART_CTRL0_RXTIMEOUT(v)		(((v) & 0x7ff) << 16)
#define AUART_CTRL0_XFER_COUNT(v)		((v) & 0xffff)

#define AUART_CTRL1_XFER_COUNT(v)		((v) & 0xffff)

#define AUART_CTRL2_DMAONERR			(1 << 26)
#define AUART_CTRL2_TXDMAE			(1 << 25)
#define AUART_CTRL2_RXDMAE			(1 << 24)

#define AUART_CTRL2_CTSEN			(1 << 15)
#define AUART_CTRL2_RTSEN			(1 << 14)
#define AUART_CTRL2_RTS				(1 << 11)
#define AUART_CTRL2_RXE				(1 << 9)
#define AUART_CTRL2_TXE				(1 << 8)
#define AUART_CTRL2_UARTEN			(1 << 0)

#define AUART_LINECTRL_BAUD_DIVINT_SHIFT	16
#define AUART_LINECTRL_BAUD_DIVINT_MASK		0xffff0000
#define AUART_LINECTRL_BAUD_DIVINT(v)		(((v) & 0xffff) << 16)
#define AUART_LINECTRL_BAUD_DIVFRAC_SHIFT	8
#define AUART_LINECTRL_BAUD_DIVFRAC_MASK	0x00003f00
#define AUART_LINECTRL_BAUD_DIVFRAC(v)		(((v) & 0x3f) << 8)
#define AUART_LINECTRL_WLEN_MASK		0x00000060
#define AUART_LINECTRL_WLEN(v)			(((v) & 0x3) << 5)
#define AUART_LINECTRL_FEN			(1 << 4)
#define AUART_LINECTRL_STP2			(1 << 3)
#define AUART_LINECTRL_EPS			(1 << 2)
#define AUART_LINECTRL_PEN			(1 << 1)
#define AUART_LINECTRL_BRK			(1 << 0)

#define AUART_INTR_RTIEN			(1 << 22)
#define AUART_INTR_TXIEN			(1 << 21)
#define AUART_INTR_RXIEN			(1 << 20)
#define AUART_INTR_CTSMIEN			(1 << 17)
#define AUART_INTR_RTIS				(1 << 6)
#define AUART_INTR_TXIS				(1 << 5)
#define AUART_INTR_RXIS				(1 << 4)
#define AUART_INTR_CTSMIS			(1 << 1)

#define AUART_STAT_BUSY				(1 << 29)
#define AUART_STAT_CTS				(1 << 28)
#define AUART_STAT_TXFE				(1 << 27)
#define AUART_STAT_TXFF				(1 << 25)
#define AUART_STAT_RXFE				(1 << 24)
#define AUART_STAT_OERR				(1 << 19)
#define AUART_STAT_BERR				(1 << 18)
#define AUART_STAT_PERR				(1 << 17)
#define AUART_STAT_FERR				(1 << 16)
#define AUART_STAT_RXCOUNT_MASK			0xffff

static struct uart_driver auart_driver;

enum mxs_auart_type {
	IMX23_AUART,
	IMX28_AUART,
};

struct mxs_auart_port {
	struct uart_port port;

#define MXS_AUART_DMA_CONFIG	0x1
#define MXS_AUART_DMA_ENABLED	0x2
#define MXS_AUART_DMA_TX_SYNC	2  /* bit 2 */
#define MXS_AUART_DMA_RX_READY	3  /* bit 3 */
	unsigned long flags;
	unsigned int ctrl;
	enum mxs_auart_type devtype;

	unsigned int irq;

	struct clk *clk;
	struct device *dev;

	/* for DMA */
	struct scatterlist tx_sgl;
	struct dma_chan	*tx_dma_chan;
	void *tx_dma_buf;

	struct scatterlist rx_sgl;
	struct dma_chan	*rx_dma_chan;
	void *rx_dma_buf;
};

static struct platform_device_id mxs_auart_devtype[] = {
	{ .name = "mxs-fiq-auart-imx23", .driver_data = IMX23_AUART },
	{ .name = "mxs-fiq-auart-imx28", .driver_data = IMX28_AUART },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, mxs_auart_devtype);

static inline int is_imx28_auart(struct mxs_auart_port *s)
{
	return s->devtype == IMX28_AUART;
}

static struct of_device_id mxs_auart_dt_ids[] = {
	{
		.compatible = "fsl,imx28-fiq-auart",
		.data = &mxs_auart_devtype[IMX28_AUART]
	}, {
		.compatible = "fsl,imx23-fiq-auart",
		.data = &mxs_auart_devtype[IMX23_AUART]
	}, { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxs_auart_dt_ids);


#define TIMROT_TIMCTRL_REG(n)	   (0x20 + (n) * 0x40)
#define TIMROT_TIMCTRL_SELECT_32K       (0xb)
#define TIMROT_TIMCTRL_ALWAYS_TICK      (0xf)
#define TIMROT_TIMCTRL_RELOAD	   (1 << 6)
#define TIMROT_TIMCTRL_UPDATE	   (1 << 7)
#define TIMROT_TIMCTRL_IRQ_EN	   (1 << 14)
#define TIMROT_TIMCTRL_IRQ	      (1 << 15)
#define TIMROT_FIXED_COUNT_REG(n)       (0x40 + (n) * 0x40)

#define HW_ICOLL_INTERRUPTn_SET(n)      (0x0124 + (n) * 0x10)

#define FIQ_RXTX_BUFFER_SIZE	256
#define FIQ_STACK_SIZE		256
#define FIQ_BUFFER_SIZE		(sizeof(struct fiq_buffer) + FIQ_STACK_SIZE)

struct char_cell {
	u16		uart_char;	// data byte plus four error bits
	unsigned int	uart_stat;	// UART status register contents
};

struct fiq_buffer {
#ifdef UART_IRQ
	void __iomem	*auart_base;
#endif
	u32	rx_head;
	u32	rx_tail;
	u32	tx_head;
	u32	tx_tail;
	struct char_cell	rx[FIQ_RXTX_BUFFER_SIZE];
	u8			tx[FIQ_RXTX_BUFFER_SIZE];
};

struct mxs_auart_data {
	struct cdev	chrdev;
	dma_addr_t	dma;
	void __iomem	*fiq_base;	/* The shared memory area, passed in r10 - yba */
	void __iomem	*icoll_base;	/* The interrupt controler block address - yba */
	unsigned int	irq;		/* The IRQ number for this driver - yba */
	void __iomem	*pinctrl_base;	/* Passed to handler in r9 to allow it to toggle pins - yba */
	struct clk	*timer_clk;	/* Local to only one subroutine in this file - yba */
	void __iomem	*timrot_base;	/* Passed to handler in r8 so it can set the timer - yba */
	void __iomem	*auart_base;
};

static inline bool auart_dma_enabled(struct mxs_auart_port *s)
{
	return s->flags & MXS_AUART_DMA_ENABLED;
}

#define BRK 0
#define SERIO_FRAME 0xFF
#define UART_FIFO_SIZE 16

#define HW_UARTAPP_DATA 0x00000060
#define HW_UARTAPP_STAT 0x00000070
#define HW_TIMROT_TIMCTRL2		0xa0
#define HW_TIMROT_FIXED_COUNT2		0xc0

#define BM_UARTAPP_STAT_TXFE 0x08000000 /* 134217728; Do not use "(1<27)", the optimizer will collapse the expression */
#define BM_UARTAPP_STAT_RXFF 0x04000000	/*  16777216; Do not use "(1<26)", the optimizer will collapse the expression */
#define BM_UARTAPP_STAT_TXFF 0x02000000 /*  33554432; Do not use "(1<25)", the optimizer will collapse the expression */
#define BM_UARTAPP_STAT_RXFE 0x01000000	/*  16777216; Do not use "(1<24)", the optimizer will collapse the expression */

#define BM_UARTAPP_STAT_OERR 0x00080000	/*  16777216; Do not use "(1<19)", the optimizer will collapse the expression */
#define BM_UARTAPP_STAT_BERR 0x00040000	/*  16777216; Do not use "(1<18)", the optimizer will collapse the expression */
#define BM_UARTAPP_STAT_PERR 0x00020000	/*  33554432; Do not use "(1<17)", the optimizer will collapse the expression */
#define BM_UARTAPP_STAT_FERR 0x00010000	/*  67108864; Do not use "(1<16)", the optimizer will collapse the expression */

#define BM_UARTAPP_STAT_BUSY 0x20000000

#define BIT_SET 0x4
#define BIT_CLR	0x8
#define BIT_TOG	0xc

#define AUART_INTR_CTSMIS		       (1 << 1)

void fiq_stub(void), fiq_stub_end(void);
#ifndef UART_IRQ
void fiq_handler_c_function(void);
#endif

// Define an exported symbol so that other devices can mmap to us
void *mxs_auart_fiq_buf = 0;
EXPORT_SYMBOL(mxs_auart_fiq_buf);

struct mxs_auart_data *mxs_auart_fiq_data;
#ifndef UART_IRQ
static struct task_struct * agui_task;
#endif
static void mxs_auart_start_tx(struct uart_port *u);
static int mxs_auart_startup(struct uart_port *u);
static void mxs_auart_stop_tx(struct uart_port *u);
#ifndef UART_IRQ
static int agui_thread(void *serial_port);

static int thread_init(struct mxs_auart_port *s)
{
	int error;

	agui_task = kthread_run(agui_thread, (void *)s, "mxs_auart_fiq");

	if (IS_ERR(agui_task)) {
		error = PTR_ERR(agui_task);
		return error;
	}

	return 0;
}

static void thread_exit(void)
{
	kthread_stop(agui_task);
}
#endif

#define to_auart_port(u) container_of(u, struct mxs_auart_port, port)

#ifdef UART_IRQ
static void mxs_auart_tx_chars(struct mxs_auart_port *s);

static void dma_tx_callback(void *param)
{
	struct mxs_auart_port *s = param;
	struct circ_buf *xmit = &s->port.state->xmit;

	dma_unmap_sg(s->dev, &s->tx_sgl, 1, DMA_TO_DEVICE);

	/* clear the bit used to serialize the DMA tx. */
	clear_bit(MXS_AUART_DMA_TX_SYNC, &s->flags);
	smp_mb__after_clear_bit();

	/* wake up the possible processes. */
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&s->port);

	mxs_auart_tx_chars(s);
}

static int mxs_auart_dma_tx(struct mxs_auart_port *s, int size)
{
	struct dma_async_tx_descriptor *desc;
	struct scatterlist *sgl = &s->tx_sgl;
	struct dma_chan *channel = s->tx_dma_chan;
	u32 pio;

	/* [1] : send PIO. Note, the first pio word is CTRL1. */
	pio = AUART_CTRL1_XFER_COUNT(size);
	desc = dmaengine_prep_slave_sg(channel, (struct scatterlist *)&pio,
					1, DMA_TRANS_NONE, 0);
	if (!desc) {
		dev_err(s->dev, "step 1 error\n");
		return -EINVAL;
	}

	/* [2] : set DMA buffer. */
	sg_init_one(sgl, s->tx_dma_buf, size);
	dma_map_sg(s->dev, sgl, 1, DMA_TO_DEVICE);
	desc = dmaengine_prep_slave_sg(channel, sgl,
			1, DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		dev_err(s->dev, "step 2 error\n");
		return -EINVAL;
	}

	/* [3] : submit the DMA */
	desc->callback = dma_tx_callback;
	desc->callback_param = s;
	dmaengine_submit(desc);
	dma_async_issue_pending(channel);
	return 0;
}

static void mxs_auart_tx_chars(struct mxs_auart_port *s)
{
	struct circ_buf *xmit = &s->port.state->xmit;

	if (auart_dma_enabled(s)) {
		u32 i = 0;
		int size;
		void *buffer = s->tx_dma_buf;

		if (test_and_set_bit(MXS_AUART_DMA_TX_SYNC, &s->flags))
			return;

		while (!uart_circ_empty(xmit) && !uart_tx_stopped(&s->port)) {
			size = min_t(u32, UART_XMIT_SIZE - i,
				     CIRC_CNT_TO_END(xmit->head,
						     xmit->tail,
						     UART_XMIT_SIZE));
			memcpy(buffer + i, xmit->buf + xmit->tail, size);
			xmit->tail = (xmit->tail + size) & (UART_XMIT_SIZE - 1);

			i += size;
			if (i >= UART_XMIT_SIZE)
				break;
		}

		if (uart_tx_stopped(&s->port))
			mxs_auart_stop_tx(&s->port);

		if (i) {
			mxs_auart_dma_tx(s, i);
		} else {
			clear_bit(MXS_AUART_DMA_TX_SYNC, &s->flags);
			smp_mb__after_clear_bit();
		}
		return;
	}


	while (!(readl(s->port.membase + AUART_STAT) &
		 AUART_STAT_TXFF)) {
		if (s->port.x_char) {
			s->port.icount.tx++;
			writel(s->port.x_char,
				     s->port.membase + AUART_DATA);
			s->port.x_char = 0;
			continue;
		}
		if (!uart_circ_empty(xmit) && !uart_tx_stopped(&s->port)) {
			s->port.icount.tx++;
			writel(xmit->buf[xmit->tail],
				     s->port.membase + AUART_DATA);
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		} else
			break;
	}
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&s->port);

	if (uart_circ_empty(&(s->port.state->xmit)))
		writel(AUART_INTR_TXIEN,
			     s->port.membase + AUART_INTR_CLR);
	else
		writel(AUART_INTR_TXIEN,
			     s->port.membase + AUART_INTR_SET);

	if (uart_tx_stopped(&s->port))
		mxs_auart_stop_tx(&s->port);
}

// For BERR_UART, see also drivers/input/serio/serport.c
#define BERR_UART
//#define BERR_UART_DEBUG
static void mxs_auart_rx_char(struct mxs_auart_port *s)
{
	int flag;
	u32 stat;
	u8 c;

	c = readl(s->port.membase + AUART_DATA);
	stat = readl(s->port.membase + AUART_STAT);

	flag = TTY_NORMAL;
	s->port.icount.rx++;

	if (stat & AUART_STAT_BERR) {
		s->port.icount.brk++;
		if (uart_handle_break(&s->port))
			goto out;
#ifdef BERR_UART_DEBUG
		printk(KERN_DEBUG "mxs_auart_rx_char stat is AUART_STAT_BERR\n");
#endif
	} else if (stat & AUART_STAT_PERR) {
		s->port.icount.parity++;
	} else if (stat & AUART_STAT_FERR) {
		s->port.icount.frame++;
	}

	/*
	 * Mask off conditions which should be ingored.
	 */
#ifndef BERR_UART
	// This mask should be fixed in userspace rather than left out of the build here
	stat &= s->port.read_status_mask;
#endif

	if (stat & AUART_STAT_BERR) {
		flag = TTY_BREAK;
#ifdef BERR_UART_DEBUG
		printk(KERN_DEBUG "mxs_auart_rx_char got TTY_BREAK\n");
#endif
	} else if (stat & AUART_STAT_PERR)
		flag = TTY_PARITY;
	else if (stat & AUART_STAT_FERR)
		flag = TTY_FRAME;

	if (stat & AUART_STAT_OERR) {
		s->port.icount.overrun++;
		printk(KERN_DEBUG "mxs_auart_rx_char(): AUART_STAT_OERR count [%d]\n", s->port.icount.overrun);
	}

	if (uart_handle_sysrq_char(&s->port, c))
		goto out;

	uart_insert_char(&s->port, stat, AUART_STAT_OERR, c, flag);
out:
	writel(stat, s->port.membase + AUART_STAT);
} 
#endif

static const char *mxs_auart_type(struct uart_port *u)
{
	struct mxs_auart_port *s = to_auart_port(u);

	return dev_name(s->dev);
}

static void mxs_auart_set_mctrl(struct uart_port *u, unsigned mctrl)
{
	struct mxs_auart_port *s = to_auart_port(u);

	u32 ctrl = readl(u->membase + AUART_CTRL2);

	ctrl &= ~(AUART_CTRL2_RTSEN | AUART_CTRL2_RTS);
	if (mctrl & TIOCM_RTS) {
		if (tty_port_cts_enabled(&u->state->port))
			ctrl |= AUART_CTRL2_RTSEN;
		else
			ctrl |= AUART_CTRL2_RTS;
	}

	s->ctrl = mctrl;
	writel(ctrl, u->membase + AUART_CTRL2);
}

#ifdef UART_IRQ
static void mxs_auart_rx_chars(struct mxs_auart_port *s)
{
	u32 stat = 0;

	for (;;) {
		stat = readl(s->port.membase + AUART_STAT);
		if (stat & AUART_STAT_RXFE)
			break;
		mxs_auart_rx_char(s);
	}

	writel(stat, s->port.membase + AUART_STAT);
	tty_flip_buffer_push(&s->port.state->port);
}

static int mxs_auart_dma_prep_rx(struct mxs_auart_port *s);
static void dma_rx_callback(void *arg)
{
	struct mxs_auart_port *s = (struct mxs_auart_port *) arg;
	struct tty_port *port = &s->port.state->port;
	int count;
	u32 stat;

	dma_unmap_sg(s->dev, &s->rx_sgl, 1, DMA_FROM_DEVICE);

	stat = readl(s->port.membase + AUART_STAT);
	stat &= ~(AUART_STAT_OERR | AUART_STAT_BERR |
			AUART_STAT_PERR | AUART_STAT_FERR);

	count = stat & AUART_STAT_RXCOUNT_MASK;
	tty_insert_flip_string(port, s->rx_dma_buf, count);

	writel(stat, s->port.membase + AUART_STAT);
	tty_flip_buffer_push(port);

	/* start the next DMA for RX. */
	mxs_auart_dma_prep_rx(s);
}

static int mxs_auart_dma_prep_rx(struct mxs_auart_port *s)
{
	struct dma_async_tx_descriptor *desc;
	struct scatterlist *sgl = &s->rx_sgl;
	struct dma_chan *channel = s->rx_dma_chan;
	u32 pio[1];

	/* [1] : send PIO */
	pio[0] = AUART_CTRL0_RXTO_ENABLE
		| AUART_CTRL0_RXTIMEOUT(0x80)
		| AUART_CTRL0_XFER_COUNT(UART_XMIT_SIZE);
	desc = dmaengine_prep_slave_sg(channel, (struct scatterlist *)pio,
					1, DMA_TRANS_NONE, 0);
	if (!desc) {
		dev_err(s->dev, "step 1 error\n");
		return -EINVAL;
	}

	/* [2] : send DMA request */
	sg_init_one(sgl, s->rx_dma_buf, UART_XMIT_SIZE);
	dma_map_sg(s->dev, sgl, 1, DMA_FROM_DEVICE);
	desc = dmaengine_prep_slave_sg(channel, sgl, 1, DMA_DEV_TO_MEM,
					DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		dev_err(s->dev, "step 2 error\n");
		return -1;
	}

	/* [3] : submit the DMA, but do not issue it. */
	desc->callback = dma_rx_callback;
	desc->callback_param = s;
	dmaengine_submit(desc);
	dma_async_issue_pending(channel);
	return 0;
}

static void mxs_auart_dma_exit_channel(struct mxs_auart_port *s)
{
	if (s->tx_dma_chan) {
		dma_release_channel(s->tx_dma_chan);
		s->tx_dma_chan = NULL;
	}
	if (s->rx_dma_chan) {
		dma_release_channel(s->rx_dma_chan);
		s->rx_dma_chan = NULL;
	}

	kfree(s->tx_dma_buf);
	kfree(s->rx_dma_buf);
	s->tx_dma_buf = NULL;
	s->rx_dma_buf = NULL;
}

static void mxs_auart_dma_exit(struct mxs_auart_port *s)
{

	writel(AUART_CTRL2_TXDMAE | AUART_CTRL2_RXDMAE | AUART_CTRL2_DMAONERR,
		s->port.membase + AUART_CTRL2_CLR);

	mxs_auart_dma_exit_channel(s);
	s->flags &= ~MXS_AUART_DMA_ENABLED;
	clear_bit(MXS_AUART_DMA_TX_SYNC, &s->flags);
	clear_bit(MXS_AUART_DMA_RX_READY, &s->flags);
}

static int mxs_auart_dma_init(struct mxs_auart_port *s)
{
	if (auart_dma_enabled(s))
		return 0;

	/* init for RX */
	s->rx_dma_chan = dma_request_slave_channel(s->dev, "rx");
	if (!s->rx_dma_chan)
		goto err_out;
	s->rx_dma_buf = kzalloc(UART_XMIT_SIZE, GFP_KERNEL | GFP_DMA);
	if (!s->rx_dma_buf)
		goto err_out;

	/* init for TX */
	s->tx_dma_chan = dma_request_slave_channel(s->dev, "tx");
	if (!s->tx_dma_chan)
		goto err_out;
	s->tx_dma_buf = kzalloc(UART_XMIT_SIZE, GFP_KERNEL | GFP_DMA);
	if (!s->tx_dma_buf)
		goto err_out;

	/* set the flags */
	s->flags |= MXS_AUART_DMA_ENABLED;
	dev_dbg(s->dev, "enabled the DMA support.");

	return 0;

err_out:
	mxs_auart_dma_exit_channel(s);
	return -EINVAL;

}

static irqreturn_t mxs_auart_irq_handle(int irq, void *context)
{
	u32 istatus, istat;
	struct mxs_auart_port *s = context;
	u32 stat = readl(s->port.membase + AUART_STAT);

	istatus = istat = readl(s->port.membase + AUART_INTR);

	if (istat & AUART_INTR_CTSMIS) {
		uart_handle_cts_change(&s->port, stat & AUART_STAT_CTS);
		writel(AUART_INTR_CTSMIS,
				s->port.membase + AUART_INTR_CLR);
		istat &= ~AUART_INTR_CTSMIS;
	}

	if (istat & (AUART_INTR_RTIS | AUART_INTR_RXIS)) {
		if (!auart_dma_enabled(s))
			mxs_auart_rx_chars(s);
		istat &= ~(AUART_INTR_RTIS | AUART_INTR_RXIS);
	}

	if (istat & AUART_INTR_TXIS) {
		mxs_auart_tx_chars(s);
		istat &= ~AUART_INTR_TXIS;
	}

	writel(istatus & (AUART_INTR_RTIS
		| AUART_INTR_TXIS
		| AUART_INTR_RXIS
		| AUART_INTR_CTSMIS),
			s->port.membase + AUART_INTR_CLR);

	return IRQ_HANDLED;
}
#endif

static int mxs_auart_request_port(struct uart_port *u)
{
	return 0;
}

static int mxs_auart_verify_port(struct uart_port *u,
				    struct serial_struct *ser)
{
	if (u->type != PORT_UNKNOWN && u->type != PORT_IMX)
		return -EINVAL;
	return 0;
}

static void mxs_auart_config_port(struct uart_port *u, int flags)
{
}

static void mxs_auart_release_port(struct uart_port *u)
{
}

static u32 mxs_auart_get_mctrl(struct uart_port *u)
{
	struct mxs_auart_port *s = to_auart_port(u);
	u32 stat = readl(u->membase + AUART_STAT);
	int ctrl2 = readl(u->membase + AUART_CTRL2);
	u32 mctrl = s->ctrl;

	mctrl &= ~TIOCM_CTS;
	if (stat & AUART_STAT_CTS)
		mctrl |= TIOCM_CTS;

	if (ctrl2 & AUART_CTRL2_RTS)
		mctrl |= TIOCM_RTS;

	return mctrl;
}

static void mxs_auart_settermios(struct uart_port *u,
				 struct ktermios *termios,
				 struct ktermios *old)
{
#ifdef UART_IRQ
	struct mxs_auart_port *s = to_auart_port(u);
#endif
	u32 bm, ctrl, ctrl2, div;
	unsigned int cflag, baud;

	cflag = termios->c_cflag;

	ctrl = AUART_LINECTRL_FEN;
	ctrl2 = readl(u->membase + AUART_CTRL2);

	/* byte size */
	switch (cflag & CSIZE) {
	case CS5:
		bm = 0;
		break;
	case CS6:
		bm = 1;
		break;
	case CS7:
		bm = 2;
		break;
	case CS8:
		bm = 3;
		break;
	default:
		return;
	}

	ctrl |= AUART_LINECTRL_WLEN(bm);

	/* parity */
	if (cflag & PARENB) {
		ctrl |= AUART_LINECTRL_PEN;
		if ((cflag & PARODD) == 0)
			ctrl |= AUART_LINECTRL_EPS;
	}

	u->read_status_mask = 0;

	if (termios->c_iflag & INPCK)
		u->read_status_mask |= AUART_STAT_PERR;
	if (termios->c_iflag & (BRKINT | PARMRK))
		u->read_status_mask |= AUART_STAT_BERR;

	/*
	 * Characters to ignore
	 */
	u->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		u->ignore_status_mask |= AUART_STAT_PERR;
	if (termios->c_iflag & IGNBRK) {
		u->ignore_status_mask |= AUART_STAT_BERR;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			u->ignore_status_mask |= AUART_STAT_OERR;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if (cflag & CREAD)
		ctrl2 |= AUART_CTRL2_RXE;
	else
		ctrl2 &= ~AUART_CTRL2_RXE;

	/* figure out the stop bits requested */
	if (cflag & CSTOPB)
		ctrl |= AUART_LINECTRL_STP2;

	/* figure out the hardware flow control settings */
	if (cflag & CRTSCTS) {
#ifdef UART_IRQ
		/*
		 * The DMA has a bug(see errata:2836) in mx23.
		 * So we can not implement the DMA for auart in mx23,
		 * we can only implement the DMA support for auart
		 * in mx28.
		 */
		if (is_imx28_auart(s) && (s->flags & MXS_AUART_DMA_CONFIG)) {
			if (!mxs_auart_dma_init(s))
				/* enable DMA tranfer */
				ctrl2 |= AUART_CTRL2_TXDMAE | AUART_CTRL2_RXDMAE
				       | AUART_CTRL2_DMAONERR;
		}
#endif
		ctrl2 |= AUART_CTRL2_CTSEN | AUART_CTRL2_RTSEN;
	} else {
		ctrl2 &= ~(AUART_CTRL2_CTSEN | AUART_CTRL2_RTSEN);
	}

	/* set baud rate */
	baud = uart_get_baud_rate(u, termios, old, 0, u->uartclk);
	div = u->uartclk * 32 / baud;
	ctrl |= AUART_LINECTRL_BAUD_DIVFRAC(div & 0x3F);
	ctrl |= AUART_LINECTRL_BAUD_DIVINT(div >> 6);

	writel(ctrl, u->membase + AUART_LINECTRL);
	writel(ctrl2, u->membase + AUART_CTRL2);

	uart_update_timeout(u, termios->c_cflag, baud);
#ifdef UART_IRQ
	/* prepare for the DMA RX. */
	if (auart_dma_enabled(s) &&
		!test_and_set_bit(MXS_AUART_DMA_RX_READY, &s->flags)) {
		if (!mxs_auart_dma_prep_rx(s)) {
			/* Disable the normal RX interrupt. */
			writel(AUART_INTR_RXIEN | AUART_INTR_RTIEN,
					u->membase + AUART_INTR_CLR);
		} else {
			mxs_auart_dma_exit(s);
			dev_err(s->dev, "We can not start up the DMA.\n");
		}
	}
#endif
}

static void mxs_auart_reset(struct uart_port *u)
{
	int i;
	unsigned int reg;

	printk(KERN_DEBUG "Entered mxs_auart_reset\n");
	writel(AUART_CTRL0_SFTRST, u->membase + AUART_CTRL0_CLR);

	for (i = 0; i < 10000; i++) {
		reg = readl(u->membase + AUART_CTRL0);
		if (!(reg & AUART_CTRL0_SFTRST))
			break;
		udelay(3);
	}
	writel(AUART_CTRL0_CLKGATE, u->membase + AUART_CTRL0_CLR);
}

static int mxs_auart_startup(struct uart_port *u)
{
	struct mxs_auart_port *s = to_auart_port(u);

	printk(KERN_DEBUG "Entered mxs_auart_startup\n");
	clk_prepare_enable(s->clk);

	writel(AUART_CTRL0_CLKGATE, u->membase + AUART_CTRL0_CLR);

	writel(AUART_CTRL2_UARTEN, u->membase + AUART_CTRL2_SET);
	writel(AUART_CTRL2_TXE, u->membase + AUART_CTRL2_SET);
#ifdef TIMER_FIQ
	writel(AUART_INTR_RXIEN | AUART_INTR_RTIEN | AUART_INTR_CTSMIEN,
			u->membase + AUART_INTR_CLR);
#else
	writel(AUART_INTR_RXIEN | AUART_INTR_RTIEN | AUART_INTR_CTSMIEN,
			u->membase + AUART_INTR);
#endif
	/*
	 * Enable fifo so all four bytes of a word are written to
	 * output (otherwise, only the LSB is written, ie. 1 in 4 bytes)
	 */
	writel(AUART_LINECTRL_FEN, u->membase + AUART_LINECTRL_SET);
	return 0;
}

static void mxs_auart_shutdown(struct uart_port *u)
{
	struct mxs_auart_port *s = to_auart_port(u);

	printk(KERN_DEBUG "Entered mxs_auart_shutdown\n");

	writel(AUART_CTRL2_UARTEN, u->membase + AUART_CTRL2_CLR);

	writel(AUART_INTR_RXIEN | AUART_INTR_RTIEN | AUART_INTR_CTSMIEN,
			u->membase + AUART_INTR_CLR);

	writel(AUART_CTRL0_CLKGATE, u->membase + AUART_CTRL0_SET);

	clk_disable_unprepare(s->clk);
}

static unsigned int mxs_auart_tx_empty(struct uart_port *u)
{
	if (readl(u->membase + AUART_STAT) & AUART_STAT_TXFE)
		return TIOCSER_TEMT;
	else
		return 0;
}

static void mxs_auart_start_tx(struct uart_port *u)
{
	/* enable transmitter */
	// printk("TX Enable set [0x%x]\n", (u32)u->membase);
	writel(AUART_CTRL2_TXE, u->membase + AUART_CTRL2_SET);
}

static void mxs_auart_stop_tx(struct uart_port *u)
{
	printk(KERN_DEBUG "Entered mxs_auart_stop_tx\n");
	writel(AUART_CTRL2_TXE, u->membase + AUART_CTRL2_CLR);
}

static void mxs_auart_stop_rx(struct uart_port *u)
{
	printk(KERN_DEBUG "Entered mxs_auart_stop_rx\n");
	writel(AUART_CTRL2_RXE, u->membase + AUART_CTRL2_CLR);
}

static void mxs_auart_break_ctl(struct uart_port *u, int ctl)
{
	if (ctl)
		writel(AUART_LINECTRL_BRK,
			     u->membase + AUART_LINECTRL_SET);
	else
		writel(AUART_LINECTRL_BRK,
			     u->membase + AUART_LINECTRL_CLR);
}

static void mxs_auart_enable_ms(struct uart_port *port)
{
	/* just empty */
}

static struct uart_ops mxs_auart_ops = {
	.tx_empty       = mxs_auart_tx_empty,
	.start_tx       = mxs_auart_start_tx,
	.stop_tx	= mxs_auart_stop_tx,
	.stop_rx	= mxs_auart_stop_rx,
	.enable_ms      = mxs_auart_enable_ms,
	.break_ctl      = mxs_auart_break_ctl,
	.set_mctrl	= mxs_auart_set_mctrl,
	.get_mctrl      = mxs_auart_get_mctrl,
	.startup	= mxs_auart_startup,
	.shutdown       = mxs_auart_shutdown,
	.set_termios    = mxs_auart_settermios,
	.type	   	= mxs_auart_type,
	.release_port   = mxs_auart_release_port,
	.request_port   = mxs_auart_request_port,
	.config_port    = mxs_auart_config_port,
	.verify_port    = mxs_auart_verify_port,
};

static struct mxs_auart_port *auart_port[MXS_AUART_PORTS];

static struct uart_driver auart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "ttyFiq",
	.dev_name	= "ttyFiq",
	.major		= 0,
	.minor		= 0,
	.nr		= MXS_AUART_PORTS,
};

/*
 * This function returns 1 if pdev isn't a device instatiated by dt, 0 if it
 * could successfully get all information from dt or a negative errno.
 */
static int serial_mxs_probe_dt(struct mxs_auart_port *s,
		struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;

	if (!np)
		/* no device tree device */
		return 1;

	ret = of_alias_get_id(np, "serial");
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get alias id: %d\n", ret);
		return ret;
	}
	s->port.line = ret;

	s->flags |= MXS_AUART_DMA_CONFIG;

	return 0;
}

#define CIRC_INC(x,s)		((x) = ((x)+1)%((s)-1))
#define CIRC_NOT_FULL(h,t,s)	((((h)+1)%((s)-1)) != (t))
#define CIRC_FULL(h,t,s)	((((h)+1)%((s)-1)) == (t))
#define CIRC_NOT_EMPTY(h,t)	((h) != (t))
#define CIRC_EMPTY(h,t)		((h) == (t))

#ifndef UART_IRQ
static int agui_thread(void *serial_port)
{
	struct mxs_auart_port *s = serial_port;
	struct fiq_buffer *shm = (struct fiq_buffer *)mxs_auart_fiq_buf;

	#define REPORTED	1
	#define UNREPORTED	0
#ifdef DEBUG
	int rx_buffer_state = UNREPORTED;
#endif
#ifdef TIMER_FIQ
	int tx_buffer_state = UNREPORTED;
#endif
	struct circ_buf *xmit;
	int count = 0;
	u32 stat;
	u16 uart_char;
	unsigned long flag;

 	do {
		if ( ! s->port.state->xmit.buf) {
			// printk("No xmit!\n");
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout (1*HZ);
			continue;
		}
		
		xmit = &s->port.state->xmit;
		count = 0;

#ifdef TIMER_FIQ
		while (CIRC_NOT_FULL(shm->tx_head, shm->tx_tail, FIQ_RXTX_BUFFER_SIZE)) {
			tx_buffer_state = UNREPORTED;
			if (s->port.x_char) {
				count++;
				printk(KERN_DEBUG "Moving char to tx buffer (0)\n");
				s->port.icount.tx++;
				shm->tx[shm->tx_head] = s->port.x_char;
				CIRC_INC(shm->tx_head, FIQ_RXTX_BUFFER_SIZE);
				s->port.x_char = 0;
				continue;
			}

			if (!uart_circ_empty(xmit) && !uart_tx_stopped(&s->port)) { // Crawl up the tail
				count++;
				printk(KERN_DEBUG "Moving char [%c] to tx buffer (1)\n", xmit->buf[xmit->tail]);
				s->port.icount.tx++;
				shm->tx[shm->tx_head] = xmit->buf[xmit->tail];
				CIRC_INC(shm->tx_head, FIQ_RXTX_BUFFER_SIZE);
				xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			} else
				break;		// we have moved all of the available chars
		}
		if (CIRC_NOT_EMPTY(shm->tx_head, shm->tx_tail))
			writel(AUART_INTR_TXIEN, s->port.membase + AUART_INTR_SET);
#ifdef DEBUG
		if ((UNREPORTED == tx_buffer_state) && CIRC_FULL(shm->tx_head, shm->tx_tail, FIQ_RXTX_BUFFER_SIZE)) {
			tx_buffer_state = REPORTED;
			printk(KERN_DEBUG "FIQ TX buffer is full, count [%d]\n", count);
		}
#endif
#else
		while ( ! (readl(s->port.membase + AUART_STAT) & AUART_STAT_TXFF)) {
			if (s->port.x_char) {
				count++;
				s->port.icount.tx++;
				writel(s->port.x_char, s->port.membase + AUART_DATA);
				s->port.x_char = 0;
				continue;
			}
			if (!uart_circ_empty(xmit) && !uart_tx_stopped(&s->port)) {
				count++;
				s->port.icount.tx++;
				writel(xmit->buf[xmit->tail], s->port.membase + AUART_DATA);
				xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			} else
			break;
		}
#endif
		if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
			uart_write_wakeup(&s->port);

		if (uart_tx_stopped(&s->port))
			mxs_auart_stop_tx(&s->port);
		else
			writel(AUART_INTR_TXIEN, s->port.membase + AUART_INTR_SET);

		count = 0;
		while (CIRC_NOT_EMPTY(shm->rx_head, shm->rx_tail)) {
#ifdef DEBUG
			rx_buffer_state = UNREPORTED;
#endif
			count++;
			s->port.icount.rx++;
			flag = TTY_NORMAL;
			uart_char = shm->rx[shm->rx_tail].uart_char;
			stat = shm->rx[shm->rx_tail].uart_stat;

			printk(KERN_DEBUG "char [0x%x], stat [0x%x] ack [%d]\n",
				shm->rx[shm->rx_tail].uart_char, stat, shm->ack_count);

			if (uart_char & 0xf00) {
				// if (stat & AUART_STAT_BERR)
				if (uart_char & 0x400)
				{
					stat |= AUART_STAT_FERR;	// Not AUART_STAT_BERR !
					s->port.icount.brk++;
					if (uart_handle_break(&s->port))
						continue;
				}
				// else if (stat & AUART_STAT_PERR)
				else if (uart_char & 0x200)
				{
					stat |= AUART_STAT_PERR;
					s->port.icount.parity++;
				}
				// else if (stat & AUART_STAT_FERR)
				else if (uart_char & 0x100)
				{
					stat |= AUART_STAT_FERR;
					s->port.icount.frame++;
				}

				/*
			 	* Mask off conditions that should be ignored.
			 	*/
				stat &= s->port.read_status_mask;

				// if (stat & AUART_STAT_BERR)
				if (uart_char & 0x400)
				{
					//flag = TTY_BREAK;
					flag = TTY_FRAME;
				}
				// else if (stat & AUART_STAT_PERR)
				else if (uart_char & 0x200)
					flag = TTY_PARITY;
				// else if (stat & AUART_STAT_FERR)
				else if (uart_char & 0x100)
					flag = TTY_FRAME;

				// if (stat & AUART_STAT_OERR)
				if (uart_char & 0x800)
					s->port.icount.overrun++;
			}

			uart_insert_char(&s->port,
					stat,
					AUART_STAT_OERR,
					shm->rx[shm->rx_tail].uart_char,
					flag);

			// printk(KERN_DEBUG "char is [0x%x], stat is [0x%x] flag is [%lu]\n", shm->rx[shm->rx_tail].uart_char, stat, flag);
			CIRC_INC(shm->rx_tail, FIQ_RXTX_BUFFER_SIZE);
		}

		if (0 < count) {
			printk(KERN_DEBUG "received [%d] chars\n", count);
			count = 0;
			tty_flip_buffer_push(&s->port.state->port);
		}
#ifdef DEBUG
		else if (UNREPORTED == rx_buffer_state) {
			rx_buffer_state = REPORTED;
			printk(KERN_DEBUG "receive buffer is empty\n");
		}
#endif
		// usleep_range(522,1044);	// 3-6 chars at 57600 baud 8n1 (10 bits/byte, 174us/char)
		usleep_range(696,1044);	// 4-6 chars at 57600 baud 8n1 (10 bits/byte, 174us/char)
		schedule();

	} while ( ! kthread_should_stop());

	return 0;
}
#endif

#if defined(UART_FIQ)
static irqreturn_t fiq_tx_window_timeout_handler(int irq, void *context)
{
	struct mxs_auart_data *d = (struct mxs_auart_data *)context;
	struct fiq_buffer *shm = d->fiq_base + FIQ_STACK_SIZE;
	u32 *timrot_base = (u32 *)d->timrot_base;

	printk(KERN_DEBUG "Entered fiq_tx_window_timeout_handler\n");
	timrot_base[(0xA0 +0x08)/4] = (1 << 15);	// ACK the timer interrupt
	shm->tx_window = 0;
	return IRQ_HANDLED;
}
#endif


#ifndef UART_IRQ
void fiq_handler_c_function(void)
{
	#define GET_UART_STAT		uart_base[HW_UARTAPP_STAT/4]
	#define SET_UART_STAT(s)	(uart_base[HW_UARTAPP_STAT/4] = (s))
	#define GET_UART_CHAR		uart_base[HW_UARTAPP_DATA/4]
	#define SEND_UART_CHAR(c)	(uart_base[HW_UARTAPP_DATA/4] = (c))
	#define SERIO_FRAME		0xFF

	// register unsigned long		*timrot_base	asm	("r8");
	register unsigned long		*uart_base	asm	("r11");	/* fp */
	register struct fiq_buffer	*shm		asm	("r10");	/* sl */

	int i;
	u32 volatile stat;
	u32 tx_tail, tx_head, rx_head, rx_tail;
	u16 volatile uart_char;
	u8 *tx;
	struct char_cell *rx;

	stat = GET_UART_STAT;
	rx_head = shm->rx_head;
	rx_tail = shm->rx_tail;
	rx = shm->rx;

	if (0 == (stat & BM_UARTAPP_STAT_RXFE)) {
#ifdef ONE_SHOT_RX
		if (CIRC_NOT_FULL(rx_head, rx_tail, FIQ_RXTX_BUFFER_SIZE)) {
#else
		while (CIRC_NOT_FULL(rx_head, rx_tail, FIQ_RXTX_BUFFER_SIZE)) {
#endif
			uart_char = GET_UART_CHAR;
			stat = GET_UART_STAT;
			rx[rx_head].uart_char = uart_char;
			rx[rx_head].uart_stat = stat;
			CIRC_INC(rx_head, FIQ_RXTX_BUFFER_SIZE);
			shm->rx_head = rx_head;
			SET_UART_STAT(stat);

#ifndef ONE_SHOT_RX
			if (stat & BM_UARTAPP_STAT_RXFE) break;
#endif
			stat = GET_UART_STAT;
		}
	}		// End of receive section

	tx_tail = shm->tx_tail;
	tx_head = shm->tx_head;
	tx = shm->tx;

	stat = GET_UART_STAT & (BM_UARTAPP_STAT_BUSY | BM_UARTAPP_STAT_TXFF);

	while ( (0 == stat) && CIRC_NOT_EMPTY(tx_head, tx_tail) ) {
		uart_char = tx[tx_tail];
		SEND_UART_CHAR(uart_char);
		CIRC_INC(tx_tail, FIQ_RXTX_BUFFER_SIZE);
		shm->tx_tail = tx_tail;
		for (i=0; i<4096; i++) {}
		stat = GET_UART_STAT & (BM_UARTAPP_STAT_BUSY | BM_UARTAPP_STAT_TXFF);
	}
}

static struct fiq_handler mxs_auart_fh = {
	.name	= "fiq_stub"
};
#endif

#if defined(TIMER_FIQ)
void fiq_stub_wrapper(void) {
	__asm__ volatile ("\
		fiq_stub:							\n\
			@ Registers: r8=timrot_base, r9=address of C function, sp	\n\
										\n\
			@ Before anything else, ACK the timer interrupt		\n\
			@ Caveat, make sure that r12 is not used		\n\
			mov     r12, #32768      @ 32768 is (1<<15)		\n\
			str     r12, [r8, #168]  @ timrot_base + 0xA0+0x08	\n\
										\n\
			@ Save the unbanked registers on our private stack	\n\
			stmdb sp!, {r0-r7,lr}					\n\
										\n\
			@ Call a C function with no arguments			\n\
			@  First, move the current program counter into the link register	\n\
			mov lr, pc						\n\
			@  Next, move the address of the C function to the program counter	\n\
			mov pc, r9						\n\
			@  We are now back from the C function			\n\
										\n\
			@ Restore the unbanked registers and return from the FIQ handler	\n\
			ldmia sp!, {r0-r7,lr}					\n\
			@ Return						\n\
			subs pc, lr, #4						\n\
		fiq_stub_end:							\n\
	");
}

#elif defined(UART_FIQ)

void fiq_stub_wrapper(void) {
	__asm__ volatile ("\
		fiq_stub:							\n\
			@ Registers: r8=timrot_base, r11=uart_base, r9=address of C function, sp	\n\
			@ Caveat, make sure that r12 is not used		\n\
										\n\
			@ Save the unbanked registers on our private stack	\n\
			stmdb sp!, {r0-r7,lr}					\n\
										\n\
			@ Call a C function with no arguments			\n\
			@  First, move the current program counter into the link register	\n\
			mov lr, pc						\n\
			@  Next, move the address of the C function to the program counter	\n\
			mov pc, r9						\n\
			@  We are now back from the C function			\n\
										\n\
			@ ACK the UART interrupt				\n\
			ldr	r12, [r11, #0x50]   @ Read the uart interrupt register into r12	\n\
			and     r12, #0x72         @ (AUART_INTR_RTIS | AUART_INTR_TXIS | AUART_INTR_RXIS | AUART_INTR_CTSMIS) 	\n\
			str     r12, [r11, #0x58]  @ uart_base + AUART_INTR_CLR \n\
										\n\
			@ Restore the unbanked registers and return from the FIQ handler	\n\
			ldmia sp!, {r0-r7,lr}					\n\
			@ Return						\n\
			subs pc, lr, #4						\n\
		fiq_stub_end:							\n\
	");
}
#endif

static int mxs_auart_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(mxs_auart_dt_ids, &pdev->dev);
	struct mxs_auart_port *s;
	u32 version;
	int ret = 0;
	struct resource *r;
	struct device_node *np;

	struct pinctrl *pinctrl;

#ifndef UART_IRQ
	struct pt_regs regs;
#endif
	struct fiq_buffer *fb;

	np = pdev->dev.of_node;
	if (!np) {
		dev_err(&pdev->dev, "No device tree data available\n");
		return -EINVAL;
	}

	s = kzalloc(sizeof(struct mxs_auart_port), GFP_KERNEL);
	if (!s) {
		ret = -ENOMEM;
		goto out;
	}

	mxs_auart_fiq_data = kzalloc(sizeof(struct mxs_auart_data), GFP_KERNEL);
	if ( ! mxs_auart_fiq_data) {
		dev_err(&pdev->dev, "kzalloc failed\n");
		ret = -ENOMEM;
		goto out;
	}

	ret = serial_mxs_probe_dt(s, pdev);
	if (ret > 0)
		s->port.line = pdev->id < 0 ? 0 : pdev->id;
	else if (ret < 0)
		goto out_free;

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		goto out_free;
	}
	printk(KERN_DEBUG "mxs-fiq-auart.c: devm_pinctrl_get_select_default succeeded\n");

	if (of_id) {
		pdev->id_entry = of_id->data;
		s->devtype = pdev->id_entry->driver_data;
	}

	s->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(s->clk)) {
		ret = PTR_ERR(s->clk);
		dev_err(&pdev->dev, "mxs-fiq-auart.c: clk_get failed\n");
		goto out_free;
	}

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		ret = -ENXIO;
		goto out_free_clk;
	}

	s->port.mapbase = r->start;
	s->port.membase = ioremap(r->start, resource_size(r));
	s->port.ops = &mxs_auart_ops;
	s->port.iotype = UPIO_MEM;
	s->port.fifosize = 16;
	s->port.uartclk = clk_get_rate(s->clk);
	s->port.type = PORT_IMX;
	s->port.dev = s->dev = &pdev->dev;
	s->ctrl = 0;

#if defined(UART_FIQ)

	/* We will request a FIQ interrupt for mxs_auart_fiq_data->irq */

	/* first vIRQ corresponds to hwIRA 112, the muxed UART interrupts */
	mxs_auart_fiq_data->irq = irq_of_parse_and_map(np, 0);
	if (mxs_auart_fiq_data->irq < 0) {
		dev_err(&pdev->dev, "Couldn't register given IRQ\n");
		return -EINVAL;
	}

	/* And an SVC interrupt for the timer in s->irq */ 
	/* Second vIRQ corresponds to hwIRQ 50, timrot2 */
	s->irq = platform_get_irq(pdev, 1);

#elif defined(TIMER_FIQ)

	/* Second vIRQ corresponds to hwIRQ 50, timrot2 */
	mxs_auart_fiq_data->irq = platform_get_irq(pdev, 1);

	/* Set an invalid value, because we aren't using this interrupt in SVC mode */
	s->irq = -1;

#elif defined(UART_IRQ)

	/* First vIRQ corresponds to hwIRQ 112, the muxed UART interrupts */
	s->irq = platform_get_irq(pdev, 0);
	s->port.irq = s->irq;

#else
	error "No interrupt handler mode defined"
#endif

#if defined(TIMER_FIQ) || defined(UART_FIQ)

	np = of_find_compatible_node(NULL, NULL, "fsl,imx28-timrot");
	mxs_auart_fiq_data->timrot_base = of_iomap(np, 0);
	if (!mxs_auart_fiq_data->timrot_base) {
		dev_err(&pdev->dev, "No fsl,imx28-timrot\n");
		return -ENOMEM;
	}
#endif
	np = of_find_compatible_node(NULL, NULL, "fsl,imx28-icoll");
	mxs_auart_fiq_data->icoll_base = of_iomap(np, 0);
	if (!mxs_auart_fiq_data->icoll_base) {
		dev_err(&pdev->dev, "No fsl,imx28-icoll\n");
		return -ENOMEM;
	}

	mxs_auart_fiq_data->fiq_base = dma_zalloc_coherent(&pdev->dev,
		FIQ_BUFFER_SIZE,
		&mxs_auart_fiq_data->dma,
		GFP_KERNEL);

	if ( ! mxs_auart_fiq_data->fiq_base) {
		dev_err(&pdev->dev, "Couldn't allocate memory\n");
		return -ENOMEM;
	}
	printk(KERN_DEBUG "fiq_base is [0x%lx]\n",
		(long unsigned)mxs_auart_fiq_data->fiq_base);

	dev_info(&pdev->dev,
		 "Allocated pages at address 0x%p, with size %dMB\n",
		 mxs_auart_fiq_data->fiq_base, FIQ_BUFFER_SIZE >> 20);

	mxs_auart_fiq_buf = mxs_auart_fiq_data->fiq_base + FIQ_STACK_SIZE;
	fb = ((struct fiq_buffer *) mxs_auart_fiq_buf);

	fb->rx_tail = 0;
	fb->tx_tail = 0;
	fb->rx_head = 0;
	fb->tx_head = 0;

#ifndef UART_IRQ
	ret = claim_fiq(&mxs_auart_fh);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't claim fiq\n");
		return ret;
	}

	set_fiq_handler(&fiq_stub, &fiq_stub_end - &fiq_stub);

	regs.ARM_r8	= (long)mxs_auart_fiq_data->timrot_base;
	regs.ARM_r9	= (long)fiq_handler_c_function;
	regs.ARM_r10	= (long)mxs_auart_fiq_buf;		// "sl", the fiq_buf start address
	regs.ARM_fp	= (long)s->port.membase;	/* "r11 */
	// r12, ip is unused
	regs.ARM_sp	= (long)mxs_auart_fiq_buf;		// "r13" stack grows down from here
	set_fiq_regs(&regs);
#endif

#ifdef TIMER_FIQ
	// Overrun after 344 chars at 9600 Baud
	// writel(175, mxs_auart_fiq_data->timrot_base + TIMROT_FIXED_COUNT_REG(2));	// 174 us is 1 UART char, required if not using the PL011 in interrupt mode

	writel(175, mxs_auart_fiq_data->timrot_base + TIMROT_FIXED_COUNT_REG(2));	// 174 us is 1 UART char, required if not using the PL011 in interrupt mode

	writel((
		TIMROT_TIMCTRL_IRQ_EN |
		TIMROT_TIMCTRL_RELOAD |
		TIMROT_TIMCTRL_UPDATE |
		TIMROT_TIMCTRL_ALWAYS_TICK
		), mxs_auart_fiq_data->timrot_base + TIMROT_TIMCTRL_REG(2));

	/* Enable the FIQ, 50 is timer2_irq See page 136 */

	dev_info(&pdev->dev, "mxs-fiq-auart.c (%d): TIMER_FIQ mode, enabling FIQ for vIRQ [%d], hwIRQ [%lu]\n",
		__LINE__,
		mxs_auart_fiq_data->irq,
		irq_get_irq_data(mxs_auart_fiq_data->irq)->hwirq);

	writel(1 << 4, mxs_auart_fiq_data->icoll_base + HW_ICOLL_INTERRUPTn_SET(irq_get_irq_data(mxs_auart_fiq_data->irq)->hwirq));
	enable_fiq(mxs_auart_fiq_data->irq);

#elif defined(UART_FIQ)
	/* Disable the timer here */
	writel(TIMROT_TIMCTRL_IRQ_EN, mxs_auart_fiq_data->timrot_base + TIMROT_FIXED_COUNT_REG(2) + 0x8);

	dev_info(&pdev->dev, "mxs-fiq-auart.c (%d): UART_FIQ mode, enabling FIQ for vIRQ [%d], hwIRQ [%lu]\n",
		__LINE__,
		mxs_auart_fiq_data->irq,
		irq_get_irq_data(mxs_auart_fiq_data->irq)->hwirq);

	writel(1 << 4, mxs_auart_fiq_data->icoll_base + HW_ICOLL_INTERRUPTn_SET(irq_get_irq_data(mxs_auart_fiq_data->irq)->hwirq));
	enable_fiq(mxs_auart_fiq_data->irq);

#elif defined(UART_IRQ)
	/*
		The context here is mxs_auart_fiq_data
		This IRQ is the tx_window timer timeout
	*/
	ret = request_irq(s->irq, mxs_auart_irq_handle, 0, dev_name(&pdev->dev), mxs_auart_fiq_data);
	if (ret)
		goto out_free_clk;

	dev_info(&pdev->dev, "mxs-fiq-auart.c (%d): UART_IRQ mode, request_irq for muxed UART IRQ [%d] (hw [%lu]) succeeded\n",
		__LINE__,
		s->irq,
		irq_get_irq_data(s->irq)->hwirq);
#else
	error "No interrupt handler mode defined"
#endif

	platform_set_drvdata(pdev, s);

	auart_port[s->port.line] = s;

	mxs_auart_reset(&s->port);

	printk(KERN_DEBUG "mxs-fiq-auart.c: probe: s is at [0x%x]\n", (u32)s);
#ifndef UART_IRQ
	thread_init(s);
#endif
	ret = uart_add_one_port(&auart_driver, &s->port);
	if (ret)
		goto out_free_irq;

	version = readl(s->port.membase + AUART_VERSION);
	dev_info(&pdev->dev, "Found FIQUART %u.%u.%u\n",
	       (version >> 24) & 0xff,
	       (version >> 16) & 0xff, version & 0xffff);

	return 0;

out_free_irq:
	auart_port[pdev->id] = NULL;
	free_irq(s->irq, s);
out_free_clk:
	clk_put(s->clk);
out_free:
	kfree(s);
out:
	return ret;
}

static int mxs_auart_remove(struct platform_device *pdev)
{
	struct mxs_auart_port *s = platform_get_drvdata(pdev);

	uart_remove_one_port(&auart_driver, &s->port);

	auart_port[pdev->id] = NULL;

	clk_put(s->clk);
	free_irq(s->irq, s);
	kfree(s);

	return 0;
}

static struct platform_driver mxs_auart_driver = {
	.probe = mxs_auart_probe,
	.remove = mxs_auart_remove,
	.driver = {
		.name = "mxs-fiq-auart",
		.owner = THIS_MODULE,
		.of_match_table = mxs_auart_dt_ids,
	},
};

static int __init mxs_auart_init(void)
{
	int r;

	r = uart_register_driver(&auart_driver);
	if (r)
		goto out;

	r = platform_driver_register(&mxs_auart_driver);
	if (r)
		goto out_err;

	return 0;
out_err:
	uart_unregister_driver(&auart_driver);
out:
	return r;
}

static void __exit mxs_auart_exit(void)
{
#ifndef UART_IRQ
	thread_exit();
#endif
	platform_driver_unregister(&mxs_auart_driver);
	uart_unregister_driver(&auart_driver);
}

module_init(mxs_auart_init);
module_exit(mxs_auart_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Freescale MXS application uart fiq driver");
MODULE_ALIAS("platform:mxs-fiq-auart");
