// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019 Viking <wj.yao@uctechip.com>
 */

/*
 * File      : wh_usart.c
 *
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-20	  ViKing       first implementation
 * 2019-03-28     Xiexz
 */

#include <clk.h>
#include <common.h>
#include <debug_uart.h>
#include <dm.h>
#include <errno.h>
#include <fdtdec.h>
#include <watchdog.h>
#include <asm/io.h>
#include <linux/compiler.h>
#include <serial.h>

//#include <ns16550.h>
#include <reset.h>
#include <linux/types.h>

#include "serial_wh.h"

DECLARE_GLOBAL_DATA_PTR;

struct uart_wh {
	char is;
	char ie;
	char con;
	char data;
	char bprl;
	char bprh;
};

struct wh_uart_platdata {
	unsigned long clock;
	int saved_input_char;
	struct uart_wh *regs;
};

void uart_delay(int nms)
{
	volatile int i;

	while(nms--)
	{
		for(i = 0; i < 12; i++)
			;
	}
}

static void _wh_serial_setbrg(struct uart_wh *regs,
		unsigned long clock,unsigned baud)
{
	//clock,baud is not used.
	baud = 115200;
	int br;
	char br_l,br_h;
	writeb(0, &regs->ie);// close the interrupt
	br = CORE_FREQ / baud; //frequence is 48MHz(CORE_FREQ=0x2DC6C00)
	br_l = (char)(br % 256);
	br_h = (char)(br / 256);
	writeb(br_l, &regs->bprl);
	writeb(((readb(&regs->bprh) & 0xF0) | br_h), &regs->bprh);
}

static void _wh_serial_init(struct uart_wh *regs)
{
	writeb((readb(&regs->con)&(~UART_CON_ODD_EN)),&regs->con);
	writeb((readb(&regs->ie) | UART_IE_FIFO_NE),&regs->ie);
}

static int _wh_serial_putc(struct uart_wh *regs,const char c)
{
	writeb((readb(&regs->con)|(UART_CON_TRS)),&regs->con);

	writeb(c,&regs->data);
	uart_delay(10);

	while(!(readb(&regs->is) & UART_IS_TXEND))
		continue;

	writeb((readb(&regs->is) & (~UART_IS_TXEND)),&regs->is);
	writeb(((readb(&regs->bprh) & 0x0F) |0x50),&regs->bprh);//set UART_ECR with 3
	//writel((0x30 |br_h),&regs->bprh);//equal the last line code,but the curcuit have a little bug when read the device, so use this line code

	while(!(readb(&regs->is) & UART_IS_ECNT0));	//the etu didn't arrive default value(3)

	writeb((readb(&regs->is) & ~UART_IS_ECNT0),&regs->is);

	return 0 ;
}

static int _wh_serial_getc(struct uart_wh *regs)  //(Maybe char?)
{
	int ch;
	writeb((readb(&regs->con) & (~UART_CON_TRS)),&regs->con); //select receive mode

	if(readb(&regs->is) & UART_IS_TRE)
		writeb((readb(&regs->is) & (~UART_IS_TRE)),&regs->is);
	if(readb(&regs->is) & UART_IS_FIFO_OV)
		writeb((readb(&regs->con) | UART_CON_FLUSH),&regs->con);  //uart_fflush();
/*
	while(!(readb(&regs->is) & UART_IS_FIFO_NE))
		continue;

	writeb((readb(&regs->ie) | UART_IE_FIFO_NE),&regs->ie);

	return readb(&regs->data);
*/
	if (!(readb(&regs->is) & UART_IS_FIFO_NE))
		return -EAGAIN;
	writeb((readb(&regs->ie) | UART_IE_FIFO_NE),&regs->ie);
	ch=readb(&regs->data);
	return (!ch) ? -EAGAIN : ch;
}

static int wh_serial_setbrg(struct udevice *dev,int baudrate)
{
	int ret;
	struct clk clk;
	struct wh_uart_platdata *platdata = dev_get_platdata(dev);
	u32 clock = 0;

	ret = clk_get_by_index(dev, 0, &clk);
	if (IS_ERR_VALUE(ret)) {
		debug("WH UART failed to get clock\n");
		ret = dev_read_u32(dev, "clock-frequency", &clock);
		if (IS_ERR_VALUE(ret)) {
			debug("WH UART clock not defined\n");
			return 0;
		}
	} else {
		clock = clk_get_rate(&clk);
		if (IS_ERR_VALUE(clock)) {
			debug("WH UART clock get rate failed\n");
			return 0;
		}
	}
	platdata->clock = clock;
	_wh_serial_setbrg(platdata->regs, platdata->clock, baudrate);

	return 0 ;
}

static int wh_serial_probe(struct udevice *dev)
{
	struct wh_uart_platdata *platdata = dev_get_platdata(dev);

	// No need to reinitialize the UART after relocation (Maybe need?)
		if (gd->flags & GD_FLG_RELOC)
			return 0;
	_wh_serial_setbrg(platdata->regs,0,115200);
	_wh_serial_init(platdata->regs);

	platdata->saved_input_char = 0;
	//_wh_serial_putc(platdata->regs,'c');

	return 0;
}

static int wh_serial_getc(struct udevice *dev) //(Maybe char?)
{
	int c;
	struct wh_uart_platdata *platdata = dev_get_platdata(dev);
	struct uart_wh *regs = platdata->regs;

	//  pending
	if (platdata->saved_input_char > 0) {
		c = platdata->saved_input_char;
		platdata->saved_input_char = 0;
		return c;
	}


	//c = _wh_serial_getc(regs);
	while ((c = _wh_serial_getc(regs)) == -EAGAIN) ;
	return c;
}

static int wh_serial_putc(struct udevice *dev,const char ch)
{
	struct wh_uart_platdata *platdata = dev_get_platdata(dev);

	_wh_serial_putc(platdata->regs,ch);

	return 0;

}

static int wh_serial_pending(struct udevice *dev, bool input)
{
	struct wh_uart_platdata *platdata = dev_get_platdata(dev);
	struct uart_wh *regs = platdata->regs;
//	int c;
//	c = _wh_serial_getc(platdata->regs);
//	_wh_serial_putc(platdata->regs,c);

	if (input) {
			if (platdata->saved_input_char > 0)
				return 1;
			platdata->saved_input_char = _wh_serial_getc(regs);
			return (platdata->saved_input_char > 0) ? 1 : 0;
		} else {
			return !!(readb(&regs->is) & UART_IS_FIFO_FU);
		}

	//return 0 ;
}

static int wh_serial_ofdata_to_platdata(struct udevice *dev)
{
	struct wh_uart_platdata *platdata = dev_get_platdata(dev);

	platdata->regs = (struct uart_wh *)dev_read_addr(dev);
	if (IS_ERR(platdata->regs))
		return PTR_ERR(platdata->regs);

	return 0;
}


static const struct dm_serial_ops wh_serial_ops = {
	.putc = wh_serial_putc,
	.getc = wh_serial_getc,
	.pending = wh_serial_pending,
	.setbrg = wh_serial_setbrg,
};

static const struct udevice_id wh_serial_ids[] = {
	{ .compatible = "wh,uart0" },
	{ }
};

U_BOOT_DRIVER(uart_wh) = {
	.name	= "uart_wh",
	.id	= UCLASS_SERIAL,
	.of_match = wh_serial_ids,
	.ofdata_to_platdata = wh_serial_ofdata_to_platdata,
	.platdata_auto_alloc_size = sizeof(struct wh_uart_platdata),
	.probe = wh_serial_probe,
	.ops	= &wh_serial_ops,
};

/*
#ifdef CONFIG_DEBUG_UART_WH
static inline void _debug_uart_init(void)
{
	struct uart_wh *regs =
			(struct uart_wh *)CONFIG_DEBUG_UART_BASE;

	_wh_serial_setbrg(regs, CONFIG_DEBUG_UART_CLOCK,
			      CONFIG_BAUDRATE);
	_wh_serial_init(regs);
}

static inline void _debug_uart_putc(int ch)
{
	struct uart_wh *regs =
			(struct uart_wh *)CONFIG_DEBUG_UART_BASE;

	while (_sifive_serial_putc(regs, ch) == -EAGAIN) //
		WATCHDOG_RESET();
}

DEBUG_UART_FUNCS

#endif
*/
