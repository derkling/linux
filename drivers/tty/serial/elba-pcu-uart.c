/*
 *  linux/drivers/char/amba.c
 *
 *  Driver for ELBA PCU UART serial port
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright 1999 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * This is a generic driver for ARM AMBA-type serial ports.  They
 * have a lot of 16550-like features, but are not register compatible.
 * Note that although they do have CTS, DCD and DSR inputs, they do
 * not have an RI input, nor do they have DTR or RTS outputs.  If
 * required, these have to be supplied via some other means (eg, GPIO)
 * and hooked into this driver.
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/amba/serial.h>
#include <linux/debugfs.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/sizes.h>

#define UART_DR_ERROR		(UART011_DR_OE|UART011_DR_BE|UART011_DR_PE|UART011_DR_FE)
#define UART_DUMMY_DR_RX	(1 << 16)
#define PCU_UART_ISR_PASS_LIMIT 256

/* Protocol messages.  */
#define GET_VERSION_CMD         "\x00\x00"
#define GET_VERSION_RES         "\x01\x05\x00\x00\x00\x00\x00"

#define SET_RTC_CMD             "\x01\x04\x00\x00\x00\x00"
#define SET_RTC_RES             "\x01\x01\x00"

#define GET_RTC_CMD             "\x02\x00"
#define GET_RTC_RES             "\x02\x04\x00\x00\x00\x00"

#define SET_LCD_FREQUENCY_CMD   "\x03\x04\x00\x00\x00\x00"
#define SET_LCD_FREQUENCY_RES   "\x03\x01\x00"

#define GET_TEMP_SENS_CMD       "\x04\x00"
#define GET_TEMP_SENS_RES       "\x04\x01\x00"

#define GET_TEMP_SENS_INFO_CMD  "\x05\x01\x00"
#define GET_TEMP_SENS_INFO_RES  "\x05\x00" /* Variable lenght.  */

#define GET_TEMPERATURE_CMD     "\x06\x01\x00"
#define GET_TEMPERATURE_RES     "\x06\x04\x00"

#define GET_STATS_CMD           "\x07\x00"
#define GET_STATS_RES           "\x07\x04\x00\x00\x00\x00"

/* Command and response are inverted.  */
#define FAULT_REQUEST           "\x08\x02\x00\x00"
#define FAULT_RES               "\x08\x01\x00"

/*
 * We wrap our port structure around the generic uart_port.
 */
struct pcu_uart_port {
	struct uart_port	port;
	struct clk		*clk;
	unsigned int            ifls;
	unsigned int		im;	/* interrupt mask */
	unsigned int		old_status;
	bool			autorts;
} *uap;

/* UART methods.  */
void elba_pcu_uart_stop_tx(void)
{
	uap->im &= ~UART011_TXIM;
	writew(uap->im, uap->port.membase + UART011_IMSC);
}
EXPORT_SYMBOL(elba_pcu_uart_stop_tx);

void elba_pcu_uart_start_tx(void)
{
	uap->im |= UART011_TXIM;
	writew(uap->im, uap->port.membase + UART011_IMSC);
}
EXPORT_SYMBOL(elba_pcu_uart_start_tx);

void elba_pcu_uart_stop_rx(void)
{
	uap->im &= ~(UART011_RXIM|UART011_RTIM|UART011_FEIM|
		     UART011_PEIM|UART011_BEIM|UART011_OEIM);
	writew(uap->im, uap->port.membase + UART011_IMSC);
}
EXPORT_SYMBOL(elba_pcu_uart_stop_rx);

void elba_pcu_uart_enable_ms(void)
{
	uap->im |= UART011_RIMIM|UART011_CTSMIM|UART011_DCDMIM|UART011_DSRMIM;
	writew(uap->im, uap->port.membase + UART011_IMSC);
}
EXPORT_SYMBOL(elba_pcu_uart_enable_ms);

void elba_pcu_uart_rx_chars(void)
{
	struct tty_struct *tty = uap->port.state->port.tty;
	unsigned int status, ch, flag, max_count = 256;

	status = readw(uap->port.membase + UART01x_FR);
	while ((status & UART01x_FR_RXFE) == 0 && max_count--) {
		ch = readw(uap->port.membase + UART01x_DR) | UART_DUMMY_DR_RX;
		flag = TTY_NORMAL;
		uap->port.icount.rx++;

		/*
		 * Note that the error handling code is
		 * out of the main execution path
		 */
		if (unlikely(ch & UART_DR_ERROR)) {
			if (ch & UART011_DR_BE) {
				ch &= ~(UART011_DR_FE | UART011_DR_PE);
				uap->port.icount.brk++;
				if (uart_handle_break(&uap->port))
					goto ignore_char;
			} else if (ch & UART011_DR_PE)
				uap->port.icount.parity++;
			else if (ch & UART011_DR_FE)
				uap->port.icount.frame++;
			if (ch & UART011_DR_OE)
				uap->port.icount.overrun++;

			ch &= uap->port.read_status_mask;

			if (ch & UART011_DR_BE)
				flag = TTY_BREAK;
			else if (ch & UART011_DR_PE)
				flag = TTY_PARITY;
			else if (ch & UART011_DR_FE)
				flag = TTY_FRAME;
		}

		uart_insert_char(&uap->port, ch, UART011_DR_OE, ch, flag);

	ignore_char:
		status = readw(uap->port.membase + UART01x_FR);
	}
	spin_unlock(&uap->port.lock);
	tty_flip_buffer_push(tty);
	spin_lock(&uap->port.lock);
}
EXPORT_SYMBOL(elba_pcu_uart_rx_chars);

void elba_pcu_uart_tx_chars(void)
{
	struct circ_buf *xmit = &uap->port.state->xmit;
	int count;

	if (uap->port.x_char) {
		writew(uap->port.x_char, uap->port.membase + UART01x_DR);
		uap->port.icount.tx++;
		uap->port.x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&uap->port)) {
		elba_pcu_uart_stop_tx();
		return;
	}

	count = uap->port.fifosize >> 1;
	do {
		writew(xmit->buf[xmit->tail], uap->port.membase + UART01x_DR);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		uap->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&uap->port);

	if (uart_circ_empty(xmit))
		elba_pcu_uart_stop_tx();
}
EXPORT_SYMBOL(elba_pcu_uart_tx_chars);

void elba_pcu_uart_modem_status(void)
{
	unsigned int status, delta;

	status = readw(uap->port.membase + UART01x_FR) & UART01x_FR_MODEM_ANY;

	delta = status ^ uap->old_status;
	uap->old_status = status;

	if (!delta)
		return;

	if (delta & UART01x_FR_DCD)
		uart_handle_dcd_change(&uap->port, status & UART01x_FR_DCD);

	if (delta & UART01x_FR_DSR)
		uap->port.icount.dsr++;

	if (delta & UART01x_FR_CTS)
		uart_handle_cts_change(&uap->port, status & UART01x_FR_CTS);

	wake_up_interruptible(&uap->port.state->port.delta_msr_wait);
}
EXPORT_SYMBOL(elba_pcu_uart_modem_status);

void elba_pcu_uart_set_termios(struct uart_port *port, struct ktermios *termios,
			       struct ktermios *old)
{
	unsigned int lcr_h, old_cr;
	unsigned long flags;
	unsigned int baud, quot;

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
	quot = port->uartclk * 4 / baud;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr_h = UART01x_LCRH_WLEN_5;
		break;
	case CS6:
		lcr_h = UART01x_LCRH_WLEN_6;
		break;
	case CS7:
		lcr_h = UART01x_LCRH_WLEN_7;
		break;
	default: // CS8
		lcr_h = UART01x_LCRH_WLEN_8;
		break;
	}
	if (termios->c_cflag & CSTOPB)
		lcr_h |= UART01x_LCRH_STP2;
	if (termios->c_cflag & PARENB) {
		lcr_h |= UART01x_LCRH_PEN;
		if (!(termios->c_cflag & PARODD))
			lcr_h |= UART01x_LCRH_EPS;
	}
	if (port->fifosize > 1)
		lcr_h |= UART01x_LCRH_FEN;

	spin_lock_irqsave(&port->lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	port->read_status_mask = UART011_DR_OE | 255;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART011_DR_FE | UART011_DR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UART011_DR_BE;

	/*
	 * Characters to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART011_DR_FE | UART011_DR_PE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= UART011_DR_BE;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= UART011_DR_OE;
	}

	/*
	 * Ignore all characters if CREAD is not set.
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_DUMMY_DR_RX;

	if (UART_ENABLE_MS(port, termios->c_cflag))
		elba_pcu_uart_enable_ms();

	/* first, disable everything */
	old_cr = readw(port->membase + UART011_CR);
	writew(0, port->membase + UART011_CR);

	if (termios->c_cflag & CRTSCTS) {
		if (old_cr & UART011_CR_RTS)
			old_cr |= UART011_CR_RTSEN;

		old_cr |= UART011_CR_CTSEN;
		uap->autorts = true;
	} else {
		old_cr &= ~(UART011_CR_CTSEN | UART011_CR_RTSEN);
		uap->autorts = false;
	}

	/* Set baud rate */
	writew(quot & 0x3f, port->membase + UART011_FBRD);
	writew(quot >> 6, port->membase + UART011_IBRD);

	/*
	 * ----------v----------v----------v----------v-----
	 * NOTE: MUST BE WRITTEN AFTER UARTLCR_M & UARTLCR_L
	 * ----------^----------^----------^----------^-----
	 */
	writew(lcr_h, port->membase + UART011_LCRH);
	writew(old_cr, port->membase + UART011_CR);

	spin_unlock_irqrestore(&port->lock, flags);
}
EXPORT_SYMBOL(elba_pcu_uart_set_termios);

/* Interrupt handler.  */
static irqreturn_t elba_pcu_uart_int(int irq, void *dev_id)
{
	unsigned int status, pass_counter = PCU_UART_ISR_PASS_LIMIT;
	int handled = 0;

	spin_lock(&uap->port.lock);

	status = readw(uap->port.membase + UART011_MIS);
	if (status) {
		do {
			writew(status & ~(UART011_TXIS|UART011_RTIS|
					  UART011_RXIS),
			       uap->port.membase + UART011_ICR);

			if (status & (UART011_RTIS|UART011_RXIS))
				elba_pcu_uart_rx_chars();
			if (status & (UART011_DSRMIS|UART011_DCDMIS|
				      UART011_CTSMIS|UART011_RIMIS))
				elba_pcu_uart_modem_status();
			if (status & UART011_TXIS)
				elba_pcu_uart_tx_chars();

			if (pass_counter-- == 0)
				break;

			status = readw(uap->port.membase + UART011_MIS);
		} while (status != 0);
		handled = 1;
	}

	spin_unlock(&uap->port.lock);

	return IRQ_RETVAL(handled);
}

int elba_pcu_uart_startup(void)
{
	unsigned int cr;
	int retval;

	/*
	 * Try to enable the clock producer.
	 */
	retval = clk_enable(uap->clk);
	if (retval)
		goto out;

	uap->port.uartclk = clk_get_rate(uap->clk);

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(uap->port.irq, elba_pcu_uart_int, 0, "uart-elba_pcu_uart", uap);
	if (retval)
		goto clk_dis;

	writew(uap->ifls, uap->port.membase + UART011_IFLS);

	/*
	 * Provoke TX FIFO interrupt into asserting.
	 */
	cr = UART01x_CR_UARTEN | UART011_CR_TXE | UART011_CR_LBE;
	writew(cr, uap->port.membase + UART011_CR);
	writew(0, uap->port.membase + UART011_FBRD);
	writew(1, uap->port.membase + UART011_IBRD);
	writew(0, uap->port.membase + UART011_LCRH);
	writew(0, uap->port.membase + UART01x_DR);
	while (readw(uap->port.membase + UART01x_FR) & UART01x_FR_BUSY)
		barrier();

	cr = UART01x_CR_UARTEN | UART011_CR_RXE | UART011_CR_TXE;
	writew(cr, uap->port.membase + UART011_CR);

	/*
	 * initialise the old status of the modem signals
	 */
	uap->old_status = readw(uap->port.membase + UART01x_FR) & UART01x_FR_MODEM_ANY;

	/*
	 * Finally, enable interrupts
	 */
	spin_lock_irq(&uap->port.lock);
	uap->im = UART011_RXIM | UART011_RTIM;
	writew(uap->im, uap->port.membase + UART011_IMSC);
	spin_unlock_irq(&uap->port.lock);

	return 0;

 clk_dis:
	clk_disable(uap->clk);
 out:
	return retval;
}
EXPORT_SYMBOL(elba_pcu_uart_startup);

void elba_pcu_uart_shutdown(void)
{
	unsigned long val;

	/*
	 * disable all interrupts
	 */
	spin_lock_irq(&uap->port.lock);
	uap->im = 0;
	writew(uap->im, uap->port.membase + UART011_IMSC);
	writew(0xffff, uap->port.membase + UART011_ICR);
	spin_unlock_irq(&uap->port.lock);

	/*
	 * Free the interrupt
	 */
	free_irq(uap->port.irq, uap);

	/*
	 * disable the port
	 */
	uap->autorts = false;
	writew(UART01x_CR_UARTEN | UART011_CR_TXE, uap->port.membase + UART011_CR);

	/*
	 * disable break condition and fifos
	 */
	val = readw(uap->port.membase + UART011_LCRH);
	val &= ~(UART01x_LCRH_BRK | UART01x_LCRH_FEN);
	writew(val, uap->port.membase + UART011_LCRH);

	/*
	 * Shut down the clock producer
	 */
	clk_disable(uap->clk);
}
EXPORT_SYMBOL(elba_pcu_uart_shutdown);

/* PROTOCOL MESSAGES.  */

void get_version(void)
{
	return;
}
EXPORT_SYMBOL(get_version);

void set_rtc(void)
{
	return;
}
EXPORT_SYMBOL(set_rtc);

void get_rtc(void)
{
	return;
}
EXPORT_SYMBOL(get_rtc);

void set_lcd_frequency(void)
{
	return;
}
EXPORT_SYMBOL(set_lcd_frequency);

void get_temp_sens(void)
{
	return;
}
EXPORT_SYMBOL(get_temp_sens);

void get_temp_sens_info(void)
{
	return;
}
EXPORT_SYMBOL(get_temp_sens_info);

void get_temperature(void)
{
	return;
}
EXPORT_SYMBOL(get_temperature);

void get_stats(void)
{
	return;
}
EXPORT_SYMBOL(get_stats);

void fault(void)
{
	return;
}
EXPORT_SYMBOL(fault);

/* Platform driver methods.  */
static int elba_pcu_uart_probe(struct platform_device *dev)
{
	void __iomem *base;
	int ret = 0;

	uap = kzalloc(sizeof(struct pcu_uart_port), GFP_KERNEL);
	if (uap == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	base = ioremap(dev->resource->start, resource_size(dev->resource));
	if (!base) {
		ret = -ENOMEM;
		goto free;
	}

	uap->clk = clk_get(&dev->dev, NULL);
	if (IS_ERR(uap->clk)) {
		ret = PTR_ERR(uap->clk);
		goto unmap;
	}

	uap->ifls = UART011_IFLS_RX4_8|UART011_IFLS_TX4_8; /* ARM vendor.  */
	uap->port.dev = &dev->dev;
	uap->port.mapbase = dev->resource->start;
	uap->port.membase = base;
	uap->port.iotype = UPIO_MEM;
	// uap->port.irq = dev->irq[0];
	uap->port.fifosize = 16;
	uap->port.ops = NULL;
	uap->port.flags = UPF_BOOT_AUTOCONF;
	uap->port.line = 0; /* We only have 1 serial port.  */
	platform_set_drvdata(dev, uap);

	/* Activate UART.  */


out:
	return ret;
unmap:
	iounmap(base);
free:
	kfree(uap);
	goto out;
}

static int elba_pcu_uart_remove(struct platform_device *dev)
{
	struct pcu_uart_port *uap = platform_get_drvdata(dev);

	/* Deactivate UART.  */


	platform_set_drvdata(dev, NULL);
	iounmap(uap->port.membase);
	clk_put(uap->clk);
	kfree(uap);

	return 0;
}

static struct platform_driver elba_pcu_uart_driver = {
	.driver = {
		.name	= "elba_pcu_uart",
		.owner  = THIS_MODULE,
	},
	.probe		= elba_pcu_uart_probe,
	.remove		= elba_pcu_uart_remove,
};

#if defined (CONFIG_DEBUG_FS)
/* Root dentry.  */
static struct dentry *pcu_uart_root;
/* Message.  */
static struct dentry *pcu_uart_msg_file;
static long pcu_uart_msg_value;

static ssize_t pcu_uart_msg_read(struct file *file, char __user *user_buf, size_t len, loff_t *ppos)
{
	char buffer[512];
	unsigned int count = 0;

	count += snprintf(buffer, sizeof(buffer), "Wakeup Interrupt mask: 0x%08lX\n", pcu_uart_msg_value);

	return simple_read_from_buffer(user_buf, len, ppos, buffer, count);
}

static ssize_t pcu_uart_msg_write(struct file *file, const char __user *user_buf, size_t len, loff_t *ppos)
{
	char buffer[20];

	if (copy_from_user(buffer, user_buf, min(len, sizeof(buffer))))
		return -EFAULT;

	pcu_uart_msg_value = simple_strtol(buffer, NULL, 16);

	// send message through UART

	return len;
}

static const struct file_operations pcu_uart_msg_fops = {
	.owner   = THIS_MODULE,
	.read    = pcu_uart_msg_read,
	.write   = pcu_uart_msg_write,
};

void pcu_uart_create_debugfs(void)
{
	pcu_uart_root = debugfs_create_dir("pcu_uart", NULL);

	pcu_uart_msg_file = debugfs_create_file("message",
						 S_IWUGO,
						 pcu_uart_root,
						 &pcu_uart_msg_value,
						 &pcu_uart_msg_fops);
}

void pcu_uart_remove_debugfs(void)
{
	if (pcu_uart_root) {
		debugfs_remove(pcu_uart_msg_file);
		debugfs_remove(pcu_uart_root);
		pcu_uart_root = 0;
	}
}
#endif


static int __init elba_pcu_uart_init(void)
{
	int ret;

	ret = platform_driver_register(&elba_pcu_uart_driver);

#if defined (CONFIG_DEBUG_FS)
	pcu_uart_create_debugfs();
#endif

	return ret;
}

static void __exit elba_pcu_uart_exit(void)
{
#if defined (CONFIG_DEBUG_FS)
	pcu_uart_remove_debugfs();
#endif

	platform_driver_unregister(&elba_pcu_uart_driver);
}

/*
 * While this can be a module, if builtin it's most likely the console
 * So let's leave module_exit but move module_init to an earlier place
 */
module_init(elba_pcu_uart_init);
module_exit(elba_pcu_uart_exit);

MODULE_AUTHOR("Giuseppe Calderaro - <giuseppe.calderaro@arm.com>");
MODULE_DESCRIPTION("ARM ELBA PCU UART driver");
MODULE_LICENSE("GPL");
