#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>       /* printk() */
#include <linux/slab.h>         /* kmalloc() */
#include <linux/fs.h>           /* everything... */
#include <linux/errno.h>        /* error codes */
#include <linux/types.h>        /* size_t */
#include <linux/fcntl.h>        /* O_ACCMODE */
#include <asm/system.h>         /* cli(), *_flags */
#include <asm/uaccess.h>        /* copy_*_user */
#include <linux/ioctl.h>
#include <linux/device.h>

#include <linux/platform_device.h>
#include <linux/sysrq.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <asm/io.h>
#include <asm/irq.h>

#define DEVICE_NAME "rs232"
#define DRIVER_NAME "rs232 driver"
#define RS232_MAJOR 100
#define RS232_MINOR 0

static struct uart_driver rs232_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = DRIVER_NAME,
	.dev_name = DEVICE_NAME,
	.major = RS232_MAJOR,
	.minor = RS232_MINOR,
	.nr = 1,
};


static struct uart_ops rs232_uart_ops = {
	.start_tx     =   rs232_uart_start_tx,
	.stop_tx      =   rs232_uart_stop_tx,
	.stop_rx      =   rs232_uart_stop_rx,
	.enable_ms    =   rs232_uart_enable_ms,
	.tx_empty     =   rs232_uart_tx_empty,
	.get_mctrl    =   rs232_uart_get_mctrl,
	.set_mctrl    =   rs232_uart_set_mctrl,
	.break_ctl    =   rs232_uart_break_ctl,
	.startup      =   rs232_uart_start_up,
	.shutdown     =   rs232_uart_shutdown,
	.set_termios  =   rs232_uart_set_termios,
	.type         =   rs232_uart_type,
	.request_port =   rs232_uart_request_port,
	.release_port =   rs232_uart_release_port,
	.config_port  =   rs232_uart_config_port,
};
