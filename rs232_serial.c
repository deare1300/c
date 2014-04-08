/**
  base web site:
	http://blog.chinaunix.net/uid-23089249-id-34481.html
http://blog.csdn.net/huangkangying/article/details/8070945
**/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>       /* printk() */
#include <linux/slab.h>         /* kmalloc() */
#include <linux/fs.h>           /* everything... */
#include <linux/errno.h>        /* error codes */
#include <linux/types.h>        /* size_t */
#include <linux/fcntl.h>        /* O_ACCMODE */
//#include <asm/system.h>         /* cli(), *_flags */
#include <asm/uaccess.h>        /* copy_*_user */
#include <linux/ioctl.h>
#include <linux/device.h>
#include <linux/interrupt.h>

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
#define FIFO_SIZE 16
#define MAP_SIZE (0X100)

#define recieve_enabled(port) ((port)->unused[0])
#define send_enabled(port) ((port)->unused[1])
#define RCV_IRQ(port) ((port)->irq + 1)
#define SEND_IRQ(port) ((port)->irq)


#define portaddr(port, reg) ((port)->membase + (reg))

/* 读8位宽的寄存器 */
#define rd_regb(port, reg) (ioread8(portaddr(port, reg)))
/* 读32位宽的寄存器 */
#define rd_regl(port, reg) (ioread32(portaddr(port, reg)))
/* 写8位宽的寄存器 */
#define wr_regb(port, reg, val) \
    do { iowrite8(val, portaddr(port, reg)); } while(0)
/* 写32位宽的寄存器 */        
#define wr_regl(port, reg, val) \
    do { iowrite32(val, portaddr(port, reg)); } while(0)

#define IRQ_S3CUART_RX2 37
#define IRQ_RS232_RX1 4
static long tick;
static struct uart_driver rs232_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = DRIVER_NAME,
	.dev_name = DEVICE_NAME,
	.major = RS232_MAJOR,
	.minor = RS232_MINOR,
	.nr = 1,
};

/*
static struct uart_port rs232_uart_port = {
	.irq      = IRQ_S3CUART_RX2,
	.fifosize = FIFO_SIZE,
	.iotype   = UPIO_MEM,
	.flags    = UPF_BOOT_AUTOCONF,
	.ops      = &rs232_uart_ops;
	.line     = 0,
	.lock     = __SPIN_LOCK_UNLOCKED(rs232_uart_port.lock),
};
*/
/*
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
*/

static void rs232_uart_stop_tx(struct uart_port *port)
{
	if(recieve_enabled(port))
	{
		disable_irq(SEND_IRQ(port));
		send_enabled(port) = 0;
	}
}

static void rs232_uart_start_tx(struct uart_port *port)
{
	if(!send_enabled(port))
	{
		enable_irq(SEND_IRQ(port));
	}
}


static void rs232_uart_stop_rx(struct uart_port *port)
{
	if(recieve_enabled(port))
	{
		disable_irq(RCV_IRQ(port));
		recieve_enabled(port) = 0;
	}
}


static unsigned int rs232_uart_tx_empty(struct uart_port *port)
{
    int ret = 1;
    unsigned long ufstat = rd_regl(port, S3C2410_UFSTAT);
    unsigned long ufcon = rd_regl(port, S3C2410_UFCON);

    if (ufcon & S3C2410_UFCON_FIFOMODE)    /* 若使能了FIFO */
    {
        if ((ufstat & S3C2410_UFSTAT_TXMASK) != 0 ||    /* 0 <FIFO <=15 */
                (ufstat & S3C2410_UFSTAT_TXFULL))       /* FIFO满 */
            ret = 0;
    }
    else    /* 若未使能了FIFO,则判断发送缓存和发送移位寄存器是否均为空 */
    {
        ret = rd_regl(port, S3C2410_UTRSTAT) & S3C2410_UTRSTAT_TXE;
    }
            
    return ret;
}


static unsigned int rs232_uart_get_mctrl(struct uart_port *port)
{
	return (TIOCM_CTS | TIOCM_DSR | TIOCM_CAR);
}

/* 使能modem的状态信号 */
static void rs232_uart_enable_ms(struct uart_port *port)
{
}

/* 设置串口modem控制 */
static void rs232_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{

}



static void rs232_uart_break_ctl(struct uart_port *port, int break_state)
{
    unsigned long flags;
    unsigned int ucon;

    spin_lock_irqsave(&port->lock, flags);

    ucon = rd_regl(port, S3C2410_UCON);

    if (break_state)
        ucon |= S3C2410_UCON_SBREAK;
    else
        ucon &= ~S3C2410_UCON_SBREAK;

    wr_regl(port, S3C2410_UCON, ucon);

    spin_unlock_irqrestore(&port->lock, flags);
}


static int rs232_uart_rx_fifocnt(unsigned long ufstat)
{
    /* 若Rx FIFO已满,返回FIFO的大小 */
    if (ufstat & S3C2410_UFSTAT_RXFULL)
        return FIFO_SIZE;

    /* 若FIFO未满,返回Rx FIFO已存了多少字节数据 */
    return (ufstat & S3C2410_UFSTAT_RXMASK) >> S3C2410_UFSTAT_RXSHIFT;
}


static irqreturn_t rs232_uart_rx_chars(int irq, void *dev_id)
{
    struct uart_port *port = dev_id;
//    struct tty_struct *tty = port->info->tty;
    unsigned int ufcon, ch, flag, ufstat, uerstat;
    int max_count = 64;

    /* 循环接收数据,最多一次中断接收64字节数据 */
    while (max_count-- > 0)
    {
        ufcon = rd_regl(port, S3C2410_UFCON);
        ufstat = rd_regl(port, S3C2410_UFSTAT);

        /* 若Rx FIFO无数据了,跳出循环 */
        if (rs232_uart_rx_fifocnt(ufstat) == 0)
            break;

        /* 读取Rx error状态寄存器 */
        uerstat = rd_regl(port, S3C2410_UERSTAT);
        /* 读取已接受到的数据 */
        ch = rd_regb(port, S3C2410_URXH);

        /* insert the character into the buffer */
        /* 先将tty标志设为正常 */
        flag = TTY_NORMAL;
        /* 递增接收字符计数器 */
        port->icount.rx++;

        /* 判断是否存在Rx error
         * if (unlikely(uerstat & S3C2410_UERSTAT_ANY))等同于
         * if (uerstat & S3C2410_UERSTAT_ANY)
         * 只是unlikely表示uerstat & S3C2410_UERSTAT_ANY的值为假的可能性大一些
         * 另外还有一个likely(value)表示value的值为真的可能性更大一些
         */
        if (unlikely(uerstat & S3C2410_UERSTAT_ANY))
        {
            /* 若break错误,递增icount.brk计算器 */
            if (uerstat & S3C2410_UERSTAT_BREAK)
            {
                port->icount.brk++;
                if (uart_handle_break(port))
                 goto ignore_char;
            }

            /* 若frame错误,递增icount.frame计算器 */
            if (uerstat & S3C2410_UERSTAT_FRAME)
                port->icount.frame++;
            /* 若overrun错误,递增icount.overrun计算器 */
            if (uerstat & S3C2410_UERSTAT_OVERRUN)
                port->icount.overrun++;

            /* 查看我们是否关心该Rx error
             * port->read_status_mask保存着我们感兴趣的Rx error status
             */
            uerstat &= port->read_status_mask;

            /* 若我们关心该Rx error,则将flag设置为对应的error flag */
            if (uerstat & S3C2410_UERSTAT_BREAK)
                flag = TTY_BREAK;
            else if (uerstat & S3C2410_UERSTAT_PARITY)
                flag = TTY_PARITY;
            else if (uerstat & ( S3C2410_UERSTAT_FRAME | S3C2410_UERSTAT_OVERRUN))
                flag = TTY_FRAME;
        }

        /* 处理sys字符 */
        if (uart_handle_sysrq_char(port, ch))
            goto ignore_char;

        /* 将接收到的字符插入到tty设备的flip缓冲 */
        uart_insert_char(port, uerstat, S3C2410_UERSTAT_OVERRUN, ch, flag);

ignore_char:
        continue;
    }
    
    /* 刷新tty设备的flip缓冲,将接受到的数据传给行规则层 */
//    tty_flip_buffer_push(tty);

    return IRQ_HANDLED;
}

/* 串口发送中断处理函数,将用户空间的数据(保存在环形缓冲xmit里)发送出去 */
static irqreturn_t rs232_uart_tx_chars(int irq, void *dev_id)
{
    return IRQ_HANDLED;
}


static int rs232_uart_start_up(struct uart_port *port)
{
    unsigned long flags;
    int ret;
    const char *portname = to_platform_device(port->dev)->name;

    /* 设置串口为不可接受,也不可发送 */
    recieve_enabled(port) = 0;
    send_enabled(port) = 0;

    spin_lock_irqsave(&port->lock, flags);

    /* 申请接收中断 */
    ret = request_irq(RCV_IRQ(port), rs232_uart_rx_chars, 0, portname, port);
    if (ret != 0)
    {
        printk(KERN_ERR "cannot get recieve irq\n");
        return ret;
    }    

    /* 设置串口为允许接收 */
    recieve_enabled(port) = 1;

    /* 申请发送中断 */
    ret = request_irq(SEND_IRQ(port), rs232_uart_tx_chars, 0, portname, port);
    if (ret)
    {
        printk(KERN_ERR "cannot get send irq \n");
        recieve_enabled(port) = 0;
        free_irq(RCV_IRQ(port), port);
        goto err;
    }    
    
    /* 设置串口为允许发送 */
    send_enabled(port) = 1;

err:
    spin_unlock_irqrestore(&port->lock, flags);
    return ret;
}


static void rs232_uart_shutdown(struct uart_port *port)
{
	recieve_enabled(port) = 0;
	free_irq(RCV_IRQ(port), port);
	send_enabled(port) = 0;
	free_irq(SEND_IRQ(port), port);
}


/* 设置串口参数 */
static void rs232_uart_set_termios(struct uart_port *port,
                 struct ktermios *termios,
                 struct ktermios *old)
{
    unsigned long flags;
    unsigned int baud, quot;
    unsigned int ulcon, ufcon = 0;

    /* 不支持moden控制信号线
     * HUPCL:    关闭时挂断moden
     * CMSPAR:    mark or space (stick) parity
     * CLOCAL:    忽略任何moden控制线
     */
    termios->c_cflag &= ~(HUPCL | CMSPAR);
    termios->c_cflag |= CLOCAL;

    /* 获取用户设置的串口波特率,并计算分频数(串口波特率除数quot) */
    baud = uart_get_baud_rate(port, termios, old, 0, 115200*8);
    if (baud == 38400 && (port->flags & UPF_SPD_MASK) == UPF_SPD_CUST)
        quot = port->custom_divisor;
    else
        quot = port->uartclk / baud / 16 - 1;

    /* 设置数据字长 */
    switch (termios->c_cflag & CSIZE)
    {
    case CS5:
        ulcon = S3C2410_LCON_CS5;
        break;
    case CS6:
        ulcon = S3C2410_LCON_CS6;
        break;
    case CS7:
        ulcon = S3C2410_LCON_CS7;
        break;
    case CS8:
    default:
        ulcon = S3C2410_LCON_CS8;
        break;
    }

    /* 是否要求设置两个停止位(CSTOPB) */        
    if (termios->c_cflag & CSTOPB)
        ulcon |= S3C2410_LCON_STOPB;

    /* 是否使用奇偶检验 */
    if (termios->c_cflag & PARENB)
    {
        if (termios->c_cflag & PARODD)  /* 奇校验 */
            ulcon |= S3C2410_LCON_PODD;
        else                            /* 偶校验 */
            ulcon |= S3C2410_LCON_PEVEN;
    }
    else                                /* 无校验 */
    {
        ulcon |= S3C2410_LCON_PNONE;
    }

    if (port->fifosize > 1)
        ufcon |= S3C2410_UFCON_FIFOMODE | S3C2410_UFCON_RXTRIG8;

    spin_lock_irqsave(&port->lock, flags);

    /* 设置FIFO控制寄存器、线控制寄存器和波特率除数寄存器 */
    wr_regl(port, S3C2410_UFCON, ufcon);
    wr_regl(port, S3C2410_ULCON, ulcon);
    wr_regl(port, S3C2410_UBRDIV, quot);

    /* 更新串口FIFO的超时时限 */
    uart_update_timeout(port, termios->c_cflag, baud);

    /* 设置我们感兴趣的Rx error */
    port->read_status_mask = S3C2410_UERSTAT_OVERRUN;
    if (termios->c_iflag & INPCK)
        port->read_status_mask |= S3C2410_UERSTAT_FRAME | S3C2410_UERSTAT_PARITY;

    /* 设置我们忽略的Rx error */
    port->ignore_status_mask = 0;
    if (termios->c_iflag & IGNPAR)
        port->ignore_status_mask |= S3C2410_UERSTAT_OVERRUN;
    if (termios->c_iflag & IGNBRK && termios->c_iflag & IGNPAR)
        port->ignore_status_mask |= S3C2410_UERSTAT_FRAME;

    /* 若未设置CREAD(使用接收器),则忽略所有Rx error*/
    if ((termios->c_cflag & CREAD) == 0)
        port->ignore_status_mask |= RXSTAT_DUMMY_READ;

    spin_unlock_irqrestore(&port->lock, flags);
}


/* 获取串口类型 */
static const char *rs232_uart_type(struct uart_port *port)
{/* 返回描述串口类型的字符串指针 */
    return port->type == PORT_S3C2410 ? "rs232_uart:s3c2410_uart2" : NULL;
}


static int rs232_uart_request_port(struct uart_port *port)
{
    struct resource *res;
    const char *name = to_platform_device(port->dev)->name;

    /* request_mem_region请求分配IO内存,从开始port->mapbase,大小MAP_SIZE
     * port->mapbase保存当前串口的寄存器基地址(物理)
     * uart2: 0x50008000
     */
    res = request_mem_region(port->mapbase, MAP_SIZE, name);
    if (res == NULL)
    {
        printk(KERN_ERR"request_mem_region error: %p\n", res);
        return -EBUSY;
    }
    
    return 0;
}


static void rs232_uart_release_port(struct uart_port *port)
{
    /* 释放已分配IO内存 */
    release_mem_region(port->mapbase, MAP_SIZE);
}


static void rs232_uart_config_port(struct uart_port *port, int flags)
{    
    int retval;

    /* 请求串口 */
    retval = rs232_uart_request_port(port);
    /* 设置串口类型 */
    if (flags & UART_CONFIG_TYPE && retval == 0)
        port->type = PORT_S3C2410;
}





static int rs232_uart_init_port(struct uart_port *port, struct platform_device *platdev)
{
    unsigned long flags;
    unsigned int gphcon;
    
    if (platdev == NULL)
        return -ENODEV;

    port->dev        = &platdev->dev;

    /* 设置串口波特率时钟频率 */
    port->uartclk    = clk_get_rate(clk_get(&platdev->dev, "pclk"));

    /* 设置串口的寄存器基地址(物理): 0x50008000 */
    port->mapbase    = S3C2410_PA_UART2;
    
    /* 设置当前串口的寄存器基地址(虚拟): 0xF5008000 */        
    port->membase    = S3C24XX_VA_UART + (S3C2410_PA_UART2 - S3C24XX_PA_UART);

    spin_lock_irqsave(&port->lock, flags);

    wr_regl(port, S3C2410_UCON, S3C2410_UCON_DEFAULT);
    wr_regl(port, S3C2410_ULCON, S3C2410_LCON_CS8 | S3C2410_LCON_PNONE);
    wr_regl(port, S3C2410_UFCON, S3C2410_UFCON_FIFOMODE
        | S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_RESETBOTH);

    /* 将I/O port H的gph6和gph7设置为TXD2和RXD2 */
    gphcon = readl(S3C2410_GPHCON);
    gphcon &= ~((0x5) << 12);
    writel(gphcon, S3C2410_GPHCON);
    
    spin_unlock_irqrestore(&port->lock, flags);
    
    return 0;
}


/**
plat driver
**/

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


static struct uart_port rs232_uart_port = {
	.irq      = IRQ_RS232_RX1,
	.fifosize = FIFO_SIZE,
	.iotype   = UPIO_MEM,
	.flags    = UPF_BOOT_AUTOCONF,
	.ops      = &rs232_uart_ops,
	.line     = 0,
	.lock     = __SPIN_LOCK_UNLOCKED(rs232_uart_port.lock),
};


static int __init rs232_uart_probe(struct platform_device *dev)
{
    int ret;
    
    /* 初始化串口 */
    ret = rs232_uart_init_port(&rs232_uart_port, dev);
    if (ret < 0)
    {
        printk(KERN_ERR"rs232_uart_probe: rs232_uart_init_port error: %d\n", ret);
        return ret;
    }    

    /* 添加串口 */
    ret = uart_add_one_port(&rs232_uart_driver, &rs232_uart_port);
    if (ret < 0)
    {
        printk(KERN_ERR"gprs_uart_probe: uart_add_one_port error: %d\n", ret);
        return ret;    
    }

    /* 将串口uart_port结构体保存在platform_device->dev->driver_data中 */
    platform_set_drvdata(dev, &rs232_uart_port);

    return 0;
}


static int rs232_uart_remove(struct platform_device *dev)
{
    platform_set_drvdata(dev, NULL);

    /* 移除串口 */
    uart_remove_one_port(&rs232_uart_driver, &rs232_uart_port);
    return 0;
}


static int rs232_uart_suspend(struct platform_device *dev, pm_message_t state)
{
    uart_suspend_port(&rs232_uart_driver, &rs232_uart_port);
    return 0;
}


static int rs232_uart_resume(struct platform_device *dev)
{
    uart_resume_port(&rs232_uart_driver, &rs232_uart_port);
    return 0;
}


/*
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


static struct uart_port rs232_uart_port = {
	.irq      = IRQ_S3CUART_RX2,
	.fifosize = FIFO_SIZE,
	.iotype   = UPIO_MEM,
	.flags    = UPF_BOOT_AUTOCONF,
	.ops      = &rs232_uart_ops;
	.line     = 0,
	.lock     = __SPIN_LOCK_UNLOCKED(rs232_uart_port.lock),
};
*/

/* Platform driver for GPRS_UART */
static struct platform_driver rs232_plat_driver = {
    .probe = rs232_uart_probe,                /* Probe method */
    .remove = __exit_p(rs232_uart_remove),    /* Detach method */
    .suspend = rs232_uart_suspend,            /* Power suspend */
    .resume = rs232_uart_resume,              /* Resume after a suspend */
    .driver = {
        .owner    = THIS_MODULE,
        .name = DRIVER_NAME,                    /* Driver name */
    },
};


struct platform_device *rs232_plat_device; 


static int __init rs232_init_module(void)
{
    int retval;

    /* Register uart_driver for GPRS_UART */
    retval = uart_register_driver(&rs232_uart_driver);
    if (0 != retval)
    {
        printk(KERN_ERR "gprs_init_module: can't register the RS232_UART driver %d\n", retval);
        return retval;
    }

    /* Register platform device for GPRS_UART. Usually called
    during architecture-specific setup */
    rs232_plat_device = platform_device_register_simple(DEVICE_NAME, 0, NULL, 0);
    if (IS_ERR(rs232_plat_device)) 
    {
        retval = PTR_ERR(rs232_plat_device);
        printk(KERN_ERR "gprs_init_module: can't register platform device %d\n", retval);
        goto fail_reg_plat_dev;
    }

    /* Announce a matching driver for the platform
    devices registered above */
    retval = platform_driver_register(&rs232_plat_driver);
    if (0 != retval)
    {
        printk(KERN_ERR "gprs_init_module: can't register platform driver %d\n", retval);
        goto fail_reg_plat_drv;
    }

    return 0; /* succeed */

fail_reg_plat_drv:
    platform_device_unregister(rs232_plat_device);
fail_reg_plat_dev:
    uart_unregister_driver(&rs232_uart_driver);
    return retval;
}


static void __exit rs232_exit_module(void)
{
    /* The order of unregistration is important. Unregistering the
    UART driver before the platform driver will crash the system */

    /* Unregister the platform driver */
    platform_driver_unregister(&rs232_plat_driver);

    /* Unregister the platform devices */
    platform_device_unregister(rs232_plat_device);

    /* Unregister the GPRS_UART driver */
    uart_unregister_driver(&rs232_uart_driver);
}


module_init(rs232_init_module);
module_exit(rs232_exit_module);

MODULE_AUTHOR("DEEN");
MODULE_LICENSE("Dual BSD/GPL");
