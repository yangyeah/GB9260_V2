/*
 * Dallas DS1302 RTC Support
 *
 *  Copyright (C) 2002  David McCullough
 *  Copyright (C) 2003 - 2007  Paul Mundt
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License version 2.  See the file "COPYING" in the main directory of
 * this archive for more details.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/bcd.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/slab.h>

//#include <asm/rtc.h>

#define DRV_NAME	"rtc-ds1302"
#define DRV_VERSION	"0.1.0"

#define	RTC_CMD_READ	0x81		/* Read command */
#define	RTC_CMD_WRITE	0x80		/* Write command */

#define RTC_ADDR_RAM0	0x20		/* Address of RAM0 */
#define RTC_ADDR_TCR	0x08		/* Address of trickle charge register */
#define	RTC_ADDR_YEAR	0x06		/* Address of year register */
#define	RTC_ADDR_DAY	0x05		/* Address of day of week register */
#define	RTC_ADDR_MON	0x04		/* Address of month register */
#define	RTC_ADDR_DATE	0x03		/* Address of day of month register */
#define	RTC_ADDR_HOUR	0x02		/* Address of hour register */
#define	RTC_ADDR_MIN	0x01		/* Address of minute register */
#define	RTC_ADDR_SEC	0x00		/* Address of second register */

#define	RTC_RESET	0x1000
#define	RTC_IODATA	0x0800
#define	RTC_SCLK	0x0400



//#error "Add support for your platform"

struct ds1302_rtc {
	struct rtc_device *rtc_dev;
	spinlock_t lock;
};


#define GB9260_RTC_DS1302_SCLK_PIN   AT91_PIN_PB16
#define GB9260_RTC_DS1302_CE_PIN     AT91_PIN_PB17
#define GB9260_RTC_DS1302_DATA_PIN   AT91_PIN_PB18


#define SET_1302_CLK	at91_set_gpio_value(GB9260_RTC_DS1302_SCLK_PIN, 1)
#define RESET_1302_CLK	at91_set_gpio_value(GB9260_RTC_DS1302_SCLK_PIN, 0)

#define SET_1302_CE		at91_set_gpio_value(GB9260_RTC_DS1302_CE_PIN, 1)
#define RESET_1302_CE	at91_set_gpio_value(GB9260_RTC_DS1302_CE_PIN, 0)


#define SET_1302_DAT	at91_set_gpio_value(GB9260_RTC_DS1302_DATA_PIN, 1)
#define RESET_1302_DAT	at91_set_gpio_value(GB9260_RTC_DS1302_DATA_PIN, 0)


#define SET_1302_DAT_IN()      at91_set_gpio_input(GB9260_RTC_DS1302_DATA_PIN, 1)
#define SET_1302_DAT_OUT()     at91_set_gpio_output(GB9260_RTC_DS1302_DATA_PIN, 0)

#define GET_1302_DAT    at91_get_gpio_value(GB9260_RTC_DS1302_DATA_PIN)

void ds1302_gpio_init(void)
{

	at91_set_GPIO_periph(GB9260_RTC_DS1302_CE_PIN, 1);
	at91_set_GPIO_periph(GB9260_RTC_DS1302_SCLK_PIN, 1);
	at91_set_GPIO_periph(GB9260_RTC_DS1302_DATA_PIN, 1);
	
	at91_set_gpio_output(GB9260_RTC_DS1302_CE_PIN, 0);
	at91_set_gpio_output(GB9260_RTC_DS1302_SCLK_PIN, 0);
	at91_set_gpio_output(GB9260_RTC_DS1302_DATA_PIN, 0);
	
	RESET_1302_DAT;
	RESET_1302_CE;
	RESET_1302_CLK;
}
void ds1302_write_byte(uint8_t dat)   //写一个字节的数据sck上升沿写数据  
{  
	uint8_t i = 0;  
	RESET_1302_CLK;                  //ds1302clk=0  
	      
	udelay(2);                      //延时大约2us  
	      
	for(i = 0;i < 8;i ++)  
	{  
		RESET_1302_CLK;             //ds1302clk=0;  
		if(dat&0x01)  
			SET_1302_DAT;  
		else                         //ds1302dat=(dat&0x01)  
			RESET_1302_DAT;                  
		udelay(2);  
		SET_1302_CLK;               //发送一位数据，clk上升沿,//ds1302clk=1  
		dat >>= 1;  
		udelay(1);  
	}     
}  
	
static unsigned int ds1302_readbyte(uint8_t add) //读数据  
{  
	uint8_t i=0;  
	uint8_t return_dat=0x00;

	 add = (((add & 0x3f) << 1) | RTC_CMD_READ);
	RESET_1302_CE;                                  //ds1302rst=0;  
	RESET_1302_CLK;                                 //ds1302clk=0;  
	udelay(3);                                    //略微延时2us  
	SET_1302_CE;                                    //ds1302rst=1;  
	udelay(3);                                    //时间要大约3us  
	ds1302_write_byte(add);                         //先写寄存器的地址
	SET_1302_DAT_IN();  
	for(i=0;i<8;i++)  
	{  
		SET_1302_CLK;                             //ds1302clk=1;  
		return_dat >>= 1;  
		RESET_1302_CLK;                             //ds1302clk=0;//拉低时钟线，以便于数据的读入  
		if(GET_1302_DAT==1)           //数据线此时为高电平  
		{
			return_dat = return_dat|0x80;
		}  
	}  	
	udelay(1);  
	RESET_1302_CE;                               //ds1302rst=0;释放总线  
	SET_1302_DAT_OUT();
	return return_dat;  
}

static void ds1302_writebyte(uint8_t add,uint8_t dat)                            //向指定寄存器写入一个字节的数据  
{  
	add = ((add & 0x3f) << 1) | RTC_CMD_WRITE;
	RESET_1302_CE;                                        //只有在rst为高电平的时候才能进行数据传输  
	RESET_1302_CLK;                                        //只有clk为低电平的时候，rst才能被置为高电平  
	//ds1302rst=0;  
	//ds1302clk=0;  
	udelay(1);                                                                                //略微延时  
	SET_1302_CE;                                          //clk = 0之后，这里将rst拉高，准备传送数据  
	//ds1302rst=1;  
	udelay(2);                                                                                //时间大约2us  
	ds1302_write_byte(add);                                                                //先发地址  
	ds1302_write_byte(dat);                                                                //然后发数据  
	RESET_1302_CE;                                        //只有在rst为高电平的时候才能进行数据传输  
	RESET_1302_CLK;                                       //拉低clk，以备下一次数据发送  
	//ds1302clk=0;  
	//ds1302rst=0;  
	udelay(1);    
}


static int ds1302_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct ds1302_rtc *rtc = dev_get_drvdata(dev);

	spin_lock_irq(&rtc->lock);

	tm->tm_sec	= bcd2bin(ds1302_readbyte(RTC_ADDR_SEC));
	tm->tm_min	= bcd2bin(ds1302_readbyte(RTC_ADDR_MIN));
	tm->tm_hour	= bcd2bin(ds1302_readbyte(RTC_ADDR_HOUR));
	tm->tm_wday	= bcd2bin(ds1302_readbyte(RTC_ADDR_DAY));
	tm->tm_mday	= bcd2bin(ds1302_readbyte(RTC_ADDR_DATE));
	tm->tm_mon	= bcd2bin(ds1302_readbyte(RTC_ADDR_MON)) - 1;
	tm->tm_year	= bcd2bin(ds1302_readbyte(RTC_ADDR_YEAR));

	if (tm->tm_year < 70)
		tm->tm_year += 100;

	spin_unlock_irq(&rtc->lock);

	dev_dbg(dev, "%s: tm is secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon + 1, tm->tm_year, tm->tm_wday);

	if (rtc_valid_tm(tm) < 0)
		dev_err(dev, "invalid date\n");

	return 0;
}

static int ds1302_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct ds1302_rtc *rtc = dev_get_drvdata(dev);

	spin_lock_irq(&rtc->lock);
	
	ds1302_writebyte(0x47,0x00);//unlock write protect

	/* Stop RTC */
	ds1302_writebyte(RTC_ADDR_SEC, ds1302_readbyte(RTC_ADDR_SEC) | 0x80);

	ds1302_writebyte(RTC_ADDR_SEC, bin2bcd(tm->tm_sec));
	ds1302_writebyte(RTC_ADDR_MIN, bin2bcd(tm->tm_min));
	ds1302_writebyte(RTC_ADDR_HOUR, bin2bcd(tm->tm_hour));
	ds1302_writebyte(RTC_ADDR_DAY, bin2bcd(tm->tm_wday));
	ds1302_writebyte(RTC_ADDR_DATE, bin2bcd(tm->tm_mday));
	ds1302_writebyte(RTC_ADDR_MON, bin2bcd(tm->tm_mon + 1));
	ds1302_writebyte(RTC_ADDR_YEAR, bin2bcd(tm->tm_year % 100));

	/* Start RTC */
	ds1302_writebyte(RTC_ADDR_SEC, ds1302_readbyte(RTC_ADDR_SEC) & ~0x80);
	
	ds1302_writebyte(0x47,0x80);//lock write protect
	spin_unlock_irq(&rtc->lock);

	return 0;
}

static int ds1302_rtc_ioctl(struct device *dev, unsigned int cmd,
			    unsigned long arg)
{
	switch (cmd) {
#ifdef RTC_SET_CHARGE
	case RTC_SET_CHARGE:
	{
		struct ds1302_rtc *rtc = dev_get_drvdata(dev);
		int tcs_val;

		if (copy_from_user(&tcs_val, (int __user *)arg, sizeof(int)))
			return -EFAULT;

		spin_lock_irq(&rtc->lock);
		ds1302_writebyte(0x47,0x00);//unlock write protect
		ds1302_writebyte(RTC_ADDR_TCR, (0xa0 | tcs_val * 0xf));
		ds1302_writebyte(0x47,0x80);//lock write protect
		spin_unlock_irq(&rtc->lock);
		return 0;
	}
#endif
	}

	return -ENOIOCTLCMD;
}

static struct rtc_class_ops ds1302_rtc_ops = {
	.read_time	= ds1302_rtc_read_time,
	.set_time	= ds1302_rtc_set_time,
	.ioctl		= ds1302_rtc_ioctl,
};

static int __devinit ds1302_rtc_probe(struct platform_device *pdev)
{
	struct ds1302_rtc *rtc;
	int ret;
	printk("ds1302_rtc_probe\n");

	/* Reset */
	//set_dp(get_dp() & ~(RTC_RESET | RTC_IODATA | RTC_SCLK));
	ds1302_gpio_init();
	/* Write a magic value to the DS1302 RAM, and see if it sticks. */
	
	ds1302_writebyte(0x47,0x00);//unlock write protect
	ds1302_writebyte(RTC_ADDR_RAM0, 0x42);
	if (ds1302_readbyte(RTC_ADDR_RAM0) != 0x42)
		{
		printk("ds1302_readbyte(RTC_ADDR_RAM0) != 0x42\n");
		return -ENODEV;
		}
	ds1302_writebyte(0x47,0x80);//lock write protect
	rtc = kzalloc(sizeof(struct ds1302_rtc), GFP_KERNEL);
	if (unlikely(!rtc))
		return -ENOMEM;

	spin_lock_init(&rtc->lock);
	rtc->rtc_dev = rtc_device_register("ds1302", &pdev->dev,
					   &ds1302_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc->rtc_dev)) {
		ret = PTR_ERR(rtc->rtc_dev);
		goto out;
	}

	platform_set_drvdata(pdev, rtc);

	return 0;
out:
	kfree(rtc);
	return ret;
}

static int __devexit ds1302_rtc_remove(struct platform_device *pdev)
{
	struct ds1302_rtc *rtc = platform_get_drvdata(pdev);

	if (likely(rtc->rtc_dev))
		rtc_device_unregister(rtc->rtc_dev);

	platform_set_drvdata(pdev, NULL);

	kfree(rtc);

	return 0;
}

static struct platform_driver ds1302_platform_driver = {
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= ds1302_rtc_probe,
	.remove		= __devexit_p(ds1302_rtc_remove),
};

static int __init ds1302_rtc_init(void)
{
	return platform_driver_register(&ds1302_platform_driver);
}

static void __exit ds1302_rtc_exit(void)
{
	platform_driver_unregister(&ds1302_platform_driver);
}

module_init(ds1302_rtc_init);
module_exit(ds1302_rtc_exit);

MODULE_DESCRIPTION("Dallas DS1302 RTC driver");
MODULE_VERSION(DRV_VERSION);
MODULE_AUTHOR("Paul Mundt, David McCullough");
MODULE_LICENSE("GPL v2");
