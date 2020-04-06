/*
 * driver/char/at91_adc.c
 *
 * Copyright (C) 2007 Embedall Technology Co., Ltd.
 *
 * Analog-to-digital Converter(ADC) Driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/poll.h>


#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/at91_adc.h>
#include <mach/at91_tc.h>
#include <mach/at91_pmc.h>




#define DRV_NAME "at91_adc"
#define DEV_NAME "adc"

#define adc_readl(adc,reg)     (__raw_readl((adc)->membase + (reg)))
#define adc_writel(adc,reg,v)  (__raw_writel((v), (adc)->membase + (reg)))
#define tc_readl(adc,reg)      (__raw_readl((adc)->tcxbase + (reg)))
#define tc_writel(adc,reg,v)   (__raw_writel((v), (adc)->tcxbase + (reg)))


#define ADC_MAX_CHANNEL CONFIG_AT91_ADC_CHANNELS
#define BUF_SIZE        8

#define buf_cnt(channel) (((channel)->head - (channel)->tail) & ((BUF_SIZE)-1))
#define buf_space(channel) (((channel)->tail-((channel)->head+1))&(BUF_SIZE-1))


#define ADCCTL_RESET                 _IO ('D', 0)
#define ADCCTL_START                 _IO ('D', 1)
#define ADCCTL_SETMODE               _IOW('D', 2, long)
#define ADCCTL_GETMODE               _IOR('D', 3, long)
#define ADCCTL_GETDATA               _IOR('D', 4, int)
#define ADCCTL_GETCNT                _IOR('D', 5, int)
#define ADCCTL_GETSTATUS             _IOR('D', 6, int)


struct adc;

struct adc_mode 
{
	unsigned int   trigger;
#define ADC_TRIGGER_SOFT   0x00
#define ADC_TRIGGER_TIMER  0x01
#define ADC_TRIGGER_EXT    0x08
	unsigned int   trigger_time;
	unsigned int   resolution;
#define ADC_M_8BIT  0x01
#define ADC_M_10BIT 0x00
	unsigned int   sleep_mode;
	unsigned int   adc_clock;
	unsigned int   startup_time;
	unsigned int   sample_time;
};


struct adc_channel 
{
	struct cdev         cdev;
	struct device       *dev;
	struct class_device *class_dev;
	
	int id;
	
	int buf[BUF_SIZE];
	int head;
	int tail;
	
	struct fasync_struct *fasync;
	struct adc *adc;
};

struct adc
{
	dev_t           devt;
	struct class    *class;
	void __iomem    *membase;
	void __iomem    *tcbbase;
	void __iomem    *tcxbase;
	unsigned int    irq;
	struct adc_mode mode;

	spinlock_t      lock;

	struct adc_channel *channel[ADC_MAX_CHANNEL];
};



static struct adc *adc;

static inline int buf_in(struct adc_channel *channel, int v)
{
	channel->buf[channel->head] = v;
	channel->head = (channel->head + 1) & (BUF_SIZE - 1);
	return 0;
}

static int adc_init_tc(struct adc *adc)
{
	unsigned int dummy = 0;

	spin_lock(&adc->lock);
	
	if (adc->mode.trigger != ADC_TRIGGER_TIMER) {
		at91_sys_write(AT91_PMC_PCDR, 1 << AT91SAM9260_ID_TC2);
		spin_unlock(&adc->lock);
		return 0;
	}
	
	tc_writel(adc, AT91_TC_CCR, AT91_TC_CLKDIS);
	
	dummy |= (AT91_TC_TIMER_CLOCK5 | AT91_TC_CPCTRG | AT91_TC_WAVE |
		  AT91_TC_WAVESEL_UP_AUTO | AT91_TC_ACPA_SET |
		  AT91_TC_ACPC_CLEAR );
	tc_writel(adc, AT91_TC_CMR, dummy);
	
	if (adc->mode.trigger_time) {
		dummy = (adc->mode.trigger_time*1000000)/(1000000000/32768);
		if (dummy > 0xffff) dummy = 0xffff;
		tc_writel(adc, AT91_TC_RC, dummy);
		tc_writel(adc, AT91_TC_RA, dummy * 3 / 5);
	} else {
		tc_writel(adc, AT91_TC_RC, 32768);
		tc_writel(adc, AT91_TC_RA, 32768 * 3 / 5);
	}
	
	at91_sys_write(AT91_PMC_PCER, 1 << AT91SAM9260_ID_TC2);
	tc_writel(adc, AT91_TC_CCR, AT91_TC_CLKEN | AT91_TC_SWTRG);
	spin_unlock(&adc->lock);
	return 0;
}

static int adc_hw_init(struct adc *adc)
{
	adc_writel(adc, AT91_ADC_CR,  AT91_ADC_SWRST);
	adc_writel(adc, AT91_ADC_IER, AT91_ADC_DRDY);
	at91_sys_write(AT91_PMC_PCER, 1 << adc->irq);
	return 0;
}

static int adc_fasync(int fd, struct file *file, int mode)
{
	struct adc_channel *channel = file->private_data;
	return fasync_helper(fd, file, mode, &channel->fasync);
}

static int adc_open(struct inode *inode, struct file *file)
{
	struct adc *adc;
	struct adc_channel *channel;
	channel = container_of(inode->i_cdev, struct adc_channel, cdev);
	file->private_data = channel;
	adc = channel->adc;

	spin_lock(&adc->lock);
	at91_set_multi_drive(PIN_BASE + 0x40 + channel->id, 1);
	adc_writel(adc, AT91_ADC_IER,  (1 << channel->id));
	adc_writel(adc, AT91_ADC_CHER, (1 << channel->id));
	spin_unlock(&adc->lock);
	
	return nonseekable_open(inode, file);
}

static int adc_release(struct inode *inode, struct file *file)
{
	struct adc *adc;
	struct adc_channel *channel;
	channel = container_of(inode->i_cdev, struct adc_channel, cdev);
	adc = channel->adc;
	
//	adc_fasync(-1, file, 0);

	spin_lock(&adc->lock);
	adc_writel(adc, AT91_ADC_IDR,  1 << channel->id);
	adc_writel(adc, AT91_ADC_CHDR, 1 << channel->id);
	spin_unlock(&adc->lock);
	
	return 0;
}

static ssize_t adc_read(struct file *file, char __user *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

static ssize_t adc_write(struct file *file, const char __user *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

/* static void adc_sigio(struct adc_channel *channel) */
/* { */
/* 	if (channel->fasync) */
/* 		kill_fasync(&channel->fasync, SIGIO, POLL_IN); */
/* } */

static int adc_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	struct adc *adc;
	struct adc_channel *channel;
	struct adc_mode *mode;

	int ret = 0;
	unsigned int dummy = 0;
	
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	
	channel = container_of(inode->i_cdev, struct adc_channel, cdev);
	adc = channel->adc;
	mode = &adc->mode;
	
	switch (cmd) {
	case ADCCTL_RESET:
		adc_writel(adc, AT91_ADC_CR, AT91_ADC_SWRST);
		return 0;
	case ADCCTL_START:
		adc_writel(adc, AT91_ADC_CR, AT91_ADC_START);
		return 0;
	case ADCCTL_SETMODE:
		ret = copy_from_user(mode, argp, sizeof(struct adc_mode));
		
		if (mode->trigger == ADC_TRIGGER_TIMER)
			dummy |= AT91_ADC_TRGEN | AT91_ADC_TRGSEL_TC2;
		else if (mode->trigger == ADC_TRIGGER_EXT)
			dummy |= AT91_ADC_TRGEN | AT91_ADC_TRGSEL_EXTERNAL;
		if (mode->resolution & ADC_M_8BIT)
			dummy |= AT91_ADC_LOWRES;
		if (mode->sleep_mode)
			dummy |= AT91_ADC_SLEEP;
		if (mode->adc_clock)
			dummy |= AT91_ADC_PRESCAL_(mode->adc_clock);
		if (mode->startup_time)
			dummy |= AT91_ADC_STARTUP_(mode->startup_time);
		if (mode->sample_time)
			dummy |= AT91_ADC_SHTIM_(mode->sample_time);
		adc_init_tc(adc);
		spin_lock(&adc->lock);
		adc_writel(adc, AT91_ADC_MR, dummy);
		spin_unlock(&adc->lock);
		return 0;
	case ADCCTL_GETMODE:
		ret = copy_to_user(argp, mode, sizeof(struct adc_mode));
		return 0;
	case ADCCTL_GETDATA:
		ret = buf_cnt(channel);
		if (ret > 0) {
			if (!put_user(channel->buf[channel->tail], p))
				channel->tail = (channel->tail + 1) & (BUF_SIZE -1);
			return 0;
		}
		return -EFAULT;
	case ADCCTL_GETCNT:
		return put_user(buf_cnt(channel), p);
	case ADCCTL_GETSTATUS:
		return put_user(adc_readl(adc, AT91_ADC_SR), p);
	default:
		return -EINVAL;
	}
	return -EINVAL;
}

static struct file_operations adc_fops = {
        .owner     = THIS_MODULE,
        .read      = adc_read,
        .write     = adc_write,
        .open      = adc_open,
        .release   = adc_release,
	.ioctl     = adc_ioctl,
        .fasync    = adc_fasync,
};
		
static irqreturn_t adc_interrupt(int irq, void *dev_id)
{
	struct adc *adc = dev_id;
	struct adc_channel *channel;
	

	unsigned int status;

	status = adc_readl(adc, AT91_ADC_SR) & adc_readl(adc, AT91_ADC_IMR);
	while (status) {
		printk(KERN_DEBUG "at91_adc: interrupt status reg 0x%08x\n",
		       status);
		
		if (status & AT91_ADC_EOC(0)){
			channel = adc->channel[0];
			buf_in(channel, adc_readl(adc, AT91_ADC_CHR(0)));
//			adc_sigio(channel);
		}
		if (status & AT91_ADC_EOC(1)){
			channel = adc->channel[1];
			buf_in(channel, adc_readl(adc, AT91_ADC_CHR(1)));
//			adc_sigio(channel);
		}
		if (status & AT91_ADC_EOC(2)){
			channel = adc->channel[2];
			buf_in(channel, adc_readl(adc, AT91_ADC_CHR(2)));
//			adc_sigio(channel);
		}
		if (status & AT91_ADC_EOC(3)) {
			channel = adc->channel[3];
			buf_in(channel, adc_readl(adc, AT91_ADC_CHR(3)));
//			adc_sigio(channel);
		}
		if (status & AT91_ADC_DRDY)
			adc_readl(adc, AT91_ADC_LCDR);
		
		status = adc_readl(adc, AT91_ADC_SR) & adc_readl(adc, AT91_ADC_IMR);
	}
	return IRQ_HANDLED;
}



static int __init adc_probe(struct platform_device *pdev)
{
	struct adc_channel *channel;
	int ret;
	
	channel = kmalloc(sizeof(struct adc_channel), GFP_KERNEL);
	if (!channel){
		printk(KERN_ERR "at91_adc: failed to kmalloc channel %d\n", pdev->id);
		return -ENOMEM;
	}
//	channel->fasync = kmalloc(sizeof(struct fasync_struct), GFP_KERNEL);
//	if (!channel->fasync) return -ENOMEM;
	channel->id  = pdev->id;
	channel->dev = &pdev->dev;
	channel->adc = adc;
	channel->head = 0;
	channel->tail = 0;
	cdev_init(&channel->cdev, &adc_fops);
	channel->cdev.owner = THIS_MODULE;
	ret = cdev_add(&channel->cdev, MKDEV(MAJOR(adc->devt), pdev->id), 1);
	if (ret) {
		printk(KERN_ERR "at91_adc: failed to add channel %d device\n", pdev->id);
		
		kfree(channel);
		return ret;
	}
	channel->dev = device_create(adc->class, NULL,
						 MKDEV(MAJOR(adc->devt), pdev->id),
						 NULL,
						 DEV_NAME"%d", pdev->id);
	if (IS_ERR(channel->dev)) {
		cdev_del(&channel->cdev);
		kfree(channel);
		return PTR_ERR(channel->dev);
	}

	adc->channel[pdev->id] = channel;
	platform_set_drvdata(pdev, channel);

	printk(KERN_INFO "at91_adc.%d: register at /dev/adc%d (%d:%d)\n",
	       pdev->id, pdev->id, MAJOR(adc->devt), pdev->id);
	
	return 0;
}

static int __exit adc_remove(struct platform_device *pdev)
{
	struct adc_channel *channel = platform_get_drvdata(pdev);
	
	device_destroy(adc->class,MKDEV(MAJOR(adc->devt), pdev->id));
	cdev_del(&channel->cdev);
	kfree(channel);
	return 0;
}

static struct platform_driver adc_driver = {
	.probe         = adc_probe,
	.remove        = __exit_p(adc_remove),
	.driver        = {
		.name  = DRV_NAME,
		.owner = THIS_MODULE,
	},
};

static struct platform_device adc_channel_device[ADC_MAX_CHANNEL];

static int __init adc_add_channel_device(void)
{
	int i;
	at91_set_gpio_input(AT91_PIN_PC0, 0);
	at91_set_gpio_input(AT91_PIN_PC1, 0);
	at91_set_gpio_input(AT91_PIN_PC2, 0);
	at91_set_gpio_input(AT91_PIN_PC3, 0);

	for (i=0; i<ADC_MAX_CHANNEL; i++){
		adc_channel_device[i].name = DRV_NAME;
		adc_channel_device[i].id   = i;
		platform_device_register(&adc_channel_device[i]);
	}
	return 0;
}

static int __init adc_init(void)
{
	int ret = 0;

	adc = kmalloc(sizeof(struct adc), GFP_KERNEL);
	if (!adc)
		return -ENOMEM;

	if (!request_mem_region(AT91SAM9260_BASE_ADC, SZ_16K, DRV_NAME)){
		kfree(adc);
		return -EBUSY;
	}
	adc->membase = ioremap(AT91SAM9260_BASE_ADC, SZ_16K);
	if (adc->membase == NULL)
		goto adc_release_mem;

	if (!request_mem_region(AT91SAM9260_BASE_TC0, SZ_16K, DRV_NAME))
		goto adc_iounmap;
	adc->tcbbase = ioremap(AT91SAM9260_BASE_TC0, SZ_16K);
	if (adc->tcbbase == NULL)
		goto adc_release_mem_tc;

	adc->irq = AT91SAM9260_ID_ADC;
	adc->tcxbase = adc->tcbbase + 0x80;
	spin_lock_init(&adc->lock);
	
	adc->class = class_create(THIS_MODULE, DEV_NAME);
	if (IS_ERR(adc->class)){
		printk(KERN_ERR "at91_adc: faile to create device class\n");
		goto adc_iounmap_tc;
	}
	
	ret = alloc_chrdev_region(&adc->devt, 0, ADC_MAX_CHANNEL, DEV_NAME);
	if (ret < 0) {
		printk(KERN_ERR"%s: failed to allocate dev region\n", __FILE__);
		goto adc_destroy_class;
	}

	if (request_irq(adc->irq, adc_interrupt, IRQF_SHARED, DRV_NAME, adc)){
		printk(KERN_ERR"%s: request irq failed\n", __FILE__);
		goto adc_del_channel;
	}
	
	adc_hw_init(adc);
	
	platform_driver_register(&adc_driver);
	printk(KERN_INFO "Analog-to-Digital Converter (irq %d)\n", adc->irq);
	
	adc_add_channel_device();
	
	return 0;
		
adc_del_channel:
	unregister_chrdev_region(adc->devt, ADC_MAX_CHANNEL);
adc_destroy_class:
	class_destroy(adc->class);
adc_iounmap_tc:
	iounmap(adc->tcbbase);
adc_release_mem_tc:
	release_mem_region(AT91SAM9260_BASE_TC0, SZ_16K);
adc_iounmap:
	iounmap(adc->membase);
adc_release_mem:
	release_mem_region(AT91SAM9260_BASE_ADC, SZ_16K);
	kfree(adc);
	return ret;
}

		
static void __exit adc_exit(void)
{
	platform_driver_unregister(&adc_driver);
	unregister_chrdev_region(adc->devt, ADC_MAX_CHANNEL);
	class_destroy(adc->class);
	free_irq(adc->irq, adc);
	iounmap(adc->tcbbase);
	release_mem_region(AT91SAM9260_BASE_TC0, SZ_16K);
	iounmap(adc->membase);
	release_mem_region(AT91SAM9260_BASE_ADC, SZ_16K);
	kfree(adc);
}


module_init(adc_init);
module_exit(adc_exit);

MODULE_AUTHOR("Meng Renzhou");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AT91 Analog-to-Digital Converter Driver");

