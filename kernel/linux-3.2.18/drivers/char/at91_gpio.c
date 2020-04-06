/*
 * at91_gpio.c
 *
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/device.h>

static dev_t gpio_dev_t;
static struct class *gpio_class;

#define GPIO_HIGH   _IOW('G', 1, int)
#define GPIO_LOW    _IOW('G', 2, int)
#define GPIO_WRITE  _IOW('G', 3, int)
#define GPIO_READ   _IOR('G', 4, int)
#define GPIO_OUTPUT _IOW('G', 5, int)
#define GPIO_INPUT  _IOW('G', 6, int)
#define GPIO_INFO   _IOR('G', 7, long)


struct gpio 
{
	struct cdev         cdev;
	struct device       *dev;
	struct class_device *class_dev;
	
	unsigned int        *pin;
	const char          **name;

	struct gpio_info    *info;
	
	spinlock_t          lock;
};

static int gpio_open(struct inode *inode, struct file *file)
{
	struct gpio *gp = container_of(inode->i_cdev, struct gpio, cdev);
	file->private_data = gp;
	return nonseekable_open(inode, file);
}

static int gpio_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t gpio_write(struct file *file, const char __user *buf,
		      size_t count, loff_t *ofs)
{
	return 0;	
}

static ssize_t gpio_read(struct file *file, char __user *buf,
		     size_t count, loff_t *ofs)
{
	return 0;
}

static int gpio_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)
{
	struct gpio *gp = container_of(inode->i_cdev, struct gpio, cdev);
	struct gpio_info *info;
	
	unsigned int i, tmp;

	tmp = (unsigned int)arg;

	spin_lock_irq(&gp->lock);
	
	switch(cmd) {
	case GPIO_HIGH:
		for(i=0; gp->name[i]; i++,tmp>>=1){
			if (!gp->pin[i]) continue;
			if (tmp & 0x01) at91_set_gpio_value(gp->pin[i], 1);
			if (gp->info->output & (1 << i))
				gp->info->outval |= 1 << i;
		}
		break;
	case GPIO_LOW:
		for(i=0; gp->name[i]; i++){
			if ((tmp & 0x01) && gp->pin[i]) {
				at91_set_gpio_value(gp->pin[i], 0);
				if (gp->info->output & (1 << i))
					gp->info->outval &= ~(1 << i);
			}
			tmp >>= 1;
		}
		break;
	case GPIO_WRITE:
		for (i=0; gp->name[i]; i++,tmp>>=1) {
			if (tmp & 0x01) at91_set_gpio_value(gp->pin[i], 1);
			else at91_set_gpio_value(gp->pin[i], 0);
		}
		gp->info->outval = arg;
		break;
	case GPIO_READ:
		tmp = 0;
		for (i=0; gp->name[i]; i++) {
			if (!gp->pin[i]) continue;
			if (at91_get_gpio_value(gp->pin[i])) tmp |= 1<<i;
		}
		*((unsigned int *)arg) = tmp;
		break;
	case GPIO_OUTPUT:
		for (i=0; gp->name[i]; i++) {
			if ((tmp & 0x01) && gp->pin[i]) {
				if(arg & (1<<31)){
					at91_set_gpio_output(gp->pin[i], 1);
					gp->info->outval |= 1 << i;
				} else {
					at91_set_gpio_output(gp->pin[i], 0);
					gp->info->outval &= ~(1 << i);
				}
				gp->info->output &= ~(1 << i);
				gp->info->output |= 1 << i;
				gp->info->input  &= ~(1 << i);
			}
			tmp >>= 1;
		}
		break;
	case GPIO_INPUT:
		for (i=0; gp->name[i]; i++) {
			if ((tmp & 0x01) && gp->pin[i]) {
				if(arg & (1<<31))
					at91_set_gpio_input(gp->pin[i], 1);
				else
					at91_set_gpio_input(gp->pin[i], 0);
				gp->info->input &= ~(1 << i);
				gp->info->input |= 1 << i;
				gp->info->output &= ~(1 << i);
			}
			tmp >>= 1;
		}
		break;
	case GPIO_INFO:
		memcpy((struct gpio_info *)arg, gp->info, sizeof(struct gpio_info));
		info = (struct gpio_info *)arg;
		info->input = gp->info->input;
		info->output = gp->info->output;
		info->outval = gp->info->outval;
		break;
		
	default:
		return -ENOTTY;
	}

	spin_unlock_irq(&gp->lock);
	
	return 0;
}

static const struct file_operations gpio_fops = {
	.owner        = THIS_MODULE,
	.read         = gpio_read,
	.write        = gpio_write,
	.ioctl        = gpio_ioctl,
	.open         = gpio_open,
	.release      = gpio_release,
};


static int __devinit gpio_probe(struct platform_device *pdev)
{
	
	struct gpio *gp;
	int ret;

	gp = kmalloc(sizeof(struct gpio), GFP_KERNEL);
	if (!gp)
		return -ENOMEM;
	
	gp->info = pdev->dev.platform_data;
	gp->pin = gp->info->pin;
	gp->name = gp->info->name;
	//gp->name = gpio_name;
	
	gp->info->input = 0;
	gp->info->output = 0;
	
	spin_lock_init(&gp->lock);

	platform_set_drvdata(pdev, gp);

	cdev_init(&gp->cdev, &gpio_fops);
	gp->cdev.owner =  THIS_MODULE;
	ret = cdev_add(&gp->cdev, MKDEV(MAJOR(gpio_dev_t), pdev->id), 1);
	if (ret)
		goto e_free;

	gp->dev = device_create(gpio_class, NULL,
					    MKDEV(MAJOR(gpio_dev_t), pdev->id),
					    NULL,
					    "gpio");
	if (IS_ERR(gp->dev)){
		ret = PTR_ERR(gp->dev);
		goto e_register;
	}

	printk(KERN_INFO "GPIO: ");
	ret = 0;
	while (gp->name[ret]){
		printk("%4s ",gp->name[ret++]);
		if (ret == 9) printk("\n      ");
	}
	printk("\n");
	
	return 0;

e_register:
	cdev_del(&gp->cdev);
e_free:
	kfree(gp);
	return ret;
}

static int __devexit gpio_remove(struct platform_device *pdev)
{
	struct gpio *gp = platform_get_drvdata(pdev);

	device_destroy(gpio_class, MKDEV(MAJOR(gpio_dev_t), pdev->id));
	cdev_del(&gp->cdev);
	kfree(gp);
	return 0;
}


static struct platform_driver gpio_driver = {
	.probe       = gpio_probe,
	.remove      = __devexit_p(gpio_remove),
	.driver      = {
		.name = "gpio",
		.owner = THIS_MODULE,
	},
};


static int __init gpio_init(void)
{
	int ret;
	
	gpio_class = class_create(THIS_MODULE, "gpio");
	if (IS_ERR(gpio_class))
		return PTR_ERR(gpio_class);
	ret = alloc_chrdev_region(&gpio_dev_t, 0, 1, "gpio");
	if (ret < 0) {
		printk(KERN_ERR "%s: failed allocate dev region\n", __FILE__);
		class_destroy(gpio_class);
		return ret;
	}

	return platform_driver_register(&gpio_driver);
}

static void __exit gpio_exit(void)
{
	unregister_chrdev_region(gpio_dev_t, 1);
	class_destroy(gpio_class);
	platform_driver_unregister(&gpio_driver);
}

module_init(gpio_init);
module_exit(gpio_exit);


MODULE_AUTHOR("Renzhou Meng <ship_@163.com>");
MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("AT91 gpio driver");
