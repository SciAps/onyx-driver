
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/mm.h>

#include <asm/mach-types.h>


#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <linux/irq.h>

#include <mach/system.h>
#include <mach/hardware.h>

#include <plat/io.h>
#include <plat/gpmc.h>
#include <plat/dma.h>
#include <plat/mux.h>
#include <linux/delay.h>


#include "lib1000.h"

#define DEVICE_NAME "lib1000"
#define SCIAPS_GPMC_CS 3
#define GPIO_IRQ 167
#define IOSIZE 45092
#define ROUNDUP_PAGESIZE(x) ((x + PAGE_SIZE-1) & ~(PAGE_SIZE-1))
#define IOPAGES ROUNDUP_PAGESIZE(IOSIZE)


static lib1000_module module;
static struct platform_device* the_device;


MODULE_AUTHOR("Paul Soucy, SciAps");
MODULE_LICENSE("Dual BSD/GPL");

irqreturn_t irqhandler(int irq, void *dev_id, struct pt_regs *regs);
void irqhandler_tasklet(unsigned long arg);

static int device_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int err;
	struct sciaps_lib1000* data = filp->private_data;

	//vma->vm_ops = &mmap_vm_ops;
	vma->vm_flags |= VM_RESERVED;
	
	err = io_remap_pfn_range(vma, vma->vm_start, (data->iobase) >> PAGE_SHIFT, ROUNDUP_PAGESIZE(data->size), vma->vm_page_prot);
	if(err){
		printk(KERN_ERR "could not io_remap_pfn_range: %d\n", err);
		return err;
	}


	return 0;
}


static ssize_t device_read(struct file *filp, char __user *buf, size_t count, loff_t *pos)
{
	struct sciaps_lib1000* data = filp->private_data;

	//little hack to simulate the interrupt
	tasklet_schedule(&data->irq_handler_tasklet);
	return 0;
}


static int device_fasync(int fd, struct file *filp, int mode)
{
	struct sciaps_lib1000* data = filp->private_data;
	return fasync_helper(fd, filp, mode, &data->async_queue);
}

static int device_open(struct inode *inode, struct file *filp)
{
	struct sciaps_lib1000* data = container_of(inode->i_cdev, struct sciaps_lib1000, cdev);
	//Only 1 process should control this device at a time
	if(data->num_open > 0){
		return -EBUSY;
	}
	data->num_open++;
	try_module_get(THIS_MODULE);

	filp->private_data = data;

	return 0;
}

static int device_release(struct inode *inode, struct file *filp)
{
	struct sciaps_lib1000* data = container_of(inode->i_cdev, struct sciaps_lib1000, cdev);
	data->num_open--;
	if(data->num_open < 0){
		data->num_open = 0;
	}

	device_fasync(-1, filp, 0 );

	module_put(THIS_MODULE);
	return 0;
}


static struct file_operations fops = {
		.owner = THIS_MODULE,
		.mmap = device_mmap,
		.read = device_read,
		//.write = device_write,
		.fasync = device_fasync,
		.open = device_open,
		.release = device_release
};


//Top half irq handle
irqreturn_t lib1000_irqhandler(int irq, void *dev_id)
{
	sciaps_lib1000* data = (sciaps_lib1000*)dev_id;
	tasklet_schedule(&data->irq_handler_tasklet);
	return IRQ_HANDLED;
}

//Bottom half irq handle
void irqhandler_tasklet(unsigned long arg)
{
	sciaps_lib1000* data = (sciaps_lib1000*)arg;

	pr_info("in irq tasklet");

	if(data->async_queue) {
		kill_fasync(&data->async_queue, SIGIO, POLL_IN);
	}
}

static int lib1000_dev_probe(struct platform_device *pdev)
{
	int retval;
	struct sciaps_lib1000 *data = pdev->dev.platform_data;
	DECLARE_TASKLET(irqtasket, irqhandler_tasklet, (unsigned long)data);
	struct resource *res;
	int res_size;
	char chardevname[32];

	sprintf(chardevname, "%s-%d", DEVICE_NAME, pdev->id);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res){
		pr_err("could not allocate mem resource\n");
		retval = -ENODEV;
		goto fail;
	}

	res_size = resource_size(res);

	data->iobase = res->start;
	data->size = res_size;
	data->remapbase = ioremap_nocache(res->start, res_size);
	
	//Setup the IRQ handler
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	data->irq = res->start;
	retval = request_irq(data->irq, lib1000_irqhandler, IRQF_TRIGGER_LOW, chardevname, data);
	if(retval){
		pr_err("failed request_irq: %d", retval);
		goto fail;
	}

	//setup the tasklet
	data->irq_handler_tasklet = irqtasket;

	/* Create char device instance */
	cdev_init(&data->cdev, &fops);
	retval = cdev_add(&data->cdev, MKDEV(MAJOR(module.char_dev), pdev->id), 1);
	if(retval){
		pr_err("cdev_add failed\n");
		goto fail;
	}


	/* add the new char device to the filesystem */
	device_create(module.device_class,
			NULL, /* no parent */
			data->cdev.dev,
			NULL,
			chardevname
	);



	return 0;

	fail:
	return retval;
}

static int lib1000_dev_remove(struct platform_device *pdev)
{
	struct sciaps_lib1000 *data = pdev->dev.platform_data;



	if(data->cdev.dev) {
		device_destroy(module.device_class, data->cdev.dev);
	}

	if(data->cdev.dev){
		cdev_del(&data->cdev);
	}

	if(data->irq > 0){
		free_irq(data->irq, data);
	}

	if(data->remapbase) {
		iounmap(data->remapbase);
	}

	return 0;
}

static struct platform_driver lib1000_driver = {
	.probe = lib1000_dev_probe,
	.remove = lib1000_dev_remove,
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE
	}
};

/////// For creating a device --- This is eventual be moved to the board config

typedef struct gpio_irq_obj {
	int gpio_pin;
	int irq;
	unsigned long ctl_reg;
	unsigned short ctl_value;
} gpio_irq_obj;

static gpio_irq_obj gpios_to_monitor = {167, -1, 0x48002130, 0x11c};

static int setup_gpio_pin(void)
{
	short reg;
	short* addr;
	addr = ioremap(gpios_to_monitor.ctl_reg, 2);
	reg = ioread16(addr);

	pr_info("reg value is: 0x%x\n", reg);

	iowrite16(gpios_to_monitor.ctl_value, addr);

	iounmap(addr);
	return 0;
}

int create_device(void)
{
	int err;
	unsigned long cs_mem_base;
	sciaps_lib1000 config;
	
	struct resource gpmc_lib1000_resources[] = {
		[0] = {
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.flags	= IORESOURCE_IRQ,
		},
	};

	memset(&config, 0, sizeof(config));


	gpmc_cs_write_reg(SCIAPS_GPMC_CS, GPMC_CS_CONFIG1, (1<<23) | 0x200 | 0x1000 | 0x10);
	gpmc_cs_write_reg(SCIAPS_GPMC_CS, GPMC_CS_CONFIG2, 0x1F0000 | 0x1900 | 0x0);
	gpmc_cs_write_reg(SCIAPS_GPMC_CS, GPMC_CS_CONFIG3, 0xC0000 | 0xA00 | 0x3);
	gpmc_cs_write_reg(SCIAPS_GPMC_CS, GPMC_CS_CONFIG4, 0x1C000000 | 0xF0000 | 0x1A00 | 0xC);
	gpmc_cs_write_reg(SCIAPS_GPMC_CS, GPMC_CS_CONFIG5, 0x150000 | 0x1F00 | 0x1F);
	gpmc_cs_write_reg(SCIAPS_GPMC_CS, GPMC_CS_CONFIG6, 0x1A000000 | 0xE0000 | 0xF);

	err = gpmc_cs_request(SCIAPS_GPMC_CS, SZ_16M, &cs_mem_base);
	if(err < 0) {
		pr_err("Failed to request GPMC mem for %s\n", DEVICE_NAME);
		return err;
	}

	gpmc_lib1000_resources[0].start = cs_mem_base;
	gpmc_lib1000_resources[0].end = cs_mem_base + IOSIZE;

	//GPIO interrupt
	err = gpio_request_one(GPIO_IRQ, GPIOF_IN, "lib1000_irq");
	if(err){
		pr_err("Failed to request IRQ GPIO: %d", GPIO_IRQ);
		goto gpio_fail;
	}

	//omap_mux_init_signal("gpio_98", OMAP_PIN_INPUT_PULLUP);
	setup_gpio_pin();


	gpmc_lib1000_resources[1].start = gpio_to_irq(GPIO_IRQ);
	enable_irq_wake(gpmc_lib1000_resources[1].start);


	the_device = platform_device_register_resndata(NULL, DEVICE_NAME, 0,
		gpmc_lib1000_resources, ARRAY_SIZE(gpmc_lib1000_resources),
		&config, sizeof(config));

	return 0;

	gpio_fail:
	gpio_free(GPIO_IRQ);

	return err;
}

static int __init lib1000_init_module(void)
{
	int err;

	the_device = NULL;

	memset(&module, 0, sizeof(lib1000_module));

	/* Create device class */
	module.device_class = class_create(THIS_MODULE, DEVICE_NAME);
	if(IS_ERR(module.device_class)){
		err = PTR_ERR(module.device_class);
		pr_err("class create failed: %d", err);
		goto fail;
	}

	/* Alloc a new char region for this module. We only alloc 1 minor */
	err = alloc_chrdev_region(&module.char_dev, 0, 1, DEVICE_NAME);
	if(err < 0) {
		pr_err("alloc_chrdev_region failed");
		goto fail;
	}

	/* Register new driver */
	err = platform_driver_register(&lib1000_driver);
	if(err < 0){
		pr_err("driver register failed: %d", err);
		goto fail;
	}

	
	create_device();

	return 0;

	fail:

	if(module.device_class){
		class_destroy(module.device_class);
	}

	if(module.char_dev){
		unregister_chrdev_region(module.char_dev, 1);
	}


	return err;
}

static void __exit lib1000_exit(void)
{
	if(the_device){
		gpio_free(GPIO_IRQ);
		gpmc_cs_free(SCIAPS_GPMC_CS);
		platform_device_unregister(the_device);
		the_device = NULL;
	}

	platform_driver_unregister(&lib1000_driver);

	if(module.char_dev){
		unregister_chrdev_region(module.char_dev, 1);
	}

	if(module.device_class){
		class_destroy(module.device_class);
	}
	
}

module_init(lib1000_init_module);
module_exit(lib1000_exit);
