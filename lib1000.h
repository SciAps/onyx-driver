/*
 * lib1000.h
 *
 *  Created on: Jul 17, 2013
 *      Author: paul
 */

#ifndef LIB1000_H_
#define LIB1000_H_

#include <linux/mm.h>
#include <linux/ioport.h>

typedef struct lib1000_module {
	dev_t char_dev;
	struct class* device_class;
} lib1000_module;

typedef struct sciaps_dev {
	struct device* device;
	struct resource* resource;
	struct cdev cdev;
	int open;
	struct page* info_area_page;
	void* mapped;
} sciaps_dev;


typedef struct sciaps_lib1000 {
	struct cdev cdev;
	unsigned int num_open;
	unsigned long iobase;
	void __iomem *remapbase;
	unsigned int size;
	int irq;
	struct tasklet_struct irq_handler_tasklet;
	struct fasync_struct *async_queue;
} sciaps_lib1000;



#endif /* LIB1000_H_ */
