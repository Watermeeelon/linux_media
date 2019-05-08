/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 only, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _STAVIX_H_
#define _STAVIX_H_

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>

#include <linux/dmaengine.h>
#include <linux/version.h>

#include <linux/errno.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/wait.h>   

#include <media/demux.h> 
#include <media/dmxdev.h>
#include <media/dvb_demux.h>
#include <media/dvb_frontend.h>
#include <media/dvb_net.h>
#include <media/dvbdev.h>
#include "stavix_regs.h"
#include "stavix_dma.h"

#define STAVIX_VID		0xC111
#define STAVIX_PID		0x0001

#define STAVIX_MAX_ADAPTERS	(8)


#define STAVIX_GPIODEF_NONE	(0)
#define STAVIX_GPIODEF_HIGH	(1)
#define STAVIX_GPIODEF_LOW	(2)

#define SG_DMA_BUFFERS	8

#define HM610_BOARD 0x0610
#define HM710_BOARD 0x0710

struct stavix_dev ; 



struct stavix_gpio_pin {
	u8 lvl;
	u8 nr; 
};

struct stavix_gpio_config {  
	struct stavix_gpio_pin lnb_power;  
	struct stavix_gpio_pin lnb_voltage;
	struct stavix_gpio_pin demod_reset;
};

struct stavix_adap_config {  
	u32 ts_in;
	u8 i2c_bus_nr;
	struct stavix_gpio_config gpio;
};

enum xilinx_i2c_state {
	STATE_DONE,
	STATE_ERROR,
	STATE_START
};

struct stavix_board {
	char *name;
	int adapters;
	u8 eeprom_i2c; 
	u8 eeprom_addr;
	struct stavix_adap_config adap_config[8];
};


struct stavix_i2c {
	struct stavix_dev 		*dev;
	struct i2c_adapter 		i2c_adap;
	struct mutex 			lock;
	wait_queue_head_t 		wq;
	struct i2c_msg			*tx_msg;
	unsigned int			tx_pos;
	unsigned int			nmsgs;
	enum xilinx_i2c_state		state;
	struct i2c_msg			*rx_msg;
	int				rx_pos;
};

struct sg_dma_channel {
	dma_addr_t dma_addr; 
	u8 *buf[SG_DMA_BUFFERS + 1];
	u8 offset;
	u8 cnt;	
	struct list_head free_seg_list;
	struct sg_dma_tx_descriptor *seg_v; 
	dma_addr_t seg_p; 
	bool reach_tail; 
	bool err;
	bool tasklet_on;
	struct stavix_dev *dev; 
	u8 buf_cnt; 
};

struct stavix_adapter {
	int nr;
	struct stavix_adap_config *cfg;
	struct stavix_dev *dev;
	struct stavix_i2c *i2c;
	struct dvb_adapter dvb_adapter;
	struct dvb_frontend *fe;
	struct dvb_demux demux;
	struct dmxdev dmxdev;
	struct dvb_net dvbnet;
	struct dmx_frontend fe_hw;
	struct dmx_frontend fe_mem;
	int feeds;
	spinlock_t adap_lock; 
	struct sg_dma_channel dma;
};

struct stavix_dev {
	struct stavix_board *info;
	struct pci_dev *pci_dev;
	void __iomem *lmmio;
	bool msi;
	struct stavix_adapter adapter[STAVIX_MAX_ADAPTERS];
	struct stavix_i2c i2c_bus;	
	struct tasklet_struct tasklet;	
	spinlock_t adap_lock; 

};

#define pci_read(_b, _o)	readl(dev->lmmio + (_b + _o))   
#define pci_write(_b, _o, _v)	writel((_v), dev->lmmio + (_b + _o))


void stavix_gpio_set_pin(struct stavix_dev *dev,
			  struct stavix_gpio_pin *pin, int state);


extern int stavix_i2c_init(struct stavix_dev *dev);
extern void stavix_i2c_exit(struct stavix_dev *dev);
extern int xiic_irq_process(struct stavix_i2c *i2c);
extern struct stavix_board stavix_boards[];
extern int stavix_dvb_init(struct stavix_adapter *adapter);
extern void stavix_dvb_exit(struct stavix_adapter *adapter);
extern int  sg_dma_init(struct stavix_dev *dev);
extern void sg_dma_free(struct stavix_dev *dev);
extern void sg_dma_reg_init(struct stavix_dev *dev);
extern void sg_dma_enable(struct stavix_adapter *adap);
extern void sg_dma_disable(struct stavix_adapter *adap);
extern int  sg_dma_irq_process(struct stavix_dev *dev, u32 status); 
extern void sg_dma_enable_tasklet(struct stavix_adapter *adap);

#define SEG_SIZE  sizeof(struct sg_dma_tx_descriptor) 

#endif



