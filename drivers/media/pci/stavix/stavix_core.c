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

#include "stavix.h"

static bool enable_msi = true; 
module_param(enable_msi, bool, 0444);
MODULE_PARM_DESC(enable_msi, "Use MSI. Default: true");

unsigned long gBaseHdwr;            
unsigned long gBaseLen;            


void stavix_gpio_set_pin(struct stavix_dev *dev, 
		struct stavix_gpio_pin *pin, int state)
{
	u32 tmp;

	if (pin->lvl == STAVIX_GPIODEF_NONE)
		return;

	if (pin->lvl == STAVIX_GPIODEF_LOW)
		state = !state;

	tmp = pci_read(STAVIX_GPIO_BASE, 0);
	if (state)
		tmp |= BIT(pin->nr);
	else
		tmp &= ~BIT(pin->nr);
	pci_write(STAVIX_GPIO_BASE, 0, tmp);
}

static irqreturn_t stavix_irq_handler(int irq, void *dev_id) 
{
	struct stavix_dev *dev = (struct stavix_dev *) dev_id;
	u32 stat;
	irqreturn_t ret = IRQ_NONE;
	stat = pci_read(STAVIX_INT_BASE, STAVIX_INT_IPR);
	if (stat) 
		ret = IRQ_WAKE_THREAD;
	return ret;
}

static irqreturn_t stavix_irq_handler_threaded(int irq, void *dev_id) 
{
	struct stavix_dev *dev = (struct stavix_dev *) dev_id;
	struct stavix_i2c *i2c;
	
	u32 stat, status;
	stat = pci_read(STAVIX_INT_BASE, STAVIX_INT_IPR);

	pci_write(STAVIX_INT_BASE, STAVIX_INT_IAR, 0x03); 
	if (stat & 0x00000002) { 

		status = pci_read(SG_DMA_BASE, SG_DMA_REG_STATUS);

		pci_write(SG_DMA_BASE, SG_DMA_REG_STATUS, status & SG_DMA_XR_IRQ_ALL_MASK);
		sg_dma_irq_process(dev, status); 
	}

	if (stat & 0x00000001) {

		i2c = &dev->i2c_bus;
		xiic_irq_process(i2c); 
	}


	return IRQ_HANDLED;
}

static int stavix_adapters_attach(struct stavix_dev *dev)
{
	int i, ret = 0;
	for (i = 0; i < dev->info->adapters; i++) {
		ret = stavix_dvb_init(&dev->adapter[i]);
		if (ret) {
			dev_err(&dev->pci_dev->dev,
				"adapter%d attach failed\n",
				dev->adapter[i].nr);
			dev->adapter[i].nr = -1;
		}
	}
	return 0;
}

static void stavix_adapters_detach(struct stavix_dev *dev)
{
	struct stavix_adapter *adapter;
	int i;

	for (i = 0; i < dev->info->adapters; i++) {
		adapter = &dev->adapter[i];		
		if (adapter->nr == -1)
			continue;		
		stavix_dvb_exit(adapter);
	}
}

static void stavix_adapters_init(struct stavix_dev *dev)
{
	struct stavix_adapter *adapter = dev->adapter;
	int i;

	for (i = 0; i < dev->info->adapters; i++) {
		adapter = &dev->adapter[i];
		adapter->nr = i;
		adapter->cfg = &dev->info->adap_config[i];
		adapter->dev = dev;
		adapter->dma.dev = dev; 
		adapter->i2c = &dev->i2c_bus; 
	}
}

static void stavix_adapters_release(struct stavix_dev *dev)
{
	tasklet_kill(&dev->tasklet);

}


static bool stavix_enable_msi(struct pci_dev *pci_dev, struct stavix_dev *dev)
{
	int err;

	if (!enable_msi) {
		dev_warn(&dev->pci_dev->dev,
			"MSI disabled by module parameter 'enable_msi'\n");
		return false;
	}

	err = pci_enable_msi(pci_dev);
	if (err) {
		dev_err(&dev->pci_dev->dev,
			"Failed to enable MSI interrupt."
			" Falling back to a shared IRQ\n");
		return false;
	}

	
	err = request_threaded_irq(pci_dev->irq, stavix_irq_handler,stavix_irq_handler_threaded,
			IRQF_SHARED, "stavix", dev);
	if (err) {
		
		dev_err(&dev->pci_dev->dev,
			"Failed to get an MSI interrupt."
			" Falling back to a shared IRQ\n");
		pci_disable_msi(pci_dev);
		return false;
	}
	return true;
}


static int stavix_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct stavix_dev *dev;
	int ret = -ENODEV;
	int i;
	struct stavix_adapter *adapter;
	if (pci_enable_device(pdev) < 0)
		return -ENODEV;

	ret = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "32-bit PCI DMA not supported\n");
		goto err0;
	}
	pci_set_master(pdev);
	dev = kzalloc(sizeof(struct stavix_dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		goto err0;
	}

	dev->pci_dev = pdev;
	pci_set_drvdata(pdev, dev);

	dev->info = (struct stavix_board *) id->driver_data;
	dev_info(&pdev->dev, "%s\n", dev->info->name);

        gBaseHdwr = pci_resource_start(pdev, 0);
        if (0 > gBaseHdwr) {
        	printk(KERN_WARNING"Init: Base Address not set.\n");
 		goto err1;
   	}
   	gBaseLen = pci_resource_len(pdev, 0);

	request_mem_region(gBaseHdwr, gBaseLen, "pcie i2c test"); 
	dev->lmmio = ioremap(gBaseHdwr, gBaseLen);
	if (!dev->lmmio) {
		ret = -ENOMEM;
		goto err1;
	}

	pci_write(STAVIX_INT_BASE, STAVIX_INT_MER, 0x0003); 
	pci_write(STAVIX_INT_BASE, STAVIX_INT_IER, 0x0003);
	pci_write(STAVIX_INT_BASE, STAVIX_INT_IAR, 0x0003); 

	stavix_adapters_init(dev);

	ret = sg_dma_init(dev);
	if (ret < 0)
		goto err2;


	ret = stavix_i2c_init(dev);
	if (ret < 0){
		goto err3;
	}


	if (stavix_enable_msi(pdev, dev)) {
		dev->msi = true;
	} else {
		ret = request_threaded_irq(pdev->irq, stavix_irq_handler,stavix_irq_handler_threaded,
				IRQF_SHARED, "stavix", dev);
		if (ret < 0) {
			dev_err(&pdev->dev, "%s: can't get IRQ %d\n",
				dev->info->name, pdev->irq);
			goto err4;
		}
		dev->msi = false;
	}


	pci_write(STAVIX_GPIO_BASE, STAVIX_GPIO_TRI, 0x00000000);
	pci_write(STAVIX_GPIO_BASE, STAVIX_GPIO_DATA, 0x00000000); 
	mdelay(1);
	pci_write(STAVIX_GPIO_BASE, STAVIX_GPIO_DATA, 0x000000100); 
	mdelay(100);

	ret = stavix_adapters_attach(dev);
	if (ret < 0){ 
		printk(KERN_INFO"failed: stavix_adapters_attach \n");
		goto err5;
	}
	dev_info(&pdev->dev, "%s: PCI %s, IRQ %d, MMIO 0x%lx\n",
		dev->info->name, pci_name(pdev), pdev->irq,
		(unsigned long) pci_resource_start(pdev, 0));


	adapter = dev->adapter; 
	for (i = 0; i < 1; i++) {			
		adapter->dma.tasklet_on = false;
		sg_dma_enable_tasklet(adapter);
	}

	return 0;

err5:
	printk(KERN_INFO"failed: probe() err5 \n");
	stavix_adapters_detach(dev);	
	free_irq(dev->pci_dev->irq, dev);
	if (dev->msi) {
		pci_disable_msi(pdev);
		dev->msi = false;
	}
err4:
	printk(KERN_INFO"failed: probe() err4 \n");
	stavix_i2c_exit(dev);
err3:
	sg_dma_free(dev);
err2:
	printk(KERN_INFO"failed: probe() err2 \n");
	stavix_adapters_release(dev);
	iounmap(dev->lmmio);
	pci_write(STAVIX_INT_BASE, STAVIX_INT_MER, 0x0);
err1:
	pci_set_drvdata(pdev, NULL);
	printk(KERN_INFO"failed: probe() err1 \n");
	kfree(dev);
err0:
	pci_disable_device(pdev);
	dev_err(&pdev->dev, "probe error\n");
	return ret;
}

static void stavix_remove(struct pci_dev *pdev)
{
	struct stavix_dev *dev = pci_get_drvdata(pdev);

	pci_write(STAVIX_INT_BASE, STAVIX_INT_MER, 0x0); 
	free_irq(pdev->irq, dev);
	if (dev->msi) {
		pci_disable_msi(pdev);
		dev->msi = false;
	}
	stavix_adapters_detach(dev);
	stavix_adapters_release(dev);
	sg_dma_free(dev);
	stavix_i2c_exit(dev);
	release_mem_region(gBaseHdwr, gBaseLen);
	iounmap(dev->lmmio);
	pci_set_drvdata(pdev, NULL);
	pci_disable_device(pdev);
	kfree(dev);
}

static int stavix_resume(struct pci_dev *pdev)
{
	struct stavix_dev *dev = pci_get_drvdata(pdev);
	stavix_i2c_init(dev);
	sg_dma_reg_init(dev);
	pci_write(STAVIX_INT_BASE, STAVIX_INT_MER, 0x0003);
	return 0;
}

/* PCI IDs */
#define STAVIX_ID(_board_id , _subvend , sub_dev) { \
	.vendor = STAVIX_VID, .device = STAVIX_PID, \
	.subvendor = _subvend, .subdevice = sub_dev, \
	.driver_data = (unsigned long)&stavix_boards[_board_id] }

static const struct pci_device_id stavix_id_table[] = {
	STAVIX_ID(HM610_BOARD , 0x0610 , 0x0001),
	STAVIX_ID(HM710_BOARD , 0x0710 , 0x0001),
	{0}
};
MODULE_DEVICE_TABLE(pci, stavix_id_table);

static struct pci_driver stavix_driver = {
	.name = "stavix driver",
	.id_table = stavix_id_table,
	.probe    = stavix_probe,
	.remove   = stavix_remove,
	.resume   = stavix_resume,
	.suspend  = NULL,
};

module_pci_driver(stavix_driver);

MODULE_AUTHOR("380572278@qq.com");
MODULE_DESCRIPTION("STAVIX FPGA cards driver");
MODULE_LICENSE("GPL");


