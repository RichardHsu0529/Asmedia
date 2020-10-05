
/*
 * Asmedia 28xx/18xx GPIO driver
 *
 * Copyright (C) 2019 ASMedia Technology Inc.
 * Author: Richard Hsu <Richard_Hsu@asmedia.com.tw>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio/driver.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/pm_runtime.h>

#define DRV_VERSION "1.0.0"

/* GPIO registers offsets */
#define ASMEDIA_GPIO_CTRL		0x920
#define ASMEDIA_GPIO_OUTPUT		0x928
#define ASMEDIA_GPIO_INPUT		0x930
#define ASMEDIA_REG_SWITCH		0xFFF

#define ASMEDIA_REG_SWITCH_CTL	0x01

#define ASMEDIA_GPIO_PIN5	5
#define ASMEDIA_GPIO_DEFAULT	0


#define PCI_DEVICE_ID_ASMEDIA_28xx_PID1	0x2824
#define PCI_DEVICE_ID_ASMEDIA_28xx_PID2	0x2812
#define PCI_DEVICE_ID_ASMEDIA_28xx_PID3	0x2806
#define PCI_DEVICE_ID_ASMEDIA_18xx_PID1	0x1824
#define PCI_DEVICE_ID_ASMEDIA_18xx_PID2	0x1812
#define PCI_DEVICE_ID_ASMEDIA_18xx_PID3	0x1806
#define PCI_DEVICE_ID_ASMEDIA_81xx_PID1	0x812a
#define PCI_DEVICE_ID_ASMEDIA_81xx_PID2	0x812b
#define PCI_DEVICE_ID_ASMEDIA_80xx_PID1	0x8061

/*
 * Data for PCI driver interface
 *
 * This data only exists for exporting the supported
 * PCI ids via MODULE_DEVICE_TABLE.  We do not actually
 * register a pci_driver, because someone else might one day
 * want to register another driver on the same PCI id.
 */
static const struct pci_device_id pci_tbl[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_ASMEDIA, PCI_DEVICE_ID_ASMEDIA_28xx_PID1), 0 },
	{ PCI_DEVICE(PCI_VENDOR_ID_ASMEDIA, PCI_DEVICE_ID_ASMEDIA_28xx_PID2), 0 },
	{ PCI_DEVICE(PCI_VENDOR_ID_ASMEDIA, PCI_DEVICE_ID_ASMEDIA_28xx_PID3), 0 },
	{ PCI_DEVICE(PCI_VENDOR_ID_ASMEDIA, PCI_DEVICE_ID_ASMEDIA_18xx_PID1), 0 },
	{ PCI_DEVICE(PCI_VENDOR_ID_ASMEDIA, PCI_DEVICE_ID_ASMEDIA_18xx_PID2), 0 },
	{ PCI_DEVICE(PCI_VENDOR_ID_ASMEDIA, PCI_DEVICE_ID_ASMEDIA_18xx_PID3), 0 },
	{ PCI_DEVICE(PCI_VENDOR_ID_ASMEDIA, PCI_DEVICE_ID_ASMEDIA_81xx_PID1), 0 },
	{ PCI_DEVICE(PCI_VENDOR_ID_ASMEDIA, PCI_DEVICE_ID_ASMEDIA_81xx_PID2), 0 },
	{ PCI_DEVICE(PCI_VENDOR_ID_ASMEDIA, PCI_DEVICE_ID_ASMEDIA_80xx_PID1), 0 },
	{ 0, },	/* terminate list */
};
MODULE_DEVICE_TABLE(pci, pci_tbl);


struct asmedia_gpio {
	struct gpio_chip	chip;
	struct pci_dev		*pdev;
	spinlock_t		lock; 
};

void pci_config_pm_runtime_get(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct device *parent = dev->parent;

	if (parent)
		pm_runtime_get_sync(parent);
	pm_runtime_get_noresume(dev);
	/*
	 * pdev->current_state is set to PCI_D3cold during suspending,
	 * so wait until suspending completes
	 */
	pm_runtime_barrier(dev);
	/*
	 * Only need to resume devices in D3cold, because config
	 * registers are still accessible for devices suspended but
	 * not in D3cold.
	 */
	if (pdev->current_state == PCI_D3cold)
		pm_runtime_resume(dev);
}

void pci_config_pm_runtime_put(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct device *parent = dev->parent;

	pm_runtime_put(dev);
	if (parent)
		pm_runtime_put_sync(parent);
}

static int asmedia_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	
	printk(KERN_INFO "ASMEDIA-28xx/18xx gpio %d request\n",offset);

	if(offset == ASMEDIA_GPIO_PIN5)
		return -ENODEV;

	return 0;
}

static void asmedia_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	printk(KERN_INFO "ASMEDIA-28xx/18xx gpio %d free\n",offset);
}

static void asmedia_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct asmedia_gpio *agp = gpiochip_get_data(chip);
	unsigned char temp;
	unsigned long flags;

	pci_config_pm_runtime_get(agp->pdev);
	spin_lock_irqsave(&agp->lock, flags);
	pci_read_config_byte(agp->pdev, ASMEDIA_GPIO_OUTPUT, &temp);
	if(value)
	{
		temp |= BIT(offset);
	}
	else{
		temp &= ~BIT(offset);
	}
	pci_write_config_byte(agp->pdev, ASMEDIA_GPIO_OUTPUT, temp);
	spin_unlock_irqrestore(&agp->lock, flags);
	pci_config_pm_runtime_put(agp->pdev);
	printk(KERN_INFO "ASMEDIA-28xx/18xx gpio %d set %d reg=%02x\n",offset,value, temp);
}

static int asmedia_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct asmedia_gpio *agp = gpiochip_get_data(chip);
	unsigned char temp;
	unsigned long flags;

	pci_config_pm_runtime_get(agp->pdev);
	spin_lock_irqsave(&agp->lock, flags);	
	pci_read_config_byte(agp->pdev, ASMEDIA_GPIO_INPUT, &temp);
	spin_unlock_irqrestore(&agp->lock, flags);
	pci_config_pm_runtime_put(agp->pdev);

	printk(KERN_INFO "ASMEDIA-28xx/18xx GPIO Pin %d get reg=%02x\n",offset, temp);
	
	return (temp & BIT(offset)) ? 1 : 0;
}

static int asmedia_gpio_dirout(struct gpio_chip *chip, unsigned offset, int value)
{
	struct asmedia_gpio *agp = gpiochip_get_data(chip);
	unsigned char temp;
	unsigned long flags;

	pci_config_pm_runtime_get(agp->pdev);
	spin_lock_irqsave(&agp->lock, flags);
	pci_read_config_byte(agp->pdev, ASMEDIA_GPIO_CTRL, &temp);
	temp |= BIT(offset);
	pci_write_config_byte(agp->pdev, ASMEDIA_GPIO_CTRL, temp);
	spin_unlock_irqrestore(&agp->lock, flags);
	pci_config_pm_runtime_put(agp->pdev);
	printk(KERN_INFO "ASMEDIA-28xx/18xx dirout gpio %d  reg=%02x\n",offset,temp);
		
	return 0;
}

static int asmedia_gpio_dirin(struct gpio_chip *chip, unsigned offset)
{
	struct asmedia_gpio *agp = gpiochip_get_data(chip);
	unsigned char temp;
	unsigned long flags;

	pci_config_pm_runtime_get(agp->pdev);
	spin_lock_irqsave(&agp->lock, flags);
	pci_read_config_byte(agp->pdev, ASMEDIA_GPIO_CTRL, &temp);
	temp &= ~BIT(offset);
	pci_write_config_byte(agp->pdev, ASMEDIA_GPIO_CTRL, temp);
	spin_unlock_irqrestore(&agp->lock, flags);
	pci_config_pm_runtime_put(agp->pdev);
	printk(KERN_INFO "ASMEDIA-28xx/18xx dirin gpio %d,reg=%02x \n",offset,temp);

	return 0;
}

static struct asmedia_gpio gp = {
	.chip = {
		.label		= "ASMEDIA GPIO",
		.owner		= THIS_MODULE,
		//.base		= -1,
		.ngpio		= 8,
		.request	= asmedia_gpio_request,
		.free		= asmedia_gpio_free,
		.set		= asmedia_gpio_set,
		.get		= asmedia_gpio_get,
		.direction_output = asmedia_gpio_dirout,
		.direction_input = asmedia_gpio_dirin,
	},
};

static int __init asmedia_gpio_init(void)
{
	int err = -ENODEV;
	struct pci_dev *pdev = NULL;
	const struct pci_device_id *ent;
	u8 temp;
	unsigned long flags;
	int type;
	
	printk(KERN_INFO "ASMEDIA-28xx/18xx GPIO Driver Init-start\n");
		
	for_each_pci_dev(pdev) {
		ent = pci_match_id(pci_tbl, pdev);
		if (ent)
		{
			//Because GPIO Registers only work on Upstream port,so we detect it
			type = pci_pcie_type(pdev);
			if(type == PCI_EXP_TYPE_UPSTREAM)
			{
				dev_err(&pdev->dev, "ASMEDIA-28xx/18xx Init Upstream detected\n");
				goto found;
			}
		}
	}

	/* ASMEDIA-28xx/18xx GPIO not found. */
	dev_err(&pdev->dev,"ASMEDIA-28xx/18xx GPIO not detected\n");
	goto out;

found:
	gp.pdev = pdev;
	gp.chip.parent = &pdev->dev;

	spin_lock_init(&gp.lock);

	err = gpiochip_add_data(&gp.chip, &gp);
	if (err) {
		printk(KERN_ERR "GPIO registering failed (%d)\n",
		       err);
		goto out;
	}
	
	pci_config_pm_runtime_get(pdev);

	//Set PCI_CFG_Switch bit = 1,then we can access GPIO Registers
	spin_lock_irqsave(&gp.lock, flags);
	pci_read_config_byte(pdev, ASMEDIA_REG_SWITCH, &temp);
	temp |= ASMEDIA_REG_SWITCH_CTL;
	pci_write_config_byte(pdev, ASMEDIA_REG_SWITCH, temp);
	pci_read_config_byte(pdev, ASMEDIA_REG_SWITCH, &temp);
	spin_unlock_irqrestore(&gp.lock, flags);

	pci_config_pm_runtime_put(pdev);
	dev_err(&pdev->dev, "ASMEDIA-28xx/18xx Init ASMEDIA_REG_SWITCH = 0x%x \n",temp);
out:
	printk(KERN_INFO "ASMEDIA-28xx/18xx GPIO Driver Init-end \n");
	return err;
}

static void __exit asmedia_gpio_exit(void)
{
	unsigned long flags;

	pci_config_pm_runtime_get(gp.pdev);

	spin_lock_irqsave(&gp.lock, flags);
	//Set GPIO Registers to default value
	pci_write_config_byte(gp.pdev, ASMEDIA_GPIO_OUTPUT, ASMEDIA_GPIO_DEFAULT);
	pci_write_config_byte(gp.pdev, ASMEDIA_GPIO_INPUT, ASMEDIA_GPIO_DEFAULT);
	pci_write_config_byte(gp.pdev, ASMEDIA_GPIO_CTRL, ASMEDIA_GPIO_DEFAULT);
	//Clear PCI_CFG_Switch bit = 0,then we can't access GPIO Registers
	pci_write_config_byte(gp.pdev, ASMEDIA_REG_SWITCH, ASMEDIA_GPIO_DEFAULT);
	spin_unlock_irqrestore(&gp.lock, flags);
	
	pci_config_pm_runtime_put(gp.pdev);

	gpiochip_remove(&gp.chip);
	printk(KERN_INFO "ASMEDIA-28xx/18xx GPIO Driver Exit\n");
	
}

module_init(asmedia_gpio_init);
module_exit(asmedia_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Richard Hsu <Richard_Hsu@asmedia.com.tw>");
MODULE_DESCRIPTION("Asmedia GPIO Driver");
MODULE_VERSION(DRV_VERSION);

