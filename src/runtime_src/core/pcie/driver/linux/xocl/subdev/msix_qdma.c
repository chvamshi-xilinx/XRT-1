/*
 * A GEM style device manager for PCIe based OpenCL accelerators.
 *
 * Copyright (C) 2019 Xilinx, Inc. All rights reserved.
 *
 * Authors:
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/version.h>
#include <linux/eventfd.h>
#include "../xocl_drv.h"
#include "../lib/libqdma/QDMA/linux-kernel/driver/libqdma/libqdma_export.h"
#include "../lib/libqdma/QDMA/linux-kernel/driver/libqdma/qdma_ul_ext.h"
#include "qdma_ioctl.h"

/*
 * Interrupt controls
 */
#define XOCL_MSIX_QDMA "msix_qdma"
#define QDMA_MAX_INTR		16
#define QDMA_USER_INTR_MASK	0xffff
#define QDMA_QSETS_MAX		256
#define QDMA_QSETS_BASE		0

#define QDMA_REQ_TIMEOUT_MS	10000

unsigned int qdma_max_channel = 8;
module_param(qdma_max_channel, uint, 0644);
MODULE_PARM_DESC(qdma_max_channel, "Set number of channels for qdma, default is 8");

static unsigned int qdma_interrupt_mode = DIRECT_INTR_MODE;
module_param(qdma_interrupt_mode, uint, 0644);
MODULE_PARM_DESC(interrupt_mode, "0:auto, 1:poll, 2:direct, 3:intr_ring, default is 2");



struct qdma_irq {
	struct eventfd_ctx	*event_ctx;
	bool			    in_use;
	bool			    enabled;
	irq_handler_t	    handler;
	void			    *arg;
};

struct xocl_qdma {
	unsigned long 		    dma_hndl;
	struct qdma_dev_conf	dev_conf;
	struct platform_device	*pdev;
	struct qdma_irq		    user_msix_table[QDMA_MAX_INTR];
	u32			            user_msix_mask;
	spinlock_t		        user_msix_table_lock;
};

static int user_intr_register(struct platform_device *pdev, u32 intr,
	irq_handler_t handler, void *arg, int event_fd)
{
	struct xocl_qdma *qdma;
	struct eventfd_ctx *trigger = ERR_PTR(-EINVAL);
	unsigned long flags;
	int ret;

	qdma = platform_get_drvdata(pdev);

	if (!((1 << intr) & qdma->user_msix_mask)) {
		xocl_err(&pdev->dev, "Invalid intr %d, user intr mask %x",
				intr, qdma->user_msix_mask);
		return -EINVAL;
	}

	if (event_fd >= 0) {
		trigger = eventfd_ctx_fdget(event_fd);
		if (IS_ERR(trigger)) {
			xocl_err(&pdev->dev, "get event ctx failed");
			return -EFAULT;
		}
	}

	spin_lock_irqsave(&qdma->user_msix_table_lock, flags);
	if (qdma->user_msix_table[intr].in_use) {
		xocl_err(&pdev->dev, "IRQ %d is in use", intr);
		ret = -EPERM;
		goto failed;
	}

	qdma->user_msix_table[intr].event_ctx = trigger;
	qdma->user_msix_table[intr].handler = handler;
	qdma->user_msix_table[intr].arg = arg;
	qdma->user_msix_table[intr].in_use = true;

	spin_unlock_irqrestore(&qdma->user_msix_table_lock, flags);


	return 0;

failed:
	spin_unlock_irqrestore(&qdma->user_msix_table_lock, flags);
	if (!IS_ERR(trigger))
		eventfd_ctx_put(trigger);

	return ret;
}

static int user_intr_unreg(struct platform_device *pdev, u32 intr)
{
	struct xocl_qdma *qdma;
	unsigned long flags;
	int ret;

	qdma= platform_get_drvdata(pdev);

	if (!((1 << intr) & qdma->user_msix_mask)) {
		xocl_err(&pdev->dev, "Invalid intr %d, user intr mask %x",
				intr, qdma->user_msix_mask);
		return -EINVAL;
	}

	spin_lock_irqsave(&qdma->user_msix_table_lock, flags);
	if (!qdma->user_msix_table[intr].in_use) {
		ret = -EINVAL;
		goto failed;
	}

	qdma->user_msix_table[intr].handler = NULL;
	qdma->user_msix_table[intr].arg = NULL;
	qdma->user_msix_table[intr].in_use = false;

	spin_unlock_irqrestore(&qdma->user_msix_table_lock, flags);
	return 0;
failed:
	spin_unlock_irqrestore(&qdma->user_msix_table_lock, flags);


	return ret;
}

static int user_intr_config(struct platform_device *pdev, u32 intr, bool en)
{
	return 0;
}

static void qdma_isr(unsigned long dma_handle, int irq, unsigned long arg)
{
	struct xocl_qdma *qdma = (struct xocl_qdma *)arg;
	struct qdma_irq *irq_entry;

	irq_entry = &qdma->user_msix_table[irq];
	if (irq_entry->in_use)
		irq_entry->handler(irq, irq_entry->arg);
	else
		xocl_info(&qdma->pdev->dev, "user irq %d not in use", irq);
}

static struct xocl_msix_funcs msix_qdma_ops = {
	.user_intr_register = user_intr_register,
	.user_intr_config = user_intr_config,
	.user_intr_unreg = user_intr_unreg
};

static int msix_qdma_probe(struct platform_device *pdev)
{
	struct xocl_qdma *qdma = NULL;
	struct qdma_dev_conf *conf;
	xdev_handle_t	xdev;
	struct resource *res = NULL;
	int	i, ret = 0, dma_bar = -1;

	xdev = xocl_get_xdev(pdev);

	qdma = xocl_drvinst_alloc(&pdev->dev, sizeof(*qdma));
	if (!qdma) {
		xocl_err(&pdev->dev, "alloc mm dev failed");
		ret = -ENOMEM;
		goto failed;
	}

	qdma->pdev = pdev;
	platform_set_drvdata(pdev, qdma);

	for (i = 0, res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		res;
		res = platform_get_resource(pdev, IORESOURCE_MEM, ++i)) {
		if (!strncmp(res->name, NODE_QDMA, strlen(NODE_QDMA))) {
			ret = xocl_ioaddr_to_baroff(xdev, res->start, &dma_bar,
							NULL);
			if (ret) {
				xocl_err(&pdev->dev,
					"Invalid resource %pR", res);
				return -EINVAL;
			}
		} else {
			xocl_err(&pdev->dev, "Unknown resource: %s", res->name);
			return -EINVAL;
		}
	}

	if (dma_bar == -1) {
		xocl_err(&pdev->dev, "missing resource, dma_bar %d", dma_bar);
		return -EINVAL;
	}

	conf = &qdma->dev_conf;
	memset(conf, 0, sizeof(*conf));
	conf->pdev = XDEV(xdev)->pdev;
	conf->master_pf = 1;
	conf->qsets_base = QDMA_QSETS_BASE;
	conf->qsets_max = QDMA_QSETS_MAX;
	conf->bar_num_config = dma_bar;
	conf->bar_num_user = -1;
	conf->bar_num_bypass = -1;
	conf->data_msix_qvec_max = 1;
	conf->user_msix_qvec_max = 16;
	conf->msix_qvec_max = 32;
	conf->qdma_drv_mode = qdma_interrupt_mode;

	conf->fp_user_isr_handler = (void*)qdma_isr;
	conf->uld = (unsigned long)qdma;

	xocl_info(&pdev->dev, "dma %d, mode 0x%x.\n",
		dma_bar, conf->qdma_drv_mode);
	ret = qdma_device_open(XOCL_MODULE_NAME, conf, &qdma->dma_hndl);
	if (ret < 0) {
		xocl_err(&pdev->dev, "QDMA Device Open failed");
		goto failed;
	}

	ret = qdma_device_get_config(qdma->dma_hndl, &qdma->dev_conf, NULL, 0);
	if (ret) {
		xocl_err(&pdev->dev, "Failed to get device info");
		goto failed;
	}

	qdma->user_msix_mask = QDMA_USER_INTR_MASK;

	spin_lock_init(&qdma->user_msix_table_lock);

	return 0;

failed:
	if (qdma) {

		if (qdma->dma_hndl)
			qdma_device_close(XDEV(xdev)->pdev, qdma->dma_hndl);

		xocl_drvinst_release(qdma, NULL);
	}

	platform_set_drvdata(pdev, NULL);

	return ret;
}

static int msix_qdma_remove(struct platform_device *pdev)
{
	struct xocl_qdma *qdma= platform_get_drvdata(pdev);
	xdev_handle_t xdev;
	struct qdma_irq *irq_entry;
	void *hdl;
	int i;

	xocl_drvinst_release(qdma, &hdl);

	if (!qdma) {
		xocl_err(&pdev->dev, "driver data is NULL");
		return -EINVAL;
	}

	xdev = xocl_get_xdev(pdev);


	qdma_device_close(XDEV(xdev)->pdev, qdma->dma_hndl);

	for (i = 0; i < ARRAY_SIZE(qdma->user_msix_table); i++) {
		irq_entry = &qdma->user_msix_table[i];
		if (irq_entry->in_use) {
			if (irq_entry->enabled)
				xocl_err(&pdev->dev,
					"ERROR: Interrupt %d is still on", i);
			if(!IS_ERR_OR_NULL(irq_entry->event_ctx))
				eventfd_ctx_put(irq_entry->event_ctx);
		}
	}

	platform_set_drvdata(pdev, NULL);
	xocl_drvinst_free(hdl);

	return 0;
}

struct xocl_drv_private msix_qdma_priv = {
	.ops = &msix_qdma_ops,
};

static struct platform_device_id msix_qdma_id_table[] = {
	{ XOCL_DEVNAME(XOCL_MSIX_QDMA), (kernel_ulong_t)&msix_qdma_priv },
	{ },
};

static struct platform_driver	msix_qdma_driver = {
	.probe		= msix_qdma_probe,
	.remove		= msix_qdma_remove,
	.driver		= {
		.name = XOCL_DEVNAME(XOCL_MSIX_QDMA),
	},
	.id_table	= msix_qdma_id_table,
};

int __init xocl_init_msix_qdma(void)
{
	return platform_driver_register(&msix_qdma_driver);
}

void xocl_fini_msix_qdma(void)
{
	return platform_driver_unregister(&msix_qdma_driver);
}
