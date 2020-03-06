// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020 Ondrej Jirman <megous@megous.com>
 *
 * This driver is for internal DMA inside Allwinner UDC.
 *
 * TODO:
 * - integrate with musb_{gadget,host}.c
 *   - there are some semi-hardcoded platform specific DMA code paths
 *     there gated by per-platform runtime selected conditions
 *   - figure out max transfer length
 */

#if 0
init TX ep DMA: (see txstate)

   |= MUSB_TXCSR_AUTOSET | MUSB_TXCSR_DMAENAB | MUSB_TXCSR_DMAMODE | BIT(13) /* USBC_BP_TXCSR_D_MODE */

clear TX ep DMA:

  // needs separate writes
  &= ~ ( MUSB_TXCSR_DMAENAB | MUSB_TXCSR_AUTOSET )
  &= ~ ( MUSB_TXCSR_DMAMODE )

init RX ep DMA: (see rxstate)

  |= MUSB_RXCSR_DMAMODE
  |= MUSB_RXCSR_AUTOCLEAR | MUSB_RXCSR_DMAENAB
  &= ~MUSB_RXCSR_DMAMODE
  |= MUSB_RXCSR_DMAMODE

clear RX ep DMA:

  &= ~( MUSB_RXCSR_AUTOCLEAR | MUSB_RXCSR_DMAMODE | MUSB_RXCSR_DMAENAB )

if ep is ready for tx: (we transfer maxpacket chunks only)

   left_len = req->req.length - req->req.actual;
   left_len = left_len - (left_len % ep->ep.maxpacket);
   ep->dma_transfer_len = left_len;

on RX: (same as TX, only maxpacket blocks go via DMA)

   /* cut fragment packet part */
   left_len = req->req.length - req->req.actual;
   left_len = left_len - (left_len % ep->ep.maxpacket);
#endif

#define DEBUG

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/pfn.h>
#include <linux/sizes.h>
#include "musb_core.h"

#define  SUNXI_MUSB_DMA_INTE		0x0500
#define  SUNXI_MUSB_DMA_INTS		0x0504
#define  SUNXI_MUSB_DMA_CHAN_CFN(n)	(0x0540 + (0x10 * n))
#define  SUNXI_MUSB_DMA_SDRAM_ADD(n)	(0x0544 + (0x10 * n))
#define  SUNXI_MUSB_DMA_BC(n)		(0x0548 + (0x10 * n))

#define SUNXI_DMA_MAX_CHANNELS 8

struct sunxi_dma_channel {
	struct dma_channel channel;

	int		index;
	bool		is_allocated;
	bool		is_tx;
	struct musb_hw_ep *hw_ep;
};

struct sunxi_dma_controller {
	struct dma_controller controller;

	struct sunxi_dma_channel channels[SUNXI_DMA_MAX_CHANNELS];
};

static inline struct sunxi_dma_controller *
to_sunxi_dma_controller(struct dma_controller *c)
{
	return container_of(c, struct sunxi_dma_controller, controller);
}

static inline struct sunxi_dma_channel *
to_sunxi_dma_channel(struct dma_channel *c)
{
	return container_of(c, struct sunxi_dma_channel, channel);
}

static struct dma_channel *sunxi_dma_channel_allocate(struct dma_controller *ctl,
				struct musb_hw_ep *hw_ep, u8 is_tx)
{
	struct sunxi_dma_controller *sctl = to_sunxi_dma_controller(ctl);
	struct sunxi_dma_channel *sch = NULL;
	struct musb *musb = ctl->musb;
	int i;

	/*
	 * Find an unused channel, one of 8 available.
	 */
	for (i = 0; i < SUNXI_DMA_MAX_CHANNELS; i++) {
		sch = &sctl->channels[i];
		if (sch->is_allocated)
			continue;

		sch->channel.private_data = ctl;
		sch->channel.status = MUSB_DMA_STATUS_FREE;
		sch->channel.max_len = 0x100000; //XXX: what is max length?
		sch->channel.actual_len = 0;
		//sch->channel.desired_mode = is_tx;
		sch->index = i;
		sch->is_allocated = true;
		sch->hw_ep = hw_ep;
		sch->is_tx = is_tx;

		musb_dbg(musb, "allocated hw_ep=%d, is_tx=0x%x, channel=%d\n",
			 hw_ep->epnum, is_tx, sch->index);

		return &sch->channel;
	}

	return NULL;
}

static int sunxi_dma_is_compatible(struct dma_channel *channel,
				   u16 maxpacket, void *buf, u32 length)
{
	struct sunxi_dma_channel *sch = to_sunxi_dma_channel(channel);
	struct sunxi_dma_controller *sctl =
		to_sunxi_dma_controller(sch->channel.private_data);
	struct musb *musb = sctl->controller.musb;

	musb_dbg(musb, "is_compat chan=%d maxpacket=%d, buf=0x%p, length=%u, tx=%x\n",
		 sch->index, maxpacket, buf, length, sch->is_tx);

	/**
	 * DMA transfer conditions:
	 *
	 * - not EP0
	 * - at least one full packet
	 * - only tx for now
	 */
	return sch->hw_ep->epnum > 0 && length >= maxpacket && sch->is_tx;
}

static int sunxi_dma_channel_program(struct dma_channel *channel,
				u16 packet_sz, u8 mode,
				dma_addr_t dma_addr, u32 len)
{
	struct sunxi_dma_channel *sch = to_sunxi_dma_channel(channel);
	struct sunxi_dma_controller *sctl =
		to_sunxi_dma_controller(sch->channel.private_data);
	struct musb *musb = sctl->controller.musb;
	void __iomem *musb_base = musb->mregs;
	u32 reg;

	BUG_ON(channel->status == MUSB_DMA_STATUS_UNKNOWN ||
		channel->status == MUSB_DMA_STATUS_BUSY);

	musb_dbg(musb,
		"program packet_sz=%d, mode=%d, dma_addr=0x%llx, len=%d is_tx=%d\n",
		packet_sz, mode, (unsigned long long) dma_addr,
		len, sch->is_tx);

	channel->status = MUSB_DMA_STATUS_BUSY;
	channel->actual_len = len - (len % packet_sz);
	if (channel->actual_len == 0)
		return false;

	reg = musb_readl(musb_base, SUNXI_MUSB_DMA_CHAN_CFN(sch->index));
	reg &= ~((0xf << 0) | (1u << 4) | (0x7ff << 16));
	reg |= (sch->hw_ep->epnum & 0xf) << 0;
	reg |= (!sch->is_tx) << 4;
	reg |= (packet_sz & 0x7ff) << 16;
	musb_writel(musb_base, SUNXI_MUSB_DMA_CHAN_CFN(sch->index), reg);

	musb_writel(musb_base, SUNXI_MUSB_DMA_SDRAM_ADD(sch->index), dma_addr);
	musb_writel(musb_base, SUNXI_MUSB_DMA_BC(sch->index), channel->actual_len);

	reg = musb_readw(musb_base, SUNXI_MUSB_DMA_INTE);
	reg &= ~BIT(sch->index);
	musb_writew(musb_base, SUNXI_MUSB_DMA_INTE, reg);

	reg = musb_readw(musb_base, SUNXI_MUSB_DMA_INTE);
	reg |= BIT(sch->index);
	musb_writew(musb_base, SUNXI_MUSB_DMA_INTE, reg);

	/* start dma */
	reg = musb_readl(musb_base, SUNXI_MUSB_DMA_CHAN_CFN(sch->index));
	reg |= BIT(31);
	musb_writel(musb_base, SUNXI_MUSB_DMA_CHAN_CFN(sch->index), reg);

	return true;
}

static int sunxi_dma_channel_abort(struct dma_channel *channel)
{
	struct sunxi_dma_channel *sch = to_sunxi_dma_channel(channel);
	struct sunxi_dma_controller *sctl =
		to_sunxi_dma_controller(sch->channel.private_data);
	struct musb *musb = sctl->controller.musb;
	void __iomem *musb_base = musb->mregs;
	u32 reg;

	musb_dbg(musb, "abort channel=%d, is_tx=%d\n", sch->index, sch->is_tx);

	if (channel->status == MUSB_DMA_STATUS_BUSY) {
		/* disable interrupt enable bit */
		reg = musb_readw(musb_base, SUNXI_MUSB_DMA_INTE);
		reg &= ~BIT(sch->index);
		musb_writew(musb_base, SUNXI_MUSB_DMA_INTE, reg);

		/* disable channel */
		reg = musb_readl(musb_base, SUNXI_MUSB_DMA_CHAN_CFN(sch->index));
		reg &= ~BIT(31);
		musb_writel(musb_base, SUNXI_MUSB_DMA_CHAN_CFN(sch->index), reg);

		channel->status = MUSB_DMA_STATUS_FREE;
		channel->actual_len = 0;
	}

	return 0;
}

static void sunxi_dma_channel_release(struct dma_channel *channel)
{
	struct sunxi_dma_channel *sch = to_sunxi_dma_channel(channel);
	struct sunxi_dma_controller *sctl =
		to_sunxi_dma_controller(sch->channel.private_data);
	struct musb *musb = sctl->controller.musb;

	sunxi_dma_channel_abort(channel);

	musb_dbg(musb, "release channel=%d\n", sch->index);

	sch->is_allocated = false;

	channel->status = MUSB_DMA_STATUS_UNKNOWN;
	channel->actual_len = 0;
}

void sunxi_musb_dma_interrupt(struct musb* musb)
{
	struct sunxi_dma_controller *sctl =
		to_sunxi_dma_controller(musb->dma_controller);
	struct sunxi_dma_channel *sch;
	struct device *dev = musb->controller;
	void __iomem *musb_base = musb->mregs;
	u16 dma_irq, reg;
	int i;

	dma_irq = musb_readw(musb_base, SUNXI_MUSB_DMA_INTS);
	if (!dma_irq)
		return;

	musb_dbg(musb, "dma_irq dma_ints=%x\n", dma_irq);

	/* tx endpoint data transfers */
	for (i = 0; i < SUNXI_DMA_MAX_CHANNELS; i++) {
		if (!(dma_irq & BIT(i)))
			continue;

                sch = &sctl->channels[i];

		/* set 1 to clear pending */
		musb_writew(musb_base, SUNXI_MUSB_DMA_INTS, BIT(i));

		/* disable interrupt enable bit */
		reg = musb_readw(musb_base, SUNXI_MUSB_DMA_INTE);
		reg &= ~BIT(i);
		musb_writew(musb_base, SUNXI_MUSB_DMA_INTE, reg);

		//XXX: also detect DMA error conditions

		if (sch->is_allocated) {
			//XXX: we can just set int_tx, int_rx here
			/*
			if (sch->is_tx)
				musb->int_tx |= BIT(sch->hw_ep->epnum);
			else
				musb->int_rx |= BIT(sch->hw_ep->epnum);
*/
			//XXX: this is almost equivalent
			sch->channel.status = MUSB_DMA_STATUS_FREE;
			musb_dma_completion(musb, sch->hw_ep->epnum, sch->is_tx);
		} else {
			dev_info(dev, "dma_irq spurious interrupt channel=%d\n", i);
		}
	}
}
EXPORT_SYMBOL_GPL(sunxi_musb_dma_interrupt);

struct dma_controller *
sunxi_dma_controller_create(struct musb *musb, void __iomem *base)
{
	struct sunxi_dma_controller *sctl;

	sctl = kzalloc(sizeof(*sctl), GFP_KERNEL);
	if (!sctl)
		return ERR_PTR(-ENOMEM);

	sctl->controller.musb = musb;
	sctl->controller.channel_alloc = sunxi_dma_channel_allocate;
	sctl->controller.channel_release = sunxi_dma_channel_release;
	sctl->controller.is_compatible = sunxi_dma_is_compatible;
	sctl->controller.channel_program = sunxi_dma_channel_program;
	sctl->controller.channel_abort = sunxi_dma_channel_abort;

	return &sctl->controller;
}
EXPORT_SYMBOL_GPL(sunxi_dma_controller_create);

void sunxi_dma_controller_destroy(struct dma_controller *ctl)
{
	struct sunxi_dma_controller *sctl = to_sunxi_dma_controller(ctl);
	struct sunxi_dma_channel *sch;
	int i;

	for (i = 0; i < SUNXI_DMA_MAX_CHANNELS; i++) {
		sch = &sctl->channels[i];
		sunxi_dma_channel_release(&sch->channel);
	}

	kfree(sctl);
}
EXPORT_SYMBOL_GPL(sunxi_dma_controller_destroy);
