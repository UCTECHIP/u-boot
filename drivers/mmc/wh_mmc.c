// SPDX-License-Identifier: GPL-2.0+
/*
 * Ingenic WH MMC driver
 *
 * Copyright (c) 2013 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 */

#include <common.h>
#include <malloc.h>
#include <mmc.h>
#include <asm/io.h>
#include <asm/unaligned.h>
#include <errno.h>
#include <wait_bit.h>

//#define MMC_PRINT

#define WH_SD_ARGU                     0x00   // Command Argument Register (32,RW)
#define WH_SD_CMD                      0x04   // Command Setting Register (16,RW)
#define WH_SD_STATUS                   0x08   // Card Status Register (16,R)
#define WH_SD_RSP                      0x0c   // Command Response (32,R)
#define WH_SD_CS                       0x1c   // Controller Setting (Not in use) (16,R)
#define WH_SD_BSR                      0x20   // Block Size Register (16,R)
#define WH_SD_PCR                      0x24   // Power Control Register (8,R)
#define WH_SD_SWR                      0x28   // Software reset Register (8,RW)
#define WH_SD_TOR                      0x2c   // Timeout Register (16,RW)
#define WH_SD_NIS                      0x30   // Normal Interrupt Status Register (16,RW)
#define WH_SD_EIS                      0x34   // Error Interrupt Status Register (16,RW)
#define WH_SD_NIE                      0x38   // Normal Interrupt Enable (16,RW)
#define WH_SD_EIE                      0x3c   // Error Interrupt Enable Register (16,RW)
#define WH_SD_CAP                      0x48   // Capability Register (16,R)
#define WH_SD_CDR                      0x4c   // Clock Divider Register (8,RW)
#define WH_SD_BDBS					   0x50   // BD Status Register (16,RW)
#define WH_SD_DIS					   0x54   // Data Interrupt Status Register (16,RW)
#define WH_SD_DIE                      0x58   // Data Interrupt Enable Register (16,RW)
#define WH_SD_BDRX_L                   0x60   // BD RX (Low 32 bit) (32,W)
#define WH_SD_BDRX_H                   0x64   // BD RX (High 32 bit) (32,W)
#define WH_SD_BDTX_L                   0x80   // BD TX (Low 32 bit) (32,W)
#define WH_SD_BDTX_H                   0x84   // BD TX (High 32 bit) (32,W)
#define WH_SD_CUSTOM				   0x100  // CUSTOM  (16,RW)
#define WH_SD_RSP_2					   0x104
#define WH_SD_RSP_3					   0x108
#define WH_SD_RSP_4					   0x10c

//WordSelect(136bit Response)
#define WH_WORD_0 0x00
#define WH_WORD_1 0x40
#define WH_WORD_2 0x80
#define WH_WORD_3 0xC0

//Commands
#define WH_CMD2 0x200
#define WH_CMD3 0x300
#define WH_CMD7 0x700
#define WH_CMD8  0x800
#define WH_CMD9  0x900
#define WH_CMD13  0xD00
#define WH_CMD16  0x1000
#define WH_CMD17  0x1100

#define WH_ACMD41 0x2900
#define WH_ACMD6 0x600
#define WH_CMD55 0x3700

//CMD ARG
//CMD8
#define WH_VHS  0x100 //2.7-3.6V
#define WH_CHECK_PATTERN 0xAA
//ACMD41
#define WH_BUSY 0x80000000
#define WH_HCS 0x40000000
#define WH_VOLTAGE_MASK 0xFFFFFF

//CMD7
#define WH_READY_FOR_DATA   0x100
#define WH_CARD_STATUS_STB  0x600

//Command setting
#define WH_CICE 0x10
#define WH_CRCE 0x08
#define WH_RSP_48 0x2
#define WH_RSP_136 0x1

//Status Mask
//Card status
#define WH_CARD_BUSY 0x1

//Normal interupt status
#define WH_CMD_COMPLETE 0x1
#define WH_EI 0x8000

//Error interupt status
#define WH_CMD_TIMEOUT 0x1
#define WH_CCRC 0x1
#define WH_CIE  0x8

// Data Interrupt Status Register
#define WH_DATA_TRE		(1 << 4)
#define WH_DATA_CMDE	(1 << 3)
#define WH_DATA_FIFOE	(1 << 2)
#define WH_DATA_MRC		(1 << 1)
#define WH_DATA_TRS		(1 << 0)

#define WH_CID_MID_MASK 0x7F8000
#define WH_CID_OID_MASK 0x7FFF
#define WH_CID_B1 0x7F800000
#define WH_CID_B2 0x7F8000
#define WH_CID_B3 0x7F80
#define WH_CID_B4 0x7F

#define WH_RCA_RCA_MASK 0xFFFF0000

struct wh_mmc_plat {
	struct mmc_config cfg;
	struct mmc mmc;
};

struct wh_mmc_priv {
	void __iomem		*regs;
	u32			flags;
/* priv flags */
#define WH_MMC_BUS_WIDTH_MASK	0x3
#define WH_MMC_BUS_WIDTH_1	0x0
#define WH_MMC_BUS_WIDTH_4	0x2
#define WH_MMC_BUS_WIDTH_8	0x3
#define WH_MMC_SENT_INIT	BIT(2)
};

static int wh_mmc_clock_rate(void)
{

#ifdef MMC_PRINT
	printf("Enter wh_mmc_clock_rate\n");
#endif

	return 50000000;
}

#if CONFIG_IS_ENABLED(MMC_WRITE)
static inline void wh_mmc_write_data(struct wh_mmc_priv *priv, struct mmc_data *data)
{

#ifdef MMC_PRINT
	printf("Enter wh_mmc_write_data\n");
#endif

}
#else
static void wh_mmc_write_data(struct wh_mmc_priv *priv, struct mmc_data *data)
{}
#endif

static inline int wh_mmc_read_data(struct wh_mmc_priv *priv, struct mmc_cmd *cmd, struct mmc_data *data)
{
	int sz = data->blocks * data->blocksize;
	void *buf = data->dest;

	void __iomem	*controller_databuf;
	controller_databuf = (void *)(unsigned long)0x4b000000;

	int stat;

#ifdef MMC_PRINT
	printf("Enter wh_mmc_read_data, size=%d\n",sz);
#endif

	if ( cmd->cmdidx == 13 )
		writel(0x1, priv->regs + WH_SD_CUSTOM);

	if ( cmd->cmdidx == 6 )
		writel(0x3, priv->regs + WH_SD_CUSTOM);

	asm("fence.i");
	writel(0x4b000000 ,priv->regs + WH_SD_BDRX_L);
	writel(0x4b000000 ,priv->regs + WH_SD_BDRX_L);
	writel(cmd->cmdarg ,priv->regs + WH_SD_BDRX_L);
	writel(cmd->cmdarg ,priv->regs + WH_SD_BDRX_L);

	while( ( readl(priv->regs + WH_SD_DIS) & 0x1 ) != 0x1 )
	{
		stat = readl(priv->regs + WH_SD_DIS);
		if (stat & WH_DATA_MRC)
			return -ETIMEDOUT;
		if (stat & WH_DATA_CMDE)
			return -EINVAL;
		if (stat & WH_DATA_FIFOE)
			return -ETIMEDOUT;
		if (stat & WH_DATA_MRC)
			return -EINVAL;
	}
	writel(0 ,priv->regs +WH_SD_NIS);
	writel(0 ,priv->regs +WH_SD_EIS);
	writel(0 ,priv->regs +WH_SD_DIS);
	writel(0, priv->regs +WH_SD_CUSTOM);

	memcpy(buf, controller_databuf, sz);
	buf += sz;
	sz = 0;

	return 0;
}

static int _wh_mmc_send_cmd(struct mmc *mmc, struct wh_mmc_priv *priv,
			   struct mmc_cmd *cmd, struct mmc_data *data)
{
	u32 cmdat = 0;

	if ( cmd->cmdidx == 16 )
        {
                return 0;
        }

	/* send command */
	switch (cmd->resp_type) {
	case MMC_RSP_NONE:
		break;
	case MMC_RSP_R1:
	case MMC_RSP_R1b:
		cmdat |= WH_RSP_48;
		break;
	case MMC_RSP_R2:
		cmdat |= WH_RSP_136;
		break;
	case MMC_RSP_R3:
		cmdat |= WH_RSP_48;
		break;
	default:
		break;
	}

	cmdat |= (WH_CICE | WH_CRCE);
	cmdat |= (cmd->cmdidx << 8);

	writel(cmdat , priv->regs + WH_SD_CMD);
	writel(cmd->cmdarg	, priv->regs + WH_SD_ARGU);

	/*  wait for completion */
	while( (readl(priv->regs + WH_SD_NIS) & 0x1)!= 0x1 );
	writel(0 , priv->regs + WH_SD_NIS);

	/* read the response */
	if (cmd->resp_type & MMC_RSP_PRESENT) {
		cmd->response[0] = readl(priv->regs + WH_SD_RSP);
		cmd->response[1] = 0;
		cmd->response[2] = 0;
		cmd->response[3] = 0;
	}

	if (cmd->resp_type & MMC_RSP_136) {
		cmd->response[0] = readl(priv->regs + WH_SD_RSP);
		cmd->response[1] = readl(priv->regs + WH_SD_RSP_2);
		cmd->response[2] = readl(priv->regs + WH_SD_RSP_3);
		cmd->response[3] = readl(priv->regs + WH_SD_RSP_4);
	}

	return 0;
}

static int _wh_mmc_send_cmd_withdata(struct mmc *mmc, struct wh_mmc_priv *priv,
			   struct mmc_cmd *cmd, struct mmc_data *data)
{
	int ret;

	if ( cmd->cmdidx ==51 )
	{
		void *buf = data->dest;
		put_unaligned_le32(0x03803502, buf);
		buf += 4;
		put_unaligned_le32(0x00000000, buf);
		buf += 4;
		return	0;
	}

	if ( cmd->cmdidx == 13 )
	{
		return 0;
	}

	if ( cmd->cmdidx == 6 )
	{
		if( mmc->bus_width == 1 )
		{
			return 0;
		}
		return	0;
	}

	if (data->flags & MMC_DATA_WRITE)
		wh_mmc_write_data(priv, data);
	else if (data->flags & MMC_DATA_READ) {
		ret = wh_mmc_read_data(priv, cmd, data);
		if (ret)
		return ret;
	}

	return 0;
}


static int wh_mmc_send_cmd(struct mmc *mmc, struct wh_mmc_priv *priv,
			   struct mmc_cmd *cmd, struct mmc_data *data)
{
	if (data)
	{
#ifdef MMC_PRINT
		printf("Enter wh_mmc_send_cmd_withdata, cmdidx:%d , cmdarg:%x\n", cmd->cmdidx, cmd->cmdarg);
#endif
		_wh_mmc_send_cmd_withdata(mmc, priv, cmd, data);
		return 0;
	}
	else
	{
#ifdef MMC_PRINT
		printf("Enter wh_mmc_send_cmd, cmdidx:%d , cmdarg:%x\n", cmd->cmdidx, cmd->cmdarg);
#endif
		_wh_mmc_send_cmd(mmc, priv, cmd, data);
		return 0;
	}
}

static int wh_mmc_set_ios(struct mmc *mmc, struct wh_mmc_priv *priv)
{
	u32 real_rate = wh_mmc_clock_rate();
	u8 clk_div = 0;

#ifdef MMC_PRINT
	printf("Enter wh_mmc_set_ios\n");
#endif

	/* calculate clock divide */
	while ((real_rate > mmc->clock) && (clk_div < 7)) {
		real_rate >>= 1;
		clk_div++;
	}

	/* set the bus width for the next command */
	priv->flags &= ~WH_MMC_BUS_WIDTH_MASK;
	if (mmc->bus_width == 8)
		priv->flags |= WH_MMC_BUS_WIDTH_8;
	else if (mmc->bus_width == 4)
		priv->flags |= WH_MMC_BUS_WIDTH_4;
	else
		priv->flags |= WH_MMC_BUS_WIDTH_1;

	return 0;
}

static int wh_mmc_core_init(struct mmc *mmc)
{
	struct wh_mmc_priv *priv = mmc->priv;

	/* Maximum timeouts */
	writel(0x8ff, priv->regs + WH_SD_TOR);

	/* Set Clock Divider and Reset SD Controller*/
	writel(0x1, priv->regs + WH_SD_SWR);
	writel(0x2, priv->regs + WH_SD_CDR);
	writel(0x0, priv->regs + WH_SD_SWR);

	return 0;
}

#if !CONFIG_IS_ENABLED(DM_MMC)

static int wh_mmc_legacy_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd,
				  struct mmc_data *data)
{
	struct wh_mmc_priv *priv = mmc->priv;

	return wh_mmc_send_cmd(mmc, priv, cmd, data);
}

static int wh_mmc_legacy_set_ios(struct mmc *mmc)
{
	struct wh_mmc_priv *priv = mmc->priv;

	return wh_mmc_set_ios(mmc, priv);
};

static const struct mmc_ops wh_msc_ops = {
	.send_cmd	= wh_mmc_legacy_send_cmd,
	.set_ios	= wh_mmc_legacy_set_ios,
	.init		= wh_mmc_core_init,
};

static struct wh_mmc_priv wh_mmc_priv_static;
static struct wh_mmc_plat wh_mmc_plat_static = {
	.cfg = {
		.name = "MSC",
		.ops = &wh_msc_ops,

		.voltages = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30 |
			    MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
			    MMC_VDD_33_34 | MMC_VDD_34_35 | MMC_VDD_35_36,
		.host_caps = MMC_MODE_4BIT | MMC_MODE_HS_52MHz | MMC_MODE_HS,

		.f_min = 375000,
		.f_max = 48000000,
		.b_max = CONFIG_SYS_MMC_MAX_BLK_COUNT,
	},
};

int wh_mmc_init(void __iomem *base)
{
	struct mmc *mmc;

	wh_mmc_priv_static.regs = base;

	mmc = mmc_create(&wh_mmc_plat_static.cfg, &wh_mmc_priv_static);

	return mmc ? 0 : -ENODEV;
}

#else /* CONFIG_DM_MMC */

#include <dm.h>
DECLARE_GLOBAL_DATA_PTR;

static int wh_mmc_dm_send_cmd(struct udevice *dev, struct mmc_cmd *cmd,
			      struct mmc_data *data)
{
	struct wh_mmc_priv *priv = dev_get_priv(dev);
	struct mmc *mmc = mmc_get_mmc_dev(dev);

	return wh_mmc_send_cmd(mmc, priv, cmd, data);
}

static int wh_mmc_dm_set_ios(struct udevice *dev)
{
	struct wh_mmc_priv *priv = dev_get_priv(dev);
	struct mmc *mmc = mmc_get_mmc_dev(dev);

	return wh_mmc_set_ios(mmc, priv);
};

static const struct dm_mmc_ops wh_msc_ops = {
	.send_cmd	= wh_mmc_dm_send_cmd,
	.set_ios	= wh_mmc_dm_set_ios,
};

static int wh_mmc_ofdata_to_platdata(struct udevice *dev)
{
	struct wh_mmc_priv *priv = dev_get_priv(dev);
	struct wh_mmc_plat *plat = dev_get_platdata(dev);
	struct mmc_config *cfg;
	int ret;

	priv->regs = map_physmem(dev_read_addr(dev), 0x400, MAP_NOCACHE);
//	priv->regs = dev_read_addr(dev);
	cfg = &plat->cfg;

	cfg->name = "MSC";
	cfg->host_caps = MMC_MODE_HS_52MHz | MMC_MODE_HS | MMC_MODE_4BIT;

	ret = mmc_of_parse(dev, cfg);
	if (ret < 0) {
		dev_err(dev, "failed to parse host caps\n");
		return ret;
	}

	cfg->f_min = 400000;
	//cfg->f_max = 52000000;

	cfg->voltages = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195;
	cfg->b_max = 1;//CONFIG_SYS_MMC_MAX_BLK_COUNT;

	return 0;
}

static int wh_mmc_bind(struct udevice *dev)
{
	struct wh_mmc_plat *plat = dev_get_platdata(dev);

	return mmc_bind(dev, &plat->mmc, &plat->cfg);
}

static int wh_mmc_probe(struct udevice *dev)
{
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct wh_mmc_priv *priv = dev_get_priv(dev);
	struct wh_mmc_plat *plat = dev_get_platdata(dev);

	plat->mmc.priv = priv;
	upriv->mmc = &plat->mmc;
	return wh_mmc_core_init(&plat->mmc);
}

static const struct udevice_id wh_mmc_ids[] = {
	{ .compatible = "uctechip,wh-mmc" },
	{ }
};

U_BOOT_DRIVER(wh_mmc_drv) = {
	.name			= "wh_mmc",
	.id			= UCLASS_MMC,
	.of_match		= wh_mmc_ids,
	.ofdata_to_platdata	= wh_mmc_ofdata_to_platdata,
	.bind			= wh_mmc_bind,
	.probe			= wh_mmc_probe,
	.priv_auto_alloc_size	= sizeof(struct wh_mmc_priv),
	.platdata_auto_alloc_size = sizeof(struct wh_mmc_plat),
	.ops			= &wh_msc_ops,
};
#endif /* CONFIG_DM_MMC */
