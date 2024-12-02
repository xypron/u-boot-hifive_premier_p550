// SPDX-License-Identifier: GPL-2.0
/*
 *
 * Copyright 2024, Beijing ESWIN Computing Technology Co., Ltd.. All rights reserved.
 * SPDX-License-Identifier: GPL-2.0
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * Authors: liangshuang <liangshuang@eswincomputing.com>
 */

#include <common.h>
#include <dm.h>
#include <dt-structs.h>
#include <linux/err.h>
#include <linux/libfdt.h>
#include <malloc.h>
#include <mapmem.h>
#include <sdhci.h>
#include <clk.h>
#include <reset.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <dm/device_compat.h>
#include "eswin_sdhci.h"

/* 400KHz is max freq for card ID etc. Use that as min */
#define SD_MIN_FREQ 400000

extern void eswin_disable_card_clk(struct sdhci_host *host);
extern void eswin_enable_card_clk(struct sdhci_host *host);
extern void eswin_sdhci_set_clock(struct sdhci_host *host, unsigned int div_in);
extern unsigned int eswin_convert_drive_impedance_ohm(struct udevice *dev, unsigned int dr_ohm);
extern int eswin_sdhci_execute_tuning(struct mmc *mmc, u8 opcode);
extern void sdhci_do_enable_v4_mode(struct sdhci_host *host);
extern int eswin_sdhci_reset_init(struct udevice *dev, struct eswin_sdhci_data *eswin_sdhci);

struct eswin_sd_plat {
	struct mmc_config cfg;
	struct mmc mmc;
};

static void eswin_sdhci_set_voltage(struct sdhci_host *host)
{
	struct mmc *mmc = (struct mmc *)host->mmc;
	u32 ctrl;

	ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);

	switch (mmc->signal_voltage) {
	case MMC_SIGNAL_VOLTAGE_330:
		ctrl &= ~SDHCI_CTRL_VDD_180;
		sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);

		/* Wait for 5ms */
		mdelay(5);

		/* 3.3V regulator output should be stable within 5 ms */
		ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
		if (ctrl & SDHCI_CTRL_VDD_180) {
			pr_err("3.3V regulator output did not become stable\n");
			return;
		}

		break;
	case MMC_SIGNAL_VOLTAGE_180:
		ctrl |= SDHCI_CTRL_VDD_180;
		sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);

		/* Wait for 5 ms */
		mdelay(5);

		/* 1.8V regulator output has to be stable within 5 ms */
		ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
		if (!(ctrl & SDHCI_CTRL_VDD_180)) {
			pr_err("1.8V regulator output did not become stable\n");
			return;
		}

		break;
	default:
		/* No signal voltage switch required */
		return;
	}
}

static void eswin_sd_set_control_reg(struct sdhci_host *host)
{
	struct mmc *mmc = (struct mmc *)host->mmc;

	if (!IS_SD(host->mmc)) return;

	sdhci_do_enable_v4_mode(host);

	eswin_sdhci_set_voltage(host);

	if (mmc->selected_mode > SD_HS && mmc->selected_mode <= MMC_HS_200)
		sdhci_set_uhs_timing(host);
}

static void eswin_sdhci_sdio_config_phy_delay(struct sdhci_host *host, int delay)
{
	delay &= PHY_CLK_MAX_DELAY_MASK;

	/*phy clk delay line config*/
	sdhci_writeb(host, PHY_UPDATE_DELAY_CODE, PHY_SDCLKDL_CNFG_R);
	sdhci_writeb(host, delay, PHY_SDCLKDL_DC_R);
	sdhci_writeb(host, 0x0, PHY_SDCLKDL_CNFG_R);
}

static void eswin_sdhci_sdio_config_phy(struct sdhci_host *host)
{
	unsigned int val = 0;
	unsigned int drv = 0;
	struct udevice *dev = host->mmc->dev;
		struct eswin_sdhci_data *eswin_sdhci = dev_get_priv(dev);
	struct eswin_sdhci_phy_data *phy = &eswin_sdhci->phy;

	drv = phy->drive_impedance << PHY_PAD_SP_DRIVE_SHIF;

	eswin_disable_card_clk(host);

	/* reset phy,config phy's pad */
	sdhci_writel(host, drv | (~PHY_RSTN), PHY_CNFG_R);
	/*CMDPAD_CNFS*/
	val = (PHY_SLEW_2 << PHY_TX_SLEW_CTRL_P_BIT_SHIFT) | (PHY_SLEW_2 << PHY_TX_SLEW_CTRL_N_BIT_SHIFT) |
		  (phy->enable_cmd_pullup << PHY_PULL_BIT_SHIF) | PHY_PAD_RXSEL_1;
	sdhci_writew(host, val, PHY_CMDPAD_CNFG_R);

	/*DATA PAD CNFG*/
	val = (PHY_SLEW_2 << PHY_TX_SLEW_CTRL_P_BIT_SHIFT) | (PHY_SLEW_2 << PHY_TX_SLEW_CTRL_N_BIT_SHIFT) |
		  (phy->enable_data_pullup << PHY_PULL_BIT_SHIF) | PHY_PAD_RXSEL_1;
	sdhci_writew(host, val, PHY_DATAPAD_CNFG_R);

	/*Clock PAD Setting*/
	val = (PHY_SLEW_2 << PHY_TX_SLEW_CTRL_P_BIT_SHIFT) | (PHY_SLEW_2 << PHY_TX_SLEW_CTRL_N_BIT_SHIFT) | PHY_PAD_RXSEL_0;
	sdhci_writew(host, val, PHY_CLKPAD_CNFG_R);

	/*PHY RSTN PAD setting*/
	val = (PHY_SLEW_2 << PHY_TX_SLEW_CTRL_P_BIT_SHIFT) | (PHY_SLEW_2 << PHY_TX_SLEW_CTRL_N_BIT_SHIFT) |
		  (PHY_PULL_UP << PHY_PULL_BIT_SHIF) | PHY_PAD_RXSEL_1;
	sdhci_writew(host, val, PHY_RSTNPAD_CNFG_R);

	sdhci_writel(host, drv | PHY_RSTN, PHY_CNFG_R);

	eswin_sdhci_sdio_config_phy_delay(host, phy->delay_code);

	eswin_enable_card_clk(host);
}

static int eswin_sdhci_sdio_set_ios_post(struct sdhci_host *host)
{
	eswin_sdhci_sdio_config_phy(host);
	return 0;
}

static struct sdhci_ops eswin_sd_ops = {
	.platform_execute_tuning	= &eswin_sdhci_execute_tuning,
	.set_control_reg = &eswin_sd_set_control_reg,
	.set_clock = &eswin_sdhci_set_clock,
	.set_ios_post = &eswin_sdhci_sdio_set_ios_post,
};

static void sdhci_sd_phy_poweron(struct sdhci_host *host)
{
	unsigned long  io_mem =  (unsigned long)host->ioaddr;
	unsigned long hsp_io_mem = HOST_DIE_OFFSET(host) + HSP_IO_MEM;
	int ret = 0;

	/*CONFIGURE IN HSPTOP, VOLTAGE*/
	writel(0x10010101, (void *)(hsp_io_mem + 0x608));
	writel(0x1, (void *)(hsp_io_mem + 0x60c));
	writel(0x2000000, (void *)(hsp_io_mem + 0x0b08));

	/*************reset sd****************/
	writeb(0x1, (void *)(io_mem + SDHCI_SOFTWARE_RESET));
	/*negedge data sent out enable MSHC_CTRL_R -- negedge data sent out enable*/
	writew(0x2, (void *)(io_mem + 0x508));

	writeb(0xf, (void *)(io_mem +   SDHCI_POWER_CONTROL));
	/*TOUT_CTRL_R.TOUR_CNT*/
	writew(0x0, (void *)(io_mem + SDHCI_HOST_CONTROL2));

	writew(0xc, (void *)(io_mem + 0x52C));

	writeb(0x0, (void *)(io_mem +  + 0x31c));
	writeb(0x0, (void *)(io_mem +  + 0x31d));
	writeb(0x8, (void *)(io_mem +  + 0x320));
	writeb(0x8, (void *)(io_mem +  + 0x321));

	while (1)
	{
		ret = readl((void *)(io_mem + DWC_MSHC_PTR_PHY_R));
		if (ret == 0x2)
		{
			debug("Phy's Power on!\r\n");
			break;
		}
	}
}

void sdhci_sd_pre_init(struct sdhci_host *host)
{
	u32 reg_val;
	volatile u32 *addr;

	/*sw rst*/
	addr = (u32 *)(HOST_DIE_OFFSET(host) + ESWIN_HSPDMA_RST_CTRL);
	reg_val = *addr;
	reg_val |= ESWIN_HSPDMA_SD_RST;
	*addr = reg_val;

	sdhci_sd_phy_poweron(host);
}

static int eswin_sd_clk_reset_init(struct udevice *dev)
{
	int ret = 0;
	struct clk *clk_spll2_fout3;
	struct clk *clk_mux;
	struct eswin_sdhci_data *eswin_sdhci = dev_get_priv(dev);

	eswin_sdhci->aclk = devm_clk_get(dev, "aclk");
	if (IS_ERR(eswin_sdhci->aclk)) {
		dev_err(dev, "aclk clock not found.\n");
		return PTR_ERR(eswin_sdhci->aclk);
	}

	eswin_sdhci->clk_ahb = devm_clk_get(dev, "clk_ahb");
	if (IS_ERR(eswin_sdhci->clk_ahb)) {
		dev_err(dev, "clk_ahb clock not found.\n");
		return PTR_ERR(eswin_sdhci->clk_ahb);
	}

	eswin_sdhci->clk_xin = devm_clk_get(dev, "clk_xin");
	if (IS_ERR(eswin_sdhci->clk_xin)) {
		dev_err(dev, "clk_xin clock not found.\n");
		return PTR_ERR(eswin_sdhci->clk_xin);
	}

	clk_spll2_fout3 = devm_clk_get(dev, "clk_spll2_fout3");

	if (IS_ERR(clk_spll2_fout3)) {
		dev_err(dev, "clk_spll2_fout3 clock not found.\n");
		return PTR_ERR(clk_spll2_fout3);
	}

	clk_mux = devm_clk_get(dev, "clk_mux1_1");
	if (IS_ERR(clk_mux)) {
		dev_err(dev, "clk_mux1_1 clock not found.\n");
		return PTR_ERR(clk_spll2_fout3);
	}
	/*switch the core clk source*/
	clk_set_parent(clk_mux, clk_spll2_fout3);


	ret = clk_prepare_enable(eswin_sdhci->aclk);
	if (ret) {
		dev_err(dev, "Unable to enable aclk clock.\n");
		return ret;
	}

	ret = clk_prepare_enable(eswin_sdhci->clk_ahb);
	if (ret) {
		dev_err(dev, "Unable to enable AHB clock.\n");
		return ret;
	}

	ret = clk_prepare_enable(eswin_sdhci->clk_xin);
	if (ret) {
		dev_err(dev, "Unable to enable SD clock.\n");
		return ret;
	}

	ret = eswin_sdhci_reset_init(dev, eswin_sdhci);
	if (ret < 0) {
		dev_err(dev, "failed to reset\n");
		return ret;
	}

    return 0;
}

static int eswin_sd_probe(struct udevice *dev)
{
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct eswin_sd_plat *plat = dev_get_plat(dev);
		struct eswin_sdhci_data *eswin_sdhci_sdio = dev_get_priv(dev);
	struct sdhci_host *host = &eswin_sdhci_sdio->host;
	int max_frequency, ret;
	u32 val = 0;

	ret = dev_read_s32(dev, "index", &host->index);
	if (ret) {
		dev_err(dev, "%s %d, failed to get node id, ret %d!\r\n", __func__, __LINE__, ret);
		return ret;
	}

	if (!dev_read_u32(dev, "delay_code", &val)) {
		eswin_sdhci_sdio->phy.delay_code = val;
		debug("delay_code:%d\n", val);
	}

	if (!dev_read_u32(dev, "drive-impedance-ohm", &val))
		eswin_sdhci_sdio->phy.drive_impedance = eswin_convert_drive_impedance_ohm(dev, val);

	if (dev_read_bool(dev, "enable-cmd-pullup"))
		eswin_sdhci_sdio->phy.enable_cmd_pullup = ENABLE;
	else
		eswin_sdhci_sdio->phy.enable_cmd_pullup = DISABLE;

	if (dev_read_bool(dev, "enable-data-pullup"))
		eswin_sdhci_sdio->phy.enable_data_pullup = ENABLE;
	else
		eswin_sdhci_sdio->phy.enable_data_pullup = DISABLE;

	max_frequency = dev_read_u32_default(dev, "max-frequency", 0);

	ret = eswin_sd_clk_reset_init(dev);
	if (ret) {
		dev_err(dev, "%s %d, clk reset fail, ret %d!\r\n", __func__, __LINE__, ret);
		return ret;
	}

	host->quirks = SDHCI_QUIRK_WAIT_SEND_CMD;
	host->voltages = MMC_VDD_32_33;
	host->max_clk = max_frequency;

	host->ioaddr = (void *)dev_read_addr(dev);
	host->mmc = &plat->mmc;
	host->mmc->priv = &eswin_sdhci_sdio->host;
	host->mmc->dev = dev;
	upriv->mmc = host->mmc;

	ret = mmc_of_parse(dev, &plat->cfg);

	if (ret) {
		printf("%s fail to parse\n", __func__);
	}

	sdhci_sd_pre_init(host);
	ret = sdhci_setup_cfg(&plat->cfg, host, 0, SD_MIN_FREQ);
	if (ret) return ret;

	host->ops = &eswin_sd_ops;

	return sdhci_probe(dev);
}

static int eswin_sd_ofdata_to_platdata(struct udevice *dev)
{
#if !CONFIG_IS_ENABLED(OF_PLATDATA)
	struct sdhci_host *host = dev_get_priv(dev);

	host->name = dev->name;
	host->ioaddr = dev_read_addr_ptr(dev);
	host->bus_width = dev_read_u32_default(dev, "bus-width", 4);
#endif

	return 0;
}

static int eswin_sd_bind(struct udevice *dev)
{
	struct eswin_sd_plat *plat = dev_get_plat(dev);

	return sdhci_bind(dev, &plat->mmc, &plat->cfg);
}

static const struct udevice_id eswin_sd_ids[] = {{.compatible = "eswin,sd-sdhci"}, {}};

U_BOOT_DRIVER(eswin_sd_drv) = {
	.name       = "eswin_sd_sdhci",
	.id         = UCLASS_MMC,
	.of_match   = eswin_sd_ids,
	.of_to_plat = eswin_sd_ofdata_to_platdata,
	.ops        = &sdhci_ops,
	.bind       = eswin_sd_bind,
	.probe      = eswin_sd_probe,
	.priv_auto  = sizeof(struct eswin_sdhci_data),
	.plat_auto  = sizeof(struct eswin_sd_plat),
};
