/*
 * drivers/net/phy/realtek.c
 *
 * Driver for Realtek PHYs
 *
 * Author: Johnson Leung <r58129@freescale.com>
 *
 * Copyright (c) 2004 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/phy.h>
#include <linux/module.h>

#define RTL821x_PHYSR		0x11
#define RTL821x_PHYSR_DUPLEX	0x2000
#define RTL821x_PHYSR_SPEED	0xc000
#define RTL821x_INER		0x12
#define RTL821x_INER_INIT	0x6400
#define RTL821x_INSR		0x13
#define RTL8211E_INER_LINK_STATUS 0x400

#define RTL8211F_INER_LINK_STATUS 0x0010
#define RTL8211F_INSR		0x1d
#define RTL8211F_PAGE_SELECT	0x1f
#define RTL8211F_TX_DELAY	0x100

#ifdef CONFIG_ARCH_ADVANTECH
#define RTL8211FS_FIBER_ESR		0x0F
#define RTL8211FS_SERDES_SSR_PAGE	0xdf0
#define RTL8211FS_SERDES_SSR		0x10
#define RTL8211FS_MODE_MASK		0xC000
#define RTL8211FS_FIBER_LINK_MASK	0x1000
#define RTL8211FS_FIBER_1000M_MASK	0x20
#define RTL8211FS_FIBER_100M_MASK	0x10
#define RTL8211F_MODE_COPPER		0
#define RTL8211FS_MODE_FIBER		1
#endif

MODULE_DESCRIPTION("Realtek PHY driver");
MODULE_AUTHOR("Johnson Leung");
MODULE_LICENSE("GPL");

static int rtl821x_ack_interrupt(struct phy_device *phydev)
{
	int err;

	err = phy_read(phydev, RTL821x_INSR);

	return (err < 0) ? err : 0;
}

static int rtl8211f_ack_interrupt(struct phy_device *phydev)
{
	int err;

	phy_write(phydev, RTL8211F_PAGE_SELECT, 0xa43);
	err = phy_read(phydev, RTL8211F_INSR);
	/* restore to default page 0 */
	phy_write(phydev, RTL8211F_PAGE_SELECT, 0x0);

	return (err < 0) ? err : 0;
}

static int rtl8211b_config_intr(struct phy_device *phydev)
{
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		err = phy_write(phydev, RTL821x_INER,
				RTL821x_INER_INIT);
	else
		err = phy_write(phydev, RTL821x_INER, 0);

	return err;
}

static int rtl8211e_config_intr(struct phy_device *phydev)
{
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		err = phy_write(phydev, RTL821x_INER,
				RTL8211E_INER_LINK_STATUS);
	else
		err = phy_write(phydev, RTL821x_INER, 0);

	return err;
}

static int rtl8211f_config_intr(struct phy_device *phydev)
{
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		err = phy_write(phydev, RTL821x_INER,
				RTL8211F_INER_LINK_STATUS);
	else
		err = phy_write(phydev, RTL821x_INER, 0);

	return err;
}

static int rtl8211f_config_init(struct phy_device *phydev)
{
	int ret;
	u16 reg;

	ret = genphy_config_init(phydev);
	if (ret < 0)
		return ret;

	if (phydev->interface == PHY_INTERFACE_MODE_RGMII) {
		/* enable TXDLY */
		phy_write(phydev, RTL8211F_PAGE_SELECT, 0xd08);
		reg = phy_read(phydev, 0x11);
		reg |= RTL8211F_TX_DELAY;
		phy_write(phydev, 0x11, reg);
		/* restore to default page 0 */
		phy_write(phydev, RTL8211F_PAGE_SELECT, 0x0);
	}

	return 0;
}

#ifdef CONFIG_ARCH_ADVANTECH
static int rtl8211f_mode(struct phy_device *phydev)
{
    u16 val;
    val = phy_read(phydev, RTL8211FS_FIBER_ESR);
    val &= RTL8211FS_MODE_MASK;

    if(val)
        return RTL8211FS_MODE_FIBER;
    else
        return RTL8211F_MODE_COPPER;
}

static int rtl8211f_read_status(struct phy_device *phydev)
{
	int err;
	int fiber_state;

	if(rtl8211f_mode(phydev) == RTL8211FS_MODE_FIBER) {
		err = phy_write(phydev, RTL8211F_PAGE_SELECT, RTL8211FS_SERDES_SSR_PAGE);
		if (err)
			return err;

		fiber_state = phy_read(phydev, RTL8211FS_SERDES_SSR);
		err = phy_write(phydev, RTL8211F_PAGE_SELECT, 0);
		if (err)
			return err;

		phydev->pause = 0;
		phydev->asym_pause = 0;
		phydev->speed = SPEED_10;
		phydev->duplex = DUPLEX_HALF;
		phydev->lp_advertising = ADVERTISED_1000baseT_Full | ADVERTISED_100baseT_Full | ADVERTISED_Autoneg;

		if (fiber_state & RTL8211FS_FIBER_LINK_MASK) {
			phydev->link = 1;
			if (fiber_state & RTL8211FS_FIBER_1000M_MASK) {
				phydev->speed = SPEED_1000;
				phydev->duplex = DUPLEX_FULL;
			} else if(fiber_state & RTL8211FS_FIBER_100M_MASK) {
				phydev->speed = SPEED_100;
				phydev->duplex = DUPLEX_FULL;
			}
		} else
			phydev->link = 0;

		return 0;
	} else
		return genphy_read_status(phydev);
}
#endif

static struct phy_driver realtek_drvs[] = {
	{
		.phy_id         = 0x00008201,
		.name           = "RTL8201CP Ethernet",
		.phy_id_mask    = 0x0000ffff,
		.features       = PHY_BASIC_FEATURES,
		.flags          = PHY_HAS_INTERRUPT,
		.config_aneg    = &genphy_config_aneg,
		.read_status    = &genphy_read_status,
		.driver         = { .owner = THIS_MODULE,},
	}, {
		.phy_id		= 0x001cc912,
		.name		= "RTL8211B Gigabit Ethernet",
		.phy_id_mask	= 0x001fffff,
		.features	= PHY_GBIT_FEATURES,
		.flags		= PHY_HAS_INTERRUPT,
		.config_aneg	= &genphy_config_aneg,
		.read_status	= &genphy_read_status,
		.ack_interrupt	= &rtl821x_ack_interrupt,
		.config_intr	= &rtl8211b_config_intr,
		.driver		= { .owner = THIS_MODULE,},
	}, {
		.phy_id		= 0x001cc914,
		.name		= "RTL8211DN Gigabit Ethernet",
		.phy_id_mask	= 0x001fffff,
		.features	= PHY_GBIT_FEATURES,
		.flags		= PHY_HAS_INTERRUPT,
		.config_aneg	= genphy_config_aneg,
		.read_status	= genphy_read_status,
		.ack_interrupt	= rtl821x_ack_interrupt,
		.config_intr	= rtl8211e_config_intr,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
		.driver		= { .owner = THIS_MODULE,},
	}, {
		.phy_id		= 0x001cc915,
		.name		= "RTL8211E Gigabit Ethernet",
		.phy_id_mask	= 0x001fffff,
		.features	= PHY_GBIT_FEATURES,
		.flags		= PHY_HAS_INTERRUPT,
		.config_aneg	= &genphy_config_aneg,
		.read_status	= &genphy_read_status,
		.ack_interrupt	= &rtl821x_ack_interrupt,
		.config_intr	= &rtl8211e_config_intr,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
		.driver		= { .owner = THIS_MODULE,},
	}, {
		.phy_id		= 0x001cc916,
		.name		= "RTL8211F Gigabit Ethernet",
		.phy_id_mask	= 0x001fffff,
		.features       = PHY_GBIT_FEATURES,
		.flags          = PHY_HAS_INTERRUPT,
		.config_aneg    = &genphy_config_aneg,
#ifndef CONFIG_ARCH_ADVANTECH
		.config_init    = &rtl8211f_config_init,
		.read_status    = &genphy_read_status,
#else
		.config_init	= genphy_config_init,
		.read_status	= rtl8211f_read_status,
#endif
		.ack_interrupt  = &rtl8211f_ack_interrupt,
		.config_intr    = &rtl8211f_config_intr,
#if 0
		.features	= PHY_GBIT_FEATURES | SUPPORTED_MII |
			      SUPPORTED_AUI | SUPPORTED_FIBRE |
			      SUPPORTED_BNC,
		.config_aneg	= genphy_config_aneg,
		.config_init	= genphy_config_init,
		.read_status	= rtl8211f_read_status,
		.aneg_done	= genphy_aneg_done,
		.soft_reset	= genphy_no_soft_reset,
#endif
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
		.driver		= { .owner = THIS_MODULE },
	},
};

module_phy_driver(realtek_drvs);

static struct mdio_device_id __maybe_unused realtek_tbl[] = {
	{ 0x001cc912, 0x001fffff },
	{ 0x001cc914, 0x001fffff },
	{ 0x001cc915, 0x001fffff },
	{ 0x001cc916, 0x001fffff },
	{ }
};

MODULE_DEVICE_TABLE(mdio, realtek_tbl);
