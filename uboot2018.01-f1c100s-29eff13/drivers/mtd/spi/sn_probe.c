/*
 * SPI flash probing
 *
 * Copyright (C) 2008 Atmel Corporation
 * Copyright (C) 2010 Reinhard Meyer, EMK Elektronik
 * Copyright (C) 2013 Jagannadha Sutradharudu Teki, Xilinx Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <malloc.h>
#include <spi.h>
#ifdef CONFIG_SPI_FLASH_MTD
#include "mtd.h"
#endif
#include <spi_flash.h>
#include <spi-nand.h>

#include "sf_internal.h"

/**
 * spi_flash_probe_slave() - Probe for a SPI flash device on a bus
 *
 * @flashp: Pointer to place to put flash info, which may be NULL if the
 * space should be allocated
 */
/*--------------------------------
 * for std probe slave API
 *--------------------------------
*/
static int spi_flash_probe_slave(struct spi_nand_chip **chip)
{
	int ret;
	struct spi_slave *spi = chip[0]->spi;

	ret = spi_nand_probe_slave(spi, chip);
	if (ret)
		goto err_read_id;

#ifdef CONFIG_SPI_FLASH_MTD
	// fixed me
	ret = sn_mtd_register(*chip);
#endif
	
	return ret; // don't release bus 
err_read_id:
	spi_release_bus(spi);
	return ret;
}

#ifndef CONFIG_DM_SPI_FLASH
/* probe tail APIs */
static struct spi_nand_chip *spi_flash_probe_tail(struct spi_slave *bus)
{
	struct spi_nand_chip *chip;

	/* Allocate space if needed (not used by sf-uclass */
	chip = calloc(1, sizeof(*chip));
	if (!chip) {
		debug("SF: Failed to allocate spi_flash\n");
		return NULL;
	}
	chip->spi = bus;

	// if (spi_nand_probe_slave(bus, &chip)) {
	if (spi_flash_probe_slave(&chip)) {
		spi_free_slave(bus);
		if(chip != NULL)
			free(chip);
		return NULL;
	}

	return chip;
}

struct spi_nand_chip *spi_flash_probe(unsigned int busnum, unsigned int cs,
		unsigned int max_hz, unsigned int spi_mode)
{
	struct spi_slave *bus;

	bus = spi_setup_slave(busnum, cs, max_hz, spi_mode);
	if (!bus)
		return NULL;
	return spi_flash_probe_tail(bus);
}

#ifdef CONFIG_OF_SPI_FLASH
struct spi_nand_chip *spi_flash_probe_fdt(const void *blob, int slave_node,
				      int spi_node)
{
	struct spi_slave *bus;

	bus = spi_setup_slave_fdt(blob, slave_node, spi_node);
	if (!bus)
		return NULL;
	return spi_flash_probe_tail(bus);
}
#endif

/*--------------------------------
 * for std free API
 *--------------------------------*/
void spi_flash_free(struct spi_nand_chip *chip)
{
#ifdef CONFIG_SPI_FLASH_MTD
	// fixed me
#error fixed me
	sn_mtd_unregister();
#endif
	spi_free_slave(chip->spi);
	spi_nand_release(chip);
	// free(chip);
}

#else /* defined CONFIG_DM_SPI_FLASH */


/*--------------------------------
 * for std read API
 *--------------------------------*/
static int spi_flash_std_read(struct udevice *dev, u32 offset, size_t len,
			      void *buf)
{
	struct spi_nand_chip *chip = dev_get_uclass_priv(dev);

	return spi_nand_cmd_read_ops(chip, offset, len, buf);
}


/*--------------------------------
 * for std write API
 *--------------------------------*/
static int spi_flash_std_write(struct udevice *dev, u32 offset, size_t len,
			const void *buf)
{
	struct spi_nand_chip *chip = dev_get_uclass_priv(dev);

#if 0 // defined(CONFIG_SPI_FLASH_SST)
	if (flash->flags & SNOR_F_SST_WR) {
		if (flash->spi->mode & SPI_TX_BYTE)
			return sst_write_bp(flash, offset, len, buf);
		else
			return sst_write_wp(flash, offset, len, buf);
	}
#endif

	return spi_nand_cmd_write_ops(chip, offset, len, buf);
}

/*--------------------------------
 * for std erase API
 *--------------------------------*/
extern int spi_nand_erase(struct spi_nand_chip *chip, uint64_t addr, uint64_t len);
static int spi_flash_std_erase(struct udevice *dev, u32 offset, size_t len)
{
	struct spi_nand_chip *chip = dev_get_uclass_priv(dev);

	return spi_nand_erase(chip, offset, len);
}
/* std probe APIs */
static int spi_flash_std_probe(struct udevice *dev)
{
	struct spi_slave *slave = dev_get_parent_priv(dev);
	struct dm_spi_slave_platdata *plat = dev_get_parent_platdata(dev);
	struct spi_nand_chip *chip;

	chip = dev_get_uclass_priv(dev);
	chip->dev = dev;
	chip->spi = slave;
	debug("%s: slave=%p, cs=%d\n", __func__, slave, plat->cs);
	// return spi_nand_probe_slave(slave, &chip);
	return spi_flash_probe_slave(&chip);
}

/* std read oob APIs */
static int spi_flash_std_readoob(struct udevice *dev, loff_t from, struct mtd_oob_ops *ops) 
{
	struct spi_nand_chip *chip;
	chip = dev_get_uclass_priv(dev);
	
	return spi_nand_read_oob(chip, from, ops);
}

/* std write oob APIs */
static int spi_flash_std_writeoob(struct udevice *dev, loff_t to, struct mtd_oob_ops *ops) 
{
	struct spi_nand_chip *chip;
	chip = dev_get_uclass_priv(dev);
	
	return spi_nand_write_oob(chip, to, ops);
}

/* std check block isbad APIs */
static int spi_flash_std_block_isbad(struct udevice *dev, loff_t ofs) 
{
	struct spi_nand_chip *chip;
	chip = dev_get_uclass_priv(dev);
	
	return spi_nand_block_isbad(chip, ofs);
}

/* std block mark bad APIs */
static int spi_flash_std_block_markbad(struct udevice *dev, loff_t ofs) 
{
	struct spi_nand_chip *chip;
	chip = dev_get_uclass_priv(dev);
	
	return spi_nand_block_markbad(chip, ofs);
}

static const struct dm_spi_flash_ops spi_flash_std_ops = {
	.read = spi_flash_std_read,
	.write = spi_flash_std_write,
	.erase = spi_flash_std_erase,
	.readoob = spi_flash_std_readoob,
	.writeoob = spi_flash_std_writeoob,
	.isbad = spi_flash_std_block_isbad,
	.markbad = spi_flash_std_block_markbad,
	.isreserved = NULL,
};

static const struct udevice_id spi_flash_std_ids[] = {
	{ .compatible = "spi-flash" },
	{ }
};

U_BOOT_DRIVER(spi_flash_std) = {
	.name		= "spi_flash_std",
	.id		= UCLASS_SPI_FLASH,
	.of_match	= spi_flash_std_ids,
	.probe		= spi_flash_std_probe,
	.priv_auto_alloc_size = sizeof(struct spi_nand_chip),
	.ops		= &spi_flash_std_ops,
};

#endif /* CONFIG_DM_SPI_FLASH */
