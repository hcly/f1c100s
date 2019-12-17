/*
 * Copyright (C) 2012-2014 Daniel Schwierzeck, daniel.schwierzeck@gmail.com
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <malloc.h>
#include <linux/errno.h>
#include <linux/mtd/mtd.h>
#include <spi_flash.h>

static struct mtd_info sn_mtd_info;
static char sn_mtd_name[8];

static int sn_mtd_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct spi_nand_chip *chip = mtd->priv;
	int err;

	instr->state = MTD_ERASING;

	err = spi_flash_erase(chip, instr->addr, instr->len);
	if (err) {
		instr->state = MTD_ERASE_FAILED;
		instr->fail_addr = MTD_FAIL_ADDR_UNKNOWN;
		return -EIO;
	}

	instr->state = MTD_ERASE_DONE;
	mtd_erase_callback(instr);

	return 0;
}

static int sn_mtd_read(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, u_char *buf)
{
	struct spi_nand_chip *chip = mtd->priv;
	int err;

	err = spi_flash_read(chip, from, len, buf);
	if (!err)
		*retlen = len;

	return err;
}

static int sn_mtd_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u_char *buf)
{
	struct spi_nand_chip *chip = mtd->priv;
	int err;

	err = spi_flash_write(chip, to, len, buf);
	if (!err)
		*retlen = len;

	return err;
}

static int sn_mtd_read_oob(struct mtd_info *mtd, loff_t from, struct mtd_oob_ops *ops)
{
	struct spi_nand_chip *chip = mtd->priv;

	return spi_flash_readoob(chip, from, ops);
}

static int sn_mtd_write_oob(struct mtd_info *mtd, loff_t to, struct mtd_oob_ops *ops)
{
	struct spi_nand_chip *chip = mtd->priv;

	return spi_flash_writeoob(chip, to, ops);
}

static int sn_mtd_block_isbad(struct mtd_info *mtd, loff_t ofs)
{
	struct spi_nand_chip *chip = mtd->priv;

	return spi_flash_isbad(chip, ofs);
}

static int sn_mtd_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct spi_nand_chip *chip = mtd->priv;

	return spi_flash_markbad(chip, ofs);
}

static void sn_mtd_sync(struct mtd_info *mtd)
{
}

static int sn_mtd_number(void)
{
#ifdef CONFIG_SYS_MAX_FLASH_BANKS
	return CONFIG_SYS_MAX_FLASH_BANKS;
#else
	return 0;
#endif
}

int sn_mtd_register(struct spi_nand_chip *chip)
{
	memset(&sn_mtd_info, 0, sizeof(sn_mtd_info));
	sprintf(sn_mtd_name, "nor%d", sn_mtd_number());

	sn_mtd_info.name = sn_mtd_name;
	sn_mtd_info.type = MTD_NANDFLASH;
	sn_mtd_info.flags = MTD_CAP_NANDFLASH;
	sn_mtd_info.erasesize = chip->block_size;
	sn_mtd_info.writesize = chip->page_size;
	sn_mtd_info.writebufsize = sn_mtd_info.writesize;

	sn_mtd_info.ecclayout = chip->ecclayout;
	sn_mtd_info.oobsize = chip->oob_size;
	sn_mtd_info.oobavail = chip->ecclayout->oobavail;
	// sn_mtd_info._point = NULL;
	// sn_mtd_info._unpoint = NULL;
	sn_mtd_info._lock = NULL;
	sn_mtd_info._unlock = NULL;

	
	// sn_mtd_info._resume = NULL;
	// sn_mtd_info._suspend = NULL;
	sn_mtd_info._read_oob = sn_mtd_read_oob;
	sn_mtd_info._write_oob = sn_mtd_write_oob;
	sn_mtd_info._erase = sn_mtd_erase;
	sn_mtd_info._read = sn_mtd_read;
	sn_mtd_info._write = sn_mtd_write;
	sn_mtd_info._sync = sn_mtd_sync;
	sn_mtd_info._block_isbad = sn_mtd_block_isbad;
	sn_mtd_info._block_markbad = sn_mtd_block_markbad;
	sn_mtd_info._block_isreserved = NULL; // spi_nand_block_isreserved;

	sn_mtd_info.size = chip->size;
	sn_mtd_info.priv = chip;

	/* Only uniform flash devices for now */
	sn_mtd_info.numeraseregions = 0;

	return add_mtd_device(&sn_mtd_info);
}

void sn_mtd_unregister(void)
{
	del_mtd_device(&sn_mtd_info);
}
