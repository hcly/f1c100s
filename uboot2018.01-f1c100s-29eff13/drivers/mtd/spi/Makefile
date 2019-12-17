#
# (C) Copyright 2006
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# SPDX-License-Identifier:	GPL-2.0+
#
ifdef CONFIG_SPI_NAND
obj-$(CONFIG_DM_SPI_FLASH) += sn-uclass.o
else
obj-$(CONFIG_DM_SPI_FLASH) += sf-uclass.o
endif

ifdef CONFIG_SPL_BUILD
obj-$(CONFIG_SPL_SPI_BOOT)	+= fsl_espi_spl.o
obj-$(CONFIG_SPL_SPI_SUNXI)	+= sunxi_spi_spl.o
endif

obj-$(CONFIG_SPI_NAND) += sn_probe.o
obj-$(CONFIG_SPI_FLASH) += sf_probe.o spi_flash.o spi_flash_ids.o sf.o
obj-$(CONFIG_SPI_FLASH_DATAFLASH) += sf_dataflash.o
ifdef CONFIG_SPI_NAND
obj-$(CONFIG_SPI_FLASH_MTD) += sn_mtd.o
else
obj-$(CONFIG_SPI_FLASH_MTD) += sf_mtd.o
endif
obj-$(CONFIG_SPI_FLASH_SANDBOX) += sandbox.o
