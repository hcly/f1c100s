/*
 * Configuration settings for new Allwinner F-series (suniv) CPU
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_SUNXI_USB_PHYS 1

/*
 * Include common sunxi configuration where most the settings are
 */
/* 
	spl + uboot 512k 
	kernel 2.375m
	device tree 64k
*/
#ifdef CONFIG_SPI_NAND
#define CONFIG_BOOTCOMMAND "spi_nand probe 0 40000000; spi_nand read 80500000 80000 260000; sf read 80c00000 2e0000 10000; bootm 80500000 - 80c00000"
#else
#define CONFIG_BOOTCOMMAND "sf probe 0 40000000; sf read 80500000 80000 260000; sf read 80c00000 2e0000 10000; bootm 80500000 - 80c00000"
#endif
#include <configs/sunxi-common.h>
#ifdef BOOTENV
#undef BOOTENV
#endif
#define BOOTENV \
"bootargs=earlyprintk panic=5 rootwait root=31:03 rw rootfstype=jffs2 init=/linuxrc console=ttyS0,115200\0" \
"mtdpartsx=spi0.0:512k(uboot)ro,2432k(kernel)ro,64k(script),-(rootfs)\0" \
"bootmmc=fatload mmc 0 80500000 zImage; fatload mmc 0 80c00000 sunivw1p1-sn.dtb; bootz 80500000 - 80c00000"

#endif /* __CONFIG_H */
