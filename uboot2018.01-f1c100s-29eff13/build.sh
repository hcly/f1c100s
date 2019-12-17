#!/bin/bash
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabi- licheepi_nano_spinand_defconfig
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabi- -j8
if [ -f u-boot-sunxi-with-spl.bin ]; then
	UBOOOFFSET=$(cat u-boot.cfg | grep CONFIG_SYS_SPI_U_BOOT_OFFS | awk '{print $3}' | sed -n '1,1p')
	if [ "$UBOOOFFSET" == "0x8000" ]; then
		cp u-boot-sunxi-with-spl.bin uboot-with-spl-usb.bin
		echo "please delete GZYS_USBBURN in include/configs/sunxi-common.h file and rebuild to create spi nand image"
	else
		./f1c100_uboot_spinand.sh uboot-with-spl-spinand.bin u-boot-sunxi-with-spl.bin
		echo "burn uboot-with-spl-spinand.bin to spi nand"
		echo "cmd is sunxi-fel uboot uboot-with-spl-usb.bin write 0x80000000 uboot-with-spl-spinand.bin"
	fi
else
	echo "ERROR"
fi
echo "DONE"





