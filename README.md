# f1c100s 128MB W25N01G spi nand调试记录
## 镜像使用说明
镜像位置images文件夹
uboot-with-spl-usb.bin　FEL模式下使用的uboot镜像
uboot-with-spl-spinand.bin	spi nand启动使用的uboot镜像
FEL模式下使用标准sunxi-fel执行如下命令
```java
sunxi-fel uboot uboot-with-spl-usb.bin write 0x80000000 uboot-with-spl-spinand.bin
```
由于sunxi-fel直接写spi nand代码没有调试过，所以直接将uboot镜像传到内存后写入spi nand
执行完成，等待u-boot倒计时完后会自动将uboot-with-spl-spinand.bin写入spi nand中
FEL模式日志
```java
U-Boot SPL 2018.01-g5a65529-dirty (Dec 17 2019 - 15:19:40)
DRAM: 64 MiB
Trying to boot from FEL


U-Boot 2018.01-g5a65529-dirty (Dec 17 2019 - 15:19:40 +0800) Allwinner Technology

CPU:   Allwinner F Series (SUNIV)
Model: Lichee Pi Nano
DRAM:  64 MiB
MMC:   SUNXI SD/MMC: 0
SPI-NAND: W25N01GV is found size: 128MB.
read status2:0x18
internal ecc is on now turn off
write status2:0x8
read status2:0x8
*** Warning - bad CRC, using default environment

In:    serial@1c25000
Out:   serial@1c25000
Err:   serial@1c25000
Net:   No ethernet found.
starting USB...
No controllers found
Hit any key to stop autoboot:  0 
SPI-NAND: W25N01GV is found size: 128MB.
read status2:0x18
internal ecc is on now turn off
write status2:0x8
read status2:0x8
SPI-NAND: 1310720 bytes @ 0x0 Updated: OK
SPI-NAND: W25N01GV is found size: 128MB.
read status2:0x18
internal ecc is on now turn off
write status2:0x8
read status2:0x8
SPI-NAND: 2490368 bytes @ 0x80000 Read: OK
Unknown command 'sf' - try 'help'
Wrong Image Format for bootm command
ERROR: can't get kernel image!
=> 
```
因为没有内核所以最后会报错，不用管,烧录完成后spi nand启动日志如下
```java
U-Boot SPL 2018.01-g5a65529-dirty (Dec 17 2019 - 15:20:01)
DRAM: 64 MiB
Trying to boot from sunxi SPI


U-Boot 2018.01-g5a65529-dirty (Dec 17 2019 - 15:20:01 +0800) Allwinner Technology

CPU:   Allwinner F Series (SUNIV)
Model: Lichee Pi Nano
DRAM:  64 MiB
MMC:   SUNXI SD/MMC: 0
SPI-NAND: W25N01GV is found size: 128MB.
read status2:0x18
internal ecc is on now turn off
write status2:0x8
read status2:0x8
*** Warning - bad CRC, using default environment

In:    serial@1c25000
Out:   serial@1c25000
Err:   serial@1c25000
Net:   No ethernet found.
starting USB...
No controllers found
Hit any key to stop autoboot:  0 
=> 
```
## 源码编译
源码编译需要编译两次，分别生成FEL镜像和SPI NAND镜像
首先修改uboot源码下build.sh中交叉编译器
```java
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabi- licheepi_nano_spinand_defconfig
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabi- -j8
```
我的交叉编译工具已经加入到了系统环境变量
### 生成FEL镜像
修改uboot源码include/configs/sunxi-common.h
打开#define GZYS_USBBURN	1定义
```java
#ifndef _SUNXI_COMMON_CONFIG_H
#define _SUNXI_COMMON_CONFIG_H

#include <asm/arch/cpu.h>
#include <linux/stringify.h>
#define GZYS_USBBURN	1
#ifdef CONFIG_OLD_SUNXI_KERNEL_COMPAT
```
保存后在uboot源码根目录下执行./build.sh进行编译，最后输出信息如下
```java
  CC      spl/drivers/mtd/spi/sunxi_spi_nand_spl.o
  CC      spl/drivers/serial/serial.o
  CC      spl/drivers/serial/serial_ns16550.o
  CC      spl/drivers/serial/ns16550.o
  LD      spl/disk/built-in.o
  LD      spl/lib/built-in.o
  LD      spl/drivers/serial/built-in.o
  LD      spl/drivers/mmc/built-in.o
  LD      spl/drivers/built-in.o
  LD      spl/u-boot-spl
  OBJCOPY spl/u-boot-spl-nodtb.bin
  COPY    spl/u-boot-spl.bin
  MKSUNXI spl/sunxi-spl.bin
  BINMAN  u-boot-sunxi-with-spl.bin
  CFGCHK  u-boot.cfg
please delete GZYS_USBBURN in include/configs/sunxi-common.h file and rebuild to create spi nand image
DONE
```
此时uboot源码根目录下会生成uboot-with-spl-usb.bin文件
### 生成SPI NAND镜像
修改uboot源码include/configs/sunxi-common.h
注释掉#define GZYS_USBBURN	1定义
```java
#ifndef _SUNXI_COMMON_CONFIG_H
#define _SUNXI_COMMON_CONFIG_H

#include <asm/arch/cpu.h>
#include <linux/stringify.h>
#define GZYS_USBBURN	1
#ifdef CONFIG_OLD_SUNXI_KERNEL_COMPAT
```
保存后在uboot源码根目录下执行./build.sh进行编译，最后输出信息如下
```java
Copying block 23 to 46
1+0 records in
1+0 records out
1024 bytes (1.0 kB, 1.0 KiB) copied, 0.000163137 s, 6.3 MB/s
Copying block 24 to 48
1+0 records in
1+0 records out
1024 bytes (1.0 kB, 1.0 KiB) copied, 0.000174752 s, 5.9 MB/s
Copying block 25 to 50
1+0 records in
1+0 records out
1024 bytes (1.0 kB, 1.0 KiB) copied, 0.000165035 s, 6.2 MB/s
Appending u-boot
984+0 records in
984+0 records out
1007616 bytes (1.0 MB, 984 KiB) copied, 0.00353802 s, 285 MB/s
done
burn uboot-with-spl-spinand.bin to spi nand
cmd is sunxi-fel uboot uboot-with-spl-usb.bin write 0x80000000 uboot-with-spl-spinand.bin
DONE
```
此时uboot源码根目录下会生成uboot-with-spl-spinand.bin文件
## 调试中出现的问题及修改
### uboot源码增加对w25n01g支持
```java
diff --git a/drivers/mtd/spi-nand/spi-nand-base.c b/drivers/mtd/spi-nand/spi-nand-base.c
index 442a595..bee8a93 100755
--- a/drivers/mtd/spi-nand/spi-nand-base.c
+++ b/drivers/mtd/spi-nand/spi-nand-base.c
@@ -25,6 +25,8 @@ extern int spi_nand_issue_cmd(struct spi_nand_chip *chip, struct spi_nand_cmd *c
 int spi_nand_erase(struct spi_nand_chip *chip, uint64_t addr, uint64_t len);
 
 static struct spi_nand_flash spi_nand_table[] = {
+	SPI_NAND_INFO("W25N01GV", 0xEF, 0xAA, 2048, 64, 64, 1024,
+			1, 1, 0 | SPINAND_NEED_PLANE_SELECT),
 	SPI_NAND_INFO("GD5F1GQ4UAYIG", 0xC8, 0xF1, 2048, 64, 64, 1024,
 			1, 1, 0 | SPINAND_USED_GIGADEVICE),
 	SPI_NAND_INFO("GD5F1GQ4UBYIG", 0xC8, 0xD1, 2048, 64, 64, 1024,
@@ -1844,6 +1846,22 @@ static void spi_nand_set_rd_wr_op(struct spi_nand_chip *chip)
 		chip->write_cache_rdm_op = SPINAND_CMD_PROG_LOAD_RDM_DATA;
 	}
 }
+//add by leijie for w25n01g turn off internal ecc
+static void w25n01_turn_off_iecc(struct spi_nand_chip *chip)
+{
+	u8 buf[1];
+	spi_nand_read_reg(chip,0xb0,buf);
+	printf("read status2:0x%x\n",buf[0]);
+	if(buf[0] & 0x10) {//internal ecc
+		printf("internal ecc is on now turn off\n");
+		buf[0] = buf[0] & ~(1 << 4);
+		printf("write status2:0x%x\n",buf[0]);
+		spi_nand_write_reg(chip,0xb0,buf);
+	}
+	spi_nand_read_reg(chip,0xb0,buf);
+	printf("read status2:0x%x\n",buf[0]);
+	return;
+}
 
 /**
  * spi_nand_init - [Interface] Init SPI-NAND device driver
@@ -1879,8 +1897,9 @@ static int spi_nand_init(struct spi_slave *spi, struct spi_nand_chip **chip_ptr)
 	return -ENODEV;
 
 ident_done:
-	spi_nand_info("SPI-NAND: %s is found.\n", chip->name);
-
+	spi_nand_info("SPI-NAND: %s is found size: %dMB.\n", chip->name,chip->size/1024/1024);
+	if(id[0] == 0xef)
+		w25n01_turn_off_iecc(chip);
 	// giga-device need re-config rd/wr options
 	if( chip->options & SPINAND_USED_GIGADEVICE )
 		spi_nand_set_rd_wr_op(chip);
```
主要增加了wn25n01内部ecc判断

20191226后面发现spi nand驱动中有对内部ecc进行判断 所以需要把增加的w25n01_turn_off_iecc函数删除,这里源码就不进行更新了
### spi nand镜像脚本制作
参照
[https://whycan.cn/t_1594.html](https://whycan.cn/t_1594.html)
中的脚本
[gen_sunxi_spinand_onlyboot_img.sh](https://github.com/bhorn/openwrt/blob/dolphinpi-spinand/target/linux/sunxi/image/gen_sunxi_spinand_onlyboot_img.sh)
制作了f1c100_uboot_spinand.sh
```
#!/bin/bash

set -e
[ $# -eq 2 ] || {
    echo "SYNTAX: $0 <outputfile> <u-boot image>"
    echo "Given: $@"
    exit 1
}

OUTPUT="$1"
UBOOT="$2"
PAGESIZE=2048
BLOCKSIZE=128

TOOLCHECK=$(od --help | grep 'endia')
if [ "$TOOLCHECK" == "" ]; then
	echo "od cmd is too old not support endia"
	exit -1
fi
# SPL-Size is an uint32 at 16 bytes offset contained in the SPL header
#uboot header offset head in include/configs/sunxi-common.h CONFIG_SYS_SPI_U_BOOT_OFFS so spl max size is CONFIG_SYS_SPI_U_BOOT_OFFS
#f1c100s modify CONFIG_SYS_SPI_U_BOOT_OFFS to 0xd000(52K)
SPLSIZE=$(od -An -t u4 -j16 -N4 "$UBOOT" | xargs)
printf "SPLSIZE:%d(0x%x)\n" $SPLSIZE $SPLSIZE
# The u-boot size is an uint32 at (0xd000 + 12) bytes offset uboot start offset 0xd000(52K)
UBOOTSIZE=$(od --endian=big -An -t u4 -j$((53248 + 12)) -N4 "$UBOOT" | xargs)
printf "UBOOTSIZE:%d(0x%x)\n" $UBOOTSIZE $UBOOTSIZE
ALIGNCHECK=$(($PAGESIZE%1024))
if [ "$ALIGNCHECK" -ne "0" ]; then
	echo "Page-size is not 1k alignable and thus not supported by EGON"
	exit -1
fi

KPAGESIZE=$(($PAGESIZE/1024))
SPLBLOCKS=25

echo "Generating boot0 for boot part of max size 0x8000 SPLBLOCKS:$SPLBLOCKS"
dd if="/dev/zero" of="$OUTPUT" bs=1024 count=$((52 - $SPLBLOCKS))

for splcopy in `seq 0 $SPLBLOCKS`;
do
	to=$(($splcopy*$KPAGESIZE))
	echo "Copying block $splcopy to $to" 
	dd if="$UBOOT" of="$OUTPUT" bs=1024 count=1 seek=$to skip=$splcopy conv=notrunc
done

echo "Appending u-boot"
dd if="$UBOOT" of="$OUTPUT" bs=1024 seek=52 skip=52 conv=notrunc
sync
echo "done"
```
其中将页大小和块大小值固定,只传输出文件和输入文件名

由于f1c100s生成的spl镜像sunxi-spl.bin大小为固定值24576个字节，所以简化了脚本

内部brom spi nand启动必需1KB对齐所以将spl镜像放大两倍后小于52KB,暂使用52K这个值

也就是说spl镜像最大为52KB,同时uboot镜像的起始地址为0xd000(52KB)

同时需要修改uboot源码include/configs/sunxi-common.h中的CONFIG_SYS_SPI_U_BOOT_OFFS为0xd000

和CONFIG_SPL_PAD_TO为CONFIG_SYS_SPI_U_BOOT_OFFS

这里有个地方要注意下，如果在FEL模式下运行uboot镜像，需要将CONFIG_SYS_SPI_U_BOOT_OFFS改回0x8000,不然sunxi-fel会报错
### 启动报错
```java
U-Boot SPL 2018.01-g5a65529-dirty (Dec 17 2019 - 15:20:01)
DRAM: 64 MiB
Trying to boot from sunxi SPI
```
从spi nand启动卡在上面这个地方，通过加调试信息找到了一个问题
支持spi nand的uboot源码drivers/mtd/spi/sunxi_spi_nand_spl.c
在函数
static u32 sunxi_spi0_read_data中addr默认加了0x8000
需要把这个注释掉
```java
index f70e1ac..0e2a60d 100644
--- a/drivers/mtd/spi/sunxi_spi_nand_spl.c
+++ b/drivers/mtd/spi/sunxi_spi_nand_spl.c
@@ -266,7 +266,7 @@ static u32 sunxi_spi0_read_data(u8 *buf, u32 addr, u32 bufsize,
 #ifdef CONFIG_SPINAND
 {
 	u32 ret = bufsize;
-	addr += 0x8000;
+	//addr += 0x8000; //modify by leijie
 	int page = addr >> 11;
 	if( pages != page ) { // need read to cache
 		writel(4, spi_bc_reg); /* Burst counter (total bytes) */
```







