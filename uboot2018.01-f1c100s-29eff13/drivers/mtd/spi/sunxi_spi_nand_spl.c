/*
 * Copyright (C) 2016 Siarhei Siamashka <siarhei.siamashka@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <spl.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <libfdt.h>

#ifdef CONFIG_SPL_OS_BOOT
#error CONFIG_SPL_OS_BOOT is not supported yet
#endif

/*
 * This is a very simple U-Boot image loading implementation, trying to
 * replicate what the boot ROM is doing when loading the SPL. Because we
 * know the exact pins where the SPI Flash is connected and also know
 * that the Read Data Bytes (03h) command is supported, the hardware
 * configuration is very simple and we don't need the extra flexibility
 * of the SPI framework. Moreover, we rely on the default settings of
 * the SPI controler hardware registers and only adjust what needs to
 * be changed. This is good for the code size and this implementation
 * adds less than 400 bytes to the SPL.
 *
 * There are two variants of the SPI controller in Allwinner SoCs:
 * A10/A13/A20 (sun4i variant) and everything else (sun6i variant).
 * Both of them are supported.
 *
 * The pin mixing part is SoC specific and only A10/A13/A20/H3/A64 are
 * supported at the moment.
 */

static u32 spi0_base;

/*****************************************************************************/
/* SUN4I variant of the SPI controller                                       */
/*****************************************************************************/

#define SUN4I_SPI0_CCTL             (spi0_base + 0x1C)
#define SUN4I_SPI0_CTL              (spi0_base + 0x08)
#define SUN4I_SPI0_RX               (spi0_base + 0x00)
#define SUN4I_SPI0_TX               (spi0_base + 0x04)
#define SUN4I_SPI0_FIFO_STA         (spi0_base + 0x28)
#define SUN4I_SPI0_BC               (spi0_base + 0x20)
#define SUN4I_SPI0_TC               (spi0_base + 0x24)

#define SUN4I_CTL_ENABLE            BIT(0)
#define SUN4I_CTL_MASTER            BIT(1)
#define SUN4I_CTL_TF_RST            BIT(8)
#define SUN4I_CTL_RF_RST            BIT(9)
#define SUN4I_CTL_XCH               BIT(10)

/*****************************************************************************/
/* SUN6I variant of the SPI controller                                       */
/*****************************************************************************/

#define SUN6I_SPI0_CCTL             (spi0_base + 0x24)
#define SUN6I_SPI0_GCR              (spi0_base + 0x04)
#define SUN6I_SPI0_TCR              (spi0_base + 0x08)
#define SUN6I_SPI0_FIFO_STA         (spi0_base + 0x1C)
#define SUN6I_SPI0_MBC              (spi0_base + 0x30)
#define SUN6I_SPI0_MTC              (spi0_base + 0x34)
#define SUN6I_SPI0_BCC              (spi0_base + 0x38)
#define SUN6I_SPI0_TXD              (spi0_base + 0x200)
#define SUN6I_SPI0_RXD              (spi0_base + 0x300)

#define SUN6I_CTL_ENABLE            BIT(0)
#define SUN6I_CTL_MASTER            BIT(1)
#define SUN6I_CTL_SRST              BIT(31)
#define SUN6I_TCR_XCH               BIT(31)

/*****************************************************************************/

#define CCM_AHB_GATING0             (0x01C20000 + 0x60)
#define CCM_SPI0_CLK                (0x01C20000 + 0xA0)
#define SUN6I_BUS_SOFT_RST_REG0     (0x01C20000 + 0x2C0)

#define AHB_RESET_SPI0_SHIFT        20
#define AHB_GATE_OFFSET_SPI0        20

#define SPI0_CLK_DIV_BY_2           0x1000
#define SPI0_CLK_DIV_BY_4           0x1001
#define SPI0_CLK_DIV_BY_32          0x100f

/*****************************************************************************/

/*
 * Allwinner A10/A20 SoCs were using pins PC0,PC1,PC2,PC23 for booting
 * from SPI Flash, everything else is using pins PC0,PC1,PC2,PC3.
 */
static void spi0_pinmux_setup(unsigned int pin_function)
{
	unsigned int pin;

	for (pin = SUNXI_GPC(0); pin <= SUNXI_GPC(2); pin++)
		sunxi_gpio_set_cfgpin(pin, pin_function);

	if (IS_ENABLED(CONFIG_MACH_SUN4I) || IS_ENABLED(CONFIG_MACH_SUN7I))
		sunxi_gpio_set_cfgpin(SUNXI_GPC(23), pin_function);
	else
		sunxi_gpio_set_cfgpin(SUNXI_GPC(3), pin_function);
}

/*
 * Setup 6 MHz from OSC24M (because the BROM is doing the same).
 */
static void spi0_enable_clock(void)
{
	/* Deassert SPI0 reset on SUN6I */
	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I))
		setbits_le32(SUN6I_BUS_SOFT_RST_REG0,
			     (1 << AHB_RESET_SPI0_SHIFT));

	/* Open the SPI0 gate */
	setbits_le32(CCM_AHB_GATING0, (1 << AHB_GATE_OFFSET_SPI0));

#ifndef CONFIG_MACH_SUNIV
	/* Divide by 4 */
	writel(SPI0_CLK_DIV_BY_4, IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I) ?
				  SUN6I_SPI0_CCTL : SUN4I_SPI0_CCTL);
	/* 24MHz from OSC24M */
	writel((1 << 31), CCM_SPI0_CLK);
#else
	/* Divide by 32, clock source is AHB clock 200MHz */
	writel(SPI0_CLK_DIV_BY_32, IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I) ?
				  SUN6I_SPI0_CCTL : SUN4I_SPI0_CCTL);
#endif

	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I)) {
		/* Enable SPI in the master mode and do a soft reset */
		setbits_le32(SUN6I_SPI0_GCR, SUN6I_CTL_MASTER |
					     SUN6I_CTL_ENABLE |
					     SUN6I_CTL_SRST);
		/* Wait for completion */
		while (readl(SUN6I_SPI0_GCR) & SUN6I_CTL_SRST)
			;
	} else {
		/* Enable SPI in the master mode and reset FIFO */
		setbits_le32(SUN4I_SPI0_CTL, SUN4I_CTL_MASTER |
					     SUN4I_CTL_ENABLE |
					     SUN4I_CTL_TF_RST |
					     SUN4I_CTL_RF_RST);
	}
}

static void spi0_disable_clock(void)
{
	/* Disable the SPI0 controller */
	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I))
		clrbits_le32(SUN6I_SPI0_GCR, SUN6I_CTL_MASTER |
					     SUN6I_CTL_ENABLE);
	else
		clrbits_le32(SUN4I_SPI0_CTL, SUN4I_CTL_MASTER |
					     SUN4I_CTL_ENABLE);

#ifndef CONFIG_MACH_SUNIV
	/* Disable the SPI0 clock */
	writel(0, CCM_SPI0_CLK);
#endif

	/* Close the SPI0 gate */
	clrbits_le32(CCM_AHB_GATING0, (1 << AHB_GATE_OFFSET_SPI0));

	/* Assert SPI0 reset on SUN6I */
	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I))
		clrbits_le32(SUN6I_BUS_SOFT_RST_REG0,
			     (1 << AHB_RESET_SPI0_SHIFT));
}

static void spi0_init(void)
{
	unsigned int pin_function = SUNXI_GPC_SPI0;

	if (IS_ENABLED(CONFIG_MACH_SUN50I))
		pin_function = SUN50I_GPC_SPI0;

	if (IS_ENABLED(CONFIG_MACH_SUNIV))
		pin_function = SUNIV_GPC_SPI0;

	/*
	 * suniv comes with a sun6i-style SPI controller at the
	 * sun4i SPI base address
	 */
	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I) &&
	    !IS_ENABLED(CONFIG_MACH_SUNIV))
		spi0_base = 0x01c68000;
	else
		spi0_base = 0x01c05000;

	spi0_pinmux_setup(pin_function);
	spi0_enable_clock();
}

static void spi0_deinit(void)
{
	/* New SoCs can disable pins, older could only set them as input */
	unsigned int pin_function = SUNXI_GPIO_INPUT;
	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I))
		pin_function = SUNXI_GPIO_DISABLE;

	spi0_disable_clock();
	spi0_pinmux_setup(pin_function);
}

/*****************************************************************************/

#define SPI_READ_MAX_SIZE 56 // 60 /* FIFO size, minus 4 bytes of the header */

#define CONFIG_SPINAND
#ifdef CONFIG_SPINAND

typedef enum {
	SN_WRITE_ENABLE = 0x06 ,
	SN_WRITE_DISABLE = 0x04 ,
	SN_GET_FEATURES = 0x0F ,
	SN_SET_FEATURES = 0x1F ,
	SN_PAGE_READ_CACHE = 0x13 , 
	SN_READ_FROM_CACHE = 0x03 ,
	SN_FREAD_FROM_CACHE = 0x0B ,
	SN_READ_FROM_CACHE2X = 0x3B ,
	SN_READ_FROM_CACHE4X = 0x6B ,
	SN_READ_FROM_CACHED = 0xBB ,
	SN_READ_FROM_CACHEQ = 0xEB ,
	SN_READ_ID = 0x9F ,
	SN_PROGRAM_LOAD = 0x02 ,
	SN_PROGRAM_LOAD4X = 0x32 ,
	SN_PROGRAM_EXECUTE = 0x10 ,
	SN_PROGRAM_RANDOM_LOAD = 0x84 ,
	SN_PROGRAM_RANDOM_LOAD4X = 0xC4 , // 0x34
	SN_BLOCK_ERASE = 0xD8 ,
	SN_RESET = 0xFF ,
}spinand_cmd_t;
typedef enum {
	WRAP2112B = 0x0 ,
	WRAP2048B = 0x4 ,
	WRAP64B = 0x8 ,
	WRAP16B = 0xC ,
}spinand_wrap_lenth_t;
typedef enum {
	FEATURE_PROTECTION_ADDR = 0xA0 ,
	FEATURE_FEATURE_ADDR = 0xB0 ,
	FEATURE_STATUS_ADDR = 0xC0 ,
}spinand_feature_addr_t;

#define STATUS_OIP_OFFSET        0
#define STATUS_OIP_MASK          0x1
#define STATUS_WEL_OFFSET        1
#define STATUS_WEL_MASK          0x2
#define STATUS_EFAIL_OFFSET      2
#define STATUS_EFAIL_MASK        0x4

int pages = -1;
#endif
static u32 sunxi_spi0_read_data(u8 *buf, u32 addr, u32 bufsize,
				 ulong spi_ctl_reg,
				 ulong spi_ctl_xch_bitmask,
				 ulong spi_fifo_reg,
				 ulong spi_tx_reg,
				 ulong spi_rx_reg,
				 ulong spi_bc_reg,
				 ulong spi_tc_reg,
				 ulong spi_bcc_reg)
#ifdef CONFIG_SPINAND
{
	u32 ret = bufsize;
	//addr += 0x8000; //modify by leijie
	int page = addr >> 11;
	if( pages != page ) { // need read to cache
		writel(4, spi_bc_reg); /* Burst counter (total bytes) */
        	writel(4, spi_tc_reg);           /* Transfer counter (bytes to send) */
        	if (spi_bcc_reg)
                	writel(4, spi_bcc_reg);  /* SUN6I also needs this */
		/* Send the Read Data Bytes (03h) command header */
	        writeb(SN_PAGE_READ_CACHE, spi_tx_reg);
	        writeb((u8)(page >> 16), spi_tx_reg);
	        writeb((u8)(page >> 8), spi_tx_reg);
	        writeb((u8)(page), spi_tx_reg);

        	/* Start the data transfer */
	        setbits_le32(spi_ctl_reg, spi_ctl_xch_bitmask);
        	udelay(300);
		pages = page;
//		printf("pages = 0x%x bufsize = 0x%x\n", pages, bufsize);
        	/* Wait until everything is received in the RX FIFO */
        	while ((readl(spi_fifo_reg) & 0x7F) < 4)
                ;
        	/* Skip 4 bytes */
       	 	readl(spi_rx_reg);
	}
	addr &= 0x7FF;
	if( bufsize + addr > 0x800 ) 
		bufsize = 0x800 - addr;

	writel(4+bufsize, spi_bc_reg); /* Burst counter (total bytes) */
        writel(4, spi_tc_reg);           /* Transfer counter (bytes to send) */
        if (spi_bcc_reg)
                writel(4, spi_bcc_reg);  /* SUN6I also needs this */
	
	spinand_wrap_lenth_t wrap;
        if( bufsize > 0 && bufsize <= 16 ) {
                wrap = WRAP16B;
        } else if( bufsize > 16 && bufsize <= 64 ) {
                wrap = WRAP64B;
        } else if( bufsize > 65 && bufsize <= 2048 ) {
                wrap = WRAP2048B;
        } else {
                wrap = WRAP2112B;
        }
	// package check
	switch(wrap)
	{
		case WRAP16B:
			if( (addr&0xF) + bufsize > 16 ) wrap = WRAP64B; // suggest parentheses around comparison in operand of '&'
			break;
		case WRAP64B:
			if( (addr&0x3F) + bufsize > 65 ) wrap = WRAP2048B;
			break;
		default:
			wrap = WRAP2112B;
	}
        /* Send the Read Data Bytes (03h) command header */
// printf(">>> addr = 0x%x\n", addr);
        writeb(SN_READ_FROM_CACHE, spi_tx_reg);
        writeb((u8)((addr >> 8) & 0xF) | (wrap << 4), spi_tx_reg);
        writeb((u8)(addr ) & 0xFF, spi_tx_reg);
        writeb((u8)(0), spi_tx_reg);

        /* Start the data transfer */
        setbits_le32(spi_ctl_reg, spi_ctl_xch_bitmask);
	// for read from cache
	udelay(10);

        /* Wait until everything is received in the RX FIFO */
        while ((readl(spi_fifo_reg) & 0x7F) < 4+bufsize)
                ;

        /* Skip 4 bytes */
        readl(spi_rx_reg);

	ret = bufsize;





        /* Read the data */
	u8 da = 0;
//	printf(">>> ");
        while (bufsize-- > 0) {
                da = readb(spi_rx_reg);
                *buf++ = da;
//		printf("%02x ", da);
	}
//	printf("\n");

        /* tSHSL time is up to 100 ns in various SPI flash datasheets */
        udelay(1);
	return ret;
}
#else
{
	writel(4 + bufsize, spi_bc_reg); /* Burst counter (total bytes) */
	writel(4, spi_tc_reg);           /* Transfer counter (bytes to send) */
	if (spi_bcc_reg)
		writel(4, spi_bcc_reg);  /* SUN6I also needs this */

	/* Send the Read Data Bytes (03h) command header */
	writeb(0x03, spi_tx_reg);
	writeb((u8)(addr >> 16), spi_tx_reg);
	writeb((u8)(addr >> 8), spi_tx_reg);
	writeb((u8)(addr), spi_tx_reg);

	/* Start the data transfer */
	setbits_le32(spi_ctl_reg, spi_ctl_xch_bitmask);

	/* Wait until everything is received in the RX FIFO */
	while ((readl(spi_fifo_reg) & 0x7F) < 4 + bufsize)
		;

	/* Skip 4 bytes */
	readl(spi_rx_reg);

	/* Read the data */
	while (bufsize-- > 0)
		*buf++ = readb(spi_rx_reg);

	/* tSHSL time is up to 100 ns in various SPI flash datasheets */
	udelay(1);
}
#endif
static void spi0_read_data(void *buf, u32 addr, u32 len)
{
	u8 *buf8 = buf;
	u32 chunk_len;
	// printf("addr = 0x%x len = 0x%x\n", addr, len);
	while (len > 0) {
		chunk_len = len;
		if (chunk_len > SPI_READ_MAX_SIZE)
			chunk_len = SPI_READ_MAX_SIZE;

		if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I)) {
			chunk_len = sunxi_spi0_read_data(buf8, addr, chunk_len,
					     SUN6I_SPI0_TCR,
					     SUN6I_TCR_XCH,
					     SUN6I_SPI0_FIFO_STA,
					     SUN6I_SPI0_TXD,
					     SUN6I_SPI0_RXD,
					     SUN6I_SPI0_MBC,
					     SUN6I_SPI0_MTC,
					     SUN6I_SPI0_BCC);
		} else {
			chunk_len = sunxi_spi0_read_data(buf8, addr, chunk_len,
					     SUN4I_SPI0_CTL,
					     SUN4I_CTL_XCH,
					     SUN4I_SPI0_FIFO_STA,
					     SUN4I_SPI0_TX,
					     SUN4I_SPI0_RX,
					     SUN4I_SPI0_BC,
					     SUN4I_SPI0_TC,
					     0);
		}

		len  -= chunk_len;
		buf8 += chunk_len;
		addr += chunk_len;
	}
}

static ulong spi_load_read(struct spl_load_info *load, ulong sector,
			   ulong count, void *buf)
{
	spi0_read_data(buf, sector, count);

	return count;
}

/*****************************************************************************/

static int spl_spi_load_image(struct spl_image_info *spl_image,
			      struct spl_boot_device *bootdev)
{
	int ret = 0;
	struct image_header *header;
	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE);

	spi0_init();

	spi0_read_data((void *)header, CONFIG_SYS_SPI_U_BOOT_OFFS, 0x40);

        if (IS_ENABLED(CONFIG_SPL_LOAD_FIT) &&
		image_get_magic(header) == FDT_MAGIC) {
		struct spl_load_info load;

		debug("Found FIT image\n");
		load.dev = NULL;
		load.priv = NULL;
		load.filename = NULL;
		load.bl_len = 1;
		load.read = spi_load_read;
		ret = spl_load_simple_fit(spl_image, &load,
					  CONFIG_SYS_SPI_U_BOOT_OFFS, header);
	} else {
		ret = spl_parse_image_header(spl_image, header);
		if (ret)
			return ret;
		pages = -1;
		spi0_read_data((void *)spl_image->load_addr,
			       CONFIG_SYS_SPI_U_BOOT_OFFS, spl_image->size);
	}

	spi0_deinit();

	return ret;
}
/* Use priorty 0 to override the default if it happens to be linked in */
SPL_LOAD_IMAGE_METHOD("sunxi SPI", 0, BOOT_DEVICE_SPI, spl_spi_load_image);
