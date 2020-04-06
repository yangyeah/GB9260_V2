/*
 * (C) Copyright 2007-2008
 * Stelian Pop <stelian@popies.net>
 * Lead Tech Design <www.leadtechdesign.com>
 *
 * Configuation settings for the AT91SAM9260EK & AT91SAM9G20EK boards.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * SoC must be defined first, before hardware.h is included.
 * In this case SoC is defined in boards.cfg.
 */
#include <asm/hardware.h>

/*
 * Warning: changing CONFIG_SYS_TEXT_BASE requires
 * adapting the initial boot program.
 * Since the linker has to swallow that define, we must use a pure
 * hex number here!
 */

//#define DEBUG 1
#define CONFIG_SYS_TEXT_BASE		0x21f00000

/* ARM asynchronous clock */
#define CONFIG_SYS_AT91_SLOW_CLOCK	32768		/* slow clock xtal */
#define CONFIG_SYS_AT91_MAIN_CLOCK	18432000	/* main clock xtal */
#define CONFIG_SYS_HZ			1000

/* Define actual evaluation board type from used processor type */
#ifdef CONFIG_AT91SAM9G20
# define CONFIG_AT91SAM9G20EK	/* It's an Atmel AT91SAM9G20 EK */
#else
# define CONFIG_AT91SAM9260EK	/* It's an Atmel AT91SAM9260 EK */
#endif

/* Misc CPU related */
#define CONFIG_ARCH_CPU_INIT
#define CONFIG_CMDLINE_TAG		/* enable passing of ATAGs */
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_SKIP_LOWLEVEL_INIT
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_DISPLAY_CPUINFO

/* general purpose I/O */
#define CONFIG_ATMEL_LEGACY		/* required until (g)pio is fixed */
#define CONFIG_AT91_GPIO
#define CONFIG_AT91_GPIO_PULLUP	1	/* keep pullups on peripheral pins */

/* serial console */
#define CONFIG_ATMEL_USART
#define CONFIG_USART_BASE		ATMEL_BASE_DBGU
#define	CONFIG_USART_ID			ATMEL_ID_SYS
#define CONFIG_BAUDRATE			115200

/* LED */
#define CONFIG_AT91_LED
#define	CONFIG_RED_LED		AT91_PIN_PB23	/* this is the power led */
#define	CONFIG_GREEN_LED	AT91_PIN_PB25	/* this is the user led */

#define CONFIG_BOOTDELAY	3

/*
 * BOOTP options
 */
#define CONFIG_BOOTP_BOOTFILESIZE	1
#define CONFIG_BOOTP_BOOTPATH		1
#define CONFIG_BOOTP_GATEWAY		1
#define CONFIG_BOOTP_HOSTNAME		1

/*
 * Command line configuration.
 */
#include <config_cmd_default.h>
#undef CONFIG_CMD_BDI
#undef CONFIG_CMD_FPGA
#undef CONFIG_CMD_IMI
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_LOADS
#undef CONFIG_CMD_SOURCE
#undef CONFIG_CMD_DHCP

#define CONFIG_CMD_PING
#define CONFIG_CMD_NAND
#define CONFIG_CMD_USB

/*
 * SDRAM: 1 bank, min 32, max 128 MB
 * Initialized before u-boot gets started.
 */
#define CONFIG_NR_DRAM_BANKS		1
#define CONFIG_SYS_SDRAM_BASE		ATMEL_BASE_CS1
#define CONFIG_SYS_SDRAM_SIZE		0x02000000

/*
 * Initial stack pointer: 4k - GENERATED_GBL_DATA_SIZE in internal SRAM,
 * leaving the correct space for initial global data structure above
 * that address while providing maximum stack area below.
 */
#ifdef CONFIG_AT91SAM9XE
# define CONFIG_SYS_INIT_SP_ADDR \
	(ATMEL_BASE_SRAM + 0x1000 - GENERATED_GBL_DATA_SIZE)
#else
# define CONFIG_SYS_INIT_SP_ADDR \
	(ATMEL_BASE_SRAM1 + 0x1000 - GENERATED_GBL_DATA_SIZE)
#endif


#if 0
/* DataFlash */
#define CONFIG_ATMEL_DATAFLASH_SPI
#define CONFIG_HAS_DATAFLASH		1
#define CONFIG_SYS_SPI_WRITE_TOUT		(5*CONFIG_SYS_HZ)
#define CONFIG_SYS_MAX_DATAFLASH_BANKS		2
#define CONFIG_SYS_DATAFLASH_LOGIC_ADDR_CS0	0xC0000000	/* CS0 */
#define CONFIG_SYS_DATAFLASH_LOGIC_ADDR_CS1	0xD0000000	/* CS1 */
#define AT91_SPI_CLK			15000000
#endif


#ifdef CONFIG_AT91SAM9G20EK
#define DATAFLASH_TCSS			(0x22 << 16)
#else
#define DATAFLASH_TCSS			(0x1a << 16)
#endif
#define DATAFLASH_TCHS			(0x1 << 24)

/* NAND flash */
#ifdef CONFIG_CMD_NAND
#define CONFIG_NAND_ATMEL
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_BASE		ATMEL_BASE_CS3
#define CONFIG_SYS_NAND_DBW_8

#define CONFIG_SYS_NAND_MASK_ALE	(1 << 21)
#define CONFIG_SYS_NAND_MASK_CLE	(1 << 22)
#define CONFIG_SYS_NAND_ENABLE_PIN	AT91_PIN_PC14
#define CONFIG_SYS_NAND_READY_PIN	AT91_PIN_PC13
#define CONFIG_SYS_NAND_WP_PIN      AT91_PIN_PC22
#endif

/* NOR flash - no real flash on this board */
#define CONFIG_SYS_NO_FLASH			1

/* Ethernet */
#define CONFIG_MACB			1
#define CONFIG_RMII			1
#define CONFIG_NET_RETRY_COUNT		20
#define CONFIG_RESET_PHY_R		1

#define CONFIG_NETMASK		255.255.255.0	/* talk on MY local net */
#define CONFIG_IPADDR		192.168.1.100	/* static IP I currently own */
#define CONFIG_GATEWAYIP	192.168.1.1	/* current Gateway IP of my dev pc */
#define CONFIG_SERVERIP		192.168.1.33/* current IP of my dev pc */
#define CONFIG_ETHADDR 		00:e0:a3:a4:98:67

/* USB */
#define CONFIG_USB_ATMEL
#define CONFIG_USB_OHCI_NEW		1
#define CONFIG_DOS_PARTITION		1
#define CONFIG_SYS_USB_OHCI_CPU_INIT		1
#define CONFIG_SYS_USB_OHCI_REGS_BASE		0x00500000	/* AT91SAM9260_UHP_BASE */
#define CONFIG_SYS_USB_OHCI_SLOT_NAME		"at91sam9260"
#define CONFIG_SYS_USB_OHCI_MAX_ROOT_PORTS	2
#define CONFIG_USB_STORAGE		1
#define CONFIG_CMD_FAT			1

#define CONFIG_SYS_LOAD_ADDR			0x21000000	/* load address */

#define CONFIG_SYS_MEMTEST_START		CONFIG_SYS_SDRAM_BASE
#define CONFIG_SYS_MEMTEST_END			0x21e00000



/* bootstrap + u-boot + env + linux in nandflash */
#define CONFIG_ENV_IS_IN_NAND	1
#define CONFIG_ENV_OFFSET		0xa0000
#define CONFIG_ENV_OFFSET_REDUND	0xc0000
#define CONFIG_ENV_SIZE		0x20000		/* 1 sector = 128 kB */


#define	CONFIG_CMD_UBI
#define	CONFIG_CMD_UBIFS
#define	CONFIG_CMD_MTDPARTS
#define	CONFIG_MTD_DEVICE
#define	CONFIG_MTD_PARTITIONS
#define	CONFIG_LZO
#define	CONFIG_RBTREE
#define MTDIDS_DEFAULT "nand0=atmel_nand"
#define MTDPARTS_DEFAULT "mtdparts=atmel_nand:128K@0(bootstrap),896K@128K(bootloader),5M@1M(kernel),-@6M(rootfs)"



				
#define CONFIG_BOOTARGS		


#if 1
#define	CONFIG_EXTRA_ENV_SETTINGS 													\
	"bootfile=uImage\0"																\
	"kernel=uImage\0"                                               				\
    "rootfs=gb9260.cramfs\0"                                         				\
	"uboot=u-boot.bin\0"															\
	"bootstrap=at91sam9260ek-nandflashboot-uboot-3.8.7.bin\0"						\
	"ubootoffset=0x20000\0"															\
	"ubootsize=0xE0000\0"															\
	"kernelsize=0x300000\0"                                               			\
	"loadaddr=0x20800000\0"                                         				\
	"upkernel_usb="         "usb reset;"                       						\
                            "usb dev 0;fatload usb 0 $(loadaddr) $(kernel);"        \
                            "mtdparts default;"										\
                            "nand erase.part kernel;"								\
                            "nand write $(loadaddr) kernel $(kernelsize)\0"         \
	"uprootfs_usb=" 		"usb reset;"											\
						    "usb dev 0;fatload usb 0 $(loadaddr) $(rootfs);" 		\
						    "mtdparts default;"										\
						    "nand erase.part rootfs;" 								\
						    "nand write $(loadaddr) rootfs  $(filesize)\0" 			\
	"upkernel_ftp=" 		"mtdparts default;"										\
							"nand erase.part kernel;" 								\
							"tftp $(loadaddr) $(kernel);"							\
							"nand write $(loadaddr) kernel $(kernelsize)\0" 		\
	"uprootfs_ftp=" 		"mtdparts default;"										\
							"nand erase.part rootfs;" 								\
							"tftp $(loadaddr) $(rootfs);"							\
							"nand write $(loadaddr) rootfs $(filesize)\0" 			\
	"upuboot_usb=" 			"usb reset;"									   		\
						    "usb dev 0;fatload usb 0 $(loadaddr) $(uboot);" 		   \
						    "nand erase $(ubootoffset) $(ubootsize);"				   \
							"nand write $(loadaddr) $(ubootoffset) $(ubootsize)\0"	   \
	"upuboot_ftp=" 			"tftp $(loadaddr) $(uboot);"							   \
							"nand erase $(ubootoffset) $(ubootsize);"				   \
							"nand write $(loadaddr) $(ubootoffset) $(ubootsize)\0"	   \
	"upbootstrap_usb=" 		"usb reset;"									   			\
						    "usb dev 0;fatload usb 0 $(loadaddr) $(bootstrap);" 		\
						    "nand erase 0 0x20000;"				   						\
							"nand write $(loadaddr) 0 0x1000\0"	   						\
	"upbootstrap_ftp=" 		"tftp $(loadaddr) $(bootstrap);"							\
							"nand erase 0 0x20000;"				  						\
							"nand write $(loadaddr) 0 0x1000\0"	   						\
    "upall_ftp="			"run upbootstrap_ftp;run upuboot_ftp;run upkernel_ftp;run uprootfs_ftp;reset \0" 			   \
    "upall_usb="			"run upbootstrap_usb;run upuboot_usb;run upkernel_usb;run uprootfs_usb;reset \0" 			   \
    "bootcmd="				"nand read 0x20800000 0x100000 0x300000; bootm \0"     	   \
    "bootargs="         	"console=ttyS0,115200n8 root=ubi0:rootfs rw ubi.mtd=3,2048 rootfstype=ubifs rootwait=1 init=/linuxrc " \
					"mtdparts=atmel_nand:128K@0(bootstrap),896K@128K(bootloader),5M@1M(kernel),-@6M(rootfs)\0"


#endif



#define CONFIG_SYS_PROMPT		"U-Boot> "
#define CONFIG_SYS_CBSIZE		256
#define CONFIG_SYS_MAXARGS		16
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_LONGHELP		1
#define CONFIG_CMDLINE_EDITING	1
#define CONFIG_AUTO_COMPLETE

/*
 * Size of malloc() pool
 */
#define CONFIG_SYS_MALLOC_LEN		ROUND(3 * CONFIG_ENV_SIZE + 128*1024, 0x1000)

#endif
