# GB9260 V2 BOARD

### make uboot:
"bootargs=" "console=ttyS0,115200n8 root=ubi0:rootfs rw ubi.mtd=3,2048 rootfstype=ubifs rootwait=1 init=/linuxrc " \
					"mtdparts=atmel_nand:128K@0(bootstrap),896K@128K(bootloader),5M@1M(kernel),-@6M(rootfs)\0"

make distclean
make at91sam9260ek_nandflash_config 
make CROSS_COMPILE=/home/yangye/linux/gcc/arm-none-linux-gnueabi/bin/arm-linux-

### make kernel:
make distclean
cp gb9260_config .config
make ARCH=arm CROSS_COMPILE=/usr/local/arm/arm-2008q3-linux/bin/arm-linux-  uImage

### make rootfs:
mkfs.ubifs -m 2048 -c 864 -e 126976 -r ./roms -o fs_9260.ubifs 
ubinize -o gb9260.cramfs -m 2048 -p 128KiB -s 2048 ubinize.cfg 
