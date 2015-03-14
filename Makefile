 CROSS_COMPILE = gcc

BASE_HOME =$(shell pwd)
ARCH = S5PV210
KERNEL_VERSION = 3.10
BUSYBOX_VERSION = 1.10

export ARCH KERNEL_VERSION

include config/env.depend


.PHONY: all clean

all: kernel-3.10 uboot busybox

kernel-3.10: 
	cp -rf $(KERNEL_CONFIG) $(KERNEL_DIR)/.config
	cd $(KERNEL_DIR); \
	make zImage
	echo "######make uboot head  for zImage#####"
	mkdir -p $(IMAGE_DIR)
	$(MKIMAGE_BIN) -n linux-$(KERNEL_VERSION) -A arm -O linux -T kernel -C none -a 0x30008000 -e 0x30008040 -d \
	(KERNEL_DIR)/arch/$(ARCH)/boot/zImage $(IMAGE_DIR)/zImage.img #todo


uboot:
	cd $(UBOOT_DIR); \
	make $(ARCH)_deconfig; \
	echo "######make uboot #####"
	make 
	mkdir -p $(BOOT_DIR) 
	cp $(UBOOT_DIR)/uboot $(BOOT_DIR)

busybox:
	cp -rf $(BUSYBOX_CONFIG) $(BUSYBOX_DIR)/.config
	cd $(BUSYBOX_DIR); \
	make zImage; \
	mkdir -p $(ROOT_DIR); \
	cp -Rf $(ROOT_COMMON) $(ROOT_DIR);\ 
	cp -Rf  install/* $(ROOT_DIR)	




