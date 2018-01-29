/*
 * *  ffu.h
 *
 *  Copyright 2007-2008 Pierre Ossman
 *
 *  Modified by SanDisk Corp., Copyright (c) 2014 SanDisk Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program includes bug.h, card.h, host.h, mmc.h, scatterlist.h,
 * slab.h, ffu.h & swap.h header files
 * The original, unmodified version of this program - the mmc_test.c
 * file - is obtained under the GPL v2.0 license that is available via
 * http://www.gnu.org/licenses/,
 * or http://www.opensource.org/licenses/gpl-2.0.php
*/
/*MMC ffu support*/

/*
 * eMMC5.0 Field Firmware Update (FFU) opcodes
*/
#define MMC_FFU_INVOKE_OP 302

#define MMC_FFU_MODE_SET 0x1
#define MMC_FFU_MODE_NORMAL 0x0
#define MMC_FFU_INSTALL_SET 0x2
#define CARD_BLOCK_SIZE 512
#ifdef CONFIG_MMC_FFU
#define MMC_FFU_ENABLE 0x0
#define MMC_FFU_CONFIG 0x1
#define MMC_FFU_SUPPORTED_MODES 0x1
#define MMC_FFU_FEATURES 0x1
#define FFU_ENABLED(ffu_enable)	(ffu_enable & MMC_FFU_CONFIG)
#define FFU_SUPPORTED_MODE(ffu_sup_mode) \
	(ffu_sup_mode && MMC_FFU_SUPPORTED_MODES) 
#define FFU_CONFIG(ffu_config) (ffu_config & MMC_FFU_CONFIG) 
#define FFU_FEATURES(ffu_fetures) (ffu_fetures & MMC_FFU_FEATURES)
#define FFU_FIXUP(_manfid ,_pnm,_path,_version) \
{\
            .manfid =( _manfid),\
            .firmware_path = (_path),\
            .prod_name =(_pnm),\
            .firmware_version =(_version)\
}
int mmc_ffu_firmware_fixup (struct mmc_card *card);
#endif
