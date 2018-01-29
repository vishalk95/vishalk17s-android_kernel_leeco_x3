/*
 * *  ffu.c
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

#include <linux/bug.h>
#include <linux/errno.h>
#include <linux/mmc/card.h>
#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/swap.h>
#include <linux/firmware.h>

#include "ffu.h"

/**
 * struct mmc_ffu_pages - pages allocated by 'alloc_pages()'.
 * @page: first page in the allocation
 * @order: order of the number of pages allocated
 */
struct mmc_ffu_pages {
             struct page *page;
             unsigned int order;
};

/**
 * struct mmc_ffu_mem - allocated memory.
 * @arr: array of allocations
 * @cnt: number of allocations
 */
struct mmc_ffu_mem {
             struct mmc_ffu_pages *arr;
             unsigned int cnt;
};

struct mmc_ffu_area {
             unsigned long max_sz;
             unsigned int max_tfr;
             unsigned int max_segs;
             unsigned int max_seg_sz;
             unsigned int blocks;
             unsigned int sg_len;
             struct mmc_ffu_mem mem;
             struct sg_table sgtable;
};

struct mmc_ffu_fw{
            unsigned int    manfid;
            const char *firmware_path;
            const char  *prod_name;
            u64  firmware_version ;
};
                         
static const  struct mmc_ffu_fw firmware_fixups[] = {FFU_FIXUP(0x11,"064G70","toshiba/emmc_fw_update.img",0x3), 
                                                     };
/*
static const  struct mmc_ffu_fw firmware_fixups[] = {FFU_FIXUP(0x11,"064G70","toshiba/emmc_fw_update.img",0x3), 
													 FFU_FIXUP(0x11,"064G70","toshiba/064G70_PNM_test_FwRev02.img",0x4),
                                                     };
// for test
*/

/*
 * Map memory into a scatterlist.
*/
static unsigned int mmc_ffu_map_sg(struct mmc_ffu_mem *mem, int size,
             struct scatterlist *sglist,unsigned int max_seg_sz)
{
             struct scatterlist *sg = sglist;
             unsigned int i;
             unsigned long sz = size;
             unsigned int sctr_len = 0;
             unsigned long len = 0;
    
             for (i = 0; i < mem->cnt && sz; i++, sz -= len) {
                             len = PAGE_SIZE << mem->arr[i].order;

                             if(len >max_seg_sz ){
                                  len = max_seg_sz;
                             }
                             
                             if (len > sz) {
                                             len = sz;
                                             sz = 0;
                             }

                             sg_set_page(sg, mem->arr[i].page, len, 0);
                             sg = sg_next(sg);
                             sctr_len++;
             }
            
             return sctr_len;
}

static void mmc_ffu_free_mem(struct mmc_ffu_mem *mem)
{
             if (!mem)
                             return;

             while (mem->cnt--)
                             __free_pages(mem->arr[mem->cnt].page, mem->arr[mem->cnt].order);

             kfree(mem->arr);
}

/*
 * Cleanup struct mmc_ffu_area.
 */
static int mmc_ffu_area_cleanup(struct mmc_ffu_area *area)
{
             sg_free_table(&area->sgtable);
             mmc_ffu_free_mem(&area->mem);
             return 0;
}

/*
 * Allocate a lot of memory, preferably max_sz but at least min_sz. In case
 * there isn't much memory do not exceed 1/16th total low mem pages. Also do
 * not exceed a maximum number of segments and try not to make segments much
 * bigger than maximum segment size.
 */
static int mmc_ffu_alloc_mem(struct mmc_ffu_area *area)
{
             unsigned int cnt = 0;

             gfp_t flags = GFP_KERNEL | GFP_DMA | __GFP_NOWARN | __GFP_NORETRY;

             area->mem.cnt = DIV_ROUND_UP(area->max_tfr,area->max_seg_sz);
             
             area->mem.arr = kzalloc(sizeof(struct mmc_ffu_pages) *  area->mem.cnt,
                             GFP_KERNEL);
             
             if (!area->mem.arr)
                             goto out_free;

             while(cnt  <  area->mem.cnt){

                         struct page *page;
                          unsigned int order;
                          
                         order = get_order(area->max_seg_sz);

                         page = alloc_pages(flags, order);

                         if (!page)
                                       goto out_free;

                          area->mem.arr[cnt].page = page;
                          area->mem.arr[cnt].order = order;
                          cnt++;
             }
             return 0;

out_free:
             mmc_ffu_free_mem(&area->mem);
             return -ENOMEM;
}

/*
 * Initialize an area for data transfers.
 * Copy the data to the allocated pages.
 */
static int mmc_ffu_area_init(struct mmc_ffu_area *area, struct mmc_card *card,
             const u8 *data)
{
             int ret;
             int i;
             unsigned int length = 0, page_length;
             
             ret = mmc_ffu_alloc_mem(area);
             
             for (i = 0; i < area->mem.cnt; i++) {
                
                             if (length > area->max_tfr) {
                                             ret = -EINVAL;
                                             goto out_free;
                             }
                             
                             memcpy(page_address(area->mem.arr[i].page), data + length,
                                             min(area->max_tfr - length, area->max_seg_sz));
                             length += area->max_seg_sz;
             }

             ret = sg_alloc_table(&area->sgtable, area->mem.cnt, GFP_KERNEL);
             if (ret)
                             goto out_free;

             area->sg_len = mmc_ffu_map_sg(&area->mem, area->max_tfr,
                             area->sgtable.sgl,area->max_seg_sz);


             return 0;

out_free:
             mmc_ffu_free_mem(&area->mem);
             return ret;
}

/*
 * Fill in the mmc_request structure given a set of transfer parameters.
 */
static void mmc_ffu_prepare_mrq(struct mmc_card *card,
	struct mmc_request *mrq, struct scatterlist *sg, unsigned sg_len,
	unsigned dev_addr, unsigned blocks, unsigned blksz, int write) {
	BUG_ON(!mrq || !mrq->cmd || !mrq->data || !mrq->stop);

	if (blocks > 1) {
		mrq->cmd->opcode = write ?
			MMC_WRITE_MULTIPLE_BLOCK : MMC_READ_MULTIPLE_BLOCK;
	} else {
		mrq->cmd->opcode = write ?
			MMC_WRITE_BLOCK : MMC_READ_SINGLE_BLOCK;
	}

	mrq->cmd->arg = dev_addr;
	if (!mmc_card_blockaddr(card))
		mrq->cmd->arg <<= 9;

	mrq->cmd->flags = MMC_RSP_R1 | MMC_CMD_ADTC;

	if (blocks == 1)
		mrq->stop = NULL;
	else {
		mrq->stop->opcode = MMC_STOP_TRANSMISSION;
		mrq->stop->arg = 0;
		mrq->stop->flags = MMC_RSP_R1B | MMC_CMD_AC;
	}

	mrq->data->blksz = blksz;
	mrq->data->blocks = blocks;
	mrq->data->flags = write ? MMC_DATA_WRITE : MMC_DATA_READ;
	mrq->data->sg = sg;
	mrq->data->sg_len = sg_len;

	mmc_set_data_timeout(mrq->data, card); 
    }


static int mmc_ffu_busy(struct mmc_command *cmd) {
	return !(cmd->resp[0] & R1_READY_FOR_DATA) ||
		(R1_CURRENT_STATE(cmd->resp[0]) == R1_STATE_PRG);
 }

/*
 * Wait for the card to finish the busy state  */  
static int mmc_ffu_wait_busy(struct mmc_card *card) {
	int ret, busy = 0;
	struct mmc_command cmd = {0};

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = MMC_SEND_STATUS;
	cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;

	do {
		ret = mmc_wait_for_cmd(card->host, &cmd, 0);
		if (ret)
			break;

		if (!busy && mmc_ffu_busy(&cmd)) {
			busy = 1;
			if (card->host->caps & MMC_CAP_WAIT_WHILE_BUSY) {
				pr_warn("%s: Warning: Host did not "
					"wait for busy state to end.\n",
					mmc_hostname(card->host));
			}
		}

	} while (mmc_ffu_busy(&cmd));

	return ret;
}

static int mmc_ffu_check_result(struct mmc_request *mrq) {
	int ret;

	BUG_ON(!mrq || !mrq->cmd || !mrq->data);

	ret = 0;

	if (!ret && mrq->cmd->error)
		ret = mrq->cmd->error;
	if (!ret && mrq->data->error)
		ret = mrq->data->error;
	if (!ret && mrq->stop && mrq->stop->error)
		ret = mrq->stop->error;
	if (!ret && mrq->data->bytes_xfered !=
		mrq->data->blocks * mrq->data->blksz)
		ret = -EPERM;

	return ret;
}

/*
 * transfer with certain parameters
 */
static int mmc_ffu_simple_transfer(struct mmc_card *card,
	struct scatterlist *sg, unsigned sg_len, unsigned dev_addr,
	unsigned blocks, unsigned blksz, int write) {
	struct mmc_request mrq = {0};
	struct mmc_command cmd = {0};
	struct mmc_command stop = {0};
	struct mmc_data data = {0};

	mrq.cmd = &cmd;
	mrq.data = &data;
	mrq.stop = &stop;
    
	mmc_ffu_prepare_mrq(card, &mrq, sg, sg_len, dev_addr,
		blocks, blksz, write);

	mmc_wait_for_req(card->host, &mrq);

	mmc_ffu_wait_busy(card);

	return mmc_ffu_check_result(&mrq);
}

static int mmc_ffu_write(struct mmc_card *card, const u8 *src, u32 arg,
             int size)
{
             int rc;
             struct mmc_ffu_area area = {0};
             int block_size = card->ext_csd.data_sector_size;

             area.max_segs = card->host->max_segs;
             area.max_seg_sz = card->host->max_seg_size & ~(block_size - 1);

             do {
                             area.max_tfr = size;
                             if (area.max_tfr >> 9 > card->host->max_blk_count)
                                             area.max_tfr = card->host->max_blk_count << 9;
                             if (area.max_tfr > card->host->max_req_size)
                                             area.max_tfr = card->host->max_req_size;
                             if (DIV_ROUND_UP(area.max_tfr, area.max_seg_sz) > area.max_segs)
                                             area.max_tfr = area.max_segs * area.max_seg_sz;

                             rc = mmc_ffu_area_init(&area, card, src);
                             if (rc != 0)
                                             goto exit;

                             rc = mmc_ffu_simple_transfer(card, area.sgtable.sgl, area.sg_len,
                                             arg, area.max_tfr / block_size, block_size, 1);
                             
                             mmc_ffu_area_cleanup(&area);
                             if (rc != 0) {
                                             pr_err("%s mmc_ffu_simple_transfer %d\n", __func__, rc);
                                             goto exit;
                             }
                             src  += area.max_tfr;
                             size -= area.max_tfr;

             } while (size > 0);

exit:
             return rc;
}

/* Flush all scheduled work from the MMC work queue.
 * and initialize the MMC device */
static int mmc_ffu_restart(struct mmc_card *card)
{
             struct mmc_host *host = card->host;
             int err = 0;

             err = mmc_power_save_host(host);
             if (err) {
                             pr_warn("%s: going to sleep failed (%d)!!!\n",
                                             __func__ , err);
                             goto exit;
             }

             err = mmc_power_restore_host(host);

exit:

             return err;
}

static int mmc_ffu_install(struct mmc_card *card,u8* extcsd) {
	u8 ext_csd[CARD_BLOCK_SIZE];
	int err;
	u32 timeout;

	/* check mode operation */
	if (!FFU_FEATURES(extcsd[EXT_CSD_FFU_FEATURES])) {

                printk("#mmc_ffu_install no support install mode \n");
     		/* set device to FFU mode */
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			EXT_CSD_MODE_CONFIG, MMC_FFU_MODE_NORMAL,
			card->ext_csd.generic_cmd6_time);

		if (err) {
			pr_err("FFU: %s: error %d FFU is not supported\n",
				mmc_hostname(card->host), err);
			goto exit;
		}
        
	} else {

		/* set device to FFU mode */
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			EXT_CSD_MODE_CONFIG, MMC_FFU_MODE_SET,
			card->ext_csd.generic_cmd6_time);

		if (err) {
			pr_err("FFU: %s: error %d FFU is not supported\n",
				mmc_hostname(card->host), err);
			goto exit;
		}

		timeout = extcsd[EXT_CSD_OPERATION_CODE_TIMEOUT];
		if (timeout == 0 || timeout > 0x17) {
			timeout = 0x17;
			pr_warn("FFU: %s: operation code timeout is out "
				"of range. Using maximum timeout.\n",
				mmc_hostname(card->host));
		}

		/* timeout is at millisecond resolution */
		timeout = (100 * (1 << timeout) / 1000) + 1;

		/* set ext_csd to install mode */
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			EXT_CSD_MODE_OPERATION_CODES,
			MMC_FFU_INSTALL_SET, timeout);

		if (err) {
			pr_err("FFU: %s: error %d setting install mode\n",
				mmc_hostname(card->host), err);
			goto exit;
		}
	}

	/* read ext_csd */
	err = mmc_send_ext_csd(card, ext_csd);
	if (err) {
		pr_err("FFU: %s: error %d sending ext_csd\n",
			mmc_hostname(card->host), err);
		goto exit;
	}

	/* return status */
	err = ext_csd[EXT_CSD_FFU_STATUS];

	if (err) {
		pr_err("FFU: %s: error %d FFU install:\n",
			mmc_hostname(card->host), err);
		err = -EINVAL;
		goto exit;
	}

exit:
	return err;
}

static int mmc_ffu_playload_checksum(struct mmc_card *card,size_t fw_size)
{
           u32 fw_prog_bytes;
           u8 ext_csd[CARD_BLOCK_SIZE];
            int block_size = card->ext_csd.data_sector_size;
           int err;
              /* Read the EXT_CSD */
             err = mmc_send_ext_csd(card, ext_csd);
              
             if (err) {
                             pr_err("FFU: %s: error %d sending ext_csd\n",
                                             mmc_hostname(card->host), err);
                      return  -EINVAL;
             }

             /* check that the eMMC has received the payload */
             fw_prog_bytes = ext_csd[EXT_CSD_NUM_OF_FW_SEC_PROG] |
                             ext_csd[EXT_CSD_NUM_OF_FW_SEC_PROG + 1] << 8 |
                             ext_csd[EXT_CSD_NUM_OF_FW_SEC_PROG + 2] << 16 |
                             ext_csd[EXT_CSD_NUM_OF_FW_SEC_PROG+ 3] << 24;

             /* convert sectors to bytes: multiply by -512B or 4KB as required by the card */
             
             fw_prog_bytes *=
                             block_size << (ext_csd[EXT_CSD_DATA_SECTOR_SIZE] * 3);
             
             printk("FFU playload check sum is :  %d",fw_prog_bytes);
             
             if (fw_prog_bytes != fw_size) {
                
                    pr_err("FFU: %s: error %d number of programmed fw sector "
                     "incorrect %d %zd\n", __func__, err,
                     fw_prog_bytes, fw_size); 
                      return  -EINVAL;
             }

            return 0;
}

static int mmc_ffu_upgrade(struct mmc_card *card, const char *name)
{
             u8 ext_csd[CARD_BLOCK_SIZE];
             int err;
             u32 arg;
			 u64 ffu_fw_version_after_update;
             const struct firmware *fw;
             int block_size = card->ext_csd.data_sector_size;
            
             if (strlen(name) > 512) {
                             pr_err("FFU: %s: %.20s is not a valid argument\n",
                                             mmc_hostname(card->host), name);
                             return -EINVAL;
             }

             /* setup FW data buffer */
             err = request_firmware(&fw, name, &card->dev);
             if (err) {
                             pr_err("FFU: %s: Firmware request failed %d\n",
                                             mmc_hostname(card->host), err);
                             return err;
             }

             if ((fw->size % block_size)) {
                             pr_warn("FFU: %s: Warning %zd firmware data size "
                                             "is not aligned!!!\n", mmc_hostname(card->host),
                                             fw->size);
             } 

            mmc_claim_host(card->host);

          /* Read the EXT_CSD */
            err = mmc_send_ext_csd(card, ext_csd);
            if (err) {
            	pr_err("FFU: %s: error %d sending ext_csd\n",
            		mmc_hostname(card->host), err);
            	goto exit;
            }

             /* set CMD ARG */
             arg = ext_csd[EXT_CSD_FFU_ARG] |
                             ext_csd[EXT_CSD_FFU_ARG + 1] << 8 |
                             ext_csd[EXT_CSD_FFU_ARG + 2] << 16 |
                             ext_csd[EXT_CSD_FFU_ARG + 3] << 24;


	/* set device to FFU mode */
            err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_MODE_CONFIG,
            	MMC_FFU_MODE_SET, card->ext_csd.generic_cmd6_time);
    
            if (err) {
            	pr_err("FFU: %s: error %d FFU is not supported\n",
            		mmc_hostname(card->host), err);
            	goto exit;
            }


             err = mmc_ffu_write(card, fw->data, arg, fw->size);
             if (err) {
                             pr_err("FFU: %s: write error %d\n",
                                             mmc_hostname(card->host), err);
                             goto exit;
             }

             err = mmc_ffu_install(card, ext_csd);
             if (err) {
                             pr_err("FFU: %s: error firmware install %d\n",
                                             mmc_hostname(card->host), err);
                             goto exit;
             }


             err = mmc_ffu_playload_checksum(card,fw->size);
             if (err) {
                             pr_err("FFU: %s: error firmware playload checksum %d\n",
                                             mmc_hostname(card->host), err);
                             goto exit;
             }

           	/* restart the eMMC */
           	err = mmc_ffu_restart(card);
           	if (err) {
          		pr_err("FFU: %s: error %d FFU restart:\n",
          			mmc_hostname(card->host), err);
            }

		   	/* jasond add start */
			/* check ffu status and fw_version after restart*/
		   	/* read ext_csd */
			err = mmc_send_ext_csd(card, ext_csd);
			if (err) {
				pr_err("FFU: %s: error %d sending ext_csd\n",
					mmc_hostname(card->host), err);
				goto exit;
			}
			/* return status */
			err = ext_csd[EXT_CSD_FFU_STATUS];
			if (err) {
				pr_err("FFU: %s: error %d check ffu status after restart:\n",
					mmc_hostname(card->host), err);
				err = -EINVAL;
				goto exit;
			}
		
			/* check fw version is correct or not*/
			ffu_fw_version_after_update = ext_csd[EXT_CSD_FW_VERSION] | 
											ext_csd[EXT_CSD_FW_VERSION + 1] << 8 |
											ext_csd[EXT_CSD_FW_VERSION + 2] << 16 |
											ext_csd[EXT_CSD_FW_VERSION + 3] << 24 |
											ext_csd[EXT_CSD_FW_VERSION + 4] << 32 |
											ext_csd[EXT_CSD_FW_VERSION + 5] << 40 |
											ext_csd[EXT_CSD_FW_VERSION + 6] << 48 |
											ext_csd[EXT_CSD_FW_VERSION + 7] << 56;
			if (ffu_fw_version_after_update != firmware_fixups[0].firmware_version) {
				printk("jasond add for fw version check error after fw update, now current=%lld\n!!", ffu_fw_version_after_update);
				goto exit;
			} else {
				printk("jasond add for fw version check ok after fw update!!");
			}
			
			/*jason add end*/
		
          printk("FFU: firmversion is : %llx",card->ext_csd.ffu_fw_version);
exit:
             if (err != 0 ) {
                /* host switch back to work in normal MMC
                 * Read/Write commands */
                  mmc_switch(card, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_MODE_CONFIG,
                    	MMC_FFU_MODE_NORMAL, card->ext_csd.generic_cmd6_time);
             }
             release_firmware(fw);
             mmc_release_host(card->host);

             return err;
}

int mmc_ffu_firmware_fixup (struct mmc_card *card){

            u64  firmware_version ;
            int  size,i;

            /* check if card is eMMC 5.0 or higher */
            if (card->ext_csd.rev < 7)
            	return -EINVAL;
            
             /* Check if FFU is supported */
            if (!FFU_SUPPORTED_MODE(card->ext_csd.ffu_capable) ||
            	FFU_CONFIG(card->ext_csd.ffu_fw_config)) {
            	 return -EINVAL;
            }
            
            size = sizeof(firmware_fixups)/sizeof(firmware_fixups[0]);

            firmware_version = card->ext_csd.ffu_fw_version;

             printk("FFU: fw_version_bytes : 0x%llx\n", card->ext_csd.ffu_fw_version);
             

            for(i = 0; i < size ; i++){
                 if(firmware_fixups[i].manfid == card->cid.manfid &&
                      !strcmp( firmware_fixups[i].prod_name,card->cid.prod_name) &&
                         firmware_fixups[i].firmware_version > card->ext_csd.ffu_fw_version) 
                           break;
             }         
            
            printk("FFU: Check the index is:%d the size is %d!!!\n", i,size);
             
            if( i  >= size) 
                return -EINVAL;
            
            return mmc_ffu_upgrade(card, firmware_fixups[i].firmware_path);
}

EXPORT_SYMBOL(mmc_ffu_firmware_fixup);
