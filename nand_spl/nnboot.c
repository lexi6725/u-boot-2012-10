/*
 * (C) Copyright 2006-2008
 * Stefan Roese, DENX Software Engineering, sr@denx.de.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <nand.h>
#include <asm/io.h>

void led1_on(void)
{
	volatile unsigned long *GPMDAT = (volatile signed long *)0x7f008824;
	*GPMDAT &= ~(1<<0);
}

void led2_on(void)
{
	volatile unsigned long *GPMDAT = (volatile signed long *)0x7f008824;
	*GPMDAT &= ~(1<<1);
}

void led3_on(void)
{
	volatile unsigned long *GPMDAT = (volatile signed long *)0x7f008824;
	*GPMDAT &= ~(1<<2);
}

void led4_on(void)
{
	volatile unsigned long *GPMDAT = (volatile signed long *)0x7f008824;
	*GPMDAT &= ~(1<<3);
}


#if 0
static int nand_ecc_pos[] = CONFIG_SYS_NAND_ECCPOS;

#define ECCSTEPS	(CONFIG_SYS_NAND_PAGE_SIZE / \
					CONFIG_SYS_NAND_ECCSIZE)
#define ECCTOTAL	(ECCSTEPS * CONFIG_SYS_NAND_ECCBYTES)


#if (CONFIG_SYS_NAND_PAGE_SIZE <= 512)
/*
 * NAND command for small page NAND devices (512)
 */
static int nand_command(struct mtd_info *mtd, int block, int page, int offs, u8 cmd)
{
	struct nand_chip *this = mtd->priv;
	int page_addr = page + block * CONFIG_SYS_NAND_PAGE_COUNT;

	while (!this->dev_ready(mtd))
		;

	/* Begin command latch cycle */
	this->cmd_ctrl(mtd, cmd, NAND_CTRL_CLE | NAND_CTRL_CHANGE);
	/* Set ALE and clear CLE to start address cycle */
	/* Column address */
	this->cmd_ctrl(mtd, offs, NAND_CTRL_ALE | NAND_CTRL_CHANGE);
	this->cmd_ctrl(mtd, page_addr & 0xff, NAND_CTRL_ALE); /* A[16:9] */
	this->cmd_ctrl(mtd, (page_addr >> 8) & 0xff,
		       NAND_CTRL_ALE); /* A[24:17] */
#ifdef CONFIG_SYS_NAND_4_ADDR_CYCLE
	/* One more address cycle for devices > 32MiB */
	this->cmd_ctrl(mtd, (page_addr >> 16) & 0x0f,
		       NAND_CTRL_ALE); /* A[28:25] */
#endif
	/* Latch in address */
	this->cmd_ctrl(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

	/*
	 * Wait a while for the data to be ready
	 */
	while (!this->dev_ready(mtd))
		;

	return 0;
}
#else
/*
 * NAND command for large page NAND devices (2k)
 */
static int nand_command(struct mtd_info *mtd, int block, int page, int offs, u8 cmd)
{
	struct nand_chip *this = mtd->priv;
	int page_addr = page + block * CONFIG_SYS_NAND_PAGE_COUNT;
	void (*hwctrl)(struct mtd_info *mtd, int cmd,
			unsigned int ctrl) = this->cmd_ctrl;

	while (!this->dev_ready(mtd))
		;

	/* Emulate NAND_CMD_READOOB */
	if (cmd == NAND_CMD_READOOB) {
		offs += CONFIG_SYS_NAND_PAGE_SIZE;
		cmd = NAND_CMD_READ0;
	}

	/* Shift the offset from byte addressing to word addressing. */
	if (this->options & NAND_BUSWIDTH_16)
		offs >>= 1;

	/* Begin command latch cycle */
	hwctrl(mtd, cmd, NAND_CTRL_CLE | NAND_CTRL_CHANGE);
	/* Set ALE and clear CLE to start address cycle */
	/* Column address */
	hwctrl(mtd, offs & 0xff,
		       NAND_CTRL_ALE | NAND_CTRL_CHANGE); /* A[7:0] */
	hwctrl(mtd, (offs >> 8) & 0xff, NAND_CTRL_ALE); /* A[11:9] */
	/* Row address */
	hwctrl(mtd, (page_addr & 0xff), NAND_CTRL_ALE); /* A[19:12] */
	hwctrl(mtd, ((page_addr >> 8) & 0xff),
		       NAND_CTRL_ALE); /* A[27:20] */
#ifdef CONFIG_SYS_NAND_5_ADDR_CYCLE
	/* One more address cycle for devices > 128MiB */
	hwctrl(mtd, (page_addr >> 16) & 0x0f,
		       NAND_CTRL_ALE); /* A[31:28] */
#endif
	/* Latch in address */
	hwctrl(mtd, NAND_CMD_READSTART,
		       NAND_CTRL_CLE | NAND_CTRL_CHANGE);
	hwctrl(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

	/*
	 * Wait a while for the data to be ready
	 */
	while (!this->dev_ready(mtd))
		;

	return 0;
}
#endif

static int nand_is_bad_block(struct mtd_info *mtd, int block)
{
	struct nand_chip *this = mtd->priv;

	nand_command(mtd, block, 0, CONFIG_SYS_NAND_BAD_BLOCK_POS, NAND_CMD_READOOB);

	/*
	 * Read one byte (or two if it's a 16 bit chip).
	 */
	if (this->options & NAND_BUSWIDTH_16) {
		if (readw(this->IO_ADDR_R) != 0xffff)
			return 1;
	} else {
		if (readb(this->IO_ADDR_R) != 0xff)
			return 1;
	}

	return 0;
}

#if defined(CONFIG_SYS_NAND_4BIT_HW_ECC_OOBFIRST)
static int nand_read_page(struct mtd_info *mtd, int block, int page, uchar *dst)
{
	struct nand_chip *this = mtd->priv;
	u_char ecc_calc[ECCTOTAL];
	u_char ecc_code[ECCTOTAL];
	u_char oob_data[CONFIG_SYS_NAND_OOBSIZE];
	int i;
	int eccsize = CONFIG_SYS_NAND_ECCSIZE;
	int eccbytes = CONFIG_SYS_NAND_ECCBYTES;
	int eccsteps = ECCSTEPS;
	uint8_t *p = dst;

	nand_command(mtd, block, page, 0, NAND_CMD_READOOB);
	this->read_buf(mtd, oob_data, CONFIG_SYS_NAND_OOBSIZE);
	nand_command(mtd, block, page, 0, NAND_CMD_READ0);

	/* Pick the ECC bytes out of the oob data */
	for (i = 0; i < ECCTOTAL; i++)
		ecc_code[i] = oob_data[nand_ecc_pos[i]];


	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		this->ecc.hwctl(mtd, NAND_ECC_READ);
		this->read_buf(mtd, p, eccsize);
		this->ecc.calculate(mtd, p, &ecc_calc[i]);
		this->ecc.correct(mtd, p, &ecc_code[i], &ecc_calc[i]);
	}

	return 0;
}
#else
static int nand_read_page(struct mtd_info *mtd, int block, int page, uchar *dst)
{
	struct nand_chip *this = mtd->priv;
	u_char ecc_calc[ECCTOTAL];
	u_char ecc_code[ECCTOTAL];
	u_char oob_data[CONFIG_SYS_NAND_OOBSIZE];
	int i;
	int eccsize = CONFIG_SYS_NAND_ECCSIZE;
	int eccbytes = CONFIG_SYS_NAND_ECCBYTES;
	int eccsteps = ECCSTEPS;
	uint8_t *p = dst;

	nand_command(mtd, block, page, 0, NAND_CMD_READ0);

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		this->ecc.hwctl(mtd, NAND_ECC_READ);
		this->read_buf(mtd, p, eccsize);
		this->ecc.calculate(mtd, p, &ecc_calc[i]);
	}
	this->read_buf(mtd, oob_data, CONFIG_SYS_NAND_OOBSIZE);

	/* Pick the ECC bytes out of the oob data */
	for (i = 0; i < ECCTOTAL; i++)
		ecc_code[i] = oob_data[nand_ecc_pos[i]];

	eccsteps = ECCSTEPS;
	p = dst;

	for (i = 0 ; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		/* No chance to do something with the possible error message
		 * from correct_data(). We just hope that all possible errors
		 * are corrected by this routine.
		 */
		this->ecc.correct(mtd, p, &ecc_code[i], &ecc_calc[i]);
	}

	return 0;
}
#endif /* #if defined(CONFIG_SYS_NAND_4BIT_HW_ECC_OOBFIRST) */

static int nand_load(struct mtd_info *mtd, unsigned int offs,
		     unsigned int uboot_size, uchar *dst)
{
	unsigned int block, lastblock;
	unsigned int page;

	/*
	 * offs has to be aligned to a page address!
	 */
	block = offs / CONFIG_SYS_NAND_BLOCK_SIZE;
	lastblock = (offs + uboot_size - 1) / CONFIG_SYS_NAND_BLOCK_SIZE;
	page = (offs % CONFIG_SYS_NAND_BLOCK_SIZE) / CONFIG_SYS_NAND_PAGE_SIZE;

	while (block <= lastblock) {
		if (!nand_is_bad_block(mtd, block)) {
			/*
			 * Skip bad blocks
			 */
			while (page < CONFIG_SYS_NAND_PAGE_COUNT) {
				nand_read_page(mtd, block, page, dst);
				dst += CONFIG_SYS_NAND_PAGE_SIZE;
				page++;
			}

			page = 0;
		} else {
			lastblock++;
		}

		block++;
	}

	return 0;
}

/*
 * The main entry for NAND booting. It's necessary that SDRAM is already
 * configured and available since this code loads the main U-Boot image
 * from NAND into SDRAM and starts it from there.
 */
void nand_boot(void)
{
	struct nand_chip nand_chip;
	nand_info_t nand_info;
	//__attribute__((noreturn)) void (*uboot)(void);
    //led test
    writel(0x0023,0x7f008824);
	/*
	 * Init board specific nand support
	 */
	nand_chip.select_chip = NULL;
	nand_info.priv = &nand_chip;
	nand_chip.IO_ADDR_R = nand_chip.IO_ADDR_W = (void  __iomem *)CONFIG_SYS_NAND_BASE;
	nand_chip.dev_ready = NULL;	/* preset to NULL */
	nand_chip.options = 0;
	board_nand_init(&nand_chip);

	if (nand_chip.select_chip)
		nand_chip.select_chip(&nand_info, 0);

	/*
	 * Load U-Boot image from NAND into RAM
	 */
	nand_load(&nand_info, CONFIG_SYS_NAND_U_BOOT_OFFS, CONFIG_SYS_NAND_U_BOOT_SIZE,
		  (uchar *)CONFIG_SYS_NAND_U_BOOT_DST);

#ifdef CONFIG_NAND_ENV_DST
	nand_load(&nand_info, CONFIG_ENV_OFFSET, CONFIG_ENV_SIZE,
		  (uchar *)CONFIG_NAND_ENV_DST);

#ifdef CONFIG_ENV_OFFSET_REDUND
	nand_load(&nand_info, CONFIG_ENV_OFFSET_REDUND, CONFIG_ENV_SIZE,
		  (uchar *)CONFIG_NAND_ENV_DST + CONFIG_ENV_SIZE);
#endif
#endif

	if (nand_chip.select_chip)
		nand_chip.select_chip(&nand_info, -1);
    //led test
    //writel(0x002a,0x7f008824);
	/*
	 * Jump to U-Boot image
	 */
	//uboot = (void *)CONFIG_SYS_NAND_U_BOOT_START;
	//(*uboot)();
}
#else//if 0

//#include <regs.h>
#include <asm/arch/s3c6410.h>
/*
 * address format
 *              17 16         9 8            0
 * --------------------------------------------
 * | block(12bit) | page(5bit) | offset(9bit) |
 * --------------------------------------------
 */
#define NAND_DISABLE_CE()	(NFCONT_REG |= (1 << 1))
#define NAND_ENABLE_CE()	(NFCONT_REG &= ~(1 << 1))
#define NF_TRANSRnB()		do { while(!(NFSTAT_REG & (1 << 0))); } while(0)

static void nandll_read_page (uchar *buf, ulong addr, uint nand_page_size)
{
    int i;
	/*
    int page_size = 512;

    if (large_block==1)
        page_size = 2048;
    if (large_block==2)
        page_size = 4096;
    if(large_block==3)
        page_size = 8192;
    */
    NAND_ENABLE_CE();
    NFCMD_REG = NAND_CMD_READ0;

    /* Write Address */
    NFADDR_REG = 0;

    if (nand_page_size>=2048)
        NFADDR_REG = 0;

    NFADDR_REG = (addr) & 0xff;
    NFADDR_REG = (addr >> 8) & 0xff;
    NFADDR_REG = (addr >> 16) & 0xff;

    if (nand_page_size>=2048)
        NFCMD_REG = NAND_CMD_READSTART;

    NF_TRANSRnB();

    /* for compatibility(2460). u32 cannot be used. by scsuh */
    for(i=0; i < nand_page_size; i++) {
        *buf++ = NFDATA8_REG;
    }
    NAND_DISABLE_CE();
    //return 0;
}

/*
 * Read data from NAND.
 */
static void nandll_read_blocks (ulong dst_addr, ulong read_size, uint nand_page_size)
{
    uchar *buf = (uchar *)dst_addr;
    int i;
    uint page_shift = 9;
    /*
    if (large_block==1)
        page_shift = 11;

    if(large_block==2)
        page_shift = 12;

    if(large_block==3)
        page_shift =13;
     */
    if(nand_page_size == 4096) 
    {
		led2_on();
        page_shift = 12;//4k page
        /* Read pages */
        for (i = 2; i < 4; i++, buf+=(1<<(page_shift-1))) {
            nandll_read_page(buf, i, nand_page_size);
        }

        /* Read pages */
        for (i = 4; i < (read_size>>page_shift); i++, buf+=(1<<page_shift)) {
            nandll_read_page(buf, i, nand_page_size);
        }

    }else if(nand_page_size == 8192)  //K9GAG08U0E
    {
        page_shift = 13;//8k page
		/* Read pages */
        for (i = 0; i < 4; i++, buf+=(1<<(page_shift-2))) {
            nandll_read_page(buf, i, nand_page_size);
        }


        /* Read pages */
        for (i = 4; i < (read_size>>page_shift); i++, buf+=(1<<page_shift)) {
            nandll_read_page(buf, i, nand_page_size);
        }
    }
    else //if (nand_page_size==2048)
    {
        page_shift = 11; //2k page
        for (i = 0; i < (read_size>>page_shift); i++, buf+=(1<<page_shift)) {
            nandll_read_page(buf, i, nand_page_size);
        }
    }
    //return 0;
	led3_on();
}

void nand_boot(void)
{
    int large_block = 0;
    int i;
    vu_char id;
    __attribute__((noreturn)) void (*uboot)(void);

    NAND_ENABLE_CE();
    NFCMD_REG=NAND_CMD_RESET;
    NF_TRANSRnB();


    NFCMD_REG = NAND_CMD_READID;
    NFADDR_REG =  0x00;

    NF_TRANSRnB();
    
    /* wait for a while */
    for (i=0; i<200; i++);

    int factory = NFDATA8_REG;
    id = NFDATA8_REG;

    int cellinfo=NFDATA8_REG;
    int tmp= NFDATA8_REG;

    //int childType=tmp & 0x03; //Page size
    int childType=cellinfo; //Page size

    if (id > 0x80)
    {
        large_block = 1;
    }

    if(id == 0xd5 && childType==0x94 )//K9GAG08U0D
    {
        large_block = 2;

    }
    if(id == 0xd5 && childType==0x14 )//K9GAG08U0M
    {
        large_block = 2;

    }
    if(id == 0xd5 && childType==0x84 )//K9GAG08U0E
    {
        large_block = 3;

    }
    if(id==0xd7)//K9LBG08U0D
    {
        large_block = 2;
    }
    if(factory==0x2c && id == 0x48) //MT29F16G08ABACAWP
    {
        large_block = 2;

    }if(factory==0x2c && id == 0x38) //MT29F8G08ABABAWP
    {
        large_block = 2;

    }
	
	led1_on();
    /* read NAND Block.
     * 128KB ->240KB because of U-Boot size increase. by scsuh
     * So, read 0x3c000 bytes not 0x20000(128KB).
     */
    nandll_read_blocks(CONFIG_SYS_PHY_UBOOT_BASE, 0x80000, CONFIG_SYS_NAND_PAGE_SIZE);
	led4_on();
    uboot = (void *)CONFIG_SYS_PHY_UBOOT_BASE;
	(*uboot)();
}
#endif

