/*
 * Copyright 2014 Ivenix Corp.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/processor.h>
#include <ioports.h>
#include <lmb.h>
#include <asm/io.h>
#include <asm/mp.h>

DECLARE_GLOBAL_DATA_PTR;

#define SCR_CORE0_EN  21
#define SCR_CORE0_RST 13

int cpu_reset(int nr)
{
	struct src *src_regs = (struct src *)SRC_BASE_ADDR;
	u32 bit;

	switch (nr) {
	case 0:
	case 1:
	case 2:
	case 3:
		bit = (1 << SCR_CORE0_RST) << nr;
		setbits_le32(&src_regs->scr, bit);
		break;
	}

	return 0;
}

int cpu_status(int nr)
{
	struct src *src_regs = (struct src *)SRC_BASE_ADDR;
	u32 scr;
	int enabled = 1;

 	scr = readl(&src_regs->scr);

	switch (nr) {
	case 1:
	case 2:
	case 3:
		enabled = scr & (( 1 << SCR_CORE0_EN ) << nr);
		break;
	}

	printf("cpu %d: %sabled\n", nr, enabled ? "En" : "Dis");

	return 0;
}

static int cpu_enable(int nr, int enable)
{
	struct src *src_regs = (struct src *)SRC_BASE_ADDR;
	u32 bit;

	bit = (1 << SCR_CORE0_EN) << nr;

	switch (nr) {
	case 1:
	case 2:
	case 3:
		if (enable)
			setbits_le32(&src_regs->scr, bit);
		else
			clrbits_le32(&src_regs->scr, bit);
		break;
	default:
		printf("Invalid cpu number %d\n", nr);
		return 1;
	}

	return 0;
}

int cpu_disable(int nr)
{
	cpu_enable(nr, 0);
	return 0;
}

#if  0
int is_core_disabled(int nr) 
{
	ccsr_gur_t *gur = (void *)(CONFIG_SYS_MPC85xx_GUTS_ADDR);
	u32 devdisr = in_be32(&gur->devdisr);

	switch (nr) {
	case 0:
		return (devdisr & MPC85xx_DEVDISR_CPU0);
	case 1:
		return (devdisr & MPC85xx_DEVDISR_CPU1);
	default:
		printf("Invalid cpu number for disable %d\n", nr);
	}

	return 0;
}
#endif

#if 0
static u8 boot_entry_map[4] = {
	0,
	BOOT_ENTRY_PIR,
	BOOT_ENTRY_R3_LOWER,
};
#endif

int cpu_release(int nr, int argc, char * const argv[])
{
	struct src *src_regs = (struct src *)SRC_BASE_ADDR;
	u64 boot_addr, arg;

	boot_addr = simple_strtoull(argv[0], NULL, 16);

	if (argc > 0)
		arg = simple_strtoull(argv[1], NULL, 16);
	else
		arg = 0;

	switch (nr) {
	case 0:
		writel(boot_addr, &src_regs->gpr1);
		writel(arg, &src_regs->gpr2);
		break;
	case 1:
		writel(boot_addr, &src_regs->gpr3);
		writel(arg, &src_regs->gpr4);
		break;
	case 2:
		writel(boot_addr, &src_regs->gpr5);
		writel(arg, &src_regs->gpr6);
		break;
	case 3:
		writel(boot_addr, &src_regs->gpr7);
		writel(arg, &src_regs->gpr8);
		break;
	}

	cpu_enable(nr, 1);

#if 0
	u32 i, val, *table = (u32 *)&__spin_table + nr * NUM_BOOT_ENTRY;

	if (hold_cores_in_reset(1))
		return 0;

	if (nr == get_my_id()) {
		printf("Invalid to release the boot core.\n\n");
		return 1;
	}

	if (argc != 4) {
		printf("Invalid number of arguments to release.\n\n");
		return 1;
	}


	/* handle pir, r3 */
	for (i = 1; i < 3; i++) {
		if (argv[i][0] != '-') {
			u8 entry = boot_entry_map[i];
			val = simple_strtoul(argv[i], NULL, 16);
			table[entry] = val;
		}
	}

	table[BOOT_ENTRY_ADDR_UPPER] = (u32)(boot_addr >> 32);

	/* ensure all table updates complete before final address write */
	eieio();

	table[BOOT_ENTRY_ADDR_LOWER] = (u32)(boot_addr & 0xffffffff);
#endif

	return 0;
}

u32 determine_mp_bootpg(unsigned int *pagesize)
{
	bd_t *bd = gd->bd;

	u32 bootpg = 0;

	bootpg = bd->bi_memsize + bd->bi_memstart - 4096;
	if (pagesize)
		*pagesize = 4096;

	return bootpg;
}

#if 0
phys_addr_t get_spin_phys_addr(void)
{
	return virt_to_phys(&__spin_table);
}
#endif

#if 0
static void plat_mp_up(unsigned long bootpg, unsigned int pagesize)
{
	u32 up, cpu_up_mask, whoami;
	u32 *table = (u32 *)&__spin_table;
	volatile u32 bpcr;
	volatile ccsr_local_ecm_t *ecm = (void *)(CONFIG_SYS_MPC85xx_ECM_ADDR);
	volatile ccsr_gur_t *gur = (void *)(CONFIG_SYS_MPC85xx_GUTS_ADDR);
	volatile ccsr_pic_t *pic = (void *)(CONFIG_SYS_MPC8xxx_PIC_ADDR);
	u32 devdisr;
	int timeout = 10;

	whoami = in_be32(&pic->whoami);
	out_be32(&ecm->bptr, 0x80000000 | (bootpg >> 12));

	/* disable time base at the platform */
	devdisr = in_be32(&gur->devdisr);
	if (whoami)
		devdisr |= MPC85xx_DEVDISR_TB0;
	else
		devdisr |= MPC85xx_DEVDISR_TB1;
	out_be32(&gur->devdisr, devdisr);

	/* release the hounds */
	up = ((1 << cpu_numcores()) - 1);
	bpcr = in_be32(&ecm->eebpcr);
	bpcr |= (up << 24);
	out_be32(&ecm->eebpcr, bpcr);
	asm("sync; isync; msync");

	cpu_up_mask = 1 << whoami;
	/* wait for everyone */
	while (timeout) {
		int i;
		for (i = 0; i < cpu_numcores(); i++) {
			if (table[i * NUM_BOOT_ENTRY + BOOT_ENTRY_ADDR_LOWER])
				cpu_up_mask |= (1 << i);
		};

		if ((cpu_up_mask & up) == up)
			break;

		udelay(100);
		timeout--;
	}

	if (timeout == 0)
		printf("CPU up timeout. CPU up mask is %x should be %x\n",
			cpu_up_mask, up);

	/* enable time base at the platform */
	if (whoami)
		devdisr |= MPC85xx_DEVDISR_TB1;
	else
		devdisr |= MPC85xx_DEVDISR_TB0;
	out_be32(&gur->devdisr, devdisr);

	/* readback to sync write */
	in_be32(&gur->devdisr);

	mtspr(SPRN_TBWU, 0);
	mtspr(SPRN_TBWL, 0);

	devdisr &= ~(MPC85xx_DEVDISR_TB0 | MPC85xx_DEVDISR_TB1);
	out_be32(&gur->devdisr, devdisr);

#ifdef CONFIG_MPC8xxx_DISABLE_BPTR
	/*
	 * Disabling Boot Page Translation allows the memory region 0xfffff000
	 * to 0xffffffff to be used normally.  Leaving Boot Page Translation
	 * enabled remaps 0xfffff000 to SDRAM which makes that memory region
	 * unusable for normal operation but it does allow OSes to easily
	 * reset a processor core to put it back into U-Boot's spinloop.
	 */
	clrbits_be32(&ecm->bptr, 0x80000000);
#endif
}
#endif

void cpu_mp_lmb_reserve(struct lmb *lmb)
{
	u32 bootpg = determine_mp_bootpg(NULL);

	lmb_reserve(lmb, bootpg, 4096);
}

#if 0
void setup_mp(void)
{
	extern u32 __secondary_start_page;
	extern u32 __bootpg_addr, __spin_table_addr, __second_half_boot_page;

	int i;
	ulong fixup = (u32)&__secondary_start_page;
	u32 bootpg, bootpg_map, pagesize;

	bootpg = determine_mp_bootpg(&pagesize);

	/*
	 * pagesize is only 4K or 8K
	 * we only use the last 4K of boot page
	 * bootpg_map saves the address for the boot page
	 * 8K is used for the workaround of 3-way DDR interleaving
	 */

	bootpg_map = bootpg;

	if (pagesize == 8192)
		bootpg += 4096;	/* use 2nd half */

	/* Some OSes expect secondary cores to be held in reset */
	if (hold_cores_in_reset(0))
		return;

	/*
	 * Store the bootpg's cache-able half address for use by secondary
	 * CPU cores to continue to boot
	 */
	__bootpg_addr = (u32)virt_to_phys(&__second_half_boot_page);

	/* Store spin table's physical address for use by secondary cores */
	__spin_table_addr = (u32)get_spin_phys_addr();

	/* flush bootpg it before copying invalidate any staled cacheline */
	flush_cache(bootpg, 4096);

	/* look for the tlb covering the reset page, there better be one */
	i = find_tlb_idx((void *)CONFIG_BPTR_VIRT_ADDR, 1);

	/* we found a match */
	if (i != -1) {
		/* map reset page to bootpg so we can copy code there */
		disable_tlb(i);

		set_tlb(1, CONFIG_BPTR_VIRT_ADDR, bootpg, /* tlb, epn, rpn */
			MAS3_SX|MAS3_SW|MAS3_SR, MAS2_I|MAS2_G, /* perms, wimge */
			0, i, BOOKE_PAGESZ_4K, 1); /* ts, esel, tsize, iprot */

		memcpy((void *)CONFIG_BPTR_VIRT_ADDR, (void *)fixup, 4096);

		plat_mp_up(bootpg_map, pagesize);
	} else {
		puts("WARNING: No reset page TLB. "
			"Skipping secondary core setup\n");
	}
}
#endif
