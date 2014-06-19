/*
 * Copyright 2013 Red Hat Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors: Ben Skeggs
 */

#include <subdev/bios.h>
#include <subdev/bios/bit.h>
#include <subdev/bios/pll.h>
#include <subdev/bios/perf.h>
#include <subdev/bios/timing.h>
#include <subdev/clock/pll.h>
#include <subdev/fb.h>

#include <core/option.h>
#include <core/mm.h>

#include "ramseq.h"

#include "nv50.h"

struct nv50_ramseq {
	struct hwsq base;
	struct hwsq_reg r_0x002504;
	struct hwsq_reg r_0x004008;
	struct hwsq_reg r_0x00400c;
	struct hwsq_reg r_0x00c040;
	struct hwsq_reg r_0x100210;
	struct hwsq_reg r_0x1002d0;
	struct hwsq_reg r_0x1002d4;
	struct hwsq_reg r_0x1002dc;
	struct hwsq_reg r_0x100da0[8];
	struct hwsq_reg r_0x100e20;
	struct hwsq_reg r_0x100e24;
	struct hwsq_reg r_0x611200;
	struct hwsq_reg r_timing[9];
	struct hwsq_reg r_mr[4];
};

struct nv50_ram {
	struct nouveau_ram base;
	struct nv50_ramseq hwsq;
};

static int
nv50_mem_timing_calc(struct drm_device *dev, u32 freq,
		     struct nouveau_pm_tbl_entry *e, u8 len,
		     struct nouveau_pm_memtiming *boot,
		     struct nouveau_pm_memtiming *t)
{
	struct nouveau_device *device = nouveau_dev(dev);
	struct nouveau_fb *pfb = nouveau_fb(device);
	struct nouveau_drm *drm = nouveau_drm(dev);
	struct bit_entry P;
	uint8_t unk18 = 1, unk20 = 0, unk21 = 0, tmp7_3;

	if (bit_table(dev, 'P', &P))
		return -EINVAL;

	switch (min(len, (u8) 22)) {
	case 22:
		unk21 = e->tUNK_21;
	case 21:
		unk20 = e->tUNK_20;
	case 20:
		if (e->tCWL > 0)
			t->tCWL = e->tCWL;
	case 19:
		unk18 = e->tUNK_18;
		break;
	}

	t->reg[0] = (e->tRP << 24 | e->tRAS << 16 | e->tRFC << 8 | e->tRC);

	t->reg[1] = (e->tWR + 2 + (t->tCWL - 1)) << 24 |
				max(unk18, (u8) 1) << 16 |
				(e->tWTR + 2 + (t->tCWL - 1)) << 8;

	t->reg[2] = ((t->tCWL - 1) << 24 |
		    e->tRRD << 16 |
		    e->tRCDWR << 8 |
		    e->tRCDRD);

	t->reg[4] = e->tUNK_13 << 8  | e->tUNK_13;

	t->reg[5] = (e->tRFC << 24 | max(e->tRCDRD, e->tRCDWR) << 16 | e->tRP);

	t->reg[8] = boot->reg[8] & 0xffffff00;

		t->reg[1] |= (e->tCL + 2 - (t->tCWL - 1));

		t->reg[3] = (0x14 + e->tCL) << 24 |
			    0x16 << 16 |
			    (e->tCL - 1) << 8 |
			    (e->tCL - 1);

		t->reg[4] |= boot->reg[4] & 0xffff0000;

		t->reg[6] = (0x33 - t->tCWL) << 16 |
			    t->tCWL << 8 |
			    (0x2e + e->tCL - t->tCWL);

		t->reg[7] = 0x4000202 | (e->tCL - 1) << 16;

		/* XXX: P.version == 1 only has DDR2 and GDDR3? */
		if (pfb->ram->type == NV_MEM_TYPE_DDR2) {
			t->reg[5] |= (e->tCL + 3) << 8;
			t->reg[6] |= (t->tCWL - 2) << 8;
			t->reg[8] |= (e->tCL - 4);
		} else {
			t->reg[5] |= (e->tCL + 2) << 8;
			t->reg[6] |= t->tCWL << 8;
			t->reg[8] |= (e->tCL - 2);
		}

	NV_DEBUG(drm, "Entry %d: 220: %08x %08x %08x %08x\n", t->id,
		 t->reg[0], t->reg[1], t->reg[2], t->reg[3]);
	NV_DEBUG(drm, "         230: %08x %08x %08x %08x\n",
		 t->reg[4], t->reg[5], t->reg[6], t->reg[7]);
	NV_DEBUG(drm, "         240: %08x\n", t->reg[8]);
	return 0;
}

/**
 * MR generation methods
 */

static int
nouveau_mem_ddr2_mr(struct drm_device *dev, u32 freq,
		    struct nouveau_pm_tbl_entry *e, u8 len,
		    struct nouveau_pm_memtiming *boot,
		    struct nouveau_pm_memtiming *t)
{
	struct nouveau_drm *drm = nouveau_drm(dev);

	t->drive_strength = 0;
	if (len < 15) {
		t->odt = boot->odt;
	} else {
		t->odt = e->RAM_FT1 & 0x07;
	}

	if (e->tCL >= NV_MEM_CL_DDR2_MAX) {
		NV_WARN(drm, "(%u) Invalid tCL: %u", t->id, e->tCL);
		return -ERANGE;
	}

	if (e->tWR >= NV_MEM_WR_DDR2_MAX) {
		NV_WARN(drm, "(%u) Invalid tWR: %u", t->id, e->tWR);
		return -ERANGE;
	}

	if (t->odt > 3) {
		NV_WARN(drm, "(%u) Invalid odt value, assuming disabled: %x",
			t->id, t->odt);
		t->odt = 0;
	}

	t->mr[0] = (boot->mr[0] & 0x100f) |
		   (e->tCL) << 4 |
		   (e->tWR - 1) << 9;
	t->mr[1] = (boot->mr[1] & 0x101fbb) |
		   (t->odt & 0x1) << 2 |
		   (t->odt & 0x2) << 5;

	NV_DEBUG(drm, "(%u) MR: %08x", t->id, t->mr[0]);
	return 0;
}

static const uint8_t nv_mem_wr_lut_ddr3[NV_MEM_WR_DDR3_MAX] = {
	0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 5, 6, 6, 7, 7, 0, 0};

static int
nouveau_mem_ddr3_mr(struct drm_device *dev, u32 freq,
		    struct nouveau_pm_tbl_entry *e, u8 len,
		    struct nouveau_pm_memtiming *boot,
		    struct nouveau_pm_memtiming *t)
{
	struct nouveau_drm *drm = nouveau_drm(dev);
	u8 cl = e->tCL - 4;

	t->drive_strength = 0;
	if (len < 15) {
		t->odt = boot->odt;
	} else {
		t->odt = e->RAM_FT1 & 0x07;
	}

	if (e->tCL >= NV_MEM_CL_DDR3_MAX || e->tCL < 4) {
		NV_WARN(drm, "(%u) Invalid tCL: %u", t->id, e->tCL);
		return -ERANGE;
	}

	if (e->tWR >= NV_MEM_WR_DDR3_MAX || e->tWR < 4) {
		NV_WARN(drm, "(%u) Invalid tWR: %u", t->id, e->tWR);
		return -ERANGE;
	}

	if (e->tCWL < 5) {
		NV_WARN(drm, "(%u) Invalid tCWL: %u", t->id, e->tCWL);
		return -ERANGE;
	}

	t->mr[0] = (boot->mr[0] & 0x180b) |
		   /* CAS */
		   (cl & 0x7) << 4 |
		   (cl & 0x8) >> 1 |
		   (nv_mem_wr_lut_ddr3[e->tWR]) << 9;
	t->mr[1] = (boot->mr[1] & 0x101dbb) |
		   (t->odt & 0x1) << 2 |
		   (t->odt & 0x2) << 5 |
		   (t->odt & 0x4) << 7;
	t->mr[2] = (boot->mr[2] & 0x20ffb7) | (e->tCWL - 5) << 3;

	NV_DEBUG(drm, "(%u) MR: %08x %08x", t->id, t->mr[0], t->mr[2]);
	return 0;
}

static const uint8_t nv_mem_cl_lut_gddr3[NV_MEM_CL_GDDR3_MAX] = {
	0, 0, 0, 0, 4, 5, 6, 7, 0, 1, 2, 3, 8, 9, 10, 11};
static const uint8_t nv_mem_wr_lut_gddr3[NV_MEM_WR_GDDR3_MAX] = {
	0, 0, 0, 0, 0, 2, 3, 8, 9, 10, 11, 0, 0, 1, 1, 0, 3};

static int
nouveau_mem_gddr3_mr(struct drm_device *dev, u32 freq,
		     struct nouveau_pm_tbl_entry *e, u8 len,
		     struct nouveau_pm_memtiming *boot,
		     struct nouveau_pm_memtiming *t)
{
	struct nouveau_drm *drm = nouveau_drm(dev);

	if (len < 15) {
		t->drive_strength = boot->drive_strength;
		t->odt = boot->odt;
	} else {
		t->drive_strength = (e->RAM_FT1 & 0x30) >> 4;
		t->odt = e->RAM_FT1 & 0x07;
	}

	if (e->tCL >= NV_MEM_CL_GDDR3_MAX) {
		NV_WARN(drm, "(%u) Invalid tCL: %u", t->id, e->tCL);
		return -ERANGE;
	}

	if (e->tWR >= NV_MEM_WR_GDDR3_MAX) {
		NV_WARN(drm, "(%u) Invalid tWR: %u", t->id, e->tWR);
		return -ERANGE;
	}

	if (t->odt > 3) {
		NV_WARN(drm, "(%u) Invalid odt value, assuming autocal: %x",
			t->id, t->odt);
		t->odt = 0;
	}

	t->mr[0] = (boot->mr[0] & 0xe0b) |
		   /* CAS */
		   ((nv_mem_cl_lut_gddr3[e->tCL] & 0x7) << 4) |
		   ((nv_mem_cl_lut_gddr3[e->tCL] & 0x8) >> 2);
	t->mr[1] = (boot->mr[1] & 0x100f40) | t->drive_strength |
		   (t->odt << 2) |
		   (nv_mem_wr_lut_gddr3[e->tWR] & 0xf) << 4;
	t->mr[2] = boot->mr[2];

	NV_DEBUG(drm, "(%u) MR: %08x %08x %08x", t->id,
		      t->mr[0], t->mr[1], t->mr[2]);
	return 0;
}


static int
nv50_ram_calc(struct nouveau_fb *pfb, u32 freq)
{
	struct nouveau_bios *bios = nouveau_bios(pfb);
	struct nv50_ram *ram = (void *)pfb->ram;
	struct nv50_ramseq *hwsq = &ram->hwsq;
	struct nvbios_perfE perfE;
	struct nvbios_pll mpll;
	struct {
		u32 data;
		u8  size;
	} ramcfg, timing;
	u8  ver, hdr, cnt, len, strap;
	int N1, M1, N2, M2, P;
	int ret, i;

	/* lookup closest matching performance table entry for frequency */
	i = 0;
	do {
		ramcfg.data = nvbios_perfEp(bios, i++, &ver, &hdr, &cnt,
					   &ramcfg.size, &perfE);
		if (!ramcfg.data || (ver < 0x25 || ver >= 0x40) ||
		    (ramcfg.size < 2)) {
			nv_error(pfb, "invalid/missing perftab entry\n");
			return -EINVAL;
		}
	} while (perfE.memory < freq);

	/* locate specific data set for the attached memory */
	strap = nvbios_ramcfg_index(nv_subdev(pfb));
	if (strap >= cnt) {
		nv_error(pfb, "invalid ramcfg strap\n");
		return -EINVAL;
	}

	ramcfg.data += hdr + (strap * ramcfg.size);

	/* lookup memory timings, if bios says they're present */
	strap = nv_ro08(bios, ramcfg.data + 0x01);
	if (strap != 0xff) {
		timing.data = nvbios_timingEe(bios, strap, &ver, &hdr,
					     &cnt, &len);
		if (!timing.data || ver != 0x10 || hdr < 0x12) {
			nv_error(pfb, "invalid/missing timing entry "
				 "%02x %04x %02x %02x\n",
				 strap, timing.data, ver, hdr);
			return -EINVAL;
		}
	} else {
		timing.data = 0;
	}

	ret = ram_init(hwsq, nv_subdev(pfb));
	if (ret)
		return ret;

	ram_wait(hwsq, 0x01, 0x00); /* wait for !vblank */
	ram_wait(hwsq, 0x01, 0x01); /* wait for vblank */
	ram_wr32(hwsq, 0x611200, 0x00003300);
	ram_wr32(hwsq, 0x002504, 0x00000001); /* block fifo */
	ram_nsec(hwsq, 12000);
	ram_setf(hwsq, 0x10, 0x00); /* disable fb */
	ram_wait(hwsq, 0x00, 0x01); /* wait for fb disabled */
	ram_nsec(hwsq, 2000);

	ram_wr32(hwsq, 0x1002d4, 0x00000001); /* precharge */
	ram_wr32(hwsq, 0x1002d0, 0x00000001); /* refresh */
	ram_wr32(hwsq, 0x1002d0, 0x00000001); /* refresh */
	ram_wr32(hwsq, 0x100210, 0x00000000); /* disable auto-refresh */
	ram_wr32(hwsq, 0x1002dc, 0x00000001); /* enable self-refresh */

	ret = nvbios_pll_parse(bios, 0x004008, &mpll);
	mpll.vco2.max_freq = 0;
	if (ret == 0) {
		ret = nv04_pll_calc(nv_subdev(pfb), &mpll, freq,
				   &N1, &M1, &N2, &M2, &P);
		if (ret == 0)
			ret = -EINVAL;
	}

	if (ret < 0)
		return ret;

	ram_mask(hwsq, 0x00c040, 0xc000c000, 0x0000c000);
	ram_nuke(hwsq, 0x004008);
	/* g86 |= 8200 */
	ram_mask(hwsq, 0x004008, 0x00000200, 0x00000200);
	ram_mask(hwsq, 0x00400c, 0x0000ffff, (N1 << 8) | M1); /* XXX */
	/* g86: 200 bit not cleared */
	ram_mask(hwsq, 0x004008, 0x81ff0200, 0x80000000 | (mpll.bias_p << 19) |

					     (P << 22) | (P << 16));
	/* XXX # of partitions, not hardcoded to 8 */
	/* XXX need to toggle the 0x10 bit either on or off */
	/* xxx did this replace the 10053c manipulation? */
	/* xxx or does one use 10053c for pre-g92? or for ddr vs gddr? */
	for (i = 0; i < ram->parts; i++)
		ram_mask(hwsq, 0x100da0[i], 0x00000010, 0x00000000); /*XXX*/
	ram_nsec(hwsq, 96000); /*XXX*/

	/* xxx note: g94 with 4 da0 writes + 64000 wait */
	/* xxx note: g92 with 4 da0 writes + no wait */
	/* xxx note: g200 with 8 da0 writes + 96000 wait */

.	ram_mask(hwsq, 0x004008, 0x00002200, 0x00002000);

	ram_wr32(hwsq, 0x1002dc, 0x00000000); /* disable self-refresh */
	ram_wr32(hwsq, 0x100210, 0x80000000); /* enable auto-refresh */

	ram_nsec(hwsq, 12000);

/*

	ramcfg = nouveau_perf_ramcfg(dev, freq, &ver, &len);
	if (ramcfg) {
		int dll_off;

		if (ver == 0x00)
			dll_off = !!(ramcfg[3] & 0x04);
		else
			dll_off = !!(ramcfg[2] & 0x40);

		switch (pfb->ram->type) {
		case NV_MEM_TYPE_GDDR3:
			t->mr[1] &= ~0x00000040;
			t->mr[1] |=  0x00000040 * dll_off;
			break;
		default:
			t->mr[1] &= ~0x00000001;
			t->mr[1] |=  0x00000001 * dll_off;
			break;
		}
	}

	t->odt = 0;
	t->drive_strength = 0;

	switch (pfb->ram->type) {
	case NV_MEM_TYPE_DDR3:
		t->odt |= (t->mr[1] & 0x200) >> 7;
	case NV_MEM_TYPE_DDR2:
		t->odt |= (t->mr[1] & 0x04) >> 2 |
			  (t->mr[1] & 0x40) >> 5;
		break;
	case NV_MEM_TYPE_GDDR3:
	case NV_MEM_TYPE_GDDR5:
		t->drive_strength = t->mr[1] & 0x03;
		t->odt = (t->mr[1] & 0x0c) >> 2;
		break;
	default:
		break;
	}

	switch (pfb->ram->type) {
	case NV_MEM_TYPE_DDR2:
		tDLLK = 2000;
		mr1_dlloff = 0x00000001;
		break;
	case NV_MEM_TYPE_DDR3:
		tDLLK = 12000;
		tCKSRE = 2000;
		tXS = 1000;
		mr1_dlloff = 0x00000001;
		break;
	case NV_MEM_TYPE_GDDR3:
		tDLLK = 40000;
		mr1_dlloff = 0x00000040;
		break;
	default:
		NV_ERROR(drm, "cannot reclock unsupported memtype\n");
		return -ENODEV;
	}
*/

	switch (ram->base.type) {
	case NV_MEM_TYPE_DDR2:
		ram_nuke(hwsq, mr[0]); /* force update */
		ram_mask(hwsq, mr[0], 0x000, 0x000);
		break;
	case NV_MEM_TYPE_GDDR3:
		ram_nuke(hwsq, mr[1]);
		ram_mask(hwsq, mr[1], 0x000, 0x000); /* XXX 1002b8 -> 100228 */
		ram_nuke(hwsq, mr[0]); /* force update */
		ram_mask(hwsq, mr[0], 0x000, 0x000); /* XXX 222 -> 252 */
		break;
	default:
		break;
	}

	ram_mask(hwsq, timing[3], 0x00000000, 0x00000000); /*XXX*/
	ram_mask(hwsq, timing[1], 0x00000000, 0x00000000); /*XXX*/
	ram_mask(hwsq, timing[6], 0x00000000, 0x00000000); /*XXX*/
	ram_mask(hwsq, timing[7], 0x00000000, 0x00000000); /*XXX*/
	ram_mask(hwsq, timing[8], 0x00000000, 0x00000000); /*XXX*/
	ram_mask(hwsq, timing[2], 0x00000000, 0x00000000); /*XXX*/
	ram_mask(hwsq, timing[4], 0x00000000, 0x00000000); /*XXX*/
	ram_mask(hwsq, timing[5], 0x00000000, 0x00000000); /*XXX*/
	ram_mask(hwsq, timing[0], 0x00000000, 0x00000000); /*XXX*/

	if (pfb->device->chipset == 0xa0) { /* XXX perhaps ranks related? */
		ram_nuke(hwsq, 0x100e24);
		ram_mask(hwsq, 0x100e24, 0x00000000, 0x00000000);
		ram_nuke(hwsq, 0x100e20);
		ram_mask(hwsq, 0x100e20, 0x00000000, 0x00000000);
	}

	ram_mask(hwsq, mr[0], 0x100, 0x100); /* XXX 352 */
	ram_mask(hwsq, mr[0], 0x100, 0x000); /* XXX 252 */

	ram_nsec(hwsq, 48000);

	ram_setf(hwsq, 0x10, 0x01); /* enable fb */
	ram_wait(hwsq, 0x00, 0x00); /* wait for fb enabled */
	ram_wr32(hwsq, 0x611200, 0x00003330);
	ram_wr32(hwsq, 0x002504, 0x00000000); /* un-block fifo */
	return 0;
}

static int
nv50_ram_prog(struct nouveau_fb *pfb)
{
	struct nouveau_device *device = nv_device(pfb);
	struct nv50_ram *ram = (void *)pfb->ram;
	struct nv50_ramseq *hwsq = &ram->hwsq;
	u32 config = nv_rd32(pfb, 0x100200);

	if (config & 0x800)
		nv_wr32(pfb, 0x100200, config & ~0x800);
	ram_exec(hwsq, nouveau_boolopt(device->cfgopt, "NvMemExec", true));
	if (config & 0x800)
		nv_wr32(pfb, 0x100200, config);
	return 0;
}

static void
nv50_ram_tidy(struct nouveau_fb *pfb)
{
	struct nv50_ram *ram = (void *)pfb->ram;
	struct nv50_ramseq *hwsq = &ram->hwsq;
	ram_exec(hwsq, false);
}

void
__nv50_ram_put(struct nouveau_fb *pfb, struct nouveau_mem *mem)
{
	struct nouveau_mm_node *this;

	while (!list_empty(&mem->regions)) {
		this = list_first_entry(&mem->regions, typeof(*this), rl_entry);

		list_del(&this->rl_entry);
		nouveau_mm_free(&pfb->vram, &this);
	}

	nouveau_mm_free(&pfb->tags, &mem->tag);
}

void
nv50_ram_put(struct nouveau_fb *pfb, struct nouveau_mem **pmem)
{
	struct nouveau_mem *mem = *pmem;

	*pmem = NULL;
	if (unlikely(mem == NULL))
		return;

	mutex_lock(&pfb->base.mutex);
	__nv50_ram_put(pfb, mem);
	mutex_unlock(&pfb->base.mutex);

	kfree(mem);
}

int
nv50_ram_get(struct nouveau_fb *pfb, u64 size, u32 align, u32 ncmin,
	     u32 memtype, struct nouveau_mem **pmem)
{
	struct nouveau_mm *heap = &pfb->vram;
	struct nouveau_mm *tags = &pfb->tags;
	struct nouveau_mm_node *r;
	struct nouveau_mem *mem;
	int comp = (memtype & 0x300) >> 8;
	int type = (memtype & 0x07f);
	int back = (memtype & 0x800);
	int min, max, ret;

	max = (size >> 12);
	min = ncmin ? (ncmin >> 12) : max;
	align >>= 12;

	mem = kzalloc(sizeof(*mem), GFP_KERNEL);
	if (!mem)
		return -ENOMEM;

	mutex_lock(&pfb->base.mutex);
	if (comp) {
		if (align == 16) {
			int n = (max >> 4) * comp;

			ret = nouveau_mm_head(tags, 0, 1, n, n, 1, &mem->tag);
			if (ret)
				mem->tag = NULL;
		}

		if (unlikely(!mem->tag))
			comp = 0;
	}

	INIT_LIST_HEAD(&mem->regions);
	mem->memtype = (comp << 7) | type;
	mem->size = max;

	type = nv50_fb_memtype[type];
	do {
		if (back)
			ret = nouveau_mm_tail(heap, 0, type, max, min, align, &r);
		else
			ret = nouveau_mm_head(heap, 0, type, max, min, align, &r);
		if (ret) {
			mutex_unlock(&pfb->base.mutex);
			pfb->ram->put(pfb, &mem);
			return ret;
		}

		list_add_tail(&r->rl_entry, &mem->regions);
		max -= r->length;
	} while (max);
	mutex_unlock(&pfb->base.mutex);

	r = list_first_entry(&mem->regions, struct nouveau_mm_node, rl_entry);
	mem->offset = (u64)r->offset << 12;
	*pmem = mem;
	return 0;
}

static u32
nv50_fb_vram_rblock(struct nouveau_fb *pfb, struct nouveau_ram *ram)
{
	int colbits, rowbitsa, rowbitsb, banks;
	u64 rowsize, predicted;
	u32 r0, r4, rt, rblock_size;

	r0 = nv_rd32(pfb, 0x100200);
	r4 = nv_rd32(pfb, 0x100204);
	rt = nv_rd32(pfb, 0x100250);
	nv_debug(pfb, "memcfg 0x%08x 0x%08x 0x%08x 0x%08x\n", r0, r4, rt,
			nv_rd32(pfb, 0x001540));

	ram->parts = parts;

	colbits  =  (r4 & 0x0000f000) >> 12;
	rowbitsa = ((r4 & 0x000f0000) >> 16) + 8;
	rowbitsb = ((r4 & 0x00f00000) >> 20) + 8;
	banks    = 1 << (((r4 & 0x03000000) >> 24) + 2);

	rowsize = ram->parts * banks * (1 << colbits) * 8;
	predicted = rowsize << rowbitsa;
	if (r0 & 0x00000004)
		predicted += rowsize << rowbitsb;

	if (predicted != ram->size) {
		nv_warn(pfb, "memory controller reports %d MiB VRAM\n",
			(u32)(ram->size >> 20));
	}

	rblock_size = rowsize;
	if (rt & 1)
		rblock_size *= 3;

	nv_debug(pfb, "rblock %d bytes\n", rblock_size);
	return rblock_size;
}

int
nv50_ram_create_(struct nouveau_object *parent, struct nouveau_object *engine,
		 struct nouveau_oclass *oclass, int length, void **pobject)
{
	const u32 rsvd_head = ( 256 * 1024) >> 12; /* vga memory */
	const u32 rsvd_tail = (1024 * 1024) >> 12; /* vbios etc */
	struct nouveau_bios *bios = nouveau_bios(parent);
	struct nouveau_fb *pfb = nouveau_fb(parent);
	struct nouveau_ram *ram;
	int ret;

	ret = nouveau_ram_create_(parent, engine, oclass, length, pobject);
	ram = *pobject;
	if (ret)
		return ret;

	ram->size = nv_rd32(pfb, 0x10020c);
	ram->size = (ram->size & 0xffffff00) | ((ram->size & 0x000000ff) << 32);

	ram->part_mask = (nv_rd32(pfb, 0x001540) & 0x00ff0000) >> 16;
	ram->parts = hweight8(ram->part_mask);

	switch (nv_rd32(pfb, 0x100714) & 0x00000007) {
	case 0: ram->type = NV_MEM_TYPE_DDR1; break;
	case 1:
		if (nouveau_fb_bios_memtype(bios) == NV_MEM_TYPE_DDR3)
			ram->type = NV_MEM_TYPE_DDR3;
		else
			ram->type = NV_MEM_TYPE_DDR2;
		break;
	case 2: ram->type = NV_MEM_TYPE_GDDR3; break;
	case 3: ram->type = NV_MEM_TYPE_GDDR4; break;
	case 4: ram->type = NV_MEM_TYPE_GDDR5; break;
	default:
		break;
	}

	ret = nouveau_mm_init(&pfb->vram, rsvd_head, (ram->size >> 12) -
			      (rsvd_head + rsvd_tail),
			      nv50_fb_vram_rblock(pfb, ram) >> 12);
	if (ret)
		return ret;

	ram->ranks = (nv_rd32(pfb, 0x100200) & 0x4) ? 2 : 1;
	ram->tags  =  nv_rd32(pfb, 0x100320);
	ram->get = nv50_ram_get;
	ram->put = nv50_ram_put;
	return 0;
}

static int
nv50_ram_ctor(struct nouveau_object *parent, struct nouveau_object *engine,
	      struct nouveau_oclass *oclass, void *data, u32 datasize,
	      struct nouveau_object **pobject)
{
	struct nv50_ram *ram;
	int ret, i;

	ret = nv50_ram_create(parent, engine, oclass, &ram);
	*pobject = nv_object(ram);
	if (ret)
		return ret;

	switch (ram->base.type) {
	case NV_MEM_TYPE_DDR2:
	case NV_MEM_TYPE_GDDR3:
		ram->base.calc = nv50_ram_calc;
		ram->base.prog = nv50_ram_prog;
		ram->base.tidy = nv50_ram_tidy;
		break;
	default:
		nv_warn(ram, "reclocking of this ram type unsupported\n");
		return 0;
	}

	ram->hwsq.r_0x002504 = hwsq_reg(0x002504);
	ram->hwsq.r_0x00c040 = hwsq_reg(0x00c040);
	ram->hwsq.r_0x004008 = hwsq_reg(0x004008);
	ram->hwsq.r_0x00400c = hwsq_reg(0x00400c);
	ram->hwsq.r_0x100210 = hwsq_reg(0x100210);
	ram->hwsq.r_0x1002d0 = hwsq_reg(0x1002d0);
	ram->hwsq.r_0x1002d4 = hwsq_reg(0x1002d4);
	ram->hwsq.r_0x1002dc = hwsq_reg(0x1002dc);
	for (i = 0; i < 8; i++)
		ram->hwsq.r_0x100da0[i] = hwsq_reg(0x100da0 + (i * 0x04));
	ram->hwsq.r_0x100e20 = hwsq_reg(0x100e20);
	ram->hwsq.r_0x100e24 = hwsq_reg(0x100e24);
	ram->hwsq.r_0x611200 = hwsq_reg(0x611200);

	for (i = 0; i < 9; i++)
		ram->hwsq.r_timing[i] = hwsq_reg(0x100220 + (i * 0x04));

	if (ram->base.ranks > 1) {
		ram->hwsq.r_mr[0] = hwsq_reg2(0x1002c0, 0x1002c8);
		ram->hwsq.r_mr[1] = hwsq_reg2(0x1002c4, 0x1002cc);
		ram->hwsq.r_mr[2] = hwsq_reg2(0x1002e0, 0x1002e8);
		ram->hwsq.r_mr[3] = hwsq_reg2(0x1002e4, 0x1002ec);
	} else {
		ram->hwsq.r_mr[0] = hwsq_reg(0x1002c0);
		ram->hwsq.r_mr[1] = hwsq_reg(0x1002c4);
		ram->hwsq.r_mr[2] = hwsq_reg(0x1002e0);
		ram->hwsq.r_mr[3] = hwsq_reg(0x1002e4);
	}

	return 0;
}

struct nouveau_oclass
nv50_ram_oclass = {
	.ofuncs = &(struct nouveau_ofuncs) {
		.ctor = nv50_ram_ctor,
		.dtor = _nouveau_ram_dtor,
		.init = _nouveau_ram_init,
		.fini = _nouveau_ram_fini,
	}
};
