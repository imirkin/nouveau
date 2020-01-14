/*
 * Copyright (c) 2015, NVIDIA CORPORATION. All rights reserved.
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
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
#include "gf100.h"
#include "ctxgf100.h"

#include <core/firmware.h>
#include <subdev/acr.h>
#include <subdev/timer.h>

#include <nvif/class.h>

const struct nvkm_acr_lsf_func
gm20b_gr_fecs_acr = {
};

static void
gm20b_gr_init_gpc_mmu(struct gf100_gr *gr)
{
	struct nvkm_device *device = gr->base.engine.subdev.device;
	u32 val;

	/* Bypass MMU check for non-secure boot */
	if (!device->secboot) {
		nvkm_wr32(device, 0x100ce4, 0xffffffff);

		if (nvkm_rd32(device, 0x100ce4) != 0xffffffff)
			nvdev_warn(device,
			  "cannot bypass secure boot - expect failure soon!\n");
	}

	val = nvkm_rd32(device, 0x100c80);
	val &= 0xf000187f;
	nvkm_wr32(device, 0x418880, val);
	nvkm_wr32(device, 0x418890, 0);
	nvkm_wr32(device, 0x418894, 0);

	nvkm_wr32(device, 0x4188b0, nvkm_rd32(device, 0x100cc4));
	nvkm_wr32(device, 0x4188b4, nvkm_rd32(device, 0x100cc8));
	nvkm_wr32(device, 0x4188b8, nvkm_rd32(device, 0x100ccc));

	nvkm_wr32(device, 0x4188ac, nvkm_rd32(device, 0x100800));
}

static void
gm20b_gr_set_hww_esr_report_mask(struct gf100_gr *gr)
{
	struct nvkm_device *device = gr->base.engine.subdev.device;
	nvkm_wr32(device, 0x419e44, 0xdffffe);
	nvkm_wr32(device, 0x419e4c, 0x5);
}

static const struct gf100_gr_func
gm20b_gr = {
	.oneinit_tiles = gm200_gr_oneinit_tiles,
	.oneinit_sm_id = gm200_gr_oneinit_sm_id,
	.init = gk20a_gr_init,
	.init_zcull = gf117_gr_init_zcull,
	.init_gpc_mmu = gm20b_gr_init_gpc_mmu,
	.init_rop_active_fbps = gk104_gr_init_rop_active_fbps,
	.trap_mp = gf100_gr_trap_mp,
	.set_hww_esr_report_mask = gm20b_gr_set_hww_esr_report_mask,
	.rops = gm200_gr_rops,
	.ppc_nr = 1,
	.grctx = &gm20b_grctx,
	.zbc = &gf100_gr_zbc,
	.sclass = {
		{ -1, -1, FERMI_TWOD_A },
		{ -1, -1, KEPLER_INLINE_TO_MEMORY_B },
		{ -1, -1, MAXWELL_B, &gf100_fermi },
		{ -1, -1, MAXWELL_COMPUTE_B },
		{}
	}
};

static int
gm20b_gr_load(struct gf100_gr *gr, int ver, const struct gf100_gr_fwif *fwif)
{
	struct nvkm_subdev *subdev = &gr->base.engine.subdev;
	int ret;

	ret = nvkm_acr_lsfw_load_bl_inst_data_sig(subdev, gr->fecs.falcon,
						  NVKM_ACR_LSF_FECS,
						  "gr/fecs_", ver, fwif->fecs);
	if (ret)
		return ret;


	if (nvkm_firmware_load_blob(subdev, "gr/", "gpccs_inst", ver,
				    &gr->gpccs.inst) ||
	    nvkm_firmware_load_blob(subdev, "gr/", "gpccs_data", ver,
				    &gr->gpccs.data))
		return -ENOENT;

	gr->firmware = true;

	return gk20a_gr_load_sw(gr, "gr/", ver);
}

#if IS_ENABLED(CONFIG_ARCH_TEGRA_210_SOC)
MODULE_FIRMWARE("nvidia/gm20b/gr/fecs_bl.bin");
MODULE_FIRMWARE("nvidia/gm20b/gr/fecs_inst.bin");
MODULE_FIRMWARE("nvidia/gm20b/gr/fecs_data.bin");
MODULE_FIRMWARE("nvidia/gm20b/gr/fecs_sig.bin");
MODULE_FIRMWARE("nvidia/gm20b/gr/gpccs_inst.bin");
MODULE_FIRMWARE("nvidia/gm20b/gr/gpccs_data.bin");
MODULE_FIRMWARE("nvidia/gm20b/gr/sw_ctx.bin");
MODULE_FIRMWARE("nvidia/gm20b/gr/sw_nonctx.bin");
MODULE_FIRMWARE("nvidia/gm20b/gr/sw_bundle_init.bin");
MODULE_FIRMWARE("nvidia/gm20b/gr/sw_method_init.bin");
#endif

static const struct gf100_gr_fwif
gm20b_gr_fwif[] = {
	{ 0, gm20b_gr_load, &gm20b_gr, &gm20b_gr_fecs_acr },
	{}
};

int
gm20b_gr_new(struct nvkm_device *device, int index, struct nvkm_gr **pgr)
{
	return gf100_gr_new_(gm20b_gr_fwif, device, index, pgr);
}
