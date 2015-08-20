/*
 * Copyright 2012 Red Hat Inc.
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
#include <engine/mspdec.h>
#include <engine/falcon.h>

#include <nvif/class.h>

static int
gk104_mspdec_init(struct nvkm_object *object)
{
	struct nvkm_falcon *mspdec = (void *)object;
	struct nvkm_device *device = mspdec->engine.subdev.device;
	int ret;

	ret = nvkm_falcon_init(mspdec);
	if (ret)
		return ret;

	nvkm_wr32(device, 0x085010, 0x0000fff2);
	nvkm_wr32(device, 0x08501c, 0x0000fff2);
	return 0;
}

static const struct nvkm_falcon_func
gk104_mspdec_func = {
	.sclass = {
		{ -1, -1, GK104_MSPDEC },
		{}
	}
};

static int
gk104_mspdec_ctor(struct nvkm_object *parent, struct nvkm_object *engine,
		  struct nvkm_oclass *oclass, void *data, u32 size,
		  struct nvkm_object **pobject)
{
	struct nvkm_falcon *falcon;
	int ret;

	ret = nvkm_falcon_create(&gk104_mspdec_func, parent, engine, oclass,
				 0x085000, true, "PMSPDEC", "mspdec", &falcon);
	*pobject = nv_object(falcon);
	if (ret)
		return ret;

	nv_subdev(falcon)->unit = 0x00020000;
	return 0;
}

struct nvkm_oclass
gk104_mspdec_oclass = {
	.handle = NV_ENGINE(MSPDEC, 0xe0),
	.ofuncs = &(struct nvkm_ofuncs) {
		.ctor = gk104_mspdec_ctor,
		.dtor = _nvkm_falcon_dtor,
		.init = gk104_mspdec_init,
		.fini = _nvkm_falcon_fini,
	},
};
