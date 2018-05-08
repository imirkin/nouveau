/*
 * Copyright 2018 Red Hat Inc.
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
 */
#include "head.h"
#include "core.h"

void
head507d_procamp(struct nv50_head *head, struct nv50_head_atom *asyh)
{
	struct nv50_dmac *core = &nv50_disp(head->base.base.dev)->core->chan;
	u32 *push;
	if ((push = evo_wait(core, 2))) {
		evo_mthd(push, 0x08a8 + (head->base.index * 0x400), 1);
		evo_data(push, asyh->procamp.sat.sin << 20 |
			       asyh->procamp.sat.cos << 8);
		evo_kick(push, core);
	}
}

void
head507d_dither(struct nv50_head *head, struct nv50_head_atom *asyh)
{
	struct nv50_dmac *core = &nv50_disp(head->base.base.dev)->core->chan;
	u32 *push;
	if ((push = evo_wait(core, 2))) {
		evo_mthd(push, 0x08a0 + (head->base.index * 0x0400), 1);
		evo_data(push, asyh->dither.mode << 3 |
			       asyh->dither.bits << 1 |
			       asyh->dither.enable);
		evo_kick(push, core);
	}
}

void
head507d_ovly(struct nv50_head *head, struct nv50_head_atom *asyh)
{
	struct nv50_dmac *core = &nv50_disp(head->base.base.dev)->core->chan;
	u32 bounds = 0;
	u32 *push;

	if (asyh->ovly.cpp) {
		switch (asyh->ovly.cpp) {
		case 8: bounds |= 0x00000500; break;
		case 4: bounds |= 0x00000300; break;
		case 2: bounds |= 0x00000100; break;
		default:
			WARN_ON(1);
			break;
		}
		bounds |= 0x00000001;
	}

	if ((push = evo_wait(core, 2))) {
		evo_mthd(push, 0x0904 + head->base.index * 0x400, 1);
		evo_data(push, bounds);
		evo_kick(push, core);
	}
}

void
head507d_base(struct nv50_head *head, struct nv50_head_atom *asyh)
{
	struct nv50_dmac *core = &nv50_disp(head->base.base.dev)->core->chan;
	u32 bounds = 0;
	u32 *push;

	if (asyh->base.cpp) {
		switch (asyh->base.cpp) {
		case 8: bounds |= 0x00000500; break;
		case 4: bounds |= 0x00000300; break;
		case 2: bounds |= 0x00000100; break;
		case 1: bounds |= 0x00000000; break;
		default:
			WARN_ON(1);
			break;
		}
		bounds |= 0x00000001;
	}

	if ((push = evo_wait(core, 2))) {
		evo_mthd(push, 0x0900 + head->base.index * 0x400, 1);
		evo_data(push, bounds);
		evo_kick(push, core);
	}
}

static void
head507d_curs_clr(struct nv50_head *head)
{
	struct nv50_dmac *core = &nv50_disp(head->base.base.dev)->core->chan;
	u32 *push;
	if ((push = evo_wait(core, 2))) {
		evo_mthd(push, 0x0880 + head->base.index * 0x400, 1);
		evo_data(push, 0x05000000);
		evo_kick(push, core);
	}
}

static void
head507d_curs_set(struct nv50_head *head, struct nv50_head_atom *asyh)
{
	struct nv50_dmac *core = &nv50_disp(head->base.base.dev)->core->chan;
	u32 *push;
	if ((push = evo_wait(core, 3))) {
		evo_mthd(push, 0x0880 + head->base.index * 0x400, 2);
		evo_data(push, 0x80000000 | asyh->curs.layout << 26 |
					    asyh->curs.format << 24);
		evo_data(push, asyh->curs.offset >> 8);
		evo_kick(push, core);
	}
}

void
head507d_core_clr(struct nv50_head *head)
{
	struct nv50_dmac *core = &nv50_disp(head->base.base.dev)->core->chan;
	u32 *push;
	if ((push = evo_wait(core, 2))) {
		evo_mthd(push, 0x0874 + head->base.index * 0x400, 1);
		evo_data(push, 0x00000000);
		evo_kick(push, core);
	}
}

static void
head507d_core_set(struct nv50_head *head, struct nv50_head_atom *asyh)
{
	struct nv50_dmac *core = &nv50_disp(head->base.base.dev)->core->chan;
	u32 *push;
	if ((push = evo_wait(core, 9))) {
		evo_mthd(push, 0x0860 + head->base.index * 0x400, 1);
		evo_data(push, asyh->core.offset >> 8);
		evo_mthd(push, 0x0868 + head->base.index * 0x400, 4);
		evo_data(push, asyh->core.h << 16 | asyh->core.w);
		evo_data(push, asyh->core.layout << 20 |
			       asyh->core.pitch >> 8 << 8 |
			       asyh->core.block);
		evo_data(push, asyh->core.kind << 16 |
			       asyh->core.format << 8);
		evo_data(push, asyh->core.handle);
		evo_mthd(push, 0x08c0 + head->base.index * 0x400, 1);
		evo_data(push, asyh->core.y << 16 | asyh->core.x);
		evo_kick(push, core);

		/* EVO will complain with INVALID_STATE if we have an
		 * active cursor and (re)specify HeadSetContextDmaIso
		 * without also updating HeadSetOffsetCursor.
		 */
		asyh->set.curs = asyh->curs.visible;
	}
}

void
head507d_core_calc(struct nv50_head *head, struct nv50_head_atom *asyh)
{
	struct nv50_disp *disp = nv50_disp(head->base.base.dev);
	if ((asyh->core.visible = (asyh->base.cpp != 0))) {
		asyh->core.x = asyh->base.x;
		asyh->core.y = asyh->base.y;
		asyh->core.w = asyh->base.w;
		asyh->core.h = asyh->base.h;
	} else
	if ((asyh->core.visible = asyh->curs.visible) ||
	    (asyh->core.visible = asyh->ilut.visible)) {
		/*XXX: We need to either find some way of having the
		 *     primary base layer appear black, while still
		 *     being able to display the other layers, or we
		 *     need to allocate a dummy black surface here.
		 */
		asyh->core.x = 0;
		asyh->core.y = 0;
		asyh->core.w = asyh->state.mode.hdisplay;
		asyh->core.h = asyh->state.mode.vdisplay;
	}
	asyh->core.handle = disp->core->chan.vram.handle;
	asyh->core.offset = 0;
	asyh->core.format = 0xcf;
	asyh->core.kind = 0;
	asyh->core.layout = 1;
	asyh->core.block = 0;
	asyh->core.pitch = ALIGN(asyh->core.w, 64) * 4;
}

static void
head507d_ilut_clr(struct nv50_head *head)
{
	struct nv50_dmac *core = &nv50_disp(head->base.base.dev)->core->chan;
	u32 *push;
	if ((push = evo_wait(core, 2))) {
		evo_mthd(push, 0x0840 + (head->base.index * 0x400), 1);
		evo_data(push, 0x40000000);
		evo_kick(push, core);
	}
}

static void
head507d_ilut_set(struct nv50_head *head, struct nv50_head_atom *asyh)
{
	struct nv50_dmac *core = &nv50_disp(head->base.base.dev)->core->chan;
	u32 *push;
	if ((push = evo_wait(core, 3))) {
		evo_mthd(push, 0x0840 + (head->base.index * 0x400), 2);
		evo_data(push, 0x80000000 | asyh->ilut.mode << 30);
		evo_data(push, asyh->ilut.offset >> 8);
		evo_kick(push, core);
	}
}

void
head507d_mode(struct nv50_head *head, struct nv50_head_atom *asyh)
{
	struct nv50_dmac *core = &nv50_disp(head->base.base.dev)->core->chan;
	struct nv50_head_mode *m = &asyh->mode;
	u32 *push;
	if ((push = evo_wait(core, 13))) {
		evo_mthd(push, 0x0804 + (head->base.index * 0x400), 2);
		evo_data(push, 0x00800000 | m->clock);
		evo_data(push, m->interlace ? 0x00000002 : 0x00000000);
		evo_mthd(push, 0x0810 + (head->base.index * 0x400), 7);
		evo_data(push, 0x00000000);
		evo_data(push, m->v.active  << 16 | m->h.active );
		evo_data(push, m->v.synce   << 16 | m->h.synce  );
		evo_data(push, m->v.blanke  << 16 | m->h.blanke );
		evo_data(push, m->v.blanks  << 16 | m->h.blanks );
		evo_data(push, m->v.blank2e << 16 | m->v.blank2s);
		evo_data(push, asyh->mode.v.blankus);
		evo_mthd(push, 0x082c + (head->base.index * 0x400), 1);
		evo_data(push, 0x00000000);
		evo_kick(push, core);
	}
}

void
head507d_view(struct nv50_head *head, struct nv50_head_atom *asyh)
{
	struct nv50_dmac *core = &nv50_disp(head->base.base.dev)->core->chan;
	u32 *push;
	if ((push = evo_wait(core, 7))) {
		evo_mthd(push, 0x08a4 + (head->base.index * 0x400), 1);
		evo_data(push, 0x00000000);
		evo_mthd(push, 0x08c8 + (head->base.index * 0x400), 1);
		evo_data(push, asyh->view.iH << 16 | asyh->view.iW);
		evo_mthd(push, 0x08d8 + (head->base.index * 0x400), 2);
		evo_data(push, asyh->view.oH << 16 | asyh->view.oW);
		evo_data(push, asyh->view.oH << 16 | asyh->view.oW);
		evo_kick(push, core);
	}
}

const struct nv50_head_func
head507d = {
	.view = head507d_view,
	.mode = head507d_mode,
	.ilut_set = head507d_ilut_set,
	.ilut_clr = head507d_ilut_clr,
	.core_calc = head507d_core_calc,
	.core_set = head507d_core_set,
	.core_clr = head507d_core_clr,
	.curs_set = head507d_curs_set,
	.curs_clr = head507d_curs_clr,
	.base = head507d_base,
	.ovly = head507d_ovly,
	.dither = head507d_dither,
	.procamp = head507d_procamp,
};