/*
 * Copyright (c) 2015 Freescale Semiconductor, Inc.
 * Copyright © 2013 Vasily Khoruzhick <anarsoul@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT.  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef __g2d_renderer_h_
#define __g2d_renderer_h_

#include <libweston/libweston.h>
#include "backend.h"
#include "libweston-internal.h"

#include <g2dExt.h>
#include <errno.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/fb.h>
#include <linux/input.h>
#include <assert.h>

#ifdef ENABLE_EGL
#include <EGL/egl.h>
#include <EGL/eglext.h>
#endif

struct g2d_renderer_output_options {
	struct weston_size fb_size;
	struct weston_geometry area;
	const struct pixel_format_info *formats;
	unsigned formats_count;
};

struct g2d_renderer_interface {
	int (*create)(struct weston_compositor *ec);

	int (*drm_display_create)(struct weston_compositor *ec, void *native_window);

	int (*drm_output_create)(struct weston_output *output,
				const struct g2d_renderer_output_options *options);

	int (*create_g2d_image)(struct g2d_surfaceEx* g2dSurface,
				enum g2d_format g2dFormat,
				void *vaddr,
				int w, int h, int stride,
				int size,
				int dmafd);

	void (*output_set_buffer)(struct weston_output *output,
				struct g2d_surfaceEx *buffer);

	void (*output_destroy)(struct weston_output *output);

	int (*get_surface_fence_fd)(struct g2d_surfaceEx *buffer);
};

#endif
