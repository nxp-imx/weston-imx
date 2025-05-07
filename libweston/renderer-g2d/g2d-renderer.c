/*
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 * Copyright © 2012 Intel Corporation
 * Copyright © 2015 Collabora, Ltd.
 * Copyright 2018 NXP
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

#include "config.h"

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <float.h>
#include <math.h>
#include <assert.h>
#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <drm_fourcc.h>
#include <poll.h>
#include <errno.h>
#include <sys/stat.h>

#include <libweston/libweston.h>
#include "g2d-renderer.h"
#include "output-capture.h"
#include "vertex-clipping.h"
#include "linux-dmabuf.h"
#include "linux-dmabuf-unstable-v1-server-protocol.h"
#include "linux-explicit-synchronization.h"
#include "shared/fd-util.h"
#include "shared/helpers.h"
#include "shared/platform.h"
#include "pixel-formats.h"
#include "shared/xalloc.h"

#define BUFFER_DAMAGE_COUNT 3
#define ALIGN_TO_16(a) (((a) + 15) & ~15)
#define ALIGN_TO_64(a) (((a) + 63) & ~63)

#ifdef ENABLE_EGL
static PFNEGLGETPLATFORMDISPLAYEXTPROC get_platform_display = NULL;
#ifndef PFNEGLUPDATEWAYLANDBUFFERWL
typedef EGLBoolean (EGLAPIENTRYP PFNEGLUPDATEWAYLANDBUFFERWL)(EGLDisplay dpy, struct wl_resource *buffer, EGLint attribute);
#endif
#endif

enum g2d_rotation_angle
{
	/* rotation angle 0 */
	G2D_ROTATION_ANGLE_0 = 0x10,

	/* clockwise rotation */
	G2D_ROTATION_ANGLE_POSITIVE_90 = 0x20,
	G2D_ROTATION_ANGLE_POSITIVE_180 = 0x40,
	G2D_ROTATION_ANGLE_POSITIVE_270 = 0x80,

	/* Anticlockwise rotation */
	G2D_ROTATION_ANGLE_NEGATIVE_90 = 0x08,
	G2D_ROTATION_ANGLE_NEGATIVE_180 = 0x04,
	G2D_ROTATION_ANGLE_NEGATIVE_270 = 0x02,
};

struct wl_viv_buffer
{
	struct wl_resource *resource;
	void *surface;
	signed int width;
	signed int height;
	enum g2d_format format;
	unsigned int alignedWidth;
	unsigned int alignedHeight;
	unsigned int physical[3];
	unsigned int gpuBaseAddr;
	enum g2d_tiling tiling;
	signed int fd;

	unsigned int ts_addr;
	unsigned int fc_enabled;
	unsigned int fcValue;
	unsigned int fcValueUpper;
	unsigned int compressed;
	unsigned int tileStatus_enabled;
};

typedef struct _g2dRECT
{
	int left;
	int top;
	int right;
	int bottom;
} g2dRECT;

struct g2d_output_state {
	int current_buffer;
	struct weston_size fb_size;
	struct weston_geometry area;
	pixman_region32_t buffer_damage[BUFFER_DAMAGE_COUNT];
	struct g2d_surfaceEx *drm_hw_buffer;
	int width;
	int height;
};

struct g2d_surface_state {
	float color[4];
	bool solid_clear;
	int clcolor;
	struct weston_buffer_reference buffer_ref;
	struct weston_buffer_release_reference buffer_release_ref;
	int pitch; /* in pixels */
	int attached;
	pixman_region32_t texture_damage;
	struct g2d_surfaceEx g2d_surface;
	struct g2d_buf *shm_buf;
	struct g2d_buf *dma_buf;
	int shm_buf_length;
	int bpp;

	struct weston_surface *surface;
	struct wl_listener surface_destroy_listener;
	struct wl_listener renderer_destroy_listener;
};

struct g2d_renderer {
	struct weston_renderer base;
	struct wl_signal destroy_signal;
	struct wl_array position_stream;
#ifdef ENABLE_EGL
	NativeDisplayType display;
	EGLDisplay egl_display;
	struct wl_display *wl_display;
	PFNEGLBINDWAYLANDDISPLAYWL bind_display;
	PFNEGLUNBINDWAYLANDDISPLAYWL unbind_display;
	PFNEGLQUERYWAYLANDBUFFERWL query_buffer;
	PFNEGLUPDATEWAYLANDBUFFERWL update_buffer;

	EGLDeviceEXT egl_device;
	const char *drm_device;

	PFNEGLQUERYDISPLAYATTRIBEXTPROC query_display_attrib;
	PFNEGLQUERYDEVICESTRINGEXTPROC query_device_string;
	bool has_device_query;
	bool has_bind_display;

	bool has_dmabuf_import_modifiers;
	PFNEGLQUERYDMABUFFORMATSEXTPROC query_dmabuf_formats;
	PFNEGLQUERYDMABUFMODIFIERSEXTPROC query_dmabuf_modifiers;
#endif
	void *handle;
	int use_drm;
	struct weston_drm_format_array supported_formats;
};

static int
g2d_renderer_create_surface(struct weston_surface *surface);

static inline struct g2d_surface_state *
get_surface_state(struct weston_surface *surface)
{
	if (!surface->renderer_state)
		g2d_renderer_create_surface(surface);

	return (struct g2d_surface_state *)surface->renderer_state;
}

static inline struct g2d_renderer *
get_renderer(struct weston_compositor *ec)
{
	return (struct g2d_renderer *)ec->renderer;
}

#define max(a, b) (((a) > (b)) ? (a) : (b))
#define min(a, b) (((a) > (b)) ? (b) : (a))

static void
calculate_rect_with_transform(int surfaceWidth, int surfaceHeight,
				  uint32_t transform, g2dRECT *rect)
{
	g2dRECT tmp = *rect;

	switch (transform) {
	case WL_OUTPUT_TRANSFORM_NORMAL:
	default:
		break;
	case WL_OUTPUT_TRANSFORM_270:
		rect->right = surfaceWidth - tmp.top;
		rect->left = rect->right - (tmp.bottom - tmp.top);
		rect->top = tmp.left;
		rect->bottom = rect->top + (tmp.right - tmp.left);
		break;
	case WL_OUTPUT_TRANSFORM_90:
		rect->left = tmp.top;
		rect->right = rect->left + (tmp.bottom - tmp.top);
		rect->bottom = surfaceHeight - tmp.left;
		rect->top = rect->bottom - (tmp.right - tmp.left);
		break;
	case WL_OUTPUT_TRANSFORM_180:
		rect->left = surfaceWidth - tmp.right;
		rect->right = rect->left + (tmp.right - tmp.left);
		rect->bottom = surfaceHeight - tmp.top;
		rect->top = rect->bottom - (tmp.bottom - tmp.top);
		break;
	}
}

static void
convert_size_by_view_transform(int *width_out, int *height_out, int width, int height, uint32_t transform)
{
		switch (transform) {
	case WL_OUTPUT_TRANSFORM_NORMAL:
	case WL_OUTPUT_TRANSFORM_180:
	case WL_OUTPUT_TRANSFORM_FLIPPED:
	case WL_OUTPUT_TRANSFORM_FLIPPED_180:
	default:
		*width_out = width;
		*height_out = height;
		break;
	case WL_OUTPUT_TRANSFORM_90:
	case WL_OUTPUT_TRANSFORM_270:
	case WL_OUTPUT_TRANSFORM_FLIPPED_90:
	case WL_OUTPUT_TRANSFORM_FLIPPED_270:
		*width_out = height;
		*height_out = width;
		break;
	}
}

static enum g2d_rotation
convert_transform_to_rot(uint32_t view_transform, uint32_t output_transform)
{
	uint8_t angle = G2D_ROTATION_ANGLE_0;
	enum g2d_rotation rot;

	/* First, rotate according to the angle set by the client. */
	angle = angle << view_transform;
	/* Then, rotate according to the angle of the output. */
	angle = angle >> output_transform;

	switch(angle) {
	case G2D_ROTATION_ANGLE_0:
	default:
		rot = G2D_ROTATION_0;
		break;
	case G2D_ROTATION_ANGLE_POSITIVE_270:
	case G2D_ROTATION_ANGLE_NEGATIVE_90:
		rot = G2D_ROTATION_90;
		break;
	case G2D_ROTATION_ANGLE_POSITIVE_90:
	case G2D_ROTATION_ANGLE_NEGATIVE_270:
		rot = G2D_ROTATION_270;
		break;
	case G2D_ROTATION_ANGLE_POSITIVE_180:
	case G2D_ROTATION_ANGLE_NEGATIVE_180:
		rot = G2D_ROTATION_180;
		break;
	}
	return rot;
}

static inline struct g2d_output_state *
get_output_state(struct weston_output *output)
{
	return (struct g2d_output_state *)output->renderer_state;
}

static int
g2d_getG2dFormat_from_pixman(pixman_format_code_t Format, enum g2d_format* g2dFormat)
{
	switch(Format)
	{
	case PIXMAN_r5g6b5:
		*g2dFormat = G2D_RGB565;
		break;
	case PIXMAN_a8b8g8r8:
		*g2dFormat = G2D_RGBA8888;
		break;
	case PIXMAN_x8b8g8r8:
		*g2dFormat = G2D_RGBX8888;
		break;
	case PIXMAN_a8r8g8b8:
		*g2dFormat = G2D_BGRA8888;
		break;
	case PIXMAN_x8r8g8b8 :
		*g2dFormat = G2D_BGRX8888;
		break;
	case PIXMAN_b5g6r5:
		*g2dFormat = G2D_BGR565;
		break;
	case PIXMAN_b8g8r8a8:
		*g2dFormat = G2D_ARGB8888;
		break;
	case PIXMAN_r8g8b8a8:
		*g2dFormat = G2D_ABGR8888;
		break;
	case PIXMAN_b8g8r8x8:
		*g2dFormat = G2D_XRGB8888;
		break;
	case PIXMAN_r8g8b8x8:
		*g2dFormat = G2D_XBGR8888;
		break;
	case PIXMAN_yv12:
		*g2dFormat = G2D_YV12;
		break;
	case PIXMAN_yuy2:
		*g2dFormat = G2D_YUYV;
		break;
	default:
		weston_log("Error in function %s, Format(%d) not supported\n", __func__, Format);
		return -1;
	}
	return 0;
}

static void printG2dSurfaceInfo(struct g2d_surfaceEx* g2dSurface, const char* msg)
{
	weston_log("%s physicAddr = %x left = %d right = %d top=%d bottom=%d stride= %d tiling = %d, format=%d \n",
				msg,
				g2dSurface->base.planes[0],
				g2dSurface->base.left,
				g2dSurface->base.right,
				g2dSurface->base.top,
				g2dSurface->base.bottom,
				g2dSurface->base.stride,
				g2dSurface->tiling,
				g2dSurface->base.format);
}

static int
get_g2dSurface(struct wl_viv_buffer *buffer, struct g2d_surfaceEx *g2dSurface)
{
	if(buffer->width <= 0 || buffer->height <= 0)
	{
		weston_log("invalid EGL buffer in function %s\n", __func__);
		return -EINVAL;
	}
	g2dSurface->base.format = buffer->format;
	g2dSurface->tiling = buffer->tiling;
	g2dSurface->base.planes[0] = buffer->physical[0] + buffer->gpuBaseAddr;
	g2dSurface->base.planes[1] = buffer->physical[1] + buffer->gpuBaseAddr;
	g2dSurface->base.planes[2] = buffer->physical[2] + buffer->gpuBaseAddr;
	g2dSurface->base.left = 0;
	g2dSurface->base.top = 0;
	g2dSurface->base.right = buffer->width;
	g2dSurface->base.bottom = buffer->height;
	g2dSurface->base.stride = buffer->alignedWidth;
	g2dSurface->base.width	= buffer->width;
	g2dSurface->base.height = buffer->height;
	g2dSurface->base.rot	= G2D_ROTATION_0;

	if(buffer->ts_addr && buffer->tileStatus_enabled)
	{
		g2dSurface->tiling             |= G2D_TILED_STATUS;
		g2dSurface->ts.ts_addr         = buffer->ts_addr;
		g2dSurface->ts.fc_enabled      = buffer->fc_enabled;
		g2dSurface->ts.fc_value        = buffer->fcValue;
		g2dSurface->ts.fc_value_upper  = buffer->fcValueUpper;
	}

	return 0;
}

static void
g2d_SetSurfaceRect(struct g2d_surfaceEx* g2dSurface, g2dRECT* rect)
{
	if(g2dSurface && rect)
	{
		g2dSurface->base.left	= rect->left;
		g2dSurface->base.top	= rect->top;
		g2dSurface->base.right	= rect->right;
		g2dSurface->base.bottom = rect->bottom;
	}
}

#define _hasAlpha(format) (format==G2D_RGBA8888 || format==G2D_BGRA8888 \
	|| format==G2D_ARGB8888 || format==G2D_ABGR8888)


static int
g2d_clear_solid(void *handle, struct g2d_surfaceEx *dstG2dSurface, g2dRECT *clipRect, int clcolor)
{
	struct g2d_surfaceEx* soildSurface = dstG2dSurface;

	g2d_SetSurfaceRect(soildSurface, clipRect);
	soildSurface->base.clrcolor = clcolor;

	if(g2d_clear(handle,  &soildSurface->base)){
		printG2dSurfaceInfo(dstG2dSurface, "SOILD DST:");
		return -1;
	}
	return 0;
}

static int
g2d_blit_surface(void *handle, struct g2d_surfaceEx * srcG2dSurface, struct g2d_surfaceEx *dstG2dSurface,
	g2dRECT *srcRect, g2dRECT *dstRect)
{
	g2d_SetSurfaceRect(srcG2dSurface, srcRect);
	g2d_SetSurfaceRect(dstG2dSurface, dstRect);
	srcG2dSurface->base.blendfunc = G2D_ONE;
	dstG2dSurface->base.blendfunc = G2D_ONE_MINUS_SRC_ALPHA;
	if(!(_hasAlpha(srcG2dSurface->base.format))){
		g2d_disable(handle, G2D_BLEND);
	}

	if(g2d_blitEx(handle, srcG2dSurface, dstG2dSurface)){
		printG2dSurfaceInfo(srcG2dSurface, "SRC:");
		printG2dSurfaceInfo(dstG2dSurface, "DST:");
		return -1;
	}
	return 0;
}

static void
g2d_clip_rects(enum g2d_rotation transform,
			g2dRECT *srcRect,
			g2dRECT *dstrect,
			int dstWidth,
			int dstHeight)
{
	int srcWidth = srcRect->right - srcRect->left;
	int srcHeight = srcRect->bottom - srcRect->top;
	float scale_v = 1.0f;
	float scale_h = 1.0f;

	if(transform == G2D_ROTATION_90
		|| transform == G2D_ROTATION_270)
	{
		scale_h = (float)srcHeight / (dstrect->right - dstrect->left);
		scale_v = (float)srcWidth / (dstrect->bottom - dstrect->top);
	}
	else
	{
		scale_h = (float)srcWidth / (dstrect->right - dstrect->left);
		scale_v = (float)srcHeight / (dstrect->bottom - dstrect->top);
	}
	switch (transform) {
	case G2D_ROTATION_0:
		if(dstrect->left < 0)
		{
			srcRect->left += floorf((float)(-dstrect->left) * scale_h);
			dstrect->left = 0;
			if(srcRect->left >= srcRect->right)
				return;
		}
		if(dstrect->right > dstWidth)
		{
			srcRect->right -= floorf((float)(dstrect->right - dstWidth) * scale_h);
			dstrect->right = dstWidth;
			if(srcRect->right <= srcRect->left)
				return;
		}
		if(dstrect->top < 0)
		{
			srcRect->top += floorf((float)(-dstrect->top) * scale_v);
			dstrect->top = 0;
			if(srcRect->top >= srcRect->bottom)
				return;
		}
		if(dstrect->bottom > dstHeight)
		{
			srcRect->bottom -= floorf((float)(dstrect->bottom - dstHeight) * scale_v);
			dstrect->bottom = dstHeight;
			if(srcRect->bottom < 0)
				return;
		}
		break;
	case G2D_ROTATION_270:
		if(dstrect->left < 0)
		{
			srcRect->bottom -= floorf((float)(-dstrect->left) * scale_h);
			dstrect->left = 0;
			if(srcRect->top >= srcRect->bottom)
					return;
		}
		if(dstrect->bottom > dstHeight)
		{
			srcRect->right -= floorf((float)(dstrect->bottom - dstHeight) * scale_v);
			dstrect->bottom = dstHeight;
			if(srcRect->right < 0)
				return;
		}
		if(dstrect->top < 0)
		{
			srcRect->left += floorf((float)(-dstrect->top) * scale_v);
			dstrect->top = 0;
			if(srcRect->left > srcRect->right)
				return;
		}
		if(dstrect->right > dstWidth) {
			srcRect->top += floorf((float)(dstrect->right - dstWidth) * scale_h);
			dstrect->right = dstWidth;
			if(srcRect->top >= srcRect->bottom)
				return;
		}
		break;
	case G2D_ROTATION_90:
		if(dstrect->left < 0)
		{
			srcRect->top += floorf((float)(-dstrect->left) * scale_h);
			dstrect->left = 0;
			if(srcRect->top >= srcRect->bottom)
					return;
		}
		if(dstrect->top < 0)
		{
			srcRect->right -= floorf((float)(-dstrect->top) * scale_v);
			dstrect->top = 0;
			if(srcRect->left >= srcRect->right)
				return;
		}
		if(dstrect->bottom > dstHeight)
		{
			srcRect->left += floorf((float)(dstrect->bottom - dstHeight) * scale_v);
			dstrect->bottom = dstHeight;
			if(srcRect->right <= srcRect->left)
				return;
		}
		if(dstrect->right > dstWidth)
		{
			srcRect->bottom -= floorf((float)(dstrect->right - dstWidth) * scale_h);
			dstrect->right = dstWidth;
			if(srcRect->bottom <= srcRect->top)
				return;
		}
		break;
	case G2D_ROTATION_180:
		if(dstrect->left < 0)
		{
			srcRect->right -= floorf((float)(-dstrect->left) * scale_h);
			dstrect->left = 0;
			if(srcRect->left >= srcRect->right)
					return;
		}
		if(dstrect->right > dstWidth)
		{
			srcRect->left += floorf((float)(dstrect->right - dstWidth) * scale_h);
			dstrect->right = dstWidth;
			if(srcRect->right <= srcRect->left)
				return;
		}
		if(dstrect->top < 0)
		{
			srcRect->bottom -= floorf((float)(-dstrect->top) * scale_v);
			dstrect->top = 0;
			if(srcRect->top >= srcRect->bottom)
				return;
		}
		if(dstrect->bottom > dstHeight) {
			srcRect->top += floorf((float)(dstrect->bottom - dstHeight) * scale_v);
			dstrect->bottom = dstHeight;
			if(srcRect->top >= srcRect->bottom)
				return;
		}
		break;
	default:
		break;
	}
}

static int
g2d_renderer_read_pixels(struct weston_output *output,
				   const struct pixel_format_info *format, void *pixels,
				   uint32_t x, uint32_t y,
				   uint32_t width, uint32_t height)
{
	struct g2d_surfaceEx  dstSurface;
	struct g2d_surfaceEx  *srcSurface;
	struct g2d_output_state *go = get_output_state(output);
	struct g2d_renderer *gr = get_renderer(output->compositor);
	struct g2d_buf *read_buf = NULL;
	enum g2d_format dst_format;
	g2dRECT	srcRect  = {x, y, x + width, y + height};
	g2dRECT dstRect  = {0, 0, width, height};

	if( g2d_getG2dFormat_from_pixman(format->pixman_format, &dst_format))
		return -1;

	read_buf = g2d_alloc(width * height * 4, 0);
	if( !read_buf)
		return -1;

	srcSurface = go->drm_hw_buffer;

	dstSurface.base.planes[0] = read_buf->buf_paddr;
	dstSurface.base.format = dst_format;
	dstSurface.base.width  = width;
	dstSurface.base.height = height;
	dstSurface.base.stride = width;
	dstSurface.base.rot    = G2D_FLIP_V;
	if(g2d_blit_surface(gr->handle, srcSurface, &dstSurface, &srcRect, &dstRect)) {
		g2d_free(read_buf);
		return -1;
	}
	g2d_finish(gr->handle);

	memcpy(pixels, read_buf->buf_vaddr, width * height * PIXMAN_FORMAT_BPP(format->pixman_format)/8);
	g2d_free(read_buf);

	return 0;
}

static int g2d_int_from_double(double d)
{
	return wl_fixed_to_int(wl_fixed_from_double(d));
}

static void
repaint_region(struct weston_paint_node *pnode,
	       struct weston_output *output,
		   struct g2d_output_state *go,
	       struct clipper_quad *quads,
	       int nquads,
	       pixman_region32_t *region)
{
	struct g2d_renderer *gr = get_renderer(pnode->surface->compositor);
	struct g2d_surface_state *gs = get_surface_state(pnode->surface);
	struct weston_buffer *buffer = gs->buffer_ref.buffer;

	pixman_box32_t *rects, *bb_rects;
	int i, j, k, n, nrects, positions_size, nbb, cnt = 0;
	int nvtx = 0;
	g2dRECT srcRect = {0};
	g2dRECT dstrect = {0};
	g2dRECT clipRect = {0};
	int dstWidth = 0;
	int dstHeight = 0;
	struct clipper_vertex *positions;
	struct g2d_surfaceEx *dstsurface = go->drm_hw_buffer;
	struct g2d_surfaceEx srcsurface = gs->g2d_surface;
	uint32_t view_transform = pnode->surface->buffer_viewport.buffer.transform;
	int src_x = wl_fixed_to_int (pnode->surface->buffer_viewport.buffer.src_x);
	int src_y = wl_fixed_to_int (pnode->surface->buffer_viewport.buffer.src_y);
	int width = wl_fixed_to_int (pnode->surface->buffer_viewport.buffer.src_width);
	int height = wl_fixed_to_int (pnode->surface->buffer_viewport.buffer.src_height);
	int src_width = -1;
	int src_height = -1;
	int scale = pnode->surface->buffer_viewport.buffer.scale;
	const int nvtx_max = 8;
	if (pnode->view->alpha < 1.0) {
		/* Skip the render for global alpha, a workaround to disable the
		   fade effect, it created garbage info in the sequence test.*/
		return;
	}

	if (!gs->solid_clear) {
		if (srcsurface.base.width <= 0 || srcsurface.base.height <= 0) {
			return;
		}
	}

	bb_rects = pixman_region32_rectangles(&pnode->view->transform.boundingbox, &nbb);

	if(!gs->attached || nbb <= 0)
	{
		return;
	}

	convert_size_by_view_transform(&src_width, &src_height, width, height, view_transform);

	rects = pixman_region32_rectangles(region, &nrects);
	assert((nrects > 0) && (nquads > 0));

	/* Worst case allocation sizes per sub-mesh. */
	n = nquads * nrects;
	positions_size = n * nvtx_max * sizeof *positions;
	positions = wl_array_add(&gr->position_stream, positions_size);

	if(src_width != -1 && src_width > 0 && src_x >=0 && src_y >= 0
		&& src_x < gs->g2d_surface.base.width
		&& src_y < gs->g2d_surface.base.height)
	{
		srcRect.left = src_x * scale;
		srcRect.top = src_y * scale;
		srcRect.right = min (gs->g2d_surface.base.width, (src_x + src_width) * scale);
		srcRect.bottom = min (gs->g2d_surface.base.height, (src_y + src_height) * scale);
	}
	else
	{
		srcRect.left = srcsurface.base.left;
		srcRect.top  = srcsurface.base.top;
		srcRect.right  = srcsurface.base.right;
		srcRect.bottom = srcsurface.base.bottom;
	}

	dstWidth  = dstsurface->base.width;
	dstHeight = dstsurface->base.height;
	/*Calculate the destrect once for all*/
	dstrect.left = bb_rects[0].x1;
	dstrect.top = bb_rects[0].y1;
	dstrect.right = bb_rects[0].x2;
	dstrect.bottom = bb_rects[0].y2;
	/*Multi display support*/
	if(output->pos.c.x > 0)
	{
		dstrect.left = dstrect.left - g2d_int_from_double(output->pos.c.x);
		dstrect.right = dstrect.right - g2d_int_from_double(output->pos.c.x);
	}

	calculate_rect_with_transform(dstsurface->base.width,
					  dstsurface->base.height,
					  output->transform, &dstrect);

	/* Calculate the angle at which the frame buffer really needs to be rotated based
	 * on the rotation angle of the output and the angle set by the client.
	 */
	srcsurface.base.rot = convert_transform_to_rot(view_transform, output->transform);
	g2d_clip_rects(srcsurface.base.rot, &srcRect, &dstrect, dstWidth, dstHeight);

	for (i = 0; i < nquads; i++)
	{
		float min_x, max_x, min_y, max_y;
		for (j = 0; j < nrects; j++)
		{
			int m=0;

			/* A paint node with 'n' rects is accumulated in positions.
			 * Use index 'cnt' here to record the starting position of one node. */
			n = clipper_quad_clip_box32(&quads[i], &rects[j], &positions[nvtx]);
			nvtx += n;

			if (n < 3)
				continue;

			/* Convert weston coord from surface to global, a workaround
			 * for final region to be painted in the global coordinate. */
			for (k = cnt; k < nvtx; k++)
			{
				struct weston_coord c = {positions[k].x, positions[k].y};
				c = weston_matrix_transform_coord(&pnode->view->transform.matrix, c);
				positions[k].x = c.x;
				positions[k].y = c.y;
			}

			min_x = max_x = positions[cnt].x;
			min_y = max_y = positions[cnt].y;
			for (m = cnt + 1; m < nvtx; m++)
			{
				min_x = min(min_x, positions[m].x);
				max_x = max(max_x, positions[m].x);
				min_y = min(min_y, positions[m].y);
				max_y = max(max_y, positions[m].y);
			}
			cnt = nvtx;

			clipRect.left = g2d_int_from_double(min_x);
			clipRect.top = g2d_int_from_double(min_y);
			clipRect.right = g2d_int_from_double(max_x);
			clipRect.bottom = g2d_int_from_double(max_y);

			if(output->pos.c.x > 0)
			{
				clipRect.left = clipRect.left - g2d_int_from_double(output->pos.c.x);
				clipRect.right = clipRect.right - g2d_int_from_double(output->pos.c.x);
			}
			/* Need compute the clip rect with transform */
			calculate_rect_with_transform(dstsurface->base.width,
							  dstsurface->base.height,
							  output->transform, &clipRect);
			if(clipRect.left >= clipRect.right || clipRect.top >= clipRect.bottom)
			{
				return;
			}
			g2d_set_clipping(gr->handle, clipRect.left, clipRect.top, clipRect.right, clipRect.bottom);
			/* g2d_clear can't clear the sloid buffer with alpha.*/
			if (gs->solid_clear &&
			    buffer->type == WESTON_BUFFER_SOLID &&
			    buffer->pixel_format->format !=DRM_FORMAT_ARGB8888) {
				g2d_clear_solid(gr->handle, dstsurface, &clipRect, gs->clcolor);
			}
			else
			{
				g2d_blit_surface(gr->handle, &srcsurface, dstsurface, &srcRect, &dstrect);
			}

		}
	}

	gr->position_stream.size = 0;
}

static int sync_wait(int fd, int timeout)
{
    struct pollfd fds;
    int ret;

    if (fd < 0) {
        errno = EINVAL;
        return -1;
    }

    fds.fd = fd;
    fds.events = POLLIN;

    do {
        ret = poll(&fds, 1, timeout);
        if (ret > 0) {
            if (fds.revents & (POLLERR | POLLNVAL)) {
                errno = EINVAL;
                return -1;
            }
            return 0;
        } else if (ret == 0) {
            errno = ETIME;
            return -1;
        }
    } while (ret == -1 && (errno == EINTR || errno == EAGAIN));

    return ret;
}

static int
ensure_surface_buffer_is_ready(struct g2d_renderer *gr,
			       struct g2d_surface_state *gs)
{
	int ret = 0;
	struct weston_surface *surface = gs->surface;
	struct weston_buffer *buffer = gs->buffer_ref.buffer;

	if (!buffer)
		return 0;

	if(buffer->type == WESTON_BUFFER_RENDERER_OPAQUE) {
		if (!buffer->resource)
			return 0;

		/*Update vivBuffer and set g2d surface */
		struct wl_viv_buffer *vivBuffer = wl_resource_get_user_data(buffer->resource);

		if (gr->update_buffer)
			gr->update_buffer(gr->egl_display, (void *)buffer->resource, EGL_WAYLAND_BUFFER_WL);

		ret = get_g2dSurface(vivBuffer, &gs->g2d_surface);

		if (ret < 0)
			return ret;
	}

	if (surface->acquire_fence_fd < 0)
		return 0;

	ret = sync_wait(surface->acquire_fence_fd, 2000);

	if (ret < 0 && errno == ETIME){
		/* Print a warning. */
		weston_log("%s: Warning: wait for fence fd=%d", __func__, surface->acquire_fence_fd);

		/* Wait for ever. */
		ret = sync_wait(surface->acquire_fence_fd, -1);
	}

	return ret;
}

static bool
g2d_renderer_do_capture(struct weston_output *output, struct weston_buffer *into,
		       const struct weston_geometry *rect)
{
	struct wl_shm_buffer *shm = into->shm_buffer;
	const struct pixel_format_info *fmt = into->pixel_format;
	void *shm_pixels;
	void *read_target;
	int32_t stride;
	pixman_image_t *tmp = NULL;

	assert(into->type == WESTON_BUFFER_SHM);
	assert(shm);

	stride = wl_shm_buffer_get_stride(shm);
	if (stride % 4 != 0)
		return false;

	shm_pixels = wl_shm_buffer_get_data(shm);

	tmp = pixman_image_create_bits(fmt->pixman_format,
					   rect->width, rect->height,
					   NULL, 0);
	if (!tmp)
		return false;

	read_target = pixman_image_get_data(tmp);

	wl_shm_buffer_begin_access(shm);

	g2d_renderer_read_pixels(output, fmt, read_target, rect->x, rect->y, rect->width, rect->height);

	if (tmp) {
		pixman_image_t *shm_image;
		pixman_transform_t flip;

		shm_image = pixman_image_create_bits_no_clear(fmt->pixman_format,
							      rect->width,
							      rect->height,
							      shm_pixels,
							      stride);
		abort_oom_if_null(shm_image);

		pixman_transform_init_scale(&flip, pixman_fixed_1,
					    pixman_fixed_minus_1);
		pixman_transform_translate(&flip, NULL,	0,
					   pixman_int_to_fixed(rect->height));
		pixman_image_set_transform(tmp, &flip);

		pixman_image_composite32(PIXMAN_OP_SRC,
					 tmp,       /* src */
					 NULL,      /* mask */
					 shm_image, /* dest */
					 0, 0,      /* src x,y */
					 0, 0,      /* mask x,y */
					 0, 0,      /* dest x,y */
					 rect->width, rect->height);

		pixman_image_unref(shm_image);
		pixman_image_unref(tmp);
	}

	wl_shm_buffer_end_access(shm);

	return true;
}

static void
g2d_renderer_do_capture_tasks(struct weston_output *output,
			     enum weston_output_capture_source source)
{
	struct g2d_output_state *go = get_output_state(output);
	const struct pixel_format_info *format;
	struct weston_capture_task *ct;
	struct weston_geometry rect;

	switch (source) {
	case WESTON_OUTPUT_CAPTURE_SOURCE_FRAMEBUFFER:
		format = output->compositor->read_format;
		rect = go->area;
		rect.y = go->fb_size.height - go->area.y - go->area.height;
		break;
	case WESTON_OUTPUT_CAPTURE_SOURCE_FULL_FRAMEBUFFER:
		format = output->compositor->read_format;
		rect.x = 0;
		rect.y = 0;
		rect.width = go->fb_size.width;
		rect.height = go->fb_size.height;
		break;
	default:
		assert(0);
		return;
	}

	while ((ct = weston_output_pull_capture_task(output, source, rect.width,
						     rect.height, format))) {
		struct weston_buffer *buffer = weston_capture_task_get_buffer(ct);

		assert(buffer->width == rect.width);
		assert(buffer->height == rect.height);
		assert(buffer->pixel_format->format == format->format);

		if (buffer->type != WESTON_BUFFER_SHM ||
		    buffer->buffer_origin != ORIGIN_TOP_LEFT) {
			weston_capture_task_retire_failed(ct, "G2D: unsupported buffer");
			continue;
		}

		if (g2d_renderer_do_capture(output, buffer, &rect))
			weston_capture_task_retire_complete(ct);
		else
			weston_capture_task_retire_failed(ct, "G2D: capture failed");
	}
}

static void
global_to_surface(pixman_box32_t *rect, struct weston_view *ev,
		  struct clipper_vertex polygon[4])
{
	struct weston_coord_global rect_g[4] = {
		{ .c = weston_coord(rect->x1, rect->y1) },
		{ .c = weston_coord(rect->x2, rect->y1) },
		{ .c = weston_coord(rect->x2, rect->y2) },
		{ .c = weston_coord(rect->x1, rect->y2) },
	};
	struct weston_coord rect_s;
	int i;

	for (i = 0; i < 4; i++) {
		rect_s = weston_coord_global_to_surface(ev, rect_g[i]).c;
		polygon[i].x = (float)rect_s.x;
		polygon[i].y = (float)rect_s.y;
	}
}

static void
transform_damage(const struct weston_paint_node *pnode,
		 pixman_region32_t *region,
		 struct clipper_quad **quads,
		 int *nquads)
{
	pixman_box32_t *rects;
	int nrects, i;
	bool axis_aligned;
	struct clipper_quad *quads_alloc;
	struct clipper_vertex polygon[4];
	struct weston_view *view;

	if (*quads)
		return;

	rects = pixman_region32_rectangles(region, &nrects);

	assert(nrects > 0);
	*quads = quads_alloc = malloc(nrects * sizeof *quads_alloc);
	*nquads = nrects;

	view = pnode->view;
	axis_aligned = pnode->valid_transform;
	for (i = 0; i < nrects; i++) {
		global_to_surface(&rects[i], view, polygon);
		clipper_quad_init(&quads_alloc[i], polygon, axis_aligned);
	}
}

static void
draw_view(struct weston_paint_node *pnode,
		 pixman_region32_t *damage) /* in global coordinates */
{
	struct g2d_renderer *gr = get_renderer(pnode->surface->compositor);
	struct g2d_surface_state *gs = get_surface_state(pnode->surface);
	struct g2d_output_state *go = get_output_state(pnode->output);
	/* repaint bounding region in global coordinates: */
	pixman_region32_t repaint;
	/* opaque region in surface coordinates: */
	pixman_region32_t surface_opaque;
	/* non-opaque region in surface coordinates: */
	pixman_region32_t surface_blend;
	struct clipper_quad *quads = NULL;
	int nquads;

	pixman_region32_init(&repaint);
	pixman_region32_intersect(&repaint, &pnode->visible, damage);

	if (!pixman_region32_not_empty(&repaint))
		goto out;

	if (ensure_surface_buffer_is_ready(gr, gs) < 0)
		goto out;

	/* blended region is whole surface minus opaque region: */
	pixman_region32_init_rect(&surface_blend, 0, 0,
				  pnode->surface->width, pnode->surface->height);
	if (pnode->view->geometry.scissor_enabled)
		pixman_region32_intersect(&surface_blend, &surface_blend,
					  &pnode->view->geometry.scissor);
	pixman_region32_subtract(&surface_blend, &surface_blend,
				 &pnode->surface->opaque);

	/* XXX: Should we be using ev->transform.opaque here? */
	pixman_region32_init(&surface_opaque);
	if (pnode->view->geometry.scissor_enabled)
		pixman_region32_intersect(&surface_opaque,
					  &pnode->surface->opaque,
					  &pnode->view->geometry.scissor);
	else
		pixman_region32_copy(&surface_opaque, &pnode->surface->opaque);

	if (pixman_region32_not_empty(&surface_opaque)) {
		if (pnode->view->alpha < 1.0) {
			g2d_enable(gr->handle, G2D_BLEND);
			g2d_enable(gr->handle, G2D_GLOBAL_ALPHA);
			gs->g2d_surface.base.global_alpha = pnode->view->alpha * 0xFF;
		}
		transform_damage(pnode, &repaint, &quads, &nquads);
		repaint_region(pnode, pnode->output, go, quads, nquads, &surface_opaque);
		g2d_disable(gr->handle, G2D_GLOBAL_ALPHA);
		g2d_disable(gr->handle, G2D_BLEND);
	}

	if (pixman_region32_not_empty(&surface_blend)) {
		g2d_enable(gr->handle, G2D_BLEND);
		if (pnode->view->alpha < 1.0) {
			g2d_enable(gr->handle, G2D_GLOBAL_ALPHA);
			gs->g2d_surface.base.global_alpha = pnode->view->alpha * 0xFF;
		}
		transform_damage(pnode, &repaint, &quads, &nquads);
		repaint_region(pnode, pnode->output, go, quads, nquads, &surface_blend);
		g2d_disable(gr->handle, G2D_GLOBAL_ALPHA);
		g2d_disable(gr->handle, G2D_BLEND);
	}

	if (quads)
		free(quads);

	pixman_region32_fini(&surface_blend);
	pixman_region32_fini(&surface_opaque);

out:
	pixman_region32_fini(&repaint);
}

static void
repaint_views(struct weston_output *output, pixman_region32_t *damage)
{
	struct weston_paint_node *pnode;

	wl_list_for_each_reverse(pnode, &output->paint_node_z_order_list,
				 z_order_link) {
		if (pnode->plane == &output->primary_plane)
			draw_view(pnode, damage);
	}
}

static void
output_get_damage(struct weston_output *output,
		  pixman_region32_t *buffer_damage)
{
	struct g2d_output_state *go = get_output_state(output);
	int i;

	for (i = 0; i < BUFFER_DAMAGE_COUNT; i++)
		pixman_region32_union(buffer_damage,
			buffer_damage,
			&go->buffer_damage[i]);
}

static void
output_rotate_damage(struct weston_output *output,
			 pixman_region32_t *output_damage)
{
	struct g2d_output_state *go = get_output_state(output);

	go->current_buffer = (go->current_buffer + 1) % BUFFER_DAMAGE_COUNT;

	pixman_region32_copy(&go->buffer_damage[go->current_buffer], output_damage);
}

#if G2D_VERSION_MAJOR >= 2 && defined(BUILD_DRM_COMPOSITOR)
static void
g2d_update_buffer_release_fences(struct weston_output *output,
			     int fence_fd)
{
	struct weston_paint_node *pnode;

	wl_list_for_each_reverse(pnode, &output->paint_node_z_order_list,
			 z_order_link) {
		struct g2d_surface_state *gs;
		struct weston_buffer_release *buffer_release;

		if (pnode->plane != &output->primary_plane)
			continue;

		gs = get_surface_state(pnode->surface);
		buffer_release = gs->buffer_release_ref.buffer_release;

		if(!buffer_release) {
			continue;
		}

		/* If we have a buffer_release then it means we support fences,
		 * and we should be able to create the release fence. If we
		 * can't, something has gone horribly wrong, so disconnect the
		 * client.
		 */
		if (fence_fd == -1) {
			fd_clear(&buffer_release->fence_fd);
			continue;
		}

		fd_update(&buffer_release->fence_fd, dup(fence_fd));
	}
}
#endif

static void
g2d_renderer_repaint_output(struct weston_output *output,
				 pixman_region32_t *output_damage,
				 struct weston_renderbuffer *renderbuffer)
{
	struct weston_compositor *compositor = output->compositor;
	struct g2d_renderer *gr = get_renderer(compositor);
	pixman_region32_t buffer_damage, total_damage;
#if G2D_VERSION_MAJOR >= 2 && defined(BUILD_DRM_COMPOSITOR)
	struct g2d_output_state *go = get_output_state(output);
#endif
	int fence_fd = -1;

	pixman_region32_init(&total_damage);
	pixman_region32_init(&buffer_damage);

	output_get_damage(output, &buffer_damage);
	output_rotate_damage(output, output_damage);
	pixman_region32_union(&total_damage, &buffer_damage, output_damage);

	repaint_views(output, &total_damage);

	pixman_region32_fini(&total_damage);
	pixman_region32_fini(&buffer_damage);

	g2d_renderer_do_capture_tasks(output,
				     WESTON_OUTPUT_CAPTURE_SOURCE_FRAMEBUFFER);
	g2d_renderer_do_capture_tasks(output,
				     WESTON_OUTPUT_CAPTURE_SOURCE_FULL_FRAMEBUFFER);

#if G2D_VERSION_MAJOR >= 2 && defined(BUILD_DRM_COMPOSITOR)
	fence_fd = g2d_create_fence_fd(gr->handle);
	g2d_update_buffer_release_fences(output, fence_fd);

	fd_clear(&go->drm_hw_buffer->reserved[0]);
	go->drm_hw_buffer->reserved[0] = fence_fd;
#endif

	if(fence_fd == -1)
		g2d_finish(gr->handle);

	wl_signal_emit(&output->frame_signal, output_damage);
}

static bool
g2d_renderer_fill_buffer_info(struct weston_compositor *ec,
			     struct weston_buffer *buffer)
{
	struct g2d_renderer *gr = get_renderer(ec);
	EGLint format;
	uint32_t fourcc;
	bool ret = true;

	buffer->legacy_buffer = (struct wl_buffer *)buffer->resource;
	ret &= gr->query_buffer(gr->egl_display, buffer->legacy_buffer,
			        EGL_WIDTH, &buffer->width);
	ret &= gr->query_buffer(gr->egl_display, buffer->legacy_buffer,
				EGL_HEIGHT, &buffer->height);
	ret &= gr->query_buffer(gr->egl_display, buffer->legacy_buffer,
				EGL_TEXTURE_FORMAT, &format);
	if (!ret) {
		weston_log("eglQueryWaylandBufferWL failed\n");
		goto err_free;
	}

	/* The legacy EGL buffer interface only describes the channels we can
	 * sample from; not their depths or order. Take a stab at something
	 * which might be representative. Pessimise extremely hard for
	 * TEXTURE_EXTERNAL_OES. */
	switch (format) {
	case EGL_TEXTURE_RGB:
		fourcc = DRM_FORMAT_XRGB8888;
		break;
	case EGL_TEXTURE_RGBA:
		fourcc = DRM_FORMAT_ARGB8888;
		break;
	case EGL_TEXTURE_EXTERNAL_WL:
		fourcc = DRM_FORMAT_ARGB8888;
		break;
	case EGL_TEXTURE_Y_XUXV_WL:
		fourcc = DRM_FORMAT_YUYV;
		break;
	case EGL_TEXTURE_Y_UV_WL:
		fourcc = DRM_FORMAT_NV12;
		break;
	case EGL_TEXTURE_Y_U_V_WL:
		fourcc = DRM_FORMAT_YUV420;
		break;
	default:
		assert(0 && "not reached");
	}

	buffer->pixel_format = pixel_format_get_info(fourcc);
	assert(buffer->pixel_format);
	buffer->format_modifier = DRM_FORMAT_MOD_LINEAR;

	return true;

err_free:
	return false;

}

static void
g2d_renderer_attach_egl(struct weston_surface *es, struct weston_buffer *buffer)
{
	if (!buffer->resource)
		return;
	struct wl_viv_buffer *vivBuffer = wl_resource_get_user_data(buffer->resource);

	if (!vivBuffer)
		return;

	buffer->width = vivBuffer->width;
	buffer->height = vivBuffer->height;
}

static void
g2d_renderer_copy_shm_buffer(struct g2d_surface_state *gs, struct weston_buffer *buffer)
{
	int alignedWidth = ALIGN_TO_16(buffer->width);
	int height = 0;
	uint8_t *src = wl_shm_buffer_get_data(buffer->shm_buffer);
	uint8_t *dst = gs->shm_buf->buf_vaddr;
	int bpp = gs->bpp;
	int plane_size[3] = {0,};
	int src_plane_offset[3] = {0,};
	int dst_plane_offset[3] = {0,};
	int uv_src_stride = 0;
	int uv_dst_stride = 0;
	int n_planes = 0;
	int i, j;

	switch (wl_shm_buffer_get_format(buffer->shm_buffer)) {
		case WL_SHM_FORMAT_XRGB8888:
		case WL_SHM_FORMAT_XBGR8888:
		case WL_SHM_FORMAT_ARGB8888:
		case WL_SHM_FORMAT_ABGR8888:
		case WL_SHM_FORMAT_BGRX8888:
		case WL_SHM_FORMAT_BGRA8888:
		case WL_SHM_FORMAT_RGBA8888:
		case WL_SHM_FORMAT_RGBX8888:
		case WL_SHM_FORMAT_RGB565:
			n_planes = 1;
			height = buffer->height;
			plane_size[0] = wl_shm_buffer_get_stride(buffer->shm_buffer)*buffer->height;
			break;
		case WL_SHM_FORMAT_YUYV:
			n_planes = 1;
			height = ALIGN_TO_16(buffer->height);
			plane_size[0] = wl_shm_buffer_get_stride(buffer->shm_buffer)*buffer->height;
			break;
		case WL_SHM_FORMAT_UYVY:
			n_planes = 1;
			height = ALIGN_TO_16(buffer->height);
			plane_size[0] = wl_shm_buffer_get_stride(buffer->shm_buffer)*buffer->height;
			break;
		case WL_SHM_FORMAT_NV12:
			n_planes = 2;
			height = ALIGN_TO_16(buffer->height);
			plane_size[0] = wl_shm_buffer_get_stride(buffer->shm_buffer)*buffer->height;
			plane_size[1] = wl_shm_buffer_get_stride(buffer->shm_buffer)*buffer->height / 2;
			src_plane_offset[1] = plane_size[0];
			dst_plane_offset[1] = alignedWidth * height;
			uv_src_stride = wl_shm_buffer_get_stride(buffer->shm_buffer);
			uv_dst_stride = alignedWidth;
			break;
		case WL_SHM_FORMAT_NV21:
			n_planes = 2;
			height = ALIGN_TO_16(buffer->height);
			plane_size[0] = wl_shm_buffer_get_stride(buffer->shm_buffer)*buffer->height;
			plane_size[1] = wl_shm_buffer_get_stride(buffer->shm_buffer)*buffer->height / 2;
			src_plane_offset[1] = plane_size[0];
			dst_plane_offset[1] = alignedWidth * height;
			uv_src_stride = wl_shm_buffer_get_stride(buffer->shm_buffer);
			uv_dst_stride = alignedWidth;
			break;
		case WL_SHM_FORMAT_YUV420:
			n_planes = 3;
			height = ALIGN_TO_16(buffer->height);
			plane_size[0] = wl_shm_buffer_get_stride(buffer->shm_buffer)*buffer->height;
			plane_size[1] = wl_shm_buffer_get_stride(buffer->shm_buffer)*buffer->height / 4;
			plane_size[2] = plane_size[1];
			src_plane_offset[1] = plane_size[0];
			src_plane_offset[2] = plane_size[0] + plane_size[1];
			dst_plane_offset[1] = alignedWidth * height;
			dst_plane_offset[2] = dst_plane_offset[1] + alignedWidth * height / 4;
			uv_src_stride = wl_shm_buffer_get_stride(buffer->shm_buffer) / 2;
			uv_dst_stride = alignedWidth / 2;
			break;
		default:
			weston_log("warning: copy shm buffer meet unknown format: %08x\n",
				   wl_shm_buffer_get_format(buffer->shm_buffer));
			return;
	}

	wl_shm_buffer_begin_access(buffer->shm_buffer);
	if(alignedWidth == buffer->width && height == buffer->height)
	{
		for (i = 0; i < n_planes; i++)
			memcpy (dst + dst_plane_offset[i], src + src_plane_offset[i], plane_size[i]);
	}
	else
	{
		int src_stride = wl_shm_buffer_get_stride(buffer->shm_buffer);
		int dst_stride = alignedWidth * bpp;
		/* copy the 1st plane */
		for (i = 0; i < buffer->height; i++)
		{
			memcpy(dst + dst_plane_offset[0] + dst_stride * i, src + src_plane_offset[0] + src_stride * i, src_stride);
		}
		/* copy the rest plane */
		for (i = 1; i < n_planes; i++)
		{
			for (j = 0; j < buffer->height / 2; j++)
			{
				memcpy(dst + dst_plane_offset[i] + uv_dst_stride * j, src + src_plane_offset[i] + uv_src_stride * j, uv_src_stride);
			}
		}
	}
	wl_shm_buffer_end_access(buffer->shm_buffer);
}

static void
g2d_renderer_flush_damage(struct weston_paint_node *pnode)
{
	struct weston_surface *surface = pnode->surface;
	struct weston_buffer *buffer = surface->buffer_ref.buffer;
	struct g2d_surface_state *gs = get_surface_state(surface);
	bool texture_used;
	pixman_region32_union(&gs->texture_damage,
				  &gs->texture_damage, &surface->damage);

	if (!buffer)
		return;

	texture_used = false;
	wl_list_for_each(pnode, &surface->paint_node_list, surface_link) {
		if (!pnode->output || pnode->plane == &pnode->output->primary_plane) {
			texture_used = true;
			break;
		}
	}
	if (!texture_used)
		return;

	if (!pixman_region32_not_empty(&gs->texture_damage))
		goto done;

	if(wl_shm_buffer_get(buffer->resource))
	{
		g2d_renderer_copy_shm_buffer(gs, buffer);
	}

done:
	pixman_region32_fini(&gs->texture_damage);
	pixman_region32_init(&gs->texture_damage);

	weston_buffer_reference(&gs->buffer_ref, NULL, BUFFER_WILL_NOT_BE_ACCESSED);
	weston_buffer_release_reference(&gs->buffer_release_ref, NULL);
}

static uint32_t
pack_color(const uint32_t format, float *c)
{
	uint8_t r = round(c[0] * 255.0f);
	uint8_t g = round(c[1] * 255.0f);
	uint8_t b = round(c[2] * 255.0f);
	uint8_t a = round(c[3] * 255.0f);

	switch (format) {
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
		return (a << 24) | (b << 16) | (g << 8) | r;
	default:
		assert(0);
		return 0;
	}
}

static void
g2d_renderer_attach_solid(struct weston_surface *surface,
			struct weston_buffer *buffer)
{
	struct g2d_surface_state *gs = get_surface_state(surface);

	gs->color[0] = buffer->solid.r;
	gs->color[1] = buffer->solid.g;
	gs->color[2] = buffer->solid.b;
	gs->color[3] = buffer->solid.a;
	gs->solid_clear = true;
	gs->clcolor = pack_color(buffer->pixel_format->format, gs->color);
}

static void
g2d_renderer_attach_shm(struct weston_surface *es, struct weston_buffer *buffer)
{
	struct g2d_surface_state *gs = get_surface_state(es);
	struct wl_shm_buffer *shm_buffer = buffer->shm_buffer;
	int buffer_length = 0;
	int alloc_new_buff = 1;
	int alignedWidth = 0;
	int height = 0;
	enum g2d_format g2dFormat = 0;

	if (!shm_buffer)
		return;

	buffer->width = wl_shm_buffer_get_width(shm_buffer);
	buffer->height = wl_shm_buffer_get_height(shm_buffer);
	alignedWidth = ALIGN_TO_16(buffer->width);

	switch (wl_shm_buffer_get_format(shm_buffer)) {
	case WL_SHM_FORMAT_XRGB8888:
		g2dFormat = G2D_BGRX8888;
		gs->bpp = 4;
		break;
	case WL_SHM_FORMAT_XBGR8888:
		g2dFormat = G2D_RGBX8888;
		gs->bpp = 4;
		break;
	case WL_SHM_FORMAT_BGRX8888:
		g2dFormat = G2D_XRGB8888;
		gs->bpp = 4;
		break;
	case WL_SHM_FORMAT_RGBX8888:
		g2dFormat = G2D_XBGR8888;
		gs->bpp = 4;
		break;
	case WL_SHM_FORMAT_ARGB8888:
		g2dFormat = G2D_BGRA8888;
		gs->bpp = 4;
		break;
	case WL_SHM_FORMAT_ABGR8888:
		g2dFormat = G2D_RGBA8888;
		gs->bpp = 4;
		break;
	case WL_SHM_FORMAT_BGRA8888:
		g2dFormat = G2D_ARGB8888;
		gs->bpp = 4;
		break;
	case WL_SHM_FORMAT_RGBA8888:
		g2dFormat = G2D_ABGR8888;
		gs->bpp = 4;
		break;
	case WL_SHM_FORMAT_RGB565:
		g2dFormat = G2D_RGB565;
		gs->bpp = 2;
		break;
	case WL_SHM_FORMAT_BGR565:
		g2dFormat = G2D_BGR565;
		gs->bpp = 2;
		break;
	case WL_SHM_FORMAT_YUYV:
		g2dFormat = G2D_YUYV;
		height = ALIGN_TO_16(buffer->height);
		buffer_length = alignedWidth * height * 2;
		gs->bpp = 2;
		break;
	case WL_SHM_FORMAT_UYVY:
		g2dFormat = G2D_UYVY;
		height = ALIGN_TO_16(buffer->height);
		buffer_length = alignedWidth * height * 2;
		gs->bpp = 2;
		break;
	case WL_SHM_FORMAT_YUV420:
		g2dFormat = G2D_I420;
		height = ALIGN_TO_16(buffer->height);
		buffer_length = alignedWidth * height * 3/2;
		gs->bpp = 1;
		break;
	case WL_SHM_FORMAT_YVU420:
		g2dFormat = G2D_YV12;
		height = ALIGN_TO_16(buffer->height);
		buffer_length = alignedWidth * height * 3/2;
		gs->bpp = 1;
		break;
	case WL_SHM_FORMAT_NV12:
		g2dFormat = G2D_NV12;
		height = ALIGN_TO_16(buffer->height);
		buffer_length = alignedWidth * height * 3/2;
		gs->bpp = 1;
		break;
	case WL_SHM_FORMAT_NV16:
		g2dFormat = G2D_NV16;
		height = ALIGN_TO_16(buffer->height);
		buffer_length = alignedWidth * height * 3/2;
		gs->bpp = 1;
		break;
	case WL_SHM_FORMAT_NV21:
		g2dFormat = G2D_NV21;
		height = ALIGN_TO_16(buffer->height);
		buffer_length = alignedWidth * height * 3/2;
		gs->bpp = 1;
		break;
	default:
		weston_log("warning: unknown shm buffer format: %08x\n",
			   wl_shm_buffer_get_format(shm_buffer));
		return;
	}

	if (height == 0)
		height = buffer->height;

	if (buffer_length == 0)
		buffer_length = alignedWidth * buffer->height * gs->bpp;

	/* Only allocate a new g2d buff if it is larger than existing one.*/
	gs->shm_buf_length = buffer_length;
	if(gs->shm_buf && gs->shm_buf->buf_size > buffer_length)
	{
		alloc_new_buff = 0;
	}

	if(alloc_new_buff)
	{
		if(gs->shm_buf)
			g2d_free(gs->shm_buf);
		gs->shm_buf = g2d_alloc(buffer_length, 0);
		gs->g2d_surface.base.planes[0] = gs->shm_buf->buf_paddr;
		gs->g2d_surface.base.planes[1] = gs->g2d_surface.base.planes[0] + alignedWidth * height;
		gs->g2d_surface.base.planes[2] = gs->g2d_surface.base.planes[1] + alignedWidth * height / 4;
	}

	gs->g2d_surface.base.left = 0;
	gs->g2d_surface.base.top  = 0;
	gs->g2d_surface.base.right	= buffer->width;
	gs->g2d_surface.base.bottom = buffer->height;
	gs->g2d_surface.base.stride = alignedWidth;
	gs->g2d_surface.base.width	= buffer->width;
	gs->g2d_surface.base.height = height;
	gs->g2d_surface.base.rot	= G2D_ROTATION_0;
	gs->g2d_surface.base.clrcolor = 0xFF400000;
	gs->g2d_surface.tiling = G2D_LINEAR;
	gs->g2d_surface.base.format = g2dFormat;
}

static bool
g2d_renderer_resize_output(struct weston_output *output,
			  const struct weston_size *fb_size,
			  const struct weston_geometry *area)
{
	struct g2d_output_state *go = get_output_state(output);

	check_compositing_area(fb_size, area);

	go->fb_size = *fb_size;
	go->area = *area;

	weston_output_update_capture_info(output,
					  WESTON_OUTPUT_CAPTURE_SOURCE_FRAMEBUFFER,
					  area->width, area->height,
					  output->compositor->read_format);

	weston_output_update_capture_info(output,
					  WESTON_OUTPUT_CAPTURE_SOURCE_FULL_FRAMEBUFFER,
					  fb_size->width, fb_size->height,
					  output->compositor->read_format);

	return true;
}

static void
g2d_renderer_get_g2dformat_from_dmabuf(uint32_t dmaformat,
			enum g2d_format *g2dFormat, int *bpp)
{
	switch (dmaformat) {
		case DRM_FORMAT_ARGB8888:
			*g2dFormat = G2D_BGRA8888;
			*bpp = 4;
			break;
		case DRM_FORMAT_ABGR8888:
			*g2dFormat = G2D_RGBA8888;
			*bpp = 4;
			break;
		case DRM_FORMAT_BGRA8888:
			*g2dFormat = G2D_ARGB8888;
			*bpp = 4;
			break;
		case DRM_FORMAT_RGBA8888:
			*g2dFormat = G2D_ABGR8888;
			*bpp = 4;
			break;
		case DRM_FORMAT_XRGB8888:
			*g2dFormat = G2D_BGRX8888;
			*bpp = 4;
			break;
		case DRM_FORMAT_XBGR8888:
			*g2dFormat = G2D_RGBX8888;
			*bpp = 4;
			break;
		case DRM_FORMAT_BGRX8888:
			*g2dFormat = G2D_XRGB8888;
			*bpp = 4;
			break;
		case DRM_FORMAT_RGBX8888:
			*g2dFormat = G2D_XBGR8888;
			*bpp = 4;
			break;
		case DRM_FORMAT_RGB565:
			*g2dFormat = G2D_RGB565;
			*bpp = 2;
			break;
		case DRM_FORMAT_BGR565:
			*g2dFormat = G2D_BGR565;
			*bpp = 2;
			break;
		case DRM_FORMAT_YUYV:
			*g2dFormat = G2D_YUYV;
			*bpp = 2;
			break;
		case DRM_FORMAT_UYVY:
			*g2dFormat = G2D_UYVY;
			*bpp = 2;
			break;
		case DRM_FORMAT_NV12:
			*g2dFormat = G2D_NV12;
			*bpp = 1;
			break;
		case DRM_FORMAT_NV16:
			*g2dFormat = G2D_NV16;
			*bpp = 1;
			break;
		case DRM_FORMAT_NV21:
			*g2dFormat = G2D_NV21;
			*bpp = 1;
			break;
		case DRM_FORMAT_YUV420:
			*g2dFormat = G2D_I420;
			*bpp = 1;
			break;
		case DRM_FORMAT_YVU420:
			*g2dFormat = G2D_YV12;
			*bpp = 1;
			break;
		default:
			*g2dFormat = -1;
			weston_log("warning: unknown dmabuf buffer format: %08x\n", dmaformat);
			break;
	}
}

static void
g2d_renderer_attach_dmabuf(struct weston_surface *es, struct  weston_buffer *buffer)
{
	struct g2d_surface_state *gs = get_surface_state(es);
	struct linux_dmabuf_buffer *dmabuf = buffer->dmabuf;
	int alignedWidth = 0;
	enum g2d_format g2dFormat;
	unsigned int *paddr;
	int i = 0;
	int bpp = 1;

	if (!dmabuf)
		return;

	buffer->width = dmabuf->attributes.width;
	buffer->height = dmabuf->attributes.height;
	if(dmabuf->attributes.modifier == DRM_FORMAT_MOD_VIVANTE_SUPER_TILED ||
	   dmabuf->attributes.modifier == DRM_FORMAT_MOD_VIVANTE_SPLIT_SUPER_TILED) {
		alignedWidth  = ALIGN_TO_64(buffer->width);
	}
	g2d_renderer_get_g2dformat_from_dmabuf(dmabuf->attributes.format, &g2dFormat, &bpp);

	if (g2dFormat < 0)
		return;

	paddr = (unsigned int *)linux_dmabuf_buffer_get_user_data(dmabuf);
	for (i = 0; i < dmabuf->attributes.n_planes; i++) {
		gs->g2d_surface.base.planes[i] = paddr[i] + dmabuf->attributes.offset[i];
	}

	gs->g2d_surface.base.left = 0;
	gs->g2d_surface.base.top  = 0;
	gs->g2d_surface.base.right	= buffer->width;
	gs->g2d_surface.base.bottom = buffer->height;
	gs->g2d_surface.base.width	= buffer->width;
	gs->g2d_surface.base.height = buffer->height;
	gs->g2d_surface.base.rot	= G2D_ROTATION_0;
	if (dmabuf->attributes.modifier == DRM_FORMAT_MOD_AMPHION_TILED) {
		gs->g2d_surface.base.stride = dmabuf->attributes.stride[0];
		gs->g2d_surface.tiling = G2D_AMPHION_TILED;
	} else if(dmabuf->attributes.modifier == DRM_FORMAT_MOD_VIVANTE_SUPER_TILED ||
		      dmabuf->attributes.modifier == DRM_FORMAT_MOD_VIVANTE_SPLIT_SUPER_TILED){
		gs->g2d_surface.base.stride = alignedWidth;
		gs->g2d_surface.tiling = G2D_SUPERTILED;
	} else {
		gs->g2d_surface.base.stride = dmabuf->attributes.stride[0] / bpp;
		gs->g2d_surface.tiling = G2D_LINEAR;
	}
	gs->g2d_surface.base.format = g2dFormat;
}

static void
g2d_renderer_query_dmabuf_formats(struct weston_compositor *wc,
			int **formats, int *num_formats)
{
	struct g2d_renderer *gr = get_renderer(wc);
	int hardware_v1_available, hardware_v2_available, g2d_offset = 0;
	int num;
	static const int dma_formats[] = {
		DRM_FORMAT_ARGB8888,
		DRM_FORMAT_XRGB8888,
		DRM_FORMAT_RGB565,
		DRM_FORMAT_YUYV,
		DRM_FORMAT_UYVY,
		DRM_FORMAT_NV12,
		DRM_FORMAT_NV16,
		DRM_FORMAT_NV21,
		DRM_FORMAT_YUV420,
		DRM_FORMAT_YVU420,
		DRM_FORMAT_ABGR8888,
		DRM_FORMAT_BGRA8888,
		DRM_FORMAT_RGBA8888,
		DRM_FORMAT_XBGR8888,
		DRM_FORMAT_BGRX8888,
		DRM_FORMAT_RGBX8888,
	};

	g2d_query_hardware(gr->handle, G2D_HARDWARE_PXP_V1, &hardware_v1_available);
	g2d_query_hardware(gr->handle, G2D_HARDWARE_PXP_V2, &hardware_v2_available);
	if(hardware_v1_available == 1) {
		g2d_offset = 6;
	}
	else if(hardware_v2_available == 1) {
		g2d_offset = 2;
	}

	num = ARRAY_LENGTH(dma_formats) - g2d_offset;
	*formats = calloc(num, sizeof(int));
	memcpy(*formats, dma_formats, num * sizeof(int));

	*num_formats = num;
}

static void
g2d_renderer_query_dmabuf_modifiers(struct weston_compositor *wc, int format,
			uint64_t **modifiers,
			int *num_modifiers)
{
	struct g2d_renderer *gr = get_renderer(wc);
	int num;

	/*
	 * Set modifiers with DRM_FORMAT_MOD_LINEAR as default,
	 * if not support eglQueryDmaBufModifiersEXT.
	 */
	if (!gr->has_dmabuf_import_modifiers) {
		*num_modifiers = 1;
		*modifiers = calloc(*num_modifiers, sizeof(uint64_t));
		(*modifiers)[0] = DRM_FORMAT_MOD_LINEAR;
		return;
	}

	if (!gr->query_dmabuf_modifiers(gr->egl_display, format, 0, NULL,
					    NULL, &num) ||
		num == 0) {
		*num_modifiers = 0;
		return;
	}

	*modifiers = calloc(num, sizeof(uint64_t));
	if (*modifiers == NULL) {
		*num_modifiers = 0;
		return;
	}

	if (!gr->query_dmabuf_modifiers(gr->egl_display, format,
				num, *modifiers, NULL, &num)) {
		*num_modifiers = 0;
		free(*modifiers);
		return;
	}

	*num_modifiers = num;
}

static void
free_paddr_buf (struct linux_dmabuf_buffer *buffer)
{
	unsigned int * paddr = (unsigned int *)buffer->user_data;
	if (paddr)
		free (paddr);
}

static bool
g2d_renderer_import_dmabuf(struct weston_compositor *wc,
			struct linux_dmabuf_buffer *dmabuf)
{
	struct g2d_buf *g2dBuf = NULL;
	enum g2d_format g2dFormat;
	unsigned int *paddr = NULL;
	int i = 0;
	int bpp = 1;

	if (!dmabuf)
		return false;

	g2d_renderer_get_g2dformat_from_dmabuf(dmabuf->attributes.format, &g2dFormat, &bpp);
	if (g2dFormat < 0)
		return false;

	paddr = malloc (sizeof (unsigned int) * dmabuf->attributes.n_planes);
	if (!paddr)
		return false;

	for (i = 0; i < dmabuf->attributes.n_planes; i++) {
		if (g2dBuf)
			g2d_free(g2dBuf);
		g2dBuf = g2d_buf_from_fd(dmabuf->attributes.fd[i]);
		if(!g2dBuf)
			return false;
		paddr[i] = g2dBuf->buf_paddr;
	}

	if(!g2dBuf)
		return false;
	else
		g2d_free(g2dBuf);

	linux_dmabuf_buffer_set_user_data(dmabuf, (void *)paddr, free_paddr_buf);

	return true;
}

static const struct weston_drm_format_array *
g2d_renderer_get_supported_formats(struct weston_compositor *ec)
{
	struct g2d_renderer *gr = get_renderer(ec);

	return &gr->supported_formats;
}

static int
populate_supported_formats(struct weston_compositor *ec,
			   struct weston_drm_format_array *supported_formats)
{
	struct weston_drm_format *fmt;
	int *formats = NULL;
	uint64_t *modifiers = NULL;
	int num_formats, num_modifiers;
	int i, j;
	int ret = 0;

	/* Use EGL_EXT_image_dma_buf_import_modifiers to query the
	 * list of formats/modifiers of the renderer. */
	g2d_renderer_query_dmabuf_formats(ec, &formats, &num_formats);
	if (num_formats == 0)
		return 0;

	for (i = 0; i < num_formats; i++) {
		fmt = weston_drm_format_array_add_format(supported_formats,
							 formats[i]);
		if (!fmt) {
			ret = -1;
			goto out;
		}

		/* Always add DRM_FORMAT_MOD_INVALID, as EGL implementations
		 * support implicit modifiers. */
		ret = weston_drm_format_add_modifier(fmt, DRM_FORMAT_MOD_INVALID);
		if (ret < 0)
			goto out;

		g2d_renderer_query_dmabuf_modifiers(ec, formats[i],
						   &modifiers, &num_modifiers);
		if (num_modifiers == 0)
			continue;

		for (j = 0; j < num_modifiers; j++) {
			/* Skip MOD_INVALID, as it has already been added. */
			if (modifiers[j] == DRM_FORMAT_MOD_INVALID)
				continue;
			/* Only add 2D supported modifiers. */
			if (modifiers[j] == DRM_FORMAT_MOD_LINEAR ||
			    modifiers[j] == DRM_FORMAT_MOD_AMPHION_TILED ||
			    modifiers[j] == DRM_FORMAT_MOD_VIVANTE_SUPER_TILED ||
			    modifiers[j] == DRM_FORMAT_MOD_VIVANTE_SPLIT_SUPER_TILED) {
				ret = weston_drm_format_add_modifier(fmt, modifiers[j]);
				if (ret < 0) {
					free(modifiers);
					goto out;
				}
			}
		}
		free(modifiers);
	}

out:
	free(formats);
	return ret;
}

static void
g2d_renderer_attach(struct weston_paint_node *pnode)
{
	struct weston_surface *es = pnode->surface;
	struct weston_buffer *buffer = es->buffer_ref.buffer;
	struct g2d_surface_state *gs = get_surface_state(es);
	gs->solid_clear = false;

	if (!buffer) {
		gs->attached = 0;
		return;
	}

	switch (buffer->type) {
	case WESTON_BUFFER_SHM:
		g2d_renderer_attach_shm(es, buffer);
		break;
	case WESTON_BUFFER_DMABUF:
		g2d_renderer_attach_dmabuf(es, buffer);
		break;
	case WESTON_BUFFER_RENDERER_OPAQUE:
		g2d_renderer_attach_egl(es, buffer);
		break;
	case WESTON_BUFFER_SOLID:
		g2d_renderer_attach_solid(es, buffer);
		break;
	default:
		break;
	}
	gs->attached = 1;
	weston_buffer_reference(&gs->buffer_ref, buffer,
				BUFFER_MAY_BE_ACCESSED);
	weston_buffer_release_reference(&gs->buffer_release_ref,
					es->buffer_release_ref.buffer_release);
}

static void
surface_state_destroy(struct g2d_surface_state *gs, struct g2d_renderer *gr)
{
	wl_list_remove(&gs->surface_destroy_listener.link);
	wl_list_remove(&gs->renderer_destroy_listener.link);
	if(gs->surface)
		gs->surface->renderer_state = NULL;

	if(gs->shm_buf)
	{
		g2d_free(gs->shm_buf);
		gs->shm_buf = NULL;
	}
	if(gs->dma_buf)
	{
		g2d_free(gs->dma_buf);
		gs->dma_buf = NULL;
	}

	weston_buffer_reference(&gs->buffer_ref, NULL, BUFFER_WILL_NOT_BE_ACCESSED);
	weston_buffer_release_reference(&gs->buffer_release_ref, NULL);
	free(gs);
}

static void
surface_state_handle_surface_destroy(struct wl_listener *listener, void *data)
{
	struct g2d_surface_state *gs;
	struct g2d_renderer *gr;

	gs = container_of(listener, struct g2d_surface_state,
			  surface_destroy_listener);

	gr = get_renderer(gs->surface->compositor);
	surface_state_destroy(gs, gr);
}

static void
surface_state_handle_renderer_destroy(struct wl_listener *listener, void *data)
{
	struct g2d_surface_state *gs;
	struct g2d_renderer *gr;

	gr = data;

	gs = container_of(listener, struct g2d_surface_state,
			  renderer_destroy_listener);

	surface_state_destroy(gs, gr);
}


static int
g2d_renderer_create_surface(struct weston_surface *surface)
{
	struct g2d_surface_state *gs;
	struct g2d_renderer *gr = get_renderer(surface->compositor);

	gs = zalloc(sizeof *gs);
	if (gs == NULL)
		return -1;

	/* A buffer is never attached to solid color surfaces, yet
	 * they still go through texcoord computations. Do not divide
	 * by zero there.
	 */
	gs->surface = surface;

	surface->renderer_state = gs;

	gs->surface_destroy_listener.notify =
		surface_state_handle_surface_destroy;
	wl_signal_add(&surface->destroy_signal,
			  &gs->surface_destroy_listener);

	gs->renderer_destroy_listener.notify =
		surface_state_handle_renderer_destroy;
	wl_signal_add(&gr->destroy_signal,
			  &gs->renderer_destroy_listener);

	return 0;
}

/* create use-g2d-renderer */
static void
create_g2d_file()
{
	char *dir, *path;
	mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;

	dir = getenv("XDG_RUNTIME_DIR");
	path = malloc(strlen(dir) + 40);
	strcpy(path, dir);
	strcat(path, "/use-g2d-renderer");
	close(open(path, O_CREAT | O_RDWR, mode));
	free(path);
}

/* remove use-g2d-renderer */
static void
remove_g2d_file()
{
	char *dir, *path;

	dir = getenv("XDG_RUNTIME_DIR");
	path = malloc(strlen(dir) + 40);
	strcpy(path, dir);
	strcat(path, "/use-g2d-renderer");
	remove(path);
	free(path);
}

static void
g2d_renderer_output_destroy(struct weston_output *output)
{
	struct g2d_output_state *go = get_output_state(output);
	int i;

	for (i = 0; i < BUFFER_DAMAGE_COUNT; i++)
	{
		pixman_region32_fini(&go->buffer_damage[i]);
	}

#if G2D_VERSION_MAJOR >= 2 && defined(BUILD_DRM_COMPOSITOR)
	fd_clear(&go->drm_hw_buffer->reserved[0]);
#endif

	free(go);
}

static void
g2d_renderer_destroy(struct weston_compositor *ec)
{
	struct g2d_renderer *gr = get_renderer(ec);

	wl_signal_emit(&gr->destroy_signal, gr);
	g2d_close(gr->handle);
#ifdef ENABLE_EGL
	if(gr->has_bind_display)
		gr->unbind_display(gr->egl_display, gr->wl_display);
	eglTerminate(gr->egl_display);
#endif

	weston_drm_format_array_fini(&gr->supported_formats);

	wl_array_release(&gr->position_stream);

	free(ec->renderer);
	ec->renderer = NULL;

	remove_g2d_file();
}

static void
g2d_renderer_set_egl_device(struct g2d_renderer *gr)
{
	EGLAttrib attrib;
	const char *extensions;

	if (!gr->query_display_attrib(gr->egl_display, EGL_DEVICE_EXT, &attrib)) {
		weston_log("failed to get EGL device\n");
		return;
	}

	gr->egl_device = (EGLDeviceEXT) attrib;

	extensions = gr->query_device_string(gr->egl_device, EGL_EXTENSIONS);
	if (!extensions) {
		weston_log("failed to get EGL extensions\n");
		return;
	}

	/* Try to query the render node using EGL_DRM_RENDER_NODE_FILE_EXT */
	if (weston_check_egl_extension(extensions, "EGL_EXT_device_drm_render_node"))
		gr->drm_device = gr->query_device_string(gr->egl_device,
							 EGL_DRM_RENDER_NODE_FILE_EXT);

	/* The extension is not supported by the Mesa version of the system or
	 * the query failed. Fallback to EGL_DRM_DEVICE_FILE_EXT */
	if (!gr->drm_device && weston_check_egl_extension(extensions, "EGL_EXT_device_drm"))
		gr->drm_device = gr->query_device_string(gr->egl_device,
							 EGL_DRM_DEVICE_FILE_EXT);

	if (!gr->drm_device)
		weston_log("failed to query DRM device from EGL\n");
}

static int
g2d_renderer_setup_egl_display(struct g2d_renderer *gr,
			      void *native_window)
{
	gr->egl_display = NULL;

	if(get_platform_display)
		gr->egl_display = get_platform_display(EGL_PLATFORM_GBM_KHR,
				native_window, NULL);

	if (!gr->egl_display) {
		weston_log("failed to create display\n");
		return -1;
	}

	if (!eglInitialize(gr->egl_display, NULL, NULL)) {
		weston_log("failed to initialize display\n");
		return -1;
	}

	if (gr->has_device_query)
		g2d_renderer_set_egl_device(gr);

	return 0;
}

static int
g2d_renderer_setup_egl_client_extensions(struct g2d_renderer *gr)
{
	const char *extensions;

	extensions = eglQueryString(EGL_NO_DISPLAY, EGL_EXTENSIONS);
	if (!extensions) {
		weston_log("Retrieving EGL client extension string failed.\n");
		return -1;
	}

	if (weston_check_egl_extension(extensions, "EGL_EXT_device_query")) {
		gr->query_display_attrib =
			(void *) eglGetProcAddress("eglQueryDisplayAttribEXT");
		gr->query_device_string =
			(void *) eglGetProcAddress("eglQueryDeviceStringEXT");
		gr->has_device_query = true;
	}

	return 0;
}

static int
g2d_renderer_setup_egl_extensions(struct g2d_renderer *gr)
{
	const char *extensions;
	int ret;

	extensions =
		(const char *) eglQueryString(gr->egl_display, EGL_EXTENSIONS);
	if (!extensions) {
		weston_log("Retrieving EGL extension string failed.\n");
		return -1;
	}

	if (weston_check_egl_extension(extensions, "EGL_WL_bind_wayland_display"))
		gr->has_bind_display = true;
	if (gr->has_bind_display) {
		assert(gr->bind_display);
		assert(gr->unbind_display);
		assert(gr->query_buffer);
		ret = gr->bind_display(gr->egl_display, gr->wl_display);
		if (!ret)
			gr->has_bind_display = false;
	}

	if (weston_check_egl_extension(extensions,
				"EGL_EXT_image_dma_buf_import_modifiers")) {
		gr->query_dmabuf_formats =
			(void *) eglGetProcAddress("eglQueryDmaBufFormatsEXT");
		gr->query_dmabuf_modifiers =
			(void *) eglGetProcAddress("eglQueryDmaBufModifiersEXT");
		assert(gr->query_dmabuf_formats);
		assert(gr->query_dmabuf_modifiers);
		gr->has_dmabuf_import_modifiers = true;
	}

	return 0;
}

static int
create_default_dmabuf_feedback(struct weston_compositor *ec,
			       struct g2d_renderer *gr)
{
	struct stat dev_stat;
	struct weston_dmabuf_feedback_tranche *tranche;
	uint32_t flags = 0;

	if (stat(gr->drm_device, &dev_stat) != 0) {
		weston_log("%s: device disappeared, so we can't recover\n", __func__);
		abort();
	}

	ec->default_dmabuf_feedback =
		weston_dmabuf_feedback_create(dev_stat.st_rdev);
	if (!ec->default_dmabuf_feedback)
		return -1;

	tranche =
		weston_dmabuf_feedback_tranche_create(ec->default_dmabuf_feedback,
						      ec->dmabuf_feedback_format_table,
						      dev_stat.st_rdev, flags,
						      RENDERER_PREF);
	if (!tranche) {
		weston_dmabuf_feedback_destroy(ec->default_dmabuf_feedback);
		ec->default_dmabuf_feedback = NULL;
		return -1;
	}

	return 0;
}

static int
g2d_renderer_create(struct weston_compositor *ec)
{
	struct g2d_renderer *gr;
	int hardware_v1_available, hardware_v2_available = 0;

	gr = calloc(1, sizeof *gr);
	if (gr == NULL)
		return -1;

	weston_drm_format_array_init(&gr->supported_formats);

	gr->base.read_pixels = g2d_renderer_read_pixels;
	gr->base.repaint_output = g2d_renderer_repaint_output;
	gr->base.flush_damage = g2d_renderer_flush_damage;
	gr->base.resize_output = g2d_renderer_resize_output;
	gr->base.attach = g2d_renderer_attach;
	gr->base.destroy = g2d_renderer_destroy;
	gr->base.import_dmabuf = g2d_renderer_import_dmabuf;
	gr->base.get_supported_formats = g2d_renderer_get_supported_formats;
	gr->base.fill_buffer_info = g2d_renderer_fill_buffer_info;
	gr->base.type = WESTON_RENDERER_G2D;
	ec->renderer = &gr->base;
#ifdef ENABLE_EGL
	gr->bind_display =
		(void *) eglGetProcAddress("eglBindWaylandDisplayWL");
	gr->unbind_display =
		(void *) eglGetProcAddress("eglUnbindWaylandDisplayWL");
	gr->query_buffer =
		(void *) eglGetProcAddress("eglQueryWaylandBufferWL");
	gr->update_buffer =
		(void *) eglGetProcAddress("eglUpdateWaylandBufferWL");
	gr->query_display_attrib =
		(void *) eglGetProcAddress("eglQueryDisplayAttribEXT");
	gr->query_device_string =
		(void *) eglGetProcAddress("eglQueryDeviceStringEXT");
	if (!get_platform_display)
	{
		get_platform_display = (void *) eglGetProcAddress(
				"eglGetPlatformDisplayEXT");
	}

	ec->capabilities |= WESTON_CAP_EXPLICIT_SYNC;
#endif
	if(g2d_open(&gr->handle))
	{
		weston_log("g2d_open fail.\n");
		return -1;
	}

	ec->capabilities |= WESTON_CAP_ROTATION_ANY;
	ec->capabilities |= WESTON_CAP_CAPTURE_YFLIP;
	ec->capabilities |= WESTON_CAP_VIEW_CLIP_MASK;

	g2d_query_hardware(gr->handle, G2D_HARDWARE_PXP_V1, &hardware_v1_available);
	g2d_query_hardware(gr->handle, G2D_HARDWARE_PXP_V2, &hardware_v2_available);

	/* Configure read format to PIXMAN_x8r8g8b8 for pxp device */
	if (hardware_v1_available == 1) {
		ec->read_format = pixel_format_get_info_by_pixman(PIXMAN_x8r8g8b8);
	} else {
		ec->read_format = pixel_format_get_info_by_pixman(PIXMAN_a8r8g8b8);
	}

	wl_display_add_shm_format(ec->wl_display, WL_SHM_FORMAT_RGB565);
	wl_display_add_shm_format(ec->wl_display, WL_SHM_FORMAT_YUV420);
	wl_display_add_shm_format(ec->wl_display, WL_SHM_FORMAT_YVU420);
	wl_display_add_shm_format(ec->wl_display, WL_SHM_FORMAT_NV12);
	wl_display_add_shm_format(ec->wl_display, WL_SHM_FORMAT_NV16);
	wl_display_add_shm_format(ec->wl_display, WL_SHM_FORMAT_NV21);
	wl_display_add_shm_format(ec->wl_display, WL_SHM_FORMAT_YUYV);
	wl_display_add_shm_format(ec->wl_display, WL_SHM_FORMAT_UYVY);
	if(hardware_v1_available != 1 && hardware_v2_available != 1) {
		wl_display_add_shm_format(ec->wl_display, WL_SHM_FORMAT_BGRX8888);
		wl_display_add_shm_format(ec->wl_display, WL_SHM_FORMAT_RGBX8888);
	}
	if(hardware_v1_available != 1) {
		wl_display_add_shm_format(ec->wl_display, WL_SHM_FORMAT_BGRA8888);
		wl_display_add_shm_format(ec->wl_display, WL_SHM_FORMAT_RGBA8888);
		wl_display_add_shm_format(ec->wl_display, WL_SHM_FORMAT_ABGR8888);
		wl_display_add_shm_format(ec->wl_display, WL_SHM_FORMAT_XBGR8888);
	}

	wl_signal_init(&gr->destroy_signal);

	create_g2d_file();

	return 0;
}

static int
g2d_drm_display_create(struct weston_compositor *ec, void *native_window)
{
	struct g2d_renderer *gr;
#ifdef ENABLE_EGL
	int ret;
#endif

	if(g2d_renderer_create(ec) < 0)
	{
		weston_log("g2d_renderer_create faile.\n");
		return -1;
	}
#ifdef ENABLE_EGL
	gr = get_renderer(ec);
	gr->wl_display = ec->wl_display;

	if (g2d_renderer_setup_egl_client_extensions(gr) < 0)
		goto fail;

	if (g2d_renderer_setup_egl_display(gr, native_window) < 0)
		goto fail;

	if (g2d_renderer_setup_egl_extensions(gr) < 0)
		goto fail;

	ret = populate_supported_formats(ec, &gr->supported_formats);
	if (ret < 0)
		goto fail_terminate;

	if (gr->drm_device) {
		/* We support dma-buf feedback only when the renderer
		 * exposes a DRM-device */
		ec->dmabuf_feedback_format_table =
			weston_dmabuf_feedback_format_table_create(&gr->supported_formats);
		if (!ec->dmabuf_feedback_format_table)
			goto fail_terminate;
		ret = create_default_dmabuf_feedback(ec, gr);
		if (ret < 0)
			goto fail_feedback;
	}
#endif
	gr->use_drm = 1;

	if (linux_dmabuf_setup(ec) < 0)
		weston_log("Error: initializing dmabuf "
			   "support failed.\n");

	return 0;

fail_terminate:
	weston_drm_format_array_fini(&gr->supported_formats);
	eglTerminate(gr->egl_display);
fail_feedback:
	weston_dmabuf_feedback_format_table_destroy(ec->dmabuf_feedback_format_table);
	ec->dmabuf_feedback_format_table = NULL;
fail:
	free(gr);
	ec->renderer = NULL;

	return -1;
}

static void
g2d_renderer_output_set_buffer(struct weston_output *output, struct g2d_surfaceEx *buffer)
{
	struct g2d_output_state *go = get_output_state(output);
	go->drm_hw_buffer = buffer;
}

static int
g2d_renderer_get_surface_fence_fd(struct g2d_surfaceEx *buffer)
{
	return buffer->reserved[0];
}

static int
g2d_drm_renderer_output_create(struct weston_output *output,
				 const struct g2d_renderer_output_options *options)
{
	struct g2d_output_state *go;
	int i = 0;

	go = zalloc(sizeof *go);
	if (go == NULL)
		return -1;
	output->renderer_state = go;

	for (i = 0; i < BUFFER_DAMAGE_COUNT; i++)
		pixman_region32_init(&go->buffer_damage[i]);

	if (!g2d_renderer_resize_output(output, &options->fb_size, &options->area)) {
		weston_log("Output %s failed to create 16F shadow.\n",
			   output->name);
		output->renderer_state = NULL;
		free(go);
		return -1;
	}

	return 0;
 }

static int
drm_create_g2d_image(struct g2d_surfaceEx* g2dSurface,
				enum g2d_format g2dFormat,
				void *vaddr,
				int w, int h, int stride,
				int size,
				int dmafd)
{
	struct g2d_buf * buffer = NULL;

	buffer = g2d_buf_from_fd(dmafd);
	if (!buffer)
		return -1;

	buffer->buf_vaddr = vaddr;
	buffer->buf_size  = size;
	g2dSurface->base.planes[0] = buffer->buf_paddr;
	g2dSurface->base.left = 0;
	g2dSurface->base.top  = 0;
	g2dSurface->base.right	= w;
	g2dSurface->base.bottom = h;
	g2dSurface->base.stride = w;
	g2dSurface->base.width	= w;
	g2dSurface->base.height = h;
	g2dSurface->base.format = g2dFormat;
	g2dSurface->base.rot	= G2D_ROTATION_0;
	g2dSurface->base.clrcolor = 0xFF400000;
	g2dSurface->tiling = G2D_LINEAR;
	g2dSurface->reserved[0] = -1;

	return 0;
}

 WL_EXPORT struct g2d_renderer_interface g2d_renderer_interface = {
	.create = g2d_renderer_create,
	.drm_display_create  = g2d_drm_display_create,
	.drm_output_create   = g2d_drm_renderer_output_create,
	.create_g2d_image    = drm_create_g2d_image,
	.output_set_buffer   = g2d_renderer_output_set_buffer,
	.output_destroy      = g2d_renderer_output_destroy,
	.get_surface_fence_fd = g2d_renderer_get_surface_fence_fd,
};
