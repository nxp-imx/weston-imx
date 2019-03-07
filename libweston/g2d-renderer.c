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

#define _GNU_SOURCE

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <float.h>
#include <math.h>
#include <assert.h>
#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <HAL/gc_hal.h>
#include <drm_fourcc.h>

#include "compositor.h"
#include "g2d-renderer.h"
#include "vertex-clipping.h"
#include "shared/helpers.h"
#include "linux-dmabuf.h"
#include "linux-dmabuf-unstable-v1-server-protocol.h"

#define BUFFER_DAMAGE_COUNT 3
#define ALIGN_TO_16(a) (((a) + 15) & ~15)

static PFNEGLGETPLATFORMDISPLAYEXTPROC get_platform_display = NULL;

struct wl_viv_buffer
{
	struct wl_resource *resource;
	gcoSURF  surface;
	gctINT32 width;
	gctINT32 height;
	gctINT32 format;
	gctUINT alignedWidth;
	gctUINT alignedHeight;
	gctUINT32 physical[3];
	gctUINT32 gpuBaseAddr;
	gceTILING tiling;
};

typedef struct _g2dRECT
{
	int left;
	int top;
	int right;
	int bottom;
} g2dRECT;

struct fb_screeninfo {
	struct fb_var_screeninfo varinfo;
	struct fb_fix_screeninfo fixinfo;
	unsigned int x_resolution;
	unsigned int y_resolution;
	size_t buffer_length; /* length of frame buffer memory in bytes */
	size_t physical;
	size_t stride;
	size_t stride_bytes;
	enum g2d_format pixel_format; /* frame buffer pixel format */
	int fb_fd;
};

struct g2d_output_state {
	int current_buffer;
	pixman_region32_t buffer_damage[BUFFER_DAMAGE_COUNT];
	struct g2d_surfaceEx *renderSurf;
	int nNumBuffers;
	int activebuffer;
	struct g2d_surfaceEx offscreenSurface;
	struct g2d_buf *offscreen_buf;
	struct fb_screeninfo fb_info;
	struct fb_screeninfo *mirror_fb_info;
	struct g2d_surfaceEx *mirrorSurf;
	struct g2d_surfaceEx *drm_hw_buffer;
	int directBlit;
	int clone_display_num;
	int width;
	int height;
};

struct g2d_surface_state {
	float color[4];
	struct weston_buffer_reference buffer_ref;
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
#ifdef ENABLE_EGL
	NativeDisplayType display;
	EGLDisplay egl_display;
	struct wl_display *wl_display;
	PFNEGLBINDWAYLANDDISPLAYWL bind_display;
	PFNEGLUNBINDWAYLANDDISPLAYWL unbind_display;
#endif
	void *handle;
	int use_drm;
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
/*
 * Compute the boundary vertices of the intersection of the global coordinate
 * aligned rectangle 'rect', and an arbitrary quadrilateral produced from
 * 'surf_rect' when transformed from surface coordinates into global coordinates.
 * The vertices are written to 'ex' and 'ey', and the return value is the
 * number of vertices. Vertices are produced in clockwise winding order.
 * Guarantees to produce either zero vertices, or 3-8 vertices with non-zero
 * polygon area.
 */
static int
calculate_edges(struct weston_view *ev, pixman_box32_t *rect,
		pixman_box32_t *surf_rect, float *ex, float *ey)
{

	struct clip_context ctx;
	int i, n;
	float min_x, max_x, min_y, max_y;
	struct polygon8 surf = {
		{ surf_rect->x1, surf_rect->x2, surf_rect->x2, surf_rect->x1 },
		{ surf_rect->y1, surf_rect->y1, surf_rect->y2, surf_rect->y2 },
		4
	};

	ctx.clip.x1 = rect->x1;
	ctx.clip.y1 = rect->y1;
	ctx.clip.x2 = rect->x2;
	ctx.clip.y2 = rect->y2;

	/* transform surface to screen space: */
	for (i = 0; i < surf.n; i++)
		weston_view_to_global_float(ev, surf.x[i], surf.y[i],
						&surf.x[i], &surf.y[i]);

	/* find bounding box: */
	min_x = max_x = surf.x[0];
	min_y = max_y = surf.y[0];

	for (i = 1; i < surf.n; i++) {
		min_x = min(min_x, surf.x[i]);
		max_x = max(max_x, surf.x[i]);
		min_y = min(min_y, surf.y[i]);
		max_y = max(max_y, surf.y[i]);
	}

	/* First, simple bounding box check to discard early transformed
	 * surface rects that do not intersect with the clip region:
	 */
	if ((min_x >= ctx.clip.x2) || (max_x <= ctx.clip.x1) ||
		(min_y >= ctx.clip.y2) || (max_y <= ctx.clip.y1))
		return 0;

	/* Simple case, bounding box edges are parallel to surface edges,
	 * there will be only four edges.  We just need to clip the surface
	 * vertices to the clip rect bounds:
	 */
	if (!ev->transform.enabled)
		return clip_simple(&ctx, &surf, ex, ey);

	/* Transformed case: use a general polygon clipping algorithm to
	 * clip the surface rectangle with each side of 'rect'.
	 * The algorithm is Sutherland-Hodgman, as explained in
	 * http://www.codeguru.com/cpp/misc/misc/graphics/article.php/c8965/Polygon-Clipping.htm
	 * but without looking at any of that code.
	 */
	n = clip_transformed(&ctx, &surf, ex, ey);

	if (n < 3)
		return 0;

	return n;
}

static void
calculate_rect_with_transform(int surfaceWidth, int surfaceHeight,
				  uint32_t transform, g2dRECT *rect)
{
	g2dRECT tmp = *rect;

	switch (transform) {
	case WL_OUTPUT_TRANSFORM_NORMAL:
	default:
		break;
	case WL_OUTPUT_TRANSFORM_90:
		rect->right = surfaceWidth - tmp.top;
		rect->left = rect->right - (tmp.bottom - tmp.top);
		rect->top = tmp.left;
		rect->bottom = rect->top + (tmp.right - tmp.left);
		break;
	case WL_OUTPUT_TRANSFORM_270:
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

static enum g2d_rotation
convert_transform_to_rot(uint32_t transform)
{
	enum g2d_rotation rot;
	switch(transform) {
	case WL_OUTPUT_TRANSFORM_NORMAL:
	default:
		rot = G2D_ROTATION_0;
		break;
	case WL_OUTPUT_TRANSFORM_90:
		/*For source rotation*/
		rot = G2D_ROTATION_270;
		break;
	case WL_OUTPUT_TRANSFORM_270:
		rot = G2D_ROTATION_90;
		break;
	case WL_OUTPUT_TRANSFORM_180:
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

static void
g2d_getG2dTiling(IN gceTILING tiling, enum g2d_tiling* g2dTiling)
{
	switch(tiling)
	{
	case gcvLINEAR:
		*g2dTiling = G2D_LINEAR;
		break;
	case gcvTILED:
		*g2dTiling = G2D_TILED;
		break;
	case gcvSUPERTILED:
		*g2dTiling = G2D_SUPERTILED;
		break;
	default:
		weston_log("Error in function %s\n", __func__);
		break;
	}
}

static void
g2d_getG2dFormat(IN gceSURF_FORMAT Format, enum g2d_format* g2dFormat)
{
	switch(Format)
	{
	case gcvSURF_R5G6B5:
		*g2dFormat = G2D_RGB565;
		break;
	case gcvSURF_A8B8G8R8:
		*g2dFormat = G2D_RGBA8888;
		break;
	case gcvSURF_X8B8G8R8:
		*g2dFormat = G2D_RGBA8888;
		break;
	case gcvSURF_A8R8G8B8:
		*g2dFormat = G2D_BGRA8888;
		break;
	case gcvSURF_X8R8G8B8:
		*g2dFormat = G2D_BGRX8888;
		break;
	case gcvSURF_B5G6R5:
		*g2dFormat = G2D_BGR565;
		break;
	case gcvSURF_B8G8R8A8:
		*g2dFormat = G2D_ARGB8888;
		break;
	case gcvSURF_R8G8B8A8:
		*g2dFormat = G2D_ABGR8888;
		break;
	case gcvSURF_B8G8R8X8:
		*g2dFormat = G2D_XRGB8888;
		break;
	case gcvSURF_R8G8B8X8:
		*g2dFormat = G2D_XBGR8888;
		break;
	case gcvSURF_NV12:
		*g2dFormat = G2D_NV12;
		break;
	case gcvSURF_NV21:
		*g2dFormat = G2D_NV21;
		break;
	case gcvSURF_I420:
		*g2dFormat = G2D_I420;
		break;
	case gcvSURF_YV12:
		*g2dFormat = G2D_YV12;
		break;
	case gcvSURF_YUY2:
		*g2dFormat = G2D_YUYV;
		break;
	case gcvSURF_YVYU:
		*g2dFormat = G2D_YVYU;
		break;
	case gcvSURF_UYVY:
		*g2dFormat = G2D_UYVY;
		break;
	case gcvSURF_VYUY:
		*g2dFormat = G2D_VYUY;
		break;
	case gcvSURF_NV16:
		*g2dFormat = G2D_NV16;
		break;
	case gcvSURF_NV61:
		*g2dFormat = G2D_NV61;
		break;
	default:
		weston_log("Error in function %s, Format not supported\n", __func__);
		break;
	}
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

static void
get_g2dSurface(struct wl_viv_buffer *buffer, struct g2d_surfaceEx *g2dSurface)
{
	if(buffer->width < 0 || buffer->height < 0)
	{
		weston_log("invalid EGL buffer in function %s\n", __func__);
		return;
	}
	g2d_getG2dFormat(buffer->format, &g2dSurface->base.format);
	g2d_getG2dTiling(buffer->tiling, &g2dSurface->tiling);
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
g2d_blit_surface(void *handle, struct g2d_surfaceEx * srcG2dSurface, struct g2d_surfaceEx *dstG2dSurface,
	g2dRECT *srcRect, g2dRECT *dstRect)
{
	g2d_SetSurfaceRect(srcG2dSurface, srcRect);
	g2d_SetSurfaceRect(dstG2dSurface, dstRect);
	srcG2dSurface->base.blendfunc = G2D_ONE;
	dstG2dSurface->base.blendfunc = G2D_ONE_MINUS_SRC_ALPHA;
	if(!(_hasAlpha(srcG2dSurface->base.format)))
	{
		g2d_disable(handle, G2D_BLEND);
	}

	if(g2d_blitEx(handle, srcG2dSurface, dstG2dSurface))
	{
		printG2dSurfaceInfo(srcG2dSurface, "SRC:");
		printG2dSurfaceInfo(dstG2dSurface, "DST:");
		return -1;
	}
	return 0;
}

static void
g2d_flip_surface(struct weston_output *output)
{
	struct g2d_output_state *go = get_output_state(output);
	go->fb_info.varinfo.yoffset  = go->activebuffer * go->fb_info.y_resolution;

	if(ioctl(go->fb_info.fb_fd, FBIOPAN_DISPLAY, &(go->fb_info.varinfo)) < 0)
	{
		weston_log("FBIOPAN_DISPLAY Failed\n");
	}
	go->activebuffer = (go->activebuffer + 1)  % go->nNumBuffers;
}

static void
copy_to_framebuffer(struct weston_output *output)
{
	struct g2d_renderer *gr = get_renderer(output->compositor);
	struct g2d_output_state *go = get_output_state(output);
	g2dRECT clipRect = {0};

	if((!go->directBlit && go->nNumBuffers == 1) || (go->clone_display_num))
	{
		clipRect.left	= output->previous_damage.extents.x1;
		clipRect.top	= output->previous_damage.extents.y1;
		clipRect.right	= output->previous_damage.extents.x2;
		clipRect.bottom = output->previous_damage.extents.y2;

		if(clipRect.left >= go->offscreenSurface.base.width)
		{
			clipRect.left -= go->offscreenSurface.base.width;
			clipRect.right -= go->offscreenSurface.base.width;
		}
	}

	if(!go->directBlit && go->nNumBuffers == 1)
	{
		g2dRECT srcRect  = {0, 0, go->offscreenSurface.base.width, go->offscreenSurface.base.height};
		g2dRECT dstrect  = srcRect;
		clipRect = srcRect;
		g2d_set_clipping(gr->handle, clipRect.left, clipRect.top, clipRect.right, clipRect.bottom);
		g2d_blit_surface(gr->handle, &go->offscreenSurface,
			&go->renderSurf[go->activebuffer], &srcRect, &dstrect);
	}

	if(go->clone_display_num)
	{
		int i = 0;
		for(i = 0; i < go->clone_display_num; i++)
		{
			g2dRECT srcRect  = {0, 0, go->renderSurf[go->activebuffer].base.width, go->renderSurf[go->activebuffer].base.height};
			g2dRECT dstrect  = {0, 0, go->mirrorSurf[i].base.width, go->mirrorSurf[i].base.height};
			if(go->directBlit || go->nNumBuffers > 1)
			{
				g2d_blit_surface(gr->handle, &go->renderSurf[go->activebuffer],
				&go->mirrorSurf[i], &srcRect, &dstrect);
			}
			else
			{
				g2d_blit_surface(gr->handle, &go->offscreenSurface,
					&go->mirrorSurf[i], &srcRect, &dstrect);
			}
		}
	}

	g2d_finish(gr->handle);

	if(go->nNumBuffers > 1)
	{
		g2d_flip_surface(output);
	}
}

static int
is_view_visible(struct weston_view *view)
{
	/* Return false, if surface is guaranteed to be totally obscured. */
	int ret;
	pixman_region32_t unocc;

	pixman_region32_init(&unocc);
	pixman_region32_subtract(&unocc, &view->transform.boundingbox,
				 &view->clip);
	ret = pixman_region32_not_empty(&unocc);
	pixman_region32_fini(&unocc);

	return ret;
}
 
static void
use_output(struct weston_output *output)
{
	struct weston_compositor *compositor = output->compositor;
	struct weston_view *view;
	struct g2d_output_state *go = get_output_state(output);
	int visibleViews=0;
	int fullscreenViews=0;

	if(go->nNumBuffers == 1)
	{
		wl_list_for_each_reverse(view, &compositor->view_list, link)
			if (view->plane == &compositor->primary_plane && is_view_visible(view))
			{
				visibleViews++;
				if(view->surface->width == go->width && view->surface->height == go->height)
				{
					pixman_box32_t *bb_rects;
					int nbb=0;
					bb_rects = pixman_region32_rectangles(&view->transform.boundingbox, &nbb);
					if(nbb == 1)
						if(bb_rects[0].x1 == 0 && bb_rects[0].y1 ==0)
							fullscreenViews++;
				}
			}

		go->directBlit = ((visibleViews == 1) || (fullscreenViews > 1));
	}
}

static void
g2d_clip_rects(enum wl_output_transform transform,
			g2dRECT *srcRect,
			g2dRECT *dstrect,
			int dstWidth,
			int dstHeight)
{
	int srcWidth = srcRect->right - srcRect->left;
	int srcHeight = srcRect->bottom - srcRect->top;
	float scale_v = 1.0f;
	float scale_h = 1.0f;

	if(transform == WL_OUTPUT_TRANSFORM_90
		|| transform == WL_OUTPUT_TRANSFORM_270)
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
	case WL_OUTPUT_TRANSFORM_NORMAL:
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
	case WL_OUTPUT_TRANSFORM_90:
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
			srcRect->left += floorf((float)(-dstrect->top) * scale_h);
			dstrect->top = 0;
			if(srcRect->left > srcRect->right)
				return;
		}
		break;
	case WL_OUTPUT_TRANSFORM_270:
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
	case WL_OUTPUT_TRANSFORM_180:
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
		break;
	default:
		break;
	}
}

static int
g2d_renderer_read_pixels(struct weston_output *output,
				   pixman_format_code_t format, void *pixels,
				   uint32_t x, uint32_t y,
				   uint32_t width, uint32_t height)
{
	return 0;
}

static int g2d_int_from_double(double d)
{
	return wl_fixed_to_int(wl_fixed_from_double(d));
}

static void
repaint_region(struct weston_view *ev, struct weston_output *output, struct g2d_output_state *go, pixman_region32_t *region,
		pixman_region32_t *surf_region){

	struct g2d_renderer *gr = get_renderer(ev->surface->compositor);
	struct g2d_surface_state *gs = get_surface_state(ev->surface);

	pixman_box32_t *rects, *surf_rects, *bb_rects;
	int i, j, nrects, nsurf, nbb=0;
	g2dRECT srcRect = {0};
	g2dRECT dstrect = {0};
	g2dRECT clipRect = {0};
	int dstWidth = 0;
	int dstHeight = 0;
	struct g2d_surfaceEx *dstsurface;
	struct g2d_surfaceEx srcsurface = gs->g2d_surface;
	uint32_t view_transform = ev->surface->buffer_viewport.buffer.transform;
	int src_x = wl_fixed_to_int (ev->surface->buffer_viewport.buffer.src_x);
	int src_y = wl_fixed_to_int (ev->surface->buffer_viewport.buffer.src_y);
	int src_width = wl_fixed_to_int (ev->surface->buffer_viewport.buffer.src_width);
	int src_height = wl_fixed_to_int (ev->surface->buffer_viewport.buffer.src_height);
	int scale = ev->surface->buffer_viewport.buffer.scale;
	if (ev->alpha < 1.0) {
		/* Skip the render for global alpha, a workaround to disable the
		   fade effect, it created garbage info in the sequence test.*/
		return;
	}

	bb_rects = pixman_region32_rectangles(&ev->transform.boundingbox, &nbb);

	if(!gs->attached || nbb <= 0)
	{
		return;
	}

	rects = pixman_region32_rectangles(region, &nrects);
	surf_rects = pixman_region32_rectangles(surf_region, &nsurf);
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
	if(go->drm_hw_buffer && gr->use_drm)
	{
		dstsurface = go->drm_hw_buffer;
	}
	else
	{
		if(go->nNumBuffers > 1 || go->directBlit)
		{
			dstsurface = &go->renderSurf[go->activebuffer];
		}
		else
		{
			dstsurface = &go->offscreenSurface;
		}
	}
	dstWidth  = dstsurface->base.width;
	dstHeight = dstsurface->base.height;
	/*Calculate the destrect once for all*/
	dstrect.left = bb_rects[0].x1;
	dstrect.top = bb_rects[0].y1;
	dstrect.right = bb_rects[0].x2;
	dstrect.bottom = bb_rects[0].y2;
	/*Multi display support*/
	if(output->x > 0)
	{
		dstrect.left = dstrect.left - output->x;
		dstrect.right = dstrect.right - output->x;
	}

	calculate_rect_with_transform(dstsurface->base.width,
					  dstsurface->base.height,
					  output->transform, &dstrect);

	if(view_transform != output->transform)
	{
		g2d_clip_rects(output->transform, &srcRect, &dstrect, dstWidth, dstHeight);
		srcsurface.base.rot = convert_transform_to_rot(output->transform);
	} else {
		/* if the view is transformed by compositor, we only need handle input crop
		 * according to dst rect */
		g2d_clip_rects(WL_OUTPUT_TRANSFORM_NORMAL, &srcRect, &dstrect, dstWidth, dstHeight);
	}

	for (i = 0; i < nrects; i++)
	{
		pixman_box32_t *rect = &rects[i];
		gctFLOAT min_x, max_x, min_y, max_y;

		for (j = 0; j < nsurf; j++)
		{
			pixman_box32_t *surf_rect = &surf_rects[j];
			gctFLOAT ex[8], ey[8];			/* edge points in screen space */
			int n;
			int m=0;
			n = calculate_edges(ev, rect, surf_rect, ex, ey);
			if (n < 3)
				continue;

			min_x = max_x = ex[0];
			min_y = max_y = ey[0];
			for (m = 1; m < n; m++)
			{
				min_x = min(min_x, ex[m]);
				max_x = max(max_x, ex[m]);
				min_y = min(min_y, ey[m]);
				max_y = max(max_y, ey[m]);
			}

			clipRect.left = g2d_int_from_double(min_x);
			clipRect.top = g2d_int_from_double(min_y);
			clipRect.right = g2d_int_from_double(max_x);
			clipRect.bottom = g2d_int_from_double(max_y);

			if(output->x > 0)
			{
				clipRect.left = clipRect.left - output->x;
				clipRect.right = clipRect.right - output->x;
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
			g2d_blit_surface(gr->handle, &srcsurface, dstsurface, &srcRect, &dstrect);
		}
	}
}

static void
draw_view(struct weston_view *ev, struct weston_output *output,
		 pixman_region32_t *damage) /* in global coordinates */
{
	struct weston_compositor *ec = ev->surface->compositor;	
	struct g2d_output_state *go = get_output_state(output);
	struct g2d_surface_state *gs = get_surface_state(ev->surface);
	struct g2d_renderer *gr = get_renderer(ec);
	/* repaint bounding region in global coordinates: */
	pixman_region32_t repaint;
	/* opaque region in surface coordinates: */
	pixman_region32_t surface_opaque;
	/* non-opaque region in surface coordinates: */
	pixman_region32_t surface_blend;

	pixman_region32_init(&repaint);
	pixman_region32_intersect(&repaint,
				  &ev->transform.boundingbox, damage);
	pixman_region32_subtract(&repaint, &repaint, &ev->clip);

	if (!pixman_region32_not_empty(&repaint))
		goto out;

	/* blended region is whole surface minus opaque region: */
	pixman_region32_init_rect(&surface_blend, 0, 0,
				  ev->surface->width, ev->surface->height);
	if (ev->geometry.scissor_enabled)
		pixman_region32_intersect(&surface_blend, &surface_blend,
					  &ev->geometry.scissor);
	pixman_region32_subtract(&surface_blend, &surface_blend,
				 &ev->surface->opaque);

	/* XXX: Should we be using ev->transform.opaque here? */
	pixman_region32_init(&surface_opaque);
	if (ev->geometry.scissor_enabled)
		pixman_region32_intersect(&surface_opaque,
					  &ev->surface->opaque,
					  &ev->geometry.scissor);
	else
		pixman_region32_copy(&surface_opaque, &ev->surface->opaque);

	if (pixman_region32_not_empty(&surface_opaque)) {
		if (ev->alpha < 1.0) {
			g2d_enable(gr->handle, G2D_BLEND);
			g2d_enable(gr->handle, G2D_GLOBAL_ALPHA);
			gs->g2d_surface.base.global_alpha = ev->alpha * 0xFF;
		}
		repaint_region(ev, output, go, &repaint, &surface_opaque);
		g2d_disable(gr->handle, G2D_GLOBAL_ALPHA);
		g2d_disable(gr->handle, G2D_BLEND);
	}

	if (pixman_region32_not_empty(&surface_blend)) {
		g2d_enable(gr->handle, G2D_BLEND);
		if (ev->alpha < 1.0) {
			g2d_enable(gr->handle, G2D_GLOBAL_ALPHA);
			gs->g2d_surface.base.global_alpha = ev->alpha * 0xFF;
		}
		repaint_region(ev, output, go, &repaint, &surface_blend);
		g2d_disable(gr->handle, G2D_GLOBAL_ALPHA);
		g2d_disable(gr->handle, G2D_BLEND);
	}
	pixman_region32_fini(&surface_blend);
	pixman_region32_fini(&surface_opaque);

out:
	pixman_region32_fini(&repaint);
}

static void
repaint_views(struct weston_output *output, pixman_region32_t *damage)
{
	struct weston_compositor *compositor = output->compositor;
	struct weston_view *view;

	wl_list_for_each_reverse(view, &compositor->view_list, link)
		if (view->plane == &compositor->primary_plane)
			draw_view(view, output, damage);
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

static void
g2d_renderer_repaint_output(struct weston_output *output,
				 pixman_region32_t *output_damage)
{
	struct weston_compositor *compositor = output->compositor;
	struct g2d_renderer *gr = get_renderer(compositor);
	pixman_region32_t buffer_damage, total_damage;

	use_output(output);

	pixman_region32_init(&total_damage);
	pixman_region32_init(&buffer_damage);

	output_get_damage(output, &buffer_damage);
	output_rotate_damage(output, output_damage);
	pixman_region32_union(&total_damage, &buffer_damage, output_damage);

	repaint_views(output, &total_damage);

	pixman_region32_fini(&total_damage);
	pixman_region32_fini(&buffer_damage);

	g2d_finish(gr->handle);

	pixman_region32_copy(&output->previous_damage, output_damage);
	wl_signal_emit(&output->frame_signal, output);
	if(!gr->use_drm)
		copy_to_framebuffer(output);
}

static void
g2d_renderer_attach_egl(struct weston_surface *es, struct weston_buffer *buffer)
{
	struct wl_viv_buffer *vivBuffer = wl_resource_get_user_data(buffer->resource);
	struct g2d_surface_state *gs = get_surface_state(es);
	buffer->width = vivBuffer->width;
	buffer->height = vivBuffer->height;
	get_g2dSurface(vivBuffer, &gs->g2d_surface);
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
		case WL_SHM_FORMAT_ARGB8888:
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
g2d_renderer_flush_damage(struct weston_surface *surface)
{
	struct g2d_surface_state *gs = get_surface_state(surface);
	struct weston_buffer *buffer = gs->buffer_ref.buffer;
	struct weston_view *view;
	int texture_used;
	pixman_region32_union(&gs->texture_damage,
				  &gs->texture_damage, &surface->damage);

	if (!buffer)
		return;

	texture_used = 0;
	wl_list_for_each(view, &surface->views, surface_link) {
		if (view->plane == &surface->compositor->primary_plane) {
			texture_used = 1;
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
	else
	{
		g2d_renderer_attach_egl(surface, buffer);
	}

done:
	pixman_region32_fini(&gs->texture_damage);
	pixman_region32_init(&gs->texture_damage);

	weston_buffer_reference(&gs->buffer_ref, NULL);
}

static void
g2d_renderer_attach_shm(struct weston_surface *es, struct weston_buffer *buffer,
			struct wl_shm_buffer *shm_buffer)
{
	struct g2d_surface_state *gs = get_surface_state(es);
	int buffer_length = 0;
	int alloc_new_buff = 1;
	int alignedWidth = 0;
	int height = 0;
	enum g2d_format g2dFormat = 0;
	buffer->shm_buffer = shm_buffer;
	buffer->width = wl_shm_buffer_get_width(shm_buffer);
	buffer->height = wl_shm_buffer_get_height(shm_buffer);
	alignedWidth = ALIGN_TO_16(buffer->width);

	switch (wl_shm_buffer_get_format(shm_buffer)) {
	case WL_SHM_FORMAT_XRGB8888:
		g2dFormat = G2D_BGRX8888;
		gs->bpp = 4;
		break;
	case WL_SHM_FORMAT_ARGB8888:
		g2dFormat = G2D_BGRA8888;
		gs->bpp = 4;
		break;
	case WL_SHM_FORMAT_RGB565:
		g2dFormat = G2D_RGB565;
		gs->bpp = 2;
		break;
	case WL_SHM_FORMAT_YUYV:
		g2dFormat = G2D_YUYV;
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
	case WL_SHM_FORMAT_NV12:
		g2dFormat = G2D_NV12;
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


static void
g2d_renderer_get_g2dformat_from_dmabuf(uint32_t dmaformat,
			enum g2d_format *g2dFormat)
{
	switch (dmaformat) {
		case DRM_FORMAT_ARGB8888:
			*g2dFormat = G2D_BGRA8888;
			break;
		case DRM_FORMAT_XRGB8888:
			*g2dFormat = G2D_BGRX8888;
			break;
		case DRM_FORMAT_RGB565:
			*g2dFormat = G2D_RGB565;
			break;
		case DRM_FORMAT_YUYV:
			*g2dFormat = G2D_YUYV;
			break;
		case DRM_FORMAT_NV12:
			*g2dFormat = G2D_NV12;
			break;
		case DRM_FORMAT_YUV420:
			*g2dFormat = G2D_I420;
			break;
		default:
			*g2dFormat = -1;
			weston_log("warning: unknown dmabuf buffer format: %08x\n", dmaformat);
			break;
	}
}

static void
g2d_renderer_attach_dmabuf(struct weston_surface *es, struct  weston_buffer *buffer,
			struct linux_dmabuf_buffer *dmabuf)
{
	struct g2d_surface_state *gs = get_surface_state(es);
	int alignedWidth = 0, alignedHeight = 0;
	enum g2d_format g2dFormat;
	gctUINT32 paddr = 0;
	buffer->width = dmabuf->attributes.width;
	buffer->height = dmabuf->attributes.height;
	alignedWidth = ALIGN_TO_16(buffer->width);
	alignedHeight = ALIGN_TO_16(buffer->height);

	g2d_renderer_get_g2dformat_from_dmabuf(dmabuf->attributes.format, &g2dFormat);

	if (g2dFormat < 0)
		return;

	if(gs->dma_buf)
		g2d_free(gs->dma_buf);
	gs->dma_buf = g2d_buf_from_fd(dmabuf->attributes.fd[0]);

	paddr = gs->dma_buf->buf_paddr;
	switch(g2dFormat){
		case G2D_I420:
			gs->g2d_surface.base.planes[0] = paddr;
			gs->g2d_surface.base.planes[1] = paddr + dmabuf->attributes.offset[1];
			gs->g2d_surface.base.planes[2] = paddr + dmabuf->attributes.offset[2];
			break;
		case G2D_NV12:
			gs->g2d_surface.base.planes[0] = paddr;
			gs->g2d_surface.base.planes[1] = paddr + dmabuf->attributes.offset[1];
			break;
		case G2D_YUYV:
		case G2D_RGB565:
		case G2D_BGRX8888:
		case G2D_BGRA8888:
			gs->g2d_surface.base.planes[0] = paddr;
			break;
		default:
			weston_log("G2D render not support format!\n");
			g2d_free(gs->dma_buf);
			gs->dma_buf = NULL;
			return;
	}

	gs->g2d_surface.base.left = 0;
	gs->g2d_surface.base.top  = 0;
	gs->g2d_surface.base.right	= buffer->width;
	gs->g2d_surface.base.bottom = buffer->height;
	gs->g2d_surface.base.stride = alignedWidth;
	gs->g2d_surface.base.width	= alignedWidth;
	gs->g2d_surface.base.height = alignedHeight;
	gs->g2d_surface.base.rot	= G2D_ROTATION_0;
	gs->g2d_surface.tiling = G2D_LINEAR;
	gs->g2d_surface.base.format = g2dFormat;
}

static void
g2d_renderer_query_dmabuf_formats(struct weston_compositor *wc,
			int **formats, int *num_formats)
{
	int num;
	static const int dma_formats[] = {
		DRM_FORMAT_ARGB8888,
		DRM_FORMAT_XRGB8888,
		DRM_FORMAT_RGB565,
		DRM_FORMAT_YUYV,
		DRM_FORMAT_NV12,
		DRM_FORMAT_YUV420,
	};

	num = ARRAY_LENGTH(dma_formats);
	*formats = calloc(num, sizeof(int));
	memcpy(*formats, dma_formats, num * sizeof(int));

	*num_formats = num;
}

static void
g2d_renderer_query_dmabuf_modifiers(struct weston_compositor *wc, int format,
			uint64_t **modifiers,
			int *num_modifiers)
{
	*num_modifiers = 1;
	*modifiers = calloc(*num_modifiers, sizeof(uint64_t));
	(*modifiers)[0] = DRM_FORMAT_MOD_LINEAR;
}

static bool
g2d_renderer_import_dmabuf(struct weston_compositor *wc,
			struct linux_dmabuf_buffer *dmabuf)
{
	struct g2d_buf *g2dBuf = NULL;
	enum g2d_format g2dFormat;

	if (!dmabuf)
		return false;

	g2d_renderer_get_g2dformat_from_dmabuf(dmabuf->attributes.format, &g2dFormat);
	if (g2dFormat < 0)
		return false;

	g2dBuf = g2d_buf_from_fd(dmabuf->attributes.fd[0]);

	if(!g2dBuf)
		return false;
	else
		g2d_free(g2dBuf);

	return true;
}

static void
g2d_renderer_attach(struct weston_surface *es, struct weston_buffer *buffer)
{
	struct g2d_surface_state *gs = get_surface_state(es);
	struct wl_shm_buffer *shm_buffer;
	struct linux_dmabuf_buffer *dmabuf;
	weston_buffer_reference(&gs->buffer_ref, buffer);

	if(buffer==NULL)
		return;

	shm_buffer = wl_shm_buffer_get(buffer->resource);

	if(shm_buffer)
	{
		g2d_renderer_attach_shm(es, buffer, shm_buffer);
	}
	else if ((dmabuf = linux_dmabuf_buffer_get(buffer->resource)))
	{
		g2d_renderer_attach_dmabuf(es, buffer, dmabuf);
	}
	else
	{
		g2d_renderer_attach_egl(es, buffer);
	}
	gs->attached = 1;
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

	weston_buffer_reference(&gs->buffer_ref, NULL);
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
	gs->pitch = 1;

	gs->surface = surface;

	pixman_region32_init(&gs->texture_damage);
	surface->renderer_state = gs;

	gs->surface_destroy_listener.notify =
		surface_state_handle_surface_destroy;
	wl_signal_add(&surface->destroy_signal,
			  &gs->surface_destroy_listener);

	gs->renderer_destroy_listener.notify =
		surface_state_handle_renderer_destroy;
	wl_signal_add(&gr->destroy_signal,
			  &gs->renderer_destroy_listener);

	if (surface->buffer_ref.buffer) {
		g2d_renderer_attach(surface, surface->buffer_ref.buffer);
		g2d_renderer_flush_damage(surface);
	}
	
	return 0;
}

static void
g2d_renderer_surface_set_color(struct weston_surface *surface,
		 float red, float green, float blue, float alpha)
{
	struct g2d_surface_state *gs = get_surface_state(surface);

	gs->color[0] = red;
	gs->color[1] = green;
	gs->color[2] = blue;
	gs->color[3] = alpha;
}


static void
g2d_renderer_output_destroy(struct weston_output *output)
{
	struct g2d_output_state *go = get_output_state(output);
	int i;

	for (i = 0; i < 2; i++)
	{
		pixman_region32_fini(&go->buffer_damage[i]);
	}

	if(go->offscreen_buf)
	{
		g2d_free(go->offscreen_buf);
		go->offscreen_buf = NULL;
	}

	if(go->fb_info.fb_fd)
	{
		close(go->fb_info.fb_fd);
		go->fb_info.fb_fd = 0;
	}

	if(go->renderSurf)
	{
		free(go->renderSurf);
		go->renderSurf = NULL;
	}
	for (i = 0; i < go->clone_display_num; i++)
	{
		if(go->mirror_fb_info[i].fb_fd)
		{
			close(go->mirror_fb_info[i].fb_fd);
			go->mirror_fb_info[i].fb_fd = 0;
		}
	}
	if(go->mirrorSurf)
	{
		free(go->mirrorSurf);
		go->mirrorSurf = NULL;
	}
	if(go->mirror_fb_info)
	{
		free(go->mirror_fb_info);
		go->mirror_fb_info = NULL;
	}

	free(go);
}

static void
g2d_renderer_destroy(struct weston_compositor *ec)
{
	struct g2d_renderer *gr = get_renderer(ec);

	wl_signal_emit(&gr->destroy_signal, gr);
	g2d_close(gr->handle);
#ifdef ENABLE_EGL
	if(gr->bind_display)
		gr->bind_display(gr->egl_display, gr->wl_display);
	eglTerminate(gr->egl_display);
	if(!gr->use_drm)
		fbDestroyDisplay(gr->display);
#endif
	free(ec->renderer);
	ec->renderer = NULL;

	/* remove use-g2d-renderer */
	char *dir, *path;
	dir = getenv("XDG_RUNTIME_DIR");
	path = malloc(strlen(dir) + 40);
	strcpy(path, dir);
	strcat(path, "/use-g2d-renderer");
	remove(path);
	free(path);
}

static int
g2d_renderer_create(struct weston_compositor *ec)
{
	struct g2d_renderer *gr;
	char *dir, *path;
	mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
	gr = calloc(1, sizeof *gr);
	if (gr == NULL)
		return -1;

	gr->base.read_pixels = g2d_renderer_read_pixels;
	gr->base.repaint_output = g2d_renderer_repaint_output;
	gr->base.flush_damage = g2d_renderer_flush_damage;
	gr->base.attach = g2d_renderer_attach;
	gr->base.surface_set_color = g2d_renderer_surface_set_color;
	gr->base.destroy = g2d_renderer_destroy;
	gr->base.import_dmabuf = g2d_renderer_import_dmabuf;
	gr->base.query_dmabuf_formats = g2d_renderer_query_dmabuf_formats;
	gr->base.query_dmabuf_modifiers = g2d_renderer_query_dmabuf_modifiers;
#ifdef ENABLE_EGL
	gr->bind_display =
		(void *) eglGetProcAddress("eglBindWaylandDisplayWL");
	gr->unbind_display =
		(void *) eglGetProcAddress("eglUnbindWaylandDisplayWL");
	if (!get_platform_display)
	{
		get_platform_display = (void *) eglGetProcAddress(
				"eglGetPlatformDisplayEXT");
	}
#endif
	if(g2d_open(&gr->handle))
	{
		weston_log("g2d_open fail.\n");
		return -1;
	}
	ec->renderer = &gr->base;
	ec->capabilities |= WESTON_CAP_VIEW_CLIP_MASK;

	wl_display_add_shm_format(ec->wl_display, WL_SHM_FORMAT_RGB565);
	wl_display_add_shm_format(ec->wl_display, WL_SHM_FORMAT_YUV420);
	wl_display_add_shm_format(ec->wl_display, WL_SHM_FORMAT_NV12);
	wl_display_add_shm_format(ec->wl_display, WL_SHM_FORMAT_YUYV);

	wl_signal_init(&gr->destroy_signal);

	/* create use-g2d-renderer */
	
	dir = getenv("XDG_RUNTIME_DIR");
	path = malloc(strlen(dir) + 40);
	strcpy(path, dir);
	strcat(path, "/use-g2d-renderer");
	close(open(path, O_CREAT | O_RDWR, mode));
	free(path);

	return 0;
}

static int
g2d_drm_display_create(struct weston_compositor *ec, void *native_window)
{
	struct g2d_renderer *gr;
	if(g2d_renderer_create(ec) < 0)
	{
		weston_log("g2d_renderer_create faile.\n");
		return -1;
	}
#ifdef ENABLE_EGL
	gr = get_renderer(ec);
	gr->wl_display = ec->wl_display;
	if(get_platform_display)
		gr->egl_display = get_platform_display(EGL_PLATFORM_GBM_KHR,
				native_window, NULL);
	if(gr->bind_display)
		gr->bind_display(gr->egl_display, gr->wl_display);
	gr->use_drm = 1;
#endif
	return 0;
}

static void
g2d_renderer_output_set_buffer(struct weston_output *output, struct g2d_surfaceEx *buffer)
{
	struct g2d_output_state *go = get_output_state(output);
	go->drm_hw_buffer = buffer;
}

static int
g2d_drm_renderer_output_create(struct weston_output *output)
{
	struct g2d_output_state *go;
	int i = 0;

	go = zalloc(sizeof *go);
	if (go == NULL)
		return -1;
	output->renderer_state = go;

	for (i = 0; i < BUFFER_DAMAGE_COUNT; i++)
		pixman_region32_init(&go->buffer_damage[i]);
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

	return 0;
}

static int
calculate_g2d_format(struct fb_var_screeninfo *varinfo, enum g2d_format *g2dFormat)
{
	/* Get the color format. */
	switch (varinfo->green.length)
	{
		case 6:
			*g2dFormat= G2D_RGB565;
			break;

		case 8:
			if (varinfo->blue.offset == 0)
			{
				*g2dFormat = (varinfo->transp.length == 0) ? G2D_BGRX8888 : G2D_BGRA8888;
			}
			else
			{
				*g2dFormat = (varinfo->transp.length == 0) ? G2D_RGBX8888 : G2D_RGBA8888;
			}
			break;

		default:
			*g2dFormat = -1;
			break;
	}
	return 0;
}

static int
get_G2dSurface_from_screeninfo(struct fb_screeninfo *info, struct g2d_surfaceEx* g2dSurface)
{
	if(info && g2dSurface)
	{
		g2dSurface->base.planes[0] = info->physical;
		g2dSurface->base.left = 0;
		g2dSurface->base.top  = 0;
		g2dSurface->base.right	= info->x_resolution;
		g2dSurface->base.bottom = info->y_resolution;
		g2dSurface->base.stride = info->stride;
		g2dSurface->base.width	= info->x_resolution;
		g2dSurface->base.height = info->y_resolution;
		g2dSurface->base.format = info->pixel_format;
		g2dSurface->base.rot	= G2D_ROTATION_0;
		g2dSurface->base.clrcolor = 0xFF400000;
		g2dSurface->tiling = G2D_LINEAR;
		return 0;
	}
	return -1;
}

static int
fb_query_screen_info(struct g2d_output_state *output, int fd,
						struct fb_screeninfo *info)
{
	struct g2d_output_state *go = output;
	struct fb_var_screeninfo *varinfo = &info->varinfo;
	struct fb_fix_screeninfo *fixinfo = &info->fixinfo;

	/* Probe the device for screen information. */
	if (ioctl(fd, FBIOGET_VSCREENINFO, varinfo) < 0) {
		return -1;
	}

	if(go->nNumBuffers > 1){
		varinfo->yres_virtual = varinfo->yres * go->nNumBuffers;
		if (ioctl(fd, FBIOPUT_VSCREENINFO, varinfo) < 0)
			return -1;
	}

	if (ioctl(fd, FBIOGET_FSCREENINFO, fixinfo) < 0 ||
		ioctl(fd, FBIOGET_VSCREENINFO, varinfo) < 0){
		return -1;
	}
	/* Store the pertinent data. */
	info->x_resolution = varinfo->xres;
	info->y_resolution = varinfo->yres;
	info->physical = fixinfo->smem_start;
	info->buffer_length = fixinfo->smem_len;
	info->stride = fixinfo->line_length / (varinfo->bits_per_pixel >> 3);
	info->stride_bytes = fixinfo->line_length;
	calculate_g2d_format(varinfo, &info->pixel_format);

	if (info->pixel_format < 0) {
		weston_log("Frame buffer uses an unsupported format.\n");
		return -1;
	}

	return 0;
}

static int
fb_frame_buffer_open(struct g2d_output_state *output, const char *fb_dev,
						struct fb_screeninfo *screen_info)
{
	/* Open the frame buffer device. */
	screen_info->fb_fd = open(fb_dev, O_RDWR | O_CLOEXEC);
	if (screen_info->fb_fd < 0) {
		weston_log("Failed to open frame buffer device%s \n", fb_dev);
		return -1;
	}

	/* Grab the screen info. */
	if (fb_query_screen_info(output, screen_info->fb_fd, screen_info) < 0) {
		weston_log("Failed to get frame buffer info \n");

		close(screen_info->fb_fd);
		return -1;
	}

	return 0;
}

static void
getBufferNumber(struct g2d_output_state *go)
{
	char *p = NULL;
	p = getenv("FB_MULTI_BUFFER");
	if (p == gcvNULL)
	{
		go->nNumBuffers = BUFFER_DAMAGE_COUNT;
	}
	else
	{
		go->nNumBuffers = atoi(p);
		if (go->nNumBuffers < 2)
		{
			go->nNumBuffers  = 1;
		}
		else if(go->nNumBuffers >= BUFFER_DAMAGE_COUNT)
		{
			go->nNumBuffers = BUFFER_DAMAGE_COUNT;
			go->activebuffer = 1;
		}
	}
	weston_log("The number of the Framebuffer: %d\n", go->nNumBuffers);
}

static int
g2d_renderer_surface_create(struct g2d_output_state *go,
		struct g2d_renderer *gr,
		const char *device)
{
	int i = 0;
	int offset = 0;
	weston_log("Opend device=%s\n", device);
	if(fb_frame_buffer_open(go, device, &go->fb_info) < 0)
	{
		weston_log("Open frame buffer failed.\n");
		return -1;
	}
	go->renderSurf = zalloc(sizeof(struct g2d_surfaceEx) * go->nNumBuffers);
	offset = go->fb_info.stride_bytes * go->fb_info.y_resolution;
	for(i = 0; i < go->nNumBuffers; i++)
	{
		get_G2dSurface_from_screeninfo(&go->fb_info, &go->renderSurf[i]);
		go->renderSurf[i].base.planes[0] = go->fb_info.physical
											+ (offset * i);
		g2d_clear(gr->handle, &go->renderSurf[i].base);
	}

	if(go->nNumBuffers == 1)
	{
		go->offscreenSurface = (go->renderSurf[go->activebuffer]);
		go->offscreen_buf = g2d_alloc(go->fb_info.buffer_length, 0);
		go->offscreenSurface.base.planes[0] = go->offscreen_buf->buf_paddr;
		g2d_clear(gr->handle, &go->offscreenSurface.base);
	}
	return 0;
}

static int
g2d_fbdev_renderer_output_create(struct weston_output *output,
		struct wl_display *wl_display,
		const char *device)
 {
	struct g2d_renderer *gr = get_renderer(output->compositor);
	struct g2d_output_state *go;
	int i;
	int clone_display_num = 0;
	int count = 0;
	int k=0, dispCount = 0;
	char displays[5][32];
	weston_log("g2d_renderer_output_create device=%s\n", device);
	count = strlen(device);

	if(count > 0)
	{
		for(i= 0; i < count; i++)
		{
			if(device[i] == ',')
			{
				displays[dispCount][k] = '\0';
				dispCount++;
				k = 0;
				continue;
			}
			else if(device[i] != ' ')
			{
				displays[dispCount][k++] = device[i];
			}
		}
		displays[dispCount][k] = '\0';
		clone_display_num = dispCount++;
		weston_log("clone_display_num = %d\n", clone_display_num);
	}
	else
	{
		weston_log("Invalid device name\n");
		return -1;
	}

	go = zalloc(sizeof *go);
	if (go == NULL)
		return -1;
	go->clone_display_num  = clone_display_num;
	output->renderer_state = go;

#ifdef ENABLE_EGL
	gr->wl_display = wl_display;
	gr->display = fbGetDisplay(wl_display);
	gr->egl_display = eglGetDisplay(gr->display);
	if(gr->bind_display)
		gr->bind_display(gr->egl_display, wl_display);
#endif
	getBufferNumber(go);

	if(g2d_renderer_surface_create(go, gr, displays[0]) < 0)
	{
		weston_log("Create Render surface failed.\n");
		return -1;
	}

	if(go->clone_display_num)
	{
		go->mirrorSurf = zalloc(sizeof(struct g2d_surfaceEx) * clone_display_num);
		go->mirror_fb_info = zalloc(sizeof(struct fb_screeninfo) * clone_display_num);
		if(go->mirrorSurf == NULL || go->mirror_fb_info == NULL)
			return -1;

		for(i = 0; i < clone_display_num; i++)
		{
			if(fb_frame_buffer_open(go, displays[i + 1], &go->mirror_fb_info[i]) < 0)
			{
				weston_log("Open frame buffer failed.\n");
				return -1;
			}
			get_G2dSurface_from_screeninfo(&go->mirror_fb_info[i], &go->mirrorSurf[i]);
			go->mirrorSurf[i].base.planes[0] = go->mirror_fb_info[i].physical;
			g2d_clear(gr->handle, &go->mirrorSurf[i].base);
		}
	}
	g2d_finish(gr->handle);
	if(go->renderSurf->base.format == G2D_RGB565)
		g2d_enable(gr->handle, G2D_DITHER);
	for (i = 0; i < BUFFER_DAMAGE_COUNT; i++)
		pixman_region32_init(&go->buffer_damage[i]);
	return 0;
 }

 WL_EXPORT struct g2d_renderer_interface g2d_renderer_interface = {
	.create = g2d_renderer_create,
	.drm_display_create  = g2d_drm_display_create,
	.drm_output_create   = g2d_drm_renderer_output_create,
	.fbdev_output_create = g2d_fbdev_renderer_output_create,
	.create_g2d_image    = drm_create_g2d_image,
	.output_set_buffer   = g2d_renderer_output_set_buffer,
	.output_destroy      = g2d_renderer_output_destroy,
};
