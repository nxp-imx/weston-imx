/*
 * Copyright 2021 Collabora, Ltd.
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

#include <libweston/libweston.h>

#include "color.h"
#include "color-properties.h"
#include "shared/helpers.h"
#include "shared/xalloc.h"
#include "shared/weston-assert.h"

struct cmnoop_color_profile {
	struct weston_color_profile base;
	struct weston_color_manager_noop *cmnoop;
};

struct weston_color_manager_noop {
	struct weston_color_manager base;
	struct cmnoop_color_profile *stock_cprof; /* no real content */
	struct weston_color_profile_params params;
	struct weston_hdr_metadata_type1 *hdr_meta;
	bool clean_hdr;
};

static bool
check_output_eotf_mode(struct weston_output *output)
{
	if (output->eotf_mode == WESTON_EOTF_MODE_SDR)
		return true;

	weston_log("Error: color manager no-op does not support EOTF mode %s of output %s.\n",
		   weston_eotf_mode_to_str(output->eotf_mode),
		   output->name);
	return false;
}

static struct weston_color_manager_noop *
to_cmnoop(struct weston_color_manager *cm_base)
{
	return container_of(cm_base, struct weston_color_manager_noop, base);
}

static inline struct cmnoop_color_profile *
to_cmnoop_cprof(struct weston_color_profile *cprof_base)
{
	return container_of(cprof_base, struct cmnoop_color_profile, base);
}

static struct cmnoop_color_profile *
ref_cprof(struct cmnoop_color_profile *cprof)
{
	if (!cprof)
		return NULL;

	weston_color_profile_ref(&cprof->base);
	return cprof;
}

static void
unref_cprof(struct cmnoop_color_profile *cprof)
{
	if (!cprof)
		return;

	weston_color_profile_unref(&cprof->base);
}

static void
cmnoop_color_profile_destroy(struct cmnoop_color_profile *cprof)
{
	free(cprof->base.description);
	free(cprof);
}

static void
cmnoop_destroy_color_profile(struct weston_color_profile *cprof_base)
{
	struct cmnoop_color_profile *cprof = to_cmnoop_cprof(cprof_base);
	struct weston_color_manager_noop *cmnoop = cprof->cmnoop;
	struct weston_color_profile_params *params;

	/* Clear hdr information if needed */
	if (cmnoop) {
		params = &cmnoop->params;
		if (params->tf.info && params->tf.info->tf == WESTON_TF_ST2084_PQ){
			cmnoop->clean_hdr = true;
		}
		memset(params, 0 , sizeof(*params));
	}
	cmnoop_color_profile_destroy(cprof);
}

static struct cmnoop_color_profile *
cmnoop_color_profile_create(struct weston_color_manager_noop *cm, char *desc)
{
	struct cmnoop_color_profile *cprof;

	cprof = xzalloc(sizeof *cprof);

	weston_color_profile_init(&cprof->base, &cm->base);
	cprof->base.description = desc;

	return cprof;
}

static struct weston_color_profile *
cmnoop_ref_stock_sRGB_color_profile(struct weston_color_manager *cm_base)
{
	struct weston_color_manager_noop *cm = to_cmnoop(cm_base);
	struct cmnoop_color_profile *cprof;

	cprof = ref_cprof(cm->stock_cprof);

	return &cprof->base;
}

static bool
cmnoop_get_color_profile_from_icc(struct weston_color_manager *cm,
				  const void *icc_data,
				  size_t icc_len,
				  const char *name_part,
				  struct weston_color_profile **cprof_out,
				  char **errmsg)
{
	*errmsg = xstrdup("Error: color manager no-op does not support ICC profiles.");
	return false;
}

static float
meta_clamp(float value, const char *valname, float min, float max)
{
	float ret = value;

	if (!(ret >= min))
		ret = min;

	if (!(ret <= max))
		ret = max;

	if (ret != value) {
		weston_log("clamping %s value from %f to %f.\n", valname, value, ret);
	}

	return ret;
}

static bool
cmnoop_get_hdr_meta_from_params(const struct weston_color_profile_params *params,
		    struct weston_hdr_metadata_type1 *hdr_meta)
{
	unsigned i;

	if (!params || !hdr_meta) {
		return false;
	}

	hdr_meta->group_mask = 0;
	if (params->tf.info->tf != WESTON_TF_ST2084_PQ) {
		weston_log ("no hdr meta, tf: %d\n", params->tf.info->tf);
		return false;
	}

	for (i = 0; i < 3; i++) {
		hdr_meta->primary[i].x = meta_clamp(params->target_primaries.primary[i].x,
							"primary", 0.0, 1.0);
		hdr_meta->primary[i].y = meta_clamp(params->target_primaries.primary[i].y,
							"primary", 0.0, 1.0);
	}
	hdr_meta->group_mask = WESTON_HDR_METADATA_TYPE1_GROUP_PRIMARIES;

	hdr_meta->white.x = meta_clamp(params->target_primaries.white_point.x, "white",
					       0.0, 1.0);
	hdr_meta->white.y = meta_clamp(params->target_primaries.white_point.y, "white",
						0.0, 1.0);
	hdr_meta->group_mask |= WESTON_HDR_METADATA_TYPE1_GROUP_WHITE;


	hdr_meta->maxDML = meta_clamp(params->target_max_luminance, "maxDML",
					      1.0, 65535.0);
	hdr_meta->group_mask |= WESTON_HDR_METADATA_TYPE1_GROUP_MAXDML;


	hdr_meta->minDML = meta_clamp(params->target_min_luminance, "minDML",
					      0.0001, 6.5535);
	hdr_meta->group_mask |= WESTON_HDR_METADATA_TYPE1_GROUP_MINDML;

	hdr_meta->maxCLL = meta_clamp(params->maxCLL, "maxCLL",
					       1.0, 65535.0);
	hdr_meta->group_mask |= WESTON_HDR_METADATA_TYPE1_GROUP_MAXCLL;

	hdr_meta->maxFALL = meta_clamp(params->maxFALL, "maxFALL",
					       1.0, 65535.0);
	hdr_meta->group_mask |= WESTON_HDR_METADATA_TYPE1_GROUP_MAXFALL;

	return true;
}

static bool
cmnoop_get_color_profile_from_params(struct weston_color_manager *cm,
				     const struct weston_color_profile_params *params,
				     const char *name_part,
				     struct weston_color_profile **cprof_out,
				     char **errmsg)
{
	char *desc;
	struct cmnoop_color_profile *cprof;
	struct weston_color_manager_noop *cmnoop = to_cmnoop(cm);

	cprof = xzalloc(sizeof *cprof);
	weston_color_profile_init(&cprof->base, cm);
	desc = xstrdup(name_part);
	cprof->base.description = desc;
	cprof->cmnoop = cmnoop;

	memcpy (&cmnoop->params, params, sizeof (cmnoop->params));
	if (cmnoop->hdr_meta == NULL)
		cmnoop->hdr_meta = xzalloc(sizeof *cmnoop->hdr_meta);
	cmnoop->clean_hdr = false;
	cmnoop_get_hdr_meta_from_params(params, cmnoop->hdr_meta);

	*cprof_out = &cprof->base;

	return true;
}

static int
cmnoop_get_hdr_data (struct weston_color_manager *cm,
		struct weston_hdr_metadata_type1 *hdr_meta, enum weston_eotf_mode *eotf_mode)
{
	struct weston_color_manager_noop *cmnoop = to_cmnoop(cm);
	int ret = -1;

	*eotf_mode = WESTON_EOTF_MODE_NONE;
	if (cmnoop->clean_hdr) {
		if (cmnoop->hdr_meta) {
			free (cmnoop->hdr_meta);
			cmnoop->hdr_meta = NULL;
		}
		*eotf_mode = WESTON_EOTF_MODE_SDR;
		cmnoop->clean_hdr = false;
		return 0;
	}

	if (!cmnoop->hdr_meta) {
		return ret;
	}

	if (!cmnoop->hdr_meta->group_mask) {
		*eotf_mode = WESTON_EOTF_MODE_SDR;
		ret = 0;
	} else {
		*eotf_mode = WESTON_EOTF_MODE_ST2084;
		ret = 1;
	}
	*hdr_meta = *cmnoop->hdr_meta;
	free (cmnoop->hdr_meta);
	cmnoop->hdr_meta = NULL;

	return ret;
}

static bool
cmnoop_send_image_desc_info(struct cm_image_desc_info *cm_image_desc_info,
			    struct weston_color_profile *cprof_base)
{
	return false;
}

static void
cmnoop_destroy_color_transform(struct weston_color_transform *xform)
{
	/* Never called, as never creates an actual color transform. */
}

static bool
cmnoop_get_surface_color_transform(struct weston_color_manager *cm_base,
				   struct weston_surface *surface,
				   struct weston_output *output,
				   struct weston_surface_color_transform *surf_xform)
{
	if (!check_output_eotf_mode(output))
		return false;

	/* Identity transform */
	surf_xform->transform = NULL;
	surf_xform->identity_pipeline = true;

	return true;
}

static struct weston_output_color_outcome *
cmnoop_create_output_color_outcome(struct weston_color_manager *cm_base,
				   struct weston_output *output)
{
	struct weston_compositor *compositor = cm_base->compositor;
	struct weston_color_manager_noop *cmnoop = to_cmnoop(cm_base);
	struct weston_output_color_outcome *co;

	weston_assert_ptr_not_null(compositor, output->color_profile);
	weston_assert_ptr_eq(compositor, to_cmnoop_cprof(output->color_profile),
			     cmnoop->stock_cprof);

	if (!check_output_eotf_mode(output))
		return NULL;

	co = xzalloc(sizeof *co);

	/* Identity transform on everything */
	co->from_blend_to_output = NULL;
	co->from_sRGB_to_blend = NULL;
	co->from_sRGB_to_output = NULL;

	co->hdr_meta.group_mask = 0;

	return co;
}

static bool
cmnoop_create_stock_profile(struct weston_color_manager_noop *cm)
{
	char *desc;

	desc = xstrdup("stock sRGB color profile");

	cm->stock_cprof = cmnoop_color_profile_create(cm, desc);
	if (!cm->stock_cprof) {
		free(desc);
		return false;
	}

	return true;
}

static bool
cmnoop_init(struct weston_color_manager *cm_base)
{
	struct weston_color_manager_noop *cm = to_cmnoop(cm_base);

	if (!cmnoop_create_stock_profile(cm))
		return false;

	cm->hdr_meta = NULL;
	cm->clean_hdr = false;
	/* No renderer requirements to check. */
	return true;
}

static void
cmnoop_destroy(struct weston_color_manager *cm_base)
{
	struct weston_color_manager_noop *cmnoop = to_cmnoop(cm_base);

	/* TODO: change this assert to make sure that ref_count is equal to 1.
	 * Currently we have a bug in which we leak surfaces when shutting down
	 * Weston with client surfaces alive, and these surfaces may have a
	 * reference to the stock sRGB profile. */
	weston_assert_s32_ge(cm_base->compositor,
			     cmnoop->stock_cprof->base.ref_count, 1);
	unref_cprof(cmnoop->stock_cprof);

	free(cmnoop->hdr_meta);
	free(cmnoop);
}

struct weston_color_manager *
weston_color_manager_noop_create(struct weston_compositor *compositor)
{
	struct weston_color_manager_noop *cm;

	cm = xzalloc(sizeof *cm);

	cm->base.name = "no-op";
	cm->base.compositor = compositor;
	cm->base.supports_client_protocol = true;
	cm->base.init = cmnoop_init;
	cm->base.destroy = cmnoop_destroy;
	cm->base.destroy_color_profile = cmnoop_destroy_color_profile;
	cm->base.ref_stock_sRGB_color_profile = cmnoop_ref_stock_sRGB_color_profile;
	cm->base.get_color_profile_from_icc = cmnoop_get_color_profile_from_icc;
	cm->base.get_color_profile_from_params = cmnoop_get_color_profile_from_params;
	cm->base.send_image_desc_info = cmnoop_send_image_desc_info;
	cm->base.destroy_color_transform = cmnoop_destroy_color_transform;
	cm->base.get_surface_color_transform = cmnoop_get_surface_color_transform;
	cm->base.create_output_color_outcome = cmnoop_create_output_color_outcome;
	cm->base.cm_get_hdr_data = cmnoop_get_hdr_data;

	cm->base.supported_color_features = (1 << WESTON_COLOR_FEATURE_PARAMETRIC) |
					    (1 << WESTON_COLOR_FEATURE_SET_PRIMARIES) |
					    (1 << WESTON_COLOR_FEATURE_SET_LUMINANCES) |
					    (1 << WESTON_COLOR_FEATURE_SET_MASTERING_DISPLAY_PRIMARIES);
	cm->base.supported_rendering_intents = (1 << WESTON_RENDER_INTENT_PERCEPTUAL);
	cm->base.supported_primaries_named = (1 << WESTON_PRIMARIES_CICP_SRGB) |
					     (1 << WESTON_PRIMARIES_CICP_NTSC) |
					     (1 << WESTON_PRIMARIES_CICP_BT2020);
	cm->base.supported_tf_named = (1 << WESTON_TF_SRGB) |
				      (1 << WESTON_TF_ST2084_PQ);

	return &cm->base;
}
