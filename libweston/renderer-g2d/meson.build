if not get_option('renderer-g2d')
	subdir_done()
endif

config_h.set('ENABLE_EGL', '1')
config_h.set('ENABLE_IMXG2D', '1')

srcs_renderer_g2d = [
	'g2d-renderer.c',
	linux_dmabuf_unstable_v1_protocol_c,
	linux_dmabuf_unstable_v1_server_protocol_h,
]

dep_g2d = cc.find_library('g2d')

deps_renderer_g2d = [
	dep_libm,
	dep_pixman,
	dep_libweston_private,
	dep_libdrm_headers,
	dep_vertex_clipping,
	dep_g2d,
]

foreach name : [ 'egl' ]
	d = dependency(name, required: false)
	if not d.found()
		error('g2d-renderer requires @0@ which was not found. Or, you can use \'-Drenderer-g2d=false\'.'.format(name))
	endif
	deps_renderer_g2d += d
endforeach

plugin_g2d = shared_library(
	'g2d-renderer',
	srcs_renderer_g2d,
	include_directories: common_inc,
	dependencies: deps_renderer_g2d,
	name_prefix: '',
	install: true,
	install_dir: dir_module_libweston
)
env_modmap += 'g2d-renderer.so=@0@;'.format(plugin_g2d.full_path())
