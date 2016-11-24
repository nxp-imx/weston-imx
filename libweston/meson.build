deps_libweston = [
	dep_wayland_server,
	dep_pixman,
	dep_libm,
	dep_libdl,
	dep_libdrm,
]
srcs_libweston = [
	git_version_h,
	'compositor.c',
	'plugin-registry.c',
	'input.c',
	'bindings.c',
	'log.c',
	'timeline.c',
	'data-device.c',
	'clipboard.c',
	'animation.c',
	'zoom.c',
	'noop-renderer.c',
	'pixman-renderer.c',
	'linux-dmabuf.c',
	'pixel-formats.c',
	'screenshooter.c',
	'../shared/file-util.c',
	'../shared/matrix.c',
	'../shared/os-compatibility.c',
	gen_dmabuf_server,
	gen_dmabuf_impl,
	gen_text_cursor_pos_server,
	gen_text_cursor_pos_impl,
	gen_text_input_server,
	gen_text_input_impl,
	gen_input_method_server,
	gen_input_method_impl,
	gen_presentation_time_server,
	gen_presentation_time_impl,
	gen_viewporter_impl,
	gen_viewporter_server,
	gen_rel_ptr_server,
	gen_rel_ptr_impl,
	gen_ptr_constraints_server,
	gen_ptr_constraints_impl,
	gen_shooter_server,
	gen_shooter_impl,
]

install_headers(
	'compositor.h',
	'windowed-output-api.h',
	'plugin-registry.h',
	'timeline-object.h',
	'../shared/matrix.h',
	'../shared/config-parser.h',
	'../shared/zalloc.h',
	subdir: 'libweston-@0@'.format(libweston_major))

if get_option('xkbcommon')
	deps_libweston += dep_xkbcommon
endif

lib_weston = shared_library('weston-@0@'.format(libweston_major),
	srcs_libweston,
	include_directories: include_directories('..', '../shared'),
	link_args: [ '-export-dynamic' ],
	install: true,
	version: '0.0.0',
	dependencies: deps_libweston)

dep_libweston = declare_dependency(
	link_with: lib_weston,
	dependencies: deps_libweston)

configure_file(input: 'libweston.pc.in',
               output: 'libweston-@0@.pc'.format(libweston_major),
	       configuration: config_pc,
	       install: true,
	       install_dir: dir_pc)


srcs_session_helper = [
	'launcher-util.c',
	'launcher-weston-launch.c',
	'launcher-direct.c',
]
deps_session_helper = [
	dep_libweston,
]

if get_option('backend-drm')
	deps_session_helper += dep_libdrm
endif

if get_option('launcher-logind')
	systemd_dep = dependency('libsystemd', version: '>= 209', required: false)
	if systemd_dep.found()
		config_h.set('HAVE_SYSTEMD_LOGIN_209', '1')
	else
		systemd_dep = dependency('libsystemd-login', version: '>= 198')
	endif
	config_h.set('HAVE_DBUS', '1')
	config_h.set('HAVE_SYSTEMD_LOGIN', '1')

	srcs_session_helper += [
		'dbus.c',
		'launcher-logind.c',
	]
	deps_session_helper += [
		dependency('dbus-1', version: '>= 1.6'),
		systemd_dep,
	]
endif

lib_session_helper = static_library('session-helper',
	srcs_session_helper,
	include_directories: include_directories('..', '../shared'),
	dependencies: deps_session_helper,
	install: false)
dep_session_helper = declare_dependency(link_with: lib_session_helper)


if get_option('backend-drm')
	srcs_drm = [
		'compositor-drm.c',
		'libbacklight.c',
		'libinput-seat.c',
		'libinput-device.c',
		gen_dmabuf_server,
		gen_dmabuf_impl,
		gen_presentation_time_server,
	]

	deps_drm = [
		dep_libweston,
		dep_session_helper,
		dep_libdrm,
		dep_libinput,
		dependency('libudev', version: '>= 136'),
	]

	if get_option('renderer-gl')
		dep_gbm = dependency('gbm')
		if dep_gbm.version().version_compare('>= 17.1')
			config_h.set('HAVE_GBM_MODIFIERS', '1')
		endif
		if dep_gbm.version().version_compare('>= 17.2')
			config_h.set('HAVE_GBM_FD_IMPORT', '1')
		endif
		deps_drm += dep_gbm
	endif

	if get_option('backend-drm-screencast-vaapi')
		srcs_drm += 'vaapi-recorder.c'
		deps_drm += [
			dependency('libva', version: '>= 0.34.0'),
			dependency('libva-drm', version: '>= 0.34.0'),
			dependency('threads'),
		]
		config_h.set('BUILD_VAAPI_RECORDER', '1')
	endif

	if dep_libdrm.version().version_compare('>= 2.4.71')
		config_h.set('HAVE_DRM_ADDFB2_MODIFIERS', '1')
	endif

	if dep_libdrm.version().version_compare('>= 2.4.78')
		config_h.set('HAVE_DRM_ATOMIC', '1')
	endif

	if dep_libdrm.version().version_compare('>= 2.4.83')
		config_h.set('HAVE_DRM_FORMATS_BLOB', '1')
	endif

	plugin_drm = shared_library('drm-backend',
		srcs_drm,
		include_directories: include_directories('..', '../shared'),
		dependencies: deps_drm,
		name_prefix: '',
		install: true,
		install_dir: dir_module_libweston)
	env_modmap += 'drm-backend.so=@0@;'.format(plugin_drm.full_path())

	install_headers('compositor-drm.h',
		subdir: 'libweston-@0@'.format(libweston_major))
endif


if get_option('backend-headless')
	srcs_headless = [
		'compositor-headless.c',
		gen_presentation_time_server,
	]
	plugin_headless = shared_library('headless-backend',
		srcs_headless,
		include_directories: include_directories('..', '../shared'),
		dependencies: dep_libweston,
		name_prefix: '',
		install: true,
		install_dir: dir_module_libweston,
	)
	env_modmap += 'headless-backend.so=@0@;'.format(plugin_headless.full_path())
	install_headers('compositor-headless.h',
		subdir: 'libweston-@0@'.format(libweston_major))
endif


if get_option('backend-rdp')
	dep_frdp = dependency('freerdp2', version: '>= 2.0.0', required: false)
	if not dep_frdp.found()
		dep_frdp = dependency('freerdp', version: '>= 1.1.0')
	endif

	if cc.has_header('freerdp/version.h', dependencies: dep_frdp)
		config_h.set('HAVE_FREERDP_VERSION_H', '1')
	endif

	deps_rdp = [
		dep_libweston,
		dep_frdp,
	]
	plugin_rdp = shared_library('rdp-backend',
		'compositor-rdp.c',
		include_directories: include_directories('..', '../shared'),
		dependencies: deps_rdp,
		name_prefix: '',
		install: true,
		install_dir: dir_module_libweston)
	env_modmap += 'rdp-backend.so=@0@;'.format(plugin_rdp.full_path())
	install_headers('compositor-rdp.h',
		subdir: 'libweston-@0@'.format(libweston_major))
endif


if get_option('backend-wayland')
	srcs_wlwl = [
		'compositor-wayland.c',
		'../shared/os-compatibility.c',
		gen_fullscreen_shell_impl,
		gen_fullscreen_shell_client,
		gen_xdg_shell6_client,
		gen_xdg_shell6_impl,
		gen_presentation_time_server,
		gen_presentation_time_impl,
	]

	deps_wlwl = [
		dependency('wayland-client'),
		dependency('wayland-cursor'),
		dep_pixman,
		dep_libweston,
		dep_lib_cairo_shared,
	]

	if get_option('renderer-gl')
		deps_wlwl += dependency('wayland-egl')
	endif

	plugin_wlwl = shared_library('wayland-backend',
		srcs_wlwl,
		include_directories: include_directories('..', '../shared'),
		dependencies: deps_wlwl,
		name_prefix: '',
		install: true,
		install_dir: dir_module_libweston)
	env_modmap += 'wayland-backend.so=@0@;'.format(plugin_wlwl.full_path())
	install_headers('compositor-wayland.h',
		subdir: 'libweston-@0@'.format(libweston_major))
endif


if get_option('backend-x11')
	srcs_x11 = [
		'compositor-x11.c',
		gen_presentation_time_server,
	]

	deps_x11 = [
		dependency('xcb', version: '>= 1.8'),
		dependency('xcb-shm'),
		dependency('x11'),
		dependency('x11-xcb'),
		dep_lib_cairo_shared,
		dep_pixman,
	]

	dep_xcb_xkb = dependency('xcb-xkb', version: '>= 1.9', required: false)
	if dep_xcb_xkb.found()
		deps_x11 += dep_xcb_xkb
		config_h.set('HAVE_XCB_XKB', '1')
	endif

	if get_option('renderer-gl')
		deps_x11 += dependency('egl')
	endif

	plugin_x11 = shared_library('x11-backend',
		srcs_x11,
		include_directories: include_directories('..', '../shared'),
		dependencies: deps_x11,
		name_prefix: '',
		install: true,
		install_dir: dir_module_libweston)
	env_modmap += 'x11-backend.so=@0@;'.format(plugin_x11.full_path())

	install_headers('compositor-x11.h',
		subdir: 'libweston-@0@'.format(libweston_major))
endif


if get_option('renderer-gl')
	config_h.set('ENABLE_EGL', '1')

	srcs_renderer_gl = [
		'gl-renderer.c',
		'vertex-clipping.c',
		'../shared/matrix.c',
		gen_dmabuf_server,
		gen_dmabuf_impl,
	]
	deps_renderer_gl = [
		dependency('egl'),
		dependency('glesv2'),
		dep_pixman,
		dep_libweston,
		dep_libdrm,
	]
	plugin_gl = shared_library('gl-renderer',
		srcs_renderer_gl,
		include_directories: include_directories('..', '../shared'),
		dependencies: deps_renderer_gl,
		name_prefix: '',
		install: true,
		install_dir: dir_module_libweston)
	env_modmap += 'gl-renderer.so=@0@;'.format(plugin_gl.full_path())
endif
