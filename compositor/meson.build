srcs_weston = [
	git_version_h,
	'main.c',
	'text-backend.c',
	'weston-screenshooter.c',
	'../shared/config-parser.c',
	'../shared/option-parser.c',
	'../shared/os-compatibility.c',
	gen_text_input_server,
	gen_text_input_impl,
	gen_input_method_server,
	gen_input_method_impl,
	gen_shooter_server,
	gen_shooter_impl,
]
deps_weston = [
	dep_libweston,
	dep_libinput,
	dep_libdl,
]

if get_option('xwayland')
	srcs_weston += 'xwayland.c'
	config_h.set_quoted('XSERVER_PATH', get_option('xwayland-path'))
endif

if get_option('libunwind')
	deps_weston += dependency('libunwind')
	config_h.set('HAVE_LIBUNWIND', '1')
endif

exe_weston = executable('weston',
	srcs_weston,
	include_directories:
		include_directories('..', '../libweston', '../shared'),
	link_args: [ '-export-dynamic' ],
	dependencies: deps_weston,
	install: true)
install_headers('weston.h', subdir: 'weston')


if get_option('screenshare')
	srcs_screenshare = [
		'screen-share.c',
		gen_fullscreen_shell_server,
		gen_fullscreen_shell_client,
		gen_fullscreen_shell_impl,
	]
	deps_screenshare = [
		dep_libweston,
		dep_wayland_client,
	]
	plugin_screenshare = shared_library('screen-share',
		srcs_screenshare,
		include_directories:
			include_directories('..', '../libweston', '../shared'),
		dependencies: deps_screenshare,
		name_prefix: '',
		install: true,
		install_dir: dir_module_weston)
	env_modmap += 'screen-share.so=@0@;'.format(plugin_screenshare.full_path())
endif

if get_option('color-management-lcms')
	config_h.set('HAVE_LCMS', '1')

	srcs_lcms = [
		'cms-static.c',
		'cms-helper.c',
	]
	deps_lcms = [
		dep_libweston,
		dependency('lcms2'),
	]
	plugin_lcms = shared_library('cms-static',
		srcs_lcms,
		include_directories:
			include_directories('..', '../libweston', '../shared'),
		dependencies: deps_lcms,
		name_prefix: '',
		install: true,
		install_dir: dir_module_weston)
	env_modmap += 'cms-static.so=@0@;'.format(plugin_lcms.full_path())
endif

if get_option('color-management-colord')
	if not get_option('color-management-lcms')
		error('LCMS must be enabled to support colord')
	endif

	srcs_colord = [
		'cms-colord.c',
		'cms-helper.c',
	]
	deps_colord = [
		dep_libweston,
		dependency('colord', version: '>= 0.1.27')
	]
	plugin_colord = shared_library('cms-colord',
		srcs_colord,
		include_directories:
			include_directories('..', '../libweston', '../shared'),
		dependencies: deps_colord,
		name_prefix: '',
		install: true,
		install_dir: dir_module_weston)
	env_modmap += 'cms-colord.so=@0@;'.format(plugin_colord.full_path())
endif


weston_ini_config = configuration_data()
weston_ini_config.set('bindir', dir_bin)
weston_ini_config.set('libexecdir', dir_libexec)
weston_ini_config.set('abs_top_srcdir', meson.source_root())
weston_ini_config.set('abs_top_builddir', meson.build_root())
configure_file(input: '../weston.ini.in',
	output: 'weston.ini',
	install: true,
	install_dir: join_paths(get_option('sysconfdir'), 'xdg', 'weston'),
	configuration: weston_ini_config)
