if get_option('shell-ivi')
	srcs_shell_ivi = [
		'ivi-shell.c',
		'ivi-layout.c',
		'ivi-layout-transition.c',
		'input-panel-ivi.c',
		'../shared/matrix.c',
		'../shared/option-parser.c',
		gen_ivi_application_server,
		gen_ivi_application_impl,
		gen_input_method_server,
		gen_input_method_impl,
	]
	plugin_shell_ivi = shared_library('ivi-shell',
		srcs_shell_ivi,
		include_directories:
			include_directories('..', '../libweston', '../shared'),
		dependencies: dep_libweston,
		name_prefix: '',
		install: true,
		install_dir: dir_module_weston)
	env_modmap += 'ivi-shell.so=@0@;'.format(plugin_shell_ivi.full_path())

	install_headers('ivi-layout-export.h', subdir: 'weston')

	srcs_ivi_hmi = [
		'hmi-controller.c',
		'../shared/config-parser.c',
		'../shared/xalloc.c',
		gen_ivi_hmi_server,
		gen_ivi_hmi_impl,
	]
	plugin_ivi_hmi = shared_library('hmi-controller',
		srcs_ivi_hmi,
		include_directories:
			include_directories('..', '../libweston', '../shared'),
		dependencies: dep_libweston,
		name_prefix: '',
		install: true,
		install_dir: dir_module_weston)
	env_modmap += 'hmi-controller.so=@0@;'.format(plugin_ivi_hmi.full_path())

	ivi_test_config = configuration_data()
	ivi_test_config.set('bindir', dir_bin)
	ivi_test_config.set('libexecdir', dir_libexec)
	ivi_test_config.set('abs_top_srcdir', meson.current_source_dir())
	ivi_test_config.set('abs_top_builddir', meson.current_build_dir())
	ivi_test_config.set('plugin_prefix', meson.current_build_dir())
	ivi_test_ini = configure_file(input: '../ivi-shell/weston.ini.in',
		output: 'weston-ivi-test.ini',
		install: false,
		configuration: weston_ini_config)
endif
