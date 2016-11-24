if get_option('shell-desktop')
	config_h.set_quoted('WESTON_SHELL_CLIENT', 'weston-desktop-shell')

	srcs_shell_desktop = [
		'shell.c',
		'exposay.c',
		'input-panel.c',
		'../shared/matrix.c',
		'../shared/config-parser.c',
		gen_desktop_shell_server,
		gen_desktop_shell_impl,
		gen_input_method_server,
		gen_input_method_impl,
	]
	deps_shell_desktop = [
		dep_lib_desktop,
		dep_libweston,
	]
	plugin_shell_desktop = shared_library('desktop-shell',
		srcs_shell_desktop,
		include_directories:
			include_directories('..', '../libweston', '../shared'),
		dependencies: deps_shell_desktop,
		name_prefix: '',
		install: true,
		install_dir: dir_module_weston)
	env_modmap += 'desktop-shell.so=@0@;'.format(plugin_shell_desktop.full_path())
endif
