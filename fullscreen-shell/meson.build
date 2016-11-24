if get_option('shell-fullscreen')
	srcs_shell_fullscreen = [
		'fullscreen-shell.c',
		'../shared/matrix.c',
		gen_fullscreen_shell_server,
		gen_fullscreen_shell_impl,
	]
	shared_library('fullscreen-shell',
		srcs_shell_fullscreen,
		include_directories:
			include_directories('..', '../libweston', '../shared'),
		dependencies: dep_libweston,
		name_prefix: '',
		install: true,
		install_dir: dir_module_weston)
endif
