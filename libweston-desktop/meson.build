srcs_libdesktop = [
	'libweston-desktop.c',
	'client.c',
	'seat.c',
	'surface.c',
	'xwayland.c',
	'wl-shell.c',
	'xdg-shell-v5.c',
	'xdg-shell-v6.c',
	gen_xdg_shell5_server,
	gen_xdg_shell5_impl,
	gen_xdg_shell6_server,
	gen_xdg_shell6_impl,
]
lib_desktop = shared_library('weston-desktop-@0@'.format(libweston_major),
	srcs_libdesktop,
	include_directories:
		include_directories('..', '../libweston', '../shared'),
	install: true,
	version: '0.0.0',
	dependencies: dep_libweston)
dep_lib_desktop = declare_dependency(
	link_with: lib_desktop,
	dependencies: dep_libweston)

install_headers('libweston-desktop.h',
	subdir: 'libweston-@0@'.format(libweston_major))

configure_file(input: 'libweston-desktop.pc.in',
        output: 'libweston-desktop-@0@.pc'.format(libweston_major),
	configuration: config_pc,
	install: true,
	install_dir: dir_pc)
