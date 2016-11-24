srcs_toytoolkit = [
	'window.c',
	'../shared/xalloc.c',
	'../shared/os-compatibility.c',
	'../shared/config-parser.c',
	'../shared/option-parser.c',
	gen_xdg_shell6_client,
	gen_xdg_shell6_impl,
	gen_text_cursor_pos_client,
	gen_text_cursor_pos_impl,
	gen_rel_ptr_client,
	gen_rel_ptr_impl,
	gen_ptr_constraints_client,
	gen_ptr_constraints_impl,
	gen_ivi_application_client,
	gen_ivi_application_impl,
]
deps_toytoolkit = [
	dep_wayland_client,
	dep_lib_cairo_shared,
	dep_xkbcommon,
	dependency('wayland-cursor'),
	cc.find_library('util'),
]
lib_toytoolkit = static_library('toytoolkit',
	srcs_toytoolkit,
	include_directories: include_directories('..', '../shared'),
	dependencies: deps_toytoolkit,
	install: false,
)
dep_toytoolkit = declare_dependency(
	link_with: lib_toytoolkit,
	dependencies: deps_toytoolkit,
)

if get_option('clients-simple')
	srcs_simple = [
		   '../shared/os-compatibility.c',
		   gen_xdg_shell6_client,
		   gen_xdg_shell6_impl,
		   gen_fullscreen_shell_client,
		   gen_fullscreen_shell_impl,
		   gen_ivi_application_client,
		   gen_ivi_application_impl,
	]
	deps_simple = [
		dep_wayland_client,
		dep_xkbcommon,
		dependency('wayland-cursor'),
	]

	executable('weston-simple-damage',
		   'simple-damage.c',
		   gen_viewporter_client,
		   gen_viewporter_impl,
	           srcs_simple,
		   include_directories: include_directories('..'),
		   dependencies: deps_simple,
		   install: false
	)
	executable('weston-simple-im',
		   'simple-im.c',
		   gen_input_method_client,
		   gen_input_method_impl,
	           srcs_simple,
		   include_directories: include_directories('..'),
		   dependencies: [
		   	deps_simple,
			dependency('cairo'),
		   ],
		   install: false
	)
	executable('weston-simple-shm',
		   'simple-shm.c',
	           srcs_simple,
		   include_directories: include_directories('..'),
		   dependencies: deps_simple,
		   install: false
	)
	executable('weston-simple-touch',
		   'simple-touch.c',
	           srcs_simple,
		   include_directories: include_directories('..'),
		   dependencies: deps_simple,
		   install: false
	)

	if get_option('clients-dmabuf-drm')
		executable('weston-simple-dmabuf-drm',
			   'simple-dmabuf-drm.c',
			   gen_dmabuf_client,
			   gen_dmabuf_impl,
			   srcs_simple,
			   include_directories: include_directories('..'),
			   dependencies: [
			   	deps_simple,
				dep_libdrm,
				dependency('egl'),
				dependency('libdrm_intel'),
			   ],
			   install: false
		)
	endif

	if get_option('clients-dmabuf-v4l2')
		executable('weston-simple-dmabuf-v4l2',
			   'simple-dmabuf-v4l.c',
			   gen_dmabuf_client,
			   gen_dmabuf_impl,
			   srcs_simple,
			   include_directories: include_directories('..'),
			   dependencies: [
			   	deps_simple,
				dep_libdrm,
			   ],
			   install: false
		)
	endif

	if get_option('clients-egl')
		executable('weston-simple-egl',
			'simple-egl.c',
			srcs_simple,
			gen_viewporter_client,
			gen_viewporter_impl,
			gen_presentation_time_client,
			gen_presentation_time_impl,
			include_directories:
				include_directories('..', '../shared'),
			dependencies: [
				deps_simple,
				dep_libm,
				dependency('egl'),
				dependency('glesv2'),
				dependency('wayland-egl'),
			],
			install: false
		)
	endif
endif

if get_option('clients-suite')
	toytoolkit_clients = [
		[
			'calibrator',
			[ '../shared/matrix.c' ]
		],
		['clickdot'],
		[
			'cliptest',
			[ '../libweston/vertex-clipping.c' ]
		],
		['confine'],
		['dnd'],
		[
			'editor',
			[
				gen_text_input_client,
				gen_text_input_impl,
			],
			[ dependency('pangocairo') ]
		],
		['eventdemo'],
		['flower'],
		[
			'fullscreen',
			[
				gen_fullscreen_shell_client,
				gen_fullscreen_shell_impl,
			]
		],
		['image'],
		[
			'weston-info',
			[
				gen_presentation_time_client,
				gen_presentation_time_impl,
				gen_dmabuf_client,
				gen_dmabuf_impl,
			]
		],
		['resizor'],
		[
			'scaler',
			[
				gen_viewporter_client,
				gen_viewporter_impl,
			]
		],
		['smoke'],
		['stacking'],
		['terminal'],
		['transformed'],
	]

	foreach t : toytoolkit_clients
		t_srcs = [ '@0@.c'.format(t[0]) ]
		if t.length() > 1
			t_srcs += t[1]
		endif

		t_deps = [ dep_toytoolkit ]
		if t.length() > 2
			t_deps += t[2]
		endif

		if t[0].startswith('weston-')
			t_name = t[0]
		else
			t_name = 'weston-@0@'.format(t[0])
		endif

		executable('weston-@0@'.format(t[0]),
		           t_srcs,
			   include_directories:
				include_directories('..', '../shared'),
		           dependencies: t_deps,
			   install: false
		)
	endforeach
endif


if get_option('shell-desktop')
	exe_keyboard = executable('weston-keyboard',
		   'keyboard.c',
		   gen_text_input_client,
		   gen_text_input_impl,
		   gen_input_method_client,
		   gen_input_method_impl,
		   include_directories: include_directories('..'),
		   dependencies: dep_toytoolkit,
		   install_dir: get_option('libexecdir'),
		   install: true
	)
	env_modmap += 'weston-keyboard=@0@;'.format(exe_keyboard.full_path())

	exe_shooter = executable('weston-screenshooter',
		   'screenshot.c',
		   gen_shooter_client,
		   gen_shooter_impl,
		   include_directories: include_directories('..'),
		   dependencies: dep_toytoolkit,
		   install_dir: get_option('libexecdir'),
		   install: true
	)
	env_modmap += 'weston-screenshooter=@0@;'.format(exe_shooter.full_path())

	exe_shell_desktop = executable('weston-desktop-shell',
		   'desktop-shell.c',
		   gen_desktop_shell_client,
		   gen_desktop_shell_impl,
		   include_directories: include_directories('..'),
		   dependencies: dep_toytoolkit,
		   install_dir: get_option('libexecdir'),
		   install: true
	)
	env_modmap += 'weston-desktop-shell=@0@;'.format(exe_shell_desktop.full_path())
endif


if get_option('shell-ivi')
	exe_shell_ivi_ui = executable('weston-ivi-shell-user-interface',
		   'ivi-shell-user-interface.c',
		   gen_ivi_hmi_client,
		   gen_ivi_hmi_impl,
		   gen_ivi_application_client,
		   gen_ivi_application_impl,
		   include_directories: include_directories('..'),
		   dependencies: dep_toytoolkit,
		   install_dir: get_option('libexecdir')
	)
	env_modmap += 'weston-ivi-shell-user-interface=@0@;'.format(exe_shell_ivi_ui.full_path())
endif
