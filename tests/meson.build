plugin_test_shell_desktop = shared_library('weston-test-desktop-shell',
	'weston-test-desktop-shell.c',
	include_directories: include_directories('..', '../libweston', '../shared'),
	dependencies: [ dep_lib_desktop, dep_libweston ],
	name_prefix: '',
	install: false
)
env_modmap += 'weston-test-desktop-shell.so=@0@;'.format(plugin_test_shell_desktop.full_path())

lib_test_runner = static_library('test-runner',
	'weston-test-runner.c',
	include_directories: include_directories('..', '../shared'),
	install: false,
)
dep_test_runner = declare_dependency(link_with: lib_test_runner)

lib_test_client = static_library('test-client',
	'weston-test-client-helper.c',
	'../shared/xalloc.c',
	'../shared/os-compatibility.c',
	gen_weston_test_client,
	gen_weston_test_impl,
	include_directories: include_directories('..', '../shared'),
	dependencies: [
		dep_wayland_client,
		dep_pixman,
		dependency('cairo'),
	],
	install: false,
)
dep_test_client = declare_dependency(
	link_with: lib_test_client,
	dependencies: [
		dep_test_runner,
		dep_pixman,
	]
)

exe_plugin_test = shared_library('test-plugin',
	'weston-test.c',
	gen_weston_test_server,
	gen_weston_test_impl,
	include_directories:
		include_directories('..', '../shared', '../libweston'),
	dependencies: [
		dep_libweston,
		dependency('egl'),
	],
	name_prefix: '',
	install: false,
)

# FIXME: config-parser (zuc)
#        timespec (zuc)
#        surface-screenshot (manual)
#        matrix (manual: takes FOREVER!)
tests_standalone = [
	['string'],
	[
		'vertex-clip',
		[
			'../libweston/vertex-clipping.c'
		]
	],
]

tests_weston = [
	['bad-buffer'],
	['devices'],
	['event'],
	['keyboard'],
	['internal-screenshot'],
	[
		'presentation',
		[
			gen_presentation_time_client,
			gen_presentation_time_impl,
		]
	],
	['pointer'],
	['roles'],
	['subsurface'],
	['subsurface-shot'],
	[
		'text',
		[
			gen_text_input_client,
			gen_text_input_impl,
		]
	],
	['touch'],
	[
		'viewporter',
		[
			gen_viewporter_client,
			gen_viewporter_impl,
		]
	],
]

if get_option('xwayland')
	tests_weston += [ [ 'xwayland', [], dependency('x11') ] ]
endif

tests_weston_plugin = [
	['plugin-registry'],
	['surface'],
	['surface-global'],
]

if get_option('shell-ivi')
	tests_weston += [
		[
			'ivi-shell-app',
			[
				gen_ivi_application_client,
				gen_ivi_application_impl,
			]
		]
	]
	tests_weston_plugin += [
		['ivi-layout-internal'],
		[
			'ivi-layout',
			[
				gen_weston_test_server,
				gen_weston_test_impl,
			]
		],
	]

	exe_ivi_layout_client = executable('ivi-layout-test-client',
		'ivi-layout-test-client.c',
		gen_weston_test_client,
		gen_weston_test_impl,
		gen_ivi_application_client,
		gen_ivi_application_impl,
		include_directories:
			include_directories('..', '../shared', '../libweston'),
		dependencies: dep_test_client,
		install: false
	)

	env_modmap += 'weston-ivi-layout-test-client=@0@;'.format(exe_ivi_layout_client.full_path())
endif

env_test_weston = [
	'WESTON_TEST_REFERENCE_PATH=@0@/reference'.format(meson.current_source_dir()),
	'WESTON_MODULE_MAP=@0@'.format(env_modmap),
]

# FIXME: the multiple loops is lame. rethink this.
foreach t : tests_standalone
	srcs_t = [
		'@0@-test.c'.format(t.get(0)),
		gen_weston_test_client,
	]
	if t.length() > 1
		srcs_t += t.get(1)
	endif

	deps_t = [
		dep_test_client
	]
	if t.length() > 2
		deps_t += t.get(2)
	endif

	exe_t = executable('test-@0@'.format(t.get(0)),
		srcs_t,
		c_args: [ '-DUNIT_TEST' ],
		include_directories:
			include_directories('..', '../shared', '../libweston'),
		dependencies: deps_t,
		install: false,
	)
	test(t.get(0), exe_t)
endforeach

foreach t : tests_weston
	srcs_t = [
		'@0@-test.c'.format(t.get(0)),
		gen_weston_test_client,
	]
	if t.length() > 1
		srcs_t += t.get(1)
	endif

	deps_t = [
		dep_test_client
	]
	if t.length() > 2
		deps_t += t.get(2)
	endif

	args_t = [
		'--backend=headless-backend.so',
		'--socket=test-@0@'.format(t.get(0)),
		'--modules=@0@'.format(exe_plugin_test.full_path()),
		'--width=320',
		'--height=240',
	]

	if t.get(0) == 'xwayland'
		args_t += '--xwayland'
	endif

	# FIXME: Get this from the array ... ?
	if t.get(0) == 'internal-screenshot'
		args_t += [ '--config=@0@/internal-screenshot.ini'.format(meson.current_source_dir()) ]
		args_t += [ '--use-pixman' ]
		args_t += [ '--shell=desktop-shell.so' ]
	elif t[0] == 'subsurface-shot'
		args_t += [ '--no-config' ]
		args_t += [ '--use-pixman' ]
		args_t += [ '--width=320' ]
		args_t += [ '--height=240' ]
		args_t += [ '--shell=weston-test-desktop-shell.so' ]
	elif t.get(0).startswith('ivi-')
		args_t += [ '--config=@0@/../ivi-shell/weston-ivi-test.ini'.format(meson.current_build_dir()) ]
		args_t += [ '--shell=ivi-shell.so' ]
	else
		args_t += [ '--no-config' ]
		args_t += [ '--shell=desktop-shell.so' ]
	endif

	exe_t = executable('test-@0@'.format(t.get(0)),
		srcs_t,
		c_args: [ '-DUNIT_TEST' ],
		include_directories:
			include_directories('..', '../shared', '../libweston'),
		dependencies: deps_t,
		install: false,
	)

	env_t = [
		'WESTON_TEST_CLIENT_PATH=@0@'.format(exe_t.full_path())
	]
	env_t += env_test_weston

	test(t.get(0), exe_weston, env: env_t, args: args_t)
endforeach

foreach t : tests_weston_plugin
	srcs_t = [
		'@0@-test.c'.format(t.get(0)),
	]
	if t.length() > 1
		srcs_t += t.get(1)
	endif

	deps_t = [
		dep_libweston,
	]
	if t.length() > 2
		deps_t += t.get(2)
	endif

	exe_t = shared_library('test-@0@'.format(t.get(0)),
		srcs_t,
		include_directories:
			include_directories('..', '../shared', '../libweston'),
		dependencies: deps_t,
		name_prefix: '',
		install: false,
	)

	args_t = [
		'--backend=headless-backend.so',
		'--socket=test-@0@'.format(t.get(0)),
	]

	# FIXME: Get this from the array ... ?
	if t.get(0).startswith('ivi-')
		args_t += [ '--config=@0@/../ivi-shell/weston-ivi-test.ini'.format(meson.current_build_dir()) ]
		args_t += [ '--modules=@0@'.format(exe_plugin_test.full_path()) ]
		args_t += [ '--ivi-module=@0@'.format(exe_t.full_path()) ]
		args_t += [ '--shell=ivi-shell.so' ]
	else
		args_t += [ '--no-config' ]
		args_t += [ '--shell=desktop-shell.so' ]
		args_t += [ '--modules=@0@'.format(exe_t.full_path()) ]
	endif

	test(t.get(0), exe_weston, env: env_test_weston, args: args_t)
endforeach
