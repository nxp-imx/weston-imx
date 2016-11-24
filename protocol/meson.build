# Set up and generate all our protocol stubs. It would be nice if we had some
# more native Meson support here, to cut out some of the hideous long lines.

dep_wp = dependency('wayland-protocols', version: '>= 1.7')
dir_wp_base = dep_wp.get_pkgconfig_variable('pkgdatadir')

prog_scanner = find_program('wayland-scanner')
gen_scanner_client = generator(prog_scanner,
	output: '@BASENAME@-client-protocol.h',
	arguments: ['client-header', '@INPUT@', '@OUTPUT@'])
gen_scanner_server = generator(prog_scanner,
	output: '@BASENAME@-server-protocol.h',
	arguments: ['server-header', '@INPUT@', '@OUTPUT@'])
gen_scanner_impl = generator(prog_scanner,
	output: '@BASENAME@-protocol.c',
	arguments: ['code', '@INPUT@', '@OUTPUT@'])

proto_text_cursor = 'text-cursor-position.xml'
gen_text_cursor_pos_server = gen_scanner_server.process(proto_text_cursor)
gen_text_cursor_pos_client = gen_scanner_client.process(proto_text_cursor)
gen_text_cursor_pos_impl = gen_scanner_impl.process(proto_text_cursor)

proto_text_input = '@0@/unstable/text-input/text-input-unstable-v1.xml'.format(dir_wp_base)
gen_text_input_server = gen_scanner_server.process(proto_text_input)
gen_text_input_client = gen_scanner_client.process(proto_text_input)
gen_text_input_impl = gen_scanner_impl.process(proto_text_input)

proto_input_method = '@0@/unstable/input-method/input-method-unstable-v1.xml'.format(dir_wp_base)
gen_input_method_server = gen_scanner_server.process(proto_input_method)
gen_input_method_client = gen_scanner_client.process(proto_input_method)
gen_input_method_impl = gen_scanner_impl.process(proto_input_method)

proto_rel_ptr = '@0@/unstable/relative-pointer/relative-pointer-unstable-v1.xml'.format(dir_wp_base)
gen_rel_ptr_server = gen_scanner_server.process(proto_rel_ptr)
gen_rel_ptr_client = gen_scanner_client.process(proto_rel_ptr)
gen_rel_ptr_impl = gen_scanner_impl.process(proto_rel_ptr)

proto_ptr_constraints = '@0@/unstable/pointer-constraints/pointer-constraints-unstable-v1.xml'.format(dir_wp_base)
gen_ptr_constraints_client = gen_scanner_client.process(proto_ptr_constraints)
gen_ptr_constraints_server = gen_scanner_server.process(proto_ptr_constraints)
gen_ptr_constraints_impl = gen_scanner_impl.process(proto_ptr_constraints)


proto_dmabuf = '@0@/unstable/linux-dmabuf/linux-dmabuf-unstable-v1.xml'.format(dir_wp_base)
gen_dmabuf_server = gen_scanner_server.process(proto_dmabuf)
gen_dmabuf_client = gen_scanner_client.process(proto_dmabuf)
gen_dmabuf_impl = gen_scanner_impl.process(proto_dmabuf)

proto_viewporter = '@0@/stable/viewporter/viewporter.xml'.format(dir_wp_base)
gen_viewporter_client = gen_scanner_client.process(proto_viewporter)
gen_viewporter_server = gen_scanner_server.process(proto_viewporter)
gen_viewporter_impl = gen_scanner_impl.process(proto_viewporter)

proto_presentation_time = '@0@/stable/presentation-time/presentation-time.xml'.format(dir_wp_base)
gen_presentation_time_server = gen_scanner_server.process(proto_presentation_time)
gen_presentation_time_client = gen_scanner_client.process(proto_presentation_time)
gen_presentation_time_impl = gen_scanner_impl.process(proto_presentation_time)

proto_xdg5 = '@0@/unstable/xdg-shell/xdg-shell-unstable-v5.xml'.format(dir_wp_base)
gen_xdg_shell5_client = gen_scanner_client.process(proto_xdg5)
gen_xdg_shell5_server = gen_scanner_server.process(proto_xdg5)
gen_xdg_shell5_impl = gen_scanner_impl.process(proto_xdg5)

proto_xdg6 = '@0@/unstable/xdg-shell/xdg-shell-unstable-v6.xml'.format(dir_wp_base)
gen_xdg_shell6_client = gen_scanner_client.process(proto_xdg6)
gen_xdg_shell6_server = gen_scanner_server.process(proto_xdg6)
gen_xdg_shell6_impl = gen_scanner_impl.process(proto_xdg6)


proto_desktop_shell = 'weston-desktop-shell.xml'
gen_desktop_shell_server = gen_scanner_server.process(proto_desktop_shell)
gen_desktop_shell_client = gen_scanner_client.process(proto_desktop_shell)
gen_desktop_shell_impl = gen_scanner_impl.process(proto_desktop_shell)

proto_shooter = 'weston-screenshooter.xml'
gen_shooter_server = gen_scanner_server.process(proto_shooter)
gen_shooter_client = gen_scanner_client.process(proto_shooter)
gen_shooter_impl = gen_scanner_impl.process(proto_shooter)


proto_fullscreen_shell = '@0@/unstable/fullscreen-shell/fullscreen-shell-unstable-v1.xml'.format(dir_wp_base)
gen_fullscreen_shell_server = gen_scanner_server.process(proto_fullscreen_shell)
gen_fullscreen_shell_client = gen_scanner_client.process(proto_fullscreen_shell)
gen_fullscreen_shell_impl = gen_scanner_impl.process(proto_fullscreen_shell)


proto_ivi_application = 'ivi-application.xml'
gen_ivi_application_server = gen_scanner_server.process(proto_ivi_application)
gen_ivi_application_client = gen_scanner_client.process(proto_ivi_application)
gen_ivi_application_impl = gen_scanner_impl.process(proto_ivi_application)

proto_ivi_hmi = 'ivi-hmi-controller.xml'
gen_ivi_hmi_server = gen_scanner_server.process(proto_ivi_hmi)
gen_ivi_hmi_client = gen_scanner_client.process(proto_ivi_hmi)
gen_ivi_hmi_impl = gen_scanner_impl.process(proto_ivi_hmi)


proto_weston_test = 'weston-test.xml'
gen_weston_test_server = gen_scanner_server.process(proto_weston_test)
gen_weston_test_client = gen_scanner_client.process(proto_weston_test)
gen_weston_test_impl = gen_scanner_impl.process(proto_weston_test)
