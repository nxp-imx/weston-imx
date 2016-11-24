project('weston',
	'c',
	version: '2.99.90',
	default_options: [
		'warning_level=2',
		'c_std=gnu99',
		'b_lundef=false',
	],
	meson_version: '>= 0.36',
	license: 'MIT/Expat',
)

libweston_major = 3
version_weston = meson.project_version()

dir_prefix = get_option('prefix')
dir_bin = join_paths(dir_prefix, get_option('bindir'))
dir_data = join_paths(dir_prefix, get_option('datadir'))
dir_include = join_paths(dir_prefix, get_option('includedir'))
dir_include_libweston = join_paths(dir_include, 'libweston-@0@'.format(libweston_major))
dir_lib = join_paths(dir_prefix, get_option('libdir'))
dir_libexec = join_paths(dir_prefix, get_option('libexecdir'))
dir_module_weston = join_paths(dir_lib, 'weston')
dir_module_libweston = join_paths(dir_lib, 'libweston-@0@'.format(libweston_major))
dir_pc = join_paths(dir_data, 'pkgconfig')

pkgconfig = import('pkgconfig')

version_weston_arr = version_weston.split('.')
libweston_version_h = configuration_data()
libweston_version_h.set('WESTON_VERSION_MAJOR', version_weston_arr[0])
libweston_version_h.set('WESTON_VERSION_MINOR', version_weston_arr[1])
libweston_version_h.set('WESTON_VERSION_MICRO', version_weston_arr[2])
libweston_version_h.set('WESTON_VERSION', version_weston)
version_h = configure_file(
	input: 'libweston/version.h.in',
	output: 'version.h',
	configuration: libweston_version_h,
	install_dir: dir_include_libweston)
git_version_h = vcs_tag(
	input: 'libweston/git-version.h.meson',
	output: 'git-version.h',
	fallback: version_weston)

config_h = configuration_data()

cc = meson.get_compiler('c')

global_args = []
global_args_maybe = [
	'-Wno-unused-parameter',
	'-Wno-shift-negative-value', # required due to Pixman
	'-Wno-missing-field-initializers',
	'-fvisibility=hidden',
	'-DIN_WESTON',
]
foreach a : global_args_maybe
	if cc.has_argument(a)
		global_args += global_args_maybe
	endif
endforeach
add_global_arguments(global_args, language: 'c')

env_modmap = ''

config_h.set('_GNU_SOURCE', '1')

config_h.set('PACKAGE_STRING', '"weston @0@"'.format(version_weston))
config_h.set_quoted('PACKAGE_VERSION', version_weston)
config_h.set_quoted('VERSION', version_weston)
config_h.set('PACKAGE_URL', '"https://wayland.freedesktop.org"')
config_h.set('PACKAGE_BUGREPORT',
             '"https://bugs.freedesktop.org/enter_bug.cgi?product=Wayland&component=weston"')

config_h.set_quoted('BINDIR', dir_bin)
config_h.set_quoted('DATADIR', dir_data)
config_h.set_quoted('LIBEXECDIR', dir_libexec)
config_h.set_quoted('MODULEDIR', dir_module_weston)
config_h.set_quoted('LIBWESTON_MODULEDIR', dir_module_libweston)

config_pc = configuration_data()
config_pc.set('prefix', dir_prefix)
config_pc.set('exec_prefix', dir_prefix)
config_pc.set('includedir', dir_include)
config_pc.set('libdir', dir_lib)
config_pc.set('libexecdir', dir_libexec)
config_pc.set('WESTON_VERSION', version_weston)
config_pc.set('LIBWESTON_MAJOR', libweston_major)

opt_backend_native = get_option('backend-default')
if opt_backend_native == ''
	if get_option('backend-drm')
		opt_backend_native = 'drm-backend.so'
	elif get_option('backend-wayland')
		opt_backend_native = 'wayland-backend.so'
	elif get_option('backend-x11')
		opt_backend_native = 'x11-backend.so'
	elif get_option('backend-headless')
		opt_backend_native = 'headless-backend.so'
	endif
endif
config_h.set_quoted('WESTON_NATIVE_BACKEND', opt_backend_native)

if get_option('xkbcommon')
	dep_xkbcommon = dependency('xkbcommon', version: '>= 0.3.0')
	config_h.set('ENABLE_XKBCOMMON', '1')
	if dep_xkbcommon.version().version_compare('>= 0.5.0')
		config_h.set('HAVE_XKBCOMMON_COMPOSE', '1')
	endif
endif

dep_wayland_server = dependency('wayland-server', version: '>= 1.12.0')
dep_wayland_client = dependency('wayland-client', version: '>= 1.12.0')
dep_pixman = dependency('pixman-1', version: '>= 0.25.2')
dep_libinput = dependency('libinput', version: '>= 0.8.0')
dep_libm = cc.find_library('m')
dep_libdl = cc.find_library('dl')
dep_libdrm = dependency('libdrm', version: '>= 2.4.30')

subdir('protocol')
subdir('shared')
subdir('libweston')
subdir('libweston-desktop')
subdir('xwayland')
subdir('compositor')
subdir('desktop-shell')
subdir('fullscreen-shell')
subdir('ivi-shell')
subdir('clients')
subdir('tests')

configure_file(output: 'config.h', install: false, configuration: config_h)
