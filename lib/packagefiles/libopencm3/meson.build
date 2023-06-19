project('libopencm3', 'c',
	version: 'v0.8.0',
	license: 'GPL-3.0-or-later OR BSD-3-Clause OR MIT',
)

# Ideally this would provide a complete meson build system, but libopencm3 includes some complexities (code generation) that make it not trivial to do so.
#
# To save myself from madness I opted to integrate it with the external project meson module instead (unstable) and hack it a bit to make it work.
# Why 'hack'? The libopencm3 build system is not very flexible, it does not provide a way to build out-of-tree, and does not provide an install target,
# both are required for the meson external project module to work, the hack is a script copying the required sources and a wrapper Makefile.
# (see lib/packagefiles/libopencm3/configure)
#
# If you are reading this and have the drive to port a proper meson build system for libopencm3, please go ahead (and thank you!).

# Ensure we are cross-compiling and not building for the build host
assert(meson.is_cross_build(), 'libopencm3 must be cross-compiled')

# Ensure we are using a GCC compiler
compiler = meson.get_compiler('c')
assert(compiler.get_id() == 'gcc', 'libopencm3 must be compiled with GCC')

# Import the external project meson module
external_project = import('unstable-external_project')

# Parse the compiler prefix
compiler_prefix = compiler.cmd_array()[0].substring(0, -3) # Remove 'gcc' suffix
message('GCC prefix: ' + compiler_prefix)

# Add the libopencm3 project as an external project
libopencm3_project = external_project.add_project('configure',
	configure_options: [
		'--prefix=@PREFIX@',
		'--libdir=@LIBDIR@',
		'--incdir=@INCLUDEDIR@',
		f'--toolchain=@compiler_prefix@',
		'--verbose',
	],
)

# Get all targets/libraries supported by libopencm3
make = find_program('make')
libopencm3_targets = run_command(make, '-C', meson.current_source_dir(), 'list-targets', check: true).stdout().split(' ')

# Declare a libopencm3 dependency for each library
foreach target: libopencm3_targets
	library = ''.join(['opencm3_', target.replace('/', '')])
	meson.override_dependency(library, libopencm3_project.dependency(library))
endforeach
