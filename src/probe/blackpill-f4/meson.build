# This file is part of the Black Magic Debug project.
#
# Copyright (C) 2022 1BitSquared <info@1bitsquared.com>
# Written by Rafael Silva <perigoso@riseup.net>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

probe = get_option('probe')

probe_blackpill_f4_includes = include_directories(
	'.',
	f'include/@probe@',
)

probe_blackpill_f4_sources = files(
	'blackpill-f4.c',
)

probe_blackpill_f4_dfu_sources = files(
	'usbdfu.c',
)

bmd_bootloader = get_option('bmd_bootloader')

probe_blackpill_f4_dfu_serial_length = bmd_bootloader ? '9' : '13'
probe_blackpill_f4_load_address = bmd_bootloader ? '0x08004000' : '0x08000000'

probe_blackpill_f4_args = [
	f'-DDFU_SERIAL_LENGTH=@probe_blackpill_f4_dfu_serial_length@',
	f'-DAPP_START=@probe_blackpill_f4_load_address@',
]

blackpill_alternative_pinout = get_option('blackpill_alternative_pinout')
if blackpill_alternative_pinout != '0'
	probe_blackpill_f4_args += [
		f'-DALTERNATIVE_PINOUT=@blackpill_alternative_pinout@',
	]
endif

if bmd_bootloader
	probe_blackpill_f4_args += [
		'-DBMD_BOOTLOADER',
	]
endif

probe_blackpill_f4_common_link_args = [
	'-L@0@'.format(meson.current_source_dir()),
	'-T@0@.ld'.format(probe),
]

probe_blackpill_f4_link_args = []
if bmd_bootloader
	probe_blackpill_f4_link_args += [
		f'-Wl,-Ttext=@probe_blackpill_f4_load_address@',
	]
endif

probe_host = declare_dependency(
	include_directories: probe_blackpill_f4_includes,
	sources: probe_blackpill_f4_sources,
	compile_args: probe_blackpill_f4_args,
	link_args: [probe_blackpill_f4_common_link_args, probe_blackpill_f4_link_args],
	dependencies: [platform_common, platform_stm32f4, fixme_platform_stm32_traceswo],
)

probe_bootloader = declare_dependency(
	include_directories: [platform_common_includes, probe_blackpill_f4_includes],
	sources: probe_blackpill_f4_dfu_sources,
	compile_args: probe_blackpill_f4_args,
	link_args: probe_blackpill_f4_common_link_args,
	dependencies: platform_stm32f4_dfu,
)

summary({
	'Name': 'WeAct Studio F4X1 BlackPill',
	'Platform': 'STM32F4',
	'Bootloader': bmd_bootloader ? 'Black Magic Debug Bootloader' : 'OEM ST Bootloader',
	'Load Address': probe_blackpill_f4_load_address,
}, section: 'Probe')
