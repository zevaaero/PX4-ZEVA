############################################################################
#
<<<<<<< HEAD:src/lib/terrain/CMakeLists.txt
#   Copyright (c) 2020 PX4 Development Team. All rights reserved.
=======
#   Copyright (c) 2021 PX4 Development Team. All rights reserved.
>>>>>>> 94884594bb85d6127b879f58d1aa13b8aeee94a2:platforms/nuttx/src/px4/rpi/rpi_common/hrt/CMakeLists.txt
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

<<<<<<< HEAD:src/lib/terrain/CMakeLists.txt
px4_add_library(terrain
	util.cpp
	terrain_provider.cpp
)
target_link_libraries(terrain PRIVATE geo)

=======
px4_add_library(arch_hrt
	hrt.c
)
target_compile_options(arch_hrt
	PRIVATE
		${MAX_CUSTOM_OPT_LEVEL}
		-Wno-cast-align # TODO: fix and enable
)
>>>>>>> 94884594bb85d6127b879f58d1aa13b8aeee94a2:platforms/nuttx/src/px4/rpi/rpi_common/hrt/CMakeLists.txt
