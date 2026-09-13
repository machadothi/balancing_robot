# =============================================================================
# Host unit tests: hardware-independent modules built with the native compiler
#
#   cmake --preset host && cmake --build --preset host && ctest --preset host
#
# shim/ stands in for FreeRTOS, libopencm3 and the generated app_config.h.
# =============================================================================

set(SRC ${CMAKE_SOURCE_DIR}/src)

add_compile_options(
    -Wall -Wextra -Werror -Wshadow -Wundef -Wmissing-prototypes -Wstrict-prototypes
    -g -fsanitize=address,undefined -fno-sanitize-recover=all
)
add_link_options(-fsanitize=address,undefined)
include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/shim ${CMAKE_CURRENT_SOURCE_DIR} ${SRC})

function(host_test name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} m)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

host_test(test_control test_control.c ${SRC}/control/pid.c ${SRC}/control/mixer.c)
host_test(test_filters test_filters.c ${SRC}/filter/kalman.c ${SRC}/filter/complementary.c)
host_test(test_fmt test_fmt.c ${SRC}/util/fmt.c)
host_test(test_at test_at.c
    ${SRC}/cmd/at_cmd.c ${SRC}/robot/robot_commands.c ${SRC}/control/pid.c ${SRC}/util/fmt.c)
