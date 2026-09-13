# =============================================================================
# Configuration system: .conf files, options and features
# =============================================================================
#
# Every configurable value is declared once, in CMakeLists.txt:
#
#   robot_option(<NAME> <BOOL|INT|STRING> <default> "<description>" [CHOICES <value>...])
#   robot_feature(<NAME> <default> "<description>"
#                 [REQUIRES <if-expression>...] [SOURCES <file>...])
#   robot_define(<NAME> <if-expression>)    derived ON/OFF value, not user-settable
#
# Values come from, lowest to highest precedence:
#   1. the default in the declaration
#   2. prj.conf
#   3. cmake/boards/<BOARD>.conf, if it exists
#   4. the files in -DEXTRA_CONF_FILE=a.conf;b.conf, in order
#   5. -D<NAME>=... or ccmake. Such an override stays in effect until the value
#      is set back to the .conf value or removed with -U<NAME>.
#
# .conf files hold KEY=VALUE lines; '#' starts a comment. Editing one re-runs
# CMake on the next build. All declared values are written to
# generated/app_config.h:
#   BOOL   -> #define NAME 1 or 0
#   INT    -> #define NAME <value>
#   STRING -> #define NAME_<CHOICE> 1 or 0 for every choice
# =============================================================================

set(EXTRA_CONF_FILE "" CACHE STRING
    "Extra .conf files applied after prj.conf and the board .conf (;-separated)")

function(_robot_load_conf file)
    if(NOT EXISTS "${file}")
        message(FATAL_ERROR "Configuration file not found: ${file}")
    endif()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${file}")

    file(STRINGS "${file}" lines)
    foreach(line IN LISTS lines)
        string(REGEX REPLACE "#.*$" "" line "${line}")
        string(STRIP "${line}" line)
        if(line STREQUAL "")
            continue()
        endif()
        if(NOT line MATCHES "^([A-Z][A-Z0-9_]*)[ \t]*=[ \t]*(.*)$")
            message(FATAL_ERROR "${file}: expected KEY=VALUE, got '${line}'")
        endif()
        set(key "${CMAKE_MATCH_1}")
        string(STRIP "${CMAKE_MATCH_2}" value)
        string(REGEX REPLACE "^\"(.*)\"$" "\\1" value "${value}")
        if(key STREQUAL "BOARD")
            message(FATAL_ERROR "${file}: BOARD is chosen by the preset (cmake --preset <board>), not in a .conf file")
        endif()
        set(ROBOT_CONF_${key} "${value}" PARENT_SCOPE)
        set_property(GLOBAL APPEND PROPERTY ROBOT_CONF_KEYS "${key}")
        set_property(GLOBAL PROPERTY ROBOT_CONF_ORIGIN_${key} "${file}")
    endforeach()
endfunction()

# Loads prj.conf, the board .conf and EXTRA_CONF_FILE overlays, in precedence order
macro(robot_load_configuration)
    _robot_load_conf("${CMAKE_SOURCE_DIR}/prj.conf")
    if(EXISTS "${CMAKE_SOURCE_DIR}/cmake/boards/${BOARD}.conf")
        _robot_load_conf("${CMAKE_SOURCE_DIR}/cmake/boards/${BOARD}.conf")
    endif()
    foreach(_robot_conf IN LISTS EXTRA_CONF_FILE)
        get_filename_component(_robot_conf "${_robot_conf}" ABSOLUTE BASE_DIR "${CMAKE_SOURCE_DIR}")
        _robot_load_conf("${_robot_conf}")
    endforeach()
endmacro()

function(_robot_normalize_bool out value)
    if(value)
        set(${out} ON PARENT_SCOPE)
    else()
        set(${out} OFF PARENT_SCOPE)
    endif()
endfunction()

function(robot_option name type builtin doc)
    cmake_parse_arguments(OPT "" "" "CHOICES" ${ARGN})

    if(NOT type MATCHES "^(BOOL|INT|STRING)$")
        message(FATAL_ERROR "robot_option(${name}): unknown type '${type}'")
    endif()
    set(cache_type STRING)
    if(type STREQUAL "BOOL")
        set(cache_type BOOL)
        _robot_normalize_bool(builtin "${builtin}")
    endif()

    set(configured "${builtin}")
    if(DEFINED ROBOT_CONF_${name})
        set(configured "${ROBOT_CONF_${name}}")
        if(type STREQUAL "BOOL")
            _robot_normalize_bool(configured "${configured}")
        endif()
    endif()

    if(NOT DEFINED CACHE{${name}})
        set(${name} "${configured}" CACHE ${cache_type} "${doc}")
    else()
        set(current "$CACHE{${name}}")
        if(type STREQUAL "BOOL")
            _robot_normalize_bool(current "${current}")
        endif()
        if(DEFINED CACHE{ROBOT_SEED_${name}} AND current STREQUAL "$CACHE{ROBOT_SEED_${name}}")
            # Not overridden by hand: follow the .conf files
            set_property(CACHE ${name} PROPERTY VALUE "${configured}")
        elseif(NOT current STREQUAL configured)
            message(STATUS "${name}=${current} (overrides the .conf value ${configured}; cmake -U${name} to drop)")
        endif()
        set_property(CACHE ${name} PROPERTY TYPE ${cache_type})
        set_property(CACHE ${name} PROPERTY HELPSTRING "${doc}")
    endif()
    set(ROBOT_SEED_${name} "${configured}" CACHE INTERNAL "Value of ${name} from the .conf files" FORCE)

    set(value "$CACHE{${name}}")
    if(type STREQUAL "BOOL")
        _robot_normalize_bool(value "${value}")
    elseif(type STREQUAL "INT" AND NOT value MATCHES "^[0-9]+$")
        message(FATAL_ERROR "${name} must be a non-negative integer, got '${value}'")
    endif()
    if(OPT_CHOICES)
        set_property(CACHE ${name} PROPERTY STRINGS ${OPT_CHOICES})
        if(NOT value IN_LIST OPT_CHOICES)
            message(FATAL_ERROR "${name}='${value}' is not one of: ${OPT_CHOICES}")
        endif()
    endif()

    set(${name} "${value}" PARENT_SCOPE)
    set_property(GLOBAL APPEND PROPERTY ROBOT_OPTIONS ${name})
    set_property(GLOBAL PROPERTY ROBOT_OPTION_TYPE_${name} ${type})
    set_property(GLOBAL PROPERTY ROBOT_OPTION_CHOICES_${name} "${OPT_CHOICES}")
    set_property(GLOBAL PROPERTY ROBOT_OPTION_DEFAULT_${name} "${builtin}")
endfunction()

# A feature whose requirements are not met is off. Setting it to a non-default
# ON without them is an error, so a request is never silently ignored.
function(robot_feature name builtin doc)
    cmake_parse_arguments(F "" "" "REQUIRES;SOURCES" ${ARGN})
    robot_option(${name} BOOL "${builtin}" "${doc}")
    set(value ${${name}})
    get_property(default GLOBAL PROPERTY ROBOT_OPTION_DEFAULT_${name})

    if(value AND APP_BLINK_ONLY)
        set(value OFF)
    elseif(value)
        foreach(requirement IN LISTS F_REQUIRES)
            separate_arguments(expression UNIX_COMMAND "${requirement}")
            if(NOT (${expression}))
                if(NOT value STREQUAL default)
                    message(FATAL_ERROR "${name}=ON requires ${requirement}")
                endif()
                message(STATUS "${name}: off (requires ${requirement})")
                set(value OFF)
                break()
            endif()
        endforeach()
    endif()

    set(${name} ${value} PARENT_SCOPE)
    if(value AND F_SOURCES)
        set_property(GLOBAL APPEND PROPERTY ROBOT_FEATURE_SOURCES ${F_SOURCES})
    endif()
endfunction()

function(robot_define name)
    if(${ARGN})
        set(value ON)
    else()
        set(value OFF)
    endif()
    set(${name} ${value} PARENT_SCOPE)
    set_property(GLOBAL APPEND PROPERTY ROBOT_OPTIONS ${name})
    set_property(GLOBAL PROPERTY ROBOT_OPTION_TYPE_${name} BOOL)
endfunction()

# Checks for unknown .conf keys, then generates the config header from a template
# containing @ROBOT_CONFIG_DEFINES@
function(robot_write_config_header template output)
    get_property(options GLOBAL PROPERTY ROBOT_OPTIONS)
    get_property(conf_keys GLOBAL PROPERTY ROBOT_CONF_KEYS)
    foreach(key IN LISTS conf_keys)
        if(NOT key IN_LIST options)
            get_property(origin GLOBAL PROPERTY ROBOT_CONF_ORIGIN_${key})
            message(FATAL_ERROR "${origin}: unknown configuration key '${key}'")
        endif()
    endforeach()

    set(ROBOT_CONFIG_DEFINES "")
    set(summary "")
    foreach(name IN LISTS options)
        get_property(type GLOBAL PROPERTY ROBOT_OPTION_TYPE_${name})
        get_property(choices GLOBAL PROPERTY ROBOT_OPTION_CHOICES_${name})
        set(value "${${name}}")
        if(type STREQUAL "BOOL")
            if(value)
                string(APPEND ROBOT_CONFIG_DEFINES "#define ${name} 1\n")
            else()
                string(APPEND ROBOT_CONFIG_DEFINES "#define ${name} 0\n")
            endif()
        elseif(type STREQUAL "INT")
            string(APPEND ROBOT_CONFIG_DEFINES "#define ${name} ${value}\n")
        elseif(choices)
            foreach(choice IN LISTS choices)
                string(TOUPPER "${name}_${choice}" macro)
                if(choice STREQUAL value)
                    string(APPEND ROBOT_CONFIG_DEFINES "#define ${macro} 1\n")
                else()
                    string(APPEND ROBOT_CONFIG_DEFINES "#define ${macro} 0\n")
                endif()
            endforeach()
        else()
            string(APPEND ROBOT_CONFIG_DEFINES "#define ${name} \"${value}\"\n")
        endif()
        string(APPEND summary "\n--   ${name} = ${value}")
    endforeach()

    configure_file("${template}" "${output}")
    message(STATUS "Configuration:${summary}")
endfunction()
