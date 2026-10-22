# Shared warning flags, used by the root build and by the standalone GUI build
# so that `cmake -S src/gui -B build` behaves exactly like `cmake --preset debug`.

if(TARGET pendulum_warnings)
    return()
endif()

option(PENDULUM_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

add_library(pendulum_warnings INTERFACE)
target_compile_options(pendulum_warnings INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wsign-conversion
        -Wshadow
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wcast-align
        -Wunused
        -Woverloaded-virtual
        -Wnull-dereference>
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive->)

if(PENDULUM_WARNINGS_AS_ERRORS)
    target_compile_options(pendulum_warnings INTERFACE
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Werror>
        $<$<CXX_COMPILER_ID:MSVC>:/WX>)
endif()
