# Runs clang-tidy over the given files using a sanitized compilation database.
#
# Qt's imported targets add GNU-only compile options (for example
# -mno-direct-extern-access) that the clang frontend inside clang-tidy rejects
# with "unknown argument". Those options have no effect on analysis, so they are
# stripped from a copy of compile_commands.json before clang-tidy is invoked.
#
# Required: CLANG_TIDY_EXECUTABLE, BUILD_DIR, FILES (absolute .cpp paths).

if(NOT DEFINED CLANG_TIDY_EXECUTABLE OR NOT DEFINED BUILD_DIR OR NOT DEFINED FILES)
    message(FATAL_ERROR "CLANG_TIDY_EXECUTABLE, BUILD_DIR and FILES must be set")
endif()

set(database_path "${BUILD_DIR}/compile_commands.json")
if(NOT EXISTS "${database_path}")
    message(FATAL_ERROR "compile_commands.json not found in ${BUILD_DIR}; configure first")
endif()

set(sanitized_dir "${BUILD_DIR}/clang-tidy")
file(MAKE_DIRECTORY "${sanitized_dir}")

file(READ "${database_path}" database)
string(REPLACE "-mno-direct-extern-access" "" database "${database}")
file(WRITE "${sanitized_dir}/compile_commands.json" "${database}")

foreach(file IN LISTS FILES)
    message(STATUS "clang-tidy: ${file}")
    execute_process(
        COMMAND "${CLANG_TIDY_EXECUTABLE}" -p "${sanitized_dir}" "${file}"
        RESULT_VARIABLE tidy_result)
    if(NOT tidy_result EQUAL 0)
        message(FATAL_ERROR "clang-tidy reported problems in ${file}")
    endif()
endforeach()
