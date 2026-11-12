# Writes OUT (build_info.h) with BUILD_GIT_DESC; touches it only when it changes.
# Inputs: OUT, SRC_DIR, GIT_EXECUTABLE (optional).
set(desc "unknown")
if(GIT_EXECUTABLE)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --always --dirty
        WORKING_DIRECTORY ${SRC_DIR}
        OUTPUT_VARIABLE out
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE rc)
    if(rc EQUAL 0 AND out)
        set(desc "${out}")
    endif()
endif()

set(text "#ifndef __BUILD_INFO_H\n#define __BUILD_INFO_H\n\n#define BUILD_GIT_DESC \"${desc}\"\n\n#endif /* __BUILD_INFO_H */\n")

set(old "")
if(EXISTS "${OUT}")
    file(READ "${OUT}" old)
endif()
if(NOT old STREQUAL text)
    file(WRITE "${OUT}" "${text}")
    message(STATUS "Build identity: ${desc}")
endif()
