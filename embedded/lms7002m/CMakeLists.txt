add_library(lms7002m STATIC lms7002m.c calibrations.c privates.c save_state.c spi.c)

set_property(TARGET lms7002m PROPERTY POSITION_INDEPENDENT_CODE TRUE)

# deny usage of floating point operations, ideally everything should be done using integers only
if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_compile_options(lms7002m PRIVATE -fvisibility=hidden)
    target_compile_options(lms7002m PRIVATE -mgeneral-regs-only)
endif(CMAKE_C_COMPILER_ID STREQUAL "GNU")
if(CMAKE_C_COMPILER_ID STREQUAL "Clang")
    target_compile_options(lms7002m PRIVATE -mno-sse)
endif()

# public headers
target_include_directories(lms7002m PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

# private headers, for building tests
target_include_directories(lms7002m PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>)

# Don't install for now.
# install(
#     TARGETS lms7002m
#     EXPORT lms7002m
#     LIBRARY DESTINATION lib${LIB_SUFFIX} # .so file
#     ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
#     RUNTIME DESTINATION bin # .dll file
# )
