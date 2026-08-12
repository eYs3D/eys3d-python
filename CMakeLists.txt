cmake_minimum_required(VERSION 3.20)
project(pyeys3d LANGUAGES CXX VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# scikit-build-core injects Python_EXECUTABLE etc. for the active build env.
find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)
find_package(OpenMP REQUIRED)

# Bundled eSPDI SDK (headers + SDK binary per platform/arch). ESPDI_INCLUDE
# is the header directory the engine compiles against; ESPDI_LIB is what the
# binding links; ESPDI_ERR_HEADER feeds the error-name table below (the two
# SDKs keep their error codes in differently named headers).
set(ESPDI_ROOT "${CMAKE_SOURCE_DIR}/eSPDI")

if(WIN32)
    set(ESPDI_ARCH "win_x64")
    set(ESPDI_INCLUDE "${ESPDI_ROOT}/win_x64/include")
    set(ESPDI_LIB "${ESPDI_ROOT}/win_x64/eSPDI_DM.lib")
    set(ESPDI_ERR_HEADER "${ESPDI_INCLUDE}/eSPDI_ErrCode.h")
else()
    set(ESPDI_INCLUDE "${ESPDI_ROOT}")
    set(ESPDI_ERR_HEADER "${ESPDI_ROOT}/eSPDI_def.h")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
        set(ESPDI_LIB_NAME "libeSPDI_X86_64.so")
        set(ESPDI_ARCH "x86_64")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
        set(ESPDI_LIB_NAME "libeSPDI_NVIDIA_64.so")
        set(ESPDI_ARCH "aarch64")
    else()
        message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    endif()

    file(GLOB _espdi_libs "${ESPDI_ROOT}/${ESPDI_LIB_NAME}.*")
    if(NOT _espdi_libs)
        message(FATAL_ERROR "libeSPDI not found: ${ESPDI_ROOT}/${ESPDI_LIB_NAME}.*")
    endif()
    list(SORT _espdi_libs COMPARE NATURAL)
    list(GET _espdi_libs -1 ESPDI_LIB)
endif()

# APC error-code name table, generated from the SDK header so runtime error
# messages carry the define name (APC_NOT_SUPPORT_RES) instead of a bare rc.
set(APC_ERROR_INC "${CMAKE_CURRENT_BINARY_DIR}/generated/apc_error_names.inc")
add_custom_command(
    OUTPUT "${APC_ERROR_INC}"
    COMMAND ${Python_EXECUTABLE} "${CMAKE_SOURCE_DIR}/cmake/gen_apc_error_names.py"
            "${ESPDI_ERR_HEADER}" "${APC_ERROR_INC}"
    DEPENDS "${ESPDI_ERR_HEADER}"
            "${CMAKE_SOURCE_DIR}/cmake/gen_apc_error_names.py"
    COMMENT "Generating APC error-code name table"
)
add_custom_target(gen_apc_error_names DEPENDS "${APC_ERROR_INC}")

# The platform seam: per-OS implementations of cpp/platform.hpp and of
# the CaptureEngine fetch backend (os_open_streams / the fetch layer).
if(WIN32)
    set(PYEYS3D_PLATFORM_SRC cpp/platform_windows.cpp cpp/capture_windows.cpp)
else()
    set(PYEYS3D_PLATFORM_SRC cpp/platform_linux.cpp cpp/capture_linux.cpp)
endif()

pybind11_add_module(_pyeys3d_native
    MODULE
    cpp/bindings.cpp
    cpp/capture_engine.cpp
    ${PYEYS3D_PLATFORM_SRC}
    cpp/simd_kernels.cpp
    cpp/spatial_filter.cpp
    cpp/temporal_filter.cpp
    cpp/hole_filling.cpp
    cpp/zd_lookup.cpp
    cpp/point_cloud.cpp
)

add_dependencies(_pyeys3d_native gen_apc_error_names)

target_include_directories(_pyeys3d_native PRIVATE
    cpp/
    "${ESPDI_INCLUDE}"
    "${CMAKE_CURRENT_BINARY_DIR}/generated"
)

target_link_libraries(_pyeys3d_native PRIVATE "${ESPDI_LIB}")

if(WIN32)
    target_link_libraries(_pyeys3d_native PRIVATE OpenMP::OpenMP_CXX)
else()
    target_compile_options(_pyeys3d_native PRIVATE ${OpenMP_CXX_FLAGS})
    target_link_libraries(_pyeys3d_native PRIVATE OpenMP::OpenMP_CXX)
    # Distribution wheels must import on a minimal host that lacks a matching
    # GCC runtime, and auditwheel repair is disabled to preserve the eSPDI
    # $ORIGIN/eSPDI RUNPATH -- so the release pipeline turns this on and we
    # bundle the runtime by hand. libstdc++ / libgcc static-link cleanly (their
    # static forms are PIC-safe); libgomp cannot be static-linked into a .so
    # (even the devtoolset libgomp.a is built without -fPIC, and its TLS
    # relocations are rejected in a shared object), so libgomp.so.1 is vendored
    # next to libeSPDI instead, resolved through the same RUNPATH. Editable and
    # plain builds leave this off and rely on the host's GCC runtime.
    option(PYEYS3D_SELF_CONTAINED
           "Bundle the GCC runtime: static libstdc++/libgcc + vendored libgomp"
           OFF)
    if(PYEYS3D_SELF_CONTAINED)
        target_link_options(_pyeys3d_native PRIVATE
            -static-libstdc++ -static-libgcc)
        # Ship the toolchain's real libgomp.so.1 (resolve the linker symlink)
        # beside libeSPDI; the binding's NEEDED libgomp.so.1 then loads from
        # $ORIGIN/eSPDI on hosts without their own libgomp.
        execute_process(
            COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libgomp.so.1
            OUTPUT_VARIABLE _gomp_link OUTPUT_STRIP_TRAILING_WHITESPACE)
        get_filename_component(_gomp_real "${_gomp_link}" REALPATH)
        if(NOT EXISTS "${_gomp_real}")
            message(FATAL_ERROR
                "PYEYS3D_SELF_CONTAINED: libgomp.so.1 not found via the compiler")
        endif()
        install(FILES "${_gomp_real}" DESTINATION pyeys3d/eSPDI
                RENAME libgomp.so.1)
    endif()
endif()

# Single-source the version from pyproject.toml: scikit-build-core sets
# SKBUILD_PROJECT_VERSION; fall back to the project() version for a plain
# `cmake` build. The bindings expose this as the module __version__.
if(DEFINED SKBUILD_PROJECT_VERSION)
    set(_PYEYS3D_VERSION "${SKBUILD_PROJECT_VERSION}")
else()
    set(_PYEYS3D_VERSION "${PROJECT_VERSION}")
endif()
target_compile_definitions(_pyeys3d_native PRIVATE
    PYEYS3D_VERSION="${_PYEYS3D_VERSION}")

# Resolve libeSPDI at runtime relative to the installed _pyeys3d_native.so.
# The .so lands in pyeys3d/, libeSPDI lands in pyeys3d/eSPDI/, so the
# rpath needs to climb up once then down into eSPDI/. (Windows has no
# rpath — the package __init__ registers its own directory as a DLL
# search path instead.)
if(NOT WIN32)
    set_target_properties(_pyeys3d_native PROPERTIES
        INSTALL_RPATH "$ORIGIN/eSPDI"
        BUILD_WITH_INSTALL_RPATH TRUE
    )
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(_pyeys3d_native PRIVATE
        -Wall -Wextra -Wpedantic
        -fvisibility=hidden
    )
endif()

# Install the bindings module alongside the eSPDI directory inside the wheel.
install(TARGETS _pyeys3d_native LIBRARY DESTINATION pyeys3d)

if(WIN32)
    # The binding links the import library; the wheel ships the DLL next to
    # the .pyd (pyeys3d/__init__ adds that directory to the DLL search
    # path). eSPDI_DM.dll additionally imports the system OpenCL.dll — the
    # Khronos ICD loader every GPU driver installs into System32.
    install(FILES "${ESPDI_ROOT}/win_x64/eSPDI_DM.dll" DESTINATION pyeys3d)
else()
    # Ship only the libeSPDI matching this wheel's architecture, as the single
    # versioned file the binding records in DT_NEEDED. The SONAME is version-
    # tagged (libeSPDI_<arch>.so.5.1.0.6), so the runtime loader resolves that
    # exact name via the binding's RUNPATH ($ORIGIN/eSPDI). The other
    # architecture's binary is never loaded here, and the headers are
    # build-time only — none are shipped.
    install(FILES "${ESPDI_LIB}" DESTINATION pyeys3d/eSPDI)

    # libeSPDI is linked against libOpenCL.so.1 and resolves it from its own
    # RPATH ($ORIGIN/OpenCL/<arch>/lib). Ship that architecture's OpenCL ICD
    # loader stub so the dependency is satisfied on hosts with no OpenCL
    # runtime.
    install(DIRECTORY "${ESPDI_ROOT}/OpenCL/${ESPDI_ARCH}/lib/"
        DESTINATION "pyeys3d/eSPDI/OpenCL/${ESPDI_ARCH}/lib"
        USE_SOURCE_PERMISSIONS
    )
endif()

# Native kernel unit tests: the SDK-free depth-compute core (disparity
# promotion, the spatial / temporal filters, hole fill). Off by default so
# the wheel build skips them; enable with -DPYEYS3D_BUILD_TESTS=ON and run
# via ctest.
# The test sources live under tests/, which the release cut strips; guard on
# their presence so enabling the option in a release tree skips cleanly
# instead of failing configure on a missing source.
option(PYEYS3D_BUILD_TESTS "Build the native kernel unit tests" OFF)
if(PYEYS3D_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/cpp/test_kernels.cpp")
    enable_testing()
    add_executable(test_kernels
        tests/cpp/test_kernels.cpp
        cpp/spatial_filter.cpp
        cpp/temporal_filter.cpp
        cpp/hole_filling.cpp
    )
    target_include_directories(test_kernels PRIVATE cpp/)
    target_link_libraries(test_kernels PRIVATE OpenMP::OpenMP_CXX)
    add_test(NAME kernels COMMAND test_kernels)
endif()
