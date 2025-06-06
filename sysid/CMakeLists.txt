project(sysid)

include(CompileWarnings)
include(GenResources)
include(LinkMacOSGUI)
include(AddTest)

configure_file(src/main/generate/WPILibVersion.cpp.in WPILibVersion.cpp)
generate_resources(src/main/native/resources generated/main/cpp SYSID sysid sysid_resources_src)

file(GLOB_RECURSE sysid_src src/main/native/cpp/*.cpp ${CMAKE_CURRENT_BINARY_DIR}/WPILibVersion.cpp)

if(WIN32)
    set(sysid_rc src/main/native/win/sysid.rc)
elseif(APPLE)
    set(MACOSX_BUNDLE_ICON_FILE sysid.icns)
    set(APP_ICON_MACOSX src/main/native/mac/sysid.icns)
    set_source_files_properties(${APP_ICON_MACOSX} PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")
endif()

add_executable(sysid ${sysid_src} ${sysid_resources_src} ${sysid_rc} ${APP_ICON_MACOSX})
if(MSVC)
    target_compile_options(sysid PRIVATE /utf-8)
endif()
wpilib_link_macos_gui(sysid)
wpilib_target_warnings(sysid)
target_include_directories(sysid PRIVATE src/main/native/include)
target_link_libraries(sysid wpimath libglass datalog)

if(WIN32)
    set_target_properties(sysid PROPERTIES WIN32_EXECUTABLE YES)
elseif(APPLE)
    set_target_properties(sysid PROPERTIES MACOSX_BUNDLE YES OUTPUT_NAME "SysId")
endif()

if(WITH_TESTS)
    wpilib_add_test(sysid src/test/native/cpp)
    wpilib_link_macos_gui(sysid_test)
    target_sources(sysid_test PRIVATE ${sysid_src})
    target_compile_definitions(sysid_test PRIVATE RUNNING_SYSID_TESTS)
    if(MSVC)
        target_compile_options(sysid_test PRIVATE /utf-8)
    endif()
    target_include_directories(sysid_test PRIVATE src/main/native/cpp src/main/native/include)
    target_link_libraries(sysid_test wpimath libglass datalog googletest)
endif()
