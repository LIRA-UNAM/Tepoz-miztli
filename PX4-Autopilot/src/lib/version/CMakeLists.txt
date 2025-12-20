px4_add_library(version
    version.c
)

# CAMBIO CRITICO: Usamos PUBLIC para que mavlink y logger vean estas definiciones
target_compile_definitions(version PUBLIC
    PX4_BOARD_NAME="px4_sitl"
    PX4_BOARD_LABEL="default"
)

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/build_git_version.h
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/px_update_git_header.py ${CMAKE_CURRENT_BINARY_DIR}/build_git_version.h --validate
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/px_update_git_header.py
    COMMENT "Generating git version header"
)

add_custom_target(ver_gen ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/build_git_version.h)

add_dependencies(version ver_gen)

target_include_directories(version PUBLIC ${CMAKE_CURRENT_BINARY_DIR})