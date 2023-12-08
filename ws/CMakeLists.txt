file(GLOB HW_SUBDIRECTORIES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *)

# Find OMPL package
find_package(ompl REQUIRED)
message(STATUS "Found OMPL: ${OMPL_INCLUDE_DIRS}")

# Include directories
include_directories(
    includes
    ${YAML_CPP_INCLUDE_DIRS}
    /usr/include/eigen3
    /usr/local/include/ompl-1.6
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/shared")
    message("Found user-created libraries")
    file(GLOB USER_LIB_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/shared/*.cpp")
    add_library(user_library STATIC ${USER_LIB_SOURCES})

    # Link libraries including OMPL
    target_link_libraries(user_library PRIVATE
        yaml-cpp
        Eigen3::Eigen
        AMPCorePrivate
        AMPCorePublic
        ${OMPL_LIBRARIES}
        ompl::ompl
    )

    # Include OMPL directories
    target_include_directories(user_library PUBLIC
        ${AMP_INCLUDE_DIRS}
        ${AMP_DEPENDENCY_INCLUDE_DIRS}
        /usr/local/include/ompl-1.6
        /usr/include/eigen3
    )

endif()

foreach(SUBDIR ${HW_SUBDIRECTORIES})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR} AND NOT SUBDIR STREQUAL "shared" AND NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR}/IGNORE")
        file(GLOB_RECURSE USER_SOURCES "${SUBDIR}/*.cpp")
        set(TARGET_NAME "main-${SUBDIR}")
        add_executable(${TARGET_NAME} ${USER_SOURCES})

        # Link libraries including OMPL
        target_link_libraries(${TARGET_NAME} PRIVATE
            yaml-cpp
            Eigen3::Eigen
            AMPCorePrivate
            AMPCorePublic
            ${OMPL_LIBRARIES}
            ompl::ompl
        )


        # Include OMPL directories
        target_include_directories(${TARGET_NAME} PUBLIC
            ${AMP_INCLUDE_DIRS}
            ${AMP_DEPENDENCY_INCLUDE_DIRS}
            /usr/include/eigen3
            /usr/local/include/ompl-1.6
            ${OMPL_INCLUDE_DIRS}
        )

        if(TARGET user_library)
            target_link_libraries(${TARGET_NAME} PRIVATE user_library)
            target_include_directories(${TARGET_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/shared")
        endif()

        if(AMP_EXCLUDE_VIS)
            target_compile_definitions(${TARGET_NAME} PRIVATE AMP_EXCLUDE_VIS)
            if(CMAKE_BUILD_TYPE STREQUAL "Debug")
                target_compile_options(${TARGET_NAME} PRIVATE -g)
            endif()
        endif()

        if (AMP_EXCLUDE_LOGS)
            message("Building ${TARGET_NAME} without logs")
            target_compile_definitions(${TARGET_NAME} PRIVATE AMP_EXCLUDE_LOGS)
        endif()
    endif()
endforeach()

