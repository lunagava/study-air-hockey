# requires minimum cmake version
cmake_minimum_required(VERSION 3.5)

# produce the cmake var PROJECT_NAME
project(motion-prediction)

find_package(event-driven REQUIRED)
find_package(YARP 3.3.0 REQUIRED COMPONENTS os sig dev math)
find_package(ICUB REQUIRED COMPONENTS ctrlLib)
#find_package(ICUBcontrib REQUIRED)
find_package(Python3 COMPONENTS Development NumPy)
find_package(OpenCV REQUIRED)

#default the install location to that of event-driven
add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ev::event-driven ${OpenCV_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

#install .ini files and .xml.template files to share event-driven
#yarp_install(FILES ${PROJECT_NAME}.ini
#             DESTINATION ${EVENT-DRIVEN_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})


add_custom_target(${PROJECT_NAME}_docs SOURCES ${PROJECT_NAME}.ini README.md)


#uninstall target?
