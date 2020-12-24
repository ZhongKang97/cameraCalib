add_library(calibtool 180925_calibCamera.cpp)
target_link_libraries(calibtool ${THIRD_PARTY_LIBS})
add_executable(calib_mono runCalib_mono.cpp)
target_link_libraries(calib_mono calibtool)

add_executable(calib_stereo runCalib_stereo.cpp)
target_link_libraries(calib_stereo calibtool)