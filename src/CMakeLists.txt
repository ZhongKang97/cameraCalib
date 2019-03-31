add_library(calibtool 180925_calibCamera.cpp)
target_link_libraries(calibtool ${THIRD_PARTY_LIBS})
add_executable(calib_mono runCalib.cpp)
target_link_libraries(calib_mono calibtool)