add_executable(test_vnav test_vnav.cpp)
target_link_libraries(test_vnav PRIVATE vnav)
target_include_directories(test_vnav PRIVATE ${CMAKE_SOURCE_DIR}/include)
add_test(NAME test_vnav COMMAND test_vnav)
