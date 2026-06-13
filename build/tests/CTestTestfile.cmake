# CMake generated Testfile for 
# Source directory: D:/SOLO-11/22-avionics-vnav-calc/tests
# Build directory: D:/SOLO-11/22-avionics-vnav-calc/build/tests
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test(test_vnav "D:/SOLO-11/22-avionics-vnav-calc/build/tests/Debug/test_vnav.exe")
  set_tests_properties(test_vnav PROPERTIES  _BACKTRACE_TRIPLES "D:/SOLO-11/22-avionics-vnav-calc/tests/CMakeLists.txt;4;add_test;D:/SOLO-11/22-avionics-vnav-calc/tests/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test(test_vnav "D:/SOLO-11/22-avionics-vnav-calc/build/tests/Release/test_vnav.exe")
  set_tests_properties(test_vnav PROPERTIES  _BACKTRACE_TRIPLES "D:/SOLO-11/22-avionics-vnav-calc/tests/CMakeLists.txt;4;add_test;D:/SOLO-11/22-avionics-vnav-calc/tests/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
  add_test(test_vnav "D:/SOLO-11/22-avionics-vnav-calc/build/tests/MinSizeRel/test_vnav.exe")
  set_tests_properties(test_vnav PROPERTIES  _BACKTRACE_TRIPLES "D:/SOLO-11/22-avionics-vnav-calc/tests/CMakeLists.txt;4;add_test;D:/SOLO-11/22-avionics-vnav-calc/tests/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test(test_vnav "D:/SOLO-11/22-avionics-vnav-calc/build/tests/RelWithDebInfo/test_vnav.exe")
  set_tests_properties(test_vnav PROPERTIES  _BACKTRACE_TRIPLES "D:/SOLO-11/22-avionics-vnav-calc/tests/CMakeLists.txt;4;add_test;D:/SOLO-11/22-avionics-vnav-calc/tests/CMakeLists.txt;0;")
else()
  add_test(test_vnav NOT_AVAILABLE)
endif()
