
ament_auto_add_executable(test_govern_server_exec
  test_govern_server.cpp
  govern_server_tester.cpp
)
target_link_libraries(test_govern_server_exec govern_core)
