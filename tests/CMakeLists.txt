# 端到端冒烟测试：用仓库自带样本数据拼一张图
add_test(NAME panostitch_smoke
  COMMAND $<TARGET_FILE:panostitch>
          --frames "${PROJECT_SOURCE_DIR}/data/frames"
          --out "${CMAKE_CURRENT_BINARY_DIR}/smoke.jpg"
          --blend feather
          --threads 2)

set_tests_properties(panostitch_smoke PROPERTIES
  WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
  PASS_REGULAR_EXPRESSION "saved:"
  TIMEOUT 120)
