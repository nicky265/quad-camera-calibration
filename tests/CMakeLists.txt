# 端到端冒烟测试：用仓库自带样本数据拼图。
# 显式传 --config，保证构建目录在仓库外时也能找到标定/帧数据。
set(PANOSTITCH_TEST_ARGS
  --config "${PROJECT_SOURCE_DIR}/config.json"
  --frames "${PROJECT_SOURCE_DIR}/data/frames"
  --threads 2
)

add_test(NAME panostitch_smoke_feather
  COMMAND $<TARGET_FILE:panostitch>
          ${PANOSTITCH_TEST_ARGS}
          --out "${CMAKE_CURRENT_BINARY_DIR}/smoke_feather.jpg"
          --blend feather)

add_test(NAME panostitch_smoke_multiband
  COMMAND $<TARGET_FILE:panostitch>
          ${PANOSTITCH_TEST_ARGS}
          --out "${CMAKE_CURRENT_BINARY_DIR}/smoke_multiband.jpg"
          --blend multiband
          --seam)

set_tests_properties(panostitch_smoke_feather panostitch_smoke_multiband PROPERTIES
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  PASS_REGULAR_EXPRESSION "saved:"
  TIMEOUT 120)
