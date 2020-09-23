# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# set(EXTRA_COMPONENT_DIRS "../../components/board"
#                          "../../components/lcd"
#                          "../../components/jpeg"
#                          "../../components/lvgl"
#                          "../../components/i2s_parallel"
#                          "../../components/esp_rainmaker/components/json_generator"
#                          "../../components/esp_rainmaker/components/json_parser"
#                          "../../components/esp_rainmaker/components/esp_rainmaker"
#                          "../../components/esp_rainmaker/example/common"
# )

set(EXTRA_COMPONENT_DIRS "../../components"
                         "../../esp-rainmaker/components"
                         "../../esp-rainmaker/examples/common"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
add_compile_options (-fdiagnostics-color=always)
project(esp32-s2-hmi)
