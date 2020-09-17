
idf_component_register(SRC_DIRS 
                       "port/source" 
                       "lvgl/src/lv_core" 
                       "lvgl/src/lv_draw"
                       "lvgl/src/lv_font"
                       "lvgl/src/lv_gpu"
                       "lvgl/src/lv_hal"
                       "lvgl/src/lv_misc"
                       "lvgl/src/lv_themes"
                       "lvgl/src/lv_widgets"       
                       INCLUDE_DIRS "port/include" "."
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "driver"
                       REQUIRES "lcd" "board")

target_compile_options(${COMPONENT_LIB} PUBLIC -DLV_CONF_INCLUDE_SIMPLE)