DISPLAY_MOD_DIR := $(USERMOD_DIR)

# stm32f469
ifeq ($(CMSIS_MCU),STM32F469xx)
# The module itself
SRC_USERMOD += $(DISPLAY_MOD_DIR)/display.c

# LCD display support
SRC_USERMOD += $(DISPLAY_MOD_DIR)/BSP_DISCO_F469NI/Utilities/Fonts/font12.c
SRC_USERMOD += $(DISPLAY_MOD_DIR)/BSP_DISCO_F469NI/Utilities/Fonts/font16.c
SRC_USERMOD += $(DISPLAY_MOD_DIR)/BSP_DISCO_F469NI/Utilities/Fonts/font20.c
SRC_USERMOD += $(DISPLAY_MOD_DIR)/BSP_DISCO_F469NI/Utilities/Fonts/font24.c
SRC_USERMOD += $(DISPLAY_MOD_DIR)/BSP_DISCO_F469NI/Utilities/Fonts/font8.c
SRC_USERMOD += $(DISPLAY_MOD_DIR)/BSP_DISCO_F469NI/Drivers/BSP/STM32469I-Discovery/stm32469i_discovery.c
SRC_USERMOD += $(DISPLAY_MOD_DIR)/BSP_DISCO_F469NI/Drivers/BSP/STM32469I-Discovery/stm32469i_discovery_lcd.c
SRC_USERMOD += $(DISPLAY_MOD_DIR)/BSP_DISCO_F469NI/Drivers/BSP/STM32469I-Discovery/stm32469i_discovery_sdram.c
SRC_USERMOD += $(DISPLAY_MOD_DIR)/BSP_DISCO_F469NI/Drivers/BSP/Components/otm8009a/otm8009a.c

# Touchscreen support
SRC_USERMOD += $(DISPLAY_MOD_DIR)/BSP_DISCO_F469NI/Drivers/BSP/STM32469I-Discovery/stm32469i_discovery_ts.c
SRC_USERMOD += $(DISPLAY_MOD_DIR)/BSP_DISCO_F469NI/Drivers/BSP/Components/ft6x06/ft6x06.c

# Only STM32 series is currently supported
ifeq ($(MCU_SERIES),$(filter $(MCU_SERIES),f0 f4 f7 l0 l4 wb))

# Reproduce environment variables from main makefile (not defined at the moment)
DISPLAY_MPY_DIR = $(USERMOD_DIR)/../../micropython
DISPLAY_MCU_SERIES_UPPER = $(shell echo $(MCU_SERIES) | tr '[:lower:]' '[:upper:]')
DISPLAY_HAL_DIR = $(DISPLAY_MPY_DIR)/lib/stm32lib/STM32$(DISPLAY_MCU_SERIES_UPPER)xx_HAL_Driver

else # MCU_SERIES == [f0, f4, f7, l0, l4, wb]

$(error Unsupported platform)

endif # MCU_SERIES == [f0, f4, f7, l0, l4, wb]

# FIXME: this should be included automatically 
# as these files are in micropython repo as well
# Probably changes in mpboardconfigport.mk or some #defines can help
SRC_USERMOD += $(DISPLAY_HAL_DIR)/Src/stm32f4xx_hal_dsi.c

# display driver
SRC_USERMOD += $(DISPLAY_MOD_DIR)/lv_stm_hal/lv_stm_hal.c

# roboto mono font
SRC_USERMOD += $(DISPLAY_MOD_DIR)/fonts/font_roboto_mono_28.c
SRC_USERMOD += $(DISPLAY_MOD_DIR)/fonts/font_roboto_mono_22.c
SRC_USERMOD += $(DISPLAY_MOD_DIR)/fonts/font_roboto_mono_16.c
SRC_USERMOD += $(DISPLAY_MOD_DIR)/fonts/font_roboto_mono_12.c

# Dirs with header files
CFLAGS_USERMOD += -I$(DISPLAY_MOD_DIR)
CFLAGS_USERMOD += -I$(DISPLAY_MOD_DIR)/lv_stm_hal
CFLAGS_USERMOD += -I$(DISPLAY_MOD_DIR)/BSP_DISCO_F469NI
CFLAGS_USERMOD += -I$(DISPLAY_MOD_DIR)/BSP_DISCO_F469NI/Drivers/BSP/STM32469I-Discovery

endif

ifneq ($(UNAME_S),)
# unixport (mac / linux)

# The module itself
SRC_USERMOD += $(DISPLAY_MOD_DIR)/display_unix.c

# roboto mono font
SRC_USERMOD += $(DISPLAY_MOD_DIR)/fonts/font_roboto_mono_28.c
SRC_USERMOD += $(DISPLAY_MOD_DIR)/fonts/font_roboto_mono_22.c
SRC_USERMOD += $(DISPLAY_MOD_DIR)/fonts/font_roboto_mono_16.c
SRC_USERMOD += $(DISPLAY_MOD_DIR)/fonts/font_roboto_mono_12.c

# Dirs with header files
CFLAGS_USERMOD += -I$(DISPLAY_MOD_DIR)

LDFLAGS_MOD += -lSDL2

endif
