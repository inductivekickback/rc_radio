# The PROJECT_NAME can be changed arbitrarily (it does not depend on anything).
PROJECT_NAME := poly_rx

# The LINKER_SCRIPT is usually in the same directory as the Makefile.
LINKER_SCRIPT := poly_rx_gcc_nrf52.ld

BOARD := PCA10040

DBG_BUILD_DIR := ./_debug
REL_BUILD_DIR := ./_release
DEPS_DIR := ./_deps

SDK_ROOT := ../../../../../../../../..
PROJ_DIR := ../../..

# JLINK_MON_DEBUG_DIR should point to the directory that contains the files
# in JLINK_MON_DEBUG_FILES plus JLINK_MONITOR.h. If Monitor Mode Debugging is
# not required then leave JLINK_MON_DEBUG_DIR empty.
JLINK_MON_DEBUG_DIR :=

# The offset of the application needs to be specified so the J-Link knows
# the location of the DebugMon_Handler. For the S132 v4 this is 0x1F000.
JLINK_MON_DEBUG_APP_FLASH_LOCATION := 0x00

# If the JLINK_MON_DEBUG_DIR is empty then make sure it is evaluated correctly.
JLINK_MON_DEBUG_DIR := $(strip $(JLINK_MON_DEBUG_DIR))

# These are the required files as of version 6.12a of the J-Link driver.
JLINK_MON_DEBUG_FILES := JLINK_MONITOR.c JLINK_MONITOR_ISR_SES.s

# Init commands will be written to a file and then specified when starting GDB
# instead of relying on .gdbinit (to avoid having to enable .gbdinit files).
GDB_CMD_PATH := $(DBG_BUILD_DIR)/gdb.txt

# Include the correct template Makefile depending on platform.
TEMPLATE_PATH = $(SDK_ROOT)/components/toolchain/gcc
ifeq ($(OS),Windows_NT)
	include $(TEMPLATE_PATH)/Makefile.windows
# The Windows command shell 'start' function is used so the executable
# is started in its own window.
	TERMINAL := cmd /c start ""
	TERMINAL_END := 
	NRFJPROG := nrfjprog.exe
	GDBSERVER := JLinkGDBServerCL.exe
	RTT_CLIENT := JLinkRTTClient.exe
else
	include $(TEMPLATE_PATH)/Makefile.posix
	TERMINAL := gnome-terminal -e "
	TERMINAL_END := "
	NRFJPROG := nrfjprog
	GDBSERVER := JLinkGDBServer
	RTT_CLIENT := JLinkRTTClient
endif

# If multiple J-Links are attached to the computer then "SN=1234" can be used
# to specify a serial number for GDB and flash targets. A random GDB port will
# be generated so multiple targets can be debugged simultaneously.
GDB_PORT := 2331
ifdef SN
	NRFJPROG_SN := --snr $(SN)
	GDB_SERVER_SN := -select usb=$(SN)
	GDB_PORT := $(shell awk 'BEGIN{srand();printf("%4.4s", 1000+9999*rand())}')
endif

# Toolchain commands (these rely on the inclusion of the template Makefile)
CC := $(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc
AS := $(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as
AR := $(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar -r
LD := $(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld
NM := $(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm
OBJDUMP := $(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump
OBJCOPY := $(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy
SIZE := $(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size
GDB := $(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gdb

SHAREDFLAGS += \
	-mcpu=cortex-m4 \
	-mthumb \
	-DNRF52 \
	-DNRF52832_XXAA \
	-DCONFIG_GPIO_AS_PINRESET \
	-DBOARD_$(BOARD) \
	-DNRF52_PAN_74 \
	-DBSP_DEFINES_ONLY \
	-DESB_PRESENT

CFLAGS += \
	--std=gnu99 \
	-Wall \
	-Werror \
	-mfloat-abi=hard \
	-mfpu=fpv4-sp-d16 \
	-ffunction-sections \
	-fdata-sections \
	-fno-strict-aliasing \
	-fno-builtin \
	--short-enums \
	-mabi=aapcs \
	$(SHAREDFLAGS)

ASMFLAGS += \
	-x assembler-with-cpp \
	$(SHAREDFLAGS)

LDFLAGS += \
	-mthumb \
	-mabi=aapcs \
	-L $(TEMPLATE_PATH) \
	-T$(LINKER_SCRIPT) \
	-mcpu=cortex-m4 \
	-mfloat-abi=hard \
	-mfpu=fpv4-sp-d16 \
	-Wl,--gc-sections \
	--specs=nano.specs \
	-lc \
	-lnosys

SRC_FILES += \
	$(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_serial.c \
	$(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
	$(SDK_ROOT)/components/boards/boards.c \
	$(SDK_ROOT)/components/libraries/util/app_error.c \
	$(SDK_ROOT)/components/libraries/util/app_error_weak.c \
	$(SDK_ROOT)/components/libraries/util/app_util_platform.c \
	$(SDK_ROOT)/components/libraries/util/nrf_assert.c \
	$(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
	$(SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
	$(SDK_ROOT)/components/drivers_nrf/uart/nrf_drv_uart.c \
	$(PROJ_DIR)/main.c \
	$(PROJ_DIR)/../common/radioshack_micro_servo.c \
	$(PROJ_DIR)/../common/brushed_dc_motor.c \
	$(PROJ_DIR)/../../rc_radio.c \
	$(SDK_ROOT)/external/segger_rtt/RTT_Syscalls_GCC.c \
	$(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
	$(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \
	$(SDK_ROOT)/components/proprietary_rf/esb/nrf_esb.c \
	$(SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf52.S \
	$(SDK_ROOT)/components/toolchain/system_nrf52.c \
	$(SDK_ROOT)/components/drivers_nrf/pwm/nrf_drv_pwm.c \
	$(SDK_ROOT)/components/proprietary_rf/esb/nrf_esb.c \
	$(SDK_ROOT)/components/drivers_nrf/timer/nrf_drv_timer.c

INC_DIRS += \
	$(SDK_ROOT)/components \
	$(SDK_ROOT)/components/libraries/util \
	$(SDK_ROOT)/components/toolchain/gcc \
	$(SDK_ROOT)/components/drivers_nrf/uart \
	../config \
	$(SDK_ROOT)/components/drivers_nrf/common \
	$(SDK_ROOT)/components/libraries/strerror \
	$(SDK_ROOT)/components/proprietary_rf/esb \
	$(PROJ_DIR) \
	$(PROJ_DIR)/../common \
	$(PROJ_DIR)/../.. \
	$(SDK_ROOT)/external/segger_rtt \
	$(SDK_ROOT)/components/libraries/bsp \
	$(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd \
	$(SDK_ROOT)/components/toolchain \
	$(SDK_ROOT)/components/device \
	$(SDK_ROOT)/components/libraries/log \
	$(SDK_ROOT)/components/boards \
	$(SDK_ROOT)/components/drivers_nrf/delay \
	$(SDK_ROOT)/components/toolchain/cmsis/include \
	$(SDK_ROOT)/components/drivers_nrf/hal \
	$(SDK_ROOT)/components/libraries/log/src \
	$(SDK_ROOT)/components/drivers_nrf/pwm \
	$(SDK_ROOT)/components/drivers_nrf/timer

# The Monitor Mode debugging files will always be tracked by make if they are
# used during debugging. They won't be passed to the linker for the release
# version of the build.
DBG_SRC_FILES :=
ifneq "$(JLINK_MON_DEBUG_DIR)" ""
	INC_DIRS += $(JLINK_MON_DEBUG_DIR)
	DBG_SRC_FILES += $(addprefix $(JLINK_MON_DEBUG_DIR)/, $(JLINK_MON_DEBUG_FILES))
else
$(warning Define JLINK_MON_DEBUG_DIR to enable Monitor Mode Debugging.)
endif

ASSEMBLY_EXTS := s S

# Convert to absolute paths and sort to remove duplicates.
SRC_FILES := $(sort $(foreach f,$(SRC_FILES),$(abspath $(f))))
DBG_SRC_FILES := $(sort $(foreach f,$(DBG_SRC_FILES),$(abspath $(f))))
INC_DIRS := $(sort $(foreach d,$(INC_DIRS),$(abspath $(d))))

# Split the file paths into file names and directories. And remove duplicates.
SRC_DIRS := $(sort $(dir $(SRC_FILES)))
SRC_FILE_NAMES := $(notdir $(SRC_FILES))
DBG_SRC_DIRS := $(sort $(dir $(DBG_SRC_FILES)))
DBG_SRC_FILE_NAMES := $(notdir $(DBG_SRC_FILES))

# Convert each source file name into the form '$(OBJ_DIR)/$(SRC_FILE).o'.
OBJ := $(patsubst %,%.o,$(basename $(SRC_FILE_NAMES)))
DBG_OBJ := $(patsubst %,%.o,$(basename $(DBG_SRC_FILE_NAMES)))
DBG_OBJ := $(addprefix $(DBG_BUILD_DIR)/,$(OBJ) $(DBG_OBJ))
REL_OBJ := $(addprefix $(REL_BUILD_DIR)/,$(OBJ))

.PHONY: default
default: CFLAGS += -O0 -ggdb -DDEBUG -DDEBUG_NRF
default: OBJ_DIR := $(DBG_BUILD_DIR)
default: $(DBG_BUILD_DIR)/$(PROJECT_NAME).hex
	$(call echosize,$(DBG_BUILD_DIR)/$(PROJECT_NAME).out)

.PHONY: release
release: CFLAGS += -O3 -g3
release: OBJ_DIR := $(REL_BUILD_DIR)
release: $(REL_BUILD_DIR)/$(PROJECT_NAME).bin $(REL_BUILD_DIR)/$(PROJECT_NAME).hex
	$(call echosize,$(REL_BUILD_DIR)/$(PROJECT_NAME).out)

.PHONY: all
all: default release

DEPS := $(addprefix $(DEPS_DIR)/,$(OBJ:.o=.d))
-include $(DEPS)

INCLUDES := $(patsubst %,-I%,$(INC_DIRS))

# Look for all source files in SRC_DIRS.
vpath % $(SRC_DIRS) $(DBG_SRC_DIRS)

define echosize
	@echo ''
	@'$(SIZE)' $1;
	@echo ''
endef

define flash
	@"$(NRFJPROG)" $(NRFJPROG_SN) --program $1 -f nrf52 --sectorerase --verify
	@"$(NRFJPROG)" $(NRFJPROG_SN) --reset -f nrf52
endef

$(DBG_BUILD_DIR)/%.o: %.c | $(DBG_BUILD_DIR)
	@echo Compiling file: $(notdir $<)
	@'$(CC)' $(CFLAGS) $(INCLUDES) -c -o $@ $<

$(DBG_BUILD_DIR)/$(PROJECT_NAME).hex: $(DBG_BUILD_DIR)/$(PROJECT_NAME).out
	@echo Creating hex file: $(notdir $@)
	@'$(OBJCOPY)' -O ihex $(DBG_BUILD_DIR)/$(PROJECT_NAME).out $@

$(DBG_BUILD_DIR)/$(PROJECT_NAME).bin: $(DBG_BUILD_DIR)/$(PROJECT_NAME).out
	@echo Creating bin file: $(notdir $@)
	@'$(OBJCOPY)' -O binary $(DBG_BUILD_DIR)/$(PROJECT_NAME).out $@

$(DBG_BUILD_DIR)/$(PROJECT_NAME).out: $(DBG_OBJ)
	@echo Linking ELF file: $(notdir $@)
	@'$(CC)' $(LDFLAGS) $(DBG_OBJ) -lm -o $@

$(REL_BUILD_DIR)/%.o: %.c | $(REL_BUILD_DIR)
	@echo Compiling file: $(notdir $<)
	@'$(CC)' $(CFLAGS) $(INCLUDES) -c -o $@ $<

$(REL_BUILD_DIR)/$(PROJECT_NAME).hex: $(REL_BUILD_DIR)/$(PROJECT_NAME).out
	@echo Creating hex file: $(notdir $@)
	@'$(OBJCOPY)' -O ihex $(REL_BUILD_DIR)/$(PROJECT_NAME).out $@

$(REL_BUILD_DIR)/$(PROJECT_NAME).bin: $(REL_BUILD_DIR)/$(PROJECT_NAME).out
	@echo Creating bin file: $(notdir $@)
	@'$(OBJCOPY)' -O binary $(REL_BUILD_DIR)/$(PROJECT_NAME).out $@

$(REL_BUILD_DIR)/$(PROJECT_NAME).out: $(REL_OBJ)
	@echo Linking ELF file: $(notdir $@)
	@'$(CC)' $(LDFLAGS) $(REL_OBJ) -lm -o $@

$(DEPS_DIR)/%.d: %.c | $(DEPS_DIR)
	@echo Adding dependency for file: $(notdir $<)
	@echo -n "$(DBG_BUILD_DIR)/$(notdir $(<:.c=.o)) " > $@
	@'$(CC)' $(CFLAGS) $(INCLUDES) -c $< -MM \
		-MT $(REL_BUILD_DIR)/$(notdir $(<:.c=.o)) >> $@

# The eval command is used to create a different version of these targets for
# each extension in the ASSEMBLY_EXTS list.
define assembly_targets
$$(DBG_BUILD_DIR)/%.o: %.$1 | $$(DBG_BUILD_DIR)
	@echo Compiling assembly file: $$(notdir $$<)
	@'$$(CC)' $$(ASMFLAGS) $$(INCLUDES) -c -o $$@ $$<

$$(REL_BUILD_DIR)/%.o: %.$1 | $$(REL_BUILD_DIR)
	@echo Compiling assembly file: $$(notdir $$<)
	@'$$(CC)' $$(ASMFLAGS) $$(INCLUDES) -c -o $$@ $$<

$$(DEPS_DIR)/%.d: %.$1 | $$(DEPS_DIR)
	@echo Adding dependency for file: $$(notdir $$<)
	@echo -n "$$(DBG_BUILD_DIR)/$$(notdir $$(<:.$1=.o)) " > $$@
	@'$$(CC)' $$(ASMFLAGS) $$(INCLUDES) -c $$< -MM \
		-MT $$(REL_BUILD_DIR)/$$(notdir $$(<:.$1=.o)) >> $$@
endef
$(foreach EXT,$(ASSEMBLY_EXTS),$(eval $(call assembly_targets,$(EXT))))

$(DEPS_DIR) $(DBG_BUILD_DIR) $(REL_BUILD_DIR):; @mkdir $@

.PHONY: flash_debug
flash_debug: default
	$(call flash,$(DBG_BUILD_DIR)/$(PROJECT_NAME).hex)

.PHONY: flash_release
flash_release: release
	$(call flash,$(REL_BUILD_DIR)/$(PROJECT_NAME).hex)

.PHONY: gdb_cmd_file
gdb_cmd_file: default
	@echo "target remote localhost:$(GDB_PORT)" > $(GDB_CMD_PATH)
	@echo "mon speed 10000" >> $(GDB_CMD_PATH)
	@echo "mon flash download=1" >> $(GDB_CMD_PATH)
	@echo "load $(DBG_BUILD_DIR)/$(PROJECT_NAME).out" >> $(GDB_CMD_PATH)
	@echo "break main" >> $(GDB_CMD_PATH)
	@echo "mon reset 0" >> $(GDB_CMD_PATH)
	@echo "c" >> $(GDB_CMD_PATH)

.PHONY: gdb_mon_cmd_file
gdb_mon_cmd_file: gdb_cmd_file
	@echo "mon exec SetMonModeDebug=1" >> $(GDB_CMD_PATH)
	@echo "mon exec SetMonModeVTableAddr=$(JLINK_MON_DEBUG_APP_FLASH_LOCATION)" >> $(GDB_CMD_PATH)

.PHONY: gdb_cli
gdb_cli:
	@$(TERMINAL) '$(GDBSERVER)' $(GDB_SERVER_SN) -device nrf52832_XXAA \
		-if swd -port $(GDB_PORT) $(TERMINAL_END)
	@$(TERMINAL) '$(GDB)' $(DBG_BUILD_DIR)/$(PROJECT_NAME).out -x $(GDB_CMD_PATH) $(TERMINAL_END)

.PHONY: rtt
rtt:
	@$(TERMINAL) '$(RTT_CLIENT)' $(TERMINAL_END)

.PHONY: gdb
gdb: gdb_cmd_file gdb_cli

.PHONY: gdb_mon
gdb_mon: gdb_mon_cmd_file gdb_cli

.PHONY: gdb_rtt
gdb_rtt: gdb rtt

.PHONY: gdb_mon_rtt
gdb_mon_rtt: gdb_mon rtt

.PHONY: clean
clean:
	@rm -rf $(DBG_BUILD_DIR)
	@rm -rf $(REL_BUILD_DIR)
	@rm -rf $(DEPS_DIR)

.PHONY: help
help:
	@echo "The following targets are available:"
	@echo "    (default)              - Compile with debug flags"
	@echo "    release                - Compile with release flags"
	@echo "    all                    - Compile debug and release targets"
	@echo "    clean                  - Remove object and dependency directories"
	@echo "    gdb [SN=1234]          - Perform debug compile and then launch gdb"
	@echo "    gdb_rtt [SN=1234]      - Call gdb target and then open RTT Client"
ifneq "$(JLINK_MON_DEBUG_DIR)" ""
	@echo "    gdb_mon [SN=1234]      - Enable Monitor Mode Debugging"
	@echo "    gdb_mon_rtt [SN=1234]  - Enable Mon Debugging and open RTT Client"
endif
	@echo "    flash_debug [SN=1234]  - Flash the debug build"
	@echo "    flash_release [SN=1234]- Flash the release build"

# This is a special target that tells make to delete a file if an error occurs
# while the file is being generated.
.DELETE_ON_ERROR:
