ROM_NAME=monitor

ASM_SOURCES=monitor.s via.s
FIRMWARE_CFG=firmware.cfg

CA65_BINARY=/usr/bin/ca65
CC65_BINARY=/usr/bin/cc65
LD65_BINARY=/usr/bin/ld65
AR65_BINARY=/usr/bin/ar65

CPU_FLAG=--cpu 65C02
ARCH_FLAG=-t none
EXTRA_FLAGS=--target none

CC65_FLAGS=$(CPU_FLAG) $(ARCH_FLAG) $(EXTRA_FLAGS) -O
CA65_FLAGS=$(CPU_FLAG) $(EXTRA_FLAGS)
LD65_FLAGS=
AR65_FLAGS=r

# Standard utilities (rm/mkdir)
RM_BINARY=rm
RM_FLAGS=-f
MKDIR_BINARY=mkdir
MKDIR_FLAGS=-p
CP_BINARY=cp
CP_FLAGS=-f

BUILD_FOLDER=../build
TEMP_FOLDER=$(BUILD_FOLDER)/$(ROM_NAME)
ROM_FILE=$(BUILD_FOLDER)/$(ROM_NAME).bin
MAP_FILE=$(TEMP_FOLDER)/$(ROM_NAME).map
ASM_OBJECTS=$(ASM_SOURCES:%.s=$(TEMP_FOLDER)/%.o)

# Compile assembler sources
$(TEMP_FOLDER)/%.o: %.s
	@$(MKDIR_BINARY) $(MKDIR_FLAGS) $(TEMP_FOLDER)
	$(CA65_BINARY) $(CA65_FLAGS) -o $@ -l $(@:.o=.lst) $<
	
# Link ROM image
$(ROM_FILE): $(ASM_OBJECTS) $(FIRMWARE_CFG)
	@$(MKDIR_BINARY) $(MKDIR_FLAGS) $(BUILD_FOLDER)
	$(LD65_BINARY) $(LD65_FLAGS) -C $(FIRMWARE_CFG) -o $@ -m $(MAP_FILE) $(ASM_OBJECTS)

# Default target
all: $(ROM_FILE)

# Build and dump output
test: $(ROM_FILE)
	$(HEXDUMP_BINARY) $(HEXDUMP_FLAGS) $<
	$(MD5_BINARY) $<

# Clean build artifacts
clean:
	$(RM_BINARY) -f $(ROM_FILE) \
	$(MAP_FILE) \
	$(ASM_OBJECTS) \
	$(ASM_OBJECTS:%.o=%.lst)
