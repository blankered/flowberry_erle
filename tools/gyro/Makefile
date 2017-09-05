# Set to @ if you want to suppress command echo
CMD_ECHO = @

# Project name
BIN = gyro

# Important directories
BUILD_DIR = ../build

# Include paths
INC = -I. \
      -I../../src

# C ource files outside src directory
SRC_C = 

# Defines required by included libraries
DEF =
DEF += -DDEBUG

# Compiler and linker flags
ARCHFLAGS =
DBGFLAGS = -ggdb

# CC: Place functions and data into separate sections to allow dead code removal
# by the linker (-f*-sections). Enable link time optimization (-flto)
CFLAGS = $(ARCHFLAGS) -std=gnu99 -Wall -Wno-format -Werror \
         -Wno-error=unused-function -Wno-error=unused-variable \
         -ffunction-sections -fdata-sections \
         -Os -flto $(DBGFLAGS)

# LD: Remove unused sections, link with newlib-nano implementation, generate map
LDFLAGS = $(ARCHFLAGS) -Wl,--gc-sections
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(BIN).map $(DBGFLAGS)

# Generate object list from source files and add their dirs to search path
SRC_C += $(wildcard *.c) 
FILENAMES_C = $(notdir $(SRC_C))
OBJS_C = $(addprefix $(BUILD_DIR)/, $(FILENAMES_C:.c=.o))
vpath %.c $(dir $(SRC_C))

# Tools selection
CC = gcc
LD = gcc
NM = nm
OBJCOPY = objcopy
OBJDUMP = objdump
SIZE = size

all: $(BUILD_DIR) $(BUILD_DIR)/$(BIN)
	@echo ""
	$(CMD_ECHO) @$(SIZE) $(BUILD_DIR)/$(BIN)

$(BUILD_DIR):
	$(CMD_ECHO) mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: %.c
	@echo "Compiling C file: $(notdir $<)"
	$(CMD_ECHO) $(CC) $(CFLAGS) $(DEF) $(INC) -c -o $@ $<

$(BUILD_DIR)/$(BIN): $(OBJS_C)
	@echo "Linking binary: $(notdir $@)"
	$(CMD_ECHO) $(LD) $(LDFLAGS) -o $@ $^
	
	@echo "Generating name list: $(BIN).sym"
	$(CMD_ECHO) $(NM) -n $@ > $(BUILD_DIR)/$(BIN).sym

	@echo "Generating disassembly: $(BIN).disasm"
	$(CMD_ECHO) $(OBJDUMP) -S $@ > $(BUILD_DIR)/$(BIN).disasm

clean:
	rm -f $(BUILD_DIR)/*.elf $(BUILD_DIR)/*.hex $(BUILD_DIR)/*.map
	rm -f $(BUILD_DIR)/*.o $(BUILD_DIR)/*.sym $(BUILD_DIR)/*.disasm
	rm -f $(BUILD_DIR)/*.gdbinit
