
CPPSRC := $(wildcard src/*.cpp)                  \
          $(wildcard src/sys/*.cpp)              
          
		  
CSRC   := $(wildcard stm32_lib/src/*.c) \
          $(wildcard src/sys/*.c)       \
          $(wildcard stm32_lib/periphs/src/*.c)

DEF = -DFW_VERSION_MAJOR=1 -DFW_VERSION_MINOR=0

INC = -Isrc/sys                         \
      -isystem stm32_lib/inc            \
      -isystem stm32_lib/periphs/inc   

STARTUP = src/sys/startup.s

#
# UAVCAN library
#

DEF +=  -DUAVCAN_STM32_BAREMETAL=1         \
        -DUAVCAN_STM32_TIMER_NUMBER=4    \
        -DUAVCAN_TINY=0                  \
        -DUAVCAN_STM32_NUM_IFACES=1      \
        -DUAVCAN_MEM_POOL_BLOCK_SIZE=48

include libuavcan/libuavcan/include.mk
CPPSRC += $(LIBUAVCAN_SRC)
INC += -I$(LIBUAVCAN_INC)

include libuavcan/libuavcan_drivers/stm32/driver/include.mk
CPPSRC += $(LIBUAVCAN_STM32_SRC)
INC += -I$(LIBUAVCAN_STM32_INC)

$(info $(shell $(LIBUAVCAN_DSDLC) $(UAVCAN_DSDL_DIR)))
INC += -Idsdlc_generated

#
# Build configuration
#

BUILDDIR = build
OBJDIR = $(BUILDDIR)/obj
DEPDIR = $(BUILDDIR)/dep

DEF += -DNDEBUG -DSTM32F437xx -DCORE_M4 -DARM_MATH_CM4 -DTHUMB_NO_INTERWORKING -U__STRICT_ANSI__ -D__FPU_USED

FLAGS = -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mno-thumb-interwork -O0 -g3 -Wall -Wextra -Wundef -ffunction-sections \
        -fdata-sections -fno-common -fno-exceptions -fno-unwind-tables -fno-stack-protector -fomit-frame-pointer \
        -Wfloat-equal -Wconversion -Wsign-conversion -Wmissing-declarations

C_CPP_FLAGS = $(FLAGS) -MD -MP -MF $(DEPDIR)/$(@F).d

CFLAGS = $(C_CPP_FLAGS) -std=c99 -I/home/abhishek/xmake/stm32_lib

CPPFLAGS = $(C_CPP_FLAGS) -pedantic -std=c++11 -fno-rtti -fno-threadsafe-statics -I/home/abhishek/xmake/stm32_lib

LDFLAGS = $(FLAGS) -nodefaultlibs -L/home/abhishek/xmake/stm32_lib -larm_cortexM4lf_math  \
                   -lstdc++ -lm -lc -lgcc -lnosys -Tstm32f437.ld -Xlinker --gc-sections \
                   -Wl,-Map,$(BUILDDIR)/output.map 

# Link with nano newlib. Other toolchains may not support this option, so it can be safely removed.
LDFLAGS += --specs=nano.specs

COBJ   = $(addprefix $(OBJDIR)/, $(notdir $(CSRC:.c=.o)))
CPPOBJ = $(addprefix $(OBJDIR)/, $(notdir $(CPPSRC:.cpp=.o)))
SOBJ   = $(addprefix $(OBJDIR)/,$(notdir $(STARTUP:.s=.o)))
OBJ = $(COBJ) $(CPPOBJ) $(SOBJ)

VPATH = $(sort $(dir $(CSRC)) $(dir $(CPPSRC)))

ELF = $(BUILDDIR)/firmware.elf
BIN = $(BUILDDIR)/firmware.bin
HEX = $(BUILDDIR)/firmware.hex

#
# Rules
#

TOOLCHAIN ?= arm-none-eabi-
CC   = $(TOOLCHAIN)gcc
CPPC = $(TOOLCHAIN)g++
AS   = $(TOOLCHAIN)as
LD   = $(TOOLCHAIN)g++
CP   = $(TOOLCHAIN)objcopy
SIZE = $(TOOLCHAIN)size



all: $(OBJ) $(ELF) $(BIN) $(HEX) size 

$(OBJ): | $(BUILDDIR)

$(BUILDDIR):
	@mkdir -p $(BUILDDIR)
	@mkdir -p $(DEPDIR)
	@mkdir -p $(OBJDIR)

$(HEX): $(ELF)
	@echo
	$(CP) -O ihex $(ELF) $@

$(BIN): $(ELF)
	@echo
	$(CP) -O binary $(ELF) $@

$(ELF): $(OBJ)
	@echo
	$(LD) $(OBJ) $(LDFLAGS) -o $@

$(COBJ): $(OBJDIR)/%.o: %.c
	@echo
	$(CC) -c $(DEF) $(INC) $(CFLAGS) $< -o $@

$(CPPOBJ): $(OBJDIR)/%.o: %.cpp 
	@echo
	$(CPPC) -c $(DEF) $(INC) $(CPPFLAGS) $< -o $@

$(SOBJ): $(OBJDIR)/%.o: %.s
	@echo
	$(AS) -c -mthumb $< -o $@
	
clean:
	rm -rf $(BUILDDIR) dsdlc_generated

size: $(ELF)
	@if [ -f $(ELF) ]; then echo; $(SIZE) $(ELF); echo; fi;

.PHONY: all clean size $(BUILDDIR)

-include $(shell mkdir $(DEPDIR) 2>/dev/null) $(wildcard $(DEPDIR)/*)
