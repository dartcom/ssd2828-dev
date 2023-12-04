PROGRAMMING_PORT = ...

BUILD_DIR = build
SRC_DIR = src
LIBS_DIR = libs
SYSTEM_DIR = system


CPU = cortex-m4
FPU = fpv4-sp-d16


START_SCRIPT = startup_stm32f401xc.s
INIT_SCRIPT = system_stm32f4xx.c
LD_SCRIPT = STM32F401CCUX_FLASH.ld
TARGET = main

ASFLAGS += -c
ASFLAGS += -O0
ASFLAGS += -mcpu=$(CPU)
ASFLAGS += -mthumb
ASFLAGS += -mfpu=$(FPU)
ASFLAGS += -mfloat-abi=hard
ASFLAGS += -fmessage-length=0

CFLAGS += -mcpu=$(CPU)
CFLAGS += -mthumb
CFLAGS += -mfpu=$(FPU)
CFLAGS += -mfloat-abi=hard
CFLAGS += -Wall
CFLAGS += -g3 -gdwarf-2
CFLAGS += -fmessage-length=0
CFLAGS += -specs=nosys.specs

LSCRIPT = $(SYSTEM_DIR)/$(LD_SCRIPT)
LFLAGS += -mcpu=$(CPU)
LFLAGS += -mthumb
LFLAGS += -mfpu=$(FPU)
LFLAGS += -mfloat-abi=hard
LFLAGS += -Wall
LFLAGS += -specs=nosys.specs
LFLAGS += -lgcc
LFLAGS += -T$(LSCRIPT)
LFLAGS += -lc  
LFLAGS += -lm
LFLAGS += -lnosys 



CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
OC = arm-none-eabi-objcopy
OD = arm-none-eabi-objdump
OS = arm-none-eabi-size



AS_SRC = 	$(START_SCRIPT)
C_SRC = 	$(INIT_SCRIPT) main.c delay.c spi.c ssd2828.c clk.c 

OBJS = 	$(AS_SRC:%.s=$(BUILD_DIR)/%.o)
OBJS += $(C_SRC:%.c=$(BUILD_DIR)/%.o)

all: $(BUILD_DIR)/$(TARGET).bin


$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OC) -S -O binary $^ $@
	$(OS) $^

$(BUILD_DIR)/$(TARGET).elf: $(OBJS)
	$(CC) $^ $(LFLAGS) -o $@


$(BUILD_DIR)/%.o: $(SYSTEM_DIR)/%.s
	$(CC) -x assembler-with-cpp $(ASFLAGS) $^ -o $@

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.s
	$(CC) -x assembler-with-cpp $(ASFLAGS) $^ -o $@

$(BUILD_DIR)/%.o: $(LIBS_DIR)/%.s
	$(CC) -x assembler-with-cpp $(ASFLAGS) $^ -o $@

$(BUILD_DIR)/%.o: $(SYSTEM_DIR)/%.c
	$(CC) -c $(CFLAGS) $(INCLUDE) $< -o $@

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) -c $(CFLAGS) $(INCLUDE) $< -o $@

$(BUILD_DIR)/%.o: $(LIBS_DIR)/%.c
	$(CC) -c $(CFLAGS) $(INCLUDE) $< -o $@

clean: $(BUILD_DIR)/$(TARGET).bin
	-rm -fR $(BUILD_DIR)/*.o
	
flash_cmsis_dap: $(BUILD_DIR)/$(TARGET).elf
	openocd -f interface/cmsis-dap.cfg -f board/atmel_sam3x_ek.cfg -c "program $< verify reset exit"

flash_stlink: $(BUILD_DIR)/$(TARGET).elf
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"