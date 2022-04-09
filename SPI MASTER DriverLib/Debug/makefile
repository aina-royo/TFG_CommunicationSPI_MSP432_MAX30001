################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./adc14.obj" \
"./aes256.obj" \
"./comp_e.obj" \
"./cpu.obj" \
"./crc32.obj" \
"./cs.obj" \
"./dma.obj" \
"./flash.obj" \
"./fpu.obj" \
"./gpio.obj" \
"./i2c.obj" \
"./interrupt.obj" \
"./main.obj" \
"./mpu.obj" \
"./pcm.obj" \
"./pmap.obj" \
"./pss.obj" \
"./ref_a.obj" \
"./reset.obj" \
"./rtc_c.obj" \
"./spi.obj" \
"./startup_msp432p401r_ccs.obj" \
"./sysctl.obj" \
"./system_msp432p401r.obj" \
"./systick.obj" \
"./timer32.obj" \
"./timer_a.obj" \
"./uart.obj" \
"./wdt_a.obj" \
"../msp432p401r.cmd" \
"../ccs/msp432p4xx_driverlib.lib" \
"../keil/msp432p4xx_driverlib.lib" \
$(GEN_CMDS__FLAG) \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include ccs/subdir_vars.mk
-include ewarm/subdir_vars.mk
-include gcc/subdir_vars.mk
-include keil/subdir_vars.mk
-include subdir_rules.mk
-include ccs/subdir_rules.mk
-include ewarm/subdir_rules.mk
-include gcc/subdir_rules.mk
-include keil/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
SPI\ MASTER\ DriverLib.out \

EXE_OUTPUTS__QUOTED += \
"SPI MASTER DriverLib.out" \

BIN_OUTPUTS += \
SPI\ MASTER\ DriverLib.hex \

BIN_OUTPUTS__QUOTED += \
"SPI MASTER DriverLib.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(A_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "SPI MASTER DriverLib.out"

# Tool invocations
SPI\ MASTER\ DriverLib.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(A_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: Arm Linker'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --advice:power=all --define=__MSP432P401R__ --define=ccs -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi -z -m"SPI MASTER DriverLib.map" --heap_size=1024 --stack_size=512 -i"C:/ti/ccs1040/ccs/ccs_base/arm/include" -i"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/lib" -i"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="SPI MASTER DriverLib_linkInfo.xml" --rom_model -o "SPI MASTER DriverLib.out" $(ORDERED_OBJS) $(A_SRCS)
	@echo 'Finished building target: "$@"'
	@echo ' '

SPI\ MASTER\ DriverLib.hex: $(EXE_OUTPUTS)
	@echo 'Building secondary target: "$@"'
	@echo 'Invoking: Arm Hex Utility'
	"C:/ti/ccs1040/ccs/tools/compiler/ti-cgt-arm_20.2.5.LTS/bin/armhex" --memwidth=8 --romwidth=8 --diag_wrap=off -o "SPI MASTER DriverLib.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building secondary target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "adc14.obj" "aes256.obj" "comp_e.obj" "cpu.obj" "crc32.obj" "cs.obj" "dma.obj" "flash.obj" "fpu.obj" "gpio.obj" "i2c.obj" "interrupt.obj" "main.obj" "mpu.obj" "pcm.obj" "pmap.obj" "pss.obj" "ref_a.obj" "reset.obj" "rtc_c.obj" "spi.obj" "startup_msp432p401r_ccs.obj" "sysctl.obj" "system_msp432p401r.obj" "systick.obj" "timer32.obj" "timer_a.obj" "uart.obj" "wdt_a.obj" 
	-$(RM) "adc14.d" "aes256.d" "comp_e.d" "cpu.d" "crc32.d" "cs.d" "dma.d" "flash.d" "fpu.d" "gpio.d" "i2c.d" "interrupt.d" "main.d" "mpu.d" "pcm.d" "pmap.d" "pss.d" "ref_a.d" "reset.d" "rtc_c.d" "spi.d" "startup_msp432p401r_ccs.d" "sysctl.d" "system_msp432p401r.d" "systick.d" "timer32.d" "timer_a.d" "uart.d" "wdt_a.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

