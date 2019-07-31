################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs910/ccs/tools/compiler/ti-cgt-msp430_18.12.2.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./low_level_init.obj" \
"./src/app.obj" \
"./src/driver/cs.obj" \
"./src/driver/eusci_a_spi.obj" \
"./src/driver/eusci_a_uart.obj" \
"./src/driver/eusci_b_i2c.obj" \
"./src/driver/eusci_b_spi.obj" \
"./src/driver/flashctl.obj" \
"./src/driver/gpio.obj" \
"./src/driver/mpy.obj" \
"./src/driver/pmm.obj" \
"./src/driver/sd24.obj" \
"./src/driver/sfr.obj" \
"./src/driver/timer_a.obj" \
"./src/driver/tlv.obj" \
"./src/driver/wdt.obj" \
"./src/task/epower.obj" \
"./src/task/rtc.obj" \
"./src/task/serial.obj" \
"../lnk_msp430i2041.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include src/subdir_vars.mk
-include src/driver/subdir_vars.mk
-include src/task/subdir_vars.mk
-include subdir_rules.mk
-include src/subdir_rules.mk
-include src/driver/subdir_rules.mk
-include src/task/subdir_rules.mk
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
smart_plug_i2040s.out \

EXE_OUTPUTS__QUOTED += \
"smart_plug_i2040s.out" \

BIN_OUTPUTS += \
smart_plug_i2040s.hex \

BIN_OUTPUTS__QUOTED += \
"smart_plug_i2040s.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "smart_plug_i2040s.out"

# Tool invocations
smart_plug_i2040s.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: MSP430 Linker'
	"C:/ti/ccs910/ccs/tools/compiler/ti-cgt-msp430_18.12.2.LTS/bin/cl430" -vmsp -O0 --opt_for_speed=0 --use_hw_mpy=16 --advice:power=all --define=__MSP430i2041__ -g --c99 --c++14 --printf_support=minimal --diag_warning=225 --diag_wrap=off --display_error_number -z -m"smart_plug_i2040s.map" --heap_size=80 --stack_size=80 -i"C:/ti/ccs910/ccs/ccs_base/msp430/include" -i"C:/ti/ccs910/ccs/tools/compiler/ti-cgt-msp430_18.12.2.LTS/lib" -i"C:/ti/ccs910/ccs/tools/compiler/ti-cgt-msp430_18.12.2.LTS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="smart_plug_i2040s_linkInfo.xml" --use_hw_mpy=16 --rom_model -o "smart_plug_i2040s.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

smart_plug_i2040s.hex: $(EXE_OUTPUTS)
	@echo 'Building files: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo 'Invoking: MSP430 Hex Utility'
	"C:/ti/ccs910/ccs/tools/compiler/ti-cgt-msp430_18.12.2.LTS/bin/hex430" --memwidth=8 --romwidth=8 -o "smart_plug_i2040s.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "low_level_init.obj" "src\app.obj" "src\driver\cs.obj" "src\driver\eusci_a_spi.obj" "src\driver\eusci_a_uart.obj" "src\driver\eusci_b_i2c.obj" "src\driver\eusci_b_spi.obj" "src\driver\flashctl.obj" "src\driver\gpio.obj" "src\driver\mpy.obj" "src\driver\pmm.obj" "src\driver\sd24.obj" "src\driver\sfr.obj" "src\driver\timer_a.obj" "src\driver\tlv.obj" "src\driver\wdt.obj" "src\task\epower.obj" "src\task\rtc.obj" "src\task\serial.obj" 
	-$(RM) "low_level_init.d" "src\app.d" "src\driver\cs.d" "src\driver\eusci_a_spi.d" "src\driver\eusci_a_uart.d" "src\driver\eusci_b_i2c.d" "src\driver\eusci_b_spi.d" "src\driver\flashctl.d" "src\driver\gpio.d" "src\driver\mpy.d" "src\driver\pmm.d" "src\driver\sd24.d" "src\driver\sfr.d" "src\driver\timer_a.d" "src\driver\tlv.d" "src\driver\wdt.d" "src\task\epower.d" "src\task\rtc.d" "src\task\serial.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

