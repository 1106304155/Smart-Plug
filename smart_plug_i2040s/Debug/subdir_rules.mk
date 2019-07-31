################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: MSP430 Compiler'
	"C:/ti/ccs910/ccs/tools/compiler/ti-cgt-msp430_18.12.2.LTS/bin/cl430" -vmsp -O0 --opt_for_speed=0 --use_hw_mpy=16 --include_path="C:/ti/ccs910/ccs/ccs_base/msp430/include" --include_path="D:/TEMP/smart_plug/smart_plug_i2040s" --include_path="C:/ti/ccs910/ccs/tools/compiler/ti-cgt-msp430_18.12.2.LTS/include" --advice:power=all --define=__MSP430i2041__ -g --c99 --c++14 --printf_support=minimal --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


