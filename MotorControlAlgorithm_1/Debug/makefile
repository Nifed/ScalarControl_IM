################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_18.12.2.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./src/03_isr.obj" \
"./src/MotorControlAlgorithm_0.obj" \
"./src/adc_config.obj" \
"./src/main.obj" \
"./src/modulator.obj" \
"./src/qcos.obj" \
"./src/qsin.obj" \
"./src/qsqrt.obj" \
"../28335_RAM_lnk.cmd" \
"../DSP2833x_Headers_nonBIOS.cmd" \
"../lib/IQmath_fpu32.lib" \
"../lib/SFO_TI_Build_fpu.lib" \
"../lib/rts2800_fpu32_fast_supplement.lib" \
$(GEN_CMDS__FLAG) \
-l"C:/Users/nicco/TexasInstWorkspace/MLC_interface_lib/Debug/MLC_interface_lib.lib" \
-l"C:/Users/nicco/TexasInstWorkspace/MRP_MLC28335_MotorControlAlgorithm_1/lib/rts2800_fpu32_fast_supplement.lib" \
-l"C:/Users/nicco/TexasInstWorkspace/MRP_MLC28335_MotorControlAlgorithm_1/lib/SFO_TI_Build_fpu.lib" \
-l"C:/Users/nicco/TexasInstWorkspace/MLC_interface_lib/MLC_iface_Headers.cmd" \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include lib/subdir_vars.mk
-include src/subdir_vars.mk
-include subdir_rules.mk
-include lib/subdir_rules.mk
-include src/subdir_rules.mk
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
MRP_MLC28335_MotorControlAlgorithm_1.out \

EXE_OUTPUTS__QUOTED += \
"MRP_MLC28335_MotorControlAlgorithm_1.out" \


# All Target
all: MRP_MLC28335_MotorControlAlgorithm_1.out

dependents:
	-cd C:/Users/nicco/TexasInstWorkspace/MLC_interface_lib/Debug && $(MAKE) all

# Tool invocations
MRP_MLC28335_MotorControlAlgorithm_1.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS) C:/Users/nicco/TexasInstWorkspace/MLC_interface_lib/Debug/MLC_interface_lib.lib
	@echo 'Building target: "$@"'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_18.12.2.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -g --diag_warning=225 --display_error_number -z -m"MRP_MLC28335_MotorControlAlgorithm_1.map" --stack_size=0x600 --warn_sections -i"C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_18.12.2.LTS/lib" -i"C:/Users/nicco/TexasInstWorkspace/MRP_MLC28335_MotorControlAlgorithm_1/lib" -i"C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_18.12.2.LTS/include" --reread_libs --xml_link_info="MRP_MLC28335_MotorControlAlgorithm_1_linkInfo.xml" --rom_model -o "MRP_MLC28335_MotorControlAlgorithm_1.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "src\03_isr.obj" "src\MotorControlAlgorithm_0.obj" "src\adc_config.obj" "src\main.obj" "src\modulator.obj" "src\qcos.obj" "src\qsin.obj" "src\qsqrt.obj" 
	-$(RM) "src\03_isr.d" "src\MotorControlAlgorithm_0.d" "src\adc_config.d" "src\main.d" "src\modulator.d" 
	-$(RM) "src\qcos.d" "src\qsin.d" "src\qsqrt.d" 
	-@echo 'Finished clean'
	-@echo ' '

post-build:
	-@echo 'Get size of program'
	-"" "MRP_MLC28335_MotorControlAlgorithm_1.map"
	-"C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_18.12.2.LTS/bin/ofd2000" -xg --obj_display=none --dwarf_display=none,dinfo,types "MRP_MLC28335_MotorControlAlgorithm_1.out" > "MRP_MLC28335_MotorControlAlgorithm_1.xml"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:
C:/Users/nicco/TexasInstWorkspace/MLC_interface_lib/Debug/MLC_interface_lib.lib:

-include ../makefile.targets

