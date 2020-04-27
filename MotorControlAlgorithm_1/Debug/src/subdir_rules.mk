################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
src/%.obj: ../src/%.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_18.12.2.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="C:/Users/nicco/TexasInstWorkspace/MLC_interface_lib/include" --include_path="C:/Users/nicco/TexasInstWorkspace/MLC_interface_lib/include_local" --include_path="C:/Users/nicco/TexasInstWorkspace/MLC_interface_lib/includeTI" --include_path="C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_18.12.2.LTS/include" --include_path="C:/Users/nicco/TexasInstWorkspace/MRP_MLC28335_MotorControlAlgorithm_1/include" -g --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="src/$(basename $(<F)).d_raw" --obj_directory="src" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

src/%.obj: ../src/%.asm $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_18.12.2.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="C:/Users/nicco/TexasInstWorkspace/MLC_interface_lib/include" --include_path="C:/Users/nicco/TexasInstWorkspace/MLC_interface_lib/include_local" --include_path="C:/Users/nicco/TexasInstWorkspace/MLC_interface_lib/includeTI" --include_path="C:/ti/ccs910/ccs/tools/compiler/ti-cgt-c2000_18.12.2.LTS/include" --include_path="C:/Users/nicco/TexasInstWorkspace/MRP_MLC28335_MotorControlAlgorithm_1/include" -g --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="src/$(basename $(<F)).d_raw" --obj_directory="src" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


