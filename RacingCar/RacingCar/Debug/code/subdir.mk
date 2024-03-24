################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/Camera.c \
../code/Car.c \
../code/DataStore.c \
../code/Element.c \
../code/Encoder.c \
../code/Flash_Interface.c \
../code/Function.c \
../code/Menu.c \
../code/Motor.c \
../code/PID.c \
../code/PID_LQR.c \
../code/Steer.c \
../code/System.c \
../code/Vofa.c \
../code/forward.c 

COMPILED_SRCS += \
./code/Camera.src \
./code/Car.src \
./code/DataStore.src \
./code/Element.src \
./code/Encoder.src \
./code/Flash_Interface.src \
./code/Function.src \
./code/Menu.src \
./code/Motor.src \
./code/PID.src \
./code/PID_LQR.src \
./code/Steer.src \
./code/System.src \
./code/Vofa.src \
./code/forward.src 

C_DEPS += \
./code/Camera.d \
./code/Car.d \
./code/DataStore.d \
./code/Element.d \
./code/Encoder.d \
./code/Flash_Interface.d \
./code/Function.d \
./code/Menu.d \
./code/Motor.d \
./code/PID.d \
./code/PID_LQR.d \
./code/Steer.d \
./code/System.d \
./code/Vofa.d \
./code/forward.d 

OBJS += \
./code/Camera.o \
./code/Car.o \
./code/DataStore.o \
./code/Element.o \
./code/Encoder.o \
./code/Flash_Interface.o \
./code/Function.o \
./code/Menu.o \
./code/Motor.o \
./code/PID.o \
./code/PID_LQR.o \
./code/Steer.o \
./code/System.o \
./code/Vofa.o \
./code/forward.o 


# Each subdirectory must supply rules for building sources it contributes
code/%.src: ../code/%.c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/CODE/Abornblatt_Reposity/RacingCar/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<" && \
	if [ -f "$(basename $@).d" ]; then sed.exe -r  -e 's/\b(.+\.o)\b/code\/\1/g' -e 's/\\/\//g' -e 's/\/\//\//g' -e 's/"//g' -e 's/([a-zA-Z]:\/)/\L\1/g' -e 's/\d32:/@TARGET_DELIMITER@/g; s/\\\d32/@ESCAPED_SPACE@/g; s/\d32/\\\d32/g; s/@ESCAPED_SPACE@/\\\d32/g; s/@TARGET_DELIMITER@/\d32:/g' "$(basename $@).d" > "$(basename $@).d_sed" && cp "$(basename $@).d_sed" "$(basename $@).d" && rm -f "$(basename $@).d_sed" 2>/dev/null; else echo 'No dependency file to process';fi
	@echo 'Finished building: $<'
	@echo ' '

code/%.o: ./code/%.src code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-code

clean-code:
	-$(RM) ./code/Camera.d ./code/Camera.o ./code/Camera.src ./code/Car.d ./code/Car.o ./code/Car.src ./code/DataStore.d ./code/DataStore.o ./code/DataStore.src ./code/Element.d ./code/Element.o ./code/Element.src ./code/Encoder.d ./code/Encoder.o ./code/Encoder.src ./code/Flash_Interface.d ./code/Flash_Interface.o ./code/Flash_Interface.src ./code/Function.d ./code/Function.o ./code/Function.src ./code/Menu.d ./code/Menu.o ./code/Menu.src ./code/Motor.d ./code/Motor.o ./code/Motor.src ./code/PID.d ./code/PID.o ./code/PID.src ./code/PID_LQR.d ./code/PID_LQR.o ./code/PID_LQR.src ./code/Steer.d ./code/Steer.o ./code/Steer.src ./code/System.d ./code/System.o ./code/System.src ./code/Vofa.d ./code/Vofa.o ./code/Vofa.src ./code/forward.d ./code/forward.o ./code/forward.src

.PHONY: clean-code

