################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../main.cpp \
../port.cpp \
../rtklib.cpp \
../sbp.cpp \
../sbp_functions.cpp 

OBJS += \
./main.o \
./port.o \
./rtklib.o \
./sbp.o \
./sbp_functions.o 

CPP_DEPS += \
./main.d \
./port.d \
./rtklib.d \
./sbp.d \
./sbp_functions.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/allard/Documents/Piksi/rtklib" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


