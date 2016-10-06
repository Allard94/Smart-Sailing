################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../ephemeris.cpp \
../main.cpp \
../observation.cpp \
../port.cpp \
../sbp.cpp \
../sbp_functions.cpp \
../signal.cpp \
../solution.cpp 

OBJS += \
./ephemeris.o \
./main.o \
./observation.o \
./port.o \
./sbp.o \
./sbp_functions.o \
./signal.o \
./solution.o 

CPP_DEPS += \
./ephemeris.d \
./main.d \
./observation.d \
./port.d \
./sbp.d \
./sbp_functions.d \
./signal.d \
./solution.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


