################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../rcv/binex.c \
../rcv/crescent.c \
../rcv/gw10.c \
../rcv/javad.c \
../rcv/novatel.c \
../rcv/nvs.c \
../rcv/rcvlex.c \
../rcv/rt17.c \
../rcv/septentrio.c \
../rcv/skytraq.c \
../rcv/ss2.c \
../rcv/ublox.c 

OBJS += \
./rcv/binex.o \
./rcv/crescent.o \
./rcv/gw10.o \
./rcv/javad.o \
./rcv/novatel.o \
./rcv/nvs.o \
./rcv/rcvlex.o \
./rcv/rt17.o \
./rcv/septentrio.o \
./rcv/skytraq.o \
./rcv/ss2.o \
./rcv/ublox.o 

C_DEPS += \
./rcv/binex.d \
./rcv/crescent.d \
./rcv/gw10.d \
./rcv/javad.d \
./rcv/novatel.d \
./rcv/nvs.d \
./rcv/rcvlex.d \
./rcv/rt17.d \
./rcv/septentrio.d \
./rcv/skytraq.d \
./rcv/ss2.d \
./rcv/ublox.d 


# Each subdirectory must supply rules for building sources it contributes
rcv/%.o: ../rcv/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"/home/allard/Documents/Piksi" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


