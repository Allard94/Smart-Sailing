################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../rtklib/rcv/binex.c \
../rtklib/rcv/crescent.c \
../rtklib/rcv/gw10.c \
../rtklib/rcv/javad.c \
../rtklib/rcv/novatel.c \
../rtklib/rcv/nvs.c \
../rtklib/rcv/rcvlex.c \
../rtklib/rcv/rt17.c \
../rtklib/rcv/septentrio.c \
../rtklib/rcv/skytraq.c \
../rtklib/rcv/ss2.c \
../rtklib/rcv/ublox.c 

OBJS += \
./rtklib/rcv/binex.o \
./rtklib/rcv/crescent.o \
./rtklib/rcv/gw10.o \
./rtklib/rcv/javad.o \
./rtklib/rcv/novatel.o \
./rtklib/rcv/nvs.o \
./rtklib/rcv/rcvlex.o \
./rtklib/rcv/rt17.o \
./rtklib/rcv/septentrio.o \
./rtklib/rcv/skytraq.o \
./rtklib/rcv/ss2.o \
./rtklib/rcv/ublox.o 

C_DEPS += \
./rtklib/rcv/binex.d \
./rtklib/rcv/crescent.d \
./rtklib/rcv/gw10.d \
./rtklib/rcv/javad.d \
./rtklib/rcv/novatel.d \
./rtklib/rcv/nvs.d \
./rtklib/rcv/rcvlex.d \
./rtklib/rcv/rt17.d \
./rtklib/rcv/septentrio.d \
./rtklib/rcv/skytraq.d \
./rtklib/rcv/ss2.d \
./rtklib/rcv/ublox.d 


# Each subdirectory must supply rules for building sources it contributes
rtklib/rcv/%.o: ../rtklib/rcv/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"/home/scubakay/workspace-ss/Smart-Sailing.git/rtklib" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


