################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../rtklib/convkml.c \
../rtklib/convrnx.c \
../rtklib/datum.c \
../rtklib/download.c \
../rtklib/ephemeris.c \
../rtklib/geoid.c \
../rtklib/ionex.c \
../rtklib/lambda.c \
../rtklib/options.c \
../rtklib/pntpos.c \
../rtklib/postpos.c \
../rtklib/ppp.c \
../rtklib/ppp_ar.c \
../rtklib/preceph.c \
../rtklib/qzslex.c \
../rtklib/rcvraw.c \
../rtklib/rinex.c \
../rtklib/rtcm.c \
../rtklib/rtcm2.c \
../rtklib/rtcm3.c \
../rtklib/rtcm3e.c \
../rtklib/rtkcmn.c \
../rtklib/rtkpos.c \
../rtklib/rtksvr.c \
../rtklib/sbas.c \
../rtklib/solution.c \
../rtklib/stream.c \
../rtklib/streamsvr.c \
../rtklib/tle.c 

OBJS += \
./rtklib/convkml.o \
./rtklib/convrnx.o \
./rtklib/datum.o \
./rtklib/download.o \
./rtklib/ephemeris.o \
./rtklib/geoid.o \
./rtklib/ionex.o \
./rtklib/lambda.o \
./rtklib/options.o \
./rtklib/pntpos.o \
./rtklib/postpos.o \
./rtklib/ppp.o \
./rtklib/ppp_ar.o \
./rtklib/preceph.o \
./rtklib/qzslex.o \
./rtklib/rcvraw.o \
./rtklib/rinex.o \
./rtklib/rtcm.o \
./rtklib/rtcm2.o \
./rtklib/rtcm3.o \
./rtklib/rtcm3e.o \
./rtklib/rtkcmn.o \
./rtklib/rtkpos.o \
./rtklib/rtksvr.o \
./rtklib/sbas.o \
./rtklib/solution.o \
./rtklib/stream.o \
./rtklib/streamsvr.o \
./rtklib/tle.o 

C_DEPS += \
./rtklib/convkml.d \
./rtklib/convrnx.d \
./rtklib/datum.d \
./rtklib/download.d \
./rtklib/ephemeris.d \
./rtklib/geoid.d \
./rtklib/ionex.d \
./rtklib/lambda.d \
./rtklib/options.d \
./rtklib/pntpos.d \
./rtklib/postpos.d \
./rtklib/ppp.d \
./rtklib/ppp_ar.d \
./rtklib/preceph.d \
./rtklib/qzslex.d \
./rtklib/rcvraw.d \
./rtklib/rinex.d \
./rtklib/rtcm.d \
./rtklib/rtcm2.d \
./rtklib/rtcm3.d \
./rtklib/rtcm3e.d \
./rtklib/rtkcmn.d \
./rtklib/rtkpos.d \
./rtklib/rtksvr.d \
./rtklib/sbas.d \
./rtklib/solution.d \
./rtklib/stream.d \
./rtklib/streamsvr.d \
./rtklib/tle.d 


# Each subdirectory must supply rules for building sources it contributes
rtklib/%.o: ../rtklib/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"/home/allard/Documents/Piksi/rtklib" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


