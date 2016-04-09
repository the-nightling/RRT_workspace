################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CU_SRCS += \
../src/RRT.cu \
../src/floyd_warshall_algo.cu \
../src/main.cu 

OBJS += \
./src/RRT.o \
./src/floyd_warshall_algo.o \
./src/main.o 

CU_DEPS += \
./src/RRT.d \
./src/floyd_warshall_algo.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cu
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-7.0/bin/nvcc -I"/usr/local/cuda-7.0/samples/0_Simple" -I"/usr/local/cuda-7.0/samples/common/inc" -I"/home/nightling/RRT_workspace/simple_pendulum/RRT_pend_CUDA/RRT_pend_CUDA" -G -g -O0 -gencode arch=compute_30,code=sm_30  -odir "src" -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-7.0/bin/nvcc -I"/usr/local/cuda-7.0/samples/0_Simple" -I"/usr/local/cuda-7.0/samples/common/inc" -I"/home/nightling/RRT_workspace/simple_pendulum/RRT_pend_CUDA/RRT_pend_CUDA" -G -g -O0 --compile --relocatable-device-code=false -gencode arch=compute_30,code=compute_30 -gencode arch=compute_30,code=sm_30  -x cu -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


