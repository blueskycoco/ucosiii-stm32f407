#******************************************************************************
#
# Makefile - Rules for building the uart_echo example.
#
# Copyright (c) 2013-2014 Texas Instruments Incorporated.  All rights reserved.
# Software License Agreement
# 
# Texas Instruments (TI) is supplying this software for use solely and
# exclusively on TI's microcontroller products. The software is owned by
# TI and/or its suppliers, and is protected under applicable copyright
# laws. You may not combine this software with "viral" open-source
# software in order to form a larger program.
# 
# THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
# NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
# NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
# CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES, FOR ANY REASON WHATSOEVER.
# 
# This is part of revision 2.1.0.12573 of the EK-TM4C1294XL Firmware Package.
#
#******************************************************************************

#
# The base directory for TivaWare.
#
ROOT=.

#
# Include the common make definitions.
#
include ${ROOT}/makedefs

#
# Where to find header files that do not live in the source directory.
#
IPATH=./Micrium/Software/uC-CPU/ARM-Cortex-M4/GNU
IPATH+=./Micrium/Examples/ST/STM3240G-EVAL/BSP/TrueSTUDIO
IPATH+=./Micrium/Software/uC-LIB/Ports/ARM-Cortex-M4/GNU
IPATH+=./Micrium/Software/uCOS-III/Ports/ARM-Cortex-M4/Generic/GNU
IPATH+=./Micrium/Examples/ST/STM3240G-EVAL/OS3
IPATH+=./Micrium/Examples/ST/STM3240G-EVAL/BSP
IPATH+=./Micrium/Software/uC-CPU
IPATH+=./Micrium/Software/uC-LIB
IPATH+=./Micrium/Examples/ST/STM3240G-EVAL/BSP/CMSIS
IPATH+=./Micrium/Software/uC-Serial/Line
IPATH+=./Micrium/Software/uC-Serial/OS
IPATH+=./Micrium/Software/uC-Serial/Source
IPATH+=./Micrium/Software/uC-Serial/Driver/ST
IPATH+=./Micrium/Software/uCOS-III/Source
IPATH+=./Micrium/Examples/ST/STM3240G-EVAL/BSP/OS/uCOS-III
IPATH+=./Micrium/Examples/ST/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Inc
IPATH+=./Micrium/Examples/ST/STM32CubeF4/Drivers/CMSIS/Include
IPATH+=./Micrium/Examples/ST/STM32CubeF4/Drivers/CMSIS/Device/ST/STM32F4xx/Include


VPATH=./Micrium/Software/uC-CPU/ARM-Cortex-M4/GNU
VPATH+=./Micrium/Examples/ST/STM3240G-EVAL/BSP/TrueSTUDIO
VPATH+=./Micrium/Software/uC-LIB/Ports/ARM-Cortex-M4/GNU
VPATH+=./Micrium/Software/uCOS-III/Ports/ARM-Cortex-M4/Generic/GNU
VPATH+=./Micrium/Examples/ST/STM3240G-EVAL/OS3
VPATH+=./Micrium/Examples/ST/STM3240G-EVAL/BSP
VPATH+=./Micrium/Software/uC-CPU
VPATH+=./Micrium/Software/uC-LIB
VPATH+=./Micrium/Examples/ST/STM3240G-EVAL/BSP/CMSIS
VPATH+=./Micrium/Software/uC-Serial/Line
VPATH+=./Micrium/Software/uC-Serial/OS/uCOS-III
VPATH+=./Micrium/Software/uC-Serial/Source
VPATH+=./Micrium/Software/uC-Serial/Driver/ST
VPATH+=./Micrium/Software/uCOS-III/Source
VPATH+=./Micrium/Examples/ST/STM3240G-EVAL/BSP/OS/uCOS-III
VPATH+=./Micrium/Examples/ST/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src
VPATH+=./Micrium/Examples/ST/STM3240G-EVAL/OS3/TrueSTUDIO
#
# The default rule, which causes the uart_echo example to be built.
#
all: ${COMPILER}
all: ${COMPILER}/uCosIII.axf

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~}
install:
	openocd -f openocd.cfg -c "flash_image"
#	jlink.exe burn.txt

#
# The rule to create the target directory.
#
${COMPILER}:
	@mkdir -p ${COMPILER}

#
# Rules for building the uart_echo example.
#
${COMPILER}/uCosIII.axf: ${COMPILER}/app.o
${COMPILER}/uCosIII.axf: ${COMPILER}/app_serial.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_app_hooks.o
${COMPILER}/uCosIII.axf: ${COMPILER}/bsp.o
${COMPILER}/uCosIII.axf: ${COMPILER}/bsp_int.o
${COMPILER}/uCosIII.axf: ${COMPILER}/bsp_periph.o
${COMPILER}/uCosIII.axf: ${COMPILER}/cpu_bsp.o
${COMPILER}/uCosIII.axf: ${COMPILER}/serial_bsp_stm3240x.o
${COMPILER}/uCosIII.axf: ${COMPILER}/cpu_core.o
${COMPILER}/uCosIII.axf: ${COMPILER}/lib_ascii.o
${COMPILER}/uCosIII.axf: ${COMPILER}/lib_math.o
${COMPILER}/uCosIII.axf: ${COMPILER}/lib_mem.o
${COMPILER}/uCosIII.axf: ${COMPILER}/lib_str.o
${COMPILER}/uCosIII.axf: ${COMPILER}/system_stm32f4xx.o
${COMPILER}/uCosIII.axf: ${COMPILER}/serial_line_dflt.o
${COMPILER}/uCosIII.axf: ${COMPILER}/serial_line_probe.o
${COMPILER}/uCosIII.axf: ${COMPILER}/serial_line_tty.o
${COMPILER}/uCosIII.axf: ${COMPILER}/serial.o
${COMPILER}/uCosIII.axf: ${COMPILER}/serial_buf.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_cfg_app.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_core.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_dbg.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_flag.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_int.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_mem.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_msg.o
${COMPILER}/uCosIII.axf: ${COMPILER}/bsp_os.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_mutex.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_pend_multi.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_q.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_prio.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_sem.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_stat.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_task.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_tick.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_time.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_tmr.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_var.o
${COMPILER}/uCosIII.axf: ${COMPILER}/cpu_c.o
${COMPILER}/uCosIII.axf: ${COMPILER}/serial_drv_stm32.o
${COMPILER}/uCosIII.axf: ${COMPILER}/serial_os.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_adc.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_adc_ex.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_can.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_cec.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_cortex.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_crc.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_cryp.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_cryp_ex.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_dac.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_dac_ex.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_dcmi.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_dcmi_ex.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_dma.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_dma2d.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_dma_ex.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_eth.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_flash.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_flash_ex.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_flash_ramfunc.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_fmpi2c.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_fmpi2c_ex.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_gpio.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_hash.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_hash_ex.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_hcd.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_i2c.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_i2c_ex.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_i2s.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_i2s_ex.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_irda.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_iwdg.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_ltdc.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_msp_template.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_nand.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_nor.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_pccard.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_pcd.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_pcd_ex.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_pwr.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_pwr_ex.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_qspi.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_rcc.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_rcc_ex.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_rng.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_rtc.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_rtc_ex.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_sai.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_sai_ex.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_sd.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_sdram.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_smartcard.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_spdifrx.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_spi.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_sram.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_tim.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_tim_ex.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_uart.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_usart.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_hal_wwdg.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_ll_fmc.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_ll_fsmc.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_ll_sdmmc.o
${COMPILER}/uCosIII.axf: ${COMPILER}/stm32f4xx_ll_usb.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_cpu_c.o
${COMPILER}/uCosIII.axf: ${COMPILER}/cpu_a.o
${COMPILER}/uCosIII.axf: ${COMPILER}/startup.o
${COMPILER}/uCosIII.axf: ${COMPILER}/lib_mem_a.o
${COMPILER}/uCosIII.axf: ${COMPILER}/os_cpu_a.o
${COMPILER}/uCosIII.axf: ${COMPILER}/_sbrk.o
${COMPILER}/uCosIII.axf: stm32f4_flash.ld
SCATTERgcc_uCosIII=stm32f4_flash.ld
ENTRY_uCosIII=Reset_Handler
CFLAGSgcc=-DSTM32F407xx -DUSE_HAL_DRIVER

#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif
