#!/bin/sh

perl -i -0pe 's/\$\(CC\) \$\(OBJECTS\)/\$\(CC\) -Wl,--print-memory-usage \$\(OBJECTS\)/m' Makefile
perl -i -0pe 's/# C sources(.*\n)*?# ASM sources/# C sources\nC_SOURCES =  \\\n\$(wildcard \/Users\/imashio\/Electronics\/ARM\/_Library\/*.c) \\\n\$(wildcard Src\/*.c) \\\n\$(wildcard Drivers\/*.c) \\\n\$(wildcard Drivers\/STM32F3xx_HAL_Driver\/Src\/*.c)\n\n# ASM sources/m' Makefile
perl -i -0pe 's/BINPATH =(.*\n)/BINPATH = \/usr\/local\/gcc-arm-none-eabi-7-2018-q2-update\/bin\n/m' Makefile
perl -i -0pe 's/C_INCLUDES =  \\\n-IInc \\/C_INCLUDES =  \\\n-I\/Users\/imashio\/Electronics\/ARM\/_Library \\\n-IInc \\/m' Makefile
perl -i -0pe 's/#define USE_FULL_ASSERT/\/\/ #define USE_FULL_ASSERT/m' 'Inc/stm32f3xx_hal_conf.h'
