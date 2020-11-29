# EmbeddedDriversDevptSTM32F446xx

Development drivers for STM32F446xx as practice and study of the ARM based STM32F446 Embedded Micro Controller
Done while taking Udemy course to brush up my skills.
In this project I will develop drivers for STM64F446re (Arm corext 4 based MCU, by ST Microelectronics).
The drivers will be developed from scratch, without using any of the supplied drivers by ST Microelectronics.
Some of the drivers that are planned to be developed as excercise:
1. GPIOs
	Input/Output port set/clear/read.
	Support for various ports configurations e.g. speed, otput type, pull-up/pull down, etc.
	Interrupts support - (Also with interrupts support initiated from GPIO ports, including the EXTI (External Interrupts) peripheral and the Cortex's NVIC (Nested Vector Interrupts Cntroller)).
2. SPI,

And if time will allow, also

3. I2C
4. UART / USART
 	


Tested on STM32 Nucleo-64 development boart of ST Microelectronics - https://www.st.com/en/evaluation-tools/nucleo-f446re.html with STM64F446re MCU.



Online resources used for the development of the drivers:

Development Board - Nucleo-446re
=================================
https://www.st.com/en/evaluation-tools/nucleo-f446re.html#resource

Nucleo-446re Data Brief https://www.st.com/resource/en/data_brief/nucleo-f446re.pdf

STM32F446RE resources
======================

https://www.st.com/en/microcontrollers-microprocessors/stm32f446.html#resource

STM32F446RE Reference Manual - https://www.st.com/resource/en/reference_manual/dm00135183-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf

STM32F446RE Data Sheet - https://www.st.com/resource/en/datasheet/stm32f446re.pdf

STM32F446RE Programmer Manual - https://www.st.com/resource/en/programming_manual/dm00046982-stm32-cortexm4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf

ARM Cortex M4 devices Generic User Guide
=========================================
https://developer.arm.com/documentation/dui0553/b/

https://documentation-service.arm.com/static/5f2ac76d60a93e65927bbdc5?token=

https://static.docs.arm.com/dui0553/a/DUI0553A_cortex_m4_dgug.pdf

ARM Cortex M4 Technical Reference Manual -  https://static.docs.arm.com/100166/0001/arm_cortexm4_processor_trm_100166_0001_00_en.pdf

