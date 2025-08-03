/******************************************************************************
* Copyright (C) 2023 Advanced Micro Devices, Inc. All Rights Reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/
/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include "platform.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "stdbool.h"

#include "xgpio.h"
#include "xuartlite.h"
#include "xspi.h"
#include <unistd.h>

#include "adxl362.h"
#include "mpu6500.h"

#include "xiltimer.h"
#include "xtmrctr.h"

#include <stdlib.h> // abs

#include "xintc.h"
#include "xiic.h"
#include "xil_exception.h"

#define USE_HCSR04	    1
#define USE_ADT7420	    1
#define USE_ADXL362	    1
#define USE_MPU6500     0

/*
	NOTES:

	Microblaze spi io0_o is MOSI, io1_i MISO (do not select io0_i)

	Add files placed in src folder to Sources of UserConfig.cmake

	Input gpio must be pulled up or down in constraint file definition

    I2C 

*/

#define LED 						0xFFFF
#define LED_DELAY     				1000000
#define LED_CHANNEL 				1

#define TRIG_CHANNEL              	1

#define ECHO_CHANNEL              	1

XIntc IntcInstance;
XIic IicInstance;

XTmrCtr TimerInstance;

XSpi Spi0Instance;
XSpi Spi1Instance;

XGpio Gpio0; /* The Instance of the GPIO Driver */

XGpio Gpio1;

XGpio Gpio2;
XIntc IntcInstance;   // Interrupt controller instance

XUartLite UartLite;		/* Instance of the UartLite Device */

XIic IicInstance0;
XIic IicInstance1;


uint8_t uart_rx_buffer[512];
uint8_t uart_tx_buffer[512];

uint32_t gpio_state = 0x00000000;  // Tracks current output state

int counter = 0;
int reg_id_counter = 0;

uint8_t send_buf[3] = {0x0B, 0x02, 0xFF};  // Read command, DEVID_AD, dummy
uint8_t recv_buf[4];

int16_t x_val, y_val, z_val;
uint8_t spi_rx_buffer[32];

int16_t accel[3], gyro[3];
float gyro_dps[3];
uint8_t dev_id;

volatile uint64_t echo_start = 0;
volatile uint64_t echo_end = 0;
volatile int echo_done = 0;
volatile uint32_t last_echo_state = 0;


void toggle_led(int led_num) {
    uint32_t bit_mask = 1u << (led_num - 1);

    // Toggle the bit
    gpio_state ^= bit_mask;

    // Write updated state
    XGpio_DiscreteWrite(&Gpio0, LED_CHANNEL, gpio_state);
}

void blink_led(void){

	for(int j = 0; j < 10; j++){
        
		XGpio_DiscreteWrite(&Gpio0, LED_CHANNEL, LED);

		usleep(10000);
		
		XGpio_DiscreteWrite(&Gpio0, LED_CHANNEL, ~LED);
        
		usleep(10000);
	}
}

void trigger_hcsr04() {
    
    // Set TRIG high
    XGpio_DiscreteWrite(&Gpio1, TRIG_CHANNEL, 1); // bit 1 of 2 gpio output pins is set
    
    usleep(10);  // 10 microseconds
    
    // Set TRIG low
    XGpio_DiscreteWrite(&Gpio1, TRIG_CHANNEL, 0); // both bits are reseted
}

uint32_t measure_distance_us_interrupt() {
    if (echo_done) {
        uint64_t delta = echo_end - echo_start;
        echo_done = 0;  // reset for next measurement
        return (uint32_t)(delta / 100);  // convert counts to microseconds at 100 MHz
    }
    return 0;  // no valid measurement yet
}

uint32_t measure_distance_us() {

    // Wait for echo HIGH
    while (XGpio_DiscreteRead(&Gpio2, ECHO_CHANNEL) == 0); // read first bit of 2 gpio input pins

    u64 start = XTmrCtr_GetValue(&TimerInstance, 0);

    // Wait while high
    while (XGpio_DiscreteRead(&Gpio2, ECHO_CHANNEL) == 1);

    u64 delta = XTmrCtr_GetValue(&TimerInstance, 0) - start;
    
    return delta / 100; // timer counter count 100 in 1 sec since its clock is 100mhz
}

int i2c_read_register(XIic *iic, u8 slave7, u8 reg, u8 *out) {
    int byte_num;

    // Send register address with repeated start (no STOP so bus stays owned)
    byte_num = XIic_Send(iic->BaseAddress, slave7 , &reg, 1, XIIC_REPEATED_START);
    if (byte_num == 0) {
        xil_printf("byte_num: %d\r\n",byte_num);
        return byte_num;
    }

    // Now read one byte with STOP to end transaction
    byte_num = XIic_Recv(iic->BaseAddress, slave7 , out, 1, XIIC_STOP);
    if (byte_num == 0) {
        xil_printf("byte_num: %d\r\n",byte_num);
        return byte_num;
    }

    return byte_num;
}

int i2c_read_temperature(XIic *iic, u8 slave7, u8 reg, int16_t *temp_out) {
    int byte_num;
    u8 buf[2];

    // Send register address with repeated start (no STOP so bus stays owned)
    byte_num = XIic_Send(iic->BaseAddress, slave7, &reg, 1, XIIC_REPEATED_START);
    if (byte_num == 0) {
        xil_printf("Send failed, byte_num: %d\r\n", byte_num);
        return 0;
    }

    // Now read 2 bytes (MSB + LSB) with STOP to end transaction
    byte_num = XIic_Recv(iic->BaseAddress, slave7, buf, 2, XIIC_STOP);
    if (byte_num != 2) {
        xil_printf("Recv failed or incomplete, byte_num: %d\r\n", byte_num);
        return 0;
    }

    // Combine MSB and LSB into int16_t
    *temp_out = (int16_t)((buf[0] << 8) | buf[1]);

    return byte_num;
}

float adt7420_convert_temp(int16_t raw_temp) {
    return (float)raw_temp / 128.0f;
}

void GpioEchoHandler(void *CallbackRef) {
    XGpio *GpioPtr = (XGpio *)CallbackRef;

    // Clear the interrupt first
    XGpio_InterruptClear(GpioPtr, 1);

    uint32_t curr = XGpio_DiscreteRead(GpioPtr, ECHO_CHANNEL) & 0x1; // bit0
    uint64_t now = XTmrCtr_GetValue(&TimerInstance, 0);

    if (curr && !last_echo_state) {
        // Rising edge
        echo_start = now;
        echo_done = 0;
    } 
    else if (!curr && last_echo_state) {
        // Falling edge
        echo_end = now;
        echo_done = 1;
    }

    last_echo_state = curr;
}

int main(void)
{
	int Status;

    xil_printf("CODE STARTED\r\n");



	// Initialize timer
    Status = XTmrCtr_Initialize(&TimerInstance, XPAR_AXI_TIMER_0_BASEADDR);
    if (Status != XST_SUCCESS) {
        printf("Timer init failed\r\n");
        return -1;
    }

    // Ensure cascade mode is set in hardware; don't use auto-reload for measuring
    XTmrCtr_SetOptions(&TimerInstance, 0, XTC_CASCADE_MODE_OPTION); // so it becomes 64 bit with 2 timers. 32 bit overflows after 45 sec
    XTmrCtr_SetOptions(&TimerInstance, 1, 0);

    // Reset both halves 
	XTmrCtr_Reset(&TimerInstance, 0);  // Reset Timer 0 (low 32 bits)
	XTmrCtr_Reset(&TimerInstance, 1);  // Reset Timer 1 (high 32 bits in cascade mode)

    // Start the timer (starting counter 0 will also increment the cascaded 1)
    XTmrCtr_Start(&TimerInstance, 0);




    // Initialize IIC driver
    Status = XIic_Initialize(&IicInstance0, XPAR_XIIC_0_BASEADDR);
    if (Status != XST_SUCCESS) {
        xil_printf("IIC Initialization Failed\r\n");
        return XST_FAILURE;
    }

    // Start IIC device
    Status = XIic_Start(&IicInstance0);
    if (Status != XST_SUCCESS) {
        xil_printf("IIC Start failed\r\n");
        return XST_FAILURE;
    }
	



    // Initialize IIC driver
    Status = XIic_Initialize(&IicInstance1, XPAR_XIIC_1_BASEADDR);
    if (Status != XST_SUCCESS) {
        xil_printf("IIC Initialization Failed\r\n");
        return XST_FAILURE;
    }

    // Start IIC device
    Status = XIic_Start(&IicInstance1);
    if (Status != XST_SUCCESS) {
        xil_printf("IIC Start failed\r\n");
        return XST_FAILURE;
    }
	





	// GPIO0 Init
	Status = XGpio_Initialize(&Gpio0, XPAR_XGPIO_0_BASEADDR);
	if (Status != XST_SUCCESS) {
		xil_printf("GPIO0 Initialization Failed\r\n");
		return XST_FAILURE;
	}

	XGpio_SetDataDirection(&Gpio0, LED_CHANNEL, 0x0000); // 0 is output set all output

	blink_led();




	// GPIO1 Init
	Status = XGpio_Initialize(&Gpio1, XPAR_XGPIO_1_BASEADDR);
	if (Status != XST_SUCCESS) {
		xil_printf("GPIO1 Initialization Failed\r\n");
		return XST_FAILURE;
	}

	XGpio_SetDataDirection(&Gpio1, TRIG_CHANNEL, 0x0); // both pins are output
	




	// GPIO2 Init
	Status = XGpio_Initialize(&Gpio2, XPAR_XGPIO_2_BASEADDR);
	if (Status != XST_SUCCESS) {
		xil_printf("GPIO2 Initialization Failed\r\n");
		return XST_FAILURE;
	}

    Status = XIntc_Initialize(&IntcInstance, XPAR_XINTC_0_BASEADDR);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

   	XGpio_SetDataDirection(&Gpio2, ECHO_CHANNEL, 0x3); // both pins are input

    // Connect the GPIO interrupt handler to the interrupt controller
    Status = XIntc_Connect(&IntcInstance, XPAR_FABRIC_AXI_GPIO_2_INTR, (XInterruptHandler)GpioEchoHandler, &Gpio2);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // Start the interrupt controller in real mode
    Status = XIntc_Start(&IntcInstance, XIN_REAL_MODE);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // Enable the GPIO interrupt in the interrupt controller
    XIntc_Enable(&IntcInstance, XPAR_FABRIC_AXI_GPIO_2_INTR);

    // Setup the exception handling for interrupts
    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler)XIntc_InterruptHandler, &IntcInstance);
    Xil_ExceptionEnable();

    // After GPIO2 init and direction set:
    last_echo_state = XGpio_DiscreteRead(&Gpio2, ECHO_CHANNEL) & 0x1;

    XGpio_InterruptEnable(&Gpio2, 0x1);  // Enable interrupt on first input pin (assuming echo on pin 0)
    XGpio_InterruptGlobalEnable(&Gpio2);







	// UART0 Init
	Status = XUartLite_Initialize(&UartLite, 0);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// Perform a self-test to ensure that the hardware was built correctly.
	Status = XUartLite_SelfTest(&UartLite);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}




    // SPI0 Init
    Status = XSpi_Initialize(&Spi0Instance, XPAR_AXI_QUAD_SPI_0_BASEADDR);
    if (Status != XST_SUCCESS) {
        xil_printf("SPI Initialization Failed\r\n");
        return XST_FAILURE;
    }

    // Set options: master mode and manual slave select
    Status = XSpi_SetOptions(&Spi0Instance, XSP_MASTER_OPTION | XSP_MANUAL_SSELECT_OPTION);
    if (Status != XST_SUCCESS) {
        xil_printf("SPI SetOptions Failed\r\n");
        return XST_FAILURE;
    }

    XSpi_Start(&Spi0Instance);// Start the SPI driver
    XSpi_IntrGlobalDisable(&Spi0Instance); // Disable global interrupt mode




    // SPI1 Init
    Status = XSpi_Initialize(&Spi1Instance, XPAR_AXI_QUAD_SPI_1_BASEADDR);
    if (Status != XST_SUCCESS) {
        xil_printf("SPI Initialization Failed\r\n");
        return XST_FAILURE;
    }

    // Set options: master mode and manual slave select
    Status = XSpi_SetOptions(&Spi1Instance, XSP_MASTER_OPTION | XSP_MANUAL_SSELECT_OPTION | XSP_CLK_PHASE_1_OPTION | XSP_CLK_ACTIVE_LOW_OPTION);
    if (Status != XST_SUCCESS) {
        xil_printf("SPI SetOptions Failed\r\n");
        return XST_FAILURE;
    }

    XSpi_Start(&Spi1Instance);// Start the SPI driver
    XSpi_IntrGlobalDisable(&Spi1Instance); // Disable global interrupt mode




	ADXL362_SoftReset(&Spi0Instance);
	ADXL362_Init(&Spi0Instance);

    dev_id = ADXL362_ReadDeviceID(&Spi0Instance);
	xil_printf("ADXL362 dev_id: %d\r\n", dev_id);

    // Initialize MPU6500
    dev_id = MPU6500_Init(&Spi1Instance);
	xil_printf("MPU6500 dev_id: %d\r\n", dev_id);


    u8 bytes_recv = 0;

	bytes_recv = i2c_read_register(&IicInstance0, 0x4B, 0x0B, &dev_id);
    if (bytes_recv) {
	    xil_printf("ADT7420 dev_id: %d\r\n", dev_id);
    } 
    else {
        xil_printf("I2C read failed\r\n");
    }

    u8 cfg;
    i2c_read_register(&IicInstance0, 0x48, 0x03, &cfg);
    if (cfg & 0x80) {
        xil_printf("16-bit mode enabled\r\n");
    } else {
        xil_printf("13-bit mode enabled\r\n");
    }


    bytes_recv = 0;
    bytes_recv = i2c_read_register(&IicInstance1, 0x68, 0x75, &dev_id);
    if (bytes_recv) {
	    xil_printf("MPU6050 dev_id: %d\r\n", dev_id);
    } 
    else {
        xil_printf("I2C read failed\r\n");
    }

	while (1) {
		
        u64 start = XTmrCtr_GetValue(&TimerInstance, 0);

		//uart_tx_buffer[0] = counter++;
		//uart_tx_buffer[0] = z_val & 0xFF;

		//XUartLite_Send(&UartLite, uart_tx_buffer, 1);

		//while(XUartLite_Recv(&UartLite, uart_rx_buffer, 1) == 0);

		if(counter > 255){
			counter = 0;
		}

        #if USE_ADXL362
            ADXL362_ReadXYZ(&Spi0Instance, &x_val, &y_val, &z_val, spi_rx_buffer);
            xil_printf("X: %d, Y: %d, Z: %d\r\n", x_val, y_val, z_val);
        #endif

        #if USE_MPU6500
            MPU6500_ReadAccel(&Spi1Instance, accel);
            MPU6500_ReadGyro(&Spi1Instance, gyro);
            MPU6500_ConvertGyroToDPS(gyro, gyro_dps);

            xil_printf("ACC: X=%d Y=%d Z=%d\r\n", accel[0], accel[1], accel[2]);
            xil_printf("GYRO: X=%f Y=%f Z=%f\r\n", gyro_dps[0], gyro_dps[1], gyro_dps[2]);
            /*xil_printf("GYRO: X=%d.%02d Y=%d.%02d Z=%d.%02d\r\n",
                (int)gyro_dps[0], abs((int)(gyro_dps[0] * 100) % 100),
                (int)gyro_dps[1], abs((int)(gyro_dps[1] * 100) % 100),
                (int)gyro_dps[2], abs((int)(gyro_dps[2] * 100) % 100));
            */

        #endif

        XGpio_DiscreteWrite(&Gpio0, LED_CHANNEL, gyro[0]);

		#if USE_HCSR04
			trigger_hcsr04();
			usleep(100); // allow echo to rise if needed

			//uint32_t duration_us = measure_distance_us();
            uint32_t duration_us = measure_distance_us_interrupt();

            float distance = (duration_us * 0.0343) / 2; // cm
            
            xil_printf("Duration: %d\r\n", duration_us);

			printf("Distance: %.3f\r\n", distance);

			usleep(100); // allow echo to rise if needed

		#endif

        #if USE_ADT7420
            int16_t temperature_raw;
            int result = i2c_read_temperature(&IicInstance0, 0x4B, 0x00, &temperature_raw);
            if (result == 2) {

                xil_printf("Raw temperature = %d\r\n", temperature_raw);

                temperature_raw >>= 3;
                float temp_c = (float)temperature_raw * 0.0625f;
                printf("Temperature: %.3f °C\r\n", temp_c);

            } else {
                xil_printf("Temperature read failed\r\n");
            }
        #endif
		
        

        toggle_led(1);
		toggle_led(2);
		toggle_led(3);
		toggle_led(5);
		toggle_led(10);
		toggle_led(16);


        u64 delta = XTmrCtr_GetValue(&TimerInstance, 0) - start;
        xil_printf("Timer: %d \r\n", delta);

		

		usleep(50000);
	}

}

