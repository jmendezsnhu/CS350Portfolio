/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#include <ti/drivers/Timer.h>
volatile unsigned char TimerFlag = 0;

#include <ti/drivers/UART2.h>
UART2_Handle uart;

#include <ti/drivers/I2C.h>
// I2C Global Variables
static const struct {
uint8_t address;
uint8_t resultReg;
char *id;
} sensors[3] = {
{ 0x48, 0x0000, "11X" },
{ 0x49, 0x0000, "116" },
{ 0x41, 0x0001, "006" }
};
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;
// Driver Handles - Global variables
I2C_Handle i2c;

int16_t readTemp(void);

int16_t temp = 0;
int16_t setPoint = 20;
uint8_t heat = 0;
uint8_t Button0Flag = 0;
uint8_t Button1Flag = 0;
uint32_t buttonTime = 200000;
uint32_t heatTime = 500000;
uint32_t displayTime = 1000000;
uint32_t periodTime = 100000;
uint16_t count1 = 0;
char output[64];
int bytesToSend;

void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    //UART2_write(uart, &output, snprintf(output, 64, "<%d>\n\r", count1), NULL);

    count1++;
    TimerFlag = 1;
}

// UART Global Variables
// Driver Handles - Global variables
void initUART(void)
{
UART2_Params uartParams;
// Configure the driver
UART2_Params_init(&uartParams);
uartParams.writeMode = UART2_Mode_POLLING;
uartParams.readMode = UART2_Mode_POLLING;
uartParams.readReturnMode = UART2_ReadReturnMode_FULL;
uartParams.baudRate = 115200;
// Open the driver
uart = UART2_open(CONFIG_UART2_0, &uartParams);
if (uart == NULL) {
/* UART_open() failed */
while (1);
}
}

// Make sure you call initUART() before calling this function.
void initI2C(void)
{

int8_t i, found;
I2C_Params i2cParams;
UART2_write(uart, &output, snprintf(output, 64, "Initializing I2C Driver - "), NULL);
// Init the driver
I2C_init();
// Configure the driver
I2C_Params_init(&i2cParams);
i2cParams.bitRate = I2C_400kHz;
// Open the driver
i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
if (i2c == NULL)
{
UART2_write(uart, &output, snprintf(output, 64, "Failed\n\r"), NULL);
while (1);
}
UART2_write(uart, &output, snprintf(output, 32, "Passed\n\r"), NULL);
// Boards were shipped with different sensors.
// Welcome to the world of embedded systems.
// Try to determine which sensor we have.
// Scan through the possible sensor addresses
/* Common I2C transaction setup */
i2cTransaction.writeBuf = txBuffer;
i2cTransaction.writeCount = 1;
i2cTransaction.readBuf = rxBuffer;
i2cTransaction.readCount = 0;
found = false;
for (i=0; i<3; ++i)
{
i2cTransaction.targetAddress = sensors[i].address;
txBuffer[0] = sensors[i].resultReg;
UART2_write(uart, &output, snprintf(output, 64, "Is this %s? ", sensors[i].id), NULL);
if (I2C_transfer(i2c, &i2cTransaction))
{
UART2_write(uart, &output, snprintf(output, 64, "Found\n\r"), NULL);
found = true;
break;
}
UART2_write(uart, &output, snprintf(output, 64, "No\n\r"), NULL);
}
if(found)
{
UART2_write(uart, &output, snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress), NULL);
}
else
{
UART2_write(uart, &output, snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"), NULL);
}
}

int16_t readTemp(void)
{
int j;
int16_t temperature = 0;
i2cTransaction.readCount = 2;
if (I2C_transfer(i2c, &i2cTransaction))
{
/*
* Extract degrees C from the received data;
* see TMP sensor datasheet
*/
temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
temperature *= 0.0078125;
/*
* If the MSB is set '1', then we have a 2's complement
* negative value which needs to be sign extended
*/
if (rxBuffer[0] & 0x80)
{
temperature |= 0xF000;
}
}
else
{
UART2_write(uart, &output, snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status), NULL);
UART2_write(uart, &output, snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"), NULL);
}
return temperature;
}

void initTimer(void)
{
Timer_Handle timer0;
Timer_Params params;
// Init the driver
Timer_init();
// Configure the driver
Timer_Params_init(&params);
params.period = 100000;
params.periodUnits = Timer_PERIOD_US;
params.timerMode = Timer_CONTINUOUS_CALLBACK;
params.timerCallback = timerCallback;
// Open the driver
timer0 = Timer_open(CONFIG_TIMER_0, &params);
if (timer0 == NULL) {
/* Failed to initialized timer */
while (1) {}
}
if (Timer_start(timer0) == Timer_STATUS_ERROR) {
/* Failed to start timer */
while (1) {}
}
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    Button0Flag = 1;
}

void gpioButtonFxn1(uint_least8_t index)
{
    Button1Flag = 1;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();
    initUART();
    initI2C();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);

    initTimer();

    while(1) {
        while(!TimerFlag){}
        if (buttonTime >= 200000) {
            if(Button0Flag == 1) {
                setPoint++;
                Button0Flag = 0;
            }

            if(Button1Flag == 1) {
                setPoint--;
                Button1Flag = 0;
            }
            buttonTime = 0;
        }

        if (heatTime >= 500000){
            temp = readTemp();
            if(temp < setPoint){
                heat = 1;
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            } else {
                heat = 0;
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            }
            heatTime = 0;
        }

        if (displayTime >= 1000000) {
            UART2_write(uart, &output, snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temp, setPoint, heat, count1/10), NULL);
            displayTime = 0;
        }

        buttonTime += periodTime;
        heatTime += periodTime;
        displayTime += periodTime;

        TimerFlag = 0;
    }

    return (NULL);
}
