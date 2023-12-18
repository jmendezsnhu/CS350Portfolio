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

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"
#include <ti/drivers/Timer.h>

uint8_t toggle = 0;
uint8_t state = 0;
int8_t count1 = 0;
uint8_t count2 = 0;

void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    switch(state) {
        case 0: //SOS S
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            state = 1;
            break;
        case 1:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            state = 0;
            if (count1 == 5) {
                state = 2;
                count1 = 0;
            }
            break;
        case 2: //Character Pause
            if (count1 == 2) {
                state = 3;
            }
            break;
        case 3: //O
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            state = 4;
            count1 = 0;
            break;
        case 4:
            if (count1 == 2) {
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                state = 3;
                count1 = 0;
                count2++;
                if (count2 == 3) {
                    count2 = 0;
                    state = 5;
                }
            }
            break;
        case 5: //Character Pause
            if (count1 == 2) {
                state = 6;
                count1 = 0;
            }
            break;
        case 6: //S
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            state = 7;
            break;
        case 7:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            state = 6;
            if (count1 == 6) {
                state = 17;
                count1 = 0;
            }
            break;
        case 8: //OK O
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            state = 9;
            count1 = 0;
            break;
        case 9:
            if (count1 == 2) {
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                state = 8;
                count1 = 0;
                count2++;
                if(count2 == 3) {
                    count2 = 0;
                    state = 10;
                }
            }
            break;
        case 10: //Character Pause
            if (count1 == 2) {
                state = 11;
            }
            break;
        case 11: //K
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            state = 12;
            count1 = 0;
            break;
        case 12:
            if (count1 == 2) {
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                state = 13;
            }
            break;
        case 13:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            state = 14;
            break;
        case 14:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            state = 15;
            break;
        case 15:
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
            state = 16;
            count1 = 0;
            break;
        case 16:
            if (count1 == 2) {
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                state = 17;
                count1 = 0;
            }
            break;
        case 17: //Word Pause
            if (count1 == 5){
                state = 18;
            }
            break;
        case 18: //Toggle check and Reset
            count1 = -1;
            if(toggle) {
                state = 8;
            } else {
                state = 0;
            }
            break;
        default:
            state = 0;
            count1 = -1;
            count2 = 0;
            toggle = 0;
            break;
    }

    count1++;
}
void initTimer(void)
{
    Timer_Handle timer0;
    Timer_Params params;
    Timer_init();
    Timer_Params_init(&params);
    params.period = 500000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

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
    if (!toggle) {
        toggle = 1;
    } else {
        toggle = 0;
    }
}


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    toggle = 0;

    initTimer();

    return (NULL);
}
