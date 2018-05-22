/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
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
 *  ======== rfEasyLinkRxTx.c ========
 */
/* XDCtools Header files */
#include <stdio.h>
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>

/* Board Header files */
#include "Board.h"
#include "uart_term.h"

/* EasyLink API Header files */
#include "easylink/EasyLink.h"

/***** Defines *****/

/* Undefine to remove address filter and async mode */
#define RFEASYLINKRX_ASYNC
#define RFEASYLINKRX_ADDR_FILTER

#define RFEASYLINKTX_ASYNC

#define RFEASYLINKEX_TASK_STACK_SIZE 1024
#define RFEASYLINKEX_TASK_PRIORITY   2

#define RFEASYLINKTX_BURST_SIZE         10
#define RFEASYLINKTXPAYLOAD_LENGTH      30

/* Pin driver handle */
static PIN_State ledPinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] = {
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/***** Variable declarations *****/
static Task_Params rxTaskParams;
Task_Struct rxTask;    /* not static so you can see in ROV */

static Task_Params txTaskParams;
Task_Struct txTask;    /* not static so you can see in ROV */

static uint8_t rxTaskStack[RFEASYLINKEX_TASK_STACK_SIZE];
static uint8_t txTaskStack[RFEASYLINKEX_TASK_STACK_SIZE];

/* The RX Output struct contains statistics about the RX operation of the radio */
PIN_Handle pinHandle;

#ifdef RFEASYLINKRX_ASYNC
static Semaphore_Handle rxDoneSem;
#endif

/***** Function definitions *****/
#ifdef RFEASYLINKRX_ASYNC
void rxDoneCb(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
    if (status == EasyLink_Status_Success)
    {
        UART_PRINT("%s",rxPacket->payload);

        /* Toggle LED2 to indicate RX */
        PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
        PIN_setOutputValue(pinHandle, Board_PIN_LED2,0);
    }
    else if(status == EasyLink_Status_Aborted)
    {
        /* Toggle LED1 to indicate command aborted */
       // PIN_setOutputValue(pinHandle, Board_PIN_LED1,0);
       // PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
    }
    else
    {
        /* Toggle LED1 and LED2 to indicate error */
        PIN_setOutputValue(pinHandle, Board_PIN_LED1,1);
        PIN_setOutputValue(pinHandle, Board_PIN_LED2,1);
    }

    Semaphore_post(rxDoneSem);
}
#endif

#ifdef RFEASYLINKTX_ASYNC
static Semaphore_Handle txDoneSem;
#endif //RFEASYLINKTX_ASYNC

#ifdef RFEASYLINKTX_ASYNC
void txDoneCb(EasyLink_Status status)
{
    if (status == EasyLink_Status_Success)
    {
        /* Toggle LED2 to indicate TX */
        PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
        PIN_setOutputValue(pinHandle, Board_PIN_LED1,0);
    }
    else if(status == EasyLink_Status_Aborted)
    {
        /* Toggle LED2 to indicate command aborted */
        //PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
    }
    else
    {
        /* Toggle LED1 and LED2 to indicate error */
        PIN_setOutputValue(pinHandle, Board_PIN_LED1,1);
        PIN_setOutputValue(pinHandle, Board_PIN_LED2,1);
    }

    Semaphore_post(txDoneSem);
}
#endif //RFEASYLINKTX_ASYNC


static void rfEasyLinkRxFnx(UArg arg0, UArg arg1)
{
#ifndef RFEASYLINKRX_ASYNC
    EasyLink_RxPacket rxPacket = {0};
#else
    /* Create a semaphore for Async */
    Semaphore_Params params;
    Error_Block eb;

    /* Init params */
    Semaphore_Params_init(&params);
    Error_init(&eb);

    /* Create semaphore instance */
    rxDoneSem = Semaphore_create(0, &params, &eb);
    if(rxDoneSem == NULL)
    {
        System_abort("rxDoneSem Semaphore creation failed");
    }

#endif //RX_ASYNC

    EasyLink_Params easyLink_params;
    EasyLink_Params_init(&easyLink_params);

    easyLink_params.ui32ModType = EasyLink_Phy_Custom;

    /* Initialize EasyLink */
    if(EasyLink_init(&easyLink_params) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_init failed");
    }

    /*
     * If you wish to use a frequency other than the default, use
     * the following API:
     * EasyLink_setFrequency(868000000);
     */

    /* Set output power to 12dBm */
    EasyLink_Status pwrStatus = EasyLink_setRfPower(12);

    if(pwrStatus != EasyLink_Status_Success)
    {
        // There was a problem setting the transmission power
        while(1);
    }
    /*
     * If you wish to use a frequency other than the default, use
     * the following API:
     * EasyLink_setFrequency(868000000);
     */

#ifdef RFEASYLINKRX_ADDR_FILTER
	/* 
     * The address filter is set to match on a single byte (0xAA) but 
     * EasyLink_enableRxAddrFilter will copy 
     * EASYLINK_MAX_ADDR_SIZE * EASYLINK_MAX_ADDR_FILTERS
     * bytes to the address filter bank
     */
    uint8_t addrFilter[EASYLINK_MAX_ADDR_SIZE * EASYLINK_MAX_ADDR_FILTERS] = {0xaa};
    EasyLink_enableRxAddrFilter(addrFilter, 1, 1);
#endif //RFEASYLINKRX_ADDR_FILTER

    while(1) {
#ifdef RFEASYLINKRX_ASYNC
        EasyLink_receiveAsync(rxDoneCb, 0);

        /* Wait 300ms for Rx */
        if(Semaphore_pend(rxDoneSem, (300000 / Clock_tickPeriod)) == FALSE)
        {
            /* RX timed out abort */
            if(EasyLink_abort() == EasyLink_Status_Success)
            {
               /* Wait for the abort */
               Semaphore_pend(rxDoneSem, BIOS_WAIT_FOREVER);
            }
        }

        if(Semaphore_getCount(txDoneSem)>0)
        {
            Semaphore_pend(txDoneSem, BIOS_WAIT_FOREVER);
        }

#else
        rxPacket.absTime = 0;
        EasyLink_Status result = EasyLink_receive(&rxPacket);

        if (result == EasyLink_Status_Success)
        {
            System_printf("rfEasyLinkRxFnx EasyLink_receive(%d): %s",rxPacket.len,rxPacket.payload);

            /* Toggle LED2 to indicate RX */
            PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
            PIN_setOutputValue(pinHandle, Board_PIN_LED2,0);        }
        else
        {
            /* Toggle LED1 and LED2 to indicate error */
            PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
            PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
        }
#endif //RX_ASYNC
    }
}

static void rfEasyLinkTxFnx(UArg arg0, UArg arg1)
{
    uint8_t txBurstSize = 0;
    uint32_t absTime;
    int ret=1;

#ifdef RFEASYLINKTX_ASYNC
    /* Create a semaphore for Async */
    Semaphore_Params params;
    Error_Block eb;

    /* Init params */
    Semaphore_Params_init(&params);
    Error_init(&eb);

    /* Create semaphore instance */
    txDoneSem = Semaphore_create(0, &params, &eb);
    if(txDoneSem == NULL)
    {
        System_abort("txDoneSem Semaphore creation failed");
    }


#endif //TX_ASYNC

    while(1) {
        EasyLink_TxPacket txPacket =  { {0}, 0, 0, {0} };

        ret=GetString((char*)txPacket.payload,EASYLINK_MAX_DATA_LENGTH);
        if(ret>0)
        {
            txPacket.len=ret;
            txPacket.dstAddr[0] = 0xaa;

            /* Add a Tx delay for > 500ms, so that the abort kicks in and brakes the burst */
            if(EasyLink_getAbsTime(&absTime) != EasyLink_Status_Success)
            {
                // Problem getting absolute time
            }
            if(txBurstSize++ >= RFEASYLINKTX_BURST_SIZE)
            {
              /* Set Tx absolute time to current time + 1s */
              txPacket.absTime = absTime + EasyLink_ms_To_RadioTime(1000);
              txBurstSize = 0;
            }
            /* Else set the next packet in burst to Tx in 100ms */
            else
            {
              /* Set Tx absolute time to current time + 100ms */
              txPacket.absTime = absTime + EasyLink_ms_To_RadioTime(100);
            }

#ifdef RFEASYLINKTX_ASYNC
            int retry=3;
            int retAsync=0;
            do
            {
                retAsync= EasyLink_transmitAsync(&txPacket, txDoneCb);
                if(retAsync == EasyLink_Status_Busy_Error)
                {
                    /* Rf busy, abort */
                    ret=EasyLink_abort();
                    if(ret == EasyLink_Status_Success)
                    {
                        /*
                         * Abort will cause the txDoneCb to be called and the txDoneSem
                         * to be released, so we must consume the txDoneSem
                         */
                        if(Semaphore_getCount(txDoneSem)>0)
                        {
                            Semaphore_pend(txDoneSem, BIOS_WAIT_FOREVER);
                        }
                        if(Semaphore_getCount(rxDoneSem)>0)
                        {
                            Semaphore_pend(rxDoneSem, BIOS_WAIT_FOREVER);
                        }

                    }
                }
            }while(retry-->1 && retAsync != EasyLink_Status_Success);
            /* Wait 300ms for Tx to complete */
            if(Semaphore_pend(txDoneSem, (300000 / Clock_tickPeriod)) == FALSE)
            {
                /* TX timed out, abort */
                if(EasyLink_abort() == EasyLink_Status_Success)
                {
                    /*
                     * Abort will cause the txDoneCb to be called and the txDoneSem
                     * to be released, so we must consume the txDoneSem
                     */
                   Semaphore_pend(txDoneSem, BIOS_WAIT_FOREVER);
                }
            }
#else
            EasyLink_Status result = EasyLink_transmit(&txPacket);

            if (result == EasyLink_Status_Success)
            {
                /* Toggle LED1 to indicate TX */
                PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
                PIN_setOutputValue(pinHandle, Board_PIN_LED2,0);

            }
            else
            {
                /* Toggle LED1 and LED2 to indicate error */
                PIN_setOutputValue(pinHandle, Board_PIN_LED1,1);
                PIN_setOutputValue(pinHandle, Board_PIN_LED2,1);
            }
#endif //RFEASYLINKTX_ASYNC
        }
    }
}

void rxTask_init() {

    Task_Params_init(&rxTaskParams);
    rxTaskParams.stackSize = RFEASYLINKEX_TASK_STACK_SIZE;
    rxTaskParams.priority = RFEASYLINKEX_TASK_PRIORITY;
    rxTaskParams.stack = &rxTaskStack;
    rxTaskParams.arg0 = (UInt)1000000;

    Task_construct(&rxTask, rfEasyLinkRxFnx, &rxTaskParams, NULL);
}

void txTask_init() {

    Task_Params_init(&txTaskParams);
    txTaskParams.stackSize = RFEASYLINKEX_TASK_STACK_SIZE;
    txTaskParams.priority = RFEASYLINKEX_TASK_PRIORITY;
    txTaskParams.stack = &txTaskStack;
    txTaskParams.arg0 = (UInt)1000000;

    Task_construct(&txTask, rfEasyLinkTxFnx, &txTaskParams, NULL);
}

/*
 *  ======== main ========
 */
int main(void)
{
    /* Call driver init functions */
    Board_initGeneral();

    /* Open LED pins */
    pinHandle = PIN_open(&ledPinState, pinTable);
	Assert_isTrue(pinHandle != NULL, NULL);

    /* Clear LED pins */
    PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
    PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);

    rxTask_init();
    txTask_init();

    UART_init();
    InitTerm();


    /* Start BIOS */
    BIOS_start();

    return (0);
}
