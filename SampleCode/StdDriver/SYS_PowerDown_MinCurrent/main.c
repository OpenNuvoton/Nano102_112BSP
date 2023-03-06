/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to minimize power consumption when entering power down mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "Nano1X2Series.h"


/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* Enter to Power-down mode */
    CLK_PowerDown();
}

/**
 * @brief       PortA/PortB/PortC IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PortA/PortB/PortC default IRQ, declared in startup_nano1x2series.s.
 */
void GPABC_IRQHandler(void)
{
    uint32_t reg;

    /* To check if PA.1 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PA, BIT1))
        GPIO_CLR_INT_FLAG(PA, BIT1);
    else
    {
        /* Un-expected interrupt. Just clear all PORTA, PORTB, PORTC interrupts */
        reg = PA->ISRC;
        PA->ISRC = reg;
        reg = PB->ISRC;
        PB->ISRC = reg;
        reg = PC->ISRC;
        PC->ISRC = reg;        
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK->PWRCTL &= ~CLK_PWRCTL_HIRC_EN_Msk;
    CLK->PWRCTL |= (0x1 << CLK_PWRCTL_HIRC_EN_Pos); // HIRC Enabled

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_HIRC_STB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HIRC;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

#if 0
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------+\n");
    printf("|  SYS_PowerDown_MinCurrent and Wake-up by PA.1 Sample Code   |\n");
    printf("+-------------------------------------------------------------+\n\n");

    printf("+-------------------------------------------------------------------------+\n");
    printf("+ Operating sequence                                                      |\n");
    printf("|  1. Remove all continuous load, e.g. LED.                               |\n");
    printf("|  2. Set LDO Output Voltage to 1.6v                                      |\n");
    printf("|  3. Disable UART0                                                       |\n");
    printf("|  4. Enter to Power-Down                                                 |\n");
    printf("|  5. Wait for PA.1 falling-edge interrupt event to wakeup the MCU        |\n");
    printf("+-------------------------------------------------------------------------+\n\n");
#endif

    /* Set LDO Output Voltage to 1.6v */
    SYS->LDO_CTL &= ~SYS_CTL_LDO_LEVEL_Msk;
    SYS->LDO_CTL |= (0x1 << SYS_CTL_LDO_LEVEL_Pos);

    /* Set function pin to GPIO mode */
    SYS->PA_L_MFP = 0;
    SYS->PA_H_MFP = 0;
    SYS->PB_L_MFP = 0;
    SYS->PB_H_MFP = 0;
    SYS->PC_L_MFP = 0;
    SYS->PC_H_MFP = 0;
    SYS->PD_L_MFP = 0;
    SYS->PD_H_MFP = 0;
    SYS->PE_L_MFP = 0;
    SYS->PE_H_MFP = 0;
    SYS->PF_L_MFP = 0x00FFFF00;// exclude GPF5:ICE_DAT, GPF4:ICE_CLK, GPF3:HXT_OUT, GPF2:HXT_IN

    /* Enable GPIO pull up */
    PA->PUEN = 0xFFFF;
    PB->PUEN = 0xFFFF;
    PC->PUEN = 0xFFFF;
    PD->PUEN = 0xFFFF;
    PE->PUEN = 0xFFFF;
    PF->PUEN = 0x0003;/* exclude GPF5:ICE_DAT, GPF4:ICE_CLK, GPF3:HXT_OUT, GPF2:HXT_IN */

    /* PA1 is pin 73 */
    GPIO_SetMode(PA, BIT1, GPIO_PMD_INPUT);
    NVIC_EnableIRQ(GPABC_IRQn);

    /* Configure PA.1 as Input mode and enable interrupt by falling edge trigger */
    GPIO_EnableInt(PA, 1, GPIO_INT_FALLING);


    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Waiting for PA.1 falling-edge interrupt event */

    while(1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
