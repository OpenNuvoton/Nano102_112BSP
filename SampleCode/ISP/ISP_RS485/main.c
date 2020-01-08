/***************************************************************************//**
 * @file     main.c
 * @brief    Transmit and receive ISP command from PC terminal through RS232 interface
 * @version  2.0.0
 *
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "Nano1X2Series.h"
#include "targetdev.h"

#define nRTSPin                 (PB2)
#define RECEIVE_MODE            (0)
#define TRANSMIT_MODE           (1)

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable HIRC */
    CLK->PWRCTL |= CLK_PWRCTL_HIRC_EN_Msk | CLK_PWRCTL_HXT_EN_Msk;
    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_PWRCTL_HIRC_EN_Msk);
    CLK_EnablePLL(CLK_PLLCTL_PLL_SRC_HIRC, 32000000);
    CLK_WaitClockReady(CLK_CLKSTATUS_PLL_STB_Msk);
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_PLL, CLK_HCLK_CLK_DIVIDER(1));
    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_UART_CLK_DIVIDER(1));
    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN; // UART0 Clock Enable
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB2 for RTS  */
    PB->PMD = (PB->PMD & ~GP_PMD_PMD2_Msk) | (GPIO_PMD_OUTPUT << GP_PMD_PMD2_Pos);
    nRTSPin = RECEIVE_MODE;

    /* Set PB multi-function pins for UART0 RXD and TXD  */
    SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_TX | SYS_PB_L_MFP_PB1_MFP_UART0_RX);
    /* Lock protected registers */
    // SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART0 for printf */
    UART_Init();
    CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;
    FMC->ISPCON |= FMC_ISPCON_ISPEN_Msk;
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;//using cpu clock

    while (1)
    {
        if ((bufhead >= 4) || (bUartDataReady == TRUE))
        {
            uint32_t lcmd;
            lcmd = inpw(uart_rcvbuf);

            if (lcmd == CMD_CONNECT)
            {
                goto _ISP;
            }
            else
            {
                bUartDataReady = FALSE;
                bufhead = 0;
            }
        }

        //if((SysTick->CTRL & (1 << 16)) != 0)//timeout, then goto APROM
        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            goto _APROM;
        }
    }

_ISP:

    while (1)
    {
        if (bUartDataReady == TRUE)
        {
            bUartDataReady = FALSE;
            NVIC_DisableIRQ(UART0_IRQn);
            nRTSPin = TRANSMIT_MODE;
            ParseCmd(uart_rcvbuf, 64);
            PutString();

            while ((UART0->FSR & UART_FSR_TX_EMPTY_F_Msk) == 0);
            while ((UART0->FSR & UART_FSR_TE_F_Msk) == 0);

            nRTSPin = RECEIVE_MODE;
            NVIC_EnableIRQ(UART0_IRQn);
        }
    }

_APROM:
    outpw(&SYS->RST_SRC, 3);//clear bit
    outpw(&FMC->ISPCON, inpw(&FMC->ISPCON) & 0xFFFFFFFC);
    outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

    /* Trap the CPU */
    while (1);
}

