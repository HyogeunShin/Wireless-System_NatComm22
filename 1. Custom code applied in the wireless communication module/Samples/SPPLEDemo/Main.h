/*****< main.c >***************************************************************/
/*      Copyright 2012 - 2014 Stonestreet One.                                */
/*      All Rights Reserved.                                                  */
/*                                                                            */
/*  MAIN - Stonestreet One main sample application header.                    */
/*                                                                            */
/*  Author:  Tim Cook                                                         */
/*                                                                            */
/*** MODIFICATION HISTORY *****************************************************/
/*                                                                            */
/*   mm/dd/yy  F. Lastname    Description of Modification                     */
/*   --------  -----------    ------------------------------------------------*/
/*   01/28/12  T. Cook        Initial creation.                               */
/******************************************************************************/
#ifndef __MAIN_H__
#define __MAIN_H__

#include "SS1BTPS.h"             /* Main SS1 Bluetooth Stack Header.          */
#include "SS1BTVS.h"             /* Vendor Specific Prototypes/Constants.     */
#include "BTPSKRNL.h"            /* BTPS Kernel Prototypes/Constants.         */

   /* The following is used as a printf replacement.                    */
#define Display(_x)                 do { BTPS_OutputMessage _x; } while(0)

   /* Error Return Codes.                                               */

   /* Error Codes that are smaller than these (less than -1000) are     */
   /* related to the Bluetooth Protocol Stack itself (see BTERRORS.H).  */
#define APPLICATION_ERROR_INVALID_PARAMETERS             (-1000)
#define APPLICATION_ERROR_UNABLE_TO_OPEN_STACK           (-1001)

   /* The following function is used to initialize the application      */
   /* instance.  This function should open the stack and prepare to     */
   /* execute commands based on user input.  The first parameter passed */
   /* to this function is the HCI Driver Information that will be used  */
   /* when opening the stack and the second parameter is used to pass   */
   /* parameters to BTPS_Init.  This function returns the               */
   /* BluetoothStackID returned from BSC_Initialize on success or a     */
   /* negative error code (of the form APPLICATION_ERROR_XXX).          */
int InitializeApplication(HCI_DriverInformation_t *HCI_DriverInformation, BTPS_Initialization_t *BTPS_Initialization);

///////////////////////User Function///////////////////////////////////////
void SPI_Init(void);
void DMA_Init(unsigned char *From_Addr, unsigned int length);
void UART_Init(void);
void Buffer_Reset(void);
void SPI_RHD_Init(unsigned char send, unsigned char send2);
void RHD_Init(void);
void RHD_SPI_Buffer_Save(void);
void BL_Periodinc_write(void *Userparameter);
void SPI_BL_Periodinc_write(void *Userparameter);
void AUTOMODE_Start_Automode(void);

#endif

                              /* The following variable is used to hold */
                              /* a system tick count for the Bluetopia  */
                              /* No-OS stack.                           */
static volatile unsigned long MSP430Ticks;

   /* This function is called to get the system Tick Count.             */
unsigned long HAL_GetTickCount(void);

