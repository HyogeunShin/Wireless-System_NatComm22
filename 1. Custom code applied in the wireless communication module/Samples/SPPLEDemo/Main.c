/*****< main.c >***************************************************************/
/*      Copyright 2012 - 2014 Stonestreet One.                                */
/*      All Rights Reserved.                                                  */
/*                                                                            */
/*  MAIN - Main application implementation.                                   */
/*                                                                            */
/*  Author:  Tim Cook                                                         */
/*                                                                            */
/*** MODIFICATION HISTORY *****************************************************/
/*                                                                            */
/*   mm/dd/yy  F. Lastname    Description of Modification                     */
/*   --------  -----------    ------------------------------------------------*/
/*   01/28/12  T. Cook        Initial creation.                               */
/******************************************************************************/
#include "HAL.h"                 /* Function for Hardware Abstraction.        */
#include "Main.h"                /* Main application header.                  */
#include "HCITRANS.h"            /* HCI Transport Prototypes/Constants.       */

#define LED_TOGGLE_RATE_SUCCESS                    (8*500) /* The LED Toggle    */
                                                         /* rate when the demo*/
                                                         /* successfully      */
                                                         /* starts up.        */

   /* Internal Variables to this Module (Remember that all variables    */
   /* declared static are initialized to 0 automatically by the         */
   /* compiler as part of standard C/C++).                              */
static unsigned int BluetoothStackID;
static Boolean_t    SleepEnabled;


   /* HCI Sleep Mode Callback.                                          */
static void BTPSAPI HCI_Sleep_Callback(Boolean_t SleepAllowed, unsigned long CallbackParameter);

   /* Application Tasks.                                                */
static void DisplayCallback(char Character);
static unsigned long GetTickCallback(void);
static void ProcessCharacters(void);
static void IdleFunction(void *UserParameter);
static void MainThread(void);



   /* The following is the HCI Sleep Callback.  This is registered with */
   /* the stack to note when the Host processor may enter into a sleep  */
   /* mode.                                                             */
static void BTPSAPI HCI_Sleep_Callback(Boolean_t SleepAllowed, unsigned long CallbackParameter)
{
   /* Simply store the state internally.                                */
   SleepEnabled = SleepAllowed;
}

   /* The following function is registered with the application so that */
   /* it can display strings to the debug UART.                         */
static void DisplayCallback(char Character)
{
}

   /* The following function is registered with the application so that */
   /* it can get the current System Tick Count.                         */
static unsigned long GetTickCallback(void)
{
   return(HAL_GetTickCount());
}

   /* The following function processes terminal input.                  */
static void ProcessCharacters(void)
{
      /* Attempt to process a character.                                */
      //ProcessCommandLine(Input);
}

   /* The following function is responsible for checking the idle state */
   /* and possibly entering LPM3 mode.                                  */
static void IdleFunction(void *UserParameter)
{
   unsigned long        CurrentTickCount;
   unsigned long        ElapsedTicks;
   static unsigned long PreviousTickCount = 0;

   /* If the stack is Idle and we are in HCILL Sleep, then we may enter */
   /* LPM3 mode (with Timer Interrupts disabled).                       */
   if((BSC_QueryStackIdle(BluetoothStackID)) && (SleepEnabled))
   {
      /* Attempt to suspend the UART.                                   */
      if(!HCITR_COMSuspend(0))
      {
         /* Enter MSP430 LPM3 with Timer Interrupts disabled (we will   */
         /* require an interrupt to wake us up from this state).        */
         //HAL_LowPowerMode((unsigned char)TRUE);

         /* Check to see if a wakeup is in progress (by the Controller).*/
         /* If so we will disable sleep mode so that we complete the    */
         /* process.                                                    */
         if(!HCITR_UartSuspended(0))
            SleepEnabled = FALSE;

         /* Go ahead and process any characters we may have received on */
         /* the console UART.                                           */
         ProcessCharacters();
      }
      else
      {
         /* Failed to suspend the UART which must mean that the         */
         /* controller is attempting to do a wakeup.  Therefore we will */
         /* flag that sleep mode is disabled.                           */
         SleepEnabled = FALSE;
      }
   }
   else
   {
      /* Calculate the time that we last toggled the LED to determine if*/
      /* we need to do so again.                                        */
      CurrentTickCount = BTPS_GetTickCount();
      ElapsedTicks     = CurrentTickCount - PreviousTickCount;
      if(ElapsedTicks & 0x80000000)
         ElapsedTicks = CurrentTickCount + (0xFFFFFFFF - PreviousTickCount) + 1;

      if(ElapsedTicks >= LED_TOGGLE_RATE_SUCCESS)
      {
         /* Toggle the LEDs.                                            */
         HAL_LedToggle(0);

         /* Set the tick count for the next toggle.                     */
         PreviousTickCount = CurrentTickCount;
      }

      /* Process any console characters that we may have.               */
      ProcessCharacters();

      /* Enter LPM0 while we wait for something to happen.              */
      LPM0;
   }
}

   /* The following function is the main user interface thread.  It     */
   /* opens the Bluetooth Stack and then drives the main user interface.*/
static void MainThread(void)
{
   int                           Result;
   BTPS_Initialization_t         BTPS_Initialization;
   HCI_DriverInformation_t       HCI_DriverInformation;
   HCI_HCILLConfiguration_t      HCILLConfig;
   HCI_Driver_Reconfigure_Data_t DriverReconfigureData;

  
   /* Flag that sleep is not currently enabled.                         */
   SleepEnabled = FALSE;

   /* Configure the UART Parameters.                                    */
   HCI_DRIVER_SET_COMM_INFORMATION(&HCI_DriverInformation, 1, 115200, cpHCILL_RTS_CTS);
   HCI_DriverInformation.DriverInformation.COMMDriverInformation.InitializationDelay = 100;

   /* Set up the application callbacks.                                 */
   BTPS_Initialization.GetTickCountCallback  = GetTickCallback;
   BTPS_Initialization.MessageOutputCallback = DisplayCallback;

   /* Initialize the application.                                       */
   if((Result = InitializeApplication(&HCI_DriverInformation, &BTPS_Initialization)) > 0)
   {
      /* Save the Bluetooth Stack ID.                                   */
      BluetoothStackID = (unsigned int)Result;

      /* Register a sleep mode callback if we are using HCILL Mode.     */
      if((HCI_DriverInformation.DriverInformation.COMMDriverInformation.Protocol == cpHCILL) || (HCI_DriverInformation.DriverInformation.COMMDriverInformation.Protocol == cpHCILL_RTS_CTS))
      {
         HCILLConfig.SleepCallbackFunction        = HCI_Sleep_Callback;
         HCILLConfig.SleepCallbackParameter       = 0;
         DriverReconfigureData.ReconfigureCommand = HCI_COMM_DRIVER_RECONFIGURE_DATA_COMMAND_CHANGE_HCILL_PARAMETERS;
         DriverReconfigureData.ReconfigureData    = (void *)&HCILLConfig;

         /* Register the sleep mode callback.  Note that if this        */
         /* function returns greater than 0 then sleep is currently     */
         /* enabled.                                                    */
         Result = HCI_Reconfigure_Driver(BluetoothStackID, FALSE, &DriverReconfigureData);
         if(Result > 0)
         {
            /* Flag that sleep mode is enabled.                         */
            SleepEnabled = TRUE;
         }
      }

      /* Add the idle function (which determines if LPM3 may be entered)*/
      /* to the scheduler.                                              */
      if(BTPS_AddFunctionToScheduler(IdleFunction, NULL, 0))//(1)//
      {
        AUTOMODE_Start_Automode();
         /* Loop forever and execute the scheduler.                     */
         while(1)
            BTPS_ExecuteScheduler();
      }
   }
}

   /* The following is the Main application entry point.  This function */
   /* will configure the hardware and initialize the OS Abstraction     */
   /* layer, create the Main application thread and start the scheduler.*/
int main(void)
{
   /* Turn off the watchdog timer                                       */
   WDTCTL = WDTPW | WDTHOLD;

   /* Configure the hardware for its intended use.                      */
   HAL_ConfigureHardware();

   /* Enable interrupts and call the main application thread.           */
   __enable_interrupt();
   MainThread();

   /* MainThread should run continously, if it exits an error occured.  */
   while(1)
   {
      HAL_LedToggle(0);
      BTPS_Delay(100);
   }
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////User Function//////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
