/*****< kfs.h >****************************************************************/
/*      Copyright 2011 - 2014 Stonestreet One.                                */
/*      All Rights Reserved.                                                  */
/*                                                                            */
/*  ACC - Bluetooth Stack Accelerometer Service Service (GATT based - TI      */
/*        Proprietary) Type Definitions, Prototypes, and Constants.           */
/*                                                                            */
/*  Author:  Tim Cook                                                         */
/*                                                                            */
/*** MODIFICATION HISTORY *****************************************************/
/*                                                                            */
/*   mm/dd/yy  F. Lastname    Description of Modification                     */
/*   --------  -----------    ------------------------------------------------*/
/*   08/27/12  T. Cook        Initial creation.                               */
/******************************************************************************/
#ifndef __ACCH__
#define __ACCH__

#include "BTTypes.h"            /* Bluetooth Type Definitions/Constants.      */

   /* The following function is responsible for making sure that the    */
   /* Bluetooth Stack ACC Module is Initialized correctly.  This        */
   /* function *MUST* be called before ANY other Bluetooth Stack ACC    */
   /* function can be called.  This function returns non-zero if the    */
   /* Module was initialized correctly, or a zero value if there was an */
   /* error.                                                            */
   /* * NOTE * Internally, this module will make sure that this function*/
   /*          has been called at least once so that the module will    */
   /*          function.  Calling this function from an external        */
   /*          location is not necessary.                               */
int InitializeACCModule(void);

   /* The following function is responsible for instructing the         */
   /* Bluetooth Stack ACC Module to clean up any resources that it has  */
   /* allocated.  Once this function has completed, NO other Bluetooth  */
   /* Stack ACC Functions can be called until a successful call to the  */
   /* InitializeACCModule() function is made.  The parameter to this    */
   /* function specifies the context in which this function is being    */
   /* called.  If the specified parameter is TRUE, then the module will */
   /* make sure that NO functions that would require waiting/blocking on*/
   /* Mutexes/Events are called.  This parameter would be set to TRUE if*/
   /* this function was called in a context where threads would not be  */
   /* allowed to run.  If this function is called in the context where  */
   /* threads are allowed to run then this parameter should be set to   */
   /* FALSE.                                                            */
void CleanupACCModule(Boolean_t ForceCleanup);

#endif
