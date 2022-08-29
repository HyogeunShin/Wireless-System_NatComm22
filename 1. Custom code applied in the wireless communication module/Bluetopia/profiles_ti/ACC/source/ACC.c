/*****< acc.c >****************************************************************/
/*      Copyright 2011 - 2014 Stonestreet One.                                */
/*      All Rights Reserved.                                                  */
/*                                                                            */
/*  ACC - Bluetooth Stack Accelerometer Implementation (GATT Based - TI       */
/*        Proprietary) for Stonestreet One Bluetooth Protocol Stack.          */
/*                                                                            */
/*  Author:  Tim Cook                                                         */
/*                                                                            */
/*** MODIFICATION HISTORY *****************************************************/
/*                                                                            */
/*   mm/dd/yy  F. Lastname    Description of Modification                     */
/*   --------  -----------    ------------------------------------------------*/
/*   08/27/12  T. Cook        Initial creation.                               */
/******************************************************************************/
#include "SS1BTPS.h"        /* Bluetooth Stack API Prototypes/Constants.      */
#include "SS1BTGAT.h"       /* Bluetooth Stack GATT API Prototypes/Constants. */
#include "SS1BTACC.h"       /* Bluetooth ACC API Prototypes/Constants.        */

#include "BTPSKRNL.h"       /* BTPS Kernel Prototypes/Constants.              */
#include "ACC.h"            /* Bluetooth ACC Prototypes/Constants.            */

   /* The following controls the number of supported ACC instances.     */
#define ACCELEROMETER_MAXIMUM_SUPPORTED_INSTANCES        (1)

   /* The following structure defines the ACC Instance data this unique */
   /* per ACC service instance.                                         */
typedef struct _tagACC_Instance_Data_t
{
   Word_t           RangeLength;
   NonAlignedWord_t Range;
} ACC_Instance_Data_t;

#define ACC_INSTANCE_DATA_SIZE                           (sizeof(ACC_Instance_Data_t))

   /* The following define the various instance data tags.              */
#define ACC_RANGE_INSTANCE_TAG                           (BTPS_STRUCTURE_OFFSET(ACC_Instance_Data_t, RangeLength)) 

   /* The Accelerometer Service Declaration.                            */
static BTPSCONST GATT_Primary_Service_16_Entry_t ACC_Service_UUID = 
{
   TI_ACC_SERVICE_BLUETOOTH_UUID_CONSTANT
}; 


   /* The Enable Characteristic Declaration.                            */
static BTPSCONST GATT_Characteristic_Declaration_16_Entry_t ACC_Enable_Declaration =
{
   (GATT_CHARACTERISTIC_PROPERTIES_READ | GATT_CHARACTERISTIC_PROPERTIES_WRITE),
   TI_ACC_ENABLE_CHARACTERISTIC_BLUETOOTH_UUID_CONSTANT
};

   /* The Enable Characteristic Value.                                  */
static BTPSCONST GATT_Characteristic_Value_16_Entry_t  ACC_Enable_Value = 
{
   TI_ACC_ENABLE_CHARACTERISTIC_BLUETOOTH_UUID_CONSTANT,
   0,
   NULL
};


   /* The Range Characteristic Declaration.                             */
static BTPSCONST GATT_Characteristic_Declaration_16_Entry_t ACC_Range_Declaration =
{
   GATT_CHARACTERISTIC_PROPERTIES_READ,
   TI_ACC_RANGE_CHARACTERISTIC_BLUETOOTH_UUID_CONSTANT
};

   /* The Range Characteristic Value.                                   */
static BTPSCONST GATT_Characteristic_Value_16_Entry_t  ACC_Range_Value = 
{
   TI_ACC_RANGE_CHARACTERISTIC_BLUETOOTH_UUID_CONSTANT,
   0,
   NULL
};


   /* The X-Axis Characteristic Declaration.                            */
static BTPSCONST GATT_Characteristic_Declaration_16_Entry_t ACC_XAxis_Declaration =
{
   GATT_CHARACTERISTIC_PROPERTIES_NOTIFY,
   TI_ACC_X_AXIS_CHARACTERISTIC_BLUETOOTH_UUID_CONSTANT
};

   /* The X-Axis Characteristic Value.                                  */
static BTPSCONST GATT_Characteristic_Value_16_Entry_t  ACC_XAxis_Value = 
{
   TI_ACC_X_AXIS_CHARACTERISTIC_BLUETOOTH_UUID_CONSTANT,
   0,
   NULL
};

   /* The Y-Axis Characteristic Declaration.                            */
static BTPSCONST GATT_Characteristic_Declaration_16_Entry_t ACC_YAxis_Declaration =
{
   GATT_CHARACTERISTIC_PROPERTIES_NOTIFY,
   TI_ACC_Y_AXIS_CHARACTERISTIC_BLUETOOTH_UUID_CONSTANT
};

   /* The Y-Axis Characteristic Value.                                  */
static BTPSCONST GATT_Characteristic_Value_16_Entry_t  ACC_YAxis_Value = 
{
   TI_ACC_Y_AXIS_CHARACTERISTIC_BLUETOOTH_UUID_CONSTANT,
   0,
   NULL
};

   /* The Z-Axis Characteristic Declaration.                            */
static BTPSCONST GATT_Characteristic_Declaration_16_Entry_t ACC_ZAxis_Declaration =
{
   GATT_CHARACTERISTIC_PROPERTIES_NOTIFY,
   TI_ACC_Z_AXIS_CHARACTERISTIC_BLUETOOTH_UUID_CONSTANT
};

   /* The Z-Axis Characteristic Value.                                  */
static BTPSCONST GATT_Characteristic_Value_16_Entry_t  ACC_ZAxis_Value = 
{
   TI_ACC_Z_AXIS_CHARACTERISTIC_BLUETOOTH_UUID_CONSTANT,
   0,
   NULL
};

   /* The Client Characteristic Descriptor.                             */
static BTPSCONST GATT_Characteristic_Descriptor_16_Entry_t ACC_Client_Characteristic_Configuration_Value = 
{
   {0x02, 0x29},
   0,
   NULL
};


   /* The following defines the Accelerometer Service that is registered*/
   /* with the GATT_Register_Service function call.                     */
   /* * NOTE * This array will be registered with GATT in the call to   */
   /*          GATT_Register_Service.                                   */
BTPSCONST GATT_Service_Attribute_Entry_t Accelerometer_Service[] = 
{
   {GATT_ATTRIBUTE_FLAGS_READABLE,          aetPrimaryService16,            (Byte_t *)&ACC_Service_UUID},                               /* ACC Primary Service Declaration (0).   */
   {GATT_ATTRIBUTE_FLAGS_READABLE,          aetCharacteristicDeclaration16, (Byte_t *)&ACC_Enable_Declaration},                         /* ACC Enable Declaration(1).             */
   {GATT_ATTRIBUTE_FLAGS_READABLE_WRITABLE, aetCharacteristicValue16,       (Byte_t *)&ACC_Enable_Value},                               /* ACC Enable Value(2).                   */
   {GATT_ATTRIBUTE_FLAGS_READABLE,          aetCharacteristicDeclaration16, (Byte_t *)&ACC_Range_Declaration},                          /* ACC Range Declaration(3).              */
   {GATT_ATTRIBUTE_FLAGS_READABLE,          aetCharacteristicValue16,       (Byte_t *)&ACC_Range_Value},                                /* ACC Range Value(4).                    */
   {GATT_ATTRIBUTE_FLAGS_READABLE,          aetCharacteristicDeclaration16, (Byte_t *)&ACC_XAxis_Declaration},                          /* ACC X-Axis Declaration(5).             */
   {0,                                      aetCharacteristicValue16,       (Byte_t *)&ACC_XAxis_Value},                                /* ACC X-Axis Value(6).                   */
   {GATT_ATTRIBUTE_FLAGS_READABLE_WRITABLE, aetCharacteristicDescriptor16,  (Byte_t *)&ACC_Client_Characteristic_Configuration_Value},  /* ACC X-Axis CCCD(7).                    */
   {GATT_ATTRIBUTE_FLAGS_READABLE,          aetCharacteristicDeclaration16, (Byte_t *)&ACC_YAxis_Declaration},                          /* ACC Y-Axis Declaration(8).             */
   {0,                                      aetCharacteristicValue16,       (Byte_t *)&ACC_YAxis_Value},                                /* ACC Y-Axis Value(9).                   */
   {GATT_ATTRIBUTE_FLAGS_READABLE_WRITABLE, aetCharacteristicDescriptor16,  (Byte_t *)&ACC_Client_Characteristic_Configuration_Value},  /* ACC Y-Axis CCCD(10).                   */
   {GATT_ATTRIBUTE_FLAGS_READABLE,          aetCharacteristicDeclaration16, (Byte_t *)&ACC_ZAxis_Declaration},                          /* ACC Z-Axis Declaration(11).            */
   {0,                                      aetCharacteristicValue16,       (Byte_t *)&ACC_ZAxis_Value},                                /* ACC Z-Axis Value(12).                  */
   {GATT_ATTRIBUTE_FLAGS_READABLE_WRITABLE, aetCharacteristicDescriptor16,  (Byte_t *)&ACC_Client_Characteristic_Configuration_Value},  /* ACC Z-Axis CCCD(13).                   */
};   

#define ACCELEROMETER_SERVICE_ATTRIBUTE_COUNT               (sizeof(Accelerometer_Service)/sizeof(GATT_Service_Attribute_Entry_t))

   /* The following defines the ACC GATT Service Flags MASK that should */
   /* be passed into GATT_Register_Service when the ACC Service is      */
   /* registered.                                                       */
#define ACC_SERVICE_FLAGS                                   (GATT_SERVICE_FLAGS_LE_SERVICE|GATT_SERVICE_FLAGS_BR_EDR_SERVICE)

#define ACC_ENABLE_VALUE_ATTRIBUTE_OFFSET                   2
#define ACC_RANGE_VALUE_ATTRIBUTE_OFFSET                    4

#define ACC_X_AXIS_VALUE_ATTRIBUTE_OFFSET                   6
#define ACC_X_AXIS_CCCD_ATTRIBUTE_OFFSET                    7
#define ACC_AXIS_OFFSET                                     3

   /*********************************************************************/
   /**                    END OF SERVICE TABLE                         **/
   /*********************************************************************/

   /* ACC Service Instance Block.  This structure contains All          */
   /* information associated with a specific Bluetooth Stack ID (member */
   /* is present in this structure).                                    */
typedef struct _tagACCServerInstance_t
{
   unsigned int         BluetoothStackID;
   unsigned int         ServiceID;
   ACC_Event_Callback_t EventCallback;
   unsigned long        CallbackParameter;
} ACCServerInstance_t;

#define ACC_SERVER_INSTANCE_DATA_SIZE                    (sizeof(ACCServerInstance_t))

   /* Internal Variables to this Module (Remember that all variables    */
   /* declared static are initialized to 0 automatically by the         */
   /* compiler as part of standard C/C++).                              */

static ACC_Instance_Data_t InstanceData[ACCELEROMETER_MAXIMUM_SUPPORTED_INSTANCES];
                                            /* Variable which holds all */
                                            /* data that is unique for  */
                                            /* each service instance.   */

static ACCServerInstance_t InstanceList[ACCELEROMETER_MAXIMUM_SUPPORTED_INSTANCES];    
                                            /* Variable which holds the */
                                            /* service instance data.   */

static Boolean_t InstanceListInitialized;   /* Variable that flags that */
                                            /* is used to denote that   */
                                            /* this module has been     */
                                            /* successfully initialized.*/

   /* The following are the prototypes of local functions.              */
static Boolean_t InitializeModule(void);
static void CleanupModule(void);

static Boolean_t InstanceRegisteredByStackID(unsigned int BluetoothStackID);
static ACCServerInstance_t *AcquireServiceInstance(unsigned int BluetoothStackID, unsigned int *InstanceID);
static int DecodeClientConfigurationValue(unsigned int BufferLength, Byte_t *Buffer, Boolean_t *NotificationsEnabled);

static Word_t GetAxisOffset(ACC_Characteristic_Type_t CharacteristicType, Boolean_t Descriptor); 
static ACC_Characteristic_Type_t GetAxisCharacteristicDescriptorType(Word_t AttributeOffset);

   /* Bluetooth Event Callbacks.                                        */
static void BTPSAPI GATT_ServerEventCallback(unsigned int BluetoothStackID, GATT_Server_Event_Data_t *GATT_ServerEventData, unsigned long CallbackParameter);

   /* The following function is a utility function that is used to      */
   /* reduce the ifdef blocks that are needed to handle the difference  */
   /* between module initialization for Threaded and NonThreaded stacks.*/
static Boolean_t InitializeModule(void)
{
   /* All we need to do is flag that we are initialized.                */
   if(!InstanceListInitialized)
   {
      InstanceListInitialized = TRUE;

      BTPS_MemInitialize(InstanceList, 0, sizeof(InstanceList));
   }

   return(TRUE);
}

   /* The following function is a utility function that exists to       */
   /* perform stack specific (threaded versus nonthreaded) cleanup.     */
static void CleanupModule(void)
{
   /* Flag that we are no longer initialized.                           */
   InstanceListInitialized = FALSE;
}

   /* The following function is a utility function that exists to check */
   /* to see if an instance has already been registered for a specified */
   /* Bluetooth Stack ID.                                               */
   /* * NOTE * Since this is an internal function no check is done on   */
   /*          the input parameters.                                    */
static Boolean_t InstanceRegisteredByStackID(unsigned int BluetoothStackID)
{
   Boolean_t    ret_val = FALSE;
   unsigned int Index;

   for(Index=0;Index<ACCELEROMETER_MAXIMUM_SUPPORTED_INSTANCES;Index++)
   {
      if((InstanceList[Index].BluetoothStackID == BluetoothStackID) && (InstanceList[Index].ServiceID))
      {
         ret_val = TRUE;
         break;
      }
   }

   /* Finally return the result to the caller.                          */
   return(ret_val);
}

   /* The following function is a utility function that exists to       */
   /* acquire a specified service instance.                             */
   /* * NOTE * Since this is an internal function no check is done on   */
   /*          the input parameters.                                    */
   /* * NOTE * If InstanceID is set to 0, this function will return the */
   /*          next free instance.                                      */
static ACCServerInstance_t *AcquireServiceInstance(unsigned int BluetoothStackID, unsigned int *InstanceID)
{
   unsigned int          LocalInstanceID;
   unsigned int          Index;
   ACCServerInstance_t *ret_val = NULL;

   /* Lock the Bluetooth Stack to gain exclusive access to this         */
   /* Bluetooth Protocol Stack.                                         */
   if(!BSC_LockBluetoothStack(BluetoothStackID))
   {
      /* Acquire the BSC List Lock while we are searching the instance  */
      /* list.                                                          */
      if(BSC_AcquireListLock())
      {
         /* Store a copy of the passed in InstanceID locally.           */
         LocalInstanceID = *InstanceID;

         /* Verify that the Instance ID is valid.                       */
         if((LocalInstanceID) && (LocalInstanceID <= ACCELEROMETER_MAXIMUM_SUPPORTED_INSTANCES))
         {
            /* Decrement the LocalInstanceID (to access the InstanceList*/
            /* which is 0 based).                                       */
            --LocalInstanceID;

            /* Verify that this Instance is registered and valid.       */
            if((InstanceList[LocalInstanceID].BluetoothStackID == BluetoothStackID) && (InstanceList[LocalInstanceID].ServiceID))
            {
               /* Return a pointer to this instance.                    */
               ret_val = &InstanceList[LocalInstanceID];
            }
         }
         else
         {
            /* Verify that we have been requested to find the next free */
            /* instance.                                                */
            if(!LocalInstanceID)
            {
               /* Try to find a free instance.                          */
               for(Index=0;Index<ACCELEROMETER_MAXIMUM_SUPPORTED_INSTANCES;Index++)
               {
                  /* Check to see if this instance is being used.       */
                  if(!(InstanceList[Index].ServiceID))
                  {
                     /* Return the InstanceID AND a pointer to the      */
                     /* instance.                                       */
                     *InstanceID = Index+1;
                     ret_val     = &InstanceList[Index];
                     break;
                  }
               }
            }
         }

         /* Release the previously acquired list lock.                  */
         BSC_ReleaseListLock();
      }
      
      /* If we failed to acquire the instance then we should un-lock the*/
      /* previously acquired Bluetooth Stack.                           */
      if(!ret_val)
         BSC_UnLockBluetoothStack(BluetoothStackID);
   }

   /* Finally return the result to the caller.                          */
   return(ret_val);
}

   /* The following function is a utility function that exists to decode*/
   /* an Client Configuration value into a user specified boolean value.*/
   /* This function returns the zero if successful or a negative error  */
   /* code.                                                             */
static int DecodeClientConfigurationValue(unsigned int BufferLength, Byte_t *Buffer, Boolean_t *NotificationsEnabled)
{
   int    ret_val = ACC_ERROR_INVALID_PARAMETER;
   Word_t ClientConfiguration;

   /* Verify that the input parameters are valid.                       */
   if((BufferLength == NON_ALIGNED_WORD_SIZE) && (Buffer) && (NotificationsEnabled))
   {
      /* Read the requested Client Configuration.                       */
      ClientConfiguration = READ_UNALIGNED_WORD_LITTLE_ENDIAN(Buffer);

      if(ClientConfiguration == GATT_CLIENT_CONFIGURATION_CHARACTERISTIC_NOTIFY_ENABLE)
      {
         *NotificationsEnabled = TRUE;
         ret_val               = 0;
      }
      else
      {
         if(!ClientConfiguration)
         {
            *NotificationsEnabled = FALSE;
            ret_val               = 0;
         }
      }
   }
   else
   {
      if(BufferLength == NON_ALIGNED_WORD_SIZE)
         ret_val = ACC_ERROR_INVALID_PARAMETER;
   }

   /* Finally return the result to the caller.                          */
   return(ret_val);
}

   /* The following function is used to return either the Value or CCCD */
   /* descriptor for the specified axis.                                */
static Word_t GetAxisOffset(ACC_Characteristic_Type_t CharacteristicType, Boolean_t Descriptor)
{
   Word_t AxisOffset = (Descriptor)?ACC_X_AXIS_CCCD_ATTRIBUTE_OFFSET:ACC_X_AXIS_VALUE_ATTRIBUTE_OFFSET;
   
   if(CharacteristicType == ctYAxis)
      AxisOffset += ACC_AXIS_OFFSET;
   else
   {
      if(CharacteristicType == ctZAxis)
      {
         /* Multiple the offset by 2 to get the offset for the Z-Axis.  */
         AxisOffset += (ACC_AXIS_OFFSET<<1);
      }
   }

   return(AxisOffset);
}

   /* The following function is used to return the characteristic type  */
   /* for the specified offset.                                         */
static ACC_Characteristic_Type_t GetAxisCharacteristicDescriptorType(Word_t AttributeOffset)
{
   ACC_Characteristic_Type_t Type = ctXAxis;

   if(AttributeOffset == (ACC_X_AXIS_CCCD_ATTRIBUTE_OFFSET + ACC_AXIS_OFFSET))
      Type = ctYAxis;
   else
   {
      if(AttributeOffset == (ACC_X_AXIS_CCCD_ATTRIBUTE_OFFSET + (ACC_AXIS_OFFSET*2)))
         Type = ctZAxis;
   }

   return(Type);
}

   /* The following function is the GATT Server Event Callback that     */
   /* handles all requests made to the DIS Service for all registered   */
   /* instances.                                                        */
static void BTPSAPI GATT_ServerEventCallback(unsigned int BluetoothStackID, GATT_Server_Event_Data_t *GATT_ServerEventData, unsigned long CallbackParameter)
{
   Word_t                                     AttributeOffset;
   Word_t                                     ValueLength;
   Byte_t                                    *Value;
   unsigned int                               TransactionID;
   unsigned int                               InstanceID;
   ACC_Event_Data_t                           EventData;
   ACCServerInstance_t                       *ServiceInstance;
   
   union
   {
      ACC_Read_Client_Configuration_Data_t           ReadClientData;
      ACC_Client_Configuration_Update_Data_t         ClientUpdateData;
      ACC_Accelerometer_Enable_Read_Request_Data_t   ReadRequestData;
      ACC_Accelerometer_Enable_Update_Request_Data_t UpdateData;
   } EventBuffer;

   /* Verify that all parameters to this callback are Semi-Valid.       */
   if((GATT_ServerEventData) && (CallbackParameter))
   {
      /* The Instance ID is always registered as the callback parameter.*/
      InstanceID = (unsigned int)CallbackParameter;

      /* Acquire the Service Instance for the specified service.        */
      if((ServiceInstance = AcquireServiceInstance(BluetoothStackID, &InstanceID)) != NULL)
      {
         switch(GATT_ServerEventData->Event_Data_Type)
         {
            case etGATT_Server_Read_Request:
               /* Verify that the Event Data is valid.                  */
               if(GATT_ServerEventData->Event_Data.GATT_Read_Request_Data)
               {                  
                  AttributeOffset = GATT_ServerEventData->Event_Data.GATT_Read_Request_Data->AttributeOffset;
                  TransactionID   = GATT_ServerEventData->Event_Data.GATT_Read_Request_Data->TransactionID;

                  /* Check to see if this is a read of the User         */
                  /* Description Descriptor or of the Client            */
                  /* Characteristic Configuration Descriptor.           */
                  if(Accelerometer_Service[AttributeOffset].Attribute_Entry_Type == aetCharacteristicValue16)
                  {
                     /* Verify that the offset is in range.             */
                     if(!(GATT_ServerEventData->Event_Data.GATT_Read_Request_Data->AttributeValueOffset))
                     {
                        /* Check to see what the characteristic the     */
                        /* client is trying to read.                    */
                        if(AttributeOffset == ACC_RANGE_VALUE_ATTRIBUTE_OFFSET)
                        {
                           /* Calculate the length of the data that we  */
                           /* are going to return.                      */
                           ValueLength = (Word_t)(InstanceData[InstanceID-1].RangeLength);
                           Value       = (Byte_t *)&(InstanceData[InstanceID-1].Range);
      
                           /* Respond with the data.                    */
                           GATT_Read_Response(BluetoothStackID, TransactionID, (unsigned int)ValueLength, Value);
                        }
                        else
                        {
                           /* Format a read event for the Enable        */
                           /* Characteristic.                           */
                           EventBuffer.ReadRequestData.InstanceID                          = InstanceID;
                           EventBuffer.ReadRequestData.ConnectionID                        = GATT_ServerEventData->Event_Data.GATT_Read_Request_Data->ConnectionID;
                           EventBuffer.ReadRequestData.TransactionID                       = TransactionID;
                           EventBuffer.ReadRequestData.ConnectionType                      = GATT_ServerEventData->Event_Data.GATT_Read_Request_Data->ConnectionType;
                           EventBuffer.ReadRequestData.RemoteDevice                        = GATT_ServerEventData->Event_Data.GATT_Read_Request_Data->RemoteDevice;
                                                                                           
                           EventData.Event_Data_Type                                       = etACC_Server_Accelerometer_Enable_Read_Request;
                           EventData.Event_Data_Size                                       = ACC_ACCELEROMETER_ENABLE_READ_REQUEST_DATA_SIZE;
                           EventData.Event_Data.ACC_Accelerometer_Enable_Read_Request_Data = &(EventBuffer.ReadRequestData);

                           /* Dispatch the event.                       */
                           __BTPSTRY
                           {
                              (*ServiceInstance->EventCallback)(ServiceInstance->BluetoothStackID, &EventData, ServiceInstance->CallbackParameter);
                           }
                           __BTPSEXCEPT(1)
                           {
                              /* Do Nothing.                            */
                           }
                        }
                     }
                     else
                        GATT_Error_Response(BluetoothStackID, TransactionID, AttributeOffset, ATT_PROTOCOL_ERROR_CODE_ATTRIBUTE_NOT_LONG);
                  }
                  else
                  {
                     /* This is a read of the Client Characteristic     */
                     /* Configuration Descriptor so we must dispatch    */
                     /* this to the application to handle.              */
                     EventBuffer.ReadClientData.InstanceID                   = InstanceID;
                     EventBuffer.ReadClientData.ConnectionID                 = GATT_ServerEventData->Event_Data.GATT_Read_Request_Data->ConnectionID;
                     EventBuffer.ReadClientData.TransactionID                = TransactionID;
                     EventBuffer.ReadClientData.ConnectionType               = GATT_ServerEventData->Event_Data.GATT_Read_Request_Data->ConnectionType;
                     EventBuffer.ReadClientData.RemoteDevice                 = GATT_ServerEventData->Event_Data.GATT_Read_Request_Data->RemoteDevice;
                     EventBuffer.ReadClientData.CharacteristicType           = GetAxisCharacteristicDescriptorType(AttributeOffset);   
                                                                             
                     EventData.Event_Data_Type                               = etACC_Server_Read_Client_Configuration_Request;
                     EventData.Event_Data_Size                               = ACC_READ_CLIENT_CONFIGURATION_DATA_SIZE;
                     EventData.Event_Data.ACC_Read_Client_Configuration_Data = &(EventBuffer.ReadClientData);

                     /* Dispatch the event.                             */
                     __BTPSTRY
                     {
                        (*ServiceInstance->EventCallback)(ServiceInstance->BluetoothStackID, &EventData, ServiceInstance->CallbackParameter);
                     }
                     __BTPSEXCEPT(1)
                     {
                        /* Do Nothing.                                  */
                     }
                  }
               }
               break;
            case etGATT_Server_Write_Request:
               if(GATT_ServerEventData->Event_Data.GATT_Write_Request_Data)
               {
                  AttributeOffset = GATT_ServerEventData->Event_Data.GATT_Write_Request_Data->AttributeOffset;
                  TransactionID   = GATT_ServerEventData->Event_Data.GATT_Write_Request_Data->TransactionID;
                  ValueLength     = GATT_ServerEventData->Event_Data.GATT_Write_Request_Data->AttributeValueLength;
                  Value           = GATT_ServerEventData->Event_Data.GATT_Write_Request_Data->AttributeValue;

                  if((!(GATT_ServerEventData->Event_Data.GATT_Write_Request_Data->AttributeValueOffset)) && (!(GATT_ServerEventData->Event_Data.GATT_Write_Request_Data->DelayWrite)))
                  {
                     /* Check to see if the client is writing to the    */
                     /* Enable Characterisit or one of the CCCDs.       */
                     if(Accelerometer_Service[AttributeOffset].Attribute_Entry_Type == aetCharacteristicValue16)
                     {
                        /* The client is attempting to update the enable*/
                        /* characteristic.                              */

                        /* Re-use the Attribute Offset local variable to*/
                        /* read the Value.                              */
                        if(ValueLength == 1)
                           AttributeOffset = (Word_t)READ_UNALIGNED_BYTE_LITTLE_ENDIAN(Value);
                        else
                           AttributeOffset = READ_UNALIGNED_WORD_LITTLE_ENDIAN(Value);

                        /* Format the event.                            */
                        EventBuffer.UpdateData.InstanceID                                 = InstanceID;
                        EventBuffer.UpdateData.ConnectionID                               = GATT_ServerEventData->Event_Data.GATT_Write_Request_Data->ConnectionID;
                        EventBuffer.UpdateData.TransactionID                              = TransactionID;
                        EventBuffer.UpdateData.ConnectionType                             = GATT_ServerEventData->Event_Data.GATT_Write_Request_Data->ConnectionType;
                        EventBuffer.UpdateData.RemoteDevice                               = GATT_ServerEventData->Event_Data.GATT_Write_Request_Data->RemoteDevice;
                        EventBuffer.UpdateData.EnableAccelerometer                        = (AttributeOffset == TI_ACC_ENABLE_ACCELEROMETER_DISABLED)?FALSE:TRUE;
                                                                                          
                        EventData.Event_Data_Type                                         = etACC_Server_Accelerometer_Enable_Update_Request;
                        EventData.Event_Data_Size                                         = ACC_ACCELEROMETER_ENABLE_UPDATE_REQUEST_DATA_SIZE;
                        EventData.Event_Data.ACC_Accelerometer_Enable_Update_Request_Data = &(EventBuffer.UpdateData);

                        /* Dispatch the event.                          */
                        __BTPSTRY
                        {
                           (*ServiceInstance->EventCallback)(ServiceInstance->BluetoothStackID, &EventData, ServiceInstance->CallbackParameter);
                        }
                        __BTPSEXCEPT(1)
                        {
                           /* Do Nothing.                               */
                        }
                     }
                     else
                     {
                        /* Format the Client Configuration Update Event.*/
                        EventBuffer.ClientUpdateData.InstanceID                   = InstanceID;
                        EventBuffer.ClientUpdateData.ConnectionID                 = GATT_ServerEventData->Event_Data.GATT_Write_Request_Data->ConnectionID;
                        EventBuffer.ClientUpdateData.ConnectionType               = GATT_ServerEventData->Event_Data.GATT_Write_Request_Data->ConnectionType;
                        EventBuffer.ClientUpdateData.RemoteDevice                 = GATT_ServerEventData->Event_Data.GATT_Write_Request_Data->RemoteDevice;
                        EventBuffer.ClientUpdateData.CharacteristicType           = GetAxisCharacteristicDescriptorType(AttributeOffset);
                                                                                  
                        EventData.Event_Data_Type                                 = etACC_Server_Client_Configuration_Update;
                        EventData.Event_Data_Size                                 = ACC_CLIENT_CONFIGURATION_UPDATE_DATA_SIZE;
                        EventData.Event_Data.ACC_Client_Configuration_Update_Data = &(EventBuffer.ClientUpdateData);
   
                        /* Attempt to decode the request Client         */
                        /* Configuration.                               */
                        if(!DecodeClientConfigurationValue(ValueLength, Value, &(EventBuffer.ClientUpdateData.NotificationsEnabled)))
                        {
                           /* Go ahead and accept the write request     */
                           /* since we have decode the Client           */
                           /* Configuration Value successfully.         */
                           GATT_Write_Response(BluetoothStackID, TransactionID);
   
                           /* Dispatch the event.                       */
                           __BTPSTRY
                           {
                              (*ServiceInstance->EventCallback)(ServiceInstance->BluetoothStackID, &EventData, ServiceInstance->CallbackParameter);
                           }
                           __BTPSEXCEPT(1)
                           {
                              /* Do Nothing.                            */
                           }
                        }
                        else
                           GATT_Error_Response(BluetoothStackID, TransactionID, AttributeOffset, ATT_PROTOCOL_ERROR_CODE_REQUEST_NOT_SUPPORTED);
                     }
                  }
                  else
                     GATT_Error_Response(BluetoothStackID, TransactionID, AttributeOffset, ATT_PROTOCOL_ERROR_CODE_REQUEST_NOT_SUPPORTED);
               }
               break;
            default:
               /* Do nothing, as this is just here to get rid of        */
               /* warnings that some compilers flag when not all cases  */
               /* are handled in a switch off of a enumerated value.    */
               break;
         }
   
         /* UnLock the previously locked Bluetooth Stack.               */
         BSC_UnLockBluetoothStack(ServiceInstance->BluetoothStackID);
      }
   }
}

   /* The following function is responsible for making sure that the    */
   /* Bluetooth Stack KFS Module is Initialized correctly.  This        */
   /* function *MUST* be called before ANY other Bluetooth Stack KFS    */
   /* function can be called.  This function returns non-zero if the    */
   /* Module was initialized correctly, or a zero value if there was an */
   /* error.                                                            */
   /* * NOTE * Internally, this module will make sure that this function*/
   /*          has been called at least once so that the module will    */
   /*          function.  Calling this function from an external        */
   /*          location is not necessary.                               */
int InitializeACCModule(void)
{
   return((int)InitializeModule());
}

   /* The following function is responsible for instructing the         */
   /* Bluetooth Stack KFS Module to clean up any resources that it has  */
   /* allocated.  Once this function has completed, NO other Bluetooth  */
   /* Stack KFS Functions can be called until a successful call to the  */
   /* InitializeKFSModule() function is made.  The parameter to this    */
   /* function specifies the context in which this function is being    */
   /* called.  If the specified parameter is TRUE, then the module will */
   /* make sure that NO functions that would require waiting/blocking on*/
   /* Mutexes/Events are called.  This parameter would be set to TRUE if*/
   /* this function was called in a context where threads would not be  */
   /* allowed to run.  If this function is called in the context where  */
   /* threads are allowed to run then this parameter should be set to   */
   /* FALSE.                                                            */
void CleanupACCModule(Boolean_t ForceCleanup)
{
   /* Check to make sure that this module has been initialized.         */
   if(InstanceListInitialized)
   {
      /* Wait for access to the DIS Context List.                       */
      if((ForceCleanup) || ((!ForceCleanup) && (BSC_AcquireListLock())))
      {
         /* Cleanup the Instance List.                                  */
         BTPS_MemInitialize(InstanceList, 0, sizeof(InstanceList));

         if(!ForceCleanup)
            BSC_ReleaseListLock();
      }

      /* Cleanup the module.                                            */
      CleanupModule();
   }
}

   /* The following function is responsible for opening a ACC Server.   */
   /* The first parameter is the Bluetooth Stack ID on which to open the*/
   /* server.  The final parameter is a pointer to store the GATT       */
   /* Service ID of the registered ACC service.  This can be used to    */
   /* include the service registered by this call.  This function       */
   /* returns the positive, non-zero, Instance ID or a negative error   */
   /* code.                                                             */
int BTPSAPI ACC_Initialize_Service(unsigned int BluetoothStackID, ACC_Event_Callback_t EventCallback, unsigned long CallbackParameter, unsigned int *ServiceID)
{
   int                            ret_val;
   unsigned int                   InstanceID;
   ACCServerInstance_t           *ServiceInstance;
   GATT_Attribute_Handle_Group_t  ServiceHandleGroup;

   /* Make sure the parameters passed to us are semi-valid.             */
   if(EventCallback)
   {
      /* Verify that no instance is registered to this Bluetooth Stack. */
      if(!InstanceRegisteredByStackID(BluetoothStackID))
      {
         /* Acquire a free DIS Instance.                                */
         InstanceID = 0;
         if((ServiceInstance = AcquireServiceInstance(BluetoothStackID, &InstanceID)) != NULL)
         {
            /* Call GATT to register the DIS service.                   */
            ret_val = GATT_Register_Service(BluetoothStackID, ACC_SERVICE_FLAGS, ACCELEROMETER_SERVICE_ATTRIBUTE_COUNT, (GATT_Service_Attribute_Entry_t *)Accelerometer_Service, &ServiceHandleGroup, GATT_ServerEventCallback, InstanceID);
            if(ret_val > 0)
            {
               /* Save the Instance information.                        */
               ServiceInstance->BluetoothStackID      = BluetoothStackID;
               ServiceInstance->ServiceID             = (unsigned int)ret_val;
               ServiceInstance->EventCallback         = EventCallback;
               ServiceInstance->CallbackParameter     = CallbackParameter;

               if(ServiceID)
                  *ServiceID                          = (unsigned int)ret_val;

               /* Intilize the Instance Data for this instance.         */
               BTPS_MemInitialize(&InstanceData[InstanceID-1], 0, ACC_INSTANCE_DATA_SIZE);

               InstanceData[InstanceID-1].RangeLength = TI_ACC_RANGE_VALUE_LENGTH;

               /* Return the DIS Instance ID.                           */
               ret_val                                = (int)InstanceID;
            }  

            /* UnLock the previously locked Bluetooth Stack.            */
            BSC_UnLockBluetoothStack(BluetoothStackID);
         }
         else
            ret_val = ACC_ERROR_INSUFFICIENT_RESOURCES;
      }
      else
         ret_val = ACC_ERROR_SERVICE_ALREADY_REGISTERED;
   }
   else
      ret_val = ACC_ERROR_INVALID_PARAMETER;

   /* Finally return the result to the caller.                          */
   return(ret_val);
}

   /* The following function is responsible for closing a previously ACC*/
   /* Server.  The first parameter is the Bluetooth Stack ID on which to*/
   /* close the server.  The second parameter is the InstanceID that was*/
   /* returned from a successfull call to ACC_Initialize_Service().     */
   /* This function returns a zero if successful or a negative return   */
   /* error code if an error occurs.                                    */
int BTPSAPI ACC_Cleanup_Service(unsigned int BluetoothStackID, unsigned int InstanceID)
{
   int                   ret_val;
   ACCServerInstance_t *ServiceInstance;

   /* Make sure the parameters passed to us are semi-valid.             */
   if(InstanceID)
   {
      /* Acquire the specified DIS Instance.                            */
      if((ServiceInstance = AcquireServiceInstance(BluetoothStackID, &InstanceID)) != NULL)
      {
         /* Verify that the service is actually registered.             */
         if(ServiceInstance->ServiceID)
         {
            /* Call GATT to un-register the service.                    */
            GATT_Un_Register_Service(BluetoothStackID, ServiceInstance->ServiceID);

            /* Mark the instance entry as being free.                   */
            BTPS_MemInitialize(ServiceInstance, 0, ACC_SERVER_INSTANCE_DATA_SIZE);

            /* Clear the instance data.                                 */
            BTPS_MemInitialize(InstanceData, 0, ACC_INSTANCE_DATA_SIZE);

            /* return success to the caller.                            */
            ret_val = 0;
         }
         else
            ret_val = ACC_ERROR_INVALID_PARAMETER;

         /* UnLock the previously locked Bluetooth Stack.               */
         BSC_UnLockBluetoothStack(BluetoothStackID);
      }
      else
         ret_val = ACC_ERROR_INVALID_INSTANCE_ID;
   }
   else
      ret_val = ACC_ERROR_INVALID_PARAMETER;

   /* Finally return the result to the caller.                          */
   return(ret_val);
}

   /* The following function is responsible for setting the             */
   /* Accelerometer Range Characteristic on the specified Accelerometer */
   /* Service instance.  The first parameter is the Bluetooth Stack ID  */
   /* of the Bluetooth Device.  The second parameter is the InstanceID  */
   /* returned from a successful call to ACC_Initialize_Server().  The  */
   /* final parameter is the Range value to set as the current          */
   /* Accelerometer Range for the specified Accelerometer Service       */
   /* Instance.  This function returns a zero if successful or a        */
   /* negative return error code if an error occurs.                    */
int BTPSAPI ACC_Set_Accelerometer_Range(unsigned int BluetoothStackID, unsigned int InstanceID, Word_t AccelerometerRange)
{
   int                  ret_val;
   ACCServerInstance_t *ServiceInstance;

   /* Make sure the parameters passed to us are semi-valid.             */
   if(InstanceID)
   {
      /* Acquire the specified KFS Instance.                            */
      if((ServiceInstance = AcquireServiceInstance(BluetoothStackID, &InstanceID)) != NULL)
      {
         /* Assign the requested range.                                 */
         InstanceData[InstanceID-1].RangeLength = TI_ACC_RANGE_VALUE_LENGTH;
         ASSIGN_HOST_WORD_TO_LITTLE_ENDIAN_UNALIGNED_WORD(&(InstanceData[InstanceID-1].Range), AccelerometerRange);

         /* UnLock the previously locked Bluetooth Stack.               */
         BSC_UnLockBluetoothStack(ServiceInstance->BluetoothStackID);
      }
      else
         ret_val = ACC_ERROR_INVALID_INSTANCE_ID;
   }
   else
      ret_val = ACC_ERROR_INVALID_PARAMETER;

   /* Finally return the result to the caller.                          */
   return(ret_val);
}

   /* The following function is responsible for querying the current    */
   /* Accelerometer Range Characteristic on the specified Accelerometer */
   /* Service instance.  The first parameter is the Bluetooth Stack ID  */
   /* of the Bluetooth Device.  The second parameter is the InstanceID  */
   /* returned from a successful call to ACC_Initialize_Server().  The  */
   /* final parameter is a pointer to return the current Accelerometer  */
   /* Range for the specified ACC Service Instance.  This function      */
   /* returns a zero if successful or a negative return error code if an*/
   /* error occurs.                                                     */
int BTPSAPI ACC_Query_Accelerometer_Range(unsigned int BluetoothStackID, unsigned int InstanceID, Word_t *AccelerometerRange)
{
   int                  ret_val;
   ACCServerInstance_t *ServiceInstance;

   /* Make sure the parameters passed to us are semi-valid.             */
   if((InstanceID) && (AccelerometerRange))
   {
      /* Acquire the specified KFS Instance.                            */
      if((ServiceInstance = AcquireServiceInstance(BluetoothStackID, &InstanceID)) != NULL)
      {
         /* Return the current range for the service instance.          */
         *AccelerometerRange = READ_UNALIGNED_WORD_LITTLE_ENDIAN(&(InstanceData[InstanceID-1].Range));

         /* UnLock the previously locked Bluetooth Stack.               */
         BSC_UnLockBluetoothStack(ServiceInstance->BluetoothStackID);
      }
      else
         ret_val = ACC_ERROR_INVALID_INSTANCE_ID;
   }
   else
      ret_val = ACC_ERROR_INVALID_PARAMETER;

   /* Finally return the result to the caller.                          */
   return(ret_val);
}

   /* The following function is responsible for responding to a ACC Read*/
   /* Client Configuration Request.  The first parameter is the         */
   /* Bluetooth Stack ID of the Bluetooth Device.  The second parameter */
   /* is the InstanceID returned from a successful call to              */
   /* ACC_Initialize_Server().  The third is the Transaction ID of the  */
   /* request.  The fourth and the final parameter contains the value to*/
   /* send to the remote device.  This function returns a zero if       */
   /* successful or a negative return error code if an error occurs.    */
int BTPSAPI ACC_Read_Client_Configuration_Response(unsigned int BluetoothStackID, unsigned int InstanceID, unsigned int TransactionID, Boolean_t NotificationsEnabled)
{
   int                  ret_val;
   Word_t               ValueLength;
   NonAlignedWord_t     ClientConfiguration;
   ACCServerInstance_t *ServiceInstance;

   /* Make sure the parameters passed to us are semi-valid.             */
   if((InstanceID) && (TransactionID))
   {
      /* Acquire the specified KFS Instance.                            */
      if((ServiceInstance = AcquireServiceInstance(BluetoothStackID, &InstanceID)) != NULL)
      {
         /* Format the Read Response.                                   */
         ValueLength = NON_ALIGNED_WORD_SIZE;
         ASSIGN_HOST_WORD_TO_LITTLE_ENDIAN_UNALIGNED_WORD(&ClientConfiguration, ((NotificationsEnabled)?GATT_CLIENT_CONFIGURATION_CHARACTERISTIC_NOTIFY_ENABLE:0));

         /* Send the response.                                          */
         ret_val = GATT_Read_Response(ServiceInstance->BluetoothStackID, TransactionID, (unsigned int)ValueLength, (Byte_t *)&ClientConfiguration); 

         /* UnLock the previously locked Bluetooth Stack.               */
         BSC_UnLockBluetoothStack(ServiceInstance->BluetoothStackID);
      }
      else
         ret_val = ACC_ERROR_INVALID_INSTANCE_ID;
   }
   else
      ret_val = ACC_ERROR_INVALID_PARAMETER;

   /* Finally return the result to the caller.                          */
   return(ret_val);
}

   /* The following function is responsible for responding to a ACC     */
   /* Enable Read Request.  The first parameter is the Bluetooth Stack  */
   /* ID of the Bluetooth Device.  The second parameter is the          */
   /* InstanceID returned from a successful call to                     */
   /* ACC_Initialize_Server().  The third is the Transaction ID of the  */
   /* request.  The fourth and the final parameter contains the         */
   /* Accelerometer Enable Value.  This function returns a zero if      */
   /* successful or a negative return error code if an error occurs.    */
int BTPSAPI ACCEL_Enable_Read_Request_Response(unsigned int BluetoothStackID, unsigned int InstanceID, unsigned int TransactionID, Boolean_t AccelerometerEnabled)
{
   int                  ret_val;
   NonAlignedByte_t     AccelEnable;
   ACCServerInstance_t *ServiceInstance;

   /* Make sure the parameters passed to us are semi-valid.             */
   if((InstanceID) && (TransactionID))
   {
      /* Acquire the specified KFS Instance.                            */
      if((ServiceInstance = AcquireServiceInstance(BluetoothStackID, &InstanceID)) != NULL)
      {
         /* Format the Read Response.                                   */
         ASSIGN_HOST_BYTE_TO_LITTLE_ENDIAN_UNALIGNED_BYTE(&AccelEnable, ((AccelerometerEnabled)?TI_ACC_ENABLE_ACCELEROMETER_ENABLE:TI_ACC_ENABLE_ACCELEROMETER_DISABLED));

         /* Send the response.                                          */
         ret_val = GATT_Read_Response(ServiceInstance->BluetoothStackID, TransactionID, (unsigned int)NON_ALIGNED_BYTE_SIZE, (Byte_t *)&AccelEnable); 

         /* UnLock the previously locked Bluetooth Stack.               */
         BSC_UnLockBluetoothStack(ServiceInstance->BluetoothStackID);
      }
      else
         ret_val = ACC_ERROR_INVALID_INSTANCE_ID;
   }
   else
      ret_val = ACC_ERROR_INVALID_PARAMETER;

   /* Finally return the result to the caller.                          */
   return(ret_val);
}

   /* The following function is responsible for responding to a ACC     */
   /* Enable Update Request.  The first parameter is the Bluetooth Stack*/
   /* ID of the Bluetooth Device.  The second parameter is the          */
   /* InstanceID returned from a successful call to                     */
   /* ACC_Initialize_Server().  The third is the Transaction ID of the  */
   /* request.  The final parameter is used to either accept or reject  */
   /* the update request.  This function returns a zero if successful or*/
   /* a negative return error code if an error occurs.                  */
   /* * NOTE * To accept the update request the ErrorCode parameter     */
   /*          should be set to 0.                                      */
   /* * NOTE * To reject the update request set ErrorCode to a non-zero */
   /*          error code that is to be returned in the error response. */
int BTPSAPI ACCEL_Enable_Update_Request_Response(unsigned int BluetoothStackID, unsigned int InstanceID, unsigned int TransactionID, Byte_t ErrorCode)
{
   int                  ret_val;
   ACCServerInstance_t *ServiceInstance;

   /* Make sure the parameters passed to us are semi-valid.             */
   if((InstanceID) && (TransactionID))
   {
      /* Acquire the specified KFS Instance.                            */
      if((ServiceInstance = AcquireServiceInstance(BluetoothStackID, &InstanceID)) != NULL)
      {
         /* Respond to the update request with the correct API.         */
         if(!ErrorCode)
            ret_val = GATT_Write_Response(ServiceInstance->BluetoothStackID, TransactionID);
         else
            ret_val = GATT_Error_Response(ServiceInstance->BluetoothStackID, TransactionID, (Word_t)ACC_ENABLE_VALUE_ATTRIBUTE_OFFSET, ErrorCode);

         /* UnLock the previously locked Bluetooth Stack.               */
         BSC_UnLockBluetoothStack(ServiceInstance->BluetoothStackID);
      }
      else
         ret_val = ACC_ERROR_INVALID_INSTANCE_ID;
   }
   else
      ret_val = ACC_ERROR_INVALID_PARAMETER;

   /* Finally return the result to the caller.                          */
   return(ret_val);
}

   /* The following function is responsible for sending a Key Press     */
   /* notification to a specified remote device.  The first parameter is*/
   /* the Bluetooth Stack ID of the Bluetooth Device.  The second       */
   /* parameter is the InstanceID returned from a successful call to    */
   /* KFS_Initialize_Server().  The third parameter is the ConnectionID */
   /* of the remote device to send the notification to.  The fourth and */
   /* final parameters specify the axis that is to be notified and the  */
   /* axis value.  This function returns a zero if successful or a      */
   /* negative return error code if an error occurs.                    */
int BTPSAPI ACC_Notify_Axis(unsigned int BluetoothStackID, unsigned int InstanceID, unsigned int ConnectionID, ACC_Characteristic_Type_t CharacteristicType, Byte_t Value)
{
   int                  ret_val;
   Word_t               AttributeOffset;
   NonAlignedByte_t     AxisValue;
   ACCServerInstance_t *ServiceInstance;

   /* Make sure the parameters passed to us are semi-valid.             */
   if((InstanceID) && (ConnectionID) && ((CharacteristicType == ctXAxis) || (CharacteristicType == ctYAxis) || (CharacteristicType == ctZAxis)))
   {
      /* Acquire the specified KFS Instance.                            */
      if((ServiceInstance = AcquireServiceInstance(BluetoothStackID, &InstanceID)) != NULL)
      {
         /* Convert the Characteristic Type to an Attribute Offset.     */
         AttributeOffset = GetAxisOffset(CharacteristicType, FALSE);

         /* Format the Value.                                           */
         ASSIGN_HOST_BYTE_TO_LITTLE_ENDIAN_UNALIGNED_BYTE(&AxisValue, Value);

         /* Send the notification.                                      */
         ret_val = GATT_Handle_Value_Notification(ServiceInstance->BluetoothStackID, ServiceInstance->ServiceID, ConnectionID, AttributeOffset, (unsigned int)TI_ACC_AXIS_VALUE_LENGTH, (Byte_t *)&AxisValue); 
         if(ret_val > 0)
            ret_val = 0;

         /* UnLock the previously locked Bluetooth Stack.               */
         BSC_UnLockBluetoothStack(ServiceInstance->BluetoothStackID);
      }
      else
         ret_val = ACC_ERROR_INVALID_INSTANCE_ID;
   }
   else
      ret_val = ACC_ERROR_INVALID_PARAMETER;

   /* Finally return the result to the caller.                          */
   return(ret_val);
}


