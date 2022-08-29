/*****< kfs.c >****************************************************************/
/*      Copyright 2011 - 2014 Stonestreet One.                                */
/*      All Rights Reserved.                                                  */
/*                                                                            */
/*  KFS - Bluetooth Stack Key Fob Service Implementation (GATT Based - TI     */
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
#include "SS1BTKFS.h"       /* Bluetooth KFS API Prototypes/Constants.        */

#include "BTPSKRNL.h"       /* BTPS Kernel Prototypes/Constants.              */
#include "KFS.h"            /* Bluetooth KFS Prototypes/Constants.            */

   /* The following controls the number of supported KFS instances.     */
#define KEY_FOB_MAXIMUM_SUPPORTED_INSTANCES              (1)

   /* The following structure defines the structure of a generic KFS    */
   /* Characteristic Value.                                             */
   /* * NOTE * The Data member is a pointer to a binary blob that must  */
   /*          be formatted such that if the characteristic value is    */
   /*          read, we can internally respond with the data directly   */
   /*          using this pointer.                                      */
typedef struct _tagKFSCharacteristicValue_t
{
   Word_t  Characteristic_Length;
   Byte_t *Data;
} KFSCharacteristicValue_t;

#define KFS_CHARACTERISTIC_VALUE_DATA_SIZE               (sizeof(KFSCharacteristicValue_t))

   /* The following structure defines the KFS Instance data this unique */
   /* per KFS service instance.                                         */
typedef struct _tagKFS_Instance_Data_t
{
   KFSCharacteristicValue_t CharacteristicUserDescription;
} KFS_Instance_Data_t;

#define KFS_INSTANCE_DATA_SIZE                           (sizeof(KFS_Instance_Data_t))

   /* The following define the various instance data tags.              */
#define KFS_USER_DESCRIPTION_INSTANCE_TAG                (BTPS_STRUCTURE_OFFSET(KFS_Instance_Data_t, CharacteristicUserDescription)) 

   /* The Key Fob Service Service Declaration UUID.                     */
static BTPSCONST GATT_Primary_Service_16_Entry_t KFS_Service_UUID = 
{
   TI_KFS_SERVICE_BLUETOOTH_UUID_CONSTANT
}; 

   /* The Key Pressed Characteristic Declaration.                       */
static BTPSCONST GATT_Characteristic_Declaration_16_Entry_t KFS_Key_Pressed_Declaration =
{
   GATT_CHARACTERISTIC_PROPERTIES_NOTIFY,
   TI_KFS_KEYPRESSED_CHARACTERISTIC_BLUETOOTH_UUID_CONSTANT
};

   /* The Key Pressed Characterist Value.                               */
static BTPSCONST GATT_Characteristic_Value_16_Entry_t  KFS_Key_Pressed_Value = 
{
   TI_KFS_KEYPRESSED_CHARACTERISTIC_BLUETOOTH_UUID_CONSTANT,
   0,
   NULL
};

   /* The Key Pressed Client Characteristic Descriptor.                 */
static BTPSCONST GATT_Characteristic_Descriptor_16_Entry_t KFS_Client_Characteristic_Configuration_Value = 
{
   {0x02, 0x29},
   0,
   NULL
};

   /* The Key Pressed Characteristic User Description Descriptor.       */
static GATT_Characteristic_Descriptor_16_Entry_t KFS_Characteristic_User_Description_Value = 
{
   {0x01, 0x29},
   0,
   NULL
};

   /* The following defines the Key Fob Service that is registered with */
   /* the GATT_Register_Service function call.                          */
   /* * NOTE * This array will be registered with GATT in the call to   */
   /*          GATT_Register_Service.                                   */
BTPSCONST GATT_Service_Attribute_Entry_t Key_Fob_Service[] = 
{
   {GATT_ATTRIBUTE_FLAGS_READABLE,          aetPrimaryService16,            (Byte_t *)&KFS_Service_UUID},                              /* KFS Primary Service Declaration.            */
   {GATT_ATTRIBUTE_FLAGS_READABLE,          aetCharacteristicDeclaration16, (Byte_t *)&KFS_Key_Pressed_Declaration},                   /* KFS Key Pressed Characteristic Declaration. */
   {0,                                      aetCharacteristicValue16,       (Byte_t *)&KFS_Key_Pressed_Value},                         /* KFS Key Pressed Value.                      */
   {GATT_ATTRIBUTE_FLAGS_READABLE_WRITABLE, aetCharacteristicDescriptor16,  (Byte_t *)&KFS_Client_Characteristic_Configuration_Value}, /* KFS Client Characteristic Descriptor Value. */
   {GATT_ATTRIBUTE_FLAGS_READABLE,          aetCharacteristicDescriptor16,  (Byte_t *)&KFS_Characteristic_User_Description_Value},     /* KFS Characteristic User Description Value.  */
};   

#define KEY_FOB_SERVICE_ATTRIBUTE_COUNT                      (sizeof(Key_Fob_Service)/sizeof(GATT_Service_Attribute_Entry_t))

#define KFS_KEY_PRESS_CHARACTERISTIC_OFFSET                 0x02
#define KFS_CLIENT_CONFIGURATION_DESCRIPTOR_OFFSET          0x03
#define KFS_USER_DESCRIPTION_OFFSET                         0x04

   /* The following defines the KFS GATT Service Flags MASK that should */
   /* be passed into GATT_Register_Service when the KFS Service is      */
   /* registered.                                                       */
#define KFS_SERVICE_FLAGS                                   (GATT_SERVICE_FLAGS_LE_SERVICE|GATT_SERVICE_FLAGS_BR_EDR_SERVICE)

   /*********************************************************************/
   /**                    END OF SERVICE TABLE                         **/
   /*********************************************************************/

   /* DIS Service Instance Block.  This structure contains All          */
   /* information associated with a specific Bluetooth Stack ID (member */
   /* is present in this structure).                                    */
typedef struct _tagKFSServerInstance_t
{
   unsigned int         BluetoothStackID;
   unsigned int         ServiceID;
   KFS_Event_Callback_t EventCallback;
   unsigned long        CallbackParameter;
} KFSServerInstance_t;

#define KFS_SERVER_INSTANCE_DATA_SIZE                    (sizeof(KFSServerInstance_t))

   /* Internal Variables to this Module (Remember that all variables    */
   /* declared static are initialized to 0 automatically by the         */
   /* compiler as part of standard C/C++).                              */

static KFS_Instance_Data_t InstanceData[KEY_FOB_MAXIMUM_SUPPORTED_INSTANCES];
                                            /* Variable which holds all */
                                            /* data that is unique for  */
                                            /* each service instance.   */

static KFSServerInstance_t InstanceList[KEY_FOB_MAXIMUM_SUPPORTED_INSTANCES];    
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
static KFSServerInstance_t *AcquireServiceInstance(unsigned int BluetoothStackID, unsigned int *InstanceID);
static int DecodeClientConfigurationValue(unsigned int BufferLength, Byte_t *Buffer, Boolean_t *NotificationsEnabled);

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

   for(Index=0;Index<KEY_FOB_MAXIMUM_SUPPORTED_INSTANCES;Index++)
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
static KFSServerInstance_t *AcquireServiceInstance(unsigned int BluetoothStackID, unsigned int *InstanceID)
{
   unsigned int          LocalInstanceID;
   unsigned int          Index;
   KFSServerInstance_t *ret_val = NULL;

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
         if((LocalInstanceID) && (LocalInstanceID <= KEY_FOB_MAXIMUM_SUPPORTED_INSTANCES))
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
               for(Index=0;Index<KEY_FOB_MAXIMUM_SUPPORTED_INSTANCES;Index++)
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
   int    ret_val = KFS_ERROR_INVALID_PARAMETER;
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
         ret_val = KFS_ERROR_INVALID_PARAMETER;
   }

   /* Finally return the result to the caller.                          */
   return(ret_val);
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
   KFS_Event_Data_t                           EventData;
   KFSServerInstance_t                       *ServiceInstance;
   
   union
   {
      KFS_Read_Client_Configuration_Data_t    ReadClientData;
      KFS_Client_Configuration_Update_Data_t  ClientUpdateData;
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
                  if(AttributeOffset == KFS_USER_DESCRIPTION_OFFSET)
                  {
                     /* Verify that the offset is in range.             */
                     if(GATT_ServerEventData->Event_Data.GATT_Read_Request_Data->AttributeValueOffset <= InstanceData[InstanceID-1].CharacteristicUserDescription.Characteristic_Length)
                     {
                        /* Calculate the length of the data that we are    */
                        /* going to return.                                */
                        ValueLength = (Word_t)(InstanceData[InstanceID-1].CharacteristicUserDescription.Characteristic_Length - GATT_ServerEventData->Event_Data.GATT_Read_Request_Data->AttributeValueOffset);
   
                        if(InstanceData[InstanceID-1].CharacteristicUserDescription.Data)
                           Value = &(InstanceData[InstanceID-1].CharacteristicUserDescription.Data[GATT_ServerEventData->Event_Data.GATT_Read_Request_Data->AttributeValueOffset]);
                        else
                        {
                           Value       = NULL;
                           ValueLength = 0;
                        }
   
                        /* Respond with the data.                          */
                        GATT_Read_Response(BluetoothStackID, TransactionID, (unsigned int)ValueLength, Value);
                     }
                     else
                        GATT_Error_Response(BluetoothStackID, TransactionID, AttributeOffset, ATT_PROTOCOL_ERROR_CODE_INVALID_OFFSET);
                  }
                  else
                  {
                     /* This is a read of the Client Characteristic     */
                     /* Configuration Descriptor so we must dispatch    */
                     /* this to the application to handle.              */
                     EventBuffer.ReadClientData.ConnectionID                 = GATT_ServerEventData->Event_Data.GATT_Read_Request_Data->ConnectionID;
                     EventBuffer.ReadClientData.ConnectionType               = GATT_ServerEventData->Event_Data.GATT_Read_Request_Data->ConnectionType;
                     EventBuffer.ReadClientData.RemoteDevice                 = GATT_ServerEventData->Event_Data.GATT_Read_Request_Data->RemoteDevice;
                     EventBuffer.ReadClientData.TransactionID                = TransactionID;
                     EventBuffer.ReadClientData.InstanceID                   = InstanceID;
                                                                             
                     EventData.Event_Data_Type                               = etKFS_Server_Read_Client_Configuration_Request;
                     EventData.Event_Data_Size                               = KFS_READ_CLIENT_CONFIGURATION_DATA_SIZE;
                     EventData.Event_Data.KFS_Read_Client_Configuration_Data = &(EventBuffer.ReadClientData);

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
                     /* Format the Client Configuration Update Event.   */
                     EventBuffer.ClientUpdateData.ConnectionID                 = GATT_ServerEventData->Event_Data.GATT_Write_Request_Data->ConnectionID;
                     EventBuffer.ClientUpdateData.ConnectionType               = GATT_ServerEventData->Event_Data.GATT_Write_Request_Data->ConnectionType;
                     EventBuffer.ClientUpdateData.RemoteDevice                 = GATT_ServerEventData->Event_Data.GATT_Write_Request_Data->RemoteDevice;
                     EventBuffer.ClientUpdateData.InstanceID                   = InstanceID;
                                                                               
                     EventData.Event_Data_Type                                 = etKFS_Server_Client_Configuration_Update;
                     EventData.Event_Data_Size                                 = KFS_CLIENT_CONFIGURATION_UPDATE_DATA_SIZE;
                     EventData.Event_Data.KFS_Client_Configuration_Update_Data = &(EventBuffer.ClientUpdateData);

                     /* Attempt to decode the request Client            */
                     /* Configuration.                                  */
                     if(!DecodeClientConfigurationValue(ValueLength, Value, &(EventBuffer.ClientUpdateData.NotificationsEnabled)))
                     {
                        /* Go ahead and accept the write request since  */
                        /* we have decode the Client Configuration Value*/
                        /* successfully.                                */
                        GATT_Write_Response(BluetoothStackID, TransactionID);

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
                        GATT_Error_Response(BluetoothStackID, TransactionID, AttributeOffset, ATT_PROTOCOL_ERROR_CODE_REQUEST_NOT_SUPPORTED);
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
int InitializeKFSModule(void)
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
void CleanupKFSModule(Boolean_t ForceCleanup)
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

   /* The following function is responsible for opening a KFS Server.   */
   /* The first parameter is the Bluetooth Stack ID on which to open the*/
   /* server.  The final parameter is a pointer to store the GATT       */
   /* Service ID of the registered KFS service.  This can be used to    */
   /* include the service registered by this call.  This function       */
   /* returns the positive, non-zero, Instance ID or a negative error   */
   /* code.                                                             */
int BTPSAPI KFS_Initialize_Service(unsigned int BluetoothStackID, KFS_Event_Callback_t EventCallback, unsigned long CallbackParameter, unsigned int *ServiceID)
{
   int                            ret_val;
   unsigned int                   InstanceID;
   KFSServerInstance_t           *ServiceInstance;
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
            ret_val = GATT_Register_Service(BluetoothStackID, KFS_SERVICE_FLAGS, KEY_FOB_SERVICE_ATTRIBUTE_COUNT, (GATT_Service_Attribute_Entry_t *)Key_Fob_Service, &ServiceHandleGroup, GATT_ServerEventCallback, InstanceID);
            if(ret_val > 0)
            {
               /* Save the Instance information.                        */
               ServiceInstance->BluetoothStackID              = BluetoothStackID;
               ServiceInstance->ServiceID                     = (unsigned int)ret_val;
               ServiceInstance->EventCallback                 = EventCallback;
               ServiceInstance->CallbackParameter             = CallbackParameter;

               if(ServiceID)
                  *ServiceID                                  = (unsigned int)ret_val;

               /* Intilize the Instance Data for this instance.         */
               BTPS_MemInitialize(&InstanceData[InstanceID-1], 0, KFS_INSTANCE_DATA_SIZE);

               /* Return the DIS Instance ID.                           */
               ret_val                                        = (int)InstanceID;
            }  

            /* UnLock the previously locked Bluetooth Stack.            */
            BSC_UnLockBluetoothStack(BluetoothStackID);
         }
         else
            ret_val = KFS_ERROR_INSUFFICIENT_RESOURCES;
      }
      else
         ret_val = KFS_ERROR_SERVICE_ALREADY_REGISTERED;
   }
   else
      ret_val = KFS_ERROR_INVALID_PARAMETER;

   /* Finally return the result to the caller.                          */
   return(ret_val);
}

   /* The following function is responsible for closing a previously KFS*/
   /* Server.  The first parameter is the Bluetooth Stack ID on which to*/
   /* close the server.  The second parameter is the InstanceID that was*/
   /* returned from a successfull call to KFS_Initialize_Service().     */
   /* This function returns a zero if successful or a negative return   */
   /* error code if an error occurs.                                    */
int BTPSAPI KFS_Cleanup_Service(unsigned int BluetoothStackID, unsigned int InstanceID)
{
   int                   ret_val;
   KFSServerInstance_t *ServiceInstance;

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

            /* Free any of the dynamically allocated values.            */
            if(InstanceData[InstanceID-1].CharacteristicUserDescription.Data)
               BTPS_FreeMemory(InstanceData[InstanceID-1].CharacteristicUserDescription.Data);

            /* Mark the instance entry as being free.                   */
            BTPS_MemInitialize(ServiceInstance, 0, KFS_SERVER_INSTANCE_DATA_SIZE);

            /* Clear the instance data.                                 */
            BTPS_MemInitialize(InstanceData, 0, KFS_INSTANCE_DATA_SIZE);

            /* return success to the caller.                            */
            ret_val = 0;
         }
         else
            ret_val = KFS_ERROR_INVALID_PARAMETER;

         /* UnLock the previously locked Bluetooth Stack.               */
         BSC_UnLockBluetoothStack(BluetoothStackID);
      }
      else
         ret_val = KFS_ERROR_INVALID_INSTANCE_ID;
   }
   else
      ret_val = KFS_ERROR_INVALID_PARAMETER;

   /* Finally return the result to the caller.                          */
   return(ret_val);
}

   /* The following function is responsible for setting the Key Pressed */
   /* User Characteristic Descriptor on the specified Key Fob Service   */
   /* instance.  The first parameter is the Bluetooth Stack ID of the   */
   /* Bluetooth Device.  The second parameter is the InstanceID returned*/
   /* from a successful call to KFS_Initialize_Server().  The final     */
   /* parameter is the User Description to set as the current User      */
   /* Description for the specified Key Fob Service Instance.  The User */
   /* Description parameter must be a pointer to a NULL terminated ASCII*/
   /* String of at most KFS_MAXIMUM_SUPPORTED_STRING (not counting the  */
   /* trailing NULL terminator).  This function returns a zero if       */
   /* successful or a negative return error code if an error occurs.    */
int BTPSAPI KFS_Set_Key_Press_User_Description(unsigned int BluetoothStackID, unsigned int InstanceID, char *UserDescription)
{
   int                  ret_val;
   unsigned int         StringLength;
   KFSServerInstance_t *ServiceInstance;

   /* Make sure the parameters passed to us are semi-valid.             */
   if((InstanceID) && (UserDescription) && ((StringLength = BTPS_StringLength(UserDescription)) <= KFS_MAXIMUM_SUPPORTED_STRING))
   {
      /* Acquire the specified DIS Instance.                            */
      if((ServiceInstance = AcquireServiceInstance(BluetoothStackID, &InstanceID)) != NULL)
      {
         /* Free any previous buffer allocated.                         */
         if(InstanceData[InstanceID-1].CharacteristicUserDescription.Data)
         {
            BTPS_FreeMemory(InstanceData[InstanceID-1].CharacteristicUserDescription.Data);
            InstanceData[InstanceID-1].CharacteristicUserDescription.Data = NULL;
         }

         /* Attempt to allocate memory for the Manufacturer Name string.*/
         if((InstanceData[InstanceID-1].CharacteristicUserDescription.Data = BTPS_AllocateMemory(StringLength)) != NULL)
         {
            /* Set the length and copy the data into the Manufacturer   */
            /* Name.                                                    */
            InstanceData[InstanceID-1].CharacteristicUserDescription.Characteristic_Length = (Word_t)StringLength;
            BTPS_MemCopy(InstanceData[InstanceID-1].CharacteristicUserDescription.Data, UserDescription, StringLength);

            /* Return success to the caller.                            */
            ret_val = 0;
         }
         else
            ret_val = KFS_ERROR_INSUFFICIENT_RESOURCES;

         /* UnLock the previously locked Bluetooth Stack.               */
         BSC_UnLockBluetoothStack(ServiceInstance->BluetoothStackID);
      }
      else
         ret_val = KFS_ERROR_INVALID_INSTANCE_ID;
   }
   else
      ret_val = KFS_ERROR_INVALID_PARAMETER;

   /* Finally return the result to the caller.                          */
   return(ret_val);
}
   
   /* The following function is responsible for querying the current Key*/
   /* Pressed User Characteristic Descriptor on the specified Key Fob   */
   /* Service instance.  The first parameter is the Bluetooth Stack ID  */
   /* of the Bluetooth Device.  The second parameter is the InstanceID  */
   /* returned from a successful call to KFS_Initialize_Server().  The  */
   /* final parameter is a pointer to a structure to return the current */
   /* User Description for the specified KFS Service Instance.  The User*/
   /* Description Length should be at least                             */
   /* (KFS_MAXIMUM_SUPPORTED_STRING+1) to hold the Maximum allowable    */
   /* string (plus a single character to hold the NULL terminator) This */
   /* function returns a zero if successful or a negative return error  */
   /* code if an error occurs.                                          */
int BTPSAPI KFS_Query_Key_Press_User_Description(unsigned int BluetoothStackID, unsigned int InstanceID, char *UserDescription)
{
   int                  ret_val;
   Word_t               Length;
   KFSServerInstance_t *ServiceInstance;

   /* Make sure the parameters passed to us are semi-valid.             */
   if((InstanceID) && (UserDescription))
   {
      /* Acquire the specified DIS Instance.                            */
      if((ServiceInstance = AcquireServiceInstance(BluetoothStackID, &InstanceID)) != NULL)
      {  
         /* Get the current Manufacturer Name.                          */
         Length = InstanceData[InstanceID-1].CharacteristicUserDescription.Characteristic_Length;
         if(InstanceData[InstanceID-1].CharacteristicUserDescription.Data)
            BTPS_MemCopy(UserDescription, InstanceData[InstanceID-1].CharacteristicUserDescription.Data, Length);

         UserDescription[Length] = 0;
         
         /* Return success to the caller.                               */
         ret_val = 0;

         /* UnLock the previously locked Bluetooth Stack.               */
         BSC_UnLockBluetoothStack(ServiceInstance->BluetoothStackID);
      }
      else
         ret_val = KFS_ERROR_INVALID_INSTANCE_ID;
   }
   else
      ret_val = KFS_ERROR_INVALID_PARAMETER;

   /* Finally return the result to the caller.                          */
   return(ret_val);
}

   /* The following function is responsible for responding to a KFS Read*/
   /* Client Configuration Request.  The first parameter is the         */
   /* Bluetooth Stack ID of the Bluetooth Device.  The second parameter */
   /* is the InstanceID returned from a successful call to              */
   /* HTS_Initialize_Server().  The third is the Transaction ID of the  */
   /* request.  The final parameter contains the Client Configuration to*/
   /* send to the remote device.  This function returns a zero if       */
   /* successful or a negative return error code if an error occurs.    */
int BTPSAPI KFS_Read_Client_Configuration_Response(unsigned int BluetoothStackID, unsigned int InstanceID, unsigned int TransactionID, Boolean_t NotificationsEnabled)
{
   int                  ret_val;
   Word_t               ValueLength;
   NonAlignedWord_t     ClientConfiguration;
   KFSServerInstance_t *ServiceInstance;

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
         ret_val = KFS_ERROR_INVALID_INSTANCE_ID;
   }
   else
      ret_val = KFS_ERROR_INVALID_PARAMETER;

   /* Finally return the result to the caller.                          */
   return(ret_val);
}
   /* The following function is responsible for sending a Key Press     */
   /* notification to a specified remote device.  The first parameter is*/
   /* the Bluetooth Stack ID of the Bluetooth Device.  The second       */
   /* parameter is the InstanceID returned from a successful call to    */
   /* KFS_Initialize_Server().  The third parameter is the ConnectionID */
   /* of the remote device to send the notification to.  The final      */
   /* parameter is the Key Press State to notify.  This function returns*/
   /* a zero if successful or a negative return error code if an error  */
   /* occurs.                                                           */
int BTPSAPI KFS_Notify_Key_Press_State(unsigned int BluetoothStackID, unsigned int InstanceID, unsigned int ConnectionID, KFS_Key_Pressed_State_t *KeyPressState)
{
   int                  ret_val;
   Byte_t               KeyPressed;
   NonAlignedByte_t     KeyPressedState;
   KFSServerInstance_t *ServiceInstance;

   /* Make sure the parameters passed to us are semi-valid.             */
   if((InstanceID) && (ConnectionID) && (KeyPressState))
   {
      /* Acquire the specified KFS Instance.                            */
      if((ServiceInstance = AcquireServiceInstance(BluetoothStackID, &InstanceID)) != NULL)
      {
         /* Format the notification.                                    */
         KeyPressed = 0;
         if(KeyPressState->KeyOnePressed)
            KeyPressed |= TI_KFS_KEY_PRESSED_BUTTON_ONE_PRESSED;

         if(KeyPressState->KeyTwoPressed)
            KeyPressed |= TI_KFS_KEY_PRESSED_BUTTON_TWO_PRESSED;

         ASSIGN_HOST_BYTE_TO_LITTLE_ENDIAN_UNALIGNED_BYTE(&KeyPressedState, KeyPressed);

         /* Send the notification.                                      */
         ret_val = GATT_Handle_Value_Notification(ServiceInstance->BluetoothStackID, ServiceInstance->ServiceID, ConnectionID, KFS_KEY_PRESS_CHARACTERISTIC_OFFSET, (unsigned int)NON_ALIGNED_BYTE_SIZE, (Byte_t *)&KeyPressedState); 
         if(ret_val > 0)
            ret_val = 0;

         /* UnLock the previously locked Bluetooth Stack.               */
         BSC_UnLockBluetoothStack(ServiceInstance->BluetoothStackID);
      }
      else
         ret_val = KFS_ERROR_INVALID_INSTANCE_ID;
   }
   else
      ret_val = KFS_ERROR_INVALID_PARAMETER;

   /* Finally return the result to the caller.                          */
   return(ret_val);
}

   /* KFS Client API.                                                   */

   /* The following function is responsible for parsing a Key Press     */
   /* State notification received from a remote KFS Server.  The first  */
   /* parameter is the length of the value returned by the remote KFS   */
   /* Server.  The second parameter is a pointer to the data returned by*/
   /* the remote KFS Server.  The final parameter is a pointer to store */
   /* the parsed Key Pressed State value.  This function returns a zero */
   /* if successful or a negative return error code if an error occurs. */
int BTPSAPI KFS_Decode_Key_Press_Notification(unsigned int ValueLength, Byte_t *Value, KFS_Key_Pressed_State_t *KeyPressState)
{
   int    ret_val;
   Byte_t KeyPressed;

   /* Make sure the parameters passed to us are semi-valid.             */
   if((ValueLength == NON_ALIGNED_BYTE_SIZE) && (Value) && (KeyPressState))
   {
      /* Read the Value.                                                */
      KeyPressed = READ_UNALIGNED_BYTE_LITTLE_ENDIAN(Value);

      /* Decode the received value.                                     */
      if(KeyPressed & TI_KFS_KEY_PRESSED_BUTTON_ONE_PRESSED)
         KeyPressState->KeyOnePressed = TRUE;
      else
         KeyPressState->KeyOnePressed = FALSE;

      if(KeyPressed & TI_KFS_KEY_PRESSED_BUTTON_TWO_PRESSED)
         KeyPressState->KeyTwoPressed = TRUE;
      else
         KeyPressState->KeyTwoPressed = FALSE;

      /* return success to the caller.                                  */
      ret_val = 0;
   }
   else
      ret_val = KFS_ERROR_INVALID_PARAMETER;

   /* Finally return the result to the caller.                          */
   return(ret_val);
}
