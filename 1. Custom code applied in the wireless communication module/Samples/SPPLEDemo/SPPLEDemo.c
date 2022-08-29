/*****< sppledemo.c >**********************************************************/
/*      Copyright 2012 - 2014 Stonestreet One.                                */
/*      All Rights Reserved.                                                  */
/*                                                                            */
/*  SPPLEDEMO - Embedded Bluetooth SPP Emulation using GATT (LE) application. */
/*                                                                            */
/*  Author:  Tim Cook                                                         */
/*                                                                            */
/*** MODIFICATION HISTORY *****************************************************/
/*                                                                            */
/*   mm/dd/yy  F. Lastname    Description of Modification                     */
/*   --------  -----------    ------------------------------------------------*/
/*   04/16/12  Tim Cook       Initial creation.                               */
/*   01/09/13  M. Buckley     Adapted use of ConnectionID and                 */
/*                            ConnectionBD_ADDR for use with multiple devices */
/******************************************************************************/
#include <stdio.h>               /* Included for sscanf.                      */
#include "Main.h"                /* Application Interface Abstraction.        */
#include "SPPLEDemo.h"           /* Application Header.                       */
#include "SS1BTPS.h"             /* Main SS1 BT Stack Header.                 */
#include "SS1BTGAT.h"            /* Main SS1 GATT Header.                     */
#include "SS1BTGAP.h"            /* Main SS1 GAP Service Header.              */
#include "BTPSKRNL.h"            /* BTPS Kernel Header.                       */

#include "HAL.h"                 /* Function for Hardware Abstraction.        */
#include "HCITRANS.h"            /* HCI Transport Prototypes/Constants.       */

#define MAX_SUPPORTED_COMMANDS                     (64)  /* Denotes the       */
                                                         /* maximum number of */
                                                         /* User Commands that*/
                                                         /* are supported by  */
                                                         /* this application. */

#define MAX_NUM_OF_PARAMETERS                       (5)  /* Denotes the max   */
                                                         /* number of         */
                                                         /* parameters a      */
                                                         /* command can have. */

#define MAX_INQUIRY_RESULTS                         (5)  /* Denotes the max   */
                                                         /* number of inquiry */
                                                         /* results.          */

#define MAX_SUPPORTED_LINK_KEYS                     (1)  /* Max supported Link*/
                                                         /* keys.             */

#define MAX_LE_CONNECTIONS                          (2)  /* Denotes the max   */
                                                         /* number of LE      */
                                                         /* connections that  */
                                                         /* are allowed at    */
                                                         /* the same time.    */

#define MAX_SIMULTANEOUS_SPP_PORTS                  (1) /* Maximum SPP Ports  */
                                                        /* that we support.   */

#define MAXIMUM_SPP_LOOPBACK_BUFFER_SIZE           (64) /* Maximum size of the*/
                                                        /* buffer used in     */
                                                        /* loopback mode.     */

#define SPP_PERFORM_MASTER_ROLE_SWITCH              (1) /* Defines if TRUE    */
                                                        /* that a role switch */
                                                        /* should be performed*/
                                                        /* for all SPP        */
                                                        /* connections.       */

#define DEFAULT_LE_IO_CAPABILITY   (licNoInputNoOutput)  /* Denotes the       */
                                                         /* default I/O       */
                                                         /* Capability that is*/
                                                         /* used with LE      */
                                                         /* Pairing.          */

#define DEFAULT_LE_MITM_PROTECTION              (TRUE)   /* Denotes the       */
                                                         /* default value used*/
                                                         /* for Man in the    */
                                                         /* Middle (MITM)     */
                                                         /* protection used   */
                                                         /* with LE Pairing.  */

#define DEFAULT_IO_CAPABILITY          (icDisplayYesNo)  /* Denotes the       */
                                                         /* default I/O       */
                                                         /* Capability that is*/
                                                         /* used with Secure  */
                                                         /* Simple Pairing.   */

#define DEFAULT_MITM_PROTECTION                  (TRUE)  /* Denotes the       */
                                                         /* default value used*/
                                                         /* for Man in the    */
                                                         /* Middle (MITM)     */
                                                         /* protection used   */
                                                         /* with Secure Simple*/
                                                         /* Pairing.          */

#define SPPLE_DATA_BUFFER_LENGTH  (BTPS_CONFIGURATION_GATT_MAXIMUM_SUPPORTED_MTU_SIZE)
                                                         /* Defines the length*/
                                                         /* of a SPPLE Data   */
                                                         /* Buffer.           */

#define SPPLE_DATA_CREDITS        (SPPLE_DATA_BUFFER_LENGTH*1) /* Defines the */
                                                         /* number of credits */
                                                         /* in an SPPLE Buffer*/

#define NO_COMMAND_ERROR                           (-1)  /* Denotes that no   */
                                                         /* command was       */
                                                         /* specified to the  */
                                                         /* parser.           */

#define INVALID_COMMAND_ERROR                      (-2)  /* Denotes that the  */
                                                         /* Command does not  */
                                                         /* exist for         */
                                                         /* processing.       */

#define EXIT_CODE                                  (-3)  /* Denotes that the  */
                                                         /* Command specified */
                                                         /* was the Exit      */
                                                         /* Command.          */

#define FUNCTION_ERROR                             (-4)  /* Denotes that an   */
                                                         /* error occurred in */
                                                         /* execution of the  */
                                                         /* Command Function. */

#define TO_MANY_PARAMS                             (-5)  /* Denotes that there*/
                                                         /* are more          */
                                                         /* parameters then   */
                                                         /* will fit in the   */
                                                         /* UserCommand.      */

#define INVALID_PARAMETERS_ERROR                   (-6)  /* Denotes that an   */
                                                         /* error occurred due*/
                                                         /* to the fact that  */
                                                         /* one or more of the*/
                                                         /* required          */
                                                         /* parameters were   */
                                                         /* invalid.          */

#define UNABLE_TO_INITIALIZE_STACK                 (-7)  /* Denotes that an   */
                                                         /* error occurred    */
                                                         /* while Initializing*/
                                                         /* the Bluetooth     */
                                                         /* Protocol Stack.   */

#define INVALID_STACK_ID_ERROR                     (-8)  /* Denotes that an   */
                                                         /* occurred due to   */
                                                         /* attempted         */
                                                         /* execution of a    */
                                                         /* Command when a    */
                                                         /* Bluetooth Protocol*/
                                                         /* Stack has not been*/
                                                         /* opened.           */

#define UNABLE_TO_REGISTER_SERVER                  (-9)  /* Denotes that an   */
                                                         /* error occurred    */
                                                         /* when trying to    */
                                                         /* create a Serial   */
                                                         /* Port Server.      */

#define EXIT_TEST_MODE                             (-10) /* Flags exit from   */
                                                         /* Test Mode.        */

#define EXIT_MODE                                  (-11) /* Flags exit from   */
                                                         /* any Mode.         */

   /* The following MACRO is used to convert an ASCII character into the*/
   /* equivalent decimal value.  The MACRO converts lower case          */
   /* characters to upper case before the conversion.                   */
#define ToInt(_x)                                  (((_x) > 0x39)?(((_x) & ~0x20)-0x37):((_x)-0x30))

   /* Determine the Name we will use for this compilation.              */
#define LE_DEMO_DEVICE_NAME                        "SPPLEDemo"

   /* Following converts a Sniff Parameter in Milliseconds to frames.   */
#define MILLISECONDS_TO_BASEBAND_SLOTS(_x)         ((_x) / (0.625))


   /* The following type definition represents the container type which */
   /* holds the mapping between Bluetooth devices (based on the BD_ADDR)*/
   /* and the Link Key (BD_ADDR <-> Link Key Mapping).                  */
typedef struct _tagLinkKeyInfo_t
{
   BD_ADDR_t  BD_ADDR;
   Link_Key_t LinkKey;
} LinkKeyInfo_t;

   /* The following type definition represents the structure which holds*/
   /* all information about the parameter, in particular the parameter  */
   /* as a string and the parameter as an unsigned int.                 */
typedef struct _tagParameter_t
{
   char     *strParam;
   SDWord_t  intParam;
} Parameter_t;

   /* The following type definition represents the structure which holds*/
   /* a list of parameters that are to be associated with a command The */
   /* NumberofParameters variable holds the value of the number of      */
   /* parameters in the list.                                           */
typedef struct _tagParameterList_t
{
   int         NumberofParameters;
   Parameter_t Params[MAX_NUM_OF_PARAMETERS];
} ParameterList_t;

   /* The following type definition represents the structure which holds*/
   /* the command and parameters to be executed.                        */
typedef struct _tagUserCommand_t
{
   char            *Command;
   ParameterList_t  Parameters;
} UserCommand_t;

   /* The following type definition represents the generic function     */
   /* pointer to be used by all commands that can be executed by the    */
   /* test program.                                                     */
typedef int (*CommandFunction_t)(ParameterList_t *TempParam);

   /* The following type definition represents the structure which holds*/
   /* information used in the interpretation and execution of Commands. */
typedef struct _tagCommandTable_t
{
   char              *CommandName;
   CommandFunction_t  CommandFunction;
} CommandTable_t;

   /* Structure used to hold all of the GAP LE Parameters.              */
typedef struct _tagGAPLE_Parameters_t
{
   GAP_LE_Connectability_Mode_t ConnectableMode;
   GAP_Discoverability_Mode_t   DiscoverabilityMode;
   GAP_LE_IO_Capability_t       IOCapability;
   Boolean_t                    MITMProtection;
   Boolean_t                    OOBDataPresent;
} GAPLE_Parameters_t;

#define GAPLE_PARAMETERS_DATA_SIZE                       (sizeof(GAPLE_Parameters_t))

   /* The following structure holds status information about a send     */
   /* process.                                                          */
typedef struct _tagSend_Info_t
{
   Boolean_t BufferFull;
   DWord_t   BytesToSend;
   DWord_t   BytesSent;
} Send_Info_t;

   /* The following defines the structure that is used to hold          */
   /* information about all open SPP Ports.                             */
typedef struct SPP_Context_Info_t
{
   unsigned int  LocalSerialPortID;
   unsigned int  ServerPortNumber;
   Word_t        Connection_Handle;
   BD_ADDR_t     BD_ADDR;
   DWord_t       SPPServerSDPHandle;
   Boolean_t     Connected;
   Send_Info_t   SendInfo;

#if MAXIMUM_SPP_LOOPBACK_BUFFER_SIZE > 0

   unsigned int  BufferLength;
   unsigned char Buffer[MAXIMUM_SPP_LOOPBACK_BUFFER_SIZE];

#endif

} SPP_Context_Info_t;

   /* The following defines the format of a SPPLE Data Buffer.          */
typedef struct _tagSPPLE_Data_Buffer_t
{
   unsigned int  InIndex;
   unsigned int  OutIndex;
   unsigned int  BytesFree;
   unsigned int  BufferSize;
   Byte_t        Buffer[SPPLE_DATA_CREDITS];
} SPPLE_Data_Buffer_t;

   /* The following structure represents the information we will store  */
   /* on a Discovered GAP Service.                                      */
typedef struct _tagGAPS_Client_Info_t
{
   Word_t DeviceNameHandle;
   Word_t DeviceAppearanceHandle;
} GAPS_Client_Info_t;

   /* The following structure holds information on known Device         */
   /* Appearance Values.                                                */
typedef struct _tagGAPS_Device_Appearance_Mapping_t
{
   Word_t  Appearance;
   char   *String;
} GAPS_Device_Appearance_Mapping_t;

   /* The following structure is used to hold a list of information     */
   /* on all paired devices.                                            */
typedef struct _tagDeviceInfo_t
{
   Byte_t                   Flags;
   Byte_t                   EncryptionKeySize;
   GAP_LE_Address_Type_t    ConnectionAddressType;
   BD_ADDR_t                ConnectionBD_ADDR;
   Long_Term_Key_t          LTK;
   Random_Number_t          Rand;
   Word_t                   EDIV;
   GAPS_Client_Info_t       GAPSClientInfo;
   SPPLE_Client_Info_t      ClientInfo;
   SPPLE_Server_Info_t      ServerInfo;
   struct _tagDeviceInfo_t *NextDeviceInfoInfoPtr;
} DeviceInfo_t;

#define DEVICE_INFO_DATA_SIZE                            (sizeof(DeviceInfo_t))

   /* Defines the bit mask flags that may be set in the DeviceInfo_t    */
   /* structure.                                                        */
#define DEVICE_INFO_FLAGS_LTK_VALID                      0x01
#define DEVICE_INFO_FLAGS_SPPLE_SERVER                   0x02
#define DEVICE_INFO_FLAGS_SERVICE_DISCOVERY_OUTSTANDING  0x04
#define DEVICE_INFO_FLAGS_LINK_ENCRYPTED                 0x08

   /* The following structure is used to hold all of the SPPLE related  */
   /* information pertaining to buffers and credits.                    */
typedef struct _tagSPPLE_Buffer_Info_t
{
   Send_Info_t         SendInfo;
   unsigned int        TransmitCredits;
   Word_t              QueuedCredits;
   SPPLE_Data_Buffer_t ReceiveBuffer;
   SPPLE_Data_Buffer_t TransmitBuffer;
} SPPLE_Buffer_Info_t;

   /* The following structure is used to hold information on a connected*/
   /* LE Device.                                                        */
typedef struct _tagLE_Context_Info_t
{
   BD_ADDR_t           ConnectionBD_ADDR;
   unsigned int        ConnectionID;
   SPPLE_Buffer_Info_t SPPLEBufferInfo;
   Boolean_t           BufferFull;
}  LE_Context_Info_t;

   /* User to represent a structure to hold a BD_ADDR return from       */
   /* BD_ADDRToStr.                                                     */
typedef char BoardStr_t[16];

                        /* The Encryption Root Key should be generated  */
                        /* in such a way as to guarantee 128 bits of    */
                        /* entropy.                                     */
static BTPSCONST Encryption_Key_t ER = {0x28, 0xBA, 0xE1, 0x37, 0x13, 0xB2, 0x20, 0x45, 0x16, 0xB2, 0x19, 0xD0, 0x80, 0xEE, 0x4A, 0x51};

                        /* The Identity Root Key should be generated    */
                        /* in such a way as to guarantee 128 bits of    */
                        /* entropy.                                     */
static BTPSCONST Encryption_Key_t IR = {0x41, 0x09, 0xA0, 0x88, 0x09, 0x6B, 0x70, 0xC0, 0x95, 0x23, 0x3C, 0x8C, 0x48, 0xFC, 0xC9, 0xFE};

                        /* The following keys can be regerenated on the */
                        /* fly using the constant IR and ER keys and    */
                        /* are used globally, for all devices.          */
static Encryption_Key_t DHK;
static Encryption_Key_t IRK;

   /* Internal Variables to this Module (Remember that all variables    */
   /* declared static are initialized to 0 automatically by the         */
   /* compiler as part of standard C/C++).                              */

static Byte_t              SPPLEBuffer[SPPLE_DATA_BUFFER_LENGTH+1];  /* Buffer that is */
                                                    /* used for Sending/Receiving      */
                                                    /* SPPLE Service Data.             */

static unsigned int        SPPLEServiceID;          /* The following holds the SPP LE  */
                                                    /* Service ID that is returned from*/
                                                    /* GATT_Register_Service().        */

static unsigned int        GAPSInstanceID;          /* Holds the Instance ID for the   */
                                                    /* GAP Service.                    */

static GAPLE_Parameters_t  LE_Parameters;           /* Holds GAP Parameters like       */
                                                    /* Discoverability, Connectability */
                                                    /* Modes.                          */

static DeviceInfo_t       *DeviceInfoList;          /* Holds the list head for the     */
                                                    /* device info list.               */

static unsigned int        BluetoothStackID;        /* Variable which holds the Handle */
                                                    /* of the opened Bluetooth Protocol*/
                                                    /* Stack.                          */

static Boolean_t           LocalDeviceIsMaster;     /* Boolean that tells if the local */
                                                    /* device is the master of the     */
                                                    /* current connection.             */


static LinkKeyInfo_t       LinkKeyInfo[MAX_SUPPORTED_LINK_KEYS]; /* Variable holds     */
                                                    /* BD_ADDR <-> Link Keys for       */
                                                    /* pairing.                        */

static GAP_IO_Capability_t IOCapability;            /* Variable which holds the        */
                                                    /* current I/O Capabilities that   */
                                                    /* are to be used for Secure Simple*/
                                                    /* Pairing.                        */

static Boolean_t           OOBSupport;              /* Variable which flags whether    */
                                                    /* or not Out of Band Secure Simple*/
                                                    /* Pairing exchange is supported.  */

static Boolean_t           MITMProtection;          /* Variable which flags whether or */
                                                    /* not Man in the Middle (MITM)    */
                                                    /* protection is to be requested   */
                                                    /* during a Secure Simple Pairing  */
                                                    /* procedure.                      */

static Boolean_t           LoopbackActive;          /* Variable which flags whether or */
                                                    /* not the application is currently*/
                                                    /* operating in Loopback Mode      */
                                                    /* (TRUE) or not (FALSE).          */

static Boolean_t           DisplayRawData;          /* Variable which flags whether or */
                                                    /* not the application is to       */
                                                    /* simply display the Raw Data     */
                                                    /* when it is received (when not   */
                                                    /* operating in Loopback Mode).    */

static Boolean_t           AutomaticReadActive;     /* Variable which flags whether or */
                                                    /* not the application is to       */
                                                    /* automatically read all data     */
                                                    /* as it is received.              */

static unsigned int        NumberCommands;          /* Variable which is used to hold  */
                                                    /* the number of Commands that are */
                                                    /* supported by this application.  */
                                                    /* Commands are added individually.*/


static SPP_Context_Info_t  SPPContextInfo[MAX_SIMULTANEOUS_SPP_PORTS];
                                                    /* Variable that contains          */
                                                    /* information about the current   */
                                                    /* open SPP Ports                  */

static LE_Context_Info_t   LEContextInfo[MAX_LE_CONNECTIONS]; /* Array that contains   */
                                                    /* the connection ID and BD_ADDR   */
                                                    /* of each connected device.       */

   /* The following is used to map from ATT Error Codes to a printable  */
   /* string.                                                           */
static char *ErrorCodeStr[] =
{
   "ATT_PROTOCOL_ERROR_CODE_NO_ERROR",
   "ATT_PROTOCOL_ERROR_CODE_INVALID_HANDLE",
   "ATT_PROTOCOL_ERROR_CODE_READ_NOT_PERMITTED",
   "ATT_PROTOCOL_ERROR_CODE_WRITE_NOT_PERMITTED",
   "ATT_PROTOCOL_ERROR_CODE_INVALID_PDU",
   "ATT_PROTOCOL_ERROR_CODE_INSUFFICIENT_AUTHENTICATION",
   "ATT_PROTOCOL_ERROR_CODE_REQUEST_NOT_SUPPORTED",
   "ATT_PROTOCOL_ERROR_CODE_INVALID_OFFSET",
   "ATT_PROTOCOL_ERROR_CODE_INSUFFICIENT_AUTHORIZATION",
   "ATT_PROTOCOL_ERROR_CODE_PREPARE_QUEUE_FULL",
   "ATT_PROTOCOL_ERROR_CODE_ATTRIBUTE_NOT_FOUND",
   "ATT_PROTOCOL_ERROR_CODE_ATTRIBUTE_NOT_LONG",
   "ATT_PROTOCOL_ERROR_CODE_INSUFFICIENT_ENCRYPTION_KEY_SIZE",
   "ATT_PROTOCOL_ERROR_CODE_INVALID_ATTRIBUTE_VALUE_LENGTH",
   "ATT_PROTOCOL_ERROR_CODE_UNLIKELY_ERROR",
   "ATT_PROTOCOL_ERROR_CODE_INSUFFICIENT_ENCRYPTION",
   "ATT_PROTOCOL_ERROR_CODE_UNSUPPORTED_GROUP_TYPE",
   "ATT_PROTOCOL_ERROR_CODE_INSUFFICIENT_RESOURCES"
};

#define NUMBER_GATT_ERROR_CODES  (sizeof(ErrorCodeStr)/sizeof(char *))

#define NUMBER_OF_APPEARANCE_MAPPINGS     (sizeof(AppearanceMappings)/sizeof(GAPS_Device_Appearance_Mapping_t))

   /* The following string table is used to map HCI Version information */
   /* to an easily displayable version string.                          */
static BTPSCONST char *HCIVersionStrings[] =
{
   "1.0b",
   "1.1",
   "1.2",
   "2.0",
   "2.1",
   "3.0",
   "4.0",
   "4.1",
   "Unknown (greater 4.1)"
} ;

#define NUM_SUPPORTED_HCI_VERSIONS              (sizeof(HCIVersionStrings)/sizeof(char *) - 1)

   /* The following string table is used to map the API I/O Capabilities*/
   /* values to an easily displayable string.                           */
static BTPSCONST char *IOCapabilitiesStrings[] =
{
   "Display Only",
   "Display Yes/No",
   "Keyboard Only",
   "No Input/Output",
   "Keyboard/Display"
} ;

   /* The following defines a data sequence that will be used to        */
   /* generate message data.                                            */
static char DataStr[]  = "01234567890123456789012345678901234567890123456789";
static int  DataStrLen = (sizeof(DataStr)-1);

   /*********************************************************************/
   /**                     SPPLE Service Table                         **/
   /*********************************************************************/

   /* The SPPLE Service Declaration UUID.                               */
static BTPSCONST GATT_Primary_Service_128_Entry_t SPPLE_Service_UUID =
{
   SPPLE_SERVICE_BLUETOOTH_UUID_CONSTANT
};

   /* The Tx Characteristic Declaration.                                */
static BTPSCONST GATT_Characteristic_Declaration_128_Entry_t SPPLE_Tx_Declaration =
{
   GATT_CHARACTERISTIC_PROPERTIES_NOTIFY,
   SPPLE_TX_CHARACTERISTIC_BLUETOOTH_UUID_CONSTANT
};

   /* The Tx Characteristic Value.                                      */
static BTPSCONST GATT_Characteristic_Value_128_Entry_t  SPPLE_Tx_Value =
{
   SPPLE_TX_CHARACTERISTIC_BLUETOOTH_UUID_CONSTANT,
   0,
   NULL
};

   /* The Tx Credits Characteristic Declaration.                        */
static BTPSCONST GATT_Characteristic_Declaration_128_Entry_t SPPLE_Tx_Credits_Declaration =
{
   (GATT_CHARACTERISTIC_PROPERTIES_READ|GATT_CHARACTERISTIC_PROPERTIES_WRITE_WITHOUT_RESPONSE|GATT_CHARACTERISTIC_PROPERTIES_WRITE),
   SPPLE_TX_CREDITS_CHARACTERISTIC_BLUETOOTH_UUID_CONSTANT
};

   /* The Tx Credits Characteristic Value.                              */
static BTPSCONST GATT_Characteristic_Value_128_Entry_t SPPLE_Tx_Credits_Value =
{
   SPPLE_TX_CREDITS_CHARACTERISTIC_BLUETOOTH_UUID_CONSTANT,
   0,
   NULL
};

   /* The SPPLE RX Characteristic Declaration.                          */
static BTPSCONST GATT_Characteristic_Declaration_128_Entry_t SPPLE_Rx_Declaration =
{
   (GATT_CHARACTERISTIC_PROPERTIES_WRITE_WITHOUT_RESPONSE),
   SPPLE_RX_CHARACTERISTIC_BLUETOOTH_UUID_CONSTANT
};

   /* The SPPLE RX Characteristic Value.                                */
static BTPSCONST GATT_Characteristic_Value_128_Entry_t  SPPLE_Rx_Value =
{
   SPPLE_RX_CHARACTERISTIC_BLUETOOTH_UUID_CONSTANT,
   0,
   NULL
};


   /* The SPPLE Rx Credits Characteristic Declaration.                  */
static BTPSCONST GATT_Characteristic_Declaration_128_Entry_t SPPLE_Rx_Credits_Declaration =
{
   (GATT_CHARACTERISTIC_PROPERTIES_READ|GATT_CHARACTERISTIC_PROPERTIES_NOTIFY),
   SPPLE_RX_CREDITS_CHARACTERISTIC_BLUETOOTH_UUID_CONSTANT
};

   /* The SPPLE Rx Credits Characteristic Value.                        */
static BTPSCONST GATT_Characteristic_Value_128_Entry_t SPPLE_Rx_Credits_Value =
{
   SPPLE_RX_CREDITS_CHARACTERISTIC_BLUETOOTH_UUID_CONSTANT,
   0,
   NULL
};

   /* Client Characteristic Configuration Descriptor.                   */
static GATT_Characteristic_Descriptor_16_Entry_t Client_Characteristic_Configuration =
{
   GATT_CLIENT_CHARACTERISTIC_CONFIGURATION_BLUETOOTH_UUID_CONSTANT,
   GATT_CLIENT_CHARACTERISTIC_CONFIGURATION_LENGTH,
   NULL
};

   /* The following defines the SPPLE service that is registered with   */
   /* the GATT_Register_Service function call.                          */
   /* * NOTE * This array will be registered with GATT in the call to   */
   /*          GATT_Register_Service.                                   */
BTPSCONST GATT_Service_Attribute_Entry_t SPPLE_Service[] =
{
   {GATT_ATTRIBUTE_FLAGS_READABLE,          aetPrimaryService128,            (Byte_t *)&SPPLE_Service_UUID},                  //0
   {GATT_ATTRIBUTE_FLAGS_READABLE,          aetCharacteristicDeclaration128, (Byte_t *)&SPPLE_Tx_Declaration},                //1
   {0,                                      aetCharacteristicValue128,       (Byte_t *)&SPPLE_Tx_Value},                      //2
   {GATT_ATTRIBUTE_FLAGS_READABLE_WRITABLE, aetCharacteristicDescriptor16,   (Byte_t *)&Client_Characteristic_Configuration}, //3
   {GATT_ATTRIBUTE_FLAGS_READABLE,          aetCharacteristicDeclaration128, (Byte_t *)&SPPLE_Tx_Credits_Declaration},        //4
   {GATT_ATTRIBUTE_FLAGS_READABLE_WRITABLE, aetCharacteristicValue128,       (Byte_t *)&SPPLE_Tx_Credits_Value},              //5
   {GATT_ATTRIBUTE_FLAGS_READABLE,          aetCharacteristicDeclaration128, (Byte_t *)&SPPLE_Rx_Declaration},                //6
   {GATT_ATTRIBUTE_FLAGS_WRITABLE,          aetCharacteristicValue128,       (Byte_t *)&SPPLE_Rx_Value},                      //7
   {GATT_ATTRIBUTE_FLAGS_READABLE,          aetCharacteristicDeclaration128, (Byte_t *)&SPPLE_Rx_Credits_Declaration},        //8
   {GATT_ATTRIBUTE_FLAGS_READABLE,          aetCharacteristicValue128,       (Byte_t *)&SPPLE_Rx_Credits_Value},              //9
   {GATT_ATTRIBUTE_FLAGS_READABLE_WRITABLE, aetCharacteristicDescriptor16,   (Byte_t *)&Client_Characteristic_Configuration}, //10
};

#define SPPLE_SERVICE_ATTRIBUTE_COUNT               (sizeof(SPPLE_Service)/sizeof(GATT_Service_Attribute_Entry_t))

#define SPPLE_TX_CHARACTERISTIC_ATTRIBUTE_OFFSET               2
#define SPPLE_TX_CHARACTERISTIC_CCD_ATTRIBUTE_OFFSET           3
#define SPPLE_TX_CREDITS_CHARACTERISTIC_ATTRIBUTE_OFFSET       5
#define SPPLE_RX_CHARACTERISTIC_ATTRIBUTE_OFFSET               7
#define SPPLE_RX_CREDITS_CHARACTERISTIC_ATTRIBUTE_OFFSET       9
#define SPPLE_RX_CREDITS_CHARACTERISTIC_CCD_ATTRIBUTE_OFFSET   10

   /*********************************************************************/
   /**                    END OF SERVICE TABLE                         **/
   /*********************************************************************/

   /* Internal function prototypes.                                     */
static Boolean_t CreateNewDeviceInfoEntry(DeviceInfo_t **ListHead, GAP_LE_Address_Type_t ConnectionAddressType, BD_ADDR_t ConnectionBD_ADDR);
static DeviceInfo_t *SearchDeviceInfoEntryByBD_ADDR(DeviceInfo_t **ListHead, BD_ADDR_t BD_ADDR);
static DeviceInfo_t *DeleteDeviceInfoEntry(DeviceInfo_t **ListHead, BD_ADDR_t BD_ADDR);
static void FreeDeviceInfoEntryMemory(DeviceInfo_t *EntryToFree);
static void FreeDeviceInfoList(DeviceInfo_t **ListHead);

static void UserInterface_Selection(void);
static int AddCommand(char *CommandName, CommandFunction_t CommandFunction);
static void ClearCommands(void);

static void BD_ADDRToStr(BD_ADDR_t Board_Address, BoardStr_t BoardStr);

static void DisplayAdvertisingData(GAP_LE_Advertising_Data_t *Advertising_Data);
static void DisplayPairingInformation(GAP_LE_Pairing_Capabilities_t Pairing_Capabilities);
static void DisplayUsage(char *UsageString);
static void DisplayFunctionError(char *Function,int Status);
static void DisplayFunctionSuccess(char *Function);

static int OpenStack(HCI_DriverInformation_t *HCI_DriverInformation, BTPS_Initialization_t *BTPS_Initialization);
static int CloseStack(void);

static int SetDisc(void);
static int SetConnect(void);
static int SetPairable(void);

static unsigned int AddDataToBuffer(SPPLE_Data_Buffer_t *DataBuffer, unsigned int DataLength, Byte_t *Data);
static unsigned int RemoveDataFromBuffer(SPPLE_Data_Buffer_t *DataBuffer, unsigned int BufferLength, Byte_t *Buffer);
static void InitializeBuffer(SPPLE_Data_Buffer_t *DataBuffer);

static unsigned int FillBufferWithString(SPPLE_Data_Buffer_t *DataBuffer, unsigned *CurrentBufferLength, unsigned int MaxLength, Byte_t *Buffer);

static void SPPLESendProcess(LE_Context_Info_t *LEContextInfo, DeviceInfo_t *DeviceInfo);
static void SPPLESendCredits(LE_Context_Info_t *LEContextInfo, DeviceInfo_t *DeviceInfo, unsigned int DataLength);
static void SPPLEReceiveCreditEvent(LE_Context_Info_t *LEContextInfo, DeviceInfo_t *DeviceInfo, unsigned int Credits);
static unsigned int SPPLESendData(LE_Context_Info_t *LEContextInfo, DeviceInfo_t *DeviceInfo, unsigned int DataLength, Byte_t *Data);
static void SPPLEDataIndicationEvent(LE_Context_Info_t *LEContextInfo, DeviceInfo_t *DeviceInfo, unsigned int DataLength, Byte_t *Data);
static int SPPLEReadData(LE_Context_Info_t *LEContextInfo, DeviceInfo_t *DeviceInfo, unsigned int BufferLength, Byte_t *Buffer);

static void ConfigureCapabilities(GAP_LE_Pairing_Capabilities_t *Capabilities);
static int SendPairingRequest(BD_ADDR_t BD_ADDR, Boolean_t ConnectionMaster);
static int SlavePairingRequestResponse(BD_ADDR_t BD_ADDR);
static int EncryptionInformationRequestResponse(BD_ADDR_t BD_ADDR, Byte_t KeySize, GAP_LE_Authentication_Response_Information_t *GAP_LE_Authentication_Response_Information);
static int DeleteLinkKey(BD_ADDR_t BD_ADDR);

static int SetBaudRate(ParameterList_t *TempParam);

static int OpenServer(ParameterList_t *TempParam);
static int Write(ParameterList_t *TempParam);
static int SetConfigParams(ParameterList_t *TempParam);

static int ServerMode(ParameterList_t *TempParam);

static int FindSPPPortIndex(unsigned int SerialPortID);
static int FindSPPPortIndexByServerPortNumber(unsigned int ServerPortNumber);
static int FindFreeSPPPortIndex(void);
static int FindFreeLEIndex(void);
static int FindLEIndexByAddress(BD_ADDR_t BD_ADDR);
static int UpdateConnectionID(unsigned int ConnectionID, BD_ADDR_t BD_ADDR);
static void RemoveConnectionInfo(BD_ADDR_t BD_ADDR);

   /* BTPS Callback function prototypes.                                */
static void BTPSAPI GAP_LE_Event_Callback(unsigned int BluetoothStackID,GAP_LE_Event_Data_t *GAP_LE_Event_Data, unsigned long CallbackParameter);
static void BTPSAPI GATT_ClientEventCallback_SPPLE(unsigned int BluetoothStackID, GATT_Client_Event_Data_t *GATT_Client_Event_Data, unsigned long CallbackParameter);
static void BTPSAPI GATT_Connection_Event_Callback(unsigned int BluetoothStackID, GATT_Connection_Event_Data_t *GATT_Connection_Event_Data, unsigned long CallbackParameter);
static void BTPSAPI GAP_Event_Callback(unsigned int BluetoothStackID, GAP_Event_Data_t *GAP_Event_Data, unsigned long CallbackParameter);
static void BTPSAPI SPP_Event_Callback(unsigned int BluetoothStackID, SPP_Event_Data_t *SPP_Event_Data, unsigned long CallbackParameter);
static void BTPSAPI HCI_Event_Callback(unsigned int BluetoothStackID, HCI_Event_Data_t *HCI_Event_Data, unsigned long CallbackParameter);

////////////////  /*User defined Functions*/ /////////////////////////////////////////////////////////////


unsigned char BL_Write_from_SPI(unsigned char order);
void SPI_BL_Periodic_write(void *Userparameter);
void BL_UART_Bulk_Transmission_Mode(void);

void AUTOMODE_Display(void);
void AUTOMODE_SetBaudRate(void *Userparameter);
void AUTOMODE_SetConfigParams(void *Userparameter);
void AUTOMODE_OpenServer(void *Userparameter);
void AUTOMODE_CountBeforeWrite(void *Userparameter);


/////////////////////  /* User defined definition*/       //////////////////////////////////////////////

#define SAMPLING_RATE           8 // 8kHz
#define HS_BAUD_RATE            2000000

//More than 620 Bytes can be used.
#define CHANNEL_NUMBER          16

#define SPI_PRE_SAVE_BUF_SIZE   ((SAMPLING_RATE * 2) + 2) //16-bit resolution and 1 ms data + 4Bytes
#define SPI_PRE_SAVE_BUF_NO     CHANNEL_NUMBER

#define BT_HEADER_SIZE          4 // 타임과 채널 정보 
#define BT_DATA_SIZE            (SAMPLING_RATE * 3 * 2) // 16-bit resolution and 3 ms data
#define BT_TRANS_SIZE           (BT_HEADER_SIZE + BT_DATA_SIZE)
#define BT_TX_PACKET_BUF_NO     (CHANNEL_NUMBER +24)

#define BT_TRANS_STEP_SIZE      4



//// SPI channel selection protocol ////////////////////////////

#define CH_01   0x09
#define CH_02   0x0A
#define CH_03   0x0B
#define CH_04   0x0C
#define CH_05   0x0D
#define CH_06   0x0E
#define CH_07   0x0F
#define CH_08   0x10
#define CH_09   0x11
#define CH_10   0x12
#define CH_11   0x13
#define CH_12   0x14
#define CH_13   0x15
#define CH_14   0x16
#define CH_15   0x17
#define CH_16   0x18

#define SPI_2   0x00

//// Negative threshold selection ////////////////////////////

#define NVTH_50UV       254 //spike amplitude
#define NVTH_100UV      254

#define NVTH_CH_01      NVTH_50UV
#define NVTH_CH_02      NVTH_50UV
#define NVTH_CH_03      NVTH_50UV
#define NVTH_CH_04      NVTH_50UV
#define NVTH_CH_05      NVTH_50UV
#define NVTH_CH_06      NVTH_50UV
#define NVTH_CH_07      NVTH_50UV
#define NVTH_CH_08      NVTH_50UV
#define NVTH_CH_09      NVTH_50UV
#define NVTH_CH_10      NVTH_50UV
#define NVTH_CH_11      NVTH_50UV
#define NVTH_CH_12      NVTH_50UV
#define NVTH_CH_13      NVTH_50UV
#define NVTH_CH_14      NVTH_50UV
#define NVTH_CH_15      NVTH_50UV
#define NVTH_CH_16      NVTH_50UV


////////////// User defined constant variables /////////////////////////////////////////////////
static const unsigned char ucSPSBS = SPI_PRE_SAVE_BUF_SIZE;
static const unsigned char ucBTS = BT_TRANS_SIZE;
static const unsigned char ucBTS4 = BT_TRANS_SIZE*4;
static const unsigned char ucBTPBN = BT_TX_PACKET_BUF_NO;


////////////////  /*User defined Variables*/  ////////////////////////////////////////////////////////////
//static signed long TWP_intParam;
//static int TWP_NOP;

static Byte_t SPI_Pre_Buf[SPI_PRE_SAVE_BUF_NO][SPI_PRE_SAVE_BUF_SIZE];
static Byte_t BT_Tx_Packet_Buf[BT_TX_PACKET_BUF_NO][BT_TRANS_SIZE];
static unsigned char SPI_Rx_Addr;
static unsigned char BT_Tx_Rest[BT_TX_PACKET_BUF_NO];

static unsigned char BT_Tx_Packet_Ass_From=0;//Assigned packet address in order unit (start point)
static unsigned char BT_Tx_Packet_Ass_To=0;//Assigned packet address in order unit (end point)

static unsigned char Cycle_start=0;

static unsigned char Spike[CHANNEL_NUMBER];



static unsigned char BT_Tx_Protocol[16] = 
{ 
  0x32,//pre-data
  0x02,
  0x01,
  0x20,
  
  217,//173,
  0x0,
  213,//169,
  0x0,
  
  0x40,
  0x00,
  0x09,
  0xEF,
  
  160,//72,
  0x1,
  
  0x40,//post-data
  0x31
};
  
  
////////////////////////////////////////////////////////////////////////////////////////////////////////

   /* The following function adds the specified Entry to the specified  */
   /* List.  This function allocates and adds an entry to the list that */
   /* has the same attributes as parameters to this function.  This     */
   /* function will return FALSE if NO Entry was added.  This can occur */
   /* if the element passed in was deemed invalid or the actual List    */
   /* Head was invalid.                                                 */
   /* ** NOTE ** This function does not insert duplicate entries into   */
   /*            the list.  An element is considered a duplicate if the */
   /*            Connection BD_ADDR.  When this occurs, this function   */
   /*            returns NULL.                                          */
static Boolean_t CreateNewDeviceInfoEntry(DeviceInfo_t **ListHead, GAP_LE_Address_Type_t ConnectionAddressType, BD_ADDR_t ConnectionBD_ADDR)
{
   Boolean_t     ret_val = FALSE;
   DeviceInfo_t *DeviceInfoPtr;

   /* Verify that the passed in parameters seem semi-valid.             */
   if((ListHead) && (!COMPARE_NULL_BD_ADDR(ConnectionBD_ADDR)))
   {
      /* Allocate the memory for the entry.                             */
      if((DeviceInfoPtr = BTPS_AllocateMemory(sizeof(DeviceInfo_t))) != NULL)
      {
         /* Initialize the entry.                                       */
         BTPS_MemInitialize(DeviceInfoPtr, 0, sizeof(DeviceInfo_t));
         DeviceInfoPtr->ConnectionAddressType = ConnectionAddressType;
         DeviceInfoPtr->ConnectionBD_ADDR     = ConnectionBD_ADDR;

         ret_val = BSC_AddGenericListEntry_Actual(ekBD_ADDR_t, BTPS_STRUCTURE_OFFSET(DeviceInfo_t, ConnectionBD_ADDR), BTPS_STRUCTURE_OFFSET(DeviceInfo_t, NextDeviceInfoInfoPtr), (void **)(ListHead), (void *)(DeviceInfoPtr));
         if(!ret_val)
         {
            /* Failed to add to list so we should free the memory that  */
            /* we allocated for the entry.                              */
            BTPS_FreeMemory(DeviceInfoPtr);
         }
      }
   }

   return(ret_val);
}

   /* The following function searches the specified List for the        */
   /* specified Connection BD_ADDR.  This function returns NULL if      */
   /* either the List Head is invalid, the BD_ADDR is invalid, or the   */
   /* Connection BD_ADDR was NOT found.                                 */
static DeviceInfo_t *SearchDeviceInfoEntryByBD_ADDR(DeviceInfo_t **ListHead, BD_ADDR_t BD_ADDR)
{
   return(BSC_SearchGenericListEntry(ekBD_ADDR_t, (void *)(&BD_ADDR), BTPS_STRUCTURE_OFFSET(DeviceInfo_t, ConnectionBD_ADDR), BTPS_STRUCTURE_OFFSET(DeviceInfo_t, NextDeviceInfoInfoPtr), (void **)(ListHead)));
}

   /* The following function searches the specified Key Info List for   */
   /* the specified BD_ADDR and removes it from the List.  This function*/
   /* returns NULL if either the List Head is invalid, the BD_ADDR is   */
   /* invalid, or the specified Entry was NOT present in the list.  The */
   /* entry returned will have the Next Entry field set to NULL, and    */
   /* the caller is responsible for deleting the memory associated with */
   /* this entry by calling the FreeKeyEntryMemory() function.          */
static DeviceInfo_t *DeleteDeviceInfoEntry(DeviceInfo_t **ListHead, BD_ADDR_t BD_ADDR)
{
   return(BSC_DeleteGenericListEntry(ekBD_ADDR_t, (void *)(&BD_ADDR), BTPS_STRUCTURE_OFFSET(DeviceInfo_t, ConnectionBD_ADDR), BTPS_STRUCTURE_OFFSET(DeviceInfo_t, NextDeviceInfoInfoPtr), (void **)(ListHead)));
}

   /* This function frees the specified Key Info Information member     */
   /* memory.                                                           */
static void FreeDeviceInfoEntryMemory(DeviceInfo_t *EntryToFree)
{
   BSC_FreeGenericListEntryMemory((void *)(EntryToFree));
}

   /* The following function deletes (and free's all memory) every      */
   /* element of the specified Key Info List. Upon return of this       */
   /* function, the Head Pointer is set to NULL.                        */
static void FreeDeviceInfoList(DeviceInfo_t **ListHead)
{
   BSC_FreeGenericListEntryList((void **)(ListHead), BTPS_STRUCTURE_OFFSET(DeviceInfo_t, NextDeviceInfoInfoPtr));
}

   /* The following function is responsible for choosing the user       */
   /* interface to present to the user.                                 */
static void UserInterface_Selection(void)
{
   /* Next display the available commands.                              */

   ClearCommands();

   AddCommand("SERVER", ServerMode);
}

   /* The following function is provided to allow a means to            */
   /* programatically add Commands the Global (to this module) Command  */
   /* Table.  The Command Table is simply a mapping of Command Name     */
   /* (NULL terminated ASCII string) to a command function.  This       */
   /* function returns zero if successful, or a non-zero value if the   */
   /* command could not be added to the list.                           */
static int AddCommand(char *CommandName, CommandFunction_t CommandFunction)
{
   int ret_val = 0;

   /* First, make sure that the parameters passed to us appear to be    */
   /* semi-valid.                                                       */
   if((CommandName) && (CommandFunction))
   {
      /* Next, make sure that we still have room in the Command Table   */
      /* to add commands.                                               */
      if(NumberCommands < MAX_SUPPORTED_COMMANDS)
      {

         /* Return success to the caller.                               */
         ret_val                                        = 0;
      }
      else
         ret_val = 1;
   }
   else
      ret_val = 1;

   return(ret_val);
}


   /* The following function is provided to allow a means to clear out  */
   /* all available commands from the command table.                    */
static void ClearCommands(void)
{
   /* Simply flag that there are no commands present in the table.      */
   NumberCommands = 0;
}

   /* The following function is responsible for converting data of type */
   /* BD_ADDR to a string.  The first parameter of this function is the */
   /* BD_ADDR to be converted to a string.  The second parameter of this*/
   /* function is a pointer to the string in which the converted BD_ADDR*/
   /* is to be stored.                                                  */
static void BD_ADDRToStr(BD_ADDR_t Board_Address, BoardStr_t BoardStr)
{
   BTPS_SprintF((char *)BoardStr, "0x%02X%02X%02X%02X%02X%02X", Board_Address.BD_ADDR5, Board_Address.BD_ADDR4, Board_Address.BD_ADDR3, Board_Address.BD_ADDR2, Board_Address.BD_ADDR1, Board_Address.BD_ADDR0);
}

   /* Utility function to display advertising data.                     */
static void DisplayAdvertisingData(GAP_LE_Advertising_Data_t *Advertising_Data)
{
   unsigned int Index;
   unsigned int Index2;

   /* Verify that the input parameters seem semi-valid.                 */
   if(Advertising_Data)
   {
      for(Index = 0; Index < Advertising_Data->Number_Data_Entries; Index++)
      {
         Display(("  AD Type: 0x%02X.\r\n", Advertising_Data->Data_Entries[Index].AD_Type));
         Display(("  AD Length: 0x%02X.\r\n", Advertising_Data->Data_Entries[Index].AD_Data_Length));
         if(Advertising_Data->Data_Entries[Index].AD_Data_Buffer)
         {
            Display(("  AD Data: "));
            for(Index2 = 0; Index2 < Advertising_Data->Data_Entries[Index].AD_Data_Length; Index2++)
            {
               Display(("0x%02X ", Advertising_Data->Data_Entries[Index].AD_Data_Buffer[Index2]));
            }
            Display(("\r\n"));
         }
      }
   }
}

   /* The following function displays the pairing capabalities that is  */
   /* passed into this function.                                        */
static void DisplayPairingInformation(GAP_LE_Pairing_Capabilities_t Pairing_Capabilities)
{
   /* Display the IO Capability.                                        */
   switch(Pairing_Capabilities.IO_Capability)
   {
      case licDisplayOnly:
         Display(("   IO Capability:       lcDisplayOnly.\r\n"));
         break;
      case licDisplayYesNo:
         Display(("   IO Capability:       lcDisplayYesNo.\r\n"));
         break;
      case licKeyboardOnly:
         Display(("   IO Capability:       lcKeyboardOnly.\r\n"));
         break;
      case licNoInputNoOutput:
         Display(("   IO Capability:       lcNoInputNoOutput.\r\n"));
         break;
      case licKeyboardDisplay:
         Display(("   IO Capability:       lcKeyboardDisplay.\r\n"));
         break;
   }

   Display(("   MITM:                %s.\r\n", (Pairing_Capabilities.MITM == TRUE)?"TRUE":"FALSE"));
   Display(("   Bonding Type:        %s.\r\n", (Pairing_Capabilities.Bonding_Type == lbtBonding)?"Bonding":"No Bonding"));
   Display(("   OOB:                 %s.\r\n", (Pairing_Capabilities.OOB_Present == TRUE)?"OOB":"OOB Not Present"));
   Display(("   Encryption Key Size: %d.\r\n", Pairing_Capabilities.Maximum_Encryption_Key_Size));
   Display(("   Sending Keys: \r\n"));
   Display(("      LTK:              %s.\r\n", ((Pairing_Capabilities.Sending_Keys.Encryption_Key == TRUE)?"YES":"NO")));
   Display(("      IRK:              %s.\r\n", ((Pairing_Capabilities.Sending_Keys.Identification_Key == TRUE)?"YES":"NO")));
   Display(("      CSRK:             %s.\r\n", ((Pairing_Capabilities.Sending_Keys.Signing_Key == TRUE)?"YES":"NO")));
   Display(("   Receiving Keys: \r\n"));
   Display(("      LTK:              %s.\r\n", ((Pairing_Capabilities.Receiving_Keys.Encryption_Key == TRUE)?"YES":"NO")));
   Display(("      IRK:              %s.\r\n", ((Pairing_Capabilities.Receiving_Keys.Identification_Key == TRUE)?"YES":"NO")));
   Display(("      CSRK:             %s.\r\n", ((Pairing_Capabilities.Receiving_Keys.Signing_Key == TRUE)?"YES":"NO")));
}

   /* Displays a usage string..                                         */
static void DisplayUsage(char *UsageString)
{
   Display(("Usage: %s.\r\n",UsageString));
}

   /* Displays a function error message.                                */
static void DisplayFunctionError(char *Function,int Status)
{
   Display(("%s Failed: %d.\r\n", Function, Status));
}

   /* Displays a function success message.                              */
static void DisplayFunctionSuccess(char *Function)
{
   Display(("%s success.\r\n",Function));
}

   /* The following function is responsible for opening the SS1         */
   /* Bluetooth Protocol Stack.  This function accepts a pre-populated  */
   /* HCI Driver Information structure that contains the HCI Driver     */
   /* Transport Information.  This function returns zero on successful  */
   /* execution and a negative value on all errors.                     */
static int OpenStack(HCI_DriverInformation_t *HCI_DriverInformation, BTPS_Initialization_t *BTPS_Initialization)
{
   int                           i;
   int                           Result;
   int                           ret_val = 0;
   char                          BluetoothAddress[16];
   Byte_t                        Status;
   Byte_t                        NumberLEPackets;
   Word_t                        LEPacketLength;
   BD_ADDR_t                     BD_ADDR;
   unsigned int                  ServiceID;
   HCI_Version_t                 HCIVersion;
   L2CA_Link_Connect_Params_t    L2CA_Link_Connect_Params;

   /* First check to see if the Stack has already been opened.          */
   if(!BluetoothStackID)
   {
      /* Next, makes sure that the Driver Information passed appears to */
      /* be semi-valid.                                                 */
      if(HCI_DriverInformation)
      {
         Display(("\r\n"));

         /* Initialize BTPSKNRl.                                        */
         BTPS_Init((void *)BTPS_Initialization);

         Display(("OpenStack().\r\n"));

         /* Initialize the Stack                                        */
         Result = BSC_Initialize(HCI_DriverInformation, 0);

         /* Next, check the return value of the initialization to see   */
         /* if it was successful.                                       */
         if(Result > 0)
         {
            /* The Stack was initialized successfully, inform the user  */
            /* and set the return value of the initialization function  */
            /* to the Bluetooth Stack ID.                               */
            BluetoothStackID = Result;
            Display(("Bluetooth Stack ID: %d.\r\n", BluetoothStackID));

            /* Initialize the Default Pairing Parameters.               */
            LE_Parameters.IOCapability   = DEFAULT_LE_IO_CAPABILITY;
            LE_Parameters.MITMProtection = DEFAULT_LE_MITM_PROTECTION;
            LE_Parameters.OOBDataPresent = FALSE;

            /* Initialize the default Secure Simple Pairing parameters. */
            IOCapability                 = DEFAULT_IO_CAPABILITY;
            OOBSupport                   = FALSE;
            MITMProtection               = DEFAULT_MITM_PROTECTION;

            if(!HCI_Version_Supported(BluetoothStackID, &HCIVersion))
               Display(("Device Chipset: %s.\r\n", (HCIVersion <= NUM_SUPPORTED_HCI_VERSIONS)?HCIVersionStrings[HCIVersion]:HCIVersionStrings[NUM_SUPPORTED_HCI_VERSIONS]));

            /* Let's output the Bluetooth Device Address so that the    */
            /* user knows what the Device Address is.                   */
            if(!GAP_Query_Local_BD_ADDR(BluetoothStackID, &BD_ADDR))
            {
               BD_ADDRToStr(BD_ADDR, BluetoothAddress);

               Display(("BD_ADDR: %s\r\n", BluetoothAddress));
            }

            if(HCI_Command_Supported(BluetoothStackID, HCI_SUPPORTED_COMMAND_WRITE_DEFAULT_LINK_POLICY_BIT_NUMBER) > 0)
               HCI_Write_Default_Link_Policy_Settings(BluetoothStackID, (HCI_LINK_POLICY_SETTINGS_ENABLE_MASTER_SLAVE_SWITCH|HCI_LINK_POLICY_SETTINGS_ENABLE_SNIFF_MODE), &Status);

            /* Go ahead and allow Master/Slave Role Switch.             */
            L2CA_Link_Connect_Params.L2CA_Link_Connect_Request_Config  = cqAllowRoleSwitch;
            L2CA_Link_Connect_Params.L2CA_Link_Connect_Response_Config = csMaintainCurrentRole;

            L2CA_Set_Link_Connection_Configuration(BluetoothStackID, &L2CA_Link_Connect_Params);

            if(HCI_Command_Supported(BluetoothStackID, HCI_SUPPORTED_COMMAND_WRITE_DEFAULT_LINK_POLICY_BIT_NUMBER) > 0)
               HCI_Write_Default_Link_Policy_Settings(BluetoothStackID, (HCI_LINK_POLICY_SETTINGS_ENABLE_MASTER_SLAVE_SWITCH|HCI_LINK_POLICY_SETTINGS_ENABLE_SNIFF_MODE), &Status);

            /* Delete all Stored Link Keys.                             */
            ASSIGN_BD_ADDR(BD_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);

            DeleteLinkKey(BD_ADDR);

            /* Flag that no connection is currently active.             */
            LocalDeviceIsMaster = FALSE;

            for(i=0; i<MAX_LE_CONNECTIONS; i++)
            {
               LEContextInfo[i].ConnectionID = 0;
               ASSIGN_BD_ADDR(LEContextInfo[i].ConnectionBD_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
            }

            /* Regenerate IRK and DHK from the constant Identity Root   */
            /* Key.                                                     */
            GAP_LE_Diversify_Function(BluetoothStackID, (Encryption_Key_t *)(&IR), 1,0, &IRK);
            GAP_LE_Diversify_Function(BluetoothStackID, (Encryption_Key_t *)(&IR), 3, 0, &DHK);

            /* Flag that we have no Key Information in the Key List.    */
            DeviceInfoList = NULL;

            /* Initialize the GATT Service.                             */
            if((Result = GATT_Initialize(BluetoothStackID, GATT_INITIALIZATION_FLAGS_SUPPORT_LE, GATT_Connection_Event_Callback, 0)) == 0)
            {
               /* Determine the number of LE packets that the controller*/
               /* will accept at a time.                                */
               if((!HCI_LE_Read_Buffer_Size(BluetoothStackID, &Status, &LEPacketLength, &NumberLEPackets)) && (!Status) && (LEPacketLength))
               {
                  NumberLEPackets = (NumberLEPackets/MAX_LE_CONNECTIONS);
                  NumberLEPackets = (NumberLEPackets == 0)?1:NumberLEPackets;
               }
               else
                  NumberLEPackets = 1;
               
               

               /* Set a limit on the number of packets that we will     */
               /* queue internally.                                     */
               GATT_Set_Queuing_Parameters(BluetoothStackID, (unsigned int)NumberLEPackets, (unsigned int)(NumberLEPackets-1), FALSE);

               /* Initialize the GAPS Service.                          */
               Result = GAPS_Initialize_Service(BluetoothStackID, &ServiceID);
               if(Result > 0)
               {
                  /* Save the Instance ID of the GAP Service.           */
                  GAPSInstanceID = (unsigned int)Result;

                  /* Set the GAP Device Name and Device Appearance.     */
                  GAPS_Set_Device_Name(BluetoothStackID, GAPSInstanceID, LE_DEMO_DEVICE_NAME);
                  GAPS_Set_Device_Appearance(BluetoothStackID, GAPSInstanceID, GAP_DEVICE_APPEARENCE_VALUE_GENERIC_COMPUTER);

                  /* Return success to the caller.                      */
                  ret_val        = 0;
               }
               else
               {
                  /* The Stack was NOT initialized successfully, inform */
                  /* the user and set the return value of the           */
                  /* initialization function to an error.               */
                  DisplayFunctionError("GAPS_Initialize_Service", Result);

                  /* Cleanup GATT Module.                               */
                  GATT_Cleanup(BluetoothStackID);

                  BluetoothStackID = 0;

                  ret_val          = UNABLE_TO_INITIALIZE_STACK;
               }
            }
            else
            {
               /* The Stack was NOT initialized successfully, inform the*/
               /* user and set the return value of the initialization   */
               /* function to an error.                                 */
               DisplayFunctionError("GATT_Initialize", Result);

               BluetoothStackID = 0;

               ret_val          = UNABLE_TO_INITIALIZE_STACK;
            }

            /* Initialize SPP context.                                  */
            BTPS_MemInitialize(SPPContextInfo, 0, sizeof(SPPContextInfo));
         }
         else
         {
            /* The Stack was NOT initialized successfully, inform the   */
            /* user and set the return value of the initialization      */
            /* function to an error.                                    */
            DisplayFunctionError("BSC_Initialize", Result);

            BluetoothStackID = 0;

            ret_val          = UNABLE_TO_INITIALIZE_STACK;
         }
      }
      else
      {
         /* One or more of the necessary parameters are invalid.        */
         ret_val = INVALID_PARAMETERS_ERROR;
      }
   }

   return(ret_val);
}

   /* The following function is responsible for closing the SS1         */
   /* Bluetooth Protocol Stack.  This function requires that the        */
   /* Bluetooth Protocol stack previously have been initialized via the */
   /* OpenStack() function.  This function returns zero on successful   */
   /* execution and a negative value on all errors.                     */
static int CloseStack(void)
{
   int ret_val = 0;

   /* First check to see if the Stack has been opened.                  */
   if(BluetoothStackID)
   {
      /* Cleanup GAP Service Module.                                    */
      if(GAPSInstanceID)
         GAPS_Cleanup_Service(BluetoothStackID, GAPSInstanceID);

      /* Un-registered SPP LE Service.                                  */
      if(SPPLEServiceID)
         GATT_Un_Register_Service(BluetoothStackID, SPPLEServiceID);

      /* Cleanup GATT Module.                                           */
      GATT_Cleanup(BluetoothStackID);

      /* Simply close the Stack                                         */
      BSC_Shutdown(BluetoothStackID);

      /* Free BTPSKRNL allocated memory.                                */
      BTPS_DeInit();

      Display(("Stack Shutdown.\r\n"));

      /* Free the Key List.                                             */
      FreeDeviceInfoList(&DeviceInfoList);

      /* Flag that the Stack is no longer initialized.                  */
      BluetoothStackID = 0;

      /* Flag success to the caller.                                    */
      ret_val          = 0;
   }
   else
   {
      /* A valid Stack ID does not exist, inform to user.               */
      ret_val = UNABLE_TO_INITIALIZE_STACK;
   }

   return(ret_val);
}

   /* The following function is responsible for placing the Local       */
   /* Bluetooth Device into General Discoverablity Mode.  Once in this  */
   /* mode the Device will respond to Inquiry Scans from other Bluetooth*/
   /* Devices.  This function requires that a valid Bluetooth Stack ID  */
   /* exists before running.  This function returns zero on successful  */
   /* execution and a negative value if an error occurred.              */
static int SetDisc(void)
{
   int ret_val = 0;

   /* First, check that a valid Bluetooth Stack ID exists.              */
   if(BluetoothStackID)
   {
      /* A semi-valid Bluetooth Stack ID exists, now attempt to set the */
      /* attached Devices Discoverablity Mode to General.               */
      ret_val = GAP_Set_Discoverability_Mode(BluetoothStackID, dmGeneralDiscoverableMode, 0);

      /* Next, check the return value of the GAP Set Discoverability    */
      /* Mode command for successful execution.                         */
      if(!ret_val)
      {
         /* * NOTE * Discoverability is only applicable when we are     */
         /*          advertising so save the default Discoverability    */
         /*          Mode for later.                                    */
         LE_Parameters.DiscoverabilityMode = dmGeneralDiscoverableMode;
      }
      else
      {
         /* An error occurred while trying to set the Discoverability   */
         /* Mode of the Device.                                         */
         DisplayFunctionError("Set Discoverable Mode", ret_val);
      }
   }
   else
   {
      /* No valid Bluetooth Stack ID exists.                            */
      ret_val = INVALID_STACK_ID_ERROR;
   }

   return(ret_val);
}

   /* The following function is responsible for placing the Local       */
   /* Bluetooth Device into Connectable Mode.  Once in this mode the    */
   /* Device will respond to Page Scans from other Bluetooth Devices.   */
   /* This function requires that a valid Bluetooth Stack ID exists     */
   /* before running.  This function returns zero on success and a      */
   /* negative value if an error occurred.                              */
static int SetConnect(void)
{
   int ret_val = 0;

   /* First, check that a valid Bluetooth Stack ID exists.              */
   if(BluetoothStackID)
   {
      /* Attempt to set the attached Device to be Connectable.          */
      ret_val = GAP_Set_Connectability_Mode(BluetoothStackID, cmConnectableMode);

      /* Next, check the return value of the                            */
      /* GAP_Set_Connectability_Mode() function for successful          */
      /* execution.                                                     */
      if(!ret_val)
      {
         /* * NOTE * Connectability is only an applicable when          */
         /*          advertising so we will just save the default       */
         /*          connectability for the next time we enable         */
         /*          advertising.                                       */
         LE_Parameters.ConnectableMode = lcmConnectable;
      }
      else
      {
         /* An error occurred while trying to make the Device           */
         /* Connectable.                                                */
         DisplayFunctionError("Set Connectability Mode", ret_val);
      }
   }
   else
   {
      /* No valid Bluetooth Stack ID exists.                            */
      ret_val = INVALID_STACK_ID_ERROR;
   }

   return(ret_val);
}

   /* The following function is responsible for placing the local       */
   /* Bluetooth device into Pairable mode.  Once in this mode the device*/
   /* will response to pairing requests from other Bluetooth devices.   */
   /* This function returns zero on successful execution and a negative */
   /* value on all errors.                                              */
static int SetPairable(void)
{
   int Result;
   int ret_val = 0;

   /* First, check that a valid Bluetooth Stack ID exists.              */
   if(BluetoothStackID)
   {
      /* Attempt to set the attached device to be pairable.             */
      Result = GAP_Set_Pairability_Mode(BluetoothStackID, pmPairableMode);

      /* Next, check the return value of the GAP Set Pairability mode   */
      /* command for successful execution.                              */
      if(!Result)
      {
         /* The device has been set to pairable mode, now register an   */
         /* Authentication Callback to handle the Authentication events */
         /* if required.                                                */
         Result = GAP_Register_Remote_Authentication(BluetoothStackID, GAP_Event_Callback, (unsigned long)0);

         /* Next, check the return value of the GAP Register Remote     */
         /* Authentication command for successful execution.            */
         if(!Result)
         {
            /* Now Set the LE Pairability.                              */

            /* Attempt to set the attached device to be pairable.       */
            Result = GAP_LE_Set_Pairability_Mode(BluetoothStackID, lpmPairableMode);

            /* Next, check the return value of the GAP Set Pairability  */
            /* mode command for successful execution.                   */
            if(!Result)
            {
               /* The device has been set to pairable mode, now register*/
               /* an Authentication Callback to handle the              */
               /* Authentication events if required.                    */
               Result = GAP_LE_Register_Remote_Authentication(BluetoothStackID, GAP_LE_Event_Callback, (unsigned long)0);

               /* Next, check the return value of the GAP Register      */
               /* Remote Authentication command for successful          */
               /* execution.                                            */
               if(Result)
               {
                  /* An error occurred while trying to execute this     */
                  /* function.                                          */
                  DisplayFunctionError("GAP_LE_Register_Remote_Authentication", Result);

                  ret_val = Result;
               }
            }
            else
            {
               /* An error occurred while trying to make the device     */
               /* pairable.                                             */
               DisplayFunctionError("GAP_LE_Set_Pairability_Mode", Result);

               ret_val = Result;
            }
         }
         else
         {
            /* An error occurred while trying to execute this function. */
            DisplayFunctionError("GAP_Register_Remote_Authentication", Result);

            ret_val = Result;
         }
      }
      else
      {
         /* An error occurred while trying to make the device pairable. */
         DisplayFunctionError("GAP_Set_Pairability_Mode", Result);

         ret_val = Result;
      }
   }
   else
   {
      /* No valid Bluetooth Stack ID exists.                            */
      ret_val = INVALID_STACK_ID_ERROR;
   }

   return(ret_val);
}

   /* The following function is a utility function that is used to add  */
   /* data (using InIndex as the buffer index) from the buffer specified*/
   /* by the DataBuffer parameter.  The second and third parameters     */
   /* specified the length of the data to add and the pointer to the    */
   /* data to add to the buffer.  This function returns the actual      */
   /* number of bytes that were added to the buffer (or 0 if none were  */
   /* added).                                                           */
static unsigned int AddDataToBuffer(SPPLE_Data_Buffer_t *DataBuffer, unsigned int DataLength, Byte_t *Data)
{
   unsigned int BytesAdded = 0;
   unsigned int Count;

   /* Verify that the input parameters are valid.                       */
   if((DataBuffer) && (DataLength) && (Data))
   {
      /* Loop while we have data AND space in the buffer.               */
      while(DataLength)
      {
         /* Get the number of bytes that can be placed in the buffer    */
         /* until it wraps.                                             */
         Count = DataBuffer->BufferSize - DataBuffer->InIndex;

         /* Determine if the number of bytes free is less than the      */
         /* number of bytes till we wrap and choose the smaller of the  */
         /* numbers.                                                    */
         Count = (DataBuffer->BytesFree < Count)?DataBuffer->BytesFree:Count;

         /* Cap the Count that we add to buffer to the length of the    */
         /* data provided by the caller.                                */
         Count = (Count > DataLength)?DataLength:Count;

         if(Count)
         {
            /* Copy the data into the buffer.                           */
            BTPS_MemCopy(&DataBuffer->Buffer[DataBuffer->InIndex], Data, Count);

            /* Update the counts.                                       */
            DataBuffer->InIndex   += Count;
            DataBuffer->BytesFree -= Count;
            DataLength            -= Count;
            BytesAdded            += Count;
            Data                  += Count;

            /* Wrap the InIndex if necessary.                           */
            if(DataBuffer->InIndex >= DataBuffer->BufferSize)
               DataBuffer->InIndex = 0;
         }
         else
            break;
      }
   }

   return(BytesAdded);
}

   /* The following function is a utility function that is used to      */
   /* removed data (using OutIndex as the buffer index) from the buffer */
   /* specified by the DataBuffer parameter The second parameter        */
   /* specifies the length of the Buffer that is pointed to by the third*/
   /* parameter.  This function returns the actual number of bytes that */
   /* were removed from the DataBuffer (or 0 if none were added).       */
   /* * NOTE * Buffer is optional and if not specified up to            */
   /*          BufferLength bytes will be deleted from the Buffer.      */
static unsigned int RemoveDataFromBuffer(SPPLE_Data_Buffer_t *DataBuffer, unsigned int BufferLength, Byte_t *Buffer)
{
   unsigned int Count;
   unsigned int BytesRemoved = 0;
   unsigned int MaxRemove;

   /* Verify that the input parameters are valid.                       */
   if((DataBuffer) && (BufferLength))
   {
      /* Loop while we have data to remove and space in the buffer to   */
      /* place it.                                                      */
      while(BufferLength)
      {
         /* Determine the number of bytes that are present in the       */
         /* buffer.                                                     */
         Count = DataBuffer->BufferSize - DataBuffer->BytesFree;
         if(Count)
         {
            /* Calculate the maximum number of bytes that I can remove  */
            /* from the buffer before it wraps.                         */
            MaxRemove = DataBuffer->BufferSize - DataBuffer->OutIndex;

            /* Cap max we can remove at the BufferLength of the caller's*/
            /* buffer.                                                  */
            MaxRemove = (MaxRemove > BufferLength)?BufferLength:MaxRemove;

            /* Cap the number of bytes I will remove in this iteration  */
            /* at the maximum I can remove or the number of bytes that  */
            /* are in the buffer.                                       */
            Count = (Count > MaxRemove)?MaxRemove:Count;

            /* Copy the data into the caller's buffer (If specified).   */
            if(Buffer)
            {
               BTPS_MemCopy(Buffer, &DataBuffer->Buffer[DataBuffer->OutIndex], Count);
               Buffer += Count;
            }

            /* Update the counts.                                       */
            DataBuffer->OutIndex  += Count;
            DataBuffer->BytesFree += Count;
            BytesRemoved          += Count;
            BufferLength          -= Count;

            /* Wrap the OutIndex if necessary.                          */
            if(DataBuffer->OutIndex >= DataBuffer->BufferSize)
               DataBuffer->OutIndex = 0;
         }
         else
            break;
      }
   }

   return(BytesRemoved);
}

   /* The following function is used to initialize the specified buffer */
   /* to the defaults.                                                  */
static void InitializeBuffer(SPPLE_Data_Buffer_t *DataBuffer)
{
   /* Verify that the input parameters are valid.                       */
   if(DataBuffer)
   {
      DataBuffer->BufferSize = SPPLE_DATA_CREDITS;
      DataBuffer->BytesFree  = SPPLE_DATA_CREDITS;
      DataBuffer->InIndex    = 0;
      DataBuffer->OutIndex   = 0;
   }
}


   /* The following funcition is a utility function that exists to fill */
   /* the specified buffer with the DataStr that is used to send data.  */
   /* This function will fill from the CurrentBufferLength up to Max    */
   /* Length in Buffer.  CurrentBufferLength is used to return the total*/
   /* length of the buffer.  The first parameter specifies the          */
   /* DeviceInfo which is used to fill any remainder of the string so   */
   /* that there are no breaks in the pattern.  This function returns   */
   /* the number of bytes added to the transmit buffer of the specified */
   /* device.                                                           */
static unsigned int FillBufferWithString(SPPLE_Data_Buffer_t *DataBuffer, unsigned *CurrentBufferLength, unsigned int MaxLength, Byte_t *Buffer)
{
   unsigned int DataCount;
   unsigned int Added2Buffer = 0;

   /* Verify that the input parameter is semi-valid.                    */
   if((DataBuffer) && (CurrentBufferLength) && (MaxLength) && (Buffer))
   {
      /* Copy as much of the DataStr into the Transmit buffer as is     */
      /* possible.                                                      */
      while(*CurrentBufferLength < MaxLength)
      {
         /* Cap the data to copy at the maximum of the string length and*/
         /* the remaining amount that can be placed in the buffer.      */
         DataCount = (DataStrLen > (MaxLength-*CurrentBufferLength))?(MaxLength-*CurrentBufferLength):DataStrLen;

         /* Note we should only add full strings into the transmit      */
         /* buffer.                                                     */
         if(DataCount == DataStrLen)
         {
            /* Build the data string into the SPPLEBuffer.              */
            BTPS_MemCopy(&Buffer[*CurrentBufferLength], DataStr, DataCount);

            /* Increment the index.                                     */
            *CurrentBufferLength += DataCount;
         }
         else
            break;
      }
   }

   return(Added2Buffer);
}

   /* The following function is responsible for handling a Send Process.*/
static void SPPLESendProcess(LE_Context_Info_t *LEContextInfo, DeviceInfo_t *DeviceInfo)
{
   int          Result;
   Boolean_t    Done = FALSE;
   unsigned int TransmitIndex;
   unsigned int MaxLength;
   unsigned int SPPLEBufferLength;
   unsigned int Added2Buffer;

   /* Verify that the input parameter is semi-valid.                    */
   if((LEContextInfo) && (DeviceInfo))
   {
//xxx
//xxx      Display(("Transmit Credits: %u.\r\n", DeviceInfo->TransmitCredits));

      /* Loop while we have data to send and we have not used up all    */
      /* Transmit Credits.                                              */
      TransmitIndex     = 0;
      SPPLEBufferLength = 0;
      Added2Buffer      = 0;
      while((LEContextInfo->SPPLEBufferInfo.SendInfo.BytesToSend) && (LEContextInfo->BufferFull == FALSE) && (LEContextInfo->SPPLEBufferInfo.TransmitCredits) && (!Done))
      {
         /* Get the maximum length of what we can send in this          */
         /* transaction.                                                */
         MaxLength = (LEContextInfo->SPPLEBufferInfo.SendInfo.BytesToSend > LEContextInfo->SPPLEBufferInfo.TransmitCredits)?LEContextInfo->SPPLEBufferInfo.TransmitCredits:LEContextInfo->SPPLEBufferInfo.SendInfo.BytesToSend;
         MaxLength = (MaxLength > SPPLE_DATA_BUFFER_LENGTH)?SPPLE_DATA_BUFFER_LENGTH:MaxLength;

         /* If we do not have any outstanding data get some more data.  */
         if(!SPPLEBufferLength)
         {
            /* Send any buffered data first.                            */
            if(LEContextInfo->SPPLEBufferInfo.TransmitBuffer.BytesFree != LEContextInfo->SPPLEBufferInfo.TransmitBuffer.BufferSize)
            {
               /* Remove the queued data from the Transmit Buffer.      */
               SPPLEBufferLength = RemoveDataFromBuffer(&(LEContextInfo->SPPLEBufferInfo.TransmitBuffer), MaxLength, SPPLEBuffer);

               /* If we added some data to the transmit buffer decrement*/
               /* what we just removed.                                 */
               if(Added2Buffer>=SPPLEBufferLength)
                  Added2Buffer -= SPPLEBufferLength;
            }

            /* Fill up the rest of the buffer with the data string.     */
            Added2Buffer     += FillBufferWithString(&(LEContextInfo->SPPLEBufferInfo.TransmitBuffer), &SPPLEBufferLength, MaxLength, SPPLEBuffer);

            /* Reset the Transmit Index to 0.                           */
            TransmitIndex     = 0;

            /* If we dont have any data to send (i.e.  we didn't have   */
            /* enough credits to fill up a string) we should just exit  */
            /* the loop.                                                */
            if(SPPLEBufferLength == 0)
               break;
         }

         /* Use the correct API based on device role for SPPLE.         */
         if(DeviceInfo->Flags & DEVICE_INFO_FLAGS_SPPLE_SERVER)
         {
            /* We are acting as SPPLE Server, so notify the Tx          */
            /* Characteristic.                                          */
            if(DeviceInfo->ServerInfo.Tx_Client_Configuration_Descriptor == GATT_CLIENT_CONFIGURATION_CHARACTERISTIC_NOTIFY_ENABLE)
               Result = GATT_Handle_Value_Notification(BluetoothStackID, SPPLEServiceID, LEContextInfo->ConnectionID, SPPLE_TX_CHARACTERISTIC_ATTRIBUTE_OFFSET, (Word_t)SPPLEBufferLength, &SPPLEBuffer[TransmitIndex]);
            else
            {
               /* Not configured for notifications so exit the loop.    */
               Done = TRUE;
            }
         }
         else
         {
            /* We are acting as SPPLE Client, so write to the Rx        */
            /* Characteristic.                                          */
            if(DeviceInfo->ClientInfo.Tx_Characteristic)
               Result = GATT_Write_Without_Response_Request(BluetoothStackID, LEContextInfo->ConnectionID, DeviceInfo->ClientInfo.Rx_Characteristic, (Word_t)SPPLEBufferLength, &SPPLEBuffer[TransmitIndex]);
            else
            {
               /* We have not discovered the Tx Characteristic, so exit */
               /* the loop.                                             */
               Done = TRUE;
            }
         }

         /* Check to see if any data was written.                       */
         if(!Done)
         {
            /* Check to see if the data was written successfully.       */
            if(Result >= 0)
            {
//xxx Debug Statement
//xxx               Display(("Actually sent %u of Maximum %u.\r\n", (unsigned int)Result, MaxLength));

               /* Adjust the counters.                                  */
               LEContextInfo->SPPLEBufferInfo.SendInfo.BytesToSend -= (unsigned int)Result;
               LEContextInfo->SPPLEBufferInfo.SendInfo.BytesSent   += (unsigned int)Result;
               TransmitIndex                                       += (unsigned int)Result;
               SPPLEBufferLength                                   -= (unsigned int)Result;
               LEContextInfo->SPPLEBufferInfo.TransmitCredits      -= (unsigned int)Result;

               /* If we have no more remaining Tx Credits AND we have   */
               /* data built up to send, we need to queue this in the Tx*/
               /* Buffer.                                               */
               if((!(LEContextInfo->SPPLEBufferInfo.TransmitCredits)) && (SPPLEBufferLength))
               {
                  /* Add the remaining data to the transmit buffer.     */
                  AddDataToBuffer(&(LEContextInfo->SPPLEBufferInfo.TransmitBuffer), SPPLEBufferLength, &SPPLEBuffer[TransmitIndex]);

                  SPPLEBufferLength = 0;
               }
            }
            else
            {
               /* Check to see what error has occurred.                 */
               if(Result == BTPS_ERROR_INSUFFICIENT_BUFFER_SPACE)
               {
                  /* Queue is full so add the data that we have into the*/
                  /* transmit buffer for this device and wait on the    */
                  /* buffer to empty.                                   */
                  AddDataToBuffer(&(LEContextInfo->SPPLEBufferInfo.TransmitBuffer), SPPLEBufferLength, &SPPLEBuffer[TransmitIndex]);

                  SPPLEBufferLength         = 0;

                  /* Flag that the LE buffer is full                    */
                  LEContextInfo->BufferFull = TRUE;
//xxx
//xxx                     Display(("xxx Buffer Full for device %s.\r\n", __FUNCTION__));
               }
               else
               {
                  Display(("SEND failed with error %d\r\n", Result));

                  LEContextInfo->SPPLEBufferInfo.SendInfo.BytesToSend  = 0;
               }
            }
         }
      }

      /* If we have added more bytes to the transmit buffer than we can */
      /* send in this process remove the extra.                         */
      if(Added2Buffer > LEContextInfo->SPPLEBufferInfo.SendInfo.BytesToSend)
         RemoveDataFromBuffer(&(LEContextInfo->SPPLEBufferInfo.TransmitBuffer), Added2Buffer-LEContextInfo->SPPLEBufferInfo.SendInfo.BytesToSend, NULL);

      /* Display a message if we have sent all required data.           */
      if((!(LEContextInfo->SPPLEBufferInfo.SendInfo.BytesToSend)) && (LEContextInfo->SPPLEBufferInfo.SendInfo.BytesSent))
      {
         Display(("\r\nSend Complete, Sent %lu.\r\n", LEContextInfo->SPPLEBufferInfo.SendInfo.BytesSent));

         LEContextInfo->SPPLEBufferInfo.SendInfo.BytesSent = 0;
      }
   }
}

   /* The following function is responsible for transmitting the        */
   /* specified number of credits to the remote device.                 */
static void SPPLESendCredits(LE_Context_Info_t *LEContextInfo, DeviceInfo_t *DeviceInfo, unsigned int DataLength)
{
   int              Result;
   unsigned int     ActualCredits;
   NonAlignedWord_t Credits;

   /* Verify that the input parameters are semi-valid.                  */
   if((LEContextInfo) && (DeviceInfo) && ((DataLength) || (LEContextInfo->SPPLEBufferInfo.QueuedCredits)))
   {
//xxx Debug statement
//xxx      Display(("\r\nSending %u Credits.\r\n", DataLength));

      /* Only attempt to send the credits if the LE buffer is not full. */
      if(LEContextInfo->BufferFull == FALSE)
      {
         /* Make sure that we don't credit more than can be filled in   */
         /* our receive buffer.                                         */
         ActualCredits = DataLength + LEContextInfo->SPPLEBufferInfo.QueuedCredits;
         ActualCredits = (ActualCredits > LEContextInfo->SPPLEBufferInfo.ReceiveBuffer.BytesFree)?LEContextInfo->SPPLEBufferInfo.ReceiveBuffer.BytesFree:ActualCredits;

         /* Format the credit packet.                                   */
         ASSIGN_HOST_WORD_TO_LITTLE_ENDIAN_UNALIGNED_WORD(&Credits, ActualCredits);

         /* Determine how to send credits based on the role.            */
         if(DeviceInfo->Flags & DEVICE_INFO_FLAGS_SPPLE_SERVER)
         {
            /* We are acting as a server so notify the Rx Credits       */
            /* characteristic.                                          */
            if(DeviceInfo->ServerInfo.Rx_Credit_Client_Configuration_Descriptor == GATT_CLIENT_CONFIGURATION_CHARACTERISTIC_NOTIFY_ENABLE)
               Result = GATT_Handle_Value_Notification(BluetoothStackID, SPPLEServiceID, LEContextInfo->ConnectionID, SPPLE_RX_CREDITS_CHARACTERISTIC_ATTRIBUTE_OFFSET, WORD_SIZE, (Byte_t *)&Credits);
            else
               Result = 0;
         }
         else
         {
            /* We are acting as a client so send a Write Without        */
            /* Response packet to the Tx Credit Characteristic.         */
            if(DeviceInfo->ClientInfo.Tx_Credit_Characteristic)
               Result = GATT_Write_Without_Response_Request(BluetoothStackID, LEContextInfo->ConnectionID, DeviceInfo->ClientInfo.Tx_Credit_Characteristic, WORD_SIZE, &Credits);
            else
               Result = 0;
         }

         /* If an error occurred we need to queue the credits to try    */
         /* again.                                                      */
         if(Result >= 0)
         {
            /* Clear the queued credit count as if there were any queued*/
            /* credits they have now been sent.                         */
            LEContextInfo->SPPLEBufferInfo.QueuedCredits = 0;
         }
         else
         {
            if(Result == BTPS_ERROR_INSUFFICIENT_BUFFER_SPACE)
            {
               /* Flag that the buffer is full.                         */
               LEContextInfo->BufferFull = TRUE;
//xxx
//xxx                  Display(("xxx Buffer Full for device %s.\r\n", __FUNCTION__));
            }

            LEContextInfo->SPPLEBufferInfo.QueuedCredits += DataLength;
         }
      }
      else
         LEContextInfo->SPPLEBufferInfo.QueuedCredits += DataLength;
   }
}

   /* The following function is responsible for handling a received     */
   /* credit, event.                                                    */
static void SPPLEReceiveCreditEvent(LE_Context_Info_t *LEContextInfo, DeviceInfo_t *DeviceInfo, unsigned int Credits)
{
   /* Verify that the input parameters are semi-valid.                  */
   if((LEContextInfo) && (DeviceInfo))
   {
//xxx Debug statement
//xxx      Display(("\r\nReceived %u Credits, Credit Count %u.\r\n", Credits, Credits+LEContextInfo->SPPLEBufferInfo.TransmitCredits));

      /* If this is a real credit event store the number of credits.    */
      LEContextInfo->SPPLEBufferInfo.TransmitCredits += Credits;

      /* Handle any active send process.                                */
      SPPLESendProcess(LEContextInfo, DeviceInfo);

      /* Send all queued data.                                          */
      SPPLESendData(LEContextInfo, DeviceInfo, 0, NULL);

      /* It is possible that we have received data queued, so call the  */
      /* Data Indication Event to handle this.                          */
      SPPLEDataIndicationEvent(LEContextInfo, DeviceInfo, 0, NULL);
   }
}

   /* The following function sends the specified data to the specified  */
   /* data.  This function will queue any of the data that does not go  */
   /* out.  This function returns the number of bytes sent if all the   */
   /* data was sent, or 0.                                              */
   /* * NOTE * If DataLength is 0 and Data is NULL then all queued data */
   /*          will be sent.                                            */
static unsigned int SPPLESendData(LE_Context_Info_t *LEContextInfo, DeviceInfo_t *DeviceInfo, unsigned int DataLength, Byte_t *Data)
{
   int          Result;
   Boolean_t    Done;
   unsigned int DataCount;
   unsigned int MaxLength;
   unsigned int TransmitIndex;
   unsigned int QueuedBytes;
   unsigned int SPPLEBufferLength;
   unsigned int TotalBytesTransmitted = 0;

   /* Verify that the input parameters are semi-valid.                  */
   if((LEContextInfo) && (DeviceInfo))
   {
      /* Loop while we have data to send and we can send it.            */
      Done              = FALSE;
      TransmitIndex     = 0;
      SPPLEBufferLength = 0;
      while(!Done)
      {
         /* Check to see if we have credits to use to transmit the data */
         /* (and that the buffer is not FULL).                          */
         if((LEContextInfo->SPPLEBufferInfo.TransmitCredits) && (LEContextInfo->BufferFull == FALSE))
         {
            /* Get the maximum length of what we can send in this       */
            /* transaction.                                             */
            MaxLength = (SPPLE_DATA_BUFFER_LENGTH > LEContextInfo->SPPLEBufferInfo.TransmitCredits)?LEContextInfo->SPPLEBufferInfo.TransmitCredits:SPPLE_DATA_BUFFER_LENGTH;

            /* If we do not have any outstanding data get some more     */
            /* data.                                                    */
            if(!SPPLEBufferLength)
            {
               /* Send any buffered data first.                         */
               if(LEContextInfo->SPPLEBufferInfo.TransmitBuffer.BytesFree != LEContextInfo->SPPLEBufferInfo.TransmitBuffer.BufferSize)
               {
                  /* Remove the queued data from the Transmit Buffer.   */
                  SPPLEBufferLength = RemoveDataFromBuffer(&(LEContextInfo->SPPLEBufferInfo.TransmitBuffer), MaxLength, SPPLEBuffer);
               }
               else
               {
                  /* Check to see if we have data to send.              */
                  if((DataLength) && (Data))
                  {
                     /* Copy the data to send into the SPPLEBuffer.     */
                     SPPLEBufferLength = (DataLength > MaxLength)?MaxLength:DataLength;
                     BTPS_MemCopy(SPPLEBuffer, Data, SPPLEBufferLength);

                     DataLength -= SPPLEBufferLength;
                     Data       += SPPLEBufferLength;
                  }
                  else
                  {
                     /* No data queued or data left to send so exit the */
                     /* loop.                                           */
                     Done = TRUE;
                  }
               }

               /* Set the count of data that we can send.               */
               DataCount         = SPPLEBufferLength;

               /* Reset the Transmit Index to 0.                        */
               TransmitIndex     = 0;
            }
            else
            {
               /* We have data to send so cap it at the maximum that can*/
               /* be transmitted.                                       */
               DataCount = (SPPLEBufferLength > MaxLength)?MaxLength:SPPLEBufferLength;
            }

            /* Try to write data if not exiting the loop.               */
            if(!Done)
            {
//xxx Debug statement
//xxx               Display(("\r\nTrying to send %u bytes.\r\n", DataCount));

               /* Use the correct API based on device role for SPPLE.   */
               if(DeviceInfo->Flags & DEVICE_INFO_FLAGS_SPPLE_SERVER)
               {
                  /* We are acting as SPPLE Server, so notify the Tx    */
                  /* Characteristic.                                    */
                  if(DeviceInfo->ServerInfo.Tx_Client_Configuration_Descriptor == GATT_CLIENT_CONFIGURATION_CHARACTERISTIC_NOTIFY_ENABLE)
                     Result = GATT_Handle_Value_Notification(BluetoothStackID, SPPLEServiceID, LEContextInfo->ConnectionID, SPPLE_TX_CHARACTERISTIC_ATTRIBUTE_OFFSET, (Word_t)DataCount, &SPPLEBuffer[TransmitIndex]);
                  else
                  {
                     /* Not configured for notifications so exit the    */
                     /* loop.                                           */
                     Done = TRUE;
                  }
               }
               else
               {
                  /* We are acting as SPPLE Client, so write to the Rx  */
                  /* Characteristic.                                    */
                  if(DeviceInfo->ClientInfo.Tx_Characteristic)
                     Result = GATT_Write_Without_Response_Request(BluetoothStackID, LEContextInfo->ConnectionID, DeviceInfo->ClientInfo.Rx_Characteristic, (Word_t)DataCount, &SPPLEBuffer[TransmitIndex]);
                  else
                  {
                     /* We have not discovered the Tx Characteristic, so*/
                     /* exit the loop.                                  */
                     Done = TRUE;
                  }
               }

               /* Check to see if any data was written.                 */
               if(!Done)
               {
                  /* Check to see if the data was written successfully. */
                  if(Result >= 0)
                  {
                     /* Adjust the counters.                            */
                     TransmitIndex                                  += (unsigned int)Result;
                     SPPLEBufferLength                              -= (unsigned int)Result;
                     LEContextInfo->SPPLEBufferInfo.TransmitCredits -= (unsigned int)Result;

//xxx Debug statement
//xxx                     Display(("\r\nSent %u, Remaining Credits %u.\r\n", (unsigned int)Result, DeviceInfo->TransmitCredits));

                     /* Flag that data was sent.                        */
                     TotalBytesTransmitted                          += Result;

                     /* If we have no more remaining Tx Credits AND we  */
                     /* have data built up to send, we need to queue    */
                     /* this in the Tx Buffer.                          */
                     if((!(LEContextInfo->SPPLEBufferInfo.TransmitCredits)) && (SPPLEBufferLength))
                     {
                        /* Add the remaining data to the transmit       */
                        /* buffer.                                      */
                        QueuedBytes = AddDataToBuffer(&(LEContextInfo->SPPLEBufferInfo.TransmitBuffer), SPPLEBufferLength, &SPPLEBuffer[TransmitIndex]);
                        TotalBytesTransmitted += QueuedBytes;

                        SPPLEBufferLength = 0;
                     }
                  }
                  else
                  {
                     /* Failed to send data so add the data that we have*/
                     /* into the transmit buffer for this device and    */
                     /* wait on the buffer to empty.                    */
                     QueuedBytes = AddDataToBuffer(&(LEContextInfo->SPPLEBufferInfo.TransmitBuffer), SPPLEBufferLength, &SPPLEBuffer[TransmitIndex]);
                     TotalBytesTransmitted += QueuedBytes;

                     SPPLEBufferLength = 0;

                     /* Flag that we should exit the loop.              */
                     Done              = TRUE;

                     /* Check to see what error has occurred.           */
                     if(Result == BTPS_ERROR_INSUFFICIENT_BUFFER_SPACE)
                     {
                        /* Flag that the LE buffer is full.             */
                        LEContextInfo->BufferFull = TRUE;

//xxx
//xxx                           Display(("xxx Buffer Full for device %s.\r\n", __FUNCTION__));
                     }
                     else
                        Display(("SEND failed with error %d\r\n", Result));
                  }
               }
            }
         }
         else
         {
            /* We have no transmit credits, so buffer the data.         */
            QueuedBytes = AddDataToBuffer(&(LEContextInfo->SPPLEBufferInfo.TransmitBuffer), DataLength, Data);

            TotalBytesTransmitted += QueuedBytes;

            /* Exit the loop.                                           */
            Done = TRUE;
         }
      }
   }

   return(TotalBytesTransmitted);
}

   /* The following function is responsible for handling a data         */
   /* indication event.                                                 */
static void SPPLEDataIndicationEvent(LE_Context_Info_t *LEContextInfo, DeviceInfo_t *DeviceInfo, unsigned int DataLength, Byte_t *Data)
{
   Boolean_t    Done;
   unsigned int ReadLength;
   unsigned int Length;
   unsigned int Transmitted;

   /* Verify that the input parameters are semi-valid.                  */
   if((LEContextInfo) && (DeviceInfo))
   {
//xxx Debug statement
//xxx      Display(("\r\nData Indication Event: %u bytes.\r\n", (unsigned int)DataLength));

      /* If we are automatically reading the data, go ahead and credit  */
      /* what we just received, as well as reading everying in the      */
      /* buffer.                                                        */
      if((AutomaticReadActive) || (LoopbackActive))
      {
         /* Loop until we read all of the data queued.                  */
         Done = FALSE;
         while(!Done)
         {
            /* If in loopback mode cap what we remove at the max of what*/
            /* we can send or queue.  If in loopback we will also not   */
            /* attempt to read more than can be filled in the transmit  */
            /* buffer.  This is to guard against the case where we read */
            /* data that we could not queue or transmit due to a buffer */
            /* full condition.                                          */
            if(LoopbackActive)
               ReadLength = (SPPLE_DATA_BUFFER_LENGTH > (LEContextInfo->SPPLEBufferInfo.TransmitBuffer.BytesFree))?(LEContextInfo->SPPLEBufferInfo.TransmitBuffer.BytesFree):SPPLE_DATA_BUFFER_LENGTH;
            else
               ReadLength = SPPLE_DATA_BUFFER_LENGTH;

            /* Read all queued data.                                    */
            Length = SPPLEReadData(LEContextInfo, DeviceInfo, ReadLength, SPPLEBuffer);
            if(((int)Length) > 0)
            {
               /* If loopback is active, loopback the data.             */
               if(LoopbackActive)
               {
                  Transmitted = SPPLESendData(LEContextInfo, DeviceInfo, Length, SPPLEBuffer);
                  if((Transmitted != Length) || (LEContextInfo->BufferFull))
                  {
                     /* If we failed to send all that was read (or the  */
                     /* LE buffer is now full) then we should exit the  */
                     /* loop.                                           */
                     Done = TRUE;
                  }
               }

               /* If we are displaying the data then do that here.      */
               if(DisplayRawData)
               {
                  SPPLEBuffer[Length] = '\0';
                  Display(((char *)SPPLEBuffer));
               }
            }
            else
               Done = TRUE;
         }

         /* Only send/display data just received if any is specified in */
         /* the call to this function.                                  */
         if((DataLength) && (Data))
         {
            /* If loopback is active, loopback the data just received.  */
            if((AutomaticReadActive) || (LoopbackActive))
            {
               /* If we are displaying the data then do that here.      */
               if(DisplayRawData)
               {
                  BTPS_MemCopy(SPPLEBuffer, Data, DataLength);
                  SPPLEBuffer[DataLength] = '\0';
                  Display(((char *)SPPLEBuffer));
               }

               /* Check to see if Loopback is active, if it is we will  */
               /* loopback the data we just received.                   */
               if(LoopbackActive)
               {
                  /* Only queue the data in the receive buffer that we  */
                  /* cannot send.  If in loopback we will also not      */
                  /* attempt to read more than can be filled in the     */
                  /* transmit buffer.  This is to guard against the case*/
                  /* where we read data that we could not queue or      */
                  /* transmit due to a buffer full condition.           */
                  ReadLength = (DataLength > (LEContextInfo->SPPLEBufferInfo.TransmitBuffer.BytesFree))?(LEContextInfo->SPPLEBufferInfo.TransmitBuffer.BytesFree):DataLength;

                  /* Send the data.                                     */
                  if((Transmitted = SPPLESendData(LEContextInfo, DeviceInfo, ReadLength, Data)) > 0)
                  {
                     /* Credit the data we just sent.                   */
                     SPPLESendCredits(LEContextInfo, DeviceInfo, Transmitted);

                     /* Increment what was just sent.                   */
                     DataLength -= ReadLength;
                     Data       += ReadLength;
                  }
               }
               else
               {
                  /* Loopback is not active so just credit back the data*/
                  /* we just received.                                  */
                  SPPLESendCredits(LEContextInfo, DeviceInfo, DataLength);

                  DataLength = 0;
               }

               /* If we have data left that cannot be sent, queue this  */
               /* in the receive buffer.                                */
               if((DataLength) && (Data))
               {
                  /* We are not in Loopback or Automatic Read Mode so   */
                  /* just buffer all the data.                          */
                  Length = AddDataToBuffer(&(LEContextInfo->SPPLEBufferInfo.ReceiveBuffer), DataLength, Data);
                  if(Length != DataLength)
                     Display(("Receive Buffer Overflow of %u bytes", DataLength - Length));
               }
            }

            /* If we are displaying the data then do that here.         */
            if(DisplayRawData)
            {
               BTPS_MemCopy(SPPLEBuffer, Data, DataLength);
               SPPLEBuffer[DataLength] = '\0';
               Display(((char *)SPPLEBuffer));
            }
         }
      }
      else
      {
         if((DataLength) && (Data))
         {
            /* Display a Data indication event.                         */
            Display(("\r\nData Indication Event, Connection ID %u, Received %u bytes.\r\n", LEContextInfo->ConnectionID, DataLength));

            /* We are not in Loopback or Automatic Read Mode so just    */
            /* buffer all the data.                                     */
            Length = AddDataToBuffer(&(LEContextInfo->SPPLEBufferInfo.ReceiveBuffer), DataLength, Data);
            if(Length != DataLength)
               Display(("Receive Buffer Overflow of %u bytes.\r\n", DataLength - Length));
         }
      }
   }
}

   /* The following function is used to read data from the specified    */
   /* device.  The final two parameters specify the BufferLength and the*/
   /* Buffer to read the data into.  On success this function returns   */
   /* the number of bytes read.  If an error occurs this will return a  */
   /* negative error code.                                              */
static int SPPLEReadData(LE_Context_Info_t *LEContextInfo, DeviceInfo_t *DeviceInfo, unsigned int BufferLength, Byte_t *Buffer)
{
   int          ret_val;
   Boolean_t    Done;
   unsigned int Length;
   unsigned int TotalLength;

   /* Verify that the input parameters are semi-valid.                  */
   if((LEContextInfo) && (DeviceInfo) && (BufferLength) && (Buffer))
   {
      Done        = FALSE;
      TotalLength = 0;
      while(!Done)
      {
         Length = RemoveDataFromBuffer(&(LEContextInfo->SPPLEBufferInfo.ReceiveBuffer), BufferLength, Buffer);
         if(Length > 0)
         {
            BufferLength -= Length;
            Buffer       += Length;
            TotalLength  += Length;
         }
         else
            Done = TRUE;
      }

      /* Credit what we read.                                           */
      SPPLESendCredits(LEContextInfo, DeviceInfo, TotalLength);

      /* Return the total number of bytes read.                         */
      ret_val = (int)TotalLength;
   }
   else
      ret_val = BTPS_ERROR_INVALID_PARAMETER;

   return(ret_val);
}

   /* The following function provides a mechanism to configure a        */
   /* Pairing Capabilities structure with the application's pairing     */
   /* parameters.                                                       */
static void ConfigureCapabilities(GAP_LE_Pairing_Capabilities_t *Capabilities)
{
   /* Make sure the Capabilities pointer is semi-valid.                 */
   if(Capabilities)
   {
      /* Configure the Pairing Cabilities structure.                    */
      Capabilities->Bonding_Type                    = lbtBonding;
      Capabilities->IO_Capability                   = LE_Parameters.IOCapability;
      Capabilities->MITM                            = LE_Parameters.MITMProtection;
      Capabilities->OOB_Present                     = LE_Parameters.OOBDataPresent;

      /* ** NOTE ** This application always requests that we use the    */
      /*            maximum encryption because this feature is not a    */
      /*            very good one, if we set less than the maximum we   */
      /*            will internally in GAP generate a key of the        */
      /*            maximum size (we have to do it this way) and then   */
      /*            we will zero out how ever many of the MSBs          */
      /*            necessary to get the maximum size.  Also as a slave */
      /*            we will have to use Non-Volatile Memory (per device */
      /*            we are paired to) to store the negotiated Key Size. */
      /*            By requesting the maximum (and by not storing the   */
      /*            negotiated key size if less than the maximum) we    */
      /*            allow the slave to power cycle and regenerate the   */
      /*            LTK for each device it is paired to WITHOUT storing */
      /*            any information on the individual devices we are    */
      /*            paired to.                                          */
      Capabilities->Maximum_Encryption_Key_Size        = GAP_LE_MAXIMUM_ENCRYPTION_KEY_SIZE;

      /* This application only demostrates using Long Term Key's (LTK)  */
      /* for encryption of a LE Link, however we could request and send */
      /* all possible keys here if we wanted to.                        */
      Capabilities->Receiving_Keys.Encryption_Key     = TRUE;
      Capabilities->Receiving_Keys.Identification_Key = FALSE;
      Capabilities->Receiving_Keys.Signing_Key        = FALSE;

      Capabilities->Sending_Keys.Encryption_Key       = TRUE;
      Capabilities->Sending_Keys.Identification_Key   = FALSE;
      Capabilities->Sending_Keys.Signing_Key          = FALSE;
   }
}

   /* The following function provides a mechanism for sending a pairing */
   /* request to a device that is connected on an LE Link.              */
static int SendPairingRequest(BD_ADDR_t BD_ADDR, Boolean_t ConnectionMaster)
{
   int                           ret_val;
   BoardStr_t                    BoardStr;
   GAP_LE_Pairing_Capabilities_t Capabilities;

   /* Make sure a Bluetooth Stack is open.                              */
   if(BluetoothStackID)
   {
      /* Make sure the BD_ADDR is valid.                                */
      if(!COMPARE_NULL_BD_ADDR(BD_ADDR))
      {
         /* Configure the application pairing parameters.               */
         ConfigureCapabilities(&Capabilities);

         /* Set the BD_ADDR of the device that we are attempting to pair*/
         /* with.                                                       */

         BD_ADDRToStr(BD_ADDR, BoardStr);
         Display(("Attempting to Pair to %s.\r\n", BoardStr));

         /* Attempt to pair to the remote device.                       */
         if(ConnectionMaster)
         {
            /* Start the pairing process.                               */
            ret_val = GAP_LE_Pair_Remote_Device(BluetoothStackID, BD_ADDR, &Capabilities, GAP_LE_Event_Callback, 0);

            Display(("     GAP_LE_Pair_Remote_Device returned %d.\r\n", ret_val));
         }
         else
         {
            /* As a slave we can only request that the Master start     */
            /* the pairing process.                                     */
            ret_val = GAP_LE_Request_Security(BluetoothStackID, BD_ADDR, Capabilities.Bonding_Type, Capabilities.MITM, GAP_LE_Event_Callback, 0);

            Display(("     GAP_LE_Request_Security returned %d.\r\n", ret_val));
         }
      }
      else
      {
         Display(("Invalid Parameters.\r\n"));

         ret_val = INVALID_PARAMETERS_ERROR;
      }
   }
   else
   {
      Display(("Stack ID Invalid.\r\n"));

      ret_val = INVALID_STACK_ID_ERROR;
   }

   return(ret_val);
}

   /* The following function provides a mechanism of sending a Slave    */
   /* Pairing Response to a Master's Pairing Request.                   */
static int SlavePairingRequestResponse(BD_ADDR_t BD_ADDR)
{
   int                                          ret_val;
   BoardStr_t                                   BoardStr;
   GAP_LE_Authentication_Response_Information_t AuthenticationResponseData;

   /* Make sure a Bluetooth Stack is open.                              */
   if(BluetoothStackID)
   {
      BD_ADDRToStr(BD_ADDR, BoardStr);
      Display(("Sending Pairing Response to %s.\r\n", BoardStr));

      /* We must be the slave if we have received a Pairing Request     */
      /* thus we will respond with our capabilities.                    */
      AuthenticationResponseData.GAP_LE_Authentication_Type = larPairingCapabilities;
      AuthenticationResponseData.Authentication_Data_Length = GAP_LE_PAIRING_CAPABILITIES_SIZE;

      /* Configure the Application Pairing Parameters.                  */
      ConfigureCapabilities(&(AuthenticationResponseData.Authentication_Data.Pairing_Capabilities));

      /* Attempt to pair to the remote device.                          */
      ret_val = GAP_LE_Authentication_Response(BluetoothStackID, BD_ADDR, &AuthenticationResponseData);

      Display(("GAP_LE_Authentication_Response returned %d.\r\n", ret_val));
   }
   else
   {
      Display(("Stack ID Invalid.\r\n"));

      ret_val = INVALID_STACK_ID_ERROR;
   }

   return(ret_val);
}

   /* The following function is provided to allow a mechanism of        */
   /* responding to a request for Encryption Information to send to a   */
   /* remote device.                                                    */
static int EncryptionInformationRequestResponse(BD_ADDR_t BD_ADDR, Byte_t KeySize, GAP_LE_Authentication_Response_Information_t *GAP_LE_Authentication_Response_Information)
{
   int    ret_val;
   Word_t LocalDiv;

   /* Make sure a Bluetooth Stack is open.                              */
   if(BluetoothStackID)
   {
      /* Make sure the input parameters are semi-valid.                 */
      if((!COMPARE_NULL_BD_ADDR(BD_ADDR)) && (GAP_LE_Authentication_Response_Information))
      {
         Display(("   Calling GAP_LE_Generate_Long_Term_Key.\r\n"));

         /* Generate a new LTK, EDIV and Rand tuple.                    */
         ret_val = GAP_LE_Generate_Long_Term_Key(BluetoothStackID, (Encryption_Key_t *)(&DHK), (Encryption_Key_t *)(&ER), &(GAP_LE_Authentication_Response_Information->Authentication_Data.Encryption_Information.LTK), &LocalDiv, &(GAP_LE_Authentication_Response_Information->Authentication_Data.Encryption_Information.EDIV), &(GAP_LE_Authentication_Response_Information->Authentication_Data.Encryption_Information.Rand));
         if(!ret_val)
         {
            Display(("   Encryption Information Request Response.\r\n"));

            /* Response to the request with the LTK, EDIV and Rand      */
            /* values.                                                  */
            GAP_LE_Authentication_Response_Information->GAP_LE_Authentication_Type                                     = larEncryptionInformation;
            GAP_LE_Authentication_Response_Information->Authentication_Data_Length                                     = GAP_LE_ENCRYPTION_INFORMATION_DATA_SIZE;
            GAP_LE_Authentication_Response_Information->Authentication_Data.Encryption_Information.Encryption_Key_Size = KeySize;

            ret_val = GAP_LE_Authentication_Response(BluetoothStackID, BD_ADDR, GAP_LE_Authentication_Response_Information);
            if(!ret_val)
            {
               Display(("   GAP_LE_Authentication_Response (larEncryptionInformation) success.\r\n", ret_val));
            }
            else
            {
               Display(("   Error - SM_Generate_Long_Term_Key returned %d.\r\n", ret_val));
            }
         }
         else
         {
            Display(("   Error - SM_Generate_Long_Term_Key returned %d.\r\n", ret_val));
         }
      }
      else
      {
         Display(("Invalid Parameters.\r\n"));

         ret_val = INVALID_PARAMETERS_ERROR;
      }
   }
   else
   {
      Display(("Stack ID Invalid.\r\n"));

      ret_val = INVALID_STACK_ID_ERROR;
   }

   return(ret_val);
}

   /* The following function is a utility function that exists to delete*/
   /* the specified Link Key from the Local Bluetooth Device.  If a NULL*/
   /* Bluetooth Device Address is specified, then all Link Keys will be */
   /* deleted.                                                          */
static int DeleteLinkKey(BD_ADDR_t BD_ADDR)
{
   int       Result;
   Byte_t    Status_Result;
   Word_t    Num_Keys_Deleted = 0;
   BD_ADDR_t NULL_BD_ADDR;

   Result = HCI_Delete_Stored_Link_Key(BluetoothStackID, BD_ADDR, TRUE, &Status_Result, &Num_Keys_Deleted);

   /* Any stored link keys for the specified address (or all) have been */
   /* deleted from the chip.  Now, let's make sure that our stored Link */
   /* Key Array is in sync with these changes.                          */

   /* First check to see all Link Keys were deleted.                    */
   ASSIGN_BD_ADDR(NULL_BD_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);

   if(COMPARE_BD_ADDR(BD_ADDR, NULL_BD_ADDR))
      BTPS_MemInitialize(LinkKeyInfo, 0, sizeof(LinkKeyInfo));
   else
   {
      /* Individual Link Key.  Go ahead and see if know about the entry */
      /* in the list.                                                   */
      for(Result=0;(Result<sizeof(LinkKeyInfo)/sizeof(LinkKeyInfo_t));Result++)
      {
         if(COMPARE_BD_ADDR(BD_ADDR, LinkKeyInfo[Result].BD_ADDR))
         {
            LinkKeyInfo[Result].BD_ADDR = NULL_BD_ADDR;

            break;
         }
      }
   }

   return(Result);
}

   /* The following function is responsible for opening a Serial Port   */
   /* Server on the Local Device.  This function opens the Serial Port  */
   /* Server on the specified RFCOMM Channel.  This function returns    */
   /* zero if successful, or a negative return value if an error        */
   /* occurred.                                                         */
static int OpenServer(ParameterList_t *TempParam)
{
   int           SerialPortIndex;
   int           ret_val;
   char         *ServiceName;
   DWord_t       SPPServerSDPHandle;
   unsigned int  ServerPortID;
   unsigned int  ServerPortNumber;

   /* First check to see if a valid Bluetooth Stack ID exists.          */
   if(BluetoothStackID)
   {
      /* Next, check to see if the parameters specified are valid.      */
      if((TempParam) && (TempParam->NumberofParameters >= 1) && (TempParam->Params[0].intParam))
      {
         /* Find an empty slot.                                         */
         if((SerialPortIndex = FindFreeSPPPortIndex()) >= 0)
         {
            /* Save the server port number.                             */
            ServerPortNumber = TempParam->Params[0].intParam;

            /* Simply attempt to open an Serial Server, on RFCOMM Server*/
            /* Port 1.                                                  */
            ret_val = SPP_Open_Server_Port(BluetoothStackID, ServerPortNumber, SPP_Event_Callback, (unsigned long)0);

            /* If the Open was successful, then note the Serial Port    */
            /* Server ID.                                               */
            if(ret_val > 0)
            {
               /* Note the Serial Port Server ID of the opened Serial   */
               /* Port Server.                                          */
               ServerPortID = (unsigned int)ret_val;

               /* Create a Buffer to hold the Service Name.             */
               if((ServiceName = BTPS_AllocateMemory(64)) != NULL)
               {
                  /* The Server was opened successfully, now register a */
                  /* SDP Record indicating that an Serial Port Server   */
                  /* exists. Do this by first creating a Service Name.  */
                  BTPS_SprintF(ServiceName, "Serial Port Server Port %d", ServerPortNumber);

                  /* Only register an SDP record if we have not already */
                  /* opened a server on this port number.               */
                  if(FindSPPPortIndexByServerPortNumber(ServerPortNumber) < 0)
                  {
                     /* Now that a Service Name has been created try to */
                     /* Register the SDP Record.                        */
                     ret_val = SPP_Register_Generic_SDP_Record(BluetoothStackID, ServerPortID, ServiceName, &SPPServerSDPHandle);
                  }
                  else
                  {
                     /* We already have opened another SPP Port on that */
                     /* RFCOMM Port Number so there is no need to       */
                     /* register a duplicate SDP Record.                */
                     SPPServerSDPHandle = 0;
                     ret_val            = 0;
                  }

                  /* If there was an error creating the Serial Port     */
                  /* Server's SDP Service Record then go ahead an close */
                  /* down the server an flag an error.                  */
                  if(ret_val < 0)
                  {
                     Display(("Unable to Register Server SDP Record, Error = %d.\r\n", ret_val));

                     SPP_Close_Server_Port(BluetoothStackID, ServerPortID);

                     /* Flag that there is no longer an Serial Port     */
                     /* Server Open.                                    */
                     ServerPortID = 0;

                     ret_val      = UNABLE_TO_REGISTER_SERVER;
                  }
                  else
                  {
                     /* Simply flag to the user that everything         */
                     /* initialized correctly.                          */
                     Display(("Server Opened: Server Port %u, Serial Port ID %u.\r\n", (unsigned int)TempParam->Params[0].intParam, ServerPortID));

                     /* We found an empty slot to store the context     */
                     SPPContextInfo[SerialPortIndex].LocalSerialPortID  = ServerPortID;
                     SPPContextInfo[SerialPortIndex].ServerPortNumber   = ServerPortNumber;
                     SPPContextInfo[SerialPortIndex].SPPServerSDPHandle = SPPServerSDPHandle;
                     SPPContextInfo[SerialPortIndex].Connected          = FALSE;

                     /* If this message is not seen in the terminal log */
                     /* after opening a server, then something went     */
                     /* wrong                                           */
                     Display(("Server Port Context Stored.\r\n"));

                     /* Flag success to the caller.                     */
                     ret_val = 0;
                  }

                  /* Free the Service Name buffer.                      */
                  BTPS_FreeMemory(ServiceName);
               }
               else
               {
                  Display(("Failed to allocate buffer to hold Service Name in SDP Record.\r\n"));
               }
            }
            else
            {
               Display(("Unable to Open Server on: %d, Error = %d.\r\n", TempParam->Params[0].intParam, ret_val));

               ret_val = UNABLE_TO_REGISTER_SERVER;
            }
         }
         else
         {
            /* Maximum number of ports reached.                         */
            Display(("Maximum allowed server ports open.\r\n"));

            ret_val = FUNCTION_ERROR;
         }
      }
      else
      {
         DisplayUsage("Open [Port Number]");

         ret_val = INVALID_PARAMETERS_ERROR;
      }
   }
   else
   {
      /* No valid Bluetooth Stack ID exists.                            */
      ret_val = INVALID_STACK_ID_ERROR;
   }

   return(ret_val);
}

   /* The following function is responsible for Writing Data to an Open */
   /* SPP Port.  The string that is written is defined by the constant  */
   /* TEST_DATA (at the top of this file).  This function requires that */
   /* a valid Bluetooth Stack ID and Serial Port ID exist before        */
   /* running.  This function returns zero is successful or a negative  */
   /* return value if there was an error.                               */
static int Write(ParameterList_t *TempParam)
{
//   TWP_NOP=TempParam->NumberofParameters;
//   TWP_intParam=TempParam->Params[0].intParam;
   
   UART_Init();
   SPI_Init();
   Buffer_Reset();
   RHD_Init();
   BL_UART_Bulk_Transmission_Mode();
   NumberScheduledFunctions=0;
   
   MSP430Ticks=0;
   Cycle_start=1;
   
   
   return(0);
}

   /* The following function is responsible for setting the current     */
   /* configuration parameters that are used by SPP.  This function will*/
   /* return zero on successful execution and a negative value on       */
   /* errors.                                                           */
static int SetConfigParams(ParameterList_t *TempParam)
{
   int                        ret_val;
   SPP_Configuration_Params_t SPPConfigurationParams;

   /* First check to see if the parameters required for the execution of*/
   /* this function appear to be semi-valid.                            */
   if(BluetoothStackID)
   {
      /* Next check to see if the parameters required for the execution */
      /* of this function appear to be semi-valid.                      */
      if((TempParam) && (TempParam->NumberofParameters > 2))
      {
         /* Parameters have been specified, go ahead and write them to  */
         /* the stack.                                                  */
         SPPConfigurationParams.MaximumFrameSize   = (unsigned int)(TempParam->Params[0].intParam);
         SPPConfigurationParams.TransmitBufferSize = (unsigned int)(TempParam->Params[1].intParam);
         SPPConfigurationParams.ReceiveBufferSize  = (unsigned int)(TempParam->Params[2].intParam);

         ret_val = SPP_Set_Configuration_Parameters(BluetoothStackID, &SPPConfigurationParams);

         if(ret_val >= 0)
         {
            Display(("SPP_Set_Configuration_Parameters(): Success\r\n", ret_val));
            Display(("   MaximumFrameSize   : %d (0x%X)\r\n", SPPConfigurationParams.MaximumFrameSize, SPPConfigurationParams.MaximumFrameSize));
            Display(("   TransmitBufferSize : %d (0x%X)\r\n", SPPConfigurationParams.TransmitBufferSize, SPPConfigurationParams.TransmitBufferSize));
            Display(("   ReceiveBufferSize  : %d (0x%X)\r\n", SPPConfigurationParams.ReceiveBufferSize, SPPConfigurationParams.ReceiveBufferSize));

            /* Flag success.                                            */
            ret_val = 0;
         }
         else
         {
            /* Error setting the current parameters.                    */
            Display(("SPP_Set_Configuration_Parameters(): Error %d.\r\n", ret_val));

            ret_val = FUNCTION_ERROR;
         }
      }
      else
      {
         DisplayUsage("SetConfigParams [MaximumFrameSize] [TransmitBufferSize (0: don't change)] [ReceiveBufferSize (0: don't change)]");

         ret_val = INVALID_PARAMETERS_ERROR;
      }
   }
   else
   {
      /* One or more of the necessary parameters are invalid.           */
      ret_val = INVALID_PARAMETERS_ERROR;
   }

   return(ret_val);
}


   /* The following thread is responsible for checking changing the     */
   /* current Baud Rate used to talk to the Radio.                      */
   /* * NOTE * This function ONLY configures the Baud Rate for a TI     */
   /*          Bluetooth chipset.                                       */
static int SetBaudRate(ParameterList_t *TempParam)
{
   int ret_val;

   /* First check to see if the parameters required for the execution of*/
   /* this function appear to be semi-valid.                            */
   if(BluetoothStackID)
   {
      /* Next check to see if the parameters required for the execution */
      /* of this function appear to be semi-valid.                      */
      if((TempParam) && (TempParam->NumberofParameters > 0) && (TempParam->Params[0].intParam))
      {
         /* Next, write the command to the device.                      */
         ret_val = VS_Update_UART_Baud_Rate(BluetoothStackID, (DWord_t)TempParam->Params[0].intParam);
         if(!ret_val)
         {
            Display(("VS_Update_UART_Baud_Rate(%lu): Success.\r\n", TempParam->Params[0].intParam));
         }
         else
         {
            /* Unable to write vendor specific command to chipset.      */
            Display(("VS_Update_UART_Baud_Rate(%lu): Failure %d, %d.\r\n", TempParam->Params[0].intParam, ret_val));

            ret_val = FUNCTION_ERROR;
         }
      }
      else
      {
         DisplayUsage("SetBaudRate [BaudRate]");

         ret_val = INVALID_PARAMETERS_ERROR;
      }
   }
   else
   {
      /* One or more of the necessary parameters are invalid.           */
      ret_val = INVALID_PARAMETERS_ERROR;
   }

   return(ret_val);
}


   /* The following function is responsible for changing the User       */
   /* Interface Mode to Server.                                         */
static int ServerMode(ParameterList_t *TempParam)
{
   //////////////////////////////////////////////////////////
   BTPS_AddFunctionToScheduler(AUTOMODE_SetBaudRate, NULL, 5000);
   
   return(0);
}

   /* The following function is a utility function that is used to find */
   /* the SPP Port Index by the specified Serial Port ID.  This function*/
   /* returns the index of the Serial Port or -1 on failure.            */
static int FindSPPPortIndex(unsigned int SerialPortID)
{
   int          ret_val = -1;
   unsigned int i;

   /* Search the list for the Serial Port Info.                         */
   for(i=0; i < sizeof(SPPContextInfo)/sizeof(SPP_Context_Info_t); i++)
   {
      /* Check to see if this entry matches the entry we are to search  */
      /* for.                                                           */
      if(SPPContextInfo[i].LocalSerialPortID == SerialPortID)
      {
         ret_val = (int)i;
         break;
      }
   }

   return(ret_val);
}

   /* The following function is a utility function that is used to find */
   /* the SPP Port Index by the specified Server Port Number.  This     */
   /* function returns the index of the Serial Port or -1 on failure.   */
static int FindSPPPortIndexByServerPortNumber(unsigned int ServerPortNumber)
{
   int          ret_val = -1;
   unsigned int i;

   /* Search the list for the Serial Port Info.                         */
   for(i=0; i < sizeof(SPPContextInfo)/sizeof(SPP_Context_Info_t); i++)
   {
      /* Check to see if this entry matches the entry we are to search  */
      /* for.                                                           */
      if(SPPContextInfo[i].ServerPortNumber == ServerPortNumber)
      {
         ret_val = (int)i;
         break;
      }
   }

   return(ret_val);
}

   /* The following function is a utility function that is used to find */
   /* a SPP Port Index that is not currently in use.  This function     */
   /* returns the index of the Serial Port or -1 on failure.            */
static int FindFreeSPPPortIndex(void)
{
   return(FindSPPPortIndex(0));
}

   /* The following function is responsible for iterating through the   */
   /* array BDInfoArray[MAX_LE_CONNECTIONS], which contains the         */
   /* connection information for connected LE devices.  It returns -1 if*/
   /* the a free connection index is not found.  If a free index is     */
   /* found, it returns the free index which can be used for another    */
   /* connection.                                                       */
static int FindFreeLEIndex(void)
{
   BD_ADDR_t NullBD_ADDR;

   ASSIGN_BD_ADDR(NullBD_ADDR, 0, 0, 0, 0, 0, 0);

   return(FindLEIndexByAddress(NullBD_ADDR));
}

   /* The following function is responsible for iterating through the   */
   /* array BDInfoArray[MAX_LE_CONNECTIONS], which contains the         */
   /* connection information for connected LE devices.  It returns -1 if*/
   /* the BD_ADDR is not found.  If the BD_ADDR is found, it returns the*/
   /* index at which the BD_ADDR was found in the array.                */
static int FindLEIndexByAddress(BD_ADDR_t BD_ADDR)
{
   int i;
   int ret_val = -1;

   for(i=0; i<MAX_LE_CONNECTIONS; i++)
   {
      if(COMPARE_BD_ADDR(BD_ADDR, LEContextInfo[i].ConnectionBD_ADDR))
      {
         ret_val = i;
         break;
      }
   }

   return(ret_val);
}


   /* The following function is responsible for updating the connection */
   /* identifier for a given BD_ADDR.  If an entry in the array         */
   /* BDInfoArray[MAX_LE_CONNECTIONS] has a matching BD_ADDR, then its  */
   /* ConnectionID is set to the passed value of ConnectionID and the   */
   /* function returns its index in the array.  If no matching BD_ADDR  */
   /* is found, the function returns -1.                                */
static int UpdateConnectionID(unsigned int ConnectionID, BD_ADDR_t BD_ADDR)
{
   int LEConnectionIndex;

   /* Check for the index of the entry for this connection.             */
   LEConnectionIndex = FindLEIndexByAddress(BD_ADDR);
   if(LEConnectionIndex >= 0)
      LEContextInfo[LEConnectionIndex].ConnectionID = ConnectionID;
   else
   {
      Display(("Error in updating ConnectionID.\r\n"));
   }

   return(LEConnectionIndex);
}

   /* The following function is responsible for clearing the values of  */
   /* an entry in BDInfoArray if its ConnectionBD_ADDR matches BD_ADDR. */
static void RemoveConnectionInfo(BD_ADDR_t BD_ADDR)
{
   int LEConnectionIndex;

   /* If an index is returned (any but -1), then found                  */
   LEConnectionIndex = FindLEIndexByAddress(BD_ADDR);
   if(LEConnectionIndex >= 0)
   {
      ASSIGN_BD_ADDR(LEContextInfo[LEConnectionIndex].ConnectionBD_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
      LEContextInfo[LEConnectionIndex].ConnectionID = 0;

      /* Re-initialize the Transmit and Receive Buffers, as well as the */
      /* transmit credits.                                              */
      InitializeBuffer(&(LEContextInfo[LEConnectionIndex].SPPLEBufferInfo.TransmitBuffer));
      InitializeBuffer(&(LEContextInfo[LEConnectionIndex].SPPLEBufferInfo.ReceiveBuffer));

      /* Clear the Transmit Credits count.                              */
      LEContextInfo[LEConnectionIndex].SPPLEBufferInfo.TransmitCredits      = 0;

      /* Flag that no credits are queued.                               */
      LEContextInfo[LEConnectionIndex].SPPLEBufferInfo.QueuedCredits        = 0;

      /* Clear the SPPLE Send Information.                              */
      LEContextInfo[LEConnectionIndex].SPPLEBufferInfo.SendInfo.BytesToSend = 0;
      LEContextInfo[LEConnectionIndex].SPPLEBufferInfo.SendInfo.BytesSent   = 0;
      LEContextInfo[LEConnectionIndex].SPPLEBufferInfo.SendInfo.BufferFull  = FALSE;
   }
}

   /* ***************************************************************** */
   /*                         Event Callbacks                           */
   /* ***************************************************************** */

   /* The following function is for the GAP LE Event Receive Data       */
   /* Callback.  This function will be called whenever a Callback has   */
   /* been registered for the specified GAP LE Action that is associated*/
   /* with the Bluetooth Stack.  This function passes to the caller the */
   /* GAP LE Event Data of the specified Event and the GAP LE Event     */
   /* Callback Parameter that was specified when this Callback was      */
   /* installed.  The caller is free to use the contents of the GAP LE  */
   /* Event Data ONLY in the context of this callback.  If the caller   */
   /* requires the Data for a longer period of time, then the callback  */
   /* function MUST copy the data into another Data Buffer.  This       */
   /* function is guaranteed NOT to be invoked more than once           */
   /* simultaneously for the specified installed callback (i.e.  this   */
   /* function DOES NOT have be reentrant).  It Needs to be noted       */
   /* however, that if the same Callback is installed more than once,   */
   /* then the callbacks will be called serially.  Because of this, the */
   /* processing in this function should be as efficient as possible.   */
   /* It should also be noted that this function is called in the Thread*/
   /* Context of a Thread that the User does NOT own.  Therefore,       */
   /* processing in this function should be as efficient as possible    */
   /* (this argument holds anyway because other GAP Events will not be  */
   /* processed while this function call is outstanding).               */
   /* * NOTE * This function MUST NOT Block and wait for Events that can*/
   /*          only be satisfied by Receiving a Bluetooth Event         */
   /*          Callback.  A Deadlock WILL occur because NO Bluetooth    */
   /*          Callbacks will be issued while this function is currently*/
   /*          outstanding.                                             */
static void BTPSAPI GAP_LE_Event_Callback(unsigned int BluetoothStackID, GAP_LE_Event_Data_t *GAP_LE_Event_Data, unsigned long CallbackParameter)
{
   int                                           Result;
   int                                           LEConnectionInfo;
   BoardStr_t                                    BoardStr;
   unsigned int                                  Index;
   DeviceInfo_t                                 *DeviceInfo;
   Long_Term_Key_t                               GeneratedLTK;
   GAP_LE_Security_Information_t                 GAP_LE_Security_Information;
   GAP_LE_Connection_Parameters_t                ConnectionParameters;
   GAP_LE_Advertising_Report_Data_t             *DeviceEntryPtr;
   GAP_LE_Authentication_Event_Data_t           *Authentication_Event_Data;
   GAP_LE_Authentication_Response_Information_t  GAP_LE_Authentication_Response_Information;

   /* Verify that all parameters to this callback are Semi-Valid.       */
   if((BluetoothStackID) && (GAP_LE_Event_Data))
   {
      switch(GAP_LE_Event_Data->Event_Data_Type)
      {
         case etLE_Advertising_Report:
            Display(("\r\netLE_Advertising_Report with size %d.\r\n",(int)GAP_LE_Event_Data->Event_Data_Size));
            Display(("  %d Responses.\r\n",GAP_LE_Event_Data->Event_Data.GAP_LE_Advertising_Report_Event_Data->Number_Device_Entries));

            for(Index = 0; Index < GAP_LE_Event_Data->Event_Data.GAP_LE_Advertising_Report_Event_Data->Number_Device_Entries; Index++)
            {
               DeviceEntryPtr = &(GAP_LE_Event_Data->Event_Data.GAP_LE_Advertising_Report_Event_Data->Advertising_Data[Index]);

               /* Display the packet type for the device                */
               switch(DeviceEntryPtr->Advertising_Report_Type)
               {
                  case rtConnectableUndirected:
                     Display(("  Advertising Type: %s.\r\n", "rtConnectableUndirected"));
                     break;
                  case rtConnectableDirected:
                     Display(("  Advertising Type: %s.\r\n", "rtConnectableDirected"));
                     break;
                  case rtScannableUndirected:
                     Display(("  Advertising Type: %s.\r\n", "rtScannableUndirected"));
                     break;
                  case rtNonConnectableUndirected:
                     Display(("  Advertising Type: %s.\r\n", "rtNonConnectableUndirected"));
                     break;
                  case rtScanResponse:
                     Display(("  Advertising Type: %s.\r\n", "rtScanResponse"));
                     break;
               }

               /* Display the Address Type.                             */
               if(DeviceEntryPtr->Address_Type == latPublic)
               {
                  Display(("  Address Type: %s.\r\n","atPublic"));
               }
               else
               {
                  Display(("  Address Type: %s.\r\n","atRandom"));
               }

               /* Display the Device Address.                           */
               Display(("  Address: 0x%02X%02X%02X%02X%02X%02X.\r\n", DeviceEntryPtr->BD_ADDR.BD_ADDR5, DeviceEntryPtr->BD_ADDR.BD_ADDR4, DeviceEntryPtr->BD_ADDR.BD_ADDR3, DeviceEntryPtr->BD_ADDR.BD_ADDR2, DeviceEntryPtr->BD_ADDR.BD_ADDR1, DeviceEntryPtr->BD_ADDR.BD_ADDR0));
               Display(("  RSSI: %d.\r\n", (int)DeviceEntryPtr->RSSI));
               Display(("  Data Length: %d.\r\n", DeviceEntryPtr->Raw_Report_Length));

               DisplayAdvertisingData(&(DeviceEntryPtr->Advertising_Data));
            }
            break;
         case etLE_Connection_Complete:
            Display(("\r\netLE_Connection_Complete with size %d.\r\n",(int)GAP_LE_Event_Data->Event_Data_Size));

            if(GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Complete_Event_Data)
            {
               BD_ADDRToStr(GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Complete_Event_Data->Peer_Address, BoardStr);

               Display(("   Status:       0x%02X.\r\n", GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Complete_Event_Data->Status));
               Display(("   Role:         %s.\r\n", (GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Complete_Event_Data->Master)?"Master":"Slave"));
               Display(("   Address Type: %s.\r\n", (GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Complete_Event_Data->Peer_Address_Type == latPublic)?"Public":"Random"));
               Display(("   BD_ADDR:      %s.\r\n", BoardStr));

               if(GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Complete_Event_Data->Status == HCI_ERROR_CODE_NO_ERROR)
               {
                  /* If not already in the connection info array, add   */
                  /* it.                                                */
                  if(FindLEIndexByAddress(GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Complete_Event_Data->Peer_Address) < 0)
                  {
                     /* Find an unused position in the connection info  */
                     /* array.                                          */
                     LEConnectionInfo = FindFreeLEIndex();
                     if(LEConnectionInfo >= 0)
                        LEContextInfo[LEConnectionInfo].ConnectionBD_ADDR = GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Complete_Event_Data->Peer_Address;
                  }

                  /* Set a global flag to indicate if we are the        */
                  /* connection master.                                 */
                  LocalDeviceIsMaster = GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Complete_Event_Data->Master;

                  /* Make sure that no entry already exists.            */
                  if((DeviceInfo = SearchDeviceInfoEntryByBD_ADDR(&DeviceInfoList, GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Complete_Event_Data->Peer_Address)) == NULL)
                  {
                     /* No entry exists so create one.                  */
                     if(!CreateNewDeviceInfoEntry(&DeviceInfoList, GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Complete_Event_Data->Peer_Address_Type, GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Complete_Event_Data->Peer_Address))
                        Display(("Failed to add device to Device Info List.\r\n"));
                  }
                  else
                  {
                     /* If we are the Master of the connection we will  */
                     /* attempt to Re-Establish Security if a LTK for   */
                     /* this device exists (i.e.  we previously paired).*/
                     if(LocalDeviceIsMaster)
                     {
                        /* Re-Establish Security if there is a LTK that */
                        /* is stored for this device.                   */
                        if(DeviceInfo->Flags & DEVICE_INFO_FLAGS_LTK_VALID)
                        {
                           /* Re-Establish Security with this LTK.      */
                           Display(("Attempting to Re-Establish Security.\r\n"));

                           /* Attempt to re-establish security to this  */
                           /* device.                                   */
                           GAP_LE_Security_Information.Local_Device_Is_Master                                      = TRUE;
                           GAP_LE_Security_Information.Security_Information.Master_Information.LTK                 = DeviceInfo->LTK;
                           GAP_LE_Security_Information.Security_Information.Master_Information.EDIV                = DeviceInfo->EDIV;
                           GAP_LE_Security_Information.Security_Information.Master_Information.Rand                = DeviceInfo->Rand;
                           GAP_LE_Security_Information.Security_Information.Master_Information.Encryption_Key_Size = DeviceInfo->EncryptionKeySize;

                           Result = GAP_LE_Reestablish_Security(BluetoothStackID, DeviceInfo->ConnectionBD_ADDR, &GAP_LE_Security_Information, GAP_LE_Event_Callback, 0);
                           if(Result)
                           {
                              Display(("GAP_LE_Reestablish_Security returned %d.\r\n",Result));
                           }
                        }
                     }
                  }
               }
               else
               {
                  /* Clear the Connection ID.                              */
                  RemoveConnectionInfo(GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Complete_Event_Data->Peer_Address);
               }
            }
            break;
         case etLE_Disconnection_Complete:
            Display(("\r\netLE_Disconnection_Complete with size %d.\r\n", (int)GAP_LE_Event_Data->Event_Data_Size));

            if(GAP_LE_Event_Data->Event_Data.GAP_LE_Disconnection_Complete_Event_Data)
            {
               Display(("   Status: 0x%02X.\r\n", GAP_LE_Event_Data->Event_Data.GAP_LE_Disconnection_Complete_Event_Data->Status));
               Display(("   Reason: 0x%02X.\r\n", GAP_LE_Event_Data->Event_Data.GAP_LE_Disconnection_Complete_Event_Data->Reason));

               BD_ADDRToStr(GAP_LE_Event_Data->Event_Data.GAP_LE_Disconnection_Complete_Event_Data->Peer_Address, BoardStr);

               Display(("   BD_ADDR: %s.\r\n", BoardStr));

               /* Check to see if the device info is present in the     */
               /* list.                                                 */
               if((DeviceInfo = SearchDeviceInfoEntryByBD_ADDR(&DeviceInfoList, GAP_LE_Event_Data->Event_Data.GAP_LE_Disconnection_Complete_Event_Data->Peer_Address)) != NULL)
               {
                  /* Flag that no service discovery operation is        */
                  /* outstanding for this device.                       */
                  DeviceInfo->Flags &= ~DEVICE_INFO_FLAGS_SERVICE_DISCOVERY_OUTSTANDING;

                  /* Clear the CCCDs stored for this device.            */
                  DeviceInfo->ServerInfo.Rx_Credit_Client_Configuration_Descriptor = 0;
                  DeviceInfo->ServerInfo.Tx_Client_Configuration_Descriptor        = 0;

                  /* If this device is not paired, then delete it.  The */
                  /* link will be encrypted if the device is paired.    */
                  if(!(DeviceInfo->Flags & DEVICE_INFO_FLAGS_LINK_ENCRYPTED))
                  {
                     if((DeviceInfo = DeleteDeviceInfoEntry(&DeviceInfoList, GAP_LE_Event_Data->Event_Data.GAP_LE_Disconnection_Complete_Event_Data->Peer_Address)) != NULL)
                        FreeDeviceInfoEntryMemory(DeviceInfo);
                  }
                  else
                  {
                     /* Flag that the Link is no longer encrypted since */
                     /* we have disconnected.                           */
                     DeviceInfo->Flags &= ~DEVICE_INFO_FLAGS_LINK_ENCRYPTED;
                  }
               }
            }
            break;
         case etLE_Connection_Parameter_Update_Request:
            Display(("\r\netLE_Connection_Parameter_Update_Request with size %d.\r\n", (int)GAP_LE_Event_Data->Event_Data_Size));

            if(GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Parameter_Update_Request_Event_Data)
            {
               BD_ADDRToStr(GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Parameter_Update_Request_Event_Data->BD_ADDR, BoardStr);
               Display(("   BD_ADDR:             %s.\r\n", BoardStr));
               Display(("   Minimum Interval:    %u.\r\n", (unsigned int)GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Parameter_Update_Request_Event_Data->Conn_Interval_Min));
               Display(("   Maximum Interval:    %u.\r\n", (unsigned int)GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Parameter_Update_Request_Event_Data->Conn_Interval_Max));
               Display(("   Slave Latency:       %u.\r\n", (unsigned int)GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Parameter_Update_Request_Event_Data->Slave_Latency));
               Display(("   Supervision Timeout: %u.\r\n", (unsigned int)GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Parameter_Update_Request_Event_Data->Conn_Supervision_Timeout));

               /* Initialize the connection parameters.                 */
               ConnectionParameters.Connection_Interval_Min    = GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Parameter_Update_Request_Event_Data->Conn_Interval_Min;
               ConnectionParameters.Connection_Interval_Max    = GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Parameter_Update_Request_Event_Data->Conn_Interval_Max;
               ConnectionParameters.Slave_Latency              = GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Parameter_Update_Request_Event_Data->Slave_Latency;
               ConnectionParameters.Supervision_Timeout        = GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Parameter_Update_Request_Event_Data->Conn_Supervision_Timeout;
               ConnectionParameters.Minimum_Connection_Length  = 0;
               ConnectionParameters.Maximum_Connection_Length  = 10000;

               Display(("\r\nAttempting to accept connection parameter update request.\r\n"));

               /* Go ahead and accept whatever the slave has requested. */
               Result = GAP_LE_Connection_Parameter_Update_Response(BluetoothStackID, GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Parameter_Update_Request_Event_Data->BD_ADDR, TRUE, &ConnectionParameters);
               if(!Result)
               {
                  Display(("      GAP_LE_Connection_Parameter_Update_Response() success.\r\n"));
               }
               else
               {
                  Display(("      GAP_LE_Connection_Parameter_Update_Response() error %d.\r\n", Result));
               }
            }
            break;
         case etLE_Connection_Parameter_Updated:
            Display(("\r\netLE_Connection_Parameter_Updated with size %d.\r\n", (int)GAP_LE_Event_Data->Event_Data_Size));

            if(GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Parameter_Updated_Event_Data)
            {
               BD_ADDRToStr(GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Parameter_Updated_Event_Data->BD_ADDR, BoardStr);
               Display(("   Status:              0x%02X.\r\n", GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Parameter_Updated_Event_Data->Status));
               Display(("   BD_ADDR:             %s.\r\n", BoardStr));

               if(GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Parameter_Updated_Event_Data->Status == HCI_ERROR_CODE_NO_ERROR)
               {
                  Display(("   Connection Interval: %u.\r\n", (unsigned int)GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Parameter_Updated_Event_Data->Current_Connection_Parameters.Connection_Interval));
                  Display(("   Slave Latency:       %u.\r\n", (unsigned int)GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Parameter_Updated_Event_Data->Current_Connection_Parameters.Slave_Latency));
                  Display(("   Supervision Timeout: %u.\r\n", (unsigned int)GAP_LE_Event_Data->Event_Data.GAP_LE_Connection_Parameter_Updated_Event_Data->Current_Connection_Parameters.Supervision_Timeout));
               }
            }
            break;
         case etLE_Encryption_Change:
            Display(("\r\netLE_Encryption_Change with size %d.\r\n",(int)GAP_LE_Event_Data->Event_Data_Size));

            /* Search for the device entry to see flag if the link is   */
            /* encrypted.                                               */
            if((DeviceInfo = SearchDeviceInfoEntryByBD_ADDR(&DeviceInfoList, GAP_LE_Event_Data->Event_Data.GAP_LE_Encryption_Change_Event_Data->BD_ADDR)) != NULL)
            {
               /* Check to see if the encryption change was successful. */
               if((GAP_LE_Event_Data->Event_Data.GAP_LE_Encryption_Change_Event_Data->Encryption_Change_Status == HCI_ERROR_CODE_NO_ERROR) && (GAP_LE_Event_Data->Event_Data.GAP_LE_Encryption_Change_Event_Data->Encryption_Mode == emEnabled))
                  DeviceInfo->Flags |= DEVICE_INFO_FLAGS_LINK_ENCRYPTED;
               else
                  DeviceInfo->Flags &= ~DEVICE_INFO_FLAGS_LINK_ENCRYPTED;
            }
            break;
         case etLE_Encryption_Refresh_Complete:
            Display(("\r\netLE_Encryption_Refresh_Complete with size %d.\r\n", (int)GAP_LE_Event_Data->Event_Data_Size));

            /* Search for the device entry to see flag if the link is   */
            /* encrypted.                                               */
            if((DeviceInfo = SearchDeviceInfoEntryByBD_ADDR(&DeviceInfoList, GAP_LE_Event_Data->Event_Data.GAP_LE_Encryption_Refresh_Complete_Event_Data->BD_ADDR)) != NULL)
            {
               /* Check to see if the refresh was successful.           */
               if(GAP_LE_Event_Data->Event_Data.GAP_LE_Encryption_Refresh_Complete_Event_Data->Status == HCI_ERROR_CODE_NO_ERROR)
                  DeviceInfo->Flags |= DEVICE_INFO_FLAGS_LINK_ENCRYPTED;
               else
                  DeviceInfo->Flags &= ~DEVICE_INFO_FLAGS_LINK_ENCRYPTED;
            }
            break;
         case etLE_Authentication:
            Display(("\r\netLE_Authentication with size %d.\r\n", (int)GAP_LE_Event_Data->Event_Data_Size));

            /* Make sure the authentication event data is valid before  */
            /* continueing.                                             */
            if((Authentication_Event_Data = GAP_LE_Event_Data->Event_Data.GAP_LE_Authentication_Event_Data) != NULL)
            {
               BD_ADDRToStr(Authentication_Event_Data->BD_ADDR, BoardStr);

               switch(Authentication_Event_Data->GAP_LE_Authentication_Event_Type)
               {
                  case latLongTermKeyRequest:
                     Display(("    latKeyRequest: \r\n"));
                     Display(("      BD_ADDR: %s.\r\n", BoardStr));

                     /* The other side of a connection is requesting    */
                     /* that we start encryption. Thus we should        */
                     /* regenerate LTK for this connection and send it  */
                     /* to the chip.                                    */
                     Result = GAP_LE_Regenerate_Long_Term_Key(BluetoothStackID, (Encryption_Key_t *)(&DHK), (Encryption_Key_t *)(&ER), Authentication_Event_Data->Authentication_Event_Data.Long_Term_Key_Request.EDIV, &(Authentication_Event_Data->Authentication_Event_Data.Long_Term_Key_Request.Rand), &GeneratedLTK);
                     if(!Result)
                     {
                        Display(("      GAP_LE_Regenerate_Long_Term_Key Success.\r\n"));

                        /* Respond with the Re-Generated Long Term Key. */
                        GAP_LE_Authentication_Response_Information.GAP_LE_Authentication_Type                                        = larLongTermKey;
                        GAP_LE_Authentication_Response_Information.Authentication_Data_Length                                        = GAP_LE_LONG_TERM_KEY_INFORMATION_DATA_SIZE;
                        GAP_LE_Authentication_Response_Information.Authentication_Data.Long_Term_Key_Information.Encryption_Key_Size = GAP_LE_MAXIMUM_ENCRYPTION_KEY_SIZE;
                        GAP_LE_Authentication_Response_Information.Authentication_Data.Long_Term_Key_Information.Long_Term_Key       = GeneratedLTK;
                     }
                     else
                     {
                        Display(("      GAP_LE_Regenerate_Long_Term_Key returned %d.\r\n",Result));

                        /* Since we failed to generate the requested key*/
                        /* we should respond with a negative response.  */
                        GAP_LE_Authentication_Response_Information.GAP_LE_Authentication_Type = larLongTermKey;
                        GAP_LE_Authentication_Response_Information.Authentication_Data_Length = 0;
                     }

                     /* Send the Authentication Response.               */
                     Result = GAP_LE_Authentication_Response(BluetoothStackID, Authentication_Event_Data->BD_ADDR, &GAP_LE_Authentication_Response_Information);
                     if(Result)
                     {
                        Display(("      GAP_LE_Authentication_Response returned %d.\r\n",Result));
                     }
                     break;
                  case latSecurityRequest:
                     /* Display the data for this event.                */
                     /* * NOTE * This is only sent from Slave to Master.*/
                     /*          Thus we must be the Master in this     */
                     /*          connection.                            */
                     Display(("    latSecurityRequest:.\r\n"));
                     Display(("      BD_ADDR: %s.\r\n", BoardStr));
                     Display(("      Bonding Type: %s.\r\n", ((Authentication_Event_Data->Authentication_Event_Data.Security_Request.Bonding_Type == lbtBonding)?"Bonding":"No Bonding")));
                     Display(("      MITM: %s.\r\n", ((Authentication_Event_Data->Authentication_Event_Data.Security_Request.MITM == TRUE)?"YES":"NO")));

                     /* Determine if we have previously paired with the */
                     /* device. If we have paired we will attempt to    */
                     /* re-establish security using a previously        */
                     /* exchanged LTK.                                  */
                     if((DeviceInfo = SearchDeviceInfoEntryByBD_ADDR(&DeviceInfoList, Authentication_Event_Data->BD_ADDR)) != NULL)
                     {
                        /* Determine if a Valid Long Term Key is stored */
                        /* for this device.                             */
                        if(DeviceInfo->Flags & DEVICE_INFO_FLAGS_LTK_VALID)
                        {
                           Display(("Attempting to Re-Establish Security.\r\n"));

                           /* Attempt to re-establish security to this  */
                           /* device.                                   */
                           GAP_LE_Security_Information.Local_Device_Is_Master                                      = TRUE;
                           GAP_LE_Security_Information.Security_Information.Master_Information.LTK                 = DeviceInfo->LTK;
                           GAP_LE_Security_Information.Security_Information.Master_Information.EDIV                = DeviceInfo->EDIV;
                           GAP_LE_Security_Information.Security_Information.Master_Information.Rand                = DeviceInfo->Rand;
                           GAP_LE_Security_Information.Security_Information.Master_Information.Encryption_Key_Size = DeviceInfo->EncryptionKeySize;

                           Result = GAP_LE_Reestablish_Security(BluetoothStackID, Authentication_Event_Data->BD_ADDR, &GAP_LE_Security_Information, GAP_LE_Event_Callback, 0);
                           if(Result)
                           {
                              Display(("GAP_LE_Reestablish_Security returned %d.\r\n",Result));
                           }
                        }
                        else
                        {

                           /* We do not have a stored Link Key for this */
                           /* device so go ahead and pair to this       */
                           /* device.                                   */
                           SendPairingRequest(Authentication_Event_Data->BD_ADDR, TRUE);
                        }
                     }
                     else
                     {

                        /* There is no Key Info Entry for this device   */
                        /* so we will just treat this as a slave        */
                        /* request and initiate pairing.                */
                        SendPairingRequest(Authentication_Event_Data->BD_ADDR, TRUE);
                     }

                     break;
                  case latPairingRequest:

                     Display(("Pairing Request: %s.\r\n",BoardStr));
                     DisplayPairingInformation(Authentication_Event_Data->Authentication_Event_Data.Pairing_Request);

                     /* This is a pairing request. Respond with a       */
                     /* Pairing Response.                               */
                     /* * NOTE * This is only sent from Master to Slave.*/
                     /*          Thus we must be the Slave in this      */
                     /*          connection.                            */

                     /* Send the Pairing Response.                      */
                     SlavePairingRequestResponse(Authentication_Event_Data->BD_ADDR);
                     break;
                  case latConfirmationRequest:
                     Display(("latConfirmationRequest.\r\n"));

                     if(Authentication_Event_Data->Authentication_Event_Data.Confirmation_Request.Request_Type == crtNone)
                     {
                        Display(("Invoking Just Works.\r\n"));

                        /* Just Accept Just Works Pairing.              */
                        GAP_LE_Authentication_Response_Information.GAP_LE_Authentication_Type = larConfirmation;

                        /* By setting the Authentication_Data_Length to */
                        /* any NON-ZERO value we are informing the GAP  */
                        /* LE Layer that we are accepting Just Works    */
                        /* Pairing.                                     */
                        GAP_LE_Authentication_Response_Information.Authentication_Data_Length = DWORD_SIZE;

                        Result = GAP_LE_Authentication_Response(BluetoothStackID, Authentication_Event_Data->BD_ADDR, &GAP_LE_Authentication_Response_Information);
                        if(Result)
                        {
                           Display(("GAP_LE_Authentication_Response returned %d.\r\n",Result));
                        }
                     }
                     else
                     {
                        if(Authentication_Event_Data->Authentication_Event_Data.Confirmation_Request.Request_Type == crtPasskey)
                        {
                           Display(("Call LEPasskeyResponse [PASSCODE].\r\n"));
                        }
                        else
                        {
                           if(Authentication_Event_Data->Authentication_Event_Data.Confirmation_Request.Request_Type == crtDisplay)
                           {
                              Display(("Passkey: %06l.\r\n", Authentication_Event_Data->Authentication_Event_Data.Confirmation_Request.Display_Passkey));
                           }
                        }
                     }
                     break;
                  case latSecurityEstablishmentComplete:
                     Display(("Security Re-Establishment Complete: %s.\r\n", BoardStr));
                     Display(("                            Status: 0x%02X.\r\n", Authentication_Event_Data->Authentication_Event_Data.Security_Establishment_Complete.Status));
                     break;
                  case latPairingStatus:

                     Display(("Pairing Status: %s.\r\n", BoardStr));
                     Display(("        Status: 0x%02X.\r\n", Authentication_Event_Data->Authentication_Event_Data.Pairing_Status.Status));

                     if(Authentication_Event_Data->Authentication_Event_Data.Pairing_Status.Status == GAP_LE_PAIRING_STATUS_NO_ERROR)
                     {
                        Display(("        Key Size: %d.\r\n", Authentication_Event_Data->Authentication_Event_Data.Pairing_Status.Negotiated_Encryption_Key_Size));
                     }
                     else
                     {
                        /* Failed to pair so delete the key entry for   */
                        /* this device and disconnect the link.         */
                        if((DeviceInfo = DeleteDeviceInfoEntry(&DeviceInfoList, Authentication_Event_Data->BD_ADDR)) != NULL)
                           FreeDeviceInfoEntryMemory(DeviceInfo);

                        /* Disconnect the Link.                         */
                        GAP_LE_Disconnect(BluetoothStackID, Authentication_Event_Data->BD_ADDR);
                     }
                     break;
                  case latEncryptionInformationRequest:
                     Display(("Encryption Information Request %s.\r\n", BoardStr));

                     /* Generate new LTK,EDIV and Rand and respond with */
                     /* them.                                           */
                     EncryptionInformationRequestResponse(Authentication_Event_Data->BD_ADDR, Authentication_Event_Data->Authentication_Event_Data.Encryption_Request_Information.Encryption_Key_Size, &GAP_LE_Authentication_Response_Information);
                     break;
                  case latEncryptionInformation:
                     /* Display the information from the event.         */
                     Display((" Encryption Information from RemoteDevice: %s.\r\n", BoardStr));
                     Display(("                             Key Size: %d.\r\n", Authentication_Event_Data->Authentication_Event_Data.Encryption_Information.Encryption_Key_Size));

                     /* ** NOTE ** If we are the Slave we will NOT      */
                     /*            store the LTK that is sent to us by  */
                     /*            the Master.  However if it was ever  */
                     /*            desired that the Master and Slave    */
                     /*            switch roles in a later connection   */
                     /*            we could store that information at   */
                     /*            this point.                          */
                     if(LocalDeviceIsMaster)
                     {
                        /* Search for the entry for this slave to store */
                        /* the information into.                        */
                        if((DeviceInfo = SearchDeviceInfoEntryByBD_ADDR(&DeviceInfoList, Authentication_Event_Data->BD_ADDR)) != NULL)
                        {
                           DeviceInfo->LTK               = Authentication_Event_Data->Authentication_Event_Data.Encryption_Information.LTK;
                           DeviceInfo->EDIV              = Authentication_Event_Data->Authentication_Event_Data.Encryption_Information.EDIV;
                           DeviceInfo->Rand              = Authentication_Event_Data->Authentication_Event_Data.Encryption_Information.Rand;
                           DeviceInfo->EncryptionKeySize = Authentication_Event_Data->Authentication_Event_Data.Encryption_Information.Encryption_Key_Size;
                           DeviceInfo->Flags            |= DEVICE_INFO_FLAGS_LTK_VALID;
                        }
                        else
                        {
                           Display(("No Key Info Entry for this Slave.\r\n"));
                        }
                     }
                     break;
               }
            }
            break;
      }

   }
}

   /* The following function is for an GATT Client Event Callback.  This*/
   /* function will be called whenever a GATT Response is received for a*/
   /* request that was made when this function was registered.  This    */
   /* function passes to the caller the GATT Client Event Data that     */
   /* occurred and the GATT Client Event Callback Parameter that was    */
   /* specified when this Callback was installed.  The caller is free to*/
   /* use the contents of the GATT Client Event Data ONLY in the context*/
   /* of this callback.  If the caller requires the Data for a longer   */
   /* period of time, then the callback function MUST copy the data into*/
   /* another Data Buffer.  This function is guaranteed NOT to be       */
   /* invoked more than once simultaneously for the specified installed */
   /* callback (i.e.  this function DOES NOT have be reentrant).  It    */
   /* Needs to be noted however, that if the same Callback is installed */
   /* more than once, then the callbacks will be called serially.       */
   /* Because of this, the processing in this function should be as     */
   /* efficient as possible.  It should also be noted that this function*/
   /* is called in the Thread Context of a Thread that the User does NOT*/
   /* own.  Therefore, processing in this function should be as         */
   /* efficient as possible (this argument holds anyway because another */
   /* GATT Event (Server/Client or Connection) will not be processed    */
   /* while this function call is outstanding).                         */
   /* * NOTE * This function MUST NOT Block and wait for Events that can*/
   /*          only be satisfied by Receiving a Bluetooth Event         */
   /*          Callback.  A Deadlock WILL occur because NO Bluetooth    */
   /*          Callbacks will be issued while this function is currently*/
   /*          outstanding.                                             */
static void BTPSAPI GATT_ClientEventCallback_SPPLE(unsigned int BluetoothStackID, GATT_Client_Event_Data_t *GATT_Client_Event_Data, unsigned long CallbackParameter)
{
   int           LEConnectionIndex;
   Word_t        Credits;
   BoardStr_t    BoardStr;
   DeviceInfo_t *DeviceInfo;

   /* Verify that all parameters to this callback are Semi-Valid.       */
   if((BluetoothStackID) && (GATT_Client_Event_Data))
   {
      /* Determine the event that occurred.                             */
      switch(GATT_Client_Event_Data->Event_Data_Type)
      {
         case etGATT_Client_Error_Response:
            if(GATT_Client_Event_Data->Event_Data.GATT_Request_Error_Data)
            {
               Display(("\r\nError Response.\r\n"));
               BD_ADDRToStr(GATT_Client_Event_Data->Event_Data.GATT_Request_Error_Data->RemoteDevice, BoardStr);
               Display(("Connection ID:   %u.\r\n", GATT_Client_Event_Data->Event_Data.GATT_Request_Error_Data->ConnectionID));
               Display(("Transaction ID:  %u.\r\n", GATT_Client_Event_Data->Event_Data.GATT_Request_Error_Data->TransactionID));
               Display(("Connection Type: %s.\r\n", (GATT_Client_Event_Data->Event_Data.GATT_Request_Error_Data->ConnectionType == gctLE)?"LE":"BR/EDR"));
               Display(("BD_ADDR:         %s.\r\n", BoardStr));
               Display(("Error Type:      %s.\r\n", (GATT_Client_Event_Data->Event_Data.GATT_Request_Error_Data->ErrorType == retErrorResponse)?"Response Error":"Response Timeout"));

               /* Only print out the rest if it is valid.               */
               if(GATT_Client_Event_Data->Event_Data.GATT_Request_Error_Data->ErrorType == retErrorResponse)
               {
                  Display(("Request Opcode:  0x%02X.\r\n", GATT_Client_Event_Data->Event_Data.GATT_Request_Error_Data->RequestOpCode));
                  Display(("Request Handle:  0x%04X.\r\n", GATT_Client_Event_Data->Event_Data.GATT_Request_Error_Data->RequestHandle));
                  Display(("Error Code:      0x%02X.\r\n", GATT_Client_Event_Data->Event_Data.GATT_Request_Error_Data->ErrorCode));

                  if(GATT_Client_Event_Data->Event_Data.GATT_Request_Error_Data->ErrorCode < NUMBER_GATT_ERROR_CODES)
                  {
                     Display(("Error Mesg:      %s.\r\n", ErrorCodeStr[GATT_Client_Event_Data->Event_Data.GATT_Request_Error_Data->ErrorCode]));
                  }
                  else
                  {
                     Display(("Error Mesg:      Unknown.\r\n", ErrorCodeStr[GATT_Client_Event_Data->Event_Data.GATT_Request_Error_Data->ErrorCode]));
                  }
               }
            }
            else
               Display(("Error - Null Error Response Data.\r\n"));
            break;
         case etGATT_Client_Read_Response:
            if(GATT_Client_Event_Data->Event_Data.GATT_Read_Response_Data)
            {
               /* Find the LE Connection Index for this connection.     */
               if((LEConnectionIndex = FindLEIndexByAddress(GATT_Client_Event_Data->Event_Data.GATT_Read_Response_Data->RemoteDevice)) >= 0)
               {
                  /* Grab the device info for the currently connected   */
                  /* device.                                            */
                  if((DeviceInfo = SearchDeviceInfoEntryByBD_ADDR(&DeviceInfoList, LEContextInfo[LEConnectionIndex].ConnectionBD_ADDR)) != NULL)
                  {
                     if((Word_t)CallbackParameter == DeviceInfo->ClientInfo.Rx_Credit_Characteristic)
                     {
                        /* Make sure this is the correct size for a Rx  */
                        /* Credit Characteristic.                       */
                        if(GATT_Client_Event_Data->Event_Data.GATT_Read_Response_Data->AttributeValueLength == WORD_SIZE)
                        {
                           /* Display the credits we just received.     */
                           Credits = READ_UNALIGNED_WORD_LITTLE_ENDIAN(GATT_Client_Event_Data->Event_Data.GATT_Read_Response_Data->AttributeValue);
                           Display(("\r\nReceived %u Initial Credits.\r\n", Credits));

                           /* We have received the initial credits from */
                           /* the device so go ahead and handle a       */
                           /* Receive Credit Event.                     */
                           SPPLEReceiveCreditEvent(&(LEContextInfo[LEConnectionIndex]), DeviceInfo, Credits);
                        }
                     }
                  }
               }
            }
            else
               Display(("\r\nError - Null Read Response Data.\r\n"));
            break;
         case etGATT_Client_Exchange_MTU_Response:
            if(GATT_Client_Event_Data->Event_Data.GATT_Exchange_MTU_Response_Data)
            {
               Display(("\r\nExchange MTU Response.\r\n"));
               BD_ADDRToStr(GATT_Client_Event_Data->Event_Data.GATT_Exchange_MTU_Response_Data->RemoteDevice, BoardStr);
               Display(("Connection ID:   %u.\r\n", GATT_Client_Event_Data->Event_Data.GATT_Exchange_MTU_Response_Data->ConnectionID));
               Display(("Transaction ID:  %u.\r\n", GATT_Client_Event_Data->Event_Data.GATT_Exchange_MTU_Response_Data->TransactionID));
               Display(("Connection Type: %s.\r\n", (GATT_Client_Event_Data->Event_Data.GATT_Exchange_MTU_Response_Data->ConnectionType == gctLE)?"LE":"BR/EDR"));
               Display(("BD_ADDR:         %s.\r\n", BoardStr));
               Display(("MTU:             %u.\r\n", GATT_Client_Event_Data->Event_Data.GATT_Exchange_MTU_Response_Data->ServerMTU));
            }
            else
               Display(("\r\nError - Null Write Response Data.\r\n"));
            break;
         case etGATT_Client_Write_Response:
            if(GATT_Client_Event_Data->Event_Data.GATT_Write_Response_Data)
            {
               Display(("\r\nWrite Response.\r\n"));
               BD_ADDRToStr(GATT_Client_Event_Data->Event_Data.GATT_Write_Response_Data->RemoteDevice, BoardStr);
               Display(("Connection ID:   %u.\r\n", GATT_Client_Event_Data->Event_Data.GATT_Write_Response_Data->ConnectionID));
               Display(("Transaction ID:  %u.\r\n", GATT_Client_Event_Data->Event_Data.GATT_Write_Response_Data->TransactionID));
               Display(("Connection Type: %s.\r\n", (GATT_Client_Event_Data->Event_Data.GATT_Write_Response_Data->ConnectionType == gctLE)?"LE":"BR/EDR"));
               Display(("BD_ADDR:         %s.\r\n", BoardStr));
               Display(("Bytes Written:   %u.\r\n", GATT_Client_Event_Data->Event_Data.GATT_Write_Response_Data->BytesWritten));
            }
            else
               Display(("\r\nError - Null Write Response Data.\r\n"));
            break;
      }

   }
   else
   {

      Display(("GATT Callback Data: Event_Data = NULL.\r\n"));

   }
}


   /* The following function is for an GATT Connection Event Callback.  */
   /* This function is called for GATT Connection Events that occur on  */
   /* the specified Bluetooth Stack.  This function passes to the caller*/
   /* the GATT Connection Event Data that occurred and the GATT         */
   /* Connection Event Callback Parameter that was specified when this  */
   /* Callback was installed.  The caller is free to use the contents of*/
   /* the GATT Client Event Data ONLY in the context of this callback.  */
   /* If the caller requires the Data for a longer period of time, then */
   /* the callback function MUST copy the data into another Data Buffer.*/
   /* This function is guaranteed NOT to be invoked more than once      */
   /* simultaneously for the specified installed callback (i.e.  this   */
   /* function DOES NOT have be reentrant).  It Needs to be noted       */
   /* however, that if the same Callback is installed more than once,   */
   /* then the callbacks will be called serially.  Because of this, the */
   /* processing in this function should be as efficient as possible.   */
   /* It should also be noted that this function is called in the Thread*/
   /* Context of a Thread that the User does NOT own.  Therefore,       */
   /* processing in this function should be as efficient as possible    */
   /* (this argument holds anyway because another GATT Event            */
   /* (Server/Client or Connection) will not be processed while this    */
   /* function call is outstanding).                                    */
   /* * NOTE * This function MUST NOT Block and wait for Events that can*/
   /*          only be satisfied by Receiving a Bluetooth Event         */
   /*          Callback.  A Deadlock WILL occur because NO Bluetooth    */
   /*          Callbacks will be issued while this function is currently*/
   /*          outstanding.                                             */
static void BTPSAPI GATT_Connection_Event_Callback(unsigned int BluetoothStackID, GATT_Connection_Event_Data_t *GATT_Connection_Event_Data, unsigned long CallbackParameter)
{
   int           LEConnectionIndex;
   Word_t        Credits;
   BoardStr_t    BoardStr;
   DeviceInfo_t *DeviceInfo;

   /* Verify that all parameters to this callback are Semi-Valid.       */
   if((BluetoothStackID) && (GATT_Connection_Event_Data))
   {
      /* Determine the Connection Event that occurred.                  */
      switch(GATT_Connection_Event_Data->Event_Data_Type)
      {
         case etGATT_Connection_Device_Connection:
            if(GATT_Connection_Event_Data->Event_Data.GATT_Device_Connection_Data)
            {
               /* Update the ConnectionID associated with the BD_ADDR   */
               /* If UpdateConnectionID returns -1, then it failed.     */
               if(UpdateConnectionID(GATT_Connection_Event_Data->Event_Data.GATT_Device_Connection_Data->ConnectionID, GATT_Connection_Event_Data->Event_Data.GATT_Device_Connection_Data->RemoteDevice) < 0)
                   Display(("Error - No matching ConnectionBD_ADDR found."));

               Display(("\r\netGATT_Connection_Device_Connection with size %u: \r\n", GATT_Connection_Event_Data->Event_Data_Size));
               BD_ADDRToStr(GATT_Connection_Event_Data->Event_Data.GATT_Device_Connection_Data->RemoteDevice, BoardStr);
               Display(("   Connection ID:   %u.\r\n", GATT_Connection_Event_Data->Event_Data.GATT_Device_Connection_Data->ConnectionID));
               Display(("   Connection Type: %s.\r\n", ((GATT_Connection_Event_Data->Event_Data.GATT_Device_Connection_Data->ConnectionType == gctLE)?"LE":"BR/EDR")));
               Display(("   Remote Device:   %s.\r\n", BoardStr));
               Display(("   Connection MTU:  %u.\r\n", GATT_Connection_Event_Data->Event_Data.GATT_Device_Connection_Data->MTU));

               /* Find the LE Connection Index for this connection.     */
               if((LEConnectionIndex = FindLEIndexByAddress(GATT_Connection_Event_Data->Event_Data.GATT_Device_Connection_Data->RemoteDevice)) >= 0)
               {
                  /* Search for the device info for the connection.     */
                  if((DeviceInfo = SearchDeviceInfoEntryByBD_ADDR(&DeviceInfoList, LEContextInfo[LEConnectionIndex].ConnectionBD_ADDR)) != NULL)
                  {
                     /* Clear the SPPLE Role Flag.                      */
                     DeviceInfo->Flags &= ~DEVICE_INFO_FLAGS_SPPLE_SERVER;

                     /* Initialize the Transmit and Receive Buffers.    */
                     InitializeBuffer(&(LEContextInfo[LEConnectionIndex].SPPLEBufferInfo.ReceiveBuffer));
                     InitializeBuffer(&(LEContextInfo[LEConnectionIndex].SPPLEBufferInfo.TransmitBuffer));

                     /* Flag that we do not have any transmit credits   */
                     /* yet.                                            */
                     LEContextInfo[LEConnectionIndex].SPPLEBufferInfo.TransmitCredits = 0;

                     /* Flag that no credits are queued.                */
                     LEContextInfo[LEConnectionIndex].SPPLEBufferInfo.QueuedCredits   = 0;

                     if(!LocalDeviceIsMaster)
                     {
                        /* Flag that we will act as the Server.         */
                        DeviceInfo->Flags |= DEVICE_INFO_FLAGS_SPPLE_SERVER;

                        /* Send the Initial Credits if the Rx Credit CCD*/
                        /* is already configured (for a bonded device   */
                        /* this could be the case).                     */
                        SPPLESendCredits(&(LEContextInfo[LEConnectionIndex]), DeviceInfo, LEContextInfo[LEConnectionIndex].SPPLEBufferInfo.ReceiveBuffer.BytesFree);
                     }
                     else
                     {
                        /* Attempt to update the MTU to the maximum     */
                        /* supported.                                   */
                        GATT_Exchange_MTU_Request(BluetoothStackID, GATT_Connection_Event_Data->Event_Data.GATT_Device_Connection_Data->ConnectionID, BTPS_CONFIGURATION_GATT_MAXIMUM_SUPPORTED_MTU_SIZE, GATT_ClientEventCallback_SPPLE, 0);
                     }
                  }
               }
            }
            else
               Display(("Error - Null Connection Data.\r\n"));
            break;
         case etGATT_Connection_Device_Disconnection:
            if(GATT_Connection_Event_Data->Event_Data.GATT_Device_Disconnection_Data)
            {
               /* Clear the Connection ID.                              */
               RemoveConnectionInfo(GATT_Connection_Event_Data->Event_Data.GATT_Device_Disconnection_Data->RemoteDevice);

               Display(("\r\netGATT_Connection_Device_Disconnection with size %u: \r\n", GATT_Connection_Event_Data->Event_Data_Size));
               BD_ADDRToStr(GATT_Connection_Event_Data->Event_Data.GATT_Device_Disconnection_Data->RemoteDevice, BoardStr);
               Display(("   Connection ID:   %u.\r\n", GATT_Connection_Event_Data->Event_Data.GATT_Device_Disconnection_Data->ConnectionID));
               Display(("   Connection Type: %s.\r\n", ((GATT_Connection_Event_Data->Event_Data.GATT_Device_Disconnection_Data->ConnectionType == gctLE)?"LE":"BR/EDR")));
               Display(("   Remote Device:   %s.\r\n", BoardStr));
            }
            else
               Display(("Error - Null Disconnection Data.\r\n"));
            break;
         case etGATT_Connection_Device_Buffer_Empty:
            if(GATT_Connection_Event_Data->Event_Data.GATT_Device_Buffer_Empty_Data)
            {
               /* Find the LE Connection Index for this connection.     */
               if((LEConnectionIndex = FindLEIndexByAddress(GATT_Connection_Event_Data->Event_Data.GATT_Device_Buffer_Empty_Data->RemoteDevice)) >= 0)
               {
                  /* Grab the device info for the currently connected   */
                  /* device.                                            */
                  if((DeviceInfo = SearchDeviceInfoEntryByBD_ADDR(&DeviceInfoList, LEContextInfo[LEConnectionIndex].ConnectionBD_ADDR)) != NULL)
                  {
   //xxx
   //xxx                  Display(("xxx Buffer Empty for device.\r\n"));

                     /* Flag that the buffer is no longer empty.        */
                     LEContextInfo[LEConnectionIndex].BufferFull = FALSE;

                     /* Attempt to send any queued credits that we may  */
                     /* have.                                           */
                     SPPLESendCredits(&(LEContextInfo[LEConnectionIndex]), DeviceInfo, 0);

                     /* If may be possible for transmit queued data now.*/
                     /* So fake a Receive Credit event with 0 as the    */
                     /* received credits.                               */
                     SPPLEReceiveCreditEvent(&(LEContextInfo[LEConnectionIndex]), DeviceInfo, 0);

                  }
               }
            }
            break;
         case etGATT_Connection_Server_Notification:
            if(GATT_Connection_Event_Data->Event_Data.GATT_Server_Notification_Data)
            {
               /* Find the LE Connection Index for this connection.     */
               if((LEConnectionIndex = FindLEIndexByAddress(GATT_Connection_Event_Data->Event_Data.GATT_Server_Notification_Data->RemoteDevice)) >= 0)
               {
                  /* Find the Device Info for the device that has sent  */
                  /* us the notification.                               */
                  if((DeviceInfo = SearchDeviceInfoEntryByBD_ADDR(&DeviceInfoList, LEContextInfo[LEConnectionIndex].ConnectionBD_ADDR)) != NULL)
                  {
                     /* Determine the characteristic that is being      */
                     /* notified.                                       */
                     if(GATT_Connection_Event_Data->Event_Data.GATT_Server_Notification_Data->AttributeHandle == DeviceInfo->ClientInfo.Rx_Credit_Characteristic)
                     {
                        /* Verify that the length of the Rx Credit      */
                        /* Notification is correct.                     */
                        if(GATT_Connection_Event_Data->Event_Data.GATT_Server_Notification_Data->AttributeValueLength == WORD_SIZE)
                        {
                           Credits = READ_UNALIGNED_WORD_LITTLE_ENDIAN(GATT_Connection_Event_Data->Event_Data.GATT_Server_Notification_Data->AttributeValue);

                           /* Handle the received credits event.        */
                           SPPLEReceiveCreditEvent(&(LEContextInfo[LEConnectionIndex]), DeviceInfo, Credits);

                        }
                     }
                     else
                     {
                        if(GATT_Connection_Event_Data->Event_Data.GATT_Server_Notification_Data->AttributeHandle == DeviceInfo->ClientInfo.Tx_Characteristic)
                        {
                           /* This is a Tx Characteristic Event.  So    */
                           /* call the function to handle the data      */
                           /* indication event.                         */
                           SPPLEDataIndicationEvent(&(LEContextInfo[LEConnectionIndex]), DeviceInfo, GATT_Connection_Event_Data->Event_Data.GATT_Server_Notification_Data->AttributeValueLength, GATT_Connection_Event_Data->Event_Data.GATT_Server_Notification_Data->AttributeValue);

                        }
                     }
                  }
               }
            }
            else
               Display(("Error - Null Server Notification Data.\r\n"));
            break;
      }

   }
   else
   {

      Display(("GATT Connection Callback Data: Event_Data = NULL.\r\n"));

   }
}

   /* The following function is for the GAP Event Receive Data Callback.*/
   /* This function will be called whenever a Callback has been         */
   /* registered for the specified GAP Action that is associated with   */
   /* the Bluetooth Stack.  This function passes to the caller the GAP  */
   /* Event Data of the specified Event and the GAP Event Callback      */
   /* Parameter that was specified when this Callback was installed.    */
   /* The caller is free to use the contents of the GAP Event Data ONLY */
   /* in the context of this callback.  If the caller requires the Data */
   /* for a longer period of time, then the callback function MUST copy */
   /* the data into another Data Buffer.  This function is guaranteed   */
   /* NOT to be invoked more than once simultaneously for the specified */
   /* installed callback (i.e.  this function DOES NOT have be          */
   /* reentrant).  It Needs to be noted however, that if the same       */
   /* Callback is installed more than once, then the callbacks will be  */
   /* called serially.  Because of this, the processing in this function*/
   /* should be as efficient as possible.  It should also be noted that */
   /* this function is called in the Thread Context of a Thread that the*/
   /* User does NOT own.  Therefore, processing in this function should */
   /* be as efficient as possible (this argument holds anyway because   */
   /* other GAP Events will not be processed while this function call is*/
   /* outstanding).                                                     */
   /* * NOTE * This function MUST NOT Block and wait for events that    */
   /*          can only be satisfied by Receiving other GAP Events.  A  */
   /*          Deadlock WILL occur because NO GAP Event Callbacks will  */
   /*          be issued while this function is currently outstanding.  */
static void BTPSAPI GAP_Event_Callback(unsigned int BluetoothStackID, GAP_Event_Data_t *GAP_Event_Data, unsigned long CallbackParameter)
{
   int                               Result;
   int                               Index;
   BD_ADDR_t                         NULL_BD_ADDR;
   Boolean_t                         OOB_Data;
   Boolean_t                         MITM;
   BoardStr_t                        Callback_BoardStr;
   GAP_IO_Capability_t               RemoteIOCapability;
   GAP_Inquiry_Event_Data_t         *GAP_Inquiry_Event_Data;
   GAP_Remote_Name_Event_Data_t     *GAP_Remote_Name_Event_Data;
   GAP_Authentication_Information_t  GAP_Authentication_Information;

   /* First, check to see if the required parameters appear to be       */
   /* semi-valid.                                                       */
   if((BluetoothStackID) && (GAP_Event_Data))
   {
      /* The parameters appear to be semi-valid, now check to see what  */
      /* type the incoming event is.                                    */
      switch(GAP_Event_Data->Event_Data_Type)
      {
         case etInquiry_Result:
            /* The GAP event received was of type Inquiry_Result.       */
            GAP_Inquiry_Event_Data = GAP_Event_Data->Event_Data.GAP_Inquiry_Event_Data;

            /* Next, Check to see if the inquiry event data received    */
            /* appears to be semi-valid.                                */
            if(GAP_Inquiry_Event_Data)
            {
               /* Now, check to see if the gap inquiry event data's     */
               /* inquiry data appears to be semi-valid.                */
               if(GAP_Inquiry_Event_Data->GAP_Inquiry_Data)
               {
                  Display(("\r\n"));

                  /* Display a list of all the devices found from       */
                  /* performing the inquiry.                            */
                  for(Index=0;(Index<GAP_Inquiry_Event_Data->Number_Devices) && (Index<MAX_INQUIRY_RESULTS);Index++)
                  {
                     BD_ADDRToStr(GAP_Inquiry_Event_Data->GAP_Inquiry_Data[Index].BD_ADDR, Callback_BoardStr);

                     Display(("Result: %d,%s.\r\n", (Index+1), Callback_BoardStr));
                  }

               }
            }
            break;
         case etInquiry_Entry_Result:
            /* Next convert the BD_ADDR to a string.                    */
            BD_ADDRToStr(GAP_Event_Data->Event_Data.GAP_Inquiry_Entry_Event_Data->BD_ADDR, Callback_BoardStr);

            /* Display this GAP Inquiry Entry Result.                   */
            Display(("\r\n"));
            Display(("Inquiry Entry: %s.\r\n", Callback_BoardStr));
            break;
         case etAuthentication:
            /* An authentication event occurred, determine which type of*/
            /* authentication event occurred.                           */
            switch(GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->GAP_Authentication_Event_Type)
            {
               case atLinkKeyRequest:
                  BD_ADDRToStr(GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Remote_Device, Callback_BoardStr);
                  Display(("\r\n"));
                  Display(("atLinkKeyRequest: %s\r\n", Callback_BoardStr));

                  /* Setup the authentication information response      */
                  /* structure.                                         */
                  GAP_Authentication_Information.GAP_Authentication_Type    = atLinkKey;
                  GAP_Authentication_Information.Authentication_Data_Length = 0;

                  /* See if we have stored a Link Key for the specified */
                  /* device.                                            */
                  for(Index=0;Index<(sizeof(LinkKeyInfo)/sizeof(LinkKeyInfo_t));Index++)
                  {
                     if(COMPARE_BD_ADDR(LinkKeyInfo[Index].BD_ADDR, GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Remote_Device))
                     {
                        /* Link Key information stored, go ahead and    */
                        /* respond with the stored Link Key.            */
                        GAP_Authentication_Information.Authentication_Data_Length   = sizeof(Link_Key_t);
                        GAP_Authentication_Information.Authentication_Data.Link_Key = LinkKeyInfo[Index].LinkKey;

                        break;
                     }
                  }

                  /* Submit the authentication response.                */
                  Result = GAP_Authentication_Response(BluetoothStackID, GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Remote_Device, &GAP_Authentication_Information);

                  /* Check the result of the submitted command.         */
                  if(!Result)
                     DisplayFunctionSuccess("GAP_Authentication_Response");
                  else
                     DisplayFunctionError("GAP_Authentication_Response", Result);
                  break;
               case atPINCodeRequest:
                  /* A pin code request event occurred, first display   */
                  /* the BD_ADD of the remote device requesting the pin.*/
                  BD_ADDRToStr(GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Remote_Device, Callback_BoardStr);
                  Display(("\r\n"));
                  Display(("atPINCodeRequest: %s\r\n", Callback_BoardStr));

                  /* Inform the user that they will need to respond with*/
                  /* a PIN Code Response.                               */
                  Display(("Respond with: PINCodeResponse\r\n"));
                  break;
               case atAuthenticationStatus:
                  /* An authentication status event occurred, display   */
                  /* all relevant information.                          */
                  BD_ADDRToStr(GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Remote_Device, Callback_BoardStr);
                  Display(("\r\n"));
                  Display(("atAuthenticationStatus: %d for %s\r\n", GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Authentication_Event_Data.Authentication_Status, Callback_BoardStr));

                  break;
               case atLinkKeyCreation:
                  /* A link key creation event occurred, first display  */
                  /* the remote device that caused this event.          */
                  BD_ADDRToStr(GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Remote_Device, Callback_BoardStr);
                  Display(("\r\n"));
                  Display(("atLinkKeyCreation: %s\r\n", Callback_BoardStr));

                  /* Now store the link Key in either a free location OR*/
                  /* over the old key location.                         */
                  ASSIGN_BD_ADDR(NULL_BD_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);

                  for(Index=0,Result=-1;Index<(sizeof(LinkKeyInfo)/sizeof(LinkKeyInfo_t));Index++)
                  {
                     if(COMPARE_BD_ADDR(LinkKeyInfo[Index].BD_ADDR, GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Remote_Device))
                        break;
                     else
                     {
                        if((Result == (-1)) && (COMPARE_BD_ADDR(LinkKeyInfo[Index].BD_ADDR, NULL_BD_ADDR)))
                           Result = Index;
                     }
                  }

                  /* If we didn't find a match, see if we found an empty*/
                  /* location.                                          */
                  if(Index == (sizeof(LinkKeyInfo)/sizeof(LinkKeyInfo_t)))
                     Index = Result;

                  /* Check to see if we found a location to store the   */
                  /* Link Key information into.                         */
                  if(Index != (-1))
                  {
                     LinkKeyInfo[Index].BD_ADDR = GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Remote_Device;
                     LinkKeyInfo[Index].LinkKey = GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Authentication_Event_Data.Link_Key_Info.Link_Key;

                     Display(("Link Key Stored.\r\n"));
                  }
                  else
                     Display(("Link Key array full.\r\n"));
                  break;
               case atIOCapabilityRequest:
                  BD_ADDRToStr(GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Remote_Device, Callback_BoardStr);
                  Display(("\r\n"));
                  Display(("atIOCapabilityRequest: %s\r\n", Callback_BoardStr));

                  /* Setup the Authentication Information Response      */
                  /* structure.                                         */
                  GAP_Authentication_Information.GAP_Authentication_Type                                      = atIOCapabilities;
                  GAP_Authentication_Information.Authentication_Data_Length                                   = sizeof(GAP_IO_Capabilities_t);
                  GAP_Authentication_Information.Authentication_Data.IO_Capabilities.IO_Capability            = (GAP_IO_Capability_t)IOCapability;
                  GAP_Authentication_Information.Authentication_Data.IO_Capabilities.MITM_Protection_Required = MITMProtection;
                  GAP_Authentication_Information.Authentication_Data.IO_Capabilities.OOB_Data_Present         = OOBSupport;

                  /* Submit the Authentication Response.                */
                  Result = GAP_Authentication_Response(BluetoothStackID, GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Remote_Device, &GAP_Authentication_Information);

                  /* Check the result of the submitted command.         */
                  /* Check the result of the submitted command.         */
                  if(!Result)
                     DisplayFunctionSuccess("Auth");
                  else
                     DisplayFunctionError("Auth", Result);
                  break;
               case atIOCapabilityResponse:
                  BD_ADDRToStr(GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Remote_Device, Callback_BoardStr);
                  Display(("\r\n"));
                  Display(("atIOCapabilityResponse: %s\r\n", Callback_BoardStr));

                  RemoteIOCapability = GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Authentication_Event_Data.IO_Capabilities.IO_Capability;
                  MITM               = (Boolean_t)GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Authentication_Event_Data.IO_Capabilities.MITM_Protection_Required;
                  OOB_Data           = (Boolean_t)GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Authentication_Event_Data.IO_Capabilities.OOB_Data_Present;

                  Display(("Capabilities: %s%s%s\r\n", IOCapabilitiesStrings[RemoteIOCapability], ((MITM)?", MITM":""), ((OOB_Data)?", OOB Data":"")));
                  break;
               case atUserConfirmationRequest:
                  BD_ADDRToStr(GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Remote_Device, Callback_BoardStr);
                  Display(("\r\n"));
                  Display(("atUserConfirmationRequest: %s\r\n", Callback_BoardStr));

                  if(IOCapability != icDisplayYesNo)
                  {
                     /* Invoke JUST Works Process...                    */
                     GAP_Authentication_Information.GAP_Authentication_Type          = atUserConfirmation;
                     GAP_Authentication_Information.Authentication_Data_Length       = (Byte_t)sizeof(Byte_t);
                     GAP_Authentication_Information.Authentication_Data.Confirmation = TRUE;

                     /* Submit the Authentication Response.             */
                     Display(("\r\nAuto Accepting: %l\r\n", GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Authentication_Event_Data.Numeric_Value));

                     Result = GAP_Authentication_Response(BluetoothStackID, GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Remote_Device, &GAP_Authentication_Information);

                     if(!Result)
                        DisplayFunctionSuccess("GAP_Authentication_Response");
                     else
                        DisplayFunctionError("GAP_Authentication_Response", Result);

                  }
                  else
                  {
                     Display(("User Confirmation: %l\r\n", (unsigned long)GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Authentication_Event_Data.Numeric_Value));

                     /* Inform the user that they will need to respond  */
                     /* with a PIN Code Response.                       */
                     Display(("Respond with: UserConfirmationResponse\r\n"));
                  }
                  break;
               case atPasskeyRequest:
                  BD_ADDRToStr(GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Remote_Device, Callback_BoardStr);
                  Display(("\r\n"));
                  Display(("atPasskeyRequest: %s\r\n", Callback_BoardStr));


                  /* Inform the user that they will need to respond with*/
                  /* a Passkey Response.                                */
                  Display(("Respond with: PassKeyResponse\r\n"));
                  break;
               case atRemoteOutOfBandDataRequest:
                  BD_ADDRToStr(GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Remote_Device, Callback_BoardStr);
                  Display(("\r\n"));
                  Display(("atRemoteOutOfBandDataRequest: %s\r\n", Callback_BoardStr));

                  /* This application does not support OOB data so      */
                  /* respond with a data length of Zero to force a      */
                  /* negative reply.                                    */
                  GAP_Authentication_Information.GAP_Authentication_Type    = atOutOfBandData;
                  GAP_Authentication_Information.Authentication_Data_Length = 0;

                  Result = GAP_Authentication_Response(BluetoothStackID, GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Remote_Device, &GAP_Authentication_Information);

                  if(!Result)
                     DisplayFunctionSuccess("GAP_Authentication_Response");
                  else
                     DisplayFunctionError("GAP_Authentication_Response", Result);
                  break;
               case atPasskeyNotification:
                  BD_ADDRToStr(GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Remote_Device, Callback_BoardStr);
                  Display(("\r\n"));
                  Display(("atPasskeyNotification: %s\r\n", Callback_BoardStr));

                  Display(("Passkey Value: %lu\r\n", GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Authentication_Event_Data.Numeric_Value));
                  break;
               case atKeypressNotification:
                  BD_ADDRToStr(GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Remote_Device, Callback_BoardStr);
                  Display(("\r\n"));
                  Display(("atKeypressNotification: %s\r\n", Callback_BoardStr));

                  Display(("Keypress: %d\r\n", (int)GAP_Event_Data->Event_Data.GAP_Authentication_Event_Data->Authentication_Event_Data.Keypress_Type));
                  break;
               default:
                  Display(("Un-handled Auth. Event.\r\n"));
                  break;
            }
            break;
         case etRemote_Name_Result:
            /* Bluetooth Stack has responded to a previously issued     */
            /* Remote Name Request that was issued.                     */
            GAP_Remote_Name_Event_Data = GAP_Event_Data->Event_Data.GAP_Remote_Name_Event_Data;
            if(GAP_Remote_Name_Event_Data)
            {
               /* Inform the user of the Result.                        */
               BD_ADDRToStr(GAP_Remote_Name_Event_Data->Remote_Device, Callback_BoardStr);

               Display(("\r\n"));
               Display(("BD_ADDR: %s.\r\n", Callback_BoardStr));

               if(GAP_Remote_Name_Event_Data->Remote_Name)
                  Display(("Name: %s.\r\n", GAP_Remote_Name_Event_Data->Remote_Name));
               else
                  Display(("Name: NULL.\r\n"));
            }
            break;
         case etEncryption_Change_Result:
            BD_ADDRToStr(GAP_Event_Data->Event_Data.GAP_Encryption_Mode_Event_Data->Remote_Device, Callback_BoardStr);
            Display(("\r\netEncryption_Change_Result for %s, Status: 0x%02X, Mode: %s.\r\n", Callback_BoardStr,
                                                                                             GAP_Event_Data->Event_Data.GAP_Encryption_Mode_Event_Data->Encryption_Change_Status,
                                                                                             ((GAP_Event_Data->Event_Data.GAP_Encryption_Mode_Event_Data->Encryption_Mode == emDisabled)?"Disabled": "Enabled")));
            break;
         default:
            /* An unknown/unexpected GAP event was received.            */
            Display(("\r\nUnknown Event: %d.\r\n", GAP_Event_Data->Event_Data_Type));
            break;
      }
   }
   else
   {
      /* There was an error with one or more of the input parameters.   */
      Display(("\r\n"));
      Display(("Null Event\r\n"));
   }

}

   /* The following function is for an SPP Event Callback.  This        */
   /* function will be called whenever a SPP Event occurs that is       */
   /* associated with the Bluetooth Stack.  This function passes to the */
   /* caller the SPP Event Data that occurred and the SPP Event Callback*/
   /* Parameter that was specified when this Callback was installed.    */
   /* The caller is free to use the contents of the SPP SPP Event Data  */
   /* ONLY in the context of this callback.  If the caller requires the */
   /* Data for a longer period of time, then the callback function MUST */
   /* copy the data into another Data Buffer.  This function is         */
   /* guaranteed NOT to be invoked more than once simultaneously for the*/
   /* specified installed callback (i.e.  this function DOES NOT have be*/
   /* reentrant).  It Needs to be noted however, that if the same       */
   /* Callback is installed more than once, then the callbacks will be  */
   /* called serially.  Because of this, the processing in this function*/
   /* should be as efficient as possible.  It should also be noted that */
   /* this function is called in the Thread Context of a Thread that the*/
   /* User does NOT own.  Therefore, processing in this function should */
   /* be as efficient as possible (this argument holds anyway because   */
   /* another SPP Event will not be processed while this function call  */
   /* is outstanding).                                                  */
   /* * NOTE * This function MUST NOT Block and wait for Events that    */
   /*          can only be satisfied by Receiving SPP Event Packets.  A */
   /*          Deadlock WILL occur because NO SPP Event Callbacks will  */
   /*          be issued while this function is currently outstanding.  */
static void BTPSAPI SPP_Event_Callback(unsigned int BluetoothStackID, SPP_Event_Data_t *SPP_Event_Data, unsigned long CallbackParameter)
{
   int          SerialPortIndex;
   int          ret_val = 0;
   int          Index;
   int          Index1;
   int          TempLength;

#if ((SPP_PERFORM_MASTER_ROLE_SWITCH) && (MAX_SIMULTANEOUS_SPP_PORTS > 1))
   Byte_t       StatusResult;
   Word_t       Connection_HandleResult;
   Byte_t       Current_Role;
#endif

   Word_t       ConnectionHandle;
   Boolean_t    Done;
   BoardStr_t   Callback_BoardStr;
   unsigned int LocalSerialPortID;

   Display(("SPP_Event_callback operated!\r\n"));
   /* **** SEE SPPAPI.H for a list of all possible event types.  This   */
   /* program only services its required events.                   **** */

   /* First, check to see if the required parameters appear to be       */
   /* semi-valid.                                                       */
   if((SPP_Event_Data) && (BluetoothStackID))
   {
      /* The parameters appear to be semi-valid, now check to see what  */
      /* type the incoming event is.                                    */
      switch(SPP_Event_Data->Event_Data_Type)
      {
         case etPort_Open_Indication:
            /* A remote port is requesting a connection.                */
            BD_ADDRToStr(SPP_Event_Data->Event_Data.SPP_Open_Port_Indication_Data->BD_ADDR, Callback_BoardStr);

            Display(("\r\n"));
            Display(("SPP Open Indication, ID: 0x%04X, Board: %s.\r\n", SPP_Event_Data->Event_Data.SPP_Open_Port_Indication_Data->SerialPortID, Callback_BoardStr));
            BTPS_AddFunctionToScheduler(AUTOMODE_CountBeforeWrite,NULL,1000);

            /* Find the index of the SPP Port Information.              */
            if((SerialPortIndex = FindSPPPortIndex(SPP_Event_Data->Event_Data.SPP_Open_Port_Indication_Data->SerialPortID)) >= 0)
            {
               /* Flag that we are connected to the device.             */
               SPPContextInfo[SerialPortIndex].Connected = TRUE;
               SPPContextInfo[SerialPortIndex].BD_ADDR   = SPP_Event_Data->Event_Data.SPP_Open_Port_Indication_Data->BD_ADDR;

               /* Query the connection handle.                          */
               ret_val = GAP_Query_Connection_Handle(BluetoothStackID, SPPContextInfo[SerialPortIndex].BD_ADDR, &ConnectionHandle);
               if(ret_val)
               {
                  /* Failed to Query the Connection Handle.             */
                  DisplayFunctionError("GAP_Query_Connection_Handle()",ret_val);
               }
               else
               {
                  /* Save the connection handle of this connection.     */
                  SPPContextInfo[SerialPortIndex].Connection_Handle = ConnectionHandle;

#if ((SPP_PERFORM_MASTER_ROLE_SWITCH) && (MAX_SIMULTANEOUS_SPP_PORTS > 1))

                  /* First determine the current role to determine if we*/
                  /* are already the master.                            */
                  StatusResult = 0;
                  ret_val = HCI_Role_Discovery(BluetoothStackID, SPPContextInfo[SerialPortIndex].Connection_Handle, &StatusResult, &Connection_HandleResult, &Current_Role);
                  if((ret_val == 0) && (StatusResult == HCI_ERROR_CODE_NO_ERROR))
                  {
                     /* Check to see if we aren't currently the master. */
                     if(Current_Role != HCI_CURRENT_ROLE_MASTER)
                     {
                        /* Attempt to switch to the master role.        */
                        StatusResult = 0;
                        ret_val = HCI_Switch_Role(BluetoothStackID, SPPContextInfo[SerialPortIndex].BD_ADDR, HCI_ROLE_SWITCH_BECOME_MASTER, &StatusResult);
                        if((ret_val == 0) && (StatusResult == HCI_ERROR_CODE_NO_ERROR))
                        {
                            Display(("\r\nInitiating Role Switch.\r\n"));
                        }
                        else
                        {
                            Display(("HCI Switch Role failed. %d: 0x%02X", ret_val, StatusResult));
                        }
                     }
                  }
                  else
                  {
                      Display(("HCI Role Discovery failed. %d: 0x%02X", ret_val, StatusResult));
                  }

#endif

               }
            }
            break;
         case etPort_Open_Confirmation:
            /* A Client Port was opened.  The Status indicates the      */
            /* Status of the Open.                                      */
            Display(("\r\n"));
            Display(("SPP Open Confirmation, ID: 0x%04X, Status 0x%04X.\r\n", SPP_Event_Data->Event_Data.SPP_Open_Port_Confirmation_Data->SerialPortID,
                                                                              SPP_Event_Data->Event_Data.SPP_Open_Port_Confirmation_Data->PortOpenStatus));


            /* Find the index of the SPP Port Information.              */
            if((SerialPortIndex = FindSPPPortIndex(SPP_Event_Data->Event_Data.SPP_Open_Port_Confirmation_Data->SerialPortID)) >= 0)
            {
               /* Check the Status to make sure that an error did not   */
               /* occur.                                                */
               if(SPP_Event_Data->Event_Data.SPP_Open_Port_Confirmation_Data->PortOpenStatus)
               {
                  /* An error occurred while opening the Serial Port so */
                  /* invalidate the Serial Port ID.                     */
                  BTPS_MemInitialize(&SPPContextInfo[SerialPortIndex], 0, sizeof(SPPContextInfo[SerialPortIndex]));
               }
               else
               {
                  /* Flag that we are connected to the device.          */
                  SPPContextInfo[SerialPortIndex].Connected = TRUE;

                  /* Query the connection Handle.                       */
                  ret_val = GAP_Query_Connection_Handle(BluetoothStackID, SPPContextInfo[SerialPortIndex].BD_ADDR, &ConnectionHandle);
                  if(ret_val)
                  {
                     /* Failed to Query the Connection Handle.          */
                     DisplayFunctionError("GAP_Query_Connection_Handle()",ret_val);
                  }
                  else
                  {
                     /* Save the connection handle of this connection.  */
                     SPPContextInfo[SerialPortIndex].Connection_Handle = ConnectionHandle;

#if ((SPP_PERFORM_MASTER_ROLE_SWITCH) && (MAX_SIMULTANEOUS_SPP_PORTS > 1))

                     /* First determine the current role to determine if*/
                     /* we are already the master.                      */
                     StatusResult = 0;
                     ret_val = HCI_Role_Discovery(BluetoothStackID, SPPContextInfo[SerialPortIndex].Connection_Handle, &StatusResult, &Connection_HandleResult, &Current_Role);
                     if((ret_val == 0) && (StatusResult == HCI_ERROR_CODE_NO_ERROR))
                     {
                        /* Check to see if we aren't currently the      */
                        /* master.                                      */
                        if(Current_Role != HCI_CURRENT_ROLE_MASTER)
                        {
                           /* Attempt to switch to the master role.     */
                           StatusResult = 0;
                           ret_val = HCI_Switch_Role(BluetoothStackID, SPPContextInfo[SerialPortIndex].BD_ADDR, HCI_ROLE_SWITCH_BECOME_MASTER, &StatusResult);
                           if((ret_val == 0) && (StatusResult == HCI_ERROR_CODE_NO_ERROR))
                           {
                               Display(("\r\nInitiating Role Switch.\r\n"));
                           }
                           else
                           {
                               Display(("HCI Switch Role failed. %d: 0x%02X", ret_val, StatusResult));
                           }
                        }
                     }
                     else
                     {
                         Display(("HCI Role Discovery failed. %d: 0x%02X", ret_val, StatusResult));
                     }

#endif

                  }
               }
            }
            break;
         case etPort_Close_Port_Indication:
            /* The Remote Port was Disconnected.                        */
            LocalSerialPortID = SPP_Event_Data->Event_Data.SPP_Close_Port_Indication_Data->SerialPortID;

            Display(("\r\n"));
            Display(("SPP Close Port, ID: 0x%04X\r\n", LocalSerialPortID));

            /* Find the port index of the SPP Port that just closed.    */
            if((SerialPortIndex = FindSPPPortIndex(LocalSerialPortID)) >= 0)
            {
               ASSIGN_BD_ADDR(SPPContextInfo[SerialPortIndex].BD_ADDR, 0, 0, 0, 0, 0, 0);
               SPPContextInfo[SerialPortIndex].SendInfo.BytesToSend = 0;
               SPPContextInfo[SerialPortIndex].Connected            = FALSE;

               /* If this is a client port we also need to clear the    */
               /* Serial Port ID since it is no longer in use.          */
               if(SPPContextInfo[SerialPortIndex].ServerPortNumber == 0)
                  SPPContextInfo[SerialPortIndex].LocalSerialPortID = 0;
            }
            break;
         case etPort_Status_Indication:
            /* Display Information about the new Port Status.           */
            Display(("\r\n"));
            Display(("SPP Port Status Indication: 0x%04X, Status: 0x%04X, Break Status: 0x%04X, Length: 0x%04X.\r\n", SPP_Event_Data->Event_Data.SPP_Port_Status_Indication_Data->SerialPortID,
                                                                                                                    SPP_Event_Data->Event_Data.SPP_Port_Status_Indication_Data->PortStatus,
                                                                                                                    SPP_Event_Data->Event_Data.SPP_Port_Status_Indication_Data->BreakStatus,
                                                                                                                    SPP_Event_Data->Event_Data.SPP_Port_Status_Indication_Data->BreakTimeout));

            break;
         case etPort_Data_Indication:
            /* Data was received.  Process it differently based upon the*/
            /* current state of the Loopback Mode.                      */
            LocalSerialPortID = SPP_Event_Data->Event_Data.SPP_Data_Indication_Data->SerialPortID;

            /* Find the port index of the correct SPP Port.             */
            if((SerialPortIndex = FindSPPPortIndex(LocalSerialPortID)) >= 0)
            {

#if MAXIMUM_SPP_LOOPBACK_BUFFER_SIZE > 0

               /* Determine what data mode we are in.                   */
               if(LoopbackActive)
               {
                  /* Initialize Done to false.                          */
                  Done = FALSE;

                  /* Loop until the write buffer is full or there is not*/
                  /* more data to read.                                 */
                  while((Done == FALSE) && (SPPContextInfo[SerialPortIndex].SendInfo.BufferFull == FALSE))
                  {
                     /* The application state is currently in the loop  */
                     /* back state.  Read as much data as we can read.  */
                     if((TempLength = SPP_Data_Read(BluetoothStackID, LocalSerialPortID, (Word_t)sizeof(SPPContextInfo[SerialPortIndex].Buffer), (Byte_t *)SPPContextInfo[SerialPortIndex].Buffer)) > 0)
                     {
                        /* Adjust the Current Buffer Length by the      */
                        /* number of bytes which were successfully read.*/
                        SPPContextInfo[SerialPortIndex].BufferLength = TempLength;

                        /* Next attempt to write all of the data which  */
                        /* is currently in the buffer.                  */
                        if((TempLength = SPP_Data_Write(BluetoothStackID, LocalSerialPortID, (Word_t)SPPContextInfo[SerialPortIndex].BufferLength, (Byte_t *)SPPContextInfo[SerialPortIndex].Buffer)) < (int)SPPContextInfo[SerialPortIndex].BufferLength)
                        {
                           /* Not all of the data was successfully      */
                           /* written or an error occurred, first check */
                           /* to see if an error occurred.              */
                           if(TempLength >= 0)
                           {
                              /* An error did not occur therefore the   */
                              /* Transmit Buffer must be full.  Adjust  */
                              /* the Buffer and Buffer Length by the    */
                              /* amount which as successfully written.  */
                              if(TempLength)
                              {
                                 for(Index=0,Index1=TempLength;Index1<SPPContextInfo[SerialPortIndex].BufferLength;Index++,Index1++)
                                    SPPContextInfo[SerialPortIndex].Buffer[Index] = SPPContextInfo[SerialPortIndex].Buffer[Index1];

                                 SPPContextInfo[SerialPortIndex].BufferLength -= TempLength;
                              }

                              /* Set the flag indicating that the SPP   */
                              /* Write Buffer is full.                  */
                              SPPContextInfo[SerialPortIndex].SendInfo.BufferFull = TRUE;
                           }
                           else
                              Done = TRUE;
                        }
                     }
                     else
                        Done = TRUE;
                  }

               }
               else
               {
                  /* If we are operating in Raw Data Display Mode then  */
                  /* simply display the data that was give to use.      */
                  if((DisplayRawData) || (AutomaticReadActive))
                  {
                     /* Initialize Done to false.                       */
                     Done = FALSE;

                     /* Loop through and read all data that is present  */
                     /* in the buffer.                                  */
                     while(!Done)
                     {

                        /* Read as much data as possible.               */
                        if((TempLength = SPP_Data_Read(BluetoothStackID, LocalSerialPortID, (Word_t)sizeof(SPPContextInfo[SerialPortIndex].Buffer), (Byte_t *)SPPContextInfo[SerialPortIndex].Buffer)) > 0)
                        {
                           /* Now simply display each character that we */
                           /* have just read.                           */
                           if(DisplayRawData)
                           {
                           }
                        }
                        else
                        {
                           /* Either an error occurred or there is no   */
                           /* more data to be read.                     */
                           if(TempLength < 0)
                           {
                              /* Error occurred.                        */
                              Display(("SPP_Data_Read(): Error %d.\r\n", TempLength));
                           }

                           /* Regardless if an error occurred, we are   */
                           /* finished with the current loop.           */
                           Done = TRUE;
                        }
                     }

                  }
                  else
                  {
                     /* Simply inform the user that data has arrived.   */
                     Display(("\r\n"));
                     Display(("SPP Data Indication, ID: 0x%04X, Length: 0x%04X.\r\n", SPP_Event_Data->Event_Data.SPP_Data_Indication_Data->SerialPortID,
                                                                                      SPP_Event_Data->Event_Data.SPP_Data_Indication_Data->DataLength));
                  }
               }

#else

               /* Simply inform the user that data has arrived.         */
               Display(("\r\n"));
               Display(("SPP Data Indication, ID: 0x%04X, Length: 0x%04X.\r\n", SPP_Event_Data->Event_Data.SPP_Data_Indication_Data->SerialPortID,
                                                                                SPP_Event_Data->Event_Data.SPP_Data_Indication_Data->DataLength));

#endif

            }
            break;
         case etPort_Send_Port_Information_Indication:
            /* Simply Respond with the information that was sent to us. */
            ret_val = SPP_Respond_Port_Information(BluetoothStackID, SPP_Event_Data->Event_Data.SPP_Send_Port_Information_Indication_Data->SerialPortID, &SPP_Event_Data->Event_Data.SPP_Send_Port_Information_Indication_Data->SPPPortInformation);
            break;
         case etPort_Transmit_Buffer_Empty_Indication:
            /* Locate the serial port for the SPP Port that now has a   */
            /* transmit buffer empty.                                   */
            LocalSerialPortID = SPP_Event_Data->Event_Data.SPP_Transmit_Buffer_Empty_Indication_Data->SerialPortID;

            /* Attempt to find the index of the SPP Port entry.         */
            if((SerialPortIndex = FindSPPPortIndex(LocalSerialPortID)) >= 0)
            {
               /* Flag that this buffer is no longer full.              */
               SPPContextInfo[SerialPortIndex].SendInfo.BufferFull = FALSE;

               /* The transmit buffer is now empty after being full.    */
               /* Next check the current application state.             */
               if(SPPContextInfo[SerialPortIndex].SendInfo.BytesToSend)
               {
                  /* Send the remainder of the last attempt.            */
                  TempLength                    = (DataStrLen-SPPContextInfo[SerialPortIndex].SendInfo.BytesToSend);

                  SPPContextInfo[SerialPortIndex].SendInfo.BytesSent    = SPP_Data_Write(BluetoothStackID, LocalSerialPortID, TempLength, (unsigned char *)&(DataStr[SPPContextInfo[SerialPortIndex].SendInfo.BytesSent]));
                  if((int)(SPPContextInfo[SerialPortIndex].SendInfo.BytesSent) >= 0)
                  {
                     if(SPPContextInfo[SerialPortIndex].SendInfo.BytesSent <= SPPContextInfo[SerialPortIndex].SendInfo.BytesToSend)
                        SPPContextInfo[SerialPortIndex].SendInfo.BytesToSend -= SPPContextInfo[SerialPortIndex].SendInfo.BytesSent;
                     else
                        SPPContextInfo[SerialPortIndex].SendInfo.BytesToSend  = 0;

                     while(SPPContextInfo[SerialPortIndex].SendInfo.BytesToSend)
                     {
                        /* Set the Number of bytes to send in the next  */
                        /* packet.                                      */
                        if(SPPContextInfo[SerialPortIndex].SendInfo.BytesToSend > DataStrLen)
                           TempLength = DataStrLen;
                        else
                           TempLength = SPPContextInfo[SerialPortIndex].SendInfo.BytesToSend;

                        SPPContextInfo[SerialPortIndex].SendInfo.BytesSent = SPP_Data_Write(BluetoothStackID, LocalSerialPortID, TempLength, (unsigned char *)DataStr);
                        if((int)(SPPContextInfo[SerialPortIndex].SendInfo.BytesSent) >= 0)
                        {
                           SPPContextInfo[SerialPortIndex].SendInfo.BytesToSend -= SPPContextInfo[SerialPortIndex].SendInfo.BytesSent;
                           if(SPPContextInfo[SerialPortIndex].SendInfo.BytesSent < TempLength)
                              break;
                        }
                        else
                        {
                           Display(("SPP_Data_Write returned %d.\r\n", (int)SPPContextInfo[SerialPortIndex].SendInfo.BytesSent));

                           SPPContextInfo[SerialPortIndex].SendInfo.BytesToSend = 0;
                        }
                     }
                  }
                  else
                  {
                     Display(("SPP_Data_Write returned %d.\r\n", (int)SPPContextInfo[SerialPortIndex].SendInfo.BytesSent));

                     SPPContextInfo[SerialPortIndex].SendInfo.BytesToSend = 0;
                  }
               }
               else
               {

#if MAXIMUM_SPP_LOOPBACK_BUFFER_SIZE > 0

                  if(LoopbackActive)
                  {
                     /* Initialize Done to false.                       */
                     Done = FALSE;

                     /* Loop until the write buffer is full or there is */
                     /* not more data to read.                          */
                     while(Done == FALSE)
                     {
                        /* The application state is currently in the    */
                        /* loop back state.  Read as much data as we can*/
                        /* read.                                        */
                        if(((TempLength = SPP_Data_Read(BluetoothStackID, LocalSerialPortID, (Word_t)(sizeof(SPPContextInfo[SerialPortIndex].Buffer)-SPPContextInfo[SerialPortIndex].BufferLength), (Byte_t *)&(SPPContextInfo[SerialPortIndex].Buffer[SPPContextInfo[SerialPortIndex].BufferLength]))) > 0) || (SPPContextInfo[SerialPortIndex].BufferLength > 0))
                        {
                           /* Adjust the Current Buffer Length by the   */
                           /* number of bytes which were successfully   */
                           /* read.                                     */
                           if(TempLength > 0)
                              SPPContextInfo[SerialPortIndex].BufferLength += TempLength;

                           /* Next attempt to write all of the data     */
                           /* which is currently in the buffer.         */
                           if((TempLength = SPP_Data_Write(BluetoothStackID, LocalSerialPortID, (Word_t)SPPContextInfo[SerialPortIndex].BufferLength, (Byte_t *)SPPContextInfo[SerialPortIndex].Buffer)) < (int)SPPContextInfo[SerialPortIndex].BufferLength)
                           {
                              /* Not all of the data was successfully   */
                              /* written or an error occurred, first    */
                              /* check to see if an error occurred.     */
                              if(TempLength >= 0)
                              {
                                 /* An error did not occur therefore the*/
                                 /* Transmit Buffer must be full.       */
                                 /* Adjust the Buffer and Buffer Length */
                                 /* by the amount which was successfully*/
                                 /* written.                            */
                                 if(TempLength)
                                 {
                                    for(Index=0,Index1=TempLength;Index1<SPPContextInfo[SerialPortIndex].BufferLength;Index++,Index1++)
                                       SPPContextInfo[SerialPortIndex].Buffer[Index] = SPPContextInfo[SerialPortIndex].Buffer[Index1];

                                    SPPContextInfo[SerialPortIndex].BufferLength -= TempLength;
                                 }
                                 else
                                    Done = TRUE;

                                 /* Set the flag indicating that the SPP*/
                                 /* Write Buffer is full.               */
                                 SPPContextInfo[SerialPortIndex].SendInfo.BufferFull = TRUE;
                              }
                              else
                                 Done = TRUE;
                           }
                           else
                           {
                              SPPContextInfo[SerialPortIndex].BufferLength          = 0;

                              SPPContextInfo[SerialPortIndex].SendInfo.BufferFull   = FALSE;
                           }
                        }
                        else
                           Done = TRUE;
                     }
                  }
                  else
                  {
                     /* Only print the event indication to the user if  */
                     /* we are NOT operating in Raw Data Display Mode.  */
                     if(!DisplayRawData)
                     {
                        //Display(("\r\nTransmit Buffer Empty Indication, ID: 0x%04X\r\n", SPP_Event_Data->Event_Data.SPP_Transmit_Buffer_Empty_Indication_Data->SerialPortID));
                     }
                  }

#else

                  Display(("\r\nTransmit Buffer Empty Indication, ID: 0x%04X\r\n", SPP_Event_Data->Event_Data.SPP_Transmit_Buffer_Empty_Indication_Data->SerialPortID));

#endif

               }
            }
            else
            {
               Display(("Could not find SPP server Context after Buffer Empty Indication.\r\n"));
            }

            break;
         default:
            /* An unknown/unexpected SPP event was received.            */
            Display(("\r\n"));
            Display(("Unknown Event.\r\n"));
            break;
      }

      /* Check the return value of any function that might have been    */
      /* executed in the callback.                                      */
      if(ret_val)
      {
         /* An error occurred, so output an error message.              */
         Display(("\r\n"));
         Display(("Error %d.\r\n", ret_val));
      }
   }
   else
   {
      /* There was an error with one or more of the input parameters.   */
      Display(("Null Event\r\n"));
   }

}

   /* The following function is responsible for processing HCI Mode     */
   /* change events.                                                    */
static void BTPSAPI HCI_Event_Callback(unsigned int BluetoothStackID, HCI_Event_Data_t *HCI_Event_Data, unsigned long CallbackParameter)
{
   char *Mode;

#if ((SPP_PERFORM_MASTER_ROLE_SWITCH) && (MAX_SIMULTANEOUS_SPP_PORTS > 1))
   int   SerialPortIndex;
#endif

   /* Make sure that the input parameters that were passed to us are    */
   /* semi-valid.                                                       */
   if((BluetoothStackID) && (HCI_Event_Data))
   {
      /* Process the Event Data.                                        */
      switch(HCI_Event_Data->Event_Data_Type)
      {

#if ((SPP_PERFORM_MASTER_ROLE_SWITCH) && (MAX_SIMULTANEOUS_SPP_PORTS > 1))

         case etRole_Change_Event:
            if(HCI_Event_Data->Event_Data.HCI_Role_Change_Event_Data)
            {
               /* Find the Serial Port entry for this event.            */
               if((SerialPortIndex = FindSPPPortIndexByAddress(HCI_Event_Data->Event_Data.HCI_Role_Change_Event_Data->BD_ADDR)) >= 0)
               {
                  if((HCI_Event_Data->Event_Data.HCI_Role_Change_Event_Data->Status == HCI_ERROR_CODE_NO_ERROR) && (HCI_Event_Data->Event_Data.HCI_Role_Change_Event_Data->New_Role == HCI_CURRENT_ROLE_MASTER))
                  {
                     Display(("\r\nSPP Port %u: Role Change Success.\r\n", SPPContextInfo[SerialPortIndex].LocalSerialPortID));
                  }
                  else
                  {
                     Display(("\r\nSPP Port %u: Role Change Failure (Status 0x%02X, Role 0x%02X).\r\n", SPPContextInfo[SerialPortIndex].LocalSerialPortID, HCI_Event_Data->Event_Data.HCI_Role_Change_Event_Data->Status, HCI_Event_Data->Event_Data.HCI_Role_Change_Event_Data->New_Role));
                  }

               }
            }
            break;

#endif

         case etMode_Change_Event:
            if(HCI_Event_Data->Event_Data.HCI_Mode_Change_Event_Data)
            {
               switch(HCI_Event_Data->Event_Data.HCI_Mode_Change_Event_Data->Current_Mode)
               {
                  case HCI_CURRENT_MODE_HOLD_MODE:
                     Mode = "Hold";
                     break;
                  case HCI_CURRENT_MODE_SNIFF_MODE:
                     Mode = "Sniff";
                     break;
                  case HCI_CURRENT_MODE_PARK_MODE:
                     Mode = "Park";
                     break;
                  case HCI_CURRENT_MODE_ACTIVE_MODE:
                  default:
                     Mode = "Active";
                     break;
               }

               Display(("\r\n"));
               Display(("HCI Mode Change Event, Status: 0x%02X, Connection Handle: %d, Mode: %s, Interval: %d\r\n", HCI_Event_Data->Event_Data.HCI_Mode_Change_Event_Data->Status,
                                                                                                                    HCI_Event_Data->Event_Data.HCI_Mode_Change_Event_Data->Connection_Handle,
                                                                                                                    Mode,
                                                                                                                    HCI_Event_Data->Event_Data.HCI_Mode_Change_Event_Data->Interval));
            }
            break;
      }
   }
}

   /* ***************************************************************** */
   /*                    End of Event Callbacks.                        */
   /* ***************************************************************** */

   /* The following function is used to initialize the application      */
   /* instance.  This function should open the stack and prepare to     */
   /* execute commands based on user input.  The first parameter passed */
   /* to this function is the HCI Driver Information that will be used  */
   /* when opening the stack and the second parameter is used to pass   */
   /* parameters to BTPS_Init.  This function returns the               */
   /* BluetoothStackID returned from BSC_Initialize on success or a     */
   /* negative error code (of the form APPLICATION_ERROR_XXX).          */
int InitializeApplication(HCI_DriverInformation_t *HCI_DriverInformation, BTPS_Initialization_t *BTPS_Initialization)
{
   int ret_val = APPLICATION_ERROR_UNABLE_TO_OPEN_STACK;

   /* Next, makes sure that the Driver Information passed appears to be */
   /* semi-valid.                                                       */
   if((HCI_DriverInformation) && (BTPS_Initialization))
   {
      /* Try to Open the stack and check if it was successful.          */
      if(!OpenStack(HCI_DriverInformation, BTPS_Initialization))
      {
         /* First, attempt to set the Device to be Connectable.         */
         ret_val = SetConnect();

         /* Next, check to see if the Device was successfully made      */
         /* Connectable.                                                */
         if(!ret_val)
         {
            /* Now that the device is Connectable attempt to make it    */
            /* Discoverable.                                            */
            ret_val = SetDisc();

            /* Next, check to see if the Device was successfully made   */
            /* Discoverable.                                            */
            if(!ret_val)
            {
               /* Now that the device is discoverable attempt to make it*/
               /* pairable.                                             */
               ret_val = SetPairable();
               if(!ret_val)
               {
                  /* Attempt to register a HCI Event Callback.          */
                  ret_val = HCI_Register_Event_Callback(BluetoothStackID, HCI_Event_Callback, (unsigned long)NULL);
                  if(ret_val > 0)
                  {
                     /* Set up the Selection Interface.                 */
                     UserInterface_Selection();

                     /* Return success to the caller.                   */
                     ret_val = (int)BluetoothStackID;
                     
                  }
               }
               else
                  DisplayFunctionError("SetPairable", ret_val);
            }
            else
               DisplayFunctionError("SetDisc", ret_val);
         }
         else
            DisplayFunctionError("SetDisc", ret_val);

         /* In some error occurred then close the stack.                */
         if(ret_val < 0)
         {
            /* Close the Bluetooth Stack.                               */
            CloseStack();
         }
      }
      else
      {
         /* There was an error while attempting to open the Stack.      */
         Display(("Unable to open the stack.\r\n"));
      }
   }
   else
      ret_val = APPLICATION_ERROR_INVALID_PARAMETERS;

   return(ret_val);
}


////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////User Function////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

void UART_Init(void)
{
  //Change UART module for Bluetooth communication
  //From UCA2 to UCA0
  //UCA2 TXD : P9.4
  //UCA2 RXD : P9.5
  //UCA0 TXD : P3.4
  //UCA0 RXD : P3.5
  
  P9REN &= ~(BIT4);
  P9DIR &= ~(BIT4);
  P9SEL &= ~(BIT4);
  
  P3REN |= (BIT4);
  P3DIR |= (BIT4);
  P3SEL |= (BIT4);
  //P1OUT |= (BIT0 | BIT1);
  HAL_CommConfigure(((unsigned char *)&UCA0CTLW0),HS_BAUD_RATE,0);
  
}

void SPI_Init(void)
{
  // UCB3STE (P10.4)
  // UCB3CLK (P10.3)
  // UCB3SIMO (P10.1)
  // UCB3SOMI (P10.2)
  P10SEL |= 0x0E;
  P10SEL &= ~0x10;
  
  P10OUT|=0x10;
  P10DIR |= 0x10;

  // Start SPI register initialization
  UCB3CTL1 |= 0x01;
  
  UCB3CTL0 = 0xA9;
  UCB3CTL1 |= 0x80;//SMCLK (10)
  UCB3CTL1 &= ~0x40;
  UCB3BR0 = 0x02;//25MHz/2 = 12.5MHz SPI clock frequency
  UCB3BR1 = 0x00;
  
  UCB3CTL1&= ~0x01; // End SPI register initialization
  
  UCB3IFG &= ~0x01; // Reset Rx interrupt flag

}

void DMA_Init(unsigned char *From_Addr, unsigned int length)
{
  DMACTL0 = DMA0TSEL_17;  // USCI_A0 TXIFG trigger
  __data16_write_addr((unsigned short) &DMA0SA,(unsigned long) From_Addr);
                                            // Source block address
  __data16_write_addr((unsigned short) &DMA0DA,(unsigned long) &UCA0TXBUF);
                                            // Destination single address
  
  DMA0SZ = length;                 // Block size
  DMA0CTL = DMASRCINCR_3+DMASBDB+DMALEVEL;  // Repeat, inc src
}

void Buffer_Reset(void)
{
  unsigned char i=0;
  
  for(i=0;i<CHANNEL_NUMBER;i++)
  {
    Spike[i]=0;
  }
  
  SPI_Rx_Addr = ucSPSBS-2;
  
  for(i=0;i<ucBTPBN;i++)
  {
    BT_Tx_Rest[i]=0;
  }
  
}


///////////////////////////////////////////////////////////////////
//      Function        SPI_RHD_Init
//      Description     Use 16-bit SPI communication only for
//                      SPI initialization. This function send
//                      two 8-bit data and do noting on the result
//                      (received) data.
//      Input value     send, send1
//      Return value    NONE
//////////////////////////////////////////////////////////////////
void SPI_RHD_Init(unsigned char send, unsigned char send2)
{
  // UCB1STE (P10.4)
  // UCB1CLK (P10.3)
  // UCB1SIMO (P10.1)
  // UCB1SOMI (P10.2)
  
  static unsigned char dummy;
  static unsigned int SPI_Wait_Counter;
  static unsigned int SPI_W2;
  
  //Wait a short time
  for(SPI_Wait_Counter=0;SPI_Wait_Counter<100;SPI_Wait_Counter++)
  {
    for(SPI_W2=0;SPI_W2<100;SPI_W2++);
  }
  
  /*         First 8-bit data of 16-bit send                 */
  
  //Turn off SPI_CS pin (: SPI selection)
  P10OUT &= ~0x10;              //Start SPI data send
  //wait for SPI_SOMI pin ready
//  while(P5IN & 0x10);
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  
  //Data write in Tx buffer register
  UCB3TXBUF = send;
  
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  
  dummy = UCB3RXBUF;
  
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  
  /*         Second 8-bit data send                 */
  
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  
  //Data write in Tx buffer register
  UCB3TXBUF = send2;
  
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  
  dummy = UCB3RXBUF;
  
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  
  
  //Turn on CS pin (: SPI transmit end notification)
  P10OUT |= 0x10;       //End SPI data send
}

/*
  This Function initialize RHD2132 chip
*/
void RHD_Init(void)
{
  // Set off P5.1 Because no need to connect ADC power pin
//  P5SEL &= ~0x02;
//  P5DIR |= 0x02;
//  P5OUT |= 0x02;
  
  /*     Start calibration          */
  SPI_RHD_Init(0x55,0x00);
  SPI_RHD_Init(0x00,0x00);      //dummy data for calibrating
  SPI_RHD_Init(0x00,0x00);
  SPI_RHD_Init(0x00,0x00);
  SPI_RHD_Init(0x00,0x00);
  SPI_RHD_Init(0x00,0x00);
  SPI_RHD_Init(0x00,0x00);
  SPI_RHD_Init(0x00,0x00);
  SPI_RHD_Init(0x00,0x00);
  SPI_RHD_Init(0x00,0x00);
  
  /*           Setting               */
  SPI_RHD_Init(0x80,0xDE);
  SPI_RHD_Init(0x81,0x02);
  SPI_RHD_Init(0x82,0x04);
  SPI_RHD_Init(0x83,0x02);
  
  ////////////////////////////////////////////////////////
  //DSP HPF activation
  //- effect : DC offset removal
  //- attribution : linearity more than analog filter***
  // Condition 1. DSPen = 1 (in Register 4, bit 4) --> 0x84 -> 0x10 ON
  // Condition 2. setting the k value
  //    Equation
  //            f(cutoff) = k x f(sampling)
  //    k position
  //            Register 4, bit 0 to 3
  //
  //    example
  //            Sampling rate = 13 kHz
  //            k_value = 3 (k=0.02125)
  //            Cut off = 276.25 Hz
  //            --------------------------
  //            0x84 0101 0011 = 0x84 0x53
  //
  //
  //    Summary
  //            Sampling rate   Cut off         Value
  //            13 kSps         595.27 Hz       0x84 0x52
  //            13 kSps         276.25 Hz       0x84 0x53
  //            13 kSps         133.51 Hz       0x84 0x54
  //            13 kSps         1.01049 Hz      0x84 0x5B
  //            13 kSps         OFF             0x84 0x40
  //
  //            8 kSps          366.32 Hz       0x84 0x52
  //            8 kSps          170 Hz          0x84 0x53
  //            8 kSps          82.16 Hz        0x84 0x54
  //            8 kSps          1.244 Hz        0x84 0x5A
  //            8 kSps          OFF             0x84 0x40
  //
  //    *DSP off mode
  //            0x84 0100 0000 = 0x84 0x40
  //////////////////////////////////////////////////////////
  SPI_RHD_Init(0x84,0x53);
  SPI_RHD_Init(0x85,0x00);
  //FH = 20 kHz
  //RH1 DAC1 = 8
  //RH1 DAC2 = 0
  //RH2 DAC1 = 4
  //RH2 DAC2 = 0
  //
  //FH = 5 kHz
  //RH1 DAC1 = 33
  //RH1 DAC2 = 0
  //RH2 DAC1 = 37
  //RH2 DAC2 = 0
  //
  //FL = 300 Hz
  //RL DAC1 = 15
  //RL DAC2 = 0
  //RL DAC3 = 0
  //
  //FL = 0.1 Hz
  //RL DAC1 = 16
  //RL DAC2 = 60
  //RL DAC3 = 1
  ////////////////////////////////////////////
  //Setting : FH = 20 kHz, FL = 0.1 Hz (Full range mode)
  //0x88 0000 1000 = 0x88 0x08
  //0x89 0000 0000 = 0x89 0x00
  //0x8A 0000 0100 = 0x8A 0x04
  //0x8B 0000 0000 = 0x8B 0x00
  //0x8C 0001 0000 = 0x8C 0x10
  //0x8D 0111 1100 = 0x8D 0x7C
  ///////////////////////////////////////////
  //Setting2 : FH = 5 kHz, FL = 300 Hz (AP recording mode)
  //0x88 0010 0001 = 0x88 0x21
  //0x89 0000 0000 = 0x89 0x00
  //0x8A 0010 0101 = 0x8A 0x25
  //0x8B 0000 0000 = 0x8B 0x00
  //0x8C 0000 1111 = 0x8C 0x0F
  //0x8D 0000 0000 = 0x8D 0x00
  
  SPI_RHD_Init(0x88,0x21);//0x08);
  SPI_RHD_Init(0x89,0x00);//0x00);
  SPI_RHD_Init(0x8A,0x25);//0x04);
  SPI_RHD_Init(0x8B,0x00);//0x00);
  SPI_RHD_Init(0x8C,0x0F);//0x10);
  SPI_RHD_Init(0x8D,0x00);//0x7C);
  /*Integrated Tx board design 1 (blue) */
  //Channel 23 to 26 Off, 27 to 30 on
  //Register 14 --> 0000 0000 = 0x0, 0x0
  //Register 15 --> 0000 0000
  //Register 16 --> 0000 0000 = 0x00 //1000 0000 = 0x80
  //Register 17 --> 0111 1000 = 0x78 //0000 0111 = 0x07
  
  /*Integrated Tx board design 2 (red) */
  //Register 14 --> 0000 0000 = 0x00
  //Register 15 --> 1111 1110 = 0xFE
  //Register 16 --> 1111 1111 = 0xFF
  //Register 17 --> 0000 0001 = 0x01
  
  SPI_RHD_Init(0x8E,0x00);
  SPI_RHD_Init(0x8F,0xFE);
  SPI_RHD_Init(0x90,0xFF);
  SPI_RHD_Init(0x91,0x01);
  
  /*            rest value read       */
  SPI_RHD_Init(0x00,0x00);
  SPI_RHD_Init(0x00,0x00);
}

void RHD_SPI_Buffer_Save(void)
{
  // UCB1STE (P10.4)
  // UCB1CLK (P10.3)
  // UCB1SIMO (P10.1)
  // UCB1SOMI (P10.2)
  
  //Define a pointer variable that is the address to write as a result of SPI
  //Variable    SPI_save_ptr
  //Operation 
  //            #CH-01
  //            SPI_save_ptr = SPI_initial_addr;
  //            *SPI_save_ptr <= SPI received data
  //            *(SPI_save_ptr+1) <= SPI 2nd received data
  //            SDA condition check...
  //            
  //            SPI_save_ptr+=18; (pre-save data (18B))
  //
  //            #CH-02
  //            *SPI_save_ptr <= SPI_received data
  //            *(SPI_save_ptr+1) <= SPI 2nd received data
  //            SDA condition check...
  //
  //            SPI_save_ptr+=18; ...
  //
  /////////////////////////////////////////////////////////////////////
  static unsigned char *SPI_initial_ptr = SPI_Pre_Buf[0];
  static unsigned char *SPI_save_ptr;
  static unsigned char BT_Write_ok=1;
  static unsigned char Packet_addr;
  static Byte_t *stt_addr;
  static unsigned char Current_CH;
  
  //Add two Bytes of SPI buffer address to save
  SPI_Rx_Addr+=2;
  if(SPI_Rx_Addr == ucSPSBS) SPI_Rx_Addr=0;
  
  //Set SPI_save_ptr as the address to save
  SPI_save_ptr = SPI_initial_ptr + SPI_Rx_Addr;
  
  //Check if there is any rest space in BT Buf
  if(!BT_Write_ok) 
    BT_Write_ok = (BT_Tx_Packet_Ass_To != BT_Tx_Packet_Ass_From + 1) || ((!!BT_Tx_Packet_Ass_To) || (BT_Tx_Packet_Ass_From != ucBTPBN-1));
  
  //Set current channel
  Current_CH=0;
  
  //111111111111111111111111111111//////////
  /*         First 8-bit data of 16-bit send                 */
  //Turn off SPI_CS pin (: SPI selection)
  P10OUT &= ~0x10;              //Start SPI data send
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = CH_01;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save received SPI data
  *SPI_save_ptr = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  
  /*         Second 8-bit data send                 */
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = SPI_2;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save 2nd received SPI data
  *(SPI_save_ptr+1) = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  //Turn on CS pin (: SPI transmit end notification)
  P10OUT |= 0x10;       //End SPI data send
  
  //Run SDA
  //SDA step 1. Check if the spike is already detected on CH01
  if(Spike[Current_CH])
  {
    //Save the oldest SPI_data into BT data buffer.
    Packet_addr = Spike[Current_CH] - 1;
    stt_addr = &(BT_Tx_Packet_Buf[Packet_addr][BT_Tx_Rest[Packet_addr]]);
    
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    
    BT_Tx_Rest[Packet_addr] += 2;
    
    //Check if the BT buffer is full 24x2=48 (2byte) , 3ms / 8kHz / 16bit resolution
    if(BT_Tx_Rest[Packet_addr] >= ucBTS)
    {
      Spike[Current_CH] = 0;
    }
    
  }
  //SDA step 2. Check if recently read data is enough to set as spike
  //else if((*SPI_save_ptr & 0x80) && (*SPI_save_ptr < 246) && BT_Write_ok)
  else if((*SPI_save_ptr & 0x80) && (*SPI_save_ptr < NVTH_CH_01) && BT_Write_ok)
  {
    //Assign BT buffer space and update current buffer filling state
    stt_addr = BT_Tx_Packet_Buf[BT_Tx_Packet_Ass_From];
    Spike[Current_CH] = ++BT_Tx_Packet_Ass_From;
    if(BT_Tx_Packet_Ass_From == ucBTPBN) BT_Tx_Packet_Ass_From=0;
    
    BT_Write_ok = (BT_Tx_Packet_Ass_To != BT_Tx_Packet_Ass_From + 1) || ((!!BT_Tx_Packet_Ass_To) || (BT_Tx_Packet_Ass_From != ucBTPBN-1));
    
    //Save the header information.
    *stt_addr++ = Current_CH + ((MSP430Ticks&0x0F)<<4);
    *stt_addr++ = ((MSP430Ticks>>4)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>12)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>20)&0xFF);
    
    //Save the oldest SPI data into BT data buffer.
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    //Update currently saved data size in the assigned BT buf.
    BT_Tx_Rest[Spike[Current_CH] - 1] = 6;
  }
  
  //Set Buf address as next channel
  SPI_save_ptr += 18;
  ++Current_CH;
  
  
  //2222222222222222222222222222222222222222222222222//////////
  /*         First 8-bit data of 16-bit send                 */
  //Turn off SPI_CS pin (: SPI selection)
  P10OUT &= ~0x10;              //Start SPI data send
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = CH_02;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save received SPI data
  *SPI_save_ptr = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  
  /*         Second 8-bit data send                 */
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = SPI_2;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save 2nd received SPI data
  *(SPI_save_ptr+1) = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  //Turn on CS pin (: SPI transmit end notification)
  P10OUT |= 0x10;       //End SPI data send
  
  //Run SDA
  //SDA step 1. Check if the spike is already detected on CH01
  if(Spike[Current_CH])
  {
    //Save the oldest SPI_data into BT data buffer.
    Packet_addr = Spike[Current_CH] - 1;
    stt_addr = &(BT_Tx_Packet_Buf[Packet_addr][BT_Tx_Rest[Packet_addr]]);
    
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    
    BT_Tx_Rest[Packet_addr] += 2;
    
    //Check if the BT buffer is full
    if(BT_Tx_Rest[Packet_addr] >= ucBTS)
    {
      Spike[Current_CH] = 0;
    }
    
  }
  //SDA step 2. Check if recently read data is enough to set as spike
  else if((*SPI_save_ptr & 0x80) && (*SPI_save_ptr < NVTH_CH_02) && BT_Write_ok)
  {
    //Assign BT buffer space and update current buffer filling state
    stt_addr = BT_Tx_Packet_Buf[BT_Tx_Packet_Ass_From];
    Spike[Current_CH] = ++BT_Tx_Packet_Ass_From;
    if(BT_Tx_Packet_Ass_From == ucBTPBN) BT_Tx_Packet_Ass_From=0;
    
    BT_Write_ok = (BT_Tx_Packet_Ass_To != BT_Tx_Packet_Ass_From + 1) || ((!!BT_Tx_Packet_Ass_To) || (BT_Tx_Packet_Ass_From != ucBTPBN-1));
    
    //Save the header information.
    *stt_addr++ = Current_CH + ((MSP430Ticks&0x0F)<<4);
    *stt_addr++ = ((MSP430Ticks>>4)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>12)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>20)&0xFF);
    
    //Save the oldest SPI data into BT data buffer.
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    //Update currently saved data size in the assigned BT buf.
    BT_Tx_Rest[Spike[Current_CH] - 1] = 6;
  }
  
  //Set Buf address as next channel
  SPI_save_ptr += 18;
  ++Current_CH;
  
  
  //33333333333333333333333333333333333333333333333333//////////
  /*         First 8-bit data of 16-bit send                 */
  //Turn off SPI_CS pin (: SPI selection)
  P10OUT &= ~0x10;              //Start SPI data send
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = CH_03;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save received SPI data
  *SPI_save_ptr = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  
  /*         Second 8-bit data send                 */
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = SPI_2;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save 2nd received SPI data
  *(SPI_save_ptr+1) = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  //Turn on CS pin (: SPI transmit end notification)
  P10OUT |= 0x10;       //End SPI data send
  
  //Run SDA
  //SDA step 1. Check if the spike is already detected on CH01
  if(Spike[Current_CH])
  {
    //Save the oldest SPI_data into BT data buffer.
    Packet_addr = Spike[Current_CH] - 1;
    stt_addr = &(BT_Tx_Packet_Buf[Packet_addr][BT_Tx_Rest[Packet_addr]]);
    
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    
    BT_Tx_Rest[Packet_addr] += 2;
    
    //Check if the BT buffer is full
    if(BT_Tx_Rest[Packet_addr] >= ucBTS)
    {
      Spike[Current_CH] = 0;
    }
    
  }
  //SDA step 2. Check if recently read data is enough to set as spike
  else if((*SPI_save_ptr & 0x80) && (*SPI_save_ptr < NVTH_CH_03) && BT_Write_ok)
  {
    //Assign BT buffer space and update current buffer filling state
    stt_addr = BT_Tx_Packet_Buf[BT_Tx_Packet_Ass_From];
    Spike[Current_CH] = ++BT_Tx_Packet_Ass_From;
    if(BT_Tx_Packet_Ass_From == ucBTPBN) BT_Tx_Packet_Ass_From=0;
    
    BT_Write_ok = (BT_Tx_Packet_Ass_To != BT_Tx_Packet_Ass_From + 1) || ((!!BT_Tx_Packet_Ass_To) || (BT_Tx_Packet_Ass_From != ucBTPBN-1));
    
    //Save the header information.
    *stt_addr++ = Current_CH + ((MSP430Ticks&0x0F)<<4);
    *stt_addr++ = ((MSP430Ticks>>4)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>12)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>20)&0xFF);
    
    //Save the oldest SPI data into BT data buffer.
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    //Update currently saved data size in the assigned BT buf.
    BT_Tx_Rest[Spike[Current_CH] - 1] = 6;
  }
  
  //Set Buf address as next channel
  SPI_save_ptr += 18;
  ++Current_CH;
  
  
  //44444444444444444444444444444444444444444444444444//////////
  /*         First 8-bit data of 16-bit send                 */
  //Turn off SPI_CS pin (: SPI selection)
  P10OUT &= ~0x10;              //Start SPI data send
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = CH_04;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save received SPI data
  *SPI_save_ptr = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  
  /*         Second 8-bit data send                 */
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = SPI_2;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save 2nd received SPI data
  *(SPI_save_ptr+1) = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  //Turn on CS pin (: SPI transmit end notification)
  P10OUT |= 0x10;       //End SPI data send
  
  //Run SDA
  //SDA step 1. Check if the spike is already detected on CH01
  if(Spike[Current_CH])
  {
    //Save the oldest SPI_data into BT data buffer.
    Packet_addr = Spike[Current_CH] - 1;
    stt_addr = &(BT_Tx_Packet_Buf[Packet_addr][BT_Tx_Rest[Packet_addr]]);
    
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    
    BT_Tx_Rest[Packet_addr] += 2;
    
    //Check if the BT buffer is full
    if(BT_Tx_Rest[Packet_addr] >= ucBTS)
    {
      Spike[Current_CH] = 0;
    }
    
  }
  //SDA step 2. Check if recently read data is enough to set as spike
  else if((*SPI_save_ptr & 0x80) && (*SPI_save_ptr < NVTH_CH_04) && BT_Write_ok)
  {
    //Assign BT buffer space and update current buffer filling state
    stt_addr = BT_Tx_Packet_Buf[BT_Tx_Packet_Ass_From];
    Spike[Current_CH] = ++BT_Tx_Packet_Ass_From;
    if(BT_Tx_Packet_Ass_From == ucBTPBN) BT_Tx_Packet_Ass_From=0;
    
    BT_Write_ok = (BT_Tx_Packet_Ass_To != BT_Tx_Packet_Ass_From + 1) || ((!!BT_Tx_Packet_Ass_To) || (BT_Tx_Packet_Ass_From != ucBTPBN-1));
    
    //Save the header information.
    *stt_addr++ = Current_CH + ((MSP430Ticks&0x0F)<<4);
    *stt_addr++ = ((MSP430Ticks>>4)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>12)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>20)&0xFF);
    
    //Save the oldest SPI data into BT data buffer.
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    //Update currently saved data size in the assigned BT buf.
    BT_Tx_Rest[Spike[Current_CH] - 1] = 6;
  }
  
  //Set Buf address as next channel
  SPI_save_ptr += 18;
  ++Current_CH;
  
  
  //5555555555555555555555555555555555555555555555555//////////
  /*         First 8-bit data of 16-bit send                 */
  //Turn off SPI_CS pin (: SPI selection)
  P10OUT &= ~0x10;              //Start SPI data send
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = CH_05;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save received SPI data
  *SPI_save_ptr = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  
  /*         Second 8-bit data send                 */
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = SPI_2;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save 2nd received SPI data
  *(SPI_save_ptr+1) = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  //Turn on CS pin (: SPI transmit end notification)
  P10OUT |= 0x10;       //End SPI data send
  
  //Run SDA
  //SDA step 1. Check if the spike is already detected on CH01
  if(Spike[Current_CH])
  {
    //Save the oldest SPI_data into BT data buffer.
    Packet_addr = Spike[Current_CH] - 1;
    stt_addr = &(BT_Tx_Packet_Buf[Packet_addr][BT_Tx_Rest[Packet_addr]]);
    
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    
    BT_Tx_Rest[Packet_addr] += 2;
    
    //Check if the BT buffer is full
    if(BT_Tx_Rest[Packet_addr] >= ucBTS)
    {
      Spike[Current_CH] = 0;
    }
    
  }
  //SDA step 2. Check if recently read data is enough to set as spike
  else if((*SPI_save_ptr & 0x80) && (*SPI_save_ptr < NVTH_CH_05) && BT_Write_ok)
  {
    //Assign BT buffer space and update current buffer filling state
    stt_addr = BT_Tx_Packet_Buf[BT_Tx_Packet_Ass_From];
    Spike[Current_CH] = ++BT_Tx_Packet_Ass_From;
    if(BT_Tx_Packet_Ass_From == ucBTPBN) BT_Tx_Packet_Ass_From=0;
    
    BT_Write_ok = (BT_Tx_Packet_Ass_To != BT_Tx_Packet_Ass_From + 1) || ((!!BT_Tx_Packet_Ass_To) || (BT_Tx_Packet_Ass_From != ucBTPBN-1));
    
    //Save the header information.
    *stt_addr++ = Current_CH + ((MSP430Ticks&0x0F)<<4);
    *stt_addr++ = ((MSP430Ticks>>4)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>12)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>20)&0xFF);
    
    //Save the oldest SPI data into BT data buffer.
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    //Update currently saved data size in the assigned BT buf.
    BT_Tx_Rest[Spike[Current_CH] - 1] = 6;
  }
  
  //Set Buf address as next channel
  SPI_save_ptr += 18;
  ++Current_CH;
  
  
  //6666666666666666666666666666666666666666666666666//////////
  /*         First 8-bit data of 16-bit send                 */
  //Turn off SPI_CS pin (: SPI selection)
  P10OUT &= ~0x10;              //Start SPI data send
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = CH_06;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save received SPI data
  *SPI_save_ptr = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  
  /*         Second 8-bit data send                 */
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = SPI_2;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save 2nd received SPI data
  *(SPI_save_ptr+1) = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  //Turn on CS pin (: SPI transmit end notification)
  P10OUT |= 0x10;       //End SPI data send
  
  //Run SDA
  //SDA step 1. Check if the spike is already detected on CH01
  if(Spike[Current_CH])
  {
    //Save the oldest SPI_data into BT data buffer.
    Packet_addr = Spike[Current_CH] - 1;
    stt_addr = &(BT_Tx_Packet_Buf[Packet_addr][BT_Tx_Rest[Packet_addr]]);
    
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    
    BT_Tx_Rest[Packet_addr] += 2;
    
    //Check if the BT buffer is full
    if(BT_Tx_Rest[Packet_addr] >= ucBTS)
    {
      Spike[Current_CH] = 0;
    }
    
  }
  //SDA step 2. Check if recently read data is enough to set as spike
  else if((*SPI_save_ptr & 0x80) && (*SPI_save_ptr < NVTH_CH_06) && BT_Write_ok)
  {
    //Assign BT buffer space and update current buffer filling state
    stt_addr = BT_Tx_Packet_Buf[BT_Tx_Packet_Ass_From];
    Spike[Current_CH] = ++BT_Tx_Packet_Ass_From;
    if(BT_Tx_Packet_Ass_From == ucBTPBN) BT_Tx_Packet_Ass_From=0;
    
    BT_Write_ok = (BT_Tx_Packet_Ass_To != BT_Tx_Packet_Ass_From + 1) || ((!!BT_Tx_Packet_Ass_To) || (BT_Tx_Packet_Ass_From != ucBTPBN-1));
    
    //Save the header information.
    *stt_addr++ = Current_CH + ((MSP430Ticks&0x0F)<<4);
    *stt_addr++ = ((MSP430Ticks>>4)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>12)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>20)&0xFF);
    
    //Save the oldest SPI data into BT data buffer.
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    //Update currently saved data size in the assigned BT buf.
    BT_Tx_Rest[Spike[Current_CH] - 1] = 6;
  }
  
  //Set Buf address as next channel
  SPI_save_ptr += 18;
  ++Current_CH;
  
  
  //7777777777777777777777777777777777777777777777777//////////
  /*         First 8-bit data of 16-bit send                 */
  //Turn off SPI_CS pin (: SPI selection)
  P10OUT &= ~0x10;              //Start SPI data send
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = CH_07;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save received SPI data
  *SPI_save_ptr = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  
  /*         Second 8-bit data send                 */
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = SPI_2;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save 2nd received SPI data
  *(SPI_save_ptr+1) = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  //Turn on CS pin (: SPI transmit end notification)
  P10OUT |= 0x10;       //End SPI data send
  
  //Run SDA
  //SDA step 1. Check if the spike is already detected on CH01
  if(Spike[Current_CH])
  {
    //Save the oldest SPI_data into BT data buffer.
    Packet_addr = Spike[Current_CH] - 1;
    stt_addr = &(BT_Tx_Packet_Buf[Packet_addr][BT_Tx_Rest[Packet_addr]]);
    
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    
    BT_Tx_Rest[Packet_addr] += 2;
    
    //Check if the BT buffer is full
    if(BT_Tx_Rest[Packet_addr] >= ucBTS)
    {
      Spike[Current_CH] = 0;
    }
    
  }
  //SDA step 2. Check if recently read data is enough to set as spike
  else if((*SPI_save_ptr & 0x80) && (*SPI_save_ptr < NVTH_CH_07) && BT_Write_ok)
  {
    //Assign BT buffer space and update current buffer filling state
    stt_addr = BT_Tx_Packet_Buf[BT_Tx_Packet_Ass_From];
    Spike[Current_CH] = ++BT_Tx_Packet_Ass_From;
    if(BT_Tx_Packet_Ass_From == ucBTPBN) BT_Tx_Packet_Ass_From=0;
    
    BT_Write_ok = (BT_Tx_Packet_Ass_To != BT_Tx_Packet_Ass_From + 1) || ((!!BT_Tx_Packet_Ass_To) || (BT_Tx_Packet_Ass_From != ucBTPBN-1));
    
    //Save the header information.
    *stt_addr++ = Current_CH + ((MSP430Ticks&0x0F)<<4);
    *stt_addr++ = ((MSP430Ticks>>4)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>12)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>20)&0xFF);
    
    //Save the oldest SPI data into BT data buffer.
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    //Update currently saved data size in the assigned BT buf.
    BT_Tx_Rest[Spike[Current_CH] - 1] = 6;
  }
  
  //Set Buf address as next channel
  SPI_save_ptr += 18;
  ++Current_CH;
  
  
  //8888888888888888888888888888888888888888888888888//////////
  /*         First 8-bit data of 16-bit send                 */
  //Turn off SPI_CS pin (: SPI selection)
  P10OUT &= ~0x10;              //Start SPI data send
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = CH_08;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save received SPI data
  *SPI_save_ptr = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  
  /*         Second 8-bit data send                 */
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = SPI_2;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save 2nd received SPI data
  *(SPI_save_ptr+1) = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  //Turn on CS pin (: SPI transmit end notification)
  P10OUT |= 0x10;       //End SPI data send
  
  //Run SDA
  //SDA step 1. Check if the spike is already detected on CH01
  if(Spike[Current_CH])
  {
    //Save the oldest SPI_data into BT data buffer.
    Packet_addr = Spike[Current_CH] - 1;
    stt_addr = &(BT_Tx_Packet_Buf[Packet_addr][BT_Tx_Rest[Packet_addr]]);
    
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    
    BT_Tx_Rest[Packet_addr] += 2;
    
    //Check if the BT buffer is full
    if(BT_Tx_Rest[Packet_addr] >= ucBTS)
    {
      Spike[Current_CH] = 0;
    }
    
  }
  //SDA step 2. Check if recently read data is enough to set as spike
  else if((*SPI_save_ptr & 0x80) && (*SPI_save_ptr < NVTH_CH_08) && BT_Write_ok)
  {
    //Assign BT buffer space and update current buffer filling state
    stt_addr = BT_Tx_Packet_Buf[BT_Tx_Packet_Ass_From];
    Spike[Current_CH] = ++BT_Tx_Packet_Ass_From;
    if(BT_Tx_Packet_Ass_From == ucBTPBN) BT_Tx_Packet_Ass_From=0;
    
    BT_Write_ok = (BT_Tx_Packet_Ass_To != BT_Tx_Packet_Ass_From + 1) || ((!!BT_Tx_Packet_Ass_To) || (BT_Tx_Packet_Ass_From != ucBTPBN-1));
    
    //Save the header information.
    *stt_addr++ = Current_CH + ((MSP430Ticks&0x0F)<<4);
    *stt_addr++ = ((MSP430Ticks>>4)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>12)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>20)&0xFF);
    
    //Save the oldest SPI data into BT data buffer.
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    //Update currently saved data size in the assigned BT buf.
    BT_Tx_Rest[Spike[Current_CH] - 1] = 6;
  }
  
  //Set Buf address as next channel
  SPI_save_ptr += 18;
  ++Current_CH;
  
  
  //9999999999999999999999999999999999999999999999999//////////
  /*         First 8-bit data of 16-bit send                 */
  //Turn off SPI_CS pin (: SPI selection)
  P10OUT &= ~0x10;              //Start SPI data send
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = CH_09;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save received SPI data
  *SPI_save_ptr = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  
  /*         Second 8-bit data send                 */
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = SPI_2;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save 2nd received SPI data
  *(SPI_save_ptr+1) = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  //Turn on CS pin (: SPI transmit end notification)
  P10OUT |= 0x10;       //End SPI data send
  
  //Run SDA
  //SDA step 1. Check if the spike is already detected on CH01
  if(Spike[Current_CH])
  {
    //Save the oldest SPI_data into BT data buffer.
    Packet_addr = Spike[Current_CH] - 1;
    stt_addr = &(BT_Tx_Packet_Buf[Packet_addr][BT_Tx_Rest[Packet_addr]]);
    
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    
    BT_Tx_Rest[Packet_addr] += 2;
    
    //Check if the BT buffer is full
    if(BT_Tx_Rest[Packet_addr] >= ucBTS)
    {
      Spike[Current_CH] = 0;
    }
    
  }
  //SDA step 2. Check if recently read data is enough to set as spike
  else if((*SPI_save_ptr & 0x80) && (*SPI_save_ptr < NVTH_CH_09) && BT_Write_ok)
  {
    //Assign BT buffer space and update current buffer filling state
    stt_addr = BT_Tx_Packet_Buf[BT_Tx_Packet_Ass_From];
    Spike[Current_CH] = ++BT_Tx_Packet_Ass_From;
    if(BT_Tx_Packet_Ass_From == ucBTPBN) BT_Tx_Packet_Ass_From=0;
    
    BT_Write_ok = (BT_Tx_Packet_Ass_To != BT_Tx_Packet_Ass_From + 1) || ((!!BT_Tx_Packet_Ass_To) || (BT_Tx_Packet_Ass_From != ucBTPBN-1));
    
    //Save the header information.
    *stt_addr++ = Current_CH + ((MSP430Ticks&0x0F)<<4);
    *stt_addr++ = ((MSP430Ticks>>4)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>12)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>20)&0xFF);
    
    //Save the oldest SPI data into BT data buffer.
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    //Update currently saved data size in the assigned BT buf.
    BT_Tx_Rest[Spike[Current_CH] - 1] = 6;
  }
  
  //Set Buf address as next channel
  SPI_save_ptr += 18;
  ++Current_CH;
  
  
  //AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA//////////
  /*         First 8-bit data of 16-bit send                 */
  //Turn off SPI_CS pin (: SPI selection)
  P10OUT &= ~0x10;              //Start SPI data send
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = CH_10;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save received SPI data
  *SPI_save_ptr = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  
  /*         Second 8-bit data send                 */
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = SPI_2;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save 2nd received SPI data
  *(SPI_save_ptr+1) = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  //Turn on CS pin (: SPI transmit end notification)
  P10OUT |= 0x10;       //End SPI data send
  
  //Run SDA
  //SDA step 1. Check if the spike is already detected on CH01
  if(Spike[Current_CH])
  {
    //Save the oldest SPI_data into BT data buffer.
    Packet_addr = Spike[Current_CH] - 1;
    stt_addr = &(BT_Tx_Packet_Buf[Packet_addr][BT_Tx_Rest[Packet_addr]]);
    
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    
    BT_Tx_Rest[Packet_addr] += 2;
    
    //Check if the BT buffer is full
    if(BT_Tx_Rest[Packet_addr] >= ucBTS)
    {
      Spike[Current_CH] = 0;
    }
    
  }
  //SDA step 2. Check if recently read data is enough to set as spike
  else if((*SPI_save_ptr & 0x80) && (*SPI_save_ptr < NVTH_CH_10) && BT_Write_ok)
  {
    //Assign BT buffer space and update current buffer filling state
    stt_addr = BT_Tx_Packet_Buf[BT_Tx_Packet_Ass_From];
    Spike[Current_CH] = ++BT_Tx_Packet_Ass_From;
    if(BT_Tx_Packet_Ass_From == ucBTPBN) BT_Tx_Packet_Ass_From=0;
    
    BT_Write_ok = (BT_Tx_Packet_Ass_To != BT_Tx_Packet_Ass_From + 1) || ((!!BT_Tx_Packet_Ass_To) || (BT_Tx_Packet_Ass_From != ucBTPBN-1));
    
    //Save the header information.
    *stt_addr++ = Current_CH + ((MSP430Ticks&0x0F)<<4);
    *stt_addr++ = ((MSP430Ticks>>4)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>12)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>20)&0xFF);
    
    //Save the oldest SPI data into BT data buffer.
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    //Update currently saved data size in the assigned BT buf.
    BT_Tx_Rest[Spike[Current_CH] - 1] = 6;
  }
  
  //Set Buf address as next channel
  SPI_save_ptr += 18;
  ++Current_CH;
  
  
  //BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB//////////
  /*         First 8-bit data of 16-bit send                 */
  //Turn off SPI_CS pin (: SPI selection)
  P10OUT &= ~0x10;              //Start SPI data send
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = CH_11;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save received SPI data
  *SPI_save_ptr = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  
  /*         Second 8-bit data send                 */
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = SPI_2;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save 2nd received SPI data
  *(SPI_save_ptr+1) = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  //Turn on CS pin (: SPI transmit end notification)
  P10OUT |= 0x10;       //End SPI data send
  
  //Run SDA
  //SDA step 1. Check if the spike is already detected on CH01
  if(Spike[Current_CH])
  {
    //Save the oldest SPI_data into BT data buffer.
    Packet_addr = Spike[Current_CH] - 1;
    stt_addr = &(BT_Tx_Packet_Buf[Packet_addr][BT_Tx_Rest[Packet_addr]]);
    
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    
    BT_Tx_Rest[Packet_addr] += 2;
    
    //Check if the BT buffer is full
    if(BT_Tx_Rest[Packet_addr] >= ucBTS)
    {
      Spike[Current_CH] = 0;
    }
    
  }
  //SDA step 2. Check if recently read data is enough to set as spike
  else if((*SPI_save_ptr & 0x80) && (*SPI_save_ptr < NVTH_CH_11) && BT_Write_ok)  //-50uV보다 작냐 ㅠ// 0x80이 -를 의미//&대신 !로 바꿔주면 +의미 
  {
    //Assign BT buffer space and update current buffer filling state
    stt_addr = BT_Tx_Packet_Buf[BT_Tx_Packet_Ass_From];
    Spike[Current_CH] = ++BT_Tx_Packet_Ass_From;
    if(BT_Tx_Packet_Ass_From == ucBTPBN) BT_Tx_Packet_Ass_From=0;
    
    BT_Write_ok = (BT_Tx_Packet_Ass_To != BT_Tx_Packet_Ass_From + 1) || ((!!BT_Tx_Packet_Ass_To) || (BT_Tx_Packet_Ass_From != ucBTPBN-1));
    
    //Save the header information.
    *stt_addr++ = Current_CH + ((MSP430Ticks&0x0F)<<4);
    *stt_addr++ = ((MSP430Ticks>>4)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>12)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>20)&0xFF);
    
    //Save the oldest SPI data into BT data buffer.
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    //Update currently saved data size in the assigned BT buf.
    BT_Tx_Rest[Spike[Current_CH] - 1] = 6;
  }
  
  //Set Buf address as next channel
  SPI_save_ptr += 18;
  ++Current_CH;
  
  
  //CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC//////////
  /*         First 8-bit data of 16-bit send                 */
  //Turn off SPI_CS pin (: SPI selection)
  P10OUT &= ~0x10;              //Start SPI data send
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = CH_12;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save received SPI data
  *SPI_save_ptr = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  
  /*         Second 8-bit data send                 */
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = SPI_2;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save 2nd received SPI data
  *(SPI_save_ptr+1) = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  //Turn on CS pin (: SPI transmit end notification)
  P10OUT |= 0x10;       //End SPI data send
  
  //Run SDA
  //SDA step 1. Check if the spike is already detected on CH01
  if(Spike[Current_CH])
  {
    //Save the oldest SPI_data into BT data buffer.
    Packet_addr = Spike[Current_CH] - 1;
    stt_addr = &(BT_Tx_Packet_Buf[Packet_addr][BT_Tx_Rest[Packet_addr]]);
    
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    
    BT_Tx_Rest[Packet_addr] += 2;
    
    //Check if the BT buffer is full
    if(BT_Tx_Rest[Packet_addr] >= ucBTS)
    {
      Spike[Current_CH] = 0;
    }
    
  }
  //SDA step 2. Check if recently read data is enough to set as spike
  else if((*SPI_save_ptr & 0x80) && (*SPI_save_ptr < NVTH_CH_12) && BT_Write_ok)
  {
    //Assign BT buffer space and update current buffer filling state
    stt_addr = BT_Tx_Packet_Buf[BT_Tx_Packet_Ass_From];
    Spike[Current_CH] = ++BT_Tx_Packet_Ass_From;
    if(BT_Tx_Packet_Ass_From == ucBTPBN) BT_Tx_Packet_Ass_From=0;
    
    BT_Write_ok = (BT_Tx_Packet_Ass_To != BT_Tx_Packet_Ass_From + 1) || ((!!BT_Tx_Packet_Ass_To) || (BT_Tx_Packet_Ass_From != ucBTPBN-1));
    
    //Save the header information.
    *stt_addr++ = Current_CH + ((MSP430Ticks&0x0F)<<4);
    *stt_addr++ = ((MSP430Ticks>>4)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>12)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>20)&0xFF);
    
    //Save the oldest SPI data into BT data buffer.
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    //Update currently saved data size in the assigned BT buf.
    BT_Tx_Rest[Spike[Current_CH] - 1] = 6;
  }
  
  //Set Buf address as next channel
  SPI_save_ptr += 18;
  ++Current_CH;
  
  
  //DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD//////////
  /*         First 8-bit data of 16-bit send                 */
  //Turn off SPI_CS pin (: SPI selection)
  P10OUT &= ~0x10;              //Start SPI data send
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = CH_13;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save received SPI data
  *SPI_save_ptr = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  
  /*         Second 8-bit data send                 */
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = SPI_2;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save 2nd received SPI data
  *(SPI_save_ptr+1) = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  //Turn on CS pin (: SPI transmit end notification)
  P10OUT |= 0x10;       //End SPI data send
  
  //Run SDA
  //SDA step 1. Check if the spike is already detected on CH01
  if(Spike[Current_CH])
  {
    //Save the oldest SPI_data into BT data buffer.
    Packet_addr = Spike[Current_CH] - 1;
    stt_addr = &(BT_Tx_Packet_Buf[Packet_addr][BT_Tx_Rest[Packet_addr]]);
    
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    
    BT_Tx_Rest[Packet_addr] += 2;
    
    //Check if the BT buffer is full
    if(BT_Tx_Rest[Packet_addr] >= ucBTS)
    {
      Spike[Current_CH] = 0;
    }
    
  }
  //SDA step 2. Check if recently read data is enough to set as spike
  else if((*SPI_save_ptr & 0x80) && (*SPI_save_ptr < NVTH_CH_13) && BT_Write_ok)
  {
    //Assign BT buffer space and update current buffer filling state
    stt_addr = BT_Tx_Packet_Buf[BT_Tx_Packet_Ass_From];
    Spike[Current_CH] = ++BT_Tx_Packet_Ass_From;
    if(BT_Tx_Packet_Ass_From == ucBTPBN) BT_Tx_Packet_Ass_From=0;
    
    BT_Write_ok = (BT_Tx_Packet_Ass_To != BT_Tx_Packet_Ass_From + 1) || ((!!BT_Tx_Packet_Ass_To) || (BT_Tx_Packet_Ass_From != ucBTPBN-1));
    
    //Save the header information.
    *stt_addr++ = Current_CH + ((MSP430Ticks&0x0F)<<4);
    *stt_addr++ = ((MSP430Ticks>>4)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>12)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>20)&0xFF);
    
    //Save the oldest SPI data into BT data buffer.
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    //Update currently saved data size in the assigned BT buf.
    BT_Tx_Rest[Spike[Current_CH] - 1] = 6;
  }
  
  //Set Buf address as next channel
  SPI_save_ptr += 18;
  ++Current_CH;
  
  
  //EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE//////////
  /*         First 8-bit data of 16-bit send                 */
  //Turn off SPI_CS pin (: SPI selection)
  P10OUT &= ~0x10;              //Start SPI data send
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = CH_14;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save received SPI data
  *SPI_save_ptr = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  
  /*         Second 8-bit data send                 */
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = SPI_2;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save 2nd received SPI data
  *(SPI_save_ptr+1) = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  //Turn on CS pin (: SPI transmit end notification)
  P10OUT |= 0x10;       //End SPI data send
  
  //Run SDA
  //SDA step 1. Check if the spike is already detected on CH01
  if(Spike[Current_CH])
  {
    //Save the oldest SPI_data into BT data buffer.
    Packet_addr = Spike[Current_CH] - 1;
    stt_addr = &(BT_Tx_Packet_Buf[Packet_addr][BT_Tx_Rest[Packet_addr]]);
    
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    
    BT_Tx_Rest[Packet_addr] += 2;
    
    //Check if the BT buffer is full
    if(BT_Tx_Rest[Packet_addr] >= ucBTS)
    {
      Spike[Current_CH] = 0;
    }
    
  }
  //SDA step 2. Check if recently read data is enough to set as spike
  else if((*SPI_save_ptr & 0x80) && (*SPI_save_ptr < NVTH_CH_14) && BT_Write_ok)
  {
    //Assign BT buffer space and update current buffer filling state
    stt_addr = BT_Tx_Packet_Buf[BT_Tx_Packet_Ass_From];
    Spike[Current_CH] = ++BT_Tx_Packet_Ass_From;
    if(BT_Tx_Packet_Ass_From == ucBTPBN) BT_Tx_Packet_Ass_From=0;
    
    BT_Write_ok = (BT_Tx_Packet_Ass_To != BT_Tx_Packet_Ass_From + 1) || ((!!BT_Tx_Packet_Ass_To) || (BT_Tx_Packet_Ass_From != ucBTPBN-1));
    
    //Save the header information.
    *stt_addr++ = Current_CH + ((MSP430Ticks&0x0F)<<4);
    *stt_addr++ = ((MSP430Ticks>>4)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>12)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>20)&0xFF);
    
    //Save the oldest SPI data into BT data buffer.
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    //Update currently saved data size in the assigned BT buf.
    BT_Tx_Rest[Spike[Current_CH] - 1] = 6;
  }
  
  //Set Buf address as next channel
  SPI_save_ptr += 18;
  ++Current_CH;
  
  
  //FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF//////////
  /*         First 8-bit data of 16-bit send                 */
  //Turn off SPI_CS pin (: SPI selection)
  P10OUT &= ~0x10;              //Start SPI data send
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = CH_15;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save received SPI data
  *SPI_save_ptr = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  
  /*         Second 8-bit data send                 */
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = SPI_2;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save 2nd received SPI data
  *(SPI_save_ptr+1) = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  //Turn on CS pin (: SPI transmit end notification)
  P10OUT |= 0x10;       //End SPI data send
  
  //Run SDA
  //SDA step 1. Check if the spike is already detected on CH01
  if(Spike[Current_CH])
  {
    //Save the oldest SPI_data into BT data buffer.
    Packet_addr = Spike[Current_CH] - 1;
    stt_addr = &(BT_Tx_Packet_Buf[Packet_addr][BT_Tx_Rest[Packet_addr]]);
    
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    
    BT_Tx_Rest[Packet_addr] += 2;
    
    //Check if the BT buffer is full
    if(BT_Tx_Rest[Packet_addr] >= ucBTS)
    {
      Spike[Current_CH] = 0;
    }
    
  }
  //SDA step 2. Check if recently read data is enough to set as spike
  else if((*SPI_save_ptr & 0x80) && (*SPI_save_ptr < NVTH_CH_15) && BT_Write_ok)
  {
    //Assign BT buffer space and update current buffer filling state
    stt_addr = BT_Tx_Packet_Buf[BT_Tx_Packet_Ass_From];
    Spike[Current_CH] = ++BT_Tx_Packet_Ass_From;
    if(BT_Tx_Packet_Ass_From == ucBTPBN) BT_Tx_Packet_Ass_From=0;
    
    BT_Write_ok = (BT_Tx_Packet_Ass_To != BT_Tx_Packet_Ass_From + 1) || ((!!BT_Tx_Packet_Ass_To) || (BT_Tx_Packet_Ass_From != ucBTPBN-1));
    
    //Save the header information.
    *stt_addr++ = Current_CH + ((MSP430Ticks&0x0F)<<4);
    *stt_addr++ = ((MSP430Ticks>>4)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>12)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>20)&0xFF);
    
    //Save the oldest SPI data into BT data buffer.
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    //Update currently saved data size in the assigned BT buf.
    BT_Tx_Rest[Spike[Current_CH] - 1] = 6;
  }
  
  //Set Buf address as next channel
  SPI_save_ptr += 18;
  ++Current_CH;
  
  
  //GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG//////////
  /*         First 8-bit data of 16-bit send                 */
  //Turn off SPI_CS pin (: SPI selection)
  P10OUT &= ~0x10;              //Start SPI data send
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = CH_16;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save received SPI data
  *SPI_save_ptr = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  
  /*         Second 8-bit data send                 */
  //wait for SPI transmit ready
  while(!(UCB3IFG & UCTXIFG));
  //Data write in Tx buffer register
  UCB3TXBUF = SPI_2;
  //Wait for input(to SOMI) completion
  while(!(UCB3IFG & UCRXIFG));
  //Save 2nd received SPI data
  *(SPI_save_ptr+1) = UCB3RXBUF;
  //Wait for end SPI operation
  while(UCB3STAT & UCBUSY);
  //Turn on CS pin (: SPI transmit end notification)
  P10OUT |= 0x10;       //End SPI data send
  
  //Run SDA
  //SDA step 1. Check if the spike is already detected on CH01
  if(Spike[Current_CH])
  {
    //Save the oldest SPI_data into BT data buffer.
    Packet_addr = Spike[Current_CH] - 1;
    stt_addr = &(BT_Tx_Packet_Buf[Packet_addr][BT_Tx_Rest[Packet_addr]]);
    
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    
    BT_Tx_Rest[Packet_addr] += 2;
    
    //Check if the BT buffer is full
    if(BT_Tx_Rest[Packet_addr] >= ucBTS)
    {
      Spike[Current_CH] = 0;
    }
    
  }
  //SDA step 2. Check if recently read data is enough to set as spike
  else if((*SPI_save_ptr & 0x80) && (*SPI_save_ptr < NVTH_CH_16) && BT_Write_ok)
  {
    //Assign BT buffer space and update current buffer filling state
    stt_addr = BT_Tx_Packet_Buf[BT_Tx_Packet_Ass_From];
    Spike[Current_CH] = ++BT_Tx_Packet_Ass_From;
    if(BT_Tx_Packet_Ass_From == ucBTPBN) BT_Tx_Packet_Ass_From=0;
    
    BT_Write_ok = (BT_Tx_Packet_Ass_To != BT_Tx_Packet_Ass_From + 1) || ((!!BT_Tx_Packet_Ass_To) || (BT_Tx_Packet_Ass_From != ucBTPBN-1));
    
    //Save the header information.
    *stt_addr++ = Current_CH + ((MSP430Ticks&0x0F)<<4);
    *stt_addr++ = ((MSP430Ticks>>4)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>12)&0xFF);
    *stt_addr++ = ((MSP430Ticks>>20)&0xFF);
    
    //Save the oldest SPI data into BT data buffer.
    if(SPI_Rx_Addr != ucSPSBS - 2)
    {
      *stt_addr++ = *(SPI_save_ptr + 2);
      *stt_addr = *(SPI_save_ptr +3);
    }
    else
    {
      *stt_addr++ = *(SPI_save_ptr + 2 - ucSPSBS);
      *stt_addr = *(SPI_save_ptr + 3 - ucSPSBS);
    }
    //Update currently saved data size in the assigned BT buf.
    BT_Tx_Rest[Spike[Current_CH] - 1] = 6;
  }
  
  
  
  
}

///////////////////////////////////////////////////////////////


///////////////////////////////////////////////
//      Fn      BL_Write_from_SPI
//      Des     1 cycle SPI result data transmitting
//              to BL module in UART protocol
//      Inp     order
//      Ret     (next step) order
///////////////////////////////////////////////
unsigned char BL_Write_from_SPI(unsigned char order)
{
  static unsigned char *remove_addr;
  
  ++order; // Initial input must be 0 value.
  
  
  switch(order)
  {
  case 1:
    //pre-data send
    
    if(!!(DMA0CTL & DMAEN)) --order; //DMA is in operation
    else
    {
      //DMA is not in operation
      
      //Set the DMA option.
      DMACTL0 = DMA0TSEL_17;
      __data16_write_addr((unsigned short) & DMA0SA, (unsigned long) BT_Tx_Protocol);
      __data16_write_addr((unsigned short) & DMA0DA, (unsigned long) &UCA0TXBUF);
      DMA0SZ = 14;
      DMA0CTL = DMASRCINCR_3 + DMASBDB + DMALEVEL;
      
      //Start to send pre-data.
      DMA0CTL |= DMAEN;
    }

    break;
  case 2:
    //wait until pre-data will be sent.
    if(!!(DMA0CTL & DMAIFG))
    {
      //pre-data sending was finished.
      
      //reset interrupt flag.
      DMA0CTL &= ~ DMAIFG;
      //setting to send data
      DMACTL0 = DMA0TSEL_17;
      __data16_write_addr((unsigned short) & DMA0SA, (unsigned long) &BT_Tx_Packet_Buf[BT_Tx_Packet_Ass_To]);
      __data16_write_addr((unsigned short) & DMA0DA, (unsigned long) &UCA0TXBUF);
      DMA0SZ = ucBTS4;
      DMA0CTL = DMASRCINCR_3 + DMASBDB + DMALEVEL;
      
      //start data sending.
      DMA0CTL |= DMAEN;
      
      return order;
    }
    else return --order; //pre-data sending was not finished.
    break;
  case 3:
    //Check if the data sending was finished.
    if(!!(DMA0CTL & DMAIFG))
    {
      //data transmission was finished.
      
      //Reset DMA mode.
      DMA0CTL &= ~ DMAIFG;
      
      
      //Start to send post-data.
      
      //Set the DMA option.
      DMACTL0 = DMA0TSEL_17;
      __data16_write_addr((unsigned short) & DMA0SA, (unsigned long) &BT_Tx_Protocol[14]);
      __data16_write_addr((unsigned short) & DMA0DA, (unsigned long) &UCA0TXBUF);
      DMA0SZ = 2;
      DMA0CTL = DMASRCINCR_3 + DMASBDB + DMALEVEL;
      
      //Start to send pre-data.
      DMA0CTL |= DMAEN;
      
      
      //Pass away transmitted data.
      remove_addr = BT_Tx_Rest + BT_Tx_Packet_Ass_To;
      
      *remove_addr=0;
      *(remove_addr+1)=0;
      *(remove_addr+2)=0;
      *(remove_addr+3)=0;
      
      BT_Tx_Packet_Ass_To += 4;
      
      if(BT_Tx_Packet_Ass_To == ucBTPBN) BT_Tx_Packet_Ass_To=0;
      
      return order;
    }
    else return --order; //data sending was not finished.
    break;
  case 4:
    if(!!(DMA0CTL & DMAIFG)) DMA0CTL &= ~ DMAIFG; //post-data transmission was finished.
                                                  //Reset DMA flag.
    else return --order;
    break;
  default:
    break;
  }
  
  return order;
}
    
///////////////////////////////////////////////
//      Fn      SPI_BL_Periodic_write
//      Des     Periodically SPI data comm. and
//              BL data transmission by direct 
//              control
//      Inp     NONE
//      Ret     NONE
///////////////////////////////////////////////

void SPI_BL_Periodic_write(void *Userparameter)
{
  static unsigned char order=0;
  static unsigned char work1=0;
  static unsigned char work2=0;
  
  //Periodically ADC sensing must be implemented by SPI communication every cycle
  //Read and save ADC data on no.1 to no 4 channel (no.0 was false operated)
  if(!work1)
  {
    work1=1;
    RHD_SPI_Buffer_Save();
    work1=0;
  
    //If the size of data in buffer is bigger than BL_TRANS_SIZE, 
    //transmit the data to BT module in the way of direct UART control
    if((!work2) && ((BT_Tx_Rest[BT_Tx_Packet_Ass_To+3]==ucBTS) || (order) ) )
    {
      work2=1;
      //Yes. ready to transmit
      
      //Transmit UART data to BT module
      order=BL_Write_from_SPI(order);
      if(order>BT_TRANS_STEP_SIZE)
      {
        //End transmission mode.
        //Reset all the transmission information.
        order=0;
      }
      work2=0;
    }
    
  }
}
         
void BL_UART_Bulk_Transmission_Mode(void)
{
   /* Ensure the timer is stopped.                                      */
   TA1CTL = 0;

   /* Run the timer off of the ACLK.                                    */
//   TA1CTL = 0x200u | 0xC0u; // SMCLK(16MHz), 1/8 rate ==> 2MHz

   /* ACLK, 1/1 ration division */
   //TASSEL_1   -> Using ACLK clock
   //ID_0       -> Division factor : 1
   //TA1CTL = TASSEL_1 | ID_0;
//   TA1CTL = TASSEL_2 | ID_3;
   TA1CTL = TASSEL_2 | ID_3;

   /* Clear everything to start with.                                   */
   TA1CTL |= TACLR;

   /* Set the compare match value according to the tick rate we want.   */
//   TA1CCR0 = 2000; // 2MHz / 100 = 20 kHz

   /* TA1CCR0 calculation chart
   Sampling rate                TA1CCR0
   08 kHz (07.992 kHz)          (25,000 kHz / 8 / 8 kHz) - 1 = 390
   10 kHz (09.984 kHz)          (25,000 kHz / 8 / 10 kHz) - 1 = 312
   12 kHz (12.019 kHz)          (25,000 / 8 / 12) - 1 = 259
   14 kHz (14.013 kHz)          (25,000 / 8 / 14) - 1 = 222
   16 kHz (16.108 kHz)          (25,000 / 8 / 16) - 1 = 193
   18 kHz (17.960 kHz)          (25,000 / 8 / 18) - 1 = 173
   */
   TA1CCR0 = 390; // 25MHz / 8 / (313 + 1) = 9.95 kHz

   //two division
//   TA1CCR0 = 2;
   //TA1CCR0 = ( 32768 / 1000 ) + 1  -10;

   /* Enable the interrupts.                                            */
   TA1CCTL0 = CCIE;

   /* Start up clean.                                                   */
   TA1CTL |= TACLR;

   /* Up mode.                                                          */
//   TA1CTL |= TASSEL_1 | MC_1 | ID_0;
//   TA1CTL |= TASSEL_2 | MC_1 | ID_3;
   TA1CTL |= TASSEL_2 | MC_1 | ID_3;
   
   
   //UART-BL transmission buffer ready flag set (initialization)
   UCA2IFG |= 0x02;
   
}

////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////Timer////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
   /* This function is called to get the system Tick Count.             */
unsigned long HAL_GetTickCount(void)
{
   return(MSP430Ticks);
}

   /* Timer A Get Tick Count Function for BTPSKRNL Timer A Interrupt.   */
   /* Included for Non-OS builds                                        */
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER_INTERRUPT(void)
{
   ++MSP430Ticks;
   /* Start up clean.                                                   */
   TA1CTL |= TACLR;
   
   //if(Cycle_start) RHD_SPI_Buffer_Save(NULL);
   if(Cycle_start) SPI_BL_Periodic_write(NULL);

   /* Exit from LPM if necessary (this statement will have no effect if */
   /* we are not currently in low power mode).                          */
   //LPM3_EXIT;
   
}


////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////Automatic operation mode/////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void AUTOMODE_Display(void)
{
  Display(("\r\n\n\n============================================================"));
  Display(("\r\nAutomatic Blutooth/MCU/ADC chip Control mode"));
  Display(("\r\n============================================================\r\n\n"));
}

void AUTOMODE_Start_Automode(void)
{
  ///////////////////////////////////////////////////////////
  {
    AUTOMODE_Display();
    Display(("Server (AUTO-WRITTEN)\r\n"));
    ServerMode(NULL);
  }
}

void AUTOMODE_SetBaudRate(void *Userparameter)
{
  ParameterList_t param;
  
  BTPS_DeleteFunctionFromScheduler(AUTOMODE_SetBaudRate,NULL);
  
  param.NumberofParameters=1;
  param.Params[0].intParam=(DWord_t)HS_BAUD_RATE;
  
  Display(("SETBAUDRATE 921600 (AUTO-WRITTEN)\r\n"));
  
  SetBaudRate(&param);
  BTPS_AddFunctionToScheduler(AUTOMODE_SetConfigParams, NULL, 1000);
}

void AUTOMODE_SetConfigParams(void *Userparameter)
{
  ParameterList_t param;
  
  BTPS_DeleteFunctionFromScheduler(AUTOMODE_SetConfigParams,NULL);
  
  param.NumberofParameters=3;
  param.Params[0].intParam=(DWord_t)329;
  param.Params[1].intParam=(DWord_t)987;
  param.Params[2].intParam=(DWord_t)2303;
  
  Display(("SETCONFIGPARAMS 329 987 2303 (AUTO-WRTTEN)\r\n"));
  
  SetConfigParams(&param);
  BTPS_AddFunctionToScheduler(AUTOMODE_OpenServer, NULL, 1000);
}

void AUTOMODE_OpenServer(void *Userparameter)
{
  ParameterList_t param;
  
  BTPS_DeleteFunctionFromScheduler(AUTOMODE_OpenServer,NULL);
  
  param.NumberofParameters=1;
  param.Params[0].intParam=(DWord_t)1;
  
  Display(("OPEN 1 (AUTO-WRTTEN)\r\n"));
  
  OpenServer(&param);
}

void AUTOMODE_CountBeforeWrite(void *Userparameter)
{
  ParameterList_t param;
  static unsigned char count=0;
  
  if(count<5)
  {
    if(count==0)
    {
      Display(("\r\n\n\n======================================================================"));
      Display(("\r\nAutomatic Blutooth/MCU/ADC chip Control mode"));
    }
  Display(("\r\nCaution - This device will be in continuous transmit mode after %d sec.",5-count));
  
  count++;
  if(count==5)
  {
    Display(("\r\n======================================================================"));
  }
  }
  else if(count==5)
  {
  
  
  
  
    param.NumberofParameters=1;
    param.Params[0].intParam=(DWord_t)1;
    
    Display(("\r\nWrite 1 (AUTO-WRTTEN)\r\n"));
    count++;
  }
  else
  {
    BTPS_DeleteFunctionFromScheduler(AUTOMODE_CountBeforeWrite,NULL);
    Write(&param);
  }
}