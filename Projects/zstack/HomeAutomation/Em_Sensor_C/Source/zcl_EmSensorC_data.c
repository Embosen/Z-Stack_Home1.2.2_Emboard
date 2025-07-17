/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDConfig.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ezmode.h"
#include "zcl_poll_control.h"
#include "zcl_electrical_measurement.h"
#include "zcl_diagnostic.h"
#include "zcl_meter_identification.h"
#include "zcl_appliance_identification.h"
#include "zcl_appliance_events_alerts.h"
#include "zcl_power_profile.h"
#include "zcl_appliance_control.h"
#include "zcl_appliance_statistics.h"
#include "zcl_hvac.h"

#include "zcl_EmSensorC.h"

/*********************************************************************
 * CONSTANTS
 */

#define SAMPLELIGHT_DEVICE_VERSION     0
#define SAMPLELIGHT_FLAGS              0

#define SAMPLELIGHT_HWVERSION          1
#define SAMPLELIGHT_ZCLVERSION         1

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Basic Cluster
const uint8 zclSampleLight_HWRevision = SAMPLELIGHT_HWVERSION;
const uint8 zclSampleLight_ZCLVersion = SAMPLELIGHT_ZCLVERSION;

const uint8 zclSampleLight_ManufacturerName[] = { 10, 'E','m','b','o','a','r','d','.','C','o' };
const uint8 zclSampleLight_ModelId[] = { 11, 'E','m','-','S','e','n','s','o','r','-','C' };
const uint8 zclSampleLight_DateCode[] = { 8, '2','0','2','5','0','7','1','7' };
const uint8 zclSampleLight_PowerSource = POWER_SOURCE_MAINS_1_PHASE;

uint8 zclSampleLight_LocationDescription[17] = { 16, ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' };
uint8 zclSampleLight_PhysicalEnvironment = 0;
uint8 zclSampleLight_DeviceEnable = DEVICE_ENABLED;

// Identify Cluster
uint16 zclSampleLight_IdentifyTime = 0;
#ifdef ZCL_EZMODE
uint8  zclSampleLight_IdentifyCommissionState;
#endif

// On/Off Cluster
uint8  zclSampleLight_OnOff = LIGHT_OFF;
uint8  zclSampleLight_RedOnOff = LIGHT_OFF;
uint8  zclSampleLight_Switch1OnOff = LIGHT_OFF;  // P1_2开关状态
uint8  zclSampleLight_Switch2OnOff = LIGHT_OFF;  // P1_3开关状态

// Level Control Cluster
#ifdef ZCL_LEVEL_CTRL
uint8  zclSampleLight_LevelCurrentLevel = ATTR_LEVEL_MIN_LEVEL;
uint16 zclSampleLight_LevelRemainingTime;
uint16 zclSampleLight_LevelOnOffTransitionTime = 20;
uint8  zclSampleLight_LevelOnLevel = ATTR_LEVEL_MID_LEVEL;
uint16 zclSampleLight_LevelOnTransitionTime = 20;
uint16 zclSampleLight_LevelOffTransitionTime = 20;
uint8  zclSampleLight_LevelDefaultMoveRate = 0;   // as fast as possible
#endif

#if ZCL_DISCOVER
CONST zclCommandRec_t zclSampleLight_Cmds[] =
{
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    COMMAND_BASIC_RESET_FACT_DEFAULT,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF,
    COMMAND_OFF,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF,
    COMMAND_ON,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF,
    COMMAND_TOGGLE,
    CMD_DIR_SERVER_RECEIVED
  },
#ifdef ZCL_LEVEL_CONTROL
  ,{
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_MOVE_TO_LEVEL,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_MOVE,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_STEP,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_STOP,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_MOVE_TO_LEVEL_WITH_ON_OFF,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_MOVE_WITH_ON_OFF,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_STEP_WITH_ON_OFF,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    COMMAND_LEVEL_STOP_WITH_ON_OFF,
    CMD_DIR_SERVER_RECEIVED
  }
#endif // ZCL_LEVEL_CONTROL
};

CONST uint8 zclCmdsArraySize = ( sizeof(zclSampleLight_Cmds) / sizeof(zclSampleLight_Cmds[0]) );
#endif // ZCL_DISCOVER

/*********************************************************************
 * ATTRIBUTE DEFINITIONS - Uses REAL cluster IDs
 */
CONST zclAttrRec_t zclSampleLight_Attrs[] =
{
  // *** General Basic Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_BASIC,             // Cluster IDs - defined in the foundation (ie. zcl.h)
    {  // Attribute record
      ATTRID_BASIC_HW_VERSION,            // Attribute ID - Found in Cluster Library header (ie. zcl_general.h)
      ZCL_DATATYPE_UINT8,                 // Data Type - found in zcl.h
      ACCESS_CONTROL_READ,                // Variable access control - found in zcl.h
      (void *)&zclSampleLight_HWRevision  // Pointer to attribute variable
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_ZCL_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_ZCLVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSampleLight_ManufacturerName
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MODEL_ID,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSampleLight_ModelId
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DATE_CODE,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSampleLight_DateCode
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_PowerSource
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_LOCATION_DESC,
      ZCL_DATATYPE_CHAR_STR,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)zclSampleLight_LocationDescription
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_PHYSICAL_ENV,
      ZCL_DATATYPE_UINT8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSampleLight_PhysicalEnvironment
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DEVICE_ENABLED,
      ZCL_DATATYPE_BOOLEAN,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSampleLight_DeviceEnable
    }
  },

#ifdef ZCL_IDENTIFY
  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSampleLight_IdentifyTime
    }
  },
 #ifdef ZCL_EZMODE
  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_COMMISSION_STATE,
      ZCL_DATATYPE_UINT8,
      (ACCESS_CONTROL_READ),
      (void *)&zclSampleLight_IdentifyCommissionState
    }
  },
 #endif // ZCL_EZMODE
#endif

  // *** On/Off Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF,
    { // Attribute record
      ATTRID_ON_OFF,
      ZCL_DATATYPE_BOOLEAN,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_OnOff
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF,
    { // Attribute record
      ATTRID_ON_OFF,
      ZCL_DATATYPE_BOOLEAN,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_RedOnOff
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF,
    { // Attribute record
      ATTRID_ON_OFF,
      ZCL_DATATYPE_BOOLEAN,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_Switch1OnOff
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF,
    { // Attribute record
      ATTRID_ON_OFF,
      ZCL_DATATYPE_BOOLEAN,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_Switch2OnOff
    }
  }

#ifdef ZCL_LEVEL_CTRL
  , {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_CURRENT_LEVEL,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_LevelCurrentLevel
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_REMAINING_TIME,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleLight_LevelRemainingTime
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_ON_OFF_TRANSITION_TIME,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclSampleLight_LevelOnOffTransitionTime
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_ON_LEVEL,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclSampleLight_LevelOnLevel
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_ON_TRANSITION_TIME,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclSampleLight_LevelOnTransitionTime
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_OFF_TRANSITION_TIME,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclSampleLight_LevelOffTransitionTime
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL,
    { // Attribute record
      ATTRID_LEVEL_DEFAULT_MOVE_RATE,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclSampleLight_LevelDefaultMoveRate
    }
  }
#endif
 #ifdef ZCL_DIAGNOSTIC
  , {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_NUMBER_OF_RESETS,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_PERSISTENT_MEMORY_WRITES,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_MAC_RX_BCAST,
      ZCL_DATATYPE_UINT32,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_MAC_TX_BCAST,
      ZCL_DATATYPE_UINT32,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_MAC_RX_UCAST,
      ZCL_DATATYPE_UINT32,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_MAC_TX_UCAST,
      ZCL_DATATYPE_UINT32,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_MAC_TX_UCAST_RETRY,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_MAC_TX_UCAST_FAIL,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_APS_RX_BCAST,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_APS_TX_BCAST,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_APS_RX_UCAST,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_APS_TX_UCAST_SUCCESS,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_APS_TX_UCAST_RETRY,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_APS_TX_UCAST_FAIL,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_ROUTE_DISC_INITIATED,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_NEIGHBOR_ADDED,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_NEIGHBOR_REMOVED,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_NEIGHBOR_STALE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_JOIN_INDICATION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_CHILD_MOVED,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_NWK_FC_FAILURE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_APS_FC_FAILURE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_APS_UNAUTHORIZED_KEY,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_NWK_DECRYPT_FAILURES,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_APS_DECRYPT_FAILURES,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_PACKET_BUFFER_ALLOCATE_FAILURES,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_RELAYED_UCAST,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_PHY_TO_MAC_QUEUE_LIMIT_REACHED,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_PACKET_VALIDATE_DROP_COUNT,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_AVERAGE_MAC_RETRY_PER_APS_MESSAGE_SENT,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_LAST_MESSAGE_LQI,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
  {
    ZCL_CLUSTER_ID_HA_DIAGNOSTIC,
    {  // Attribute record
      ATTRID_DIAGNOSTIC_LAST_MESSAGE_RSSI,
      ZCL_DATATYPE_INT8,
      ACCESS_CONTROL_READ,
      NULL // Use application's callback to Read this attribute
    }
  },
#endif // ZCL_DIAGNOSTIC
};

uint8 CONST zclSampleLight_NumAttributes = ( sizeof(zclSampleLight_Attrs) / sizeof(zclSampleLight_Attrs[0]) );

/*********************************************************************
 * SIMPLE DESCRIPTOR
 */
// This is the Cluster ID List and should be filled with Application
// specific cluster IDs.
const cId_t zclSampleLight_InClusterList[] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY,
  ZCL_CLUSTER_ID_GEN_GROUPS,
  ZCL_CLUSTER_ID_GEN_SCENES,
  ZCL_CLUSTER_ID_GEN_ON_OFF
#ifdef ZCL_LEVEL_CTRL
  , ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL
#endif
};
// work-around for compiler bug... IAR can't calculate size of array with #if options.
#ifdef ZCL_LEVEL_CTRL
 #define ZCLSAMPLELIGHT_MAX_INCLUSTERS   6
#else
 #define ZCLSAMPLELIGHT_MAX_INCLUSTERS   5
#endif

const cId_t zclSampleLight_OutClusterList[] =
{
  ZCL_CLUSTER_ID_GEN_BASIC
};
#define ZCLSAMPLELIGHT_MAX_OUTCLUSTERS  (sizeof(zclSampleLight_OutClusterList) / sizeof(zclSampleLight_OutClusterList[0]))

SimpleDescriptionFormat_t zclSampleLight_SimpleDesc =
{
  SAMPLELIGHT_ENDPOINT,                  //  int Endpoint;
  ZCL_HA_PROFILE_ID,                     //  uint16 AppProfId;
#ifdef ZCL_LEVEL_CTRL
  ZCL_HA_DEVICEID_DIMMABLE_LIGHT,        //  uint16 AppDeviceId;
#else
  ZCL_HA_DEVICEID_ON_OFF_LIGHT,          //  uint16 AppDeviceId;
#endif
  SAMPLELIGHT_DEVICE_VERSION,            //  int   AppDevVer:4;
  SAMPLELIGHT_FLAGS,                     //  int   AppFlags:4;
  ZCLSAMPLELIGHT_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
  (cId_t *)zclSampleLight_InClusterList, //  byte *pAppInClusterList;
  ZCLSAMPLELIGHT_MAX_OUTCLUSTERS,        //  byte  AppNumInClusters;
  (cId_t *)zclSampleLight_OutClusterList //  byte *pAppInClusterList;
};

// Red LED Simple Descriptor
SimpleDescriptionFormat_t zclSampleLight_RedSimpleDesc =
{
  SAMPLELIGHT_RED_ENDPOINT,              //  int Endpoint;
  ZCL_HA_PROFILE_ID,                     //  uint16 AppProfId;
  ZCL_HA_DEVICEID_ON_OFF_LIGHT,          //  uint16 AppDeviceId;
  SAMPLELIGHT_DEVICE_VERSION,            //  int   AppDevVer:4;
  SAMPLELIGHT_FLAGS,                     //  int   AppFlags:4;
  ZCLSAMPLELIGHT_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
  (cId_t *)zclSampleLight_InClusterList, //  byte *pAppInClusterList;
  ZCLSAMPLELIGHT_MAX_OUTCLUSTERS,        //  byte  AppNumInClusters;
  (cId_t *)zclSampleLight_OutClusterList //  byte *pAppInClusterList;
};

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/****************************************************************************
****************************************************************************/

// Red LED Simple Descriptor declaration
extern SimpleDescriptionFormat_t zclSampleLight_RedSimpleDesc;

// Switch1 Simple Descriptor (P1_2)
SimpleDescriptionFormat_t zclSampleLight_Switch1SimpleDesc =
{
  SAMPLELIGHT_SWITCH1_ENDPOINT,         //  int Endpoint;
  ZCL_HA_PROFILE_ID,                    //  uint16 AppProfId;
  ZCL_HA_DEVICEID_ON_OFF_OUTPUT,        //  uint16 AppDeviceId; - 使用输出设备类型
  SAMPLELIGHT_DEVICE_VERSION,           //  int   AppDevVer:4;
  SAMPLELIGHT_FLAGS,                    //  int   AppFlags:4;
  ZCLSAMPLELIGHT_MAX_INCLUSTERS,        //  byte  AppNumInClusters;
  (cId_t *)zclSampleLight_InClusterList, //  byte *pAppInClusterList;
  ZCLSAMPLELIGHT_MAX_OUTCLUSTERS,       //  byte  AppNumInClusters;
  (cId_t *)zclSampleLight_OutClusterList //  byte *pAppInClusterList;
};

// Switch2 Simple Descriptor (P1_3)
SimpleDescriptionFormat_t zclSampleLight_Switch2SimpleDesc =
{
  SAMPLELIGHT_SWITCH2_ENDPOINT,         //  int Endpoint;
  ZCL_HA_PROFILE_ID,                    //  uint16 AppProfId;
  ZCL_HA_DEVICEID_ON_OFF_OUTPUT,        //  uint16 AppDeviceId; - 使用输出设备类型
  SAMPLELIGHT_DEVICE_VERSION,           //  int   AppDevVer:4;
  SAMPLELIGHT_FLAGS,                    //  int   AppFlags:4;
  ZCLSAMPLELIGHT_MAX_INCLUSTERS,        //  byte  AppNumInClusters;
  (cId_t *)zclSampleLight_InClusterList, //  byte *pAppInClusterList;
  ZCLSAMPLELIGHT_MAX_OUTCLUSTERS,       //  byte  AppNumInClusters;
  (cId_t *)zclSampleLight_OutClusterList //  byte *pAppInClusterList;
};


