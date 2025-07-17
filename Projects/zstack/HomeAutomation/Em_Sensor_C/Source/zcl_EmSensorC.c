/*********************************************************************
  This application implements a ZigBee HA 1.2 Light. It can be configured as an
  On/Off light, or as a dimmable light. The following flags must be defined in
  the compiler's pre-defined symbols.

  ZCL_ON_OFF
  ZCL_LEVEL_CTRL    (only if dimming functionality desired)
  HOLD_AUTO_START
  ZCL_EZMODE

  This device supports all mandatory and optional commands/attributes for the
  OnOff (0x0006) and LevelControl (0x0008) clusters.

  SCREEN MODES
  ----------------------------------------
  Main:
    - SW1: Toggle local light
    - SW2: Invoke EZMode
    - SW4: Enable/Disable local permit join
    - SW5: Go to Help screen
  ----------------------------------------
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "MT_SYS.h"

#include "nwk_util.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ezmode.h"
#include "zcl_diagnostic.h"

#include "zcl_EmSensorC.h"

#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"

#if ( defined (ZGP_DEVICE_TARGET) || defined (ZGP_DEVICE_TARGETPLUS) \
      || defined (ZGP_DEVICE_COMBO) || defined (ZGP_DEVICE_COMBO_MIN) )
#include "zgp_translationtable.h"
  #if (SUPPORTED_S_FEATURE(SUPP_ZGP_FEATURE_TRANSLATION_TABLE))
    #define ZGP_AUTO_TT
  #endif
#endif

#if (defined HAL_BOARD_ZLIGHT) || (defined HAL_PWM)
#include "math.h"
#include "hal_timer.h"
#endif

#include "NLMEDE.h"
#include "DebugTrace.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#if (defined HAL_BOARD_ZLIGHT)
#define LEVEL_MAX                 0xFE
#define LEVEL_MIN                 0x0
#define GAMMA_VALUE               2
#define PWM_FULL_DUTY_CYCLE       1000
#elif (defined HAL_PWM)
#define LEVEL_MAX                 0xFE
#define LEVEL_MIN                 0x0
#define GAMMA_VALUE               2
#define PWM_FULL_DUTY_CYCLE       100
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclSampleLight_TaskID;
uint8 zclSampleLight_SeqNum;


/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
afAddrType_t zclSampleLight_DstAddr;

#ifdef ZCL_EZMODE
static void zclSampleLight_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg );
static void zclSampleLight_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData );


// register EZ-Mode with task information (timeout events, callback, etc...)
static const zclEZMode_RegisterData_t zclSampleLight_RegisterEZModeData =
{
  &zclSampleLight_TaskID,
  SAMPLELIGHT_EZMODE_NEXTSTATE_EVT,
  SAMPLELIGHT_EZMODE_TIMEOUT_EVT,
  &zclSampleLight_SeqNum,
  zclSampleLight_EZModeCB
};

#else
uint16 bindingInClusters[] =
{
  ZCL_CLUSTER_ID_GEN_ON_OFF
#ifdef ZCL_LEVEL_CTRL
  , ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL
#endif
};
#define ZCLSAMPLELIGHT_BINDINGLIST (sizeof(bindingInClusters) / sizeof(bindingInClusters[0]))

#endif  // ZCL_EZMODE

// Test Endpoint to allow SYS_APP_MSGs
static endPointDesc_t sampleLight_TestEp =
{
  SAMPLELIGHT_ENDPOINT,
  &zclSampleLight_TaskID,
  (SimpleDescriptionFormat_t *)NULL,  // No Simple description for this test endpoint
  (afNetworkLatencyReq_t)0            // No Network Latency req
};

// Red LED Endpoint
static endPointDesc_t sampleLight_RedEp =
{
  SAMPLELIGHT_RED_ENDPOINT,
  &zclSampleLight_TaskID,
  &zclSampleLight_RedSimpleDesc,
  (afNetworkLatencyReq_t)0            // No Network Latency req
};

// Switch1 Endpoint (P1_2)
static endPointDesc_t sampleLight_Switch1Ep =
{
  SAMPLELIGHT_SWITCH1_ENDPOINT,
  &zclSampleLight_TaskID,
  &zclSampleLight_Switch1SimpleDesc,
  (afNetworkLatencyReq_t)0            // No Network Latency req
};

// Switch2 Endpoint (P1_3)
static endPointDesc_t sampleLight_Switch2Ep =
{
  SAMPLELIGHT_SWITCH2_ENDPOINT,
  &zclSampleLight_TaskID,
  &zclSampleLight_Switch2SimpleDesc,
  (afNetworkLatencyReq_t)0            // No Network Latency req
};

uint8 giLightScreenMode = LIGHT_MAINMODE;   // display the main screen mode first

uint8 gPermitDuration = 0;    // permit joining default to disabled

devStates_t zclSampleLight_NwkState = DEV_INIT;

#if ZCL_LEVEL_CTRL
uint8 zclSampleLight_WithOnOff;       // set to TRUE if state machine should set light on/off
uint8 zclSampleLight_NewLevel;        // new level when done moving
bool  zclSampleLight_NewLevelUp;      // is direction to new level up or down?
int32 zclSampleLight_CurrentLevel32;  // current level, fixed point (e.g. 192.456)
int32 zclSampleLight_Rate32;          // rate in units, fixed point (e.g. 16.123)
uint8 zclSampleLight_LevelLastLevel;  // to save the Current Level before the light was turned OFF
#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclSampleLight_HandleKeys( byte shift, byte keys );
static void zclSampleLight_BasicResetCB( void );
static void zclSampleLight_IdentifyCB( zclIdentify_t *pCmd );
static void zclSampleLight_IdentifyQueryRspCB( zclIdentifyQueryRsp_t *pRsp );
static void zclSampleLight_OnOffCB( uint8 cmd );
static void zclSampleLight_ProcessIdentifyTimeChange( void );
static void zclSampleLight_SendSwitchReport(uint8 endpoint, uint8 state);
#ifdef ZCL_LEVEL_CTRL
static void zclSampleLight_LevelControlMoveToLevelCB( zclLCMoveToLevel_t *pCmd );
static void zclSampleLight_LevelControlMoveCB( zclLCMove_t *pCmd );
static void zclSampleLight_LevelControlStepCB( zclLCStep_t *pCmd );
static void zclSampleLight_LevelControlStopCB( void );
static void zclSampleLight_DefaultMove( void );
static uint32 zclSampleLight_TimeRateHelper( uint8 newLevel );
static uint16 zclSampleLight_GetTime ( uint8 level, uint16 time );
static void zclSampleLight_MoveBasedOnRate( uint8 newLevel, uint32 rate );
static void zclSampleLight_MoveBasedOnTime( uint8 newLevel, uint16 time );
static void zclSampleLight_AdjustLightLevel( void );
#endif

// app display functions
static void zclSampleLight_LcdDisplayUpdate( void );
#ifdef LCD_SUPPORTED
static void zclSampleLight_LcdDisplayMainMode( void );
static void zclSampleLight_LcdDisplayHelpMode( void );
#endif
static void zclSampleLight_DisplayLight( void );

#if (defined HAL_BOARD_ZLIGHT) || (defined HAL_PWM)
void zclSampleLight_UpdateLampLevel( uint8 level );
#endif

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclSampleLight_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclSampleLight_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclSampleLight_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclSampleLight_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 zclSampleLight_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleLight_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleLight_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg );
#endif

/*********************************************************************
 * STATUS STRINGS
 */
#ifdef LCD_SUPPORTED
const char sDeviceName[]   = "  Sample Light";
const char sClearLine[]    = " ";
const char sSwLight[]      = "SW1: ToggleLight";  // 16 chars max
const char sSwEZMode[]     = "SW2: EZ-Mode";
char sSwHelp[]             = "SW5: Help       ";  // last character is * if NWK open
const char sLightOn[]      = "    LIGHT ON ";
const char sLightOff[]     = "    LIGHT OFF";
 #if ZCL_LEVEL_CTRL
 char sLightLevel[]        = "    LEVEL ###"; // displays level 1-254
 #endif
#endif

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclSampleLight_CmdCallbacks =
{
  zclSampleLight_BasicResetCB,            // Basic Cluster Reset command
  zclSampleLight_IdentifyCB,              // Identify command
#ifdef ZCL_EZMODE
  NULL,                                   // Identify EZ-Mode Invoke command
  NULL,                                   // Identify Update Commission State command
#endif
  NULL,                                   // Identify Trigger Effect command
  zclSampleLight_IdentifyQueryRspCB,      // Identify Query Response command
  zclSampleLight_OnOffCB,                 // On/Off cluster commands
  NULL,                                   // On/Off cluster enhanced command Off with Effect
  NULL,                                   // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                   // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  zclSampleLight_LevelControlMoveToLevelCB, // Level Control Move to Level command
  zclSampleLight_LevelControlMoveCB,        // Level Control Move command
  zclSampleLight_LevelControlStepCB,        // Level Control Step command
  zclSampleLight_LevelControlStopCB,        // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                   // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                  // Scene Store Request command
  NULL,                                  // Scene Recall Request command
  NULL,                                  // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                  // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                  // Get Event Log command
  NULL,                                  // Publish Event Log command
#endif
  NULL,                                  // RSSI Location command
  NULL                                   // RSSI Location Response command
};

/*********************************************************************
 * @fn          zclSampleLight_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclSampleLight_Init( byte task_id )
{
  // 调试信息：初始化开始
  debug_str("Em_Sensor_C Init Start");
  
  zclSampleLight_TaskID = task_id;

  // Set destination address to indirect
  zclSampleLight_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  zclSampleLight_DstAddr.endPoint = 0;
  zclSampleLight_DstAddr.addr.shortAddr = 0;

  // This app is part of the Home Automation Profile
  zclHA_Init( &zclSampleLight_SimpleDesc );
  zclHA_Init( &zclSampleLight_RedSimpleDesc );
  zclHA_Init( &zclSampleLight_Switch1SimpleDesc );  // P1_2开关
  zclHA_Init( &zclSampleLight_Switch2SimpleDesc );  // P1_3开关

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SAMPLELIGHT_ENDPOINT, &zclSampleLight_CmdCallbacks );
  zclGeneral_RegisterCmdCallbacks( SAMPLELIGHT_RED_ENDPOINT, &zclSampleLight_CmdCallbacks );
  zclGeneral_RegisterCmdCallbacks( SAMPLELIGHT_SWITCH1_ENDPOINT, &zclSampleLight_CmdCallbacks );  // P1_2开关
  zclGeneral_RegisterCmdCallbacks( SAMPLELIGHT_SWITCH2_ENDPOINT, &zclSampleLight_CmdCallbacks );  // P1_3开关

  // Register the application's attribute list
  zcl_registerAttrList( SAMPLELIGHT_ENDPOINT, zclSampleLight_NumAttributes, zclSampleLight_Attrs );
  zcl_registerAttrList( SAMPLELIGHT_RED_ENDPOINT, zclSampleLight_NumAttributes, zclSampleLight_Attrs );
  zcl_registerAttrList( SAMPLELIGHT_SWITCH1_ENDPOINT, zclSampleLight_NumAttributes, zclSampleLight_Attrs );  // P1_2开关
  zcl_registerAttrList( SAMPLELIGHT_SWITCH2_ENDPOINT, zclSampleLight_NumAttributes, zclSampleLight_Attrs );  // P1_3开关

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclSampleLight_TaskID );

#ifdef ZCL_DISCOVER
  // Register the application's command list
  zcl_registerCmdList( SAMPLELIGHT_ENDPOINT, zclCmdsArraySize, zclSampleLight_Cmds );
#endif

  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclSampleLight_TaskID );

  // Register for a test endpoint
  afRegister( &sampleLight_TestEp );
  
  // Register for red LED endpoint
  afRegister( &sampleLight_RedEp );
  
  // Register for switch endpoints
  afRegister( &sampleLight_Switch1Ep );  // P1_2开关
  afRegister( &sampleLight_Switch2Ep );  // P1_3开关

#ifdef ZCL_EZMODE
  // Register EZ-Mode
  zcl_RegisterEZMode( &zclSampleLight_RegisterEZModeData );

  // Register with the ZDO to receive Match Descriptor Responses
  ZDO_RegisterForZDOMsg(task_id, Match_Desc_rsp);
#endif


#if (defined HAL_BOARD_ZLIGHT) || (defined HAL_PWM)
  HalTimer1Init( 0 );
  halTimer1SetChannelDuty( WHITE_LED, 0 );
  halTimer1SetChannelDuty( RED_LED, 0 );
  halTimer1SetChannelDuty( BLUE_LED, 0 );
  halTimer1SetChannelDuty( GREEN_LED, 0 );

  // find if we are already on a network from NV_RESTORE
  uint8 state;
  NLME_GetRequest( nwkNwkState, 0, &state );

  if ( state < NWK_ENDDEVICE )
  {
    // Start EZMode on Start up to avoid button press
    osal_start_timerEx( zclSampleLight_TaskID, SAMPLELIGHT_START_EZMODE_EVT, 500 );
  }
#if ZCL_LEVEL_CTRL
  zclSampleLight_DefaultMove();
#endif
#endif // #if (defined HAL_BOARD_ZLIGHT) || (defined HAL_PWM)

#ifdef ZCL_DIAGNOSTIC
  // Register the application's callback function to read/write attribute data.
  // This is only required when the attribute data format is unknown to ZCL.
  zcl_registerReadWriteCB( SAMPLELIGHT_ENDPOINT, zclDiagnostic_ReadWriteAttrCB, NULL );

  if ( zclDiagnostic_InitStats() == ZSuccess )
  {
    // Here the user could start the timer to save Diagnostics to NV
  }
#endif

#ifdef LCD_SUPPORTED
  HalLcdWriteString ( (char *)sDeviceName, HAL_LCD_LINE_3 );
#endif  // LCD_SUPPORTED

#ifdef ZGP_AUTO_TT
  zgpTranslationTable_RegisterEP ( &zclSampleLight_SimpleDesc );
  zgpTranslationTable_RegisterEP ( &zclSampleLight_RedSimpleDesc );
  zgpTranslationTable_RegisterEP ( &zclSampleLight_Switch1SimpleDesc );  // P1_2开关
  zgpTranslationTable_RegisterEP ( &zclSampleLight_Switch2SimpleDesc );  // P1_3开关
#endif

  // 调试信息：初始化完成
  debug_str("Em_Sensor_C Init Done");
  
  // 启动S1按键状态检测定时器 - 已注释，保留代码方便调试
  // osal_start_timerEx( zclSampleLight_TaskID, SAMPLELIGHT_S1_CHECK_EVT, 1000 );
}

/*********************************************************************
 * @fn          zclSample_event_loop
 *
 * @brief       Event Loop Processor for zclGeneral.
 *
 * @param       none
 *
 * @return      none
 */
uint16 zclSampleLight_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclSampleLight_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
#ifdef ZCL_EZMODE
        case ZDO_CB_MSG:
          // 调试信息：收到ZDO回调消息
          debug_str("ZDO CB Msg");
          zclSampleLight_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;
#endif
        case ZCL_INCOMING_MSG:
          // 调试信息：收到ZCL消息
          debug_str("ZCL Msg");
          // Incoming ZCL Foundation command/response messages
          zclSampleLight_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          // 调试信息：收到按键事件
          debug_str("Key Event");
          zclSampleLight_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case ZDO_STATE_CHANGE:
          // 调试信息：网络状态变化
          debug_str("NWK State");
          zclSampleLight_NwkState = (devStates_t)(MSGpkt->hdr.status);

          // 调试信息：显示具体的网络状态
          switch(zclSampleLight_NwkState)
          {
            case DEV_INIT:
              debug_str("State: INIT");
              break;
            case DEV_NWK_DISC:
              debug_str("State: DISC");
              break;
            case DEV_NWK_JOINING:
              debug_str("State: JOIN");
              break;
            case DEV_END_DEVICE:
              debug_str("State: END");
              break;
            case DEV_ROUTER:
              debug_str("State: ROUTER");
              break;
            case DEV_ZB_COORD:
              debug_str("State: COORD");
              break;
            case DEV_NWK_ORPHAN:
              debug_str("State: ORPHAN");
              break;
            case DEV_END_DEVICE_UNAUTH:
              debug_str("State: UNAUTH");
              break;
            default:
              debug_str("State: UNKNOWN");
              break;
          }

          // now on the network
          if ( (zclSampleLight_NwkState == DEV_ZB_COORD) ||
               (zclSampleLight_NwkState == DEV_ROUTER)   ||
               (zclSampleLight_NwkState == DEV_END_DEVICE) )
          {
            // 调试信息：成功加入网络
            debug_str("Network OK");
            
            giLightScreenMode = LIGHT_MAINMODE;
            zclSampleLight_LcdDisplayUpdate();
#ifdef ZCL_EZMODE
            zcl_EZModeAction( EZMODE_ACTION_NETWORK_STARTED, NULL );
#endif // ZCL_EZMODE
          }
          else if (zclSampleLight_NwkState == DEV_NWK_JOINING)
          {
            // 调试信息：正在加入网络
            debug_str("Joining...");
          }
          else if (zclSampleLight_NwkState == DEV_NWK_DISC)
          {
            // 调试信息：正在发现网络
            debug_str("Discovering");
          }
          else if (zclSampleLight_NwkState == DEV_END_DEVICE_UNAUTH)
          {
            // 调试信息：未授权状态
            HalLcdWriteString("Unauthorized", HAL_LCD_LINE_3);
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SAMPLELIGHT_IDENTIFY_TIMEOUT_EVT )
  {
    // 调试信息：识别超时事件
    debug_str("ID Timeout");
    if ( zclSampleLight_IdentifyTime > 0 )
      zclSampleLight_IdentifyTime--;
    zclSampleLight_ProcessIdentifyTimeChange();

    return ( events ^ SAMPLELIGHT_IDENTIFY_TIMEOUT_EVT );
  }

  if ( events & SAMPLELIGHT_MAIN_SCREEN_EVT )
  {
    // 调试信息：主屏幕事件
    debug_str("Main Screen");
    giLightScreenMode = LIGHT_MAINMODE;
    zclSampleLight_LcdDisplayUpdate();

    return ( events ^ SAMPLELIGHT_MAIN_SCREEN_EVT );
  }

#ifdef ZCL_EZMODE
#if (defined HAL_BOARD_ZLIGHT)
  // event to start EZMode on startup with a delay
  if ( events & SAMPLELIGHT_START_EZMODE_EVT )
  {
    // 调试信息：启动EZ-Mode事件
    debug_str("Start EZMode");
    // Invoke EZ-Mode
    zclEZMode_InvokeData_t ezModeData;

    // Invoke EZ-Mode
    ezModeData.endpoint = SAMPLELIGHT_ENDPOINT; // endpoint on which to invoke EZ-Mode
    if ( (zclSampleLight_NwkState == DEV_ZB_COORD) ||
         (zclSampleLight_NwkState == DEV_ROUTER)   ||
         (zclSampleLight_NwkState == DEV_END_DEVICE) )
    {
      ezModeData.onNetwork = TRUE;      // node is already on the network
    }
    else
    {
      ezModeData.onNetwork = FALSE;     // node is not yet on the network
    }
    ezModeData.initiator = FALSE;          // OnOffLight is a target
    ezModeData.numActiveOutClusters = 0;
    ezModeData.pActiveOutClusterIDs = NULL;
    ezModeData.numActiveInClusters = 0;
    ezModeData.pActiveOutClusterIDs = NULL;
    zcl_InvokeEZMode( &ezModeData );

    return ( events ^ SAMPLELIGHT_START_EZMODE_EVT );
  }
#endif // #if (defined HAL_BOARD_ZLIGHT)

  // going on to next state
  if ( events & SAMPLELIGHT_EZMODE_NEXTSTATE_EVT )
  {
    // 调试信息：EZ-Mode下一状态事件
    debug_str("EZ Next");
    zcl_EZModeAction ( EZMODE_ACTION_PROCESS, NULL );   // going on to next state
    return ( events ^ SAMPLELIGHT_EZMODE_NEXTSTATE_EVT );
  }

  // the overall EZMode timer expired, so we timed out
  if ( events & SAMPLELIGHT_EZMODE_TIMEOUT_EVT )
  {
    // 调试信息：EZ-Mode超时事件
    debug_str("EZ Timeout");
    zcl_EZModeAction ( EZMODE_ACTION_TIMED_OUT, NULL ); // EZ-Mode timed out
    return ( events ^ SAMPLELIGHT_EZMODE_TIMEOUT_EVT );
  }
#endif // ZLC_EZMODE

#ifdef ZCL_LEVEL_CTRL
  if ( events & SAMPLELIGHT_LEVEL_CTRL_EVT )
  {
    // 调试信息：调光控制事件
    debug_str("Level Ctrl");
    zclSampleLight_AdjustLightLevel();
    return ( events ^ SAMPLELIGHT_LEVEL_CTRL_EVT );
  }
#endif

  // 定期检测S1按键状态 - 已注释，保留代码方便调试
  /*
  if ( events & SAMPLELIGHT_S1_CHECK_EVT )
  {
    // 调试信息：S1按键状态检测
    // S1是低电平有效，按下时返回0，释放时返回1
    if (HAL_PUSH_BUTTON1()) {
      debug_str("S1: Released (periodic)");
    } else {
      debug_str("S1: Pressed (periodic)");
    }
    // 重新启动定时器，每1秒检测一次
    osal_start_timerEx( zclSampleLight_TaskID, SAMPLELIGHT_S1_CHECK_EVT, 1000 );
    return ( events ^ SAMPLELIGHT_S1_CHECK_EVT );
  }
  */

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      zclSampleLight_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_5
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void zclSampleLight_HandleKeys( byte shift, byte keys )
{
  // 检测S2按键状态变化
  static uint8 lastS2State = 0;
  uint8 currentS2State = (keys & HAL_KEY_SW_7) ? 1 : 0;
  
  if (currentS2State != lastS2State) {
    if (currentS2State) {
      debug_str("S2: Pressed!\n");
      // 发送S2按键按下报告
      zclSampleLight_SendSwitchReport(SAMPLELIGHT_SWITCH1_ENDPOINT, LIGHT_ON);
    } else {
      debug_str("S2: Released!\n");
      // 发送S2按键松开报告
      zclSampleLight_SendSwitchReport(SAMPLELIGHT_SWITCH1_ENDPOINT, LIGHT_OFF);
    }
    lastS2State = currentS2State;
  }
  
  // 检测S3按键状态变化
  static uint8 lastS3State = 0;
  uint8 currentS3State = (keys & HAL_KEY_SW_8) ? 1 : 0;
  
  if (currentS3State != lastS3State) {
    if (currentS3State) {
      debug_str("S3: Pressed!\n");
      // 发送S3按键按下报告
      zclSampleLight_SendSwitchReport(SAMPLELIGHT_SWITCH2_ENDPOINT, LIGHT_ON);
    } else {
      debug_str("S3: Released!\n");
      // 发送S3按键松开报告
      zclSampleLight_SendSwitchReport(SAMPLELIGHT_SWITCH2_ENDPOINT, LIGHT_OFF);
    }
    lastS3State = currentS3State;
  }
  
  if ( keys & HAL_KEY_SW_1 )
  {
    // 调试信息：SW1按键 - 切换灯光
    debug_str("SW1: Toggle");
    
    giLightScreenMode = LIGHT_MAINMODE;

    // toggle local light immediately
    zclSampleLight_OnOff = zclSampleLight_OnOff ? LIGHT_OFF : LIGHT_ON;
#ifdef ZCL_LEVEL_CTRL
    zclSampleLight_LevelCurrentLevel = zclSampleLight_OnOff ? zclSampleLight_LevelOnLevel : ATTR_LEVEL_MIN_LEVEL;
#endif
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    // 调试信息：SW2按键 - 绑定请求
    debug_str("SW2: Bind");
    
#if (defined HAL_BOARD_ZLIGHT)

    zclSampleLight_BasicResetCB();

#else

    giLightScreenMode = LIGHT_MAINMODE;

#ifdef ZCL_EZMODE
    {
      // Invoke EZ-Mode
      zclEZMode_InvokeData_t ezModeData;

      // Invoke EZ-Mode
      ezModeData.endpoint = SAMPLELIGHT_ENDPOINT; // endpoint on which to invoke EZ-Mode
      if ( (zclSampleLight_NwkState == DEV_ZB_COORD) ||
          (zclSampleLight_NwkState == DEV_ROUTER)   ||
            (zclSampleLight_NwkState == DEV_END_DEVICE) )
      {
        ezModeData.onNetwork = TRUE;      // node is already on the network
      }
      else
      {
        ezModeData.onNetwork = FALSE;     // node is not yet on the network
      }
      ezModeData.initiator = FALSE;          // OnOffLight is a target
      ezModeData.numActiveOutClusters = 0;
      ezModeData.pActiveOutClusterIDs = NULL;
      ezModeData.numActiveInClusters = 0;
      ezModeData.pActiveOutClusterIDs = NULL;
      zcl_InvokeEZMode( &ezModeData );
    }

#else // NOT EZ-Mode
    {
      zAddrType_t dstAddr;
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

      // Initiate an End Device Bind Request, this bind request will
      // only use a cluster list that is important to binding.
      dstAddr.addrMode = afAddr16Bit;
      dstAddr.addr.shortAddr = 0;   // Coordinator makes the match
      ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                           SAMPLELIGHT_ENDPOINT,
                           ZCL_HA_PROFILE_ID,
                           ZCLSAMPLELIGHT_BINDINGLIST, bindingInClusters,
                           0, NULL,   // No Outgoing clusters to bind
                           TRUE );
    }
#endif // ZCL_EZMODE
#endif // HAL_BOARD_ZLIGHT
  }

  if ( keys & HAL_KEY_SW_3 )
  {
    // 调试信息：SW3按键 - 网络状态
    debug_str("SW3: NWK Status");
    
    NLME_SendNetworkStatus( zclSampleLight_DstAddr.addr.shortAddr,
                       NLME_GetShortAddr(), NWKSTAT_NONTREE_LINK_FAILURE, FALSE );
  }

  if ( keys & HAL_KEY_SW_4 )
  {
    // 调试信息：SW4按键 - 允许加入
    debug_str("SW4: Permit Join");
    
    giLightScreenMode = LIGHT_MAINMODE;

    if ( ( zclSampleLight_NwkState == DEV_ZB_COORD ) ||
          ( zclSampleLight_NwkState == DEV_ROUTER ) )
    {
      zAddrType_t tmpAddr;

      tmpAddr.addrMode = Addr16Bit;
      tmpAddr.addr.shortAddr = NLME_GetShortAddr();

      // toggle permit join
      gPermitDuration = gPermitDuration ? 0 : 0xff;

      // Trust Center significance is always true
      ZDP_MgmtPermitJoinReq( &tmpAddr, gPermitDuration, TRUE, FALSE );
    }
  }

  // Shift F5 does a Basic Reset (factory defaults)
  if ( shift && ( keys & HAL_KEY_SW_5 ) )
  {
    // 调试信息：Shift+SW5按键 - 基本重置
    debug_str("Shift+SW5: Reset");
    
    zclSampleLight_BasicResetCB();
  }
  else if ( keys & HAL_KEY_SW_5 )
  {
    // 调试信息：SW5按键 - 帮助屏幕
    debug_str("SW5: Help");
    
    giLightScreenMode = giLightScreenMode ? LIGHT_MAINMODE : LIGHT_HELPMODE;
  }

  // update the display, including the light
  zclSampleLight_LcdDisplayUpdate();
}

/*********************************************************************
 * @fn      zclSampleLight_LcdDisplayUpdate
 *
 * @brief   Called to update the LCD display.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleLight_LcdDisplayUpdate( void )
{
#ifdef LCD_SUPPORTED
  if ( giLightScreenMode == LIGHT_HELPMODE )
  {
    zclSampleLight_LcdDisplayHelpMode();
  }
  else
  {
    zclSampleLight_LcdDisplayMainMode();
  }
#endif

  zclSampleLight_DisplayLight();
}

#if (defined HAL_BOARD_ZLIGHT) || (defined HAL_PWM)
/*********************************************************************
 * @fn      zclSampleLight_UpdateLampLevel
 *
 * @brief   Update lamp level output with gamma compensation
 *
 * @param   level
 *
 * @return  none
 */
void zclSampleLight_UpdateLampLevel( uint8 level )

{
  uint16 gammaCorrectedLevel;

  // gamma correct the level
  gammaCorrectedLevel = (uint16) ( pow( ( (float)level / LEVEL_MAX ), (float)GAMMA_VALUE ) * (float)LEVEL_MAX);

  halTimer1SetChannelDuty(WHITE_LED, (uint16)(((uint32)gammaCorrectedLevel*PWM_FULL_DUTY_CYCLE)/LEVEL_MAX) );
}
#endif

/*********************************************************************
 * @fn      zclSampleLight_DisplayLight
 *
 * @brief   Displays current state of light on LED and also on main display if supported.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleLight_DisplayLight( void )
{
  // set the LED1 based on light (on or off)
  if ( zclSampleLight_OnOff == LIGHT_ON )
  {
    HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
  }
  else
  {
    HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
  }
  
  // set the LED2 (red) based on red light (on or off)
  if ( zclSampleLight_RedOnOff == LIGHT_ON )
  {
    HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
  }
  else
  {
    HalLedSet ( HAL_LED_2, HAL_LED_MODE_OFF );
  }

#ifdef LCD_SUPPORTED
  if (giLightScreenMode == LIGHT_MAINMODE)
  {
#ifdef ZCL_LEVEL_CTRL
    // display current light level
    if ( ( zclSampleLight_LevelCurrentLevel == ATTR_LEVEL_MIN_LEVEL ) &&
         ( zclSampleLight_OnOff == LIGHT_OFF ) )
    {
      HalLcdWriteString( (char *)sLightOff, HAL_LCD_LINE_2 );
    }
    else if ( ( zclSampleLight_LevelCurrentLevel >= ATTR_LEVEL_MAX_LEVEL ) ||
              ( zclSampleLight_LevelCurrentLevel == zclSampleLight_LevelOnLevel ) ||
               ( ( zclSampleLight_LevelOnLevel == ATTR_LEVEL_ON_LEVEL_NO_EFFECT ) &&
                 ( zclSampleLight_LevelCurrentLevel == zclSampleLight_LevelLastLevel ) ) )
    {
      HalLcdWriteString( (char *)sLightOn, HAL_LCD_LINE_2 );
    }
    else    // "    LEVEL ###"
    {
      zclHA_uint8toa( zclSampleLight_LevelCurrentLevel, &sLightLevel[10] );
      HalLcdWriteString( (char *)sLightLevel, HAL_LCD_LINE_2 );
    }
#else
    if ( zclSampleLight_OnOff )
    {
      HalLcdWriteString( (char *)sLightOn, HAL_LCD_LINE_2 );
    }
    else
    {
      HalLcdWriteString( (char *)sLightOff, HAL_LCD_LINE_2 );
    }
#endif // ZCL_LEVEL_CTRL
  }
#endif // LCD_SUPPORTED
}

#ifdef LCD_SUPPORTED
/*********************************************************************
 * @fn      zclSampleLight_LcdDisplayMainMode
 *
 * @brief   Called to display the main screen on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleLight_LcdDisplayMainMode( void )
{
  // display line 1 to indicate NWK status
  if ( zclSampleLight_NwkState == DEV_ZB_COORD )
  {
    zclHA_LcdStatusLine1( ZCL_HA_STATUSLINE_ZC );
  }
  else if ( zclSampleLight_NwkState == DEV_ROUTER )
  {
    zclHA_LcdStatusLine1( ZCL_HA_STATUSLINE_ZR );
  }
  else if ( zclSampleLight_NwkState == DEV_END_DEVICE )
  {
    zclHA_LcdStatusLine1( ZCL_HA_STATUSLINE_ZED );
  }

  // end of line 3 displays permit join status (*)
  if ( gPermitDuration )
  {
    sSwHelp[15] = '*';
  }
  else
  {
    sSwHelp[15] = ' ';
  }
  HalLcdWriteString( (char *)sSwHelp, HAL_LCD_LINE_3 );
}

/*********************************************************************
 * @fn      zclSampleLight_LcdDisplayHelpMode
 *
 * @brief   Called to display the SW options on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleLight_LcdDisplayHelpMode( void )
{
  HalLcdWriteString( (char *)sSwLight, HAL_LCD_LINE_1 );
  HalLcdWriteString( (char *)sSwEZMode, HAL_LCD_LINE_2 );
  HalLcdWriteString( (char *)sSwHelp, HAL_LCD_LINE_3 );
}
#endif  // LCD_SUPPORTED

/*********************************************************************
 * @fn      zclSampleLight_ProcessIdentifyTimeChange
 *
 * @brief   Called to process any change to the IdentifyTime attribute.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleLight_ProcessIdentifyTimeChange( void )
{
  if ( zclSampleLight_IdentifyTime > 0 )
  {
    osal_start_timerEx( zclSampleLight_TaskID, SAMPLELIGHT_IDENTIFY_TIMEOUT_EVT, 1000 );
    HalLedBlink ( HAL_LED_4, 0xFF, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME );
  }
  else
  {
#ifdef ZCL_EZMODE
    if ( zclSampleLight_IdentifyCommissionState & EZMODE_COMMISSION_OPERATIONAL )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_ON );
    }
    else
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
    }
#endif

    osal_stop_timerEx( zclSampleLight_TaskID, SAMPLELIGHT_IDENTIFY_TIMEOUT_EVT );
  }
}

/*********************************************************************
 * @fn      zclSampleLight_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleLight_BasicResetCB( void )
{
  // 调试信息：基本重置回调
  debug_str("Basic Reset");
  
  NLME_LeaveReq_t leaveReq;
  // Set every field to 0
  osal_memset( &leaveReq, 0, sizeof( NLME_LeaveReq_t ) );

  // This will enable the device to rejoin the network after reset.
  leaveReq.rejoin = TRUE;

  // Set the NV startup option to force a "new" join.
  zgWriteStartupOptions( ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_NETWORK_STATE );

  // Leave the network, and reset afterwards
  if ( NLME_LeaveReq( &leaveReq ) != ZSuccess )
  {
    // Couldn't send out leave; prepare to reset anyway
    ZDApp_LeaveReset( FALSE );
  }
}

/*********************************************************************
 * @fn      zclSampleLight_IdentifyCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Command for this application.
 *
 * @param   srcAddr - source address and endpoint of the response message
 * @param   identifyTime - the number of seconds to identify yourself
 *
 * @return  none
 */
static void zclSampleLight_IdentifyCB( zclIdentify_t *pCmd )
{
  // 调试信息：识别命令回调
  debug_str("Identify Cmd");
  
  zclSampleLight_IdentifyTime = pCmd->identifyTime;
  zclSampleLight_ProcessIdentifyTimeChange();
}

/*********************************************************************
 * @fn      zclSampleLight_IdentifyQueryRspCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Query Response Command for this application.
 *
 * @param   srcAddr - requestor's address
 * @param   timeout - number of seconds to identify yourself (valid for query response)
 *
 * @return  none
 */
static void zclSampleLight_IdentifyQueryRspCB(  zclIdentifyQueryRsp_t *pRsp )
{
  // 调试信息：识别查询响应回调
  debug_str("ID Query Rsp");
  
  (void)pRsp;
#ifdef ZCL_EZMODE
  {
    zclEZMode_ActionData_t data;
    data.pIdentifyQueryRsp = pRsp;
    zcl_EZModeAction ( EZMODE_ACTION_IDENTIFY_QUERY_RSP, &data );
  }
#endif
}

/*********************************************************************
 * @fn      zclSampleLight_OnOffCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an On/Off Command for this application.
 *
 * @param   cmd - COMMAND_ON, COMMAND_OFF or COMMAND_TOGGLE
 *
 * @return  none
 */
static void zclSampleLight_OnOffCB( uint8 cmd )
{
  // 调试信息：开关命令回调
  debug_str("OnOff Cmd");
  
  afIncomingMSGPacket_t *pPtr = zcl_getRawAFMsg();

  zclSampleLight_DstAddr.addr.shortAddr = pPtr->srcAddr.addr.shortAddr;

  // 正确根据目标端点判断
  if ( pPtr->endPoint == SAMPLELIGHT_RED_ENDPOINT )
  {
    // Handle red LED commands
    if ( cmd == COMMAND_ON )
    {
      zclSampleLight_RedOnOff = LIGHT_ON;
    }
    else if ( cmd == COMMAND_OFF )
    {
      zclSampleLight_RedOnOff = LIGHT_OFF;
    }
    else if ( cmd == COMMAND_TOGGLE )
    {
      if ( zclSampleLight_RedOnOff == LIGHT_OFF )
      {
        zclSampleLight_RedOnOff = LIGHT_ON;
      }
      else
      {
        zclSampleLight_RedOnOff = LIGHT_OFF;
      }
    }
  }
  else if ( pPtr->endPoint == SAMPLELIGHT_ENDPOINT )
  {
    // Handle green LED commands (original endpoint)
    if ( cmd == COMMAND_ON )
    {
      zclSampleLight_OnOff = LIGHT_ON;
    }
    else if ( cmd == COMMAND_OFF )
    {
      zclSampleLight_OnOff = LIGHT_OFF;
    }
    else if ( cmd == COMMAND_TOGGLE )
    {
      if ( zclSampleLight_OnOff == LIGHT_OFF )
      {
        zclSampleLight_OnOff = LIGHT_ON;
      }
      else
      {
        zclSampleLight_OnOff = LIGHT_OFF;
      }
    }
#if ZCL_LEVEL_CTRL
    zclSampleLight_DefaultMove( );
#endif
  }
  // 注意：开关端点（10和11）不处理远程控制命令，只反映按键状态

  // update the display
  zclSampleLight_LcdDisplayUpdate( );
}

#ifdef ZCL_LEVEL_CTRL
/*********************************************************************
 * @fn      zclSampleLight_TimeRateHelper
 *
 * @brief   Calculate time based on rate, and startup level state machine
 *
 * @param   newLevel - new level for current level
 *
 * @return  diff (directly), zclSampleLight_CurrentLevel32 and zclSampleLight_NewLevel, zclSampleLight_NewLevelUp
 */
static uint32 zclSampleLight_TimeRateHelper( uint8 newLevel )
{
  uint32 diff;
  uint32 newLevel32;

  // remember current and new level
  zclSampleLight_NewLevel = newLevel;
  zclSampleLight_CurrentLevel32 = (uint32)1000 * zclSampleLight_LevelCurrentLevel;

  // calculate diff
  newLevel32 = (uint32)1000 * newLevel;
  if ( zclSampleLight_LevelCurrentLevel > newLevel )
  {
    diff = zclSampleLight_CurrentLevel32 - newLevel32;
    zclSampleLight_NewLevelUp = FALSE;  // moving down
  }
  else
  {
    diff = newLevel32 - zclSampleLight_CurrentLevel32;
    zclSampleLight_NewLevelUp = TRUE;   // moving up
  }

  return ( diff );
}

/*********************************************************************
 * @fn      zclSampleLight_MoveBasedOnRate
 *
 * @brief   Calculate time based on rate, and startup level state machine
 *
 * @param   newLevel - new level for current level
 * @param   rate16   - fixed point rate (e.g. 16.123)
 *
 * @return  none
 */
static void zclSampleLight_MoveBasedOnRate( uint8 newLevel, uint32 rate )
{
  uint32 diff;

  // determine how much time (in 10ths of seconds) based on the difference and rate
  zclSampleLight_Rate32 = rate;
  diff = zclSampleLight_TimeRateHelper( newLevel );
  zclSampleLight_LevelRemainingTime = diff / rate;
  if ( !zclSampleLight_LevelRemainingTime )
  {
    zclSampleLight_LevelRemainingTime = 1;
  }

  osal_start_timerEx( zclSampleLight_TaskID, SAMPLELIGHT_LEVEL_CTRL_EVT, 100 );
}

/*********************************************************************
 * @fn      zclSampleLight_MoveBasedOnTime
 *
 * @brief   Calculate rate based on time, and startup level state machine
 *
 * @param   newLevel  - new level for current level
 * @param   time      - in 10ths of seconds
 *
 * @return  none
 */
static void zclSampleLight_MoveBasedOnTime( uint8 newLevel, uint16 time )
{
  uint16 diff;

  // determine rate (in units) based on difference and time
  diff = zclSampleLight_TimeRateHelper( newLevel );
  zclSampleLight_LevelRemainingTime = zclSampleLight_GetTime( newLevel, time );
  zclSampleLight_Rate32 = diff / time;

  osal_start_timerEx( zclSampleLight_TaskID, SAMPLELIGHT_LEVEL_CTRL_EVT, 100 );
}

/*********************************************************************
 * @fn      zclSampleLight_GetTime
 *
 * @brief   Determine amount of time that MoveXXX will take to complete.
 *
 * @param   level = new level to move to
 *          time  = 0xffff=default, or 0x0000-n amount of time in tenths of seconds.
 *
 * @return  none
 */
static uint16 zclSampleLight_GetTime( uint8 level, uint16 time )
{
  // there is a hiearchy of the amount of time to use for transistioning
  // check each one in turn. If none of defaults are set, then use fastest
  // time possible.
  if ( time == 0xFFFF )
  {
    // use On or Off Transition Time if set (not 0xffff)
    if ( zclSampleLight_OnOff == LIGHT_ON )
    {
      time = zclSampleLight_LevelOffTransitionTime;
    }
    else
    {
      time = zclSampleLight_LevelOnTransitionTime;
    }

    // else use OnOffTransitionTime if set (not 0xffff)
    if ( time == 0xFFFF )
    {
      time = zclSampleLight_LevelOnOffTransitionTime;
    }

    // else as fast as possible
    if ( time == 0xFFFF )
    {
      time = 1;
    }
  }

  if ( !time )
  {
    time = 1; // as fast as possible
  }

  return ( time );
}

/*********************************************************************
 * @fn      zclSampleLight_DefaultMove
 *
 * @brief   We were turned on/off. Use default time to move to on or off.
 *
 * @param   zclSampleLight_OnOff - must be set prior to calling this function.
 *
 * @return  none
 */
static void zclSampleLight_DefaultMove( void )
{
  uint8  newLevel;
  uint32 rate;      // fixed point decimal (3 places, eg. 16.345)
  uint16 time;

  // if moving to on position, move to on level
  if ( zclSampleLight_OnOff )
  {
    if ( zclSampleLight_LevelOnLevel == ATTR_LEVEL_ON_LEVEL_NO_EFFECT )
    {
      // The last Level (before going OFF) should be used)
      newLevel = zclSampleLight_LevelLastLevel;
    }
    else
    {
      newLevel = zclSampleLight_LevelOnLevel;
    }

    time = zclSampleLight_LevelOnTransitionTime;
  }
  else
  {
    newLevel = ATTR_LEVEL_MIN_LEVEL;

    if ( zclSampleLight_LevelOnLevel == ATTR_LEVEL_ON_LEVEL_NO_EFFECT )
    {
      // Save the current Level before going OFF to use it when the light turns ON
      // it should be back to this level
      zclSampleLight_LevelLastLevel = zclSampleLight_LevelCurrentLevel;
    }

    time = zclSampleLight_LevelOffTransitionTime;
  }

  // else use OnOffTransitionTime if set (not 0xffff)
  if ( time == 0xFFFF )
  {
    time = zclSampleLight_LevelOnOffTransitionTime;
  }

  // else as fast as possible
  if ( time == 0xFFFF )
  {
    time = 1;
  }

  // calculate rate based on time (int 10ths) for full transition (1-254)
  rate = 255000 / time;    // units per tick, fixed point, 3 decimal places (e.g. 8500 = 8.5 units per tick)

  // start up state machine.
  zclSampleLight_WithOnOff = TRUE;
  zclSampleLight_MoveBasedOnRate( newLevel, rate );
}

/*********************************************************************
 * @fn      zclSampleLight_AdjustLightLevel
 *
 * @brief   Called each 10th of a second while state machine running
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleLight_AdjustLightLevel( void )
{
  // one tick (10th of a second) less
  if ( zclSampleLight_LevelRemainingTime )
  {
    --zclSampleLight_LevelRemainingTime;
  }

  // no time left, done
  if ( zclSampleLight_LevelRemainingTime == 0)
  {
    zclSampleLight_LevelCurrentLevel = zclSampleLight_NewLevel;
  }

  // still time left, keep increment/decrementing
  else
  {
    if ( zclSampleLight_NewLevelUp )
    {
      zclSampleLight_CurrentLevel32 += zclSampleLight_Rate32;
    }
    else
    {
      zclSampleLight_CurrentLevel32 -= zclSampleLight_Rate32;
    }
    zclSampleLight_LevelCurrentLevel = (uint8)( zclSampleLight_CurrentLevel32 / 1000 );
  }

#if (defined HAL_BOARD_ZLIGHT) || (defined HAL_PWM)
  zclSampleLight_UpdateLampLevel(zclSampleLight_LevelCurrentLevel);
#endif

  // also affect on/off
  if ( zclSampleLight_WithOnOff )
  {
    if ( zclSampleLight_LevelCurrentLevel > ATTR_LEVEL_MIN_LEVEL )
    {
      zclSampleLight_OnOff = LIGHT_ON;
#if (defined HAL_BOARD_ZLIGHT) || (defined HAL_PWM)
      ENABLE_LAMP;
#endif
    }
    else
    {
      zclSampleLight_OnOff = LIGHT_OFF;
#if (defined HAL_BOARD_ZLIGHT) || (defined HAL_PWM)
      DISABLE_LAMP;
#endif
    }
  }

  // display light level as we go
  zclSampleLight_DisplayLight( );

  // keep ticking away
  if ( zclSampleLight_LevelRemainingTime )
  {
    osal_start_timerEx( zclSampleLight_TaskID, SAMPLELIGHT_LEVEL_CTRL_EVT, 100 );
  }
}

/*********************************************************************
 * @fn      zclSampleLight_LevelControlMoveToLevelCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received a LevelControlMoveToLevel Command for this application.
 *
 * @param   pCmd - ZigBee command parameters
 *
 * @return  none
 */
static void zclSampleLight_LevelControlMoveToLevelCB( zclLCMoveToLevel_t *pCmd )
{
  // 调试信息：移动到级别命令回调
  debug_str("MoveToLevel");
  
  zclSampleLight_WithOnOff = pCmd->withOnOff;
  zclSampleLight_MoveBasedOnTime( pCmd->level, pCmd->transitionTime );
}

/*********************************************************************
 * @fn      zclSampleLight_LevelControlMoveCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received a LevelControlMove Command for this application.
 *
 * @param   pCmd - ZigBee command parameters
 *
 * @return  none
 */
static void zclSampleLight_LevelControlMoveCB( zclLCMove_t *pCmd )
{
  // 调试信息：移动命令回调
  debug_str("Level Move");
  
  uint8 newLevel;
  uint32 rate;

  // convert rate from units per second to units per tick (10ths of seconds)
  // and move at that right up or down
  zclSampleLight_WithOnOff = pCmd->withOnOff;

  if ( pCmd->moveMode == LEVEL_MOVE_UP )
  {
    newLevel = ATTR_LEVEL_MAX_LEVEL;  // fully on
  }
  else
  {
    newLevel = ATTR_LEVEL_MIN_LEVEL; // fully off
  }

  rate = (uint32)100 * pCmd->rate;
  zclSampleLight_MoveBasedOnRate( newLevel, rate );
}

/*********************************************************************
 * @fn      zclSampleLight_LevelControlStepCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an On/Off Command for this application.
 *
 * @param   pCmd - ZigBee command parameters
 *
 * @return  none
 */
static void zclSampleLight_LevelControlStepCB( zclLCStep_t *pCmd )
{
  // 调试信息：步进命令回调
  debug_str("Level Step");
  
  uint8 newLevel;

  // determine new level, but don't exceed boundaries
  if ( pCmd->stepMode == LEVEL_MOVE_UP )
  {
    if ( (uint16)zclSampleLight_LevelCurrentLevel + pCmd->amount > ATTR_LEVEL_MAX_LEVEL )
    {
      newLevel = ATTR_LEVEL_MAX_LEVEL;
    }
    else
    {
      newLevel = zclSampleLight_LevelCurrentLevel + pCmd->amount;
    }
  }
  else
  {
    if ( pCmd->amount >= zclSampleLight_LevelCurrentLevel )
    {
      newLevel = ATTR_LEVEL_MIN_LEVEL;
    }
    else
    {
      newLevel = zclSampleLight_LevelCurrentLevel - pCmd->amount;
    }
  }

  // move to the new level
  zclSampleLight_WithOnOff = pCmd->withOnOff;
  zclSampleLight_MoveBasedOnTime( newLevel, pCmd->transitionTime );
}

/*********************************************************************
 * @fn      zclSampleLight_LevelControlStopCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Level Control Stop Command for this application.
 *
 * @param   pCmd - ZigBee command parameters
 *
 * @return  none
 */
static void zclSampleLight_LevelControlStopCB( void )
{
  // 调试信息：停止命令回调
  debug_str("Level Stop");
  
  // stop immediately
  osal_stop_timerEx( zclSampleLight_TaskID, SAMPLELIGHT_LEVEL_CTRL_EVT );
  zclSampleLight_LevelRemainingTime = 0;
}
#endif

/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      zclSampleLight_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclSampleLight_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg )
{
  // 调试信息：处理ZCL消息
  debug_str("Process ZCL");
  
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      // 调试信息：读取响应
      debug_str("Read Rsp");
      zclSampleLight_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      // 调试信息：写入响应
      debug_str("Write Rsp");
      zclSampleLight_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT
    // Attribute Reporting implementation should be added here
    case ZCL_CMD_CONFIG_REPORT:
      // 调试信息：配置报告
      debug_str("Config Report");
      // zclSampleLight_ProcessInConfigReportCmd( pInMsg );
      break;

    case ZCL_CMD_CONFIG_REPORT_RSP:
      // 调试信息：配置报告响应
      debug_str("Config Rsp");
      // zclSampleLight_ProcessInConfigReportRspCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      // 调试信息：读取报告配置
      debug_str("Read Report Cfg");
      // zclSampleLight_ProcessInReadReportCfgCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG_RSP:
      // 调试信息：读取报告配置响应
      debug_str("Read Cfg Rsp");
      // zclSampleLight_ProcessInReadReportCfgRspCmd( pInMsg );
      break;

    case ZCL_CMD_REPORT:
      // 调试信息：报告命令
      debug_str("Report Cmd");
      // zclSampleLight_ProcessInReportCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      // 调试信息：默认响应
      debug_str("Default Rsp");
      zclSampleLight_ProcessInDefaultRspCmd( pInMsg );
      break;
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
      // 调试信息：发现命令响应
      debug_str("Disc Cmds Rsp");
      zclSampleLight_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
      // 调试信息：发现命令通用响应
      debug_str("Disc Gen Rsp");
      zclSampleLight_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      // 调试信息：发现属性响应
      debug_str("Disc Attrs Rsp");
      zclSampleLight_ProcessInDiscAttrsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
      // 调试信息：发现属性扩展响应
      debug_str("Disc Ext Rsp");
      zclSampleLight_ProcessInDiscAttrsExtRspCmd( pInMsg );
      break;
#endif
    default:
      // 调试信息：未知命令
      debug_str("Unknown Cmd");
      break;
  }

  if ( pInMsg->attrCmd )
    osal_mem_free( pInMsg->attrCmd );
}

#ifdef ZCL_READ
/*********************************************************************
 * @fn      zclSampleLight_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleLight_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < readRspCmd->numAttr; i++)
  {
    // Notify the originator of the results of the original read attributes
    // attempt and, for each successfull request, the value of the requested
    // attribute
  }

  return ( TRUE );
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclSampleLight_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleLight_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < writeRspCmd->numAttr; i++ )
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return ( TRUE );
}
#endif // ZCL_WRITE

/*********************************************************************
 * @fn      zclSampleLight_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleLight_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;

  // Device is notified of the Default Response command.
  (void)pInMsg;

  return ( TRUE );
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclSampleLight_ProcessInDiscCmdsRspCmd
 *
 * @brief   Process the Discover Commands Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleLight_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclSampleLight_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleLight_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclSampleLight_ProcessInDiscAttrsExtRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Extended Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleLight_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}
#endif // ZCL_DISCOVER

#if ZCL_EZMODE
/*********************************************************************
 * @fn      zclSampleLight_ProcessZDOMsgs
 *
 * @brief   Called when this node receives a ZDO/ZDP response.
 *
 * @param   none
 *
 * @return  status
 */
static void zclSampleLight_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg )
{
  zclEZMode_ActionData_t data;
  ZDO_MatchDescRsp_t *pMatchDescRsp;

  // Let EZ-Mode know of the Simple Descriptor Response
  if ( pMsg->clusterID == Match_Desc_rsp )
  {
    pMatchDescRsp = ZDO_ParseEPListRsp( pMsg );
    data.pMatchDescRsp = pMatchDescRsp;
    zcl_EZModeAction( EZMODE_ACTION_MATCH_DESC_RSP, &data );
    osal_mem_free( pMatchDescRsp );
  }
}

/*********************************************************************
 * @fn      zclSampleLight_EZModeCB
 *
 * @brief   The Application is informed of events. This can be used to show on the UI what is
*           going on during EZ-Mode steering/finding/binding.
 *
 * @param   state - an
 *
 * @return  none
 */
static void zclSampleLight_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData )
{
#ifdef LCD_SUPPORTED
  char *pStr;
  uint8 err;
#endif

  // 调试信息：EZ-Mode状态变化
  debug_str("EZ State");

  // time to go into identify mode
  if ( state == EZMODE_STATE_IDENTIFYING )
  {
    // 调试信息：进入识别模式
    debug_str("EZ: Identify");
    
#ifdef LCD_SUPPORTED
    HalLcdWriteString ( "EZMode", HAL_LCD_LINE_2 );
#endif

    zclSampleLight_IdentifyTime = ( EZMODE_TIME / 1000 );  // convert to seconds
    zclSampleLight_ProcessIdentifyTimeChange();
  }

  // autoclosing, show what happened (success, cancelled, etc...)
  if( state == EZMODE_STATE_AUTOCLOSE )
  {
    // 调试信息：自动关闭状态
    debug_str("EZ: AutoClose");
    
#ifdef LCD_SUPPORTED
    pStr = NULL;
    err = pData->sAutoClose.err;
    if ( err == EZMODE_ERR_SUCCESS )
    {
      pStr = "EZMode: Success";
    }
    else if ( err == EZMODE_ERR_NOMATCH )
    {
      pStr = "EZMode: NoMatch"; // not a match made in heaven
    }
    if ( pStr )
    {
      if ( giLightScreenMode == LIGHT_MAINMODE )
      {
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
      }
    }
#endif
  }

  // finished, either show DstAddr/EP, or nothing (depending on success or not)
  if( state == EZMODE_STATE_FINISH )
  {
    // 调试信息：EZ-Mode完成
    debug_str("EZ: Finish");
    
    // turn off identify mode
    zclSampleLight_IdentifyTime = 0;
    zclSampleLight_ProcessIdentifyTimeChange();

#ifdef LCD_SUPPORTED
    // if successful, inform user which nwkaddr/ep we bound to
    pStr = NULL;
    err = pData->sFinish.err;
    if( err == EZMODE_ERR_SUCCESS )
    {
      // already stated on autoclose
    }
    else if ( err == EZMODE_ERR_CANCELLED )
    {
      pStr = "EZMode: Cancel";
    }
    else if ( err == EZMODE_ERR_BAD_PARAMETER )
    {
      pStr = "EZMode: BadParm";
    }
    else if ( err == EZMODE_ERR_TIMEDOUT )
    {
      pStr = "EZMode: TimeOut";
    }
    if ( pStr )
    {
      if ( giLightScreenMode == LIGHT_MAINMODE )
      {
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
      }
    }
#endif
    // show main UI screen 3 seconds after binding
    osal_start_timerEx( zclSampleLight_TaskID, SAMPLELIGHT_MAIN_SCREEN_EVT, 3000 );
  }
}
#endif // ZCL_EZMODE

/****************************************************************************
****************************************************************************/

/*********************************************************************
 * @fn      zclSampleLight_SendSwitchReport
 *
 * @brief   Send switch state report to Zigbee network
 *
 * @param   endpoint - the endpoint to send report from
 * @param   state - the switch state (LIGHT_ON or LIGHT_OFF)
 *
 * @return  none
 */
static void zclSampleLight_SendSwitchReport(uint8 endpoint, uint8 state)
{
#ifdef ZCL_REPORT
  zclReportCmd_t *pReportCmd;
  afAddrType_t dstAddr;
  
  // 调试信息：发送开关报告
  debug_str("Send Switch Report");
  
  // 分配报告命令内存
  pReportCmd = osal_mem_alloc(sizeof(zclReportCmd_t) + sizeof(zclReport_t));
  if (pReportCmd != NULL)
  {
    // 设置报告命令
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_ON_OFF;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_BOOLEAN;
    pReportCmd->attrList[0].attrData = (void *)(&state);
    
    // 设置目标地址为协调器
    dstAddr.addrMode = afAddr16Bit;
    dstAddr.addr.shortAddr = 0x0000;  // 协调器地址
    dstAddr.endPoint = 1;  // 修正：协调器端点应该是1
    dstAddr.panId = 0;
    
    // 发送报告命令
    zcl_SendReportCmd(endpoint, &dstAddr, ZCL_CLUSTER_ID_GEN_ON_OFF,
                      pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSampleLight_SeqNum++);
    
    // 释放内存
    osal_mem_free(pReportCmd);
    
    // 调试信息：报告发送完成
    debug_str("Switch Report Sent");
  }
  else
  {
    // 调试信息：内存分配失败
    debug_str("Switch Report Mem Fail");
  }
#endif // ZCL_REPORT
}


