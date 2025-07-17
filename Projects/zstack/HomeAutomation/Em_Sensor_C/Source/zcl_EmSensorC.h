#ifndef ZCL_SAMPLELIGHT_H
#define ZCL_SAMPLELIGHT_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "zcl.h"

/*********************************************************************
 * CONSTANTS
 */
#define SAMPLELIGHT_ENDPOINT            8
#define SAMPLELIGHT_RED_ENDPOINT        9
#define SAMPLELIGHT_SWITCH1_ENDPOINT    10  // P1_2按键开关
#define SAMPLELIGHT_SWITCH2_ENDPOINT    11  // P1_3按键开关

#define LIGHT_OFF                       0x00
#define LIGHT_ON                        0x01

// Application Events
#define SAMPLELIGHT_IDENTIFY_TIMEOUT_EVT     0x0001
#define SAMPLELIGHT_POLL_CONTROL_TIMEOUT_EVT 0x0002
#define SAMPLELIGHT_EZMODE_TIMEOUT_EVT       0x0004
#define SAMPLELIGHT_EZMODE_NEXTSTATE_EVT     0x0008
#define SAMPLELIGHT_MAIN_SCREEN_EVT          0x0010
#define SAMPLELIGHT_LEVEL_CTRL_EVT           0x0020
#define SAMPLELIGHT_START_EZMODE_EVT         0x0040
#define SAMPLELIGHT_S1_CHECK_EVT             0x0080  

// Application Display Modes
#define LIGHT_MAINMODE      0x00
#define LIGHT_HELPMODE      0x01

/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */
extern SimpleDescriptionFormat_t zclSampleLight_SimpleDesc;
extern SimpleDescriptionFormat_t zclSampleLight_RedSimpleDesc;
extern SimpleDescriptionFormat_t zclSampleLight_Switch1SimpleDesc;  // P1_2开关
extern SimpleDescriptionFormat_t zclSampleLight_Switch2SimpleDesc;  // P1_3开关

extern CONST zclCommandRec_t zclSampleLight_Cmds[];

extern CONST uint8 zclCmdsArraySize;

// attribute list
extern CONST zclAttrRec_t zclSampleLight_Attrs[];
extern CONST uint8 zclSampleLight_NumAttributes;

// Identify attributes
extern uint16 zclSampleLight_IdentifyTime;
extern uint8  zclSampleLight_IdentifyCommissionState;

// OnOff attributes
extern uint8  zclSampleLight_OnOff;
extern uint8  zclSampleLight_RedOnOff;
extern uint8  zclSampleLight_Switch1OnOff;  // P1_2开关状态
extern uint8  zclSampleLight_Switch2OnOff;  // P1_3开关状态

// Level Control Attributes
#ifdef ZCL_LEVEL_CTRL
extern uint8  zclSampleLight_LevelCurrentLevel;
extern uint16 zclSampleLight_LevelRemainingTime;
extern uint16 zclSampleLight_LevelOnOffTransitionTime;
extern uint8  zclSampleLight_LevelOnLevel;
extern uint16 zclSampleLight_LevelOnTransitionTime;
extern uint16 zclSampleLight_LevelOffTransitionTime;
extern uint8  zclSampleLight_LevelDefaultMoveRate;
#endif

/*********************************************************************
 * FUNCTIONS
 */

 /*
  * Initialization for the task
  */
extern void zclSampleLight_Init( byte task_id );

/*
 *  Event Process for the task
 */
extern UINT16 zclSampleLight_event_loop( byte task_id, UINT16 events );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ZCL_SAMPLELIGHT_H */
