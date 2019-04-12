#ifndef __FF_CAN_H
#define __FF_CAN_H

#ifdef __cplusplus
extern "C"
{
#endif



typedef enum {
	BOOM_COLOR_OFF = 0,
	BOOM_COLOR_RED,
	BOOM_COLOR_ORANGE,
	BOOM_COLOR_YELLOW,
	BOOM_COLOR_GREEN,
	BOOM_COLOR_CYAN,
	BOOM_COLOR_BLUE,
	BOOM_COLOR_PURPLE,
	BOOM_COLOR_WHITE
} BOOM_LED_COLOR_e;

// Arms all motors and returns 0 on success, non-zero on fail
int FF_CAN_Arm_Motors(void);
int FF_CAN_Disarm_Motors(void);


#ifdef __cplusplus
}
#endif


#endif	// END __FF_CAN_H
