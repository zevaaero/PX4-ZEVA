#ifndef __FF_CAN_H
#define __FF_CAN_H

#ifdef __cplusplus
extern "C"
{
#endif

// Arms all motors and returns 0 on success, non-zero on fail
int FF_CAN_Arm_Motors(void);
int FF_CAN_Disarm_Motors(void);


#ifdef __cplusplus
}
#endif


#endif	// END __FF_CAN_H
