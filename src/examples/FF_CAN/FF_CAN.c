/**
 * @FF_CAN.c
 * Freefly Systems CAN bus based Alta motor driver implementation.
 *
 * @author Selim Ozel <selim.ozel@freeflysystems.com>
 * @author Cory Schwharzmiller <cory@freeflysystems.com>
 */

//-------------------------------------------------------------------------------------
// Includes
#include "examples/FF_CAN/FF_CAN.h"

// Required OS and general includes
#include <px4_config.h>
#include <px4_posix.h>
#include <string.h>
#include "stdbool.h"
#include <stddef.h>		// for NULL

// uOrb subscriptions
#include <uORB/topics/esc_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <drivers/drv_hrt.h>

// hardware specific includes
#include <nuttx/can/can.h>
#include "stm32.h"
#include "stm32_can.h"

//////////////////////////////////////////////////////////////////////////////////////////////
// This is a critcal define that should be set for the number of motors used
// generally it should be 8 or 6 for ALTA, and possibly 4 for quads
#define MTR_CNT		8
//////////////////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------------------------
// THE FOLLOWING ARE THE DEFINES FROM THE ALTA ESC FOR REFERENCE ONLY !!!

//CAN definitions
#define CAN_ADDR_ARM_SET 0x25
#define CAN_ADDR_ARM_ACK 0x24
#define CAN_ADDR_DISARM_SET 0x23
#define CAN_ADDR_DISARM_ACK 0x22
#define CAN_ADDR_BOOM_SET 0x21
#define CAN_ADDR_BOOM_ACK 0x20
#define CAN_ADDR_CONTROL_1 0x28
#define CAN_ADDR_CONTROL_2 0x29
#define CAN_ADDR_ARBITRARY 0x35
#define CAN_ADDR_ARBITRARY_2 0x36
#define CAN_ADDR_ARBITRARY_HIGH_PRIORITY 0x26
#define CAN_ADDR_FLASH_ACK 0x34
#define CAN_ADDR_TELEMETRY_REQUEST 0x2A
#define CAN_ADDR_TELEMETRY_RESPONSE 0x2B
#define CAN_ADDR_TELEMETRY_RESPONSE_2 0x2C
#define CAN_ADDR_PARAMETER_READ 0x30
#define CAN_ADDR_PARAMETER_READ_RESPONSE 0x31
#define CAN_ADDR_PARAMETER_SET 0x32
#define CAN_ADDR_PARAMETER_SET_RESPONSE 0x33
#define CAN_ADDR_LED_SET 0x38
#define CAN_ADDR_REVERSE 0x3F
#define CAN_ADDR_REVERSE_FAST 0x3E
#define CAN_FLASH_ACK_MESSAGE 0x2
#define CAN_UNLOCK_ACK_MESSAGE 0x1
#define CAN_LOCK_ACK_MESSAGE 0x3
#define CAN_ARB_UNLOCK 0x0
#define CAN_ARB_FLASH 0x1
#define CAN_ARB_LOCK 0x2
#define CAN_ARB_BEEP 0x3
#define CAN_ARB_ARM_LED 0x4
#define CAN_VERIFICATION_BOOM_HR 0x50039871
#define CAN_VERIFICATION_BOOM_LR 0x05956200
#define CAN_VERIFICATION_BOOM_LR_MASK 0xFFFFFF00
#define CAN_VERIFICATION_BOOM_ACK_HR 0x1e62125e
#define CAN_VERIFICATION_BOOM_ACK_LR 0x2c7bdd00 //Transmit only, no mask
#define CAN_VERIFICATION_FLASH_UNLOCK_HR 0x0a08b5bf
#define CAN_VERIFICATION_FLASH_UNLOCK_LR 0xd8160000
#define CAN_VERIFICATION_FLASH_UNLOCK_LR_MASK 0xFFFF0000
#define CAN_VERIFICATION_FLASH_UNLOCK_ACK_HR 0xd20db2dd
#define CAN_VERIFICATION_FLASH_UNLOCK_ACK_LR 0x96710000 //Transmit only, no mask
#define CAN_VERIFICATION_FLASH_LOCK_ACK_HR 0xac433647
#define CAN_VERIFICATION_FLASH_LOCK_ACK_LR 0x6e020000 //Transmit only, no mask
#define CAN_VERIFICATION_FLASH_HR 0x00572e7b
#define CAN_VERIFICATION_FLASH_LR 0xb8380000
#define CAN_VERIFICATION_FLASH_LR_MASK 0xFFFF0000
#define CAN_VERIFICATION_FLASH_ACK_HR 0x81d6863d
#define CAN_VERIFICATION_FLASH_ACK_LR 0xf6390000 //Transmit only, no mask
#define CAN_VERIFICATION_ARM_HR 0x91258e44
#define CAN_VERIFICATION_ARM_LR 0x028d5300
#define CAN_VERIFICATION_ARM_LR_MASK 0xFFFFFF00
#define CAN_VERIFICATION_DISARM_HR 0xf1d1d688
#define CAN_VERIFICATION_DISARM_LR 0x2b525c00
#define CAN_VERIFICATION_DISARM_LR_MASK 0xFFFFFF00
#define CAN_VERIFICATION_REVERSE_HR 0x1df1f778
#define CAN_VERIFICATION_REVERSE_LR 0x6418aa00
#define CAN_VERIFICATION_REVERSE_LR_MASK 0xFFFFFF00
#define CAN_VERIFICATION_ARM_ACK_HR 0xc18b42c1
#define CAN_VERIFICATION_ARM_ACK_LR 0xc4ae5f00
#define CAN_VERIFICATION_DISARM_ACK_HR 0x70d0dc37
#define CAN_VERIFICATION_DISARM_ACK_LR 0x2cee2100
#define CAN_VERIFICATION_REVERSE_FAST_LR 0xa2370000
#define CAN_VERIFICATION_REVERSE_FAST_LR_MASK 0xFFFF0000
#define CAN_VERIFICATION_REVERSE_FAST_HR 0x0000007d
#define CAN_VERIFICATION_REVERSE_FAST_HR_MASK 0x000000FF


#define MAX_READ_MSG_CNT_PER_LOOP	20

//-------------------------------------------------------------------------------------
// Verbose level of detail for debug can be enabled here if needed
//#define FF_CAN_VERBOSE_INFO



//-------------------------------------------------------------------------------------
// Type defs
typedef enum {
	FF_CAN_MsgType_BL_Jump = 0,
	FF_CAN_MsgType_MtrCmd0,
	FF_CAN_MsgType_MtrCmd1

} FF_CAN_MsgType_e;

//-------------------------------------------------------------------------------------
// Export main function for access by the main PX4 firmware
// This function must be called <module_name>_main
// It gets started by the shell using a startup script in the folder: /home/px4dev/cory/firmware/ROMFS/px4fmu_common/init.d/rcS
__EXPORT int FF_CAN_main(int argc, char *argv[]);

//-------------------------------------------------------------------------------------
// Public function prototypes
unsigned long PX4_to_Alta(unsigned long PWM_IN);	// Very simple function converting Px4 PWM value to an Alta Motor Driver PWM value.
bool Spy(void);					// Triggers debug mode of FF_CAN. 
void SetBoomID(int id);			// Sets the boom ID to "id" on ALL pwm high channels. 
int BootloaderJump(void);		// Makes the Alta Motor Driver jump to the driver code from the bootloader code in the Alta Startup
void FF_CAN_Request_IDs(void);
int FF_CAN_Send_Arm(uint8_t boomID);
void FF_CAN_Send_Disarm(uint8_t boomID);
int FF_CAN_Send_Lights(uint8_t boomID);

//-------------------------------------------------------------------------------------
// Private function prototypes

// Constructor function - called by main() - spawns other threads
void FF_CAN(void);

// CAN write task
static void FF_CAN_WriteTask(void);

// CAN read task
static void FF_CAN_ReadTask(void);

// Sends a predefined can message
int FF_CAN_MSG_Send(struct can_msg_s *msg_p, int fileDescriptor);

// Recieves a CAN message from the Rx buffer
int FF_CAN_MSG_Recv(struct can_msg_s *msg_p, int fileDescriptor);

void FF_CAN_Message_Rx_Parse(struct can_msg_s *msg_p);

// Utilities
unsigned long FF_CAN_PX4_to_Alta(unsigned long pwmIn);
bool FF_CAN_Spy(void);

// Temporary function for dev
int FF_CAN_Bootloader_Jump(void);
int FF_CAN_BootloaderJumpId(uint8_t id, uint8_t type, uint32_t address);
void FF_CAN_SetBoomID(int id);
void FF_CAN_MsgGen_ReProg(uint8_t boomID);

//-------------------------------------------------------------------------------------
// Private Global Vars
bool _moduleStarted = false;

// CAN device structure
struct can_dev_s *can_dev_p = NULL;		// Device specific CAN initializer. Mostly does pin config.

// CAN Status flags
bool _isInitialized;
bool _isRegistered;
bool _isSpyModeOn;

// Px4 Arm Flag
bool 		_isPx4Armed;
bool 		_areMotorsArming;
bool 		_areMotorsDisarming;
uint32_t 	_motorArmAck_BitFlags;
uint32_t 	_motorDisarmAck_BitFlags;
uint8_t 	_motorArmAck_Flags[MTR_CNT];
uint8_t 	_motorDisarmAck_Flags[MTR_CNT];
uint8_t 	_armCounter;
uint8_t 	_disarmCounter;

// Arm/Disarm vars
int Arm_Delay_Cntr = 0;

// PWM value local storage
unsigned long _PWM[MTR_CNT];
uint16_t	  _PWMTXRate;		// Rate of mctrl signals sent to Alta Motor Driver	

// Telemetry Variables
struct esc_status_s esc_stat;			// local storage for esc_status uORB message data
static orb_advert_t _esc_pub = NULL;	// advertisement for esc_status message
uint16_t    _telemTXRate;				// Number of telem packages requests sent per second
uint16_t 	_telemRX_1_Rate;			// Number of telem response 1 packets received per second
uint16_t 	_telemRX_2_Rate;			// Number of telem response 2 packets received per second

// CAN Rate
uint16_t 	_CANEventCounter;			// Number of times can0 was changed per second.

// Boom ID
uint8_t 	_boomID;					// Saves the ID from the last reprog operation.

// Bootloader ID Holder
uint32_t	_bootLoaderIDs[MTR_CNT];			// Contains the UIDs of each motor in the system
uint32_t	_esc_jump_address_set_ack_uid;	// Contains the last recieved UID of a jump to address set CAN command
uint32_t 	_esc_jump_execute_ack_uid;	// Contains the last recieved UID of a jump to address execute CAN command

// Daemon variables. The background threads started by apps are called daemons.
// The following control variables are imported from matlab_csv_serial
// but they are pretty standard throught out Px4 architecture.
// They are also static so they are consistent within this script but they cannot (at least should not)
// be accessed by other parts of the Px4.
bool _writeThreadShouldExit; 		// THREAD EXIT NOT IMPLEMENTED AT THE MOMENT
bool _writeThreadRunning;
int  _writeDaemonTask;

// Read thread variables. Same as write thread.
bool _readThreadShouldExit;
bool _readThreadRunning;
int  _readDaemonTask;

//-----------------------------------------------------------------------------------------------
// Public Function Defintions

//-----------------------------------------------------------------------------------------------
// FF_CAN module task
// CAN initialization and registration to FS are done here.
// It also spans subtasks to do read + write
void FF_CAN(void)
{
	// Set module started flag so we don't restart this on accident
	_moduleStarted = true;

	// Turn on the NART (No Automatic ReTries bit). -- Jeremy // This isn't working, arms won't jump
	// This ensures old data is not retransmitted if there is a timing error, which can cause the motors to lock up
	//uint32_t regval;
	//regval = getreg32(STM32_CAN_MCR_OFFSET);
  	//regval |= CAN_MCR_NART;
  	//putreg32(regval, STM32_CAN_MCR_OFFSET);

	// advertise the esc_status uORB message
	_esc_pub = orb_advertise(ORB_ID(esc_status), &esc_stat);

	// init ESC status uORB message
	memset(&esc_stat, 0, sizeof(esc_stat));
	esc_stat.esc_count = MTR_CNT;
	esc_stat.esc_connectiontype = ESC_CONNECTION_TYPE_CAN;
	
	for (int i=0; i<MTR_CNT ;i++)
	{
		esc_stat.esc[i].esc_vendor = ESC_VENDOR_GENERIC;
		esc_stat.esc[i].esc_address = i + 1;
	}

	// CAN initialization and device registration done below. Selim. 08/08/2018

	// Handle private variable initializations first.
	_isInitialized = false;
	_isRegistered = false;
	_isSpyModeOn = false;

	// Arm flag starts as false.
	_isPx4Armed = false;
	_areMotorsArming = false;
	_areMotorsDisarming = false;
	_armCounter = 1;
	_disarmCounter = 1;

	_motorArmAck_BitFlags = 0x00;
	_motorDisarmAck_BitFlags = 0x00;

	// CAN network packet rate control variables.
	_PWMTXRate = 0;

	_telemTXRate = 0;
	_telemRX_1_Rate = 0;
	_telemRX_2_Rate = 0;

	_CANEventCounter = 0;

	// Default ID is one. It's safe value.
	_boomID = 1;

	// clear all PWM to start
	for (int i = 0;i<MTR_CNT;i++){_PWM[i] = 0;}

	// Thread status variables.
	_writeThreadShouldExit = false;
	_writeThreadRunning = false;
	_writeDaemonTask = -1;

	_readThreadShouldExit = false;
	_readThreadRunning = false;
	_readDaemonTask = -1;

	// CAN port we want to access on Pixhawk is 1. Marked as CAN 2 on the Cube board.
	int canPort = 1;

	// Hardware specific can initialization function.
	// This returns the CAN device handle
	// Find stm32_can.h and stm32_can.c for definitions. They are buried inside \platforms\nuttx\..... Selim.
	can_dev_p = stm32_caninitialize(canPort);

	// Couldn't init.
	if (can_dev_p == NULL) {
		PX4_ERR("ERROR:  Failed to initialize CAN interface. \n");
		return;
	}

	// CAN initialized.
	_isInitialized = true;
	PX4_INFO("CAN initialized.");

	// can_register is a function living inside Can.c and Can.h. Selim.	
	// It saves the device name inside /dev
	int ret = can_register("/dev/can0", can_dev_p);

	// Check if register operation is successful.
	if (ret < 0) {
		PX4_ERR("ERROR: can_register failed: %d\n", ret);
		return;
	}
	// CAN registered.
	_isRegistered = true;
	PX4_INFO("CAN registered.");

	while(0)
	{
		FF_CAN_Request_IDs();
		usleep(1000000);
	}

	// CAN is initialized and registered. Handle CAN loop now.

	// Do the read thread with lower priority
	if(_readThreadRunning == false && _readDaemonTask == -1)
	{
		_readThreadShouldExit = false;
		_readDaemonTask = px4_task_spawn_cmd("FF_CANRead",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT, 			// 20.
						 //SCHED_PRIORITY_FAST_DRIVER,
						 //SCHED_PRIORITY_ACTUATOR_OUTPUTS,
						 1500,
						 (px4_main_t)&FF_CAN_ReadTask,
						 NULL);

		if(_readDaemonTask >= 0){
			PX4_INFO("FF_CANRead Task spawned.");
		}
		else{
			PX4_INFO("FF_CANRead Task spawned failed.");
		}
	}

	// Command each one of the motor drivers bootloader jump to the driver program start.
	// We should do this after the reader task has been started (we need it to read ack responses) but before the general writing starts
	if (FF_CAN_Bootloader_Jump())
	{
		PX4_ERR("FF_CAN - FAILED TO JUMP ESCS TO APPLICATION");
		return;
	}

	// Just want to make sure this won't get executed by mistake.
	// Therefore check if thread is running and if daemon is setup.
	if(_writeThreadRunning == false && _writeDaemonTask == -1)
	{
		_writeThreadShouldExit = false;
		_writeDaemonTask = px4_task_spawn_cmd("FF_CANWrite",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_ACTUATOR_OUTPUTS,	// (SCHED_PRIORITY_MAX-4) = 251.
						 1500,
						 (px4_main_t)&FF_CAN_WriteTask,
						 NULL);

		if(_writeDaemonTask >= 0){
			PX4_INFO("FF_CANWrite Task spawned.");
		}
		else{
			PX4_INFO("FF_CANWrite Task spawned failed.");
		}
	}
}

//-----------------------------------------------------------------------------------------------
// Get all of the CAN IDs
// sends a 0x0A - can_bootloader_id_bootloader_id_request
void FF_CAN_Request_IDs(void)
{
	// Open CAN with read/write
	int canFD = open("/dev/can0", O_RDWR);

	// Check if open is successful.
	if(canFD<0){
		PX4_INFO("Request_IDs-> Failed to open CAN device.");
		_writeThreadRunning = false;
		return;
	}

	// Setup message
	struct can_msg_s msg;
	msg.cm_hdr.ch_id = 0x0A;
	msg.cm_hdr.ch_rtr = 0;
	msg.cm_hdr.ch_dlc = 0;

	// set msg payload to command
	// (no data for this msg type)

	// send the message and wait until Tx complete
	FF_CAN_MSG_Send(&msg, canFD);

	// Make sure to close the local file descriptor before exiting
	close(canFD);
}

//-----------------------------------------------------------------------------------------------
// FF_CAN_Bootloader_Jump
// Function for making motor drives jump from their bootloader to their application code
// To do this, we must know the UID of each motor drive.
// First the code sends the broadcast ID request message, then listens for the responses
// The ESCs all respond with the same CAN ID, and the responses are random backoff timed to lessen the chance of collision
// In case there are collsions, the code will try to Rx all the IDs a couple times
// return ERROR status (0 = success)
int FF_CAN_Bootloader_Jump(void)
{
	PX4_INFO("--------------------------");
	PX4_INFO("Bootloader jumper started.");
	int ret = 0;

	ret = FF_CAN_BootloaderJumpId(9, 0x4, 0x08008000);	// jump each motor
	if (ret != 0){
		PX4_ERR("OSD MCU jump to application failed! - aborting bootloader jump!");
		return 1;	// stop and return if there is an error!
	}

	for (int i=1; i<MTR_CNT + 1; i++)
	{
		ret = FF_CAN_BootloaderJumpId(i, 0x6, 0x08008000);	// jump each motor
		if (ret != 0){
			PX4_ERR("Motor jump to application failed! - aborting bootloader jump!");
			return 1;	// stop and return if there is an error!
		}
	}

	PX4_INFO("Bootloader jumper finished.");
	return 0;
}

//-----------------------------------------------------------------------------------------------
// FF_CAN_MSG_Send
// Sends a predefined CAN message structure
// Inputs:
//		msg_p			: Pointer to a can_msg_s struct.
//		fileDescriptor 	: Integer describing CAN file to write.
// Output:
// 		returns 0 if write is success and -1 if fail.
int FF_CAN_MSG_Send(struct can_msg_s *msg_p, int fileDescriptor)
{
	// Variables for controlling CAN transmission
	size_t msgsize;
	ssize_t bytes_written;

	// CAN_MSGLEN is a convenience macro. It gets the data length of the message
	// and computes the actual size of the CAN message.
	msgsize = CAN_MSGLEN(msg_p->cm_hdr.ch_dlc);

	// Do the write operation
	bytes_written = write(fileDescriptor, msg_p, msgsize);

	// Check if message send is success.
	if ((size_t)bytes_written != msgsize || bytes_written < 0){
		#ifdef FF_CAN_VERBOSE_INFO
		PX4_INFO("CAN Write Error: write(%d) returned %d", msgsize, bytes_written);
		#endif
		return -1;
	}
	else{
		// Write success.
		#ifdef FF_CAN_VERBOSE_INFO
		PX4_INFO("CAN Write Success: write(%d) returned %d", msgsize, bytes_written);	
		#endif
		return 0;
	}

	// Shouldn't reach here.
	return -2;
}

//-----------------------------------------------------------------------------------------------
// FF_CAN_MSG_Recv
// Recieves a CAN message from the virtual file and packs it in a message
// Inputs:
//		msg_p			: Pointer to a can_msg_s struct. This can be viewed after function returns to get data
//		fileDescriptor 	: Integer describing CAN file to read.
// Output:
// 		returns 0 if read is success and -1 if fail.
int FF_CAN_MSG_Recv(struct can_msg_s *msg_p, int fileDescriptor)
{
	// Variables for controlling CAN recieve
	size_t msgsize;
	ssize_t bytes_read;

	// Gets the maximum possible size of a CAN message - we will try to read up to this amount
	msgsize = sizeof(struct can_msg_s);

	memset((void *)msg_p, 0, msgsize);	// clear msg to start

	#ifdef FF_CAN_VERBOSE_INFO
	PX4_INFO("CAN RECV from buffer\n");
	#endif

	// Try to read.
	bytes_read = read(fileDescriptor, msg_p, msgsize);

	// Check if read is successful.
	if ((size_t)bytes_read < CAN_MSGLEN(0) || (size_t)bytes_read > msgsize || bytes_read < 0){
		#ifdef FF_CAN_VERBOSE_INFO
		PX4_INFO("ERROR: read(%d) returned %d \n", msgsize, bytes_read);
		#endif
		return -1;
	}		
	else{
		#ifdef FF_CAN_VERBOSE_INFO
		// Read success. Write ID and DLC of the received CAN message.
		PX4_INFO("  ID: %4d DLC: %d -- OK", msg_p->cm_hdr.ch_id, msg_p->cm_hdr.ch_dlc);
		for (int j = 0; j<msg_p->cm_hdr.ch_dlc; j++){
			PX4_INFO("  Data[%d]: %4d -- OK", j, msg_p->cm_data[j]);
		}
		PX4_INFO("\n");
		#endif
		return 0;
	}

	// Shouldn't reach here.
	return -2;
}

//-----------------------------------------------------------------------------------------------
// SetBoomID
// Sets the boom ID to "int id". Does this for ALL pwm high channels. No return value as of now.
// The boom id set acknowledgement from Alta Motor Driver could be its return value in the future.
// Inputs:
// 		id 				: ID input to the boom given by the user.
// Output:
// 		None
void FF_CAN_SetBoomID(int id)
{
	// Open CAN with read/write
	int canFD = open("/dev/can0", O_RDWR);

	// Check if open is successful.
	if(canFD<0){
		PX4_INFO("SetBoomID-> Failed to open CAN device.");
		_writeThreadRunning = false;
		return;
	}

	// Setup message
	struct can_msg_s msg;
	msg.cm_hdr.ch_id = 33;
	msg.cm_hdr.ch_rtr = 0;
	msg.cm_hdr.ch_dlc = 8;

	// set msg payload to command
	msg.cm_data[0] = (id >> 0) & 0xFF;
	msg.cm_data[1] = (id >> 8) & 0xFF;
	msg.cm_data[2] = (id >> 16) & 0xFF;
	msg.cm_data[3] = (id >> 24) & 0xFF;

	// send the message and wait until Tx complete
	FF_CAN_MSG_Send(&msg, canFD);

	// Make sure to close the local file descriptor before exiting
	close(canFD);
}

//-----------------------------------------------------------------------------------------------
// Generate an ESC re-program address type message
void FF_CAN_MsgGen_ReProg(uint8_t boomID)
{
	// Open CAN with read/write
	int canFD = open("/dev/can0", O_RDWR);

	// Check if open is successful.
	if(canFD<0){
		PX4_INFO("SetBoomID-> Failed to open CAN device.");
		_writeThreadRunning = false;
		return;
	}

	// Setup message
	struct can_msg_s msg;
	msg.cm_hdr.ch_id = 33;
	msg.cm_hdr.ch_rtr = 0;
	msg.cm_hdr.ch_dlc = 8;

	// set msg payload to command
	msg.cm_data[0] = boomID;
	msg.cm_data[1] = 0x62;
	msg.cm_data[2] = 0x95;
	msg.cm_data[3] = 0x05;
	msg.cm_data[4] = 0x71;
	msg.cm_data[5] = 0x98;
	msg.cm_data[6] = 0x03;
	msg.cm_data[7] = 0x50;

	// send the message and wait until Tx complete
	FF_CAN_MSG_Send(&msg, canFD);

	// Make sure to close the local file descriptor before exiting
	close(canFD);
}

//-----------------------------------------------------------------------------------------------
// Send an arming message to the ESC
// returns error status (0 = success)
int FF_CAN_Send_Arm(uint8_t boomID)
{
	// Open CAN with read/write (blocking)
	int canFD = open("/dev/can0", O_RDWR);

	// Check if open is successful.
	if(canFD<0){
		PX4_INFO("Send Arm Cmd-> Failed to open CAN device.");
		_writeThreadRunning = false;
		return false;
	}

	// Setup message
	struct can_msg_s msg;
	msg.cm_hdr.ch_id = 0x25;
	msg.cm_hdr.ch_rtr = 0;
	msg.cm_hdr.ch_dlc = 8;

	// set msg payload to command
	msg.cm_data[0] = boomID;
	msg.cm_data[1] = 0x53;
	msg.cm_data[2] = 0x8d;
	msg.cm_data[3] = 0x02;
	msg.cm_data[4] = 0x44;
	msg.cm_data[5] = 0x8e;
	msg.cm_data[6] = 0x25;
	msg.cm_data[7] = 0x91;

	// send the message and wait until Tx complete
	FF_CAN_MSG_Send(&msg, canFD);

	// Make sure to close the local file descriptor before exiting
	close(canFD);

	// If the message doesn't get rxd or doesn't pass the checks, fail and disarm
	return 0;
}

//-----------------------------------------------------------------------------------------------
// Send a disarming message to the ESC
void FF_CAN_Send_Disarm(uint8_t boomID)
{
	// Open CAN with read/write (blocking)
	int canFD = open("/dev/can0", O_RDWR);

	// Check if open is successful.
	if(canFD<0){
		PX4_INFO("Send Disarm Cmd-> Failed to open CAN device.");
		_writeThreadRunning = false;
		return;
	}

	// Setup message
	struct can_msg_s msg;
	msg.cm_hdr.ch_id = 0x23;
	msg.cm_hdr.ch_rtr = 0;
	msg.cm_hdr.ch_dlc = 8;

	// set msg payload to command
	msg.cm_data[0] = boomID;
	msg.cm_data[1] = 0x5c;
	msg.cm_data[2] = 0x52;
	msg.cm_data[3] = 0x2b;
	msg.cm_data[4] = 0x88;
	msg.cm_data[5] = 0xd6;
	msg.cm_data[6] = 0xd1;
	msg.cm_data[7] = 0xf1;

	// send the message and wait until Tx complete
	FF_CAN_MSG_Send(&msg, canFD);

	// Make sure to close the local file descriptor before exiting
	close(canFD);
}

//-----------------------------------------------------------------------------------------------
// Send lights message to ESC
// returns error status (0 = success)
int FF_CAN_Send_Lights(uint8_t boomID)
{
	
	// Get lights values
	int32_t brightness = 0;
	int32_t color = BOOM_COLOR_RED;	
	uint32_t R = 128;
	uint32_t G = 128;
	uint32_t B = 128;
	

	param_get(param_find("BOOM_BRT"),&brightness);
	if( brightness <0 )
		brightness = 0;
	if( brightness > 15 )
		brightness = 15;

	// Get boom configured color
	switch( boomID )
	{
		case 1: param_get(param_find("BOOM1_COLOR"),&color); break;
		case 2: param_get(param_find("BOOM2_COLOR"),&color); break;
		case 3: param_get(param_find("BOOM3_COLOR"),&color); break;
		case 4: param_get(param_find("BOOM4_COLOR"),&color); break;
		case 5: param_get(param_find("BOOM5_COLOR"),&color); break;
		case 6: param_get(param_find("BOOM6_COLOR"),&color); break;
		case 7: param_get(param_find("BOOM7_COLOR"),&color); break;
		case 8: param_get(param_find("BOOM8_COLOR"),&color); break;
			
	}

	// color issues
	//white is too pink
	//yellow is too orange
	//purple is a little reddish

	switch(color) 
	{
		case BOOM_COLOR_OFF:
			R = 0; G = 0; B = 0; break;
		case BOOM_COLOR_RED:
			R = 255; G = 0; B = 0; break;
		case BOOM_COLOR_ORANGE:
			R = 255; G = 128; B = 0; break;
		case BOOM_COLOR_YELLOW:
			R = 255; G = 255; B = 0; break;
		case BOOM_COLOR_GREEN:
			R = 0; G = 255; B = 0; break;
		case BOOM_COLOR_CYAN:
			R = 0; G = 255; B = 255; break;
		case BOOM_COLOR_BLUE:
			R = 0; G = 0; B = 255; break;
		case BOOM_COLOR_PURPLE:
			R = 255; G = 0; B = 255; break;
		case BOOM_COLOR_WHITE:
			R = 220; G = 255; B = 255; break;
		default:
			R = 100; G = 100; B = 100; break;
	}

	// Adjust brightness
	R = R*brightness/20; G = G*brightness/20; B = B*brightness/20; // range is 15, divide by 20 caps max brightness to 200, which is Synapse upper brightness limit


	// Open CAN with read/write (blocking)
	int canFD = open("/dev/can0", O_RDWR);

	// Check if open is successful.
	if(canFD<0){
		PX4_INFO("Send Arm Cmd-> Failed to open CAN device.");
		_writeThreadRunning = false;
		return false;
	}

	// Setup message
	struct can_msg_s msg;
	msg.cm_hdr.ch_id = 0x38;	// Message 38 for color
	msg.cm_hdr.ch_rtr = 0;
	msg.cm_hdr.ch_dlc = 4;		// looks like 4 bytes

	// set msg payload to command
	msg.cm_data[0] = boomID;
	msg.cm_data[1] = (uint8_t)R;		// R
	msg.cm_data[2] = (uint8_t)G;		// G
	msg.cm_data[3] = (uint8_t)B;		// B


	// send the message and wait until Tx complete
	FF_CAN_MSG_Send(&msg, canFD);

	// Make sure to close the local file descriptor before exiting
	close(canFD);

	// If the message doesn't get rxd or doesn't pass the checks, fail and disarm
	return 0;
}

//-----------------------------------------------------------------------------------------------
// BootloaderJump
// Inputs:
// 			xx
// Outputs:
// 			return negative if bootloader didn't jump and zero if it did.
int FF_CAN_BootloaderJumpId(uint8_t id, uint8_t type, uint32_t address)
{
	struct can_msg_s msg;
	int msg_ret = 0;

	PX4_INFO("----------------------------");
	PX4_INFO("BootloaderJump-> sending jump command to CAN ID: %08x.", id);

	//--------------------------
	// STEP 0 - Open CAN peripheral as a virtual file for read and write

	// Open CAN with write access in blocking mode (messages will always get sent from FIFO)
	int canFD_write = open("/dev/can0", O_RDWR);

	// Check if open is successful.
	if(canFD_write<0){
		PX4_INFO("BootloaderJump-> Failed to open CAN device for write (blocking).");
		_writeThreadRunning = false;
		close(canFD_write);
		return -1;
	}
	PX4_INFO("BootloaderJump-> CAN opened for write (blocking).");

	//--------------------------
	// STEP 1 - send CANbus jump command

	// Setup message
	memset((void *)&msg, 0, sizeof(struct can_msg_s));	// clear msg to start
	msg.cm_hdr.ch_id = 0x10;	// 0x10 is jump to address
	msg.cm_hdr.ch_rtr = 0;
	msg.cm_hdr.ch_dlc = 5;

	// combine type code (0x7 for ESC) and id
	msg.cm_data[0] = ((id << 3) & 0xF8) | ((type << 0) & 0x07);

	// 4 byte jump address = ESC address to jump to (little endian)
	msg.cm_data[1] = (address >> 0) & 0xFF;
	msg.cm_data[2] = (address >> 8) & 0xFF;
	msg.cm_data[3] = (address >> 16) & 0xFF;
	msg.cm_data[4] = (address >> 24) & 0xFF;

	// clear the response UID
	_esc_jump_address_set_ack_uid = 0;

	// send the message and wait until Tx complete
	PX4_INFO("BootloaderJump-> Sending jump address value.");
	msg_ret = FF_CAN_MSG_Send(&msg, canFD_write);

	// check if write was successful
	if (msg_ret < 0){
		close(canFD_write);
		return -1;	// ERROR!
	}

	// //--------------------------
	// // STEP 2 - sleep for a little bit to wait for the response
	// usleep(5000);

	// //--------------------------
	// // STEP 3 - wait for a confirmation response message

	// // check for ack flag
	// if(_esc_jump_address_set_ack_uid != id){
	// 	close(canFD_write);
	// 	return -1;
	// }

	// //--------------------------
	// // STEP 4 - Execute ESC jump to previously set address (id=0x4E=78)
	// memset((void *)&msg, 0, sizeof(struct can_msg_s));	// clear msg to start
	// msg.cm_hdr.ch_id = 0x4E;	// Set channel id as 0x4E.	
	// msg.cm_hdr.ch_rtr = 0;		// Do not turn on Remote Transmission Request.
	// msg.cm_hdr.ch_dlc = 4; 		// Data Length Code: = 4 Byte.

	// msg.cm_data[0] = (id >> 0) & 0xFF;
	// msg.cm_data[1] = (id >> 8) & 0xFF;
	// msg.cm_data[2] = (id >> 16) & 0xFF;
	// msg.cm_data[3] = (id >> 24) & 0xFF;

	// // clear the response UID
	// _esc_jump_execute_ack_uid = 0;

	// // send the message and wait until Tx complete
	// PX4_INFO("BootloaderJump-> Sending jump execute command.");
	// msg_ret = FF_CAN_MSG_Send(&msg, canFD_write);

	// // check if write was successful
	// if (msg_ret < 0){
	// 	close(canFD_write);
	// 	return -1;	// ERROR!
	// }

	// //--------------------------
	// // STEP 4.5 - sleep for a little bit to wait for the response
	// usleep(5000);

	// //--------------------------
	// // STEP 5 - wait for a confirmation response message

	// // check for ack flag
	// if(_esc_jump_execute_ack_uid != id){
	// 	close(canFD_write);
	// 	return -1;
	// }

	// PX4_INFO("BootloaderJump-> Completed successfully!");

	close(canFD_write);
	return 0;
}

//-----------------------------------------------------------------------------------------------
// This is called by the state_machine_helper.cpp module function called "arming_state_transition()"
// It tries to send arming commands to the motors directly from that task
int FF_CAN_Arm_Motors(void)
{
	_motorArmAck_BitFlags = 0;

	// clear the arming flags
	for (int i=0; i<MTR_CNT; i++) 
	{
		_motorArmAck_Flags[i] = 0;
	}

	// cycle through motors, arming each one
	for (int i=0; i<MTR_CNT; i++) 
	{
		// send the arming request
		FF_CAN_Send_Arm(i+1);

		// poll the arming status to see if it succeeds
		int retries = 20;
		while (retries-- > 0)
		{
			usleep(1000);	// short delay to hopefully Rx a response!
			if (_motorArmAck_Flags[i]){
				break;
			}
		}

		// Check for arming success - if not, then arming has failed
		if (_motorArmAck_Flags[i] == 0)
		{
			// Disarm all the motors in case some have been armed
			FF_CAN_Disarm_Motors();

			return 1;	// Error!
		}
	}
	

	// Send lights command since we armed successfully
	for (int i=0; i<MTR_CNT; i++) 
	{
		FF_CAN_Send_Lights(i+1);
	}

	return 0;

}

//-----------------------------------------------------------------------------------------------
// This is called by the state_machine_helper.cpp module function called "arming_state_transition()"
// It tries to send disarming commands to the motors directly from that task
// NOTE: unlike the arming function, this function will always send disarm commands to all motors. 
// It does still return the status of the process though (Rx of ACKs)
int FF_CAN_Disarm_Motors(void)
{
	_motorDisarmAck_BitFlags = 0;
	bool disarm_fail = false;

	// clear the arming flags
	for (int i=0; i<MTR_CNT; i++) 
	{
		_motorDisarmAck_Flags[i] = 0;
	}

	// cycle through motors, disarming each one
	for (int i=0; i<MTR_CNT; i++) 
	{
		// send the arming request
		FF_CAN_Send_Disarm(i+1);

		// poll the arming status to see if it succeeds
		int retries = 5;
		while (retries-- > 0)
		{
			usleep(2000);	// short delay to hopefully Rx a response!
			if (_motorDisarmAck_Flags[i]){
				break;
			}
		}

		// Check for arming success - if not, then arming has failed
		if (_motorDisarmAck_Flags[i] == 0)
		{
			disarm_fail = true;
		}
	}

	// return the result of all the ACKs
	if (disarm_fail){
		return 1;	// Error!
	} else {
		return 0;	// Success!
	}
}


//-----------------------------------------------------------------------------------------------
// FF_CAN: Private Functions

//-----------------------------------------------------------------------------------------------
// CANbus write task
// Started by the main FF_CAN task, and then iterated at each run of the PWM CAN output loop.
// 
// This loop should be running at the same frequency as the actuator_outputs topic.
// NOTE: In order to run this task as soon as the actuator outputs are ready, the interval set in 
// "orb_set_interval()" should be set to the lowest value (1ms). 
// This allows the loop to run on every single recieved object update, as opposed to skipping some if it were too slow!
//
void FF_CAN_WriteTask(void)
{
	PX4_INFO("FF_CAN_Writer-> Thread started.");
	_writeThreadRunning = true;

	// Open CAN with read/write and non-block options.
	int writerFD = open("/dev/can0", O_RDWR | O_NONBLOCK);

	// Check if open is successful.
	if(writerFD<0){
		PX4_INFO("FF_CAN_Writer-> ERROR: Failed to open CAN device.");
		_writeThreadRunning = false;
		PX4_INFO("FF_CAN_Writer-> Thread Exit");
		return;
	}
	PX4_INFO("FF_CAN_Writer-> CAN opened");

	// Clock
	const hrt_abstime task_start = hrt_absolute_time();
	hrt_abstime lastRun = task_start;
	uint16_t logCounter = 0;
	uint16_t telemCounter = 0;

	// Subscriptions - uOrb
	int actuatorOutputsMinPeriod = 1;	// in [ms]
	//int oneSecLoopCounter = 1000/actuatorOutputsMinPeriod; 	// Counter tick to reach 1 second

	// If there are multiple instances use "orb_subscribe_multi"
	//int actuatorOutputs = orb_subscribe(ORB_ID(actuator_outputs));
	int actuatorOutputs = orb_subscribe_multi(ORB_ID(actuator_outputs),1);
	struct actuator_outputs_s actuatorOutputsData;

	// Run as fast as the objects get published! up to 1KHz
	orb_set_interval(actuatorOutputs, actuatorOutputsMinPeriod);

	// Make a subscription check here. 
	if(actuatorOutputs < 0)
	{
		PX4_INFO("FF_CAN_Writer-> ERROR: At least one uOrb subscription failed.");
		_writeThreadRunning = false;
		PX4_INFO("FF_CAN_Writer-> Thread Exit");
		return;
	}

	// poll descriptor
	px4_pollfd_struct_t pollfd = {};
	pollfd.fd = actuatorOutputs;
	pollfd.events = POLLIN;

	//----------------------------
	// Primary thread loop
	//_threadShouldExit = true;
	while(!_writeThreadShouldExit)
	{
		// The polling and loop control method used here is pretty standard throughout Px4 architecture.
		// I got this design from mc_att_control. 

		// Wait for up to 100ms for data while waiting for actuator_outputs
		int myRet = px4_poll(&pollfd, 1, 100);
		
		// Waited and got nothing. Start over.
		if (myRet == 0) {
			continue;
		}

		// Something really bad happened and while loop will get executed fast and it will block everything!
		// To handle that make the microcontroller sleep.
		if (myRet < 0) {
			PX4_ERR("FF_CAN_Writer-> poll error %d, %d", myRet, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		// Check if there was an object published
		if (pollfd.revents & POLLIN) {

			// copy the PWM values from the recieved object
			orb_copy(ORB_ID(actuator_outputs), actuatorOutputs, &actuatorOutputsData);
			unsigned long localPWMList[MTR_CNT];

			for (int i=0; i<MTR_CNT; i++) localPWMList[i] = FF_CAN_PX4_to_Alta(actuatorOutputsData.output[i]);					
			
			// Two messages are required to send all the command values:
			struct can_msg_s msg;

			// Setup message
			memset((void *)&msg, 0, sizeof(struct can_msg_s));
			msg.cm_hdr.ch_id = 0x28;		// Set channel id as 0b101011.	
			msg.cm_hdr.ch_rtr = 0;		// Do not turn on Remote Transmission Request.
			msg.cm_hdr.ch_dlc = 8; 		// Data Length Code: = 8 Bytes.

			// set msg payload to command
			msg.cm_data[0] = localPWMList[0] & 0xFF;
			msg.cm_data[1] = (localPWMList[0] >> 8) & 0xFF;

			msg.cm_data[2] = localPWMList[1] & 0xFF;
			msg.cm_data[3] = (localPWMList[1] >> 8) & 0xFF;

			msg.cm_data[4] = localPWMList[2] & 0xFF;
			msg.cm_data[5] = (localPWMList[2] >> 8) & 0xFF;

			msg.cm_data[6] = localPWMList[3] & 0xFF;
			msg.cm_data[7] = (localPWMList[3] >> 8) & 0xFF;

			// send the message and wait until Tx complete
			FF_CAN_MSG_Send(&msg, writerFD);

			// Setup message
			memset((void *)&msg, 0, sizeof(struct can_msg_s));
			msg.cm_hdr.ch_id = 0x29;
			msg.cm_hdr.ch_rtr = 0;		// Do not turn on Remote Transmission Request.
			msg.cm_hdr.ch_dlc = 8; 		// Data Length Code: = 8 Bytes.

			// set msg payload to command
			if (MTR_CNT >= 6)
			{
				msg.cm_data[0] = localPWMList[4] & 0xFF;
				msg.cm_data[1] = (localPWMList[4] >> 8) & 0xFF;

				msg.cm_data[2] = localPWMList[5] & 0xFF;
				msg.cm_data[3] = (localPWMList[5] >> 8) & 0xFF;
			}
			if (MTR_CNT >= 8)
			{
				msg.cm_data[4] = localPWMList[6] & 0xFF;
				msg.cm_data[5] = (localPWMList[6] >> 8) & 0xFF;

				msg.cm_data[6] = localPWMList[7] & 0xFF;
				msg.cm_data[7] = (localPWMList[7] >> 8) & 0xFF;
			}

			// send the message and wait until Tx complete
			FF_CAN_MSG_Send(&msg, writerFD);

			_PWMTXRate++;

			// Round robin request telemetry from ESCs, one at a time
			// Sets the frequency of the telemetry loop with respect to actuator_outputs.
			// The telemetry loop will generally run as fast as the control loop, but only get one ESCs data per loop
			if(!_areMotorsArming)
			{
				// Setup message
				memset((void *)&msg, 0, sizeof(struct can_msg_s));
				msg.cm_hdr.ch_id = 0x2A;	// 0x2A or 42 or 0b101010 for telemetry request packet
				msg.cm_hdr.ch_rtr = 0;		// Do not turn on Remote Transmission Request.
				msg.cm_hdr.ch_dlc = 1; 		// Data Length Code: = 1 Byte (motor ID)

				// cycle through motor IDs to get telem round robin
				msg.cm_data[0] = telemCounter++;

				// send the message and wait until Tx complete
				FF_CAN_MSG_Send(&msg, writerFD);

				_telemTXRate++;
			}

			// reset telemtry request ID if all have been requested (IDs are 1 to 8)
			if (telemCounter >= MTR_CNT) telemCounter = 1;
			
			// One second check - log to console at a slower rate
			hrt_abstime now = hrt_absolute_time();
			if((now-lastRun)> 1000000)
			{
				if(_isSpyModeOn)
				{
					printf("Time Interval is %d ms.\n\n",(now-lastRun));

					printf("\nFF_CAN_Writer-> Number of telem request reckages sent per second is %d.\n", _telemTXRate);
					printf("FF_CAN_Writer-> Number of mctrl packages written per second is %d. \n\n", _PWMTXRate);
					
					for (int i = 0; i<MTR_CNT; i++)
					{
						printf("PWM from actuator output topic at index %d is %d \n", i, (int)actuatorOutputsData.output[i]);
					}
				}
				_telemTXRate = 0;
				_PWMTXRate = 0;
				logCounter = 0;

				lastRun = now;
			}

			// Update counters.
			logCounter++;
		}
	}	// end task while loop

	_writeThreadRunning = false;
	PX4_INFO("FF_CAN_Writer-> Thread Exit");
}

//-----------------------------------------------------------------------------------------------
// CANbus read task
// Started by the main FF_CAN task
// Low priority read thread running whenever there is a CANbus event to capture telemetry.
//
void FF_CAN_ReadTask()
{
	PX4_INFO("FF_CAN_Reader-> Thread started.");
	_readThreadRunning = true;
	
	// Open CAN with read/write and non-block options.
	int readerFD = open("/dev/can0", O_RDONLY | O_NONBLOCK);

	// Check if open is successful.
	if(readerFD<0){
		PX4_INFO("FF_CAN_Reader-> ERROR: Failed to open CAN device.");
		_readThreadRunning = false;
		PX4_INFO("FF_CAN_Reader-> Thread Exit");		
		return;
	}

	// Allocate receive message variables.
	struct can_msg_s rxmsg;						// Struct holding the received message.							

	// Timer is used to clock the loop at 1 second to check message rates.
	const hrt_abstime taskStart = hrt_absolute_time();
	hrt_abstime lastRun = taskStart;	

	// File desriptor for CAN.
	struct pollfd fds = {};
	fds.fd = readerFD;
	fds.events = POLLIN;

	while(!_readThreadShouldExit)
	{
		// Similar to writer loop implementation.
		// Note that loop control is hooked to can0 changes rather than uOrb messages.
		// Any recieved CAN messages will cause the poll statement to unblock and run
		// This way CAN read runs with minimal latency.
		// Note that multiple messages may be received in a short time, so this will try to read any availible in a loop
		poll(&fds, 1, 10);

			// Only execute this part if can0 is changed.
			if (fds.revents & POLLIN) {
				_CANEventCounter++;

				// Try to read several messages per loop - this will ensure we read messages that occur before 
				for (int i=0; i<MAX_READ_MSG_CNT_PER_LOOP; i++)
				{
					if (FF_CAN_MSG_Recv(&rxmsg, readerFD) == 0)
					{
						FF_CAN_Message_Rx_Parse(&rxmsg);

					} else {
						// MSG read was not succesful - most likely all messages have been read!
						break;
					}
				}
			}
		

		// One second check.
		hrt_abstime now = hrt_absolute_time();
		if((now - lastRun) > 1000000)
		{
			if(_isSpyModeOn)
			{
				printf("FF_CAN_Reader-> Number of times CAN changed per second is %d .\n", _CANEventCounter);
				printf("FF_CAN_Reader-> Number of telem RES1 packages captured is %d .\n", _telemRX_1_Rate);
				printf("FF_CAN_Reader-> Number of telem RES2 packages captured is %d .\n\n", _telemRX_2_Rate);
				printf("FF_CAN_Reader-> Arm Ack Flags: %02x \n", _motorArmAck_BitFlags);
				printf("FF_CAN_Reader-> Disarm Ack Flags: %02x \n", _motorDisarmAck_BitFlags);
			}
			_telemRX_1_Rate = 0;
			_telemRX_2_Rate = 0;
			_CANEventCounter = 0;
			lastRun = now;
		}
	}

	_readThreadRunning = false;
	PX4_INFO("FF_CAN_Reader-> Thread Exit");
}

//-------------------------------------------------------------------------------------
// Dedicated Rx Parsing function
// Takes data from CAN messages and parses it to application vars
void FF_CAN_Message_Rx_Parse(struct can_msg_s *msg_p)
{
	int id_idx = 0;
	uint32_t uid = 0; 

	switch(msg_p->cm_hdr.ch_id)
	{
		case 0x08:	// UID RESPONSE
			uid = *(uint32_t *)&msg_p->cm_data[0];
			printf("FF_CAN_Reader-> Recieved UID discovery response from UID: %08x \n", uid);

			// check/build the ID list
			for (int i=0; i<MTR_CNT; i++)
			{
				// break out if we have already added it
				if (_bootLoaderIDs[i] == uid){
					break;

				// add it and break out if we have an empty space in the list
				} else if (_bootLoaderIDs[i] == 0){
					_bootLoaderIDs[i] = uid;
					break;
				}
			}
			break;

		case 0x41:	// JUMP ADDRESS SET ACK
			uid = *(uint32_t *)&msg_p->cm_data[0];
			_esc_jump_address_set_ack_uid = uid;
			printf("FF_CAN_Reader-> Jump address set ACK Rxd: %08x \n", uid);
			break;

		case 0x4D:	// JUMP EXECUTE ACK
			uid = *(uint32_t *)&msg_p->cm_data[0];
			_esc_jump_execute_ack_uid = uid;
			printf("FF_CAN_Reader-> Jump execute ACK Rxd: %08x \n", uid);
			break;

		case CAN_ADDR_ARM_ACK:
			id_idx = msg_p->cm_data[0] - 1;
			if ((id_idx < MTR_CNT) && (id_idx >= 0))
			{
				_motorArmAck_BitFlags |= (0x00000001 << id_idx);
				_motorArmAck_Flags[id_idx] = 1;
				printf("FF_CAN_Reader-> Arm ACK Rxd: %d \n", id_idx + 1);
				printf("FF_CAN_Reader-> Arm ACK Flags: %02x \n", _motorArmAck_BitFlags);
			}
			break;

		case CAN_ADDR_DISARM_ACK:
			id_idx = msg_p->cm_data[0] - 1;
			if ((id_idx < MTR_CNT) && (id_idx >= 0))
			{
				_motorDisarmAck_BitFlags |= (0x00000001 << id_idx);
				_motorDisarmAck_Flags[id_idx] = 1;
				printf("FF_CAN_Reader-> Disarm ACK Rxd: %d \n", id_idx + 1);
				printf("FF_CAN_Reader-> Disarm ACK Flags: %02x \n", _motorDisarmAck_BitFlags);
			}
			break;

		case CAN_ADDR_TELEMETRY_RESPONSE:

			id_idx = (msg_p->cm_data[0] & 0xF) - 1;
			if (id_idx < 0 || id_idx > MTR_CNT) break;

			// lower unsigned int
			// bits 0 to 3 = boom ID
			// bits 4 to 15 = voltage * 0.064
			// bits 16 to 23 = fault state
			// bits 24 to 31 = temperature + 100

			uint32_t voltage = *(uint32_t *)&msg_p->cm_data[0];
			voltage = (voltage >> 4) & 0xFFF;
			esc_stat.esc[id_idx].esc_voltage = (float)voltage / 0.064f;

			esc_stat.esc[id_idx].esc_state = msg_p->cm_data[2];

			float temp = msg_p->cm_data[3];
			esc_stat.esc[id_idx].esc_temperature = temp - 100.0f;

			// upper unsigned int
			// bits 0 to 15 = rpm
			// bits 16 to 31 = current (Amps) * 0.01 + 32768

			esc_stat.esc[id_idx].esc_rpm = *(uint16_t *)&msg_p->cm_data[4];

			float cur = *(uint16_t *)&msg_p->cm_data[6];
			esc_stat.esc[id_idx].esc_current = (cur - 32768.0f) / 0.01f;

			_telemRX_1_Rate++;
			break;

		case CAN_ADDR_TELEMETRY_RESPONSE_2:

			id_idx = (msg_p->cm_data[0] & 0xF) - 1;
			if (id_idx < 0 || id_idx > MTR_CNT) break;

			// lower unsigned int
			// bits 0 to 3 = boom ID
			// bits 8 to 15 = Accelerometer X
			// bits 16 to 23 = Accelerometer Y
			// bits 24 to 31 = Accelerometer Z

			_telemRX_2_Rate++;
			break;

		default:
			break;
	}

	// Each time we Rx a telemetry response we should publish the data
	if (msg_p->cm_hdr.ch_id == CAN_ADDR_TELEMETRY_RESPONSE)
	{
		esc_stat.timestamp = hrt_absolute_time();	// note, we should update this directly before publishing
		esc_stat.counter++;
		orb_publish(ORB_ID(esc_status), _esc_pub, &esc_stat);
	}

}


//-------------------------------------------------------------------------------------
// List of app arguments:
// Single Shot (SS) commands only works if started and not hooked.
//
// start 		: 	Initializes and registers CAN.
// status 		: 	Currently only tells if start was executed. More details can be added later on. For example spit out a status message using private flags in the class.
// stops 		: 	Stop. 												NOT IMPLEMENTED.
// subscribe 	: 	Subscribes the CAN onto "actuator_output" topic.	NOT IMPLEMENTED.
// armreq		:   SS - Sends an arm request to motor driver.  		NOT IMPLEMENTED.
// darmreq		:	SS - Sends a disarm req. 							NOT IMPLEMENTED.
// mCtrl 		:	SS - Sets motor driver rpm. 						NOT IMPLEMENTED.
int FF_CAN_main(int argc, char *argv[])
{

	// App entry.
	PX4_INFO("FF CAN Module Started");

	// Check if CAN is loaded. If it is don't do anything.
	if (!strcmp(argv[1], "start")) {
		
		// Create and Open CAN first
		if (_moduleStarted) 
		{
			PX4_ERR("CAN already started.");
		}
		else
		{
			// run the FF CAN task
			FF_CAN(); 
		}
	}	

	// Sets the stop flag and stops the loop running inside the spawned thread.
	// It does NOT deinit the CAN. It actually does NOTHING at the moment.
	if (!strcmp(argv[1], "stop")) {
		//threadShouldExit = true;
		return 0;
	}	

	// Give status update about CAN.
	if (!strcmp(argv[1], "status")) {
		if (_moduleStarted) {
			PX4_INFO("CAN is running.");
			return 0;
		}
	}	

	// Start broadcasting CAN network details to the system console
	if (!strcmp(argv[1], "spy")) {
		if (_moduleStarted) {
			FF_CAN_Spy();
			return 0;
		}
	}

	// Sets boom ID. Should be done once at factory.
	// We might want to remove this shell command from release version completely.
	if (!strcmp(argv[1], "setid"))
	{
		if (_moduleStarted)
		{
			if(argc < 3)
			{
				PX4_INFO("Missing argument: Boom ID");
			}
			if(argc == 3)
			{
				int myBoomID = atoi(argv[2]);
				PX4_INFO("Setting boom ID to %d on all PWM high channels.", myBoomID);
				FF_CAN_SetBoomID(myBoomID);
			}
		}
	}

	PX4_INFO("FF CAN Module Exited");
	return 0;
}

//-----------------------------------------------------------------------------------------------
// PX4_to_Alta
// Converts a PWM signal defined as Px4 type to a PWM signal defined as Alta Driver type. 
// Inputs: 
// 		PWM_IN			: PWM signal in microseconds obtained from actuator_outputs topic at Px4
// Output:
// 		returns an unsigned lon value ready to be sent to Alta Motor Driver over CAN.
unsigned long FF_CAN_PX4_to_Alta(unsigned long pwmIn)
{
	// The CAN PWM value is in units of microseconds * 16
	unsigned long pwmOut = pwmIn*16;

	// absolute maximum is 2000 uS * 16
	uint32_t max_pwm = 2000 * 16;
	if(pwmOut > max_pwm){pwmOut = max_pwm;}		// Just check upper bound since unsigned. And we can have 0 zero pwmouts.
	return pwmOut;
}

//-----------------------------------------------------------------------------------------------
// Spy
// Call once, it spits out CAN data onto the system console. Call again, it stops doing that.
// Inputs:
// 		None
// Output:
// 		return _isSpyModeOn (bool) indicating whether it's on or not.
bool FF_CAN_Spy()
{
	if(_isSpyModeOn){_isSpyModeOn = false;}
	else {_isSpyModeOn = true;}

	return _isSpyModeOn;
}


// //LED's
// #define ONBOARD_LED_FULL_BRIGHTNESS 255
// #define ONBOARD_LED_BLINK_BRIGHTNESS 200
// #define ONBOARD_LED_OFF 0

// //LED temperature compensation. Above this temperature (C), LED linearly decreases up to 155C
// #define TEMP_THRESHOLD 80
// #define TEMPERATURE_RANGE (255 - 100 - TEMP_THRESHOLD)


// //Beeping
// #define BEEP_PWM 65
// #define BEEP_ENABLE TIM3->DIER |= TIM_DIER_CC1IE
// #define BEEP_DISABLE TIM3->DIER &= ~TIM_DIER_CC1IE
// #define BEEP_PRESCALER 12000 //Prescale the system clock by n to get 1/2 tone generation frequency
// #define BEEP_TIMEOUT 600 //Prescale cycles for the beep to run
// #define BEEP_ACTIVE (TIM3->DIER & TIM_DIER_CC1IE)

// //CAN Operations
// #define PURGE_CAN CAN1->TSR |= CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2 //Call this to make room in the buffers for a critical message

// //Temperature filter
// #define TEMP_A 0.999F //A/(1-A) deltaT = Tau. 0.999 sets for tau=1second at deltaT = 1ms
// #define PWM_A 0.99F //0.9 sets for tau = 100ms at deltaT = 1ms

// //PWM Timing
// #define PWM_ACCEPT_L 800 //Min PWM that is accepted as a fail_timer reset input
// #define PWM_ACCEPT_H 2200 //Max PWM tha tis accepted as a fail_timer reset input
// #define CAN_ACCEPT_L (800 * 16)
// #define CAN_ACCEPT_H (2200 * 16)
// #define PWM_MIN_ARM 1050
// #define PWM_ARM_RPM 750
// #define PWM_RANGE 750 //RPM CL to RPM Max range from PWM_MIN_ARM to (PWM_MIN_ARM + PWM_RANGE)
// #define PWM_MAX_ERROR 2200 //Above this level, signal is treated as an error

// #define PWM_ERROR_THRESHOLD (PWM_RANGE / 10) //(1/1-e) time constant


// THE FOLLOWING ARE THE CAN IDS FROM THE ALTA ESC BOOTLOADER
// typedef enum { can_bootloader_id_system_reset_set = 0x004, can_bootloader_id_bootloader_id_response = 0x008,
//                can_bootloader_id_bootloader_id_set = 0x009, can_bootloader_id_bootloader_id_request = 0x00A,
//                can_bootloader_id_hardware_id_response = 0x00E, can_bootloader_id_hardware_id_request = 0x00F,
//                can_bootloader_id_bootloader_version_response = 0x014, can_bootloader_id_bootloader_version_request = 0x015,
//                can_bootloader_id_address_deny_response = 0x040, can_bootloader_id_address_accept_response = 0x041,
//                can_bootloader_id_address_value_set = 0x042, can_bootloader_id_verify_deny_response = 0x046,
//                can_bootloader_id_verify_accept_response = 0x047, can_bootloader_id_verify_crc_request = 0x048,
//                can_bootloader_id_jump_deny_response = 0x04C, can_bootloader_id_jump_accept_response = 0x04D,
//                can_bootloader_id_jump_execute_set = 0x04E, can_bootloader_id_file_deny_response = 0x052,
//                can_bootloader_id_file_accept_response = 0x053, can_bootloader_id_file_concluded_response = 0x054,
//                can_bootloader_id_file_abort_set = 0x055, can_bootloader_id_file_length_set = 0x056,
//                can_bootloader_id_file_data_set = 0x057 } can_bootloader_id_e;

// const uint32_t hardware_id = 0x83B40001;
// const uint32_t bootloader_version = 0x01010000;
