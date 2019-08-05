#pragma once
// MESSAGE ESC_TELEMETRY_1 PACKING

#define MAVLINK_MSG_ID_ESC_TELEMETRY_1 50010

MAVPACKED(
typedef struct __mavlink_esc_telemetry_1_t {
	int16_t RPM[8]; /*<  RPM. Range: -32768 - 32767 */
	uint16_t Voltage[8]; /*< [cV] Voltage. Range: 0 - 65535. Range in Voltage: 0.0 - 655.3 V*/
	int16_t Current[8]; /*< [cA] Current. Range: -32768 - 32767. Range in Amps: -327.6 - 327.6 A*/
	uint8_t Temperature[8]; /*< [c] Temperature. Range: 0-255 degC */
	uint8_t errorCount[8]; /*<  Error flags from ESC. */
	uint8_t errorState[8]; /*<  Error state from ESC. */
}) mavlink_esc_telemetry_1_t;

#define MAVLINK_MSG_ID_ESC_TELEMETRY_1_LEN 72
#define MAVLINK_MSG_ID_ESC_TELEMETRY_1_MIN_LEN 72
#define MAVLINK_MSG_ID_50010_LEN 72
#define MAVLINK_MSG_ID_50010_MIN_LEN 72

#define MAVLINK_MSG_ID_ESC_TELEMETRY_1_CRC 165
#define MAVLINK_MSG_ID_50010_CRC 165

#define MAVLINK_MSG_ESC_TELEMETRY_1_FIELD_RPM_LEN 8
#define MAVLINK_MSG_ESC_TELEMETRY_1_FIELD_VOLTAGE_LEN 8
#define MAVLINK_MSG_ESC_TELEMETRY_1_FIELD_CURRENT_LEN 8
#define MAVLINK_MSG_ESC_TELEMETRY_1_FIELD_TEMPERATURE_LEN 8
#define MAVLINK_MSG_ESC_TELEMETRY_1_FIELD_ERRORCOUNT_LEN 8
#define MAVLINK_MSG_ESC_TELEMETRY_1_FIELD_ERRORSTATE_LEN 8

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ESC_TELEMETRY_1 { \
		50010, \
		"ESC_TELEMETRY_1", \
		6, \
		{  { "RPM", NULL, MAVLINK_TYPE_INT16_T, 8, 0, offsetof(mavlink_esc_telemetry_1_t, RPM) }, \
			{ "Voltage", NULL, MAVLINK_TYPE_UINT16_T, 8, 16, offsetof(mavlink_esc_telemetry_1_t, Voltage) }, \
			{ "Current", NULL, MAVLINK_TYPE_INT16_T, 8, 32, offsetof(mavlink_esc_telemetry_1_t, Current) }, \
			{ "Temperature", NULL, MAVLINK_TYPE_UINT8_T, 8, 48, offsetof(mavlink_esc_telemetry_1_t, Temperature) }, \
			{ "errorCount", NULL, MAVLINK_TYPE_UINT8_T, 8, 56, offsetof(mavlink_esc_telemetry_1_t, errorCount) }, \
			{ "errorState", NULL, MAVLINK_TYPE_UINT8_T, 8, 64, offsetof(mavlink_esc_telemetry_1_t, errorState) }, \
		} \
	}
#else
#define MAVLINK_MESSAGE_INFO_ESC_TELEMETRY_1 { \
		"ESC_TELEMETRY_1", \
		6, \
		{  { "RPM", NULL, MAVLINK_TYPE_INT16_T, 8, 0, offsetof(mavlink_esc_telemetry_1_t, RPM) }, \
			{ "Voltage", NULL, MAVLINK_TYPE_UINT16_T, 8, 16, offsetof(mavlink_esc_telemetry_1_t, Voltage) }, \
			{ "Current", NULL, MAVLINK_TYPE_INT16_T, 8, 32, offsetof(mavlink_esc_telemetry_1_t, Current) }, \
			{ "Temperature", NULL, MAVLINK_TYPE_UINT8_T, 8, 48, offsetof(mavlink_esc_telemetry_1_t, Temperature) }, \
			{ "errorCount", NULL, MAVLINK_TYPE_UINT8_T, 8, 56, offsetof(mavlink_esc_telemetry_1_t, errorCount) }, \
			{ "errorState", NULL, MAVLINK_TYPE_UINT8_T, 8, 64, offsetof(mavlink_esc_telemetry_1_t, errorState) }, \
		} \
	}
#endif

/**
 * @brief Pack a esc_telemetry_1 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param RPM  RPM. Range: -32768 - 32767
 * @param Voltage [cV] Voltage. Range: 0 - 65535. Range in Voltage: 0.0 - 655.3 V
 * @param Current [cA] Current. Range: -32768 - 32767. Range in Amps: -327.6 - 327.6 A
 * @param Temperature [c] Temperature. Range: 0-255 degC
 * @param errorCount  Error flags from ESC.
 * @param errorState  Error state from ESC.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_esc_telemetry_1_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t *msg,
		const int16_t *RPM, const uint16_t *Voltage, const int16_t *Current, const uint8_t *Temperature,
		const uint8_t *errorCount, const uint8_t *errorState)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ESC_TELEMETRY_1_LEN];

	_mav_put_int16_t_array(buf, 0, RPM, 8);
	_mav_put_uint16_t_array(buf, 16, Voltage, 8);
	_mav_put_int16_t_array(buf, 32, Current, 8);
	_mav_put_uint8_t_array(buf, 48, Temperature, 8);
	_mav_put_uint8_t_array(buf, 56, errorCount, 8);
	_mav_put_uint8_t_array(buf, 64, errorState, 8);
	memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ESC_TELEMETRY_1_LEN);
#else
	mavlink_esc_telemetry_1_t packet;

	mav_array_memcpy(packet.RPM, RPM, sizeof(int16_t) * 8);
	mav_array_memcpy(packet.Voltage, Voltage, sizeof(uint16_t) * 8);
	mav_array_memcpy(packet.Current, Current, sizeof(int16_t) * 8);
	mav_array_memcpy(packet.Temperature, Temperature, sizeof(uint8_t) * 8);
	mav_array_memcpy(packet.errorCount, errorCount, sizeof(uint8_t) * 8);
	mav_array_memcpy(packet.errorState, errorState, sizeof(uint8_t) * 8);
	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ESC_TELEMETRY_1_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ESC_TELEMETRY_1;
	return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ESC_TELEMETRY_1_MIN_LEN,
					MAVLINK_MSG_ID_ESC_TELEMETRY_1_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_CRC);
}

/**
 * @brief Pack a esc_telemetry_1 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param RPM  RPM. Range: -32768 - 32767
 * @param Voltage [cV] Voltage. Range: 0 - 65535. Range in Voltage: 0.0 - 655.3 V
 * @param Current [cA] Current. Range: -32768 - 32767. Range in Amps: -327.6 - 327.6 A
 * @param Temperature [c] Temperature. Range: 0-255 degC
 * @param errorCount  Error flags from ESC.
 * @param errorState  Error state from ESC.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_esc_telemetry_1_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
		mavlink_message_t *msg,
		const int16_t *RPM, const uint16_t *Voltage, const int16_t *Current, const uint8_t *Temperature,
		const uint8_t *errorCount, const uint8_t *errorState)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ESC_TELEMETRY_1_LEN];

	_mav_put_int16_t_array(buf, 0, RPM, 8);
	_mav_put_uint16_t_array(buf, 16, Voltage, 8);
	_mav_put_int16_t_array(buf, 32, Current, 8);
	_mav_put_uint8_t_array(buf, 48, Temperature, 8);
	_mav_put_uint8_t_array(buf, 56, errorCount, 8);
	_mav_put_uint8_t_array(buf, 64, errorState, 8);
	memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ESC_TELEMETRY_1_LEN);
#else
	mavlink_esc_telemetry_1_t packet;

	mav_array_memcpy(packet.RPM, RPM, sizeof(int16_t) * 8);
	mav_array_memcpy(packet.Voltage, Voltage, sizeof(uint16_t) * 8);
	mav_array_memcpy(packet.Current, Current, sizeof(int16_t) * 8);
	mav_array_memcpy(packet.Temperature, Temperature, sizeof(uint8_t) * 8);
	mav_array_memcpy(packet.errorCount, errorCount, sizeof(uint8_t) * 8);
	mav_array_memcpy(packet.errorState, errorState, sizeof(uint8_t) * 8);
	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ESC_TELEMETRY_1_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ESC_TELEMETRY_1;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ESC_TELEMETRY_1_MIN_LEN,
					     MAVLINK_MSG_ID_ESC_TELEMETRY_1_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_CRC);
}

/**
 * @brief Encode a esc_telemetry_1 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param esc_telemetry_1 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_esc_telemetry_1_encode(uint8_t system_id, uint8_t component_id,
		mavlink_message_t *msg, const mavlink_esc_telemetry_1_t *esc_telemetry_1)
{
	return mavlink_msg_esc_telemetry_1_pack(system_id, component_id, msg, esc_telemetry_1->RPM, esc_telemetry_1->Voltage,
						esc_telemetry_1->Current, esc_telemetry_1->Temperature, esc_telemetry_1->errorCount, esc_telemetry_1->errorState);
}

/**
 * @brief Encode a esc_telemetry_1 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param esc_telemetry_1 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_esc_telemetry_1_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
		mavlink_message_t *msg, const mavlink_esc_telemetry_1_t *esc_telemetry_1)
{
	return mavlink_msg_esc_telemetry_1_pack_chan(system_id, component_id, chan, msg, esc_telemetry_1->RPM,
			esc_telemetry_1->Voltage, esc_telemetry_1->Current, esc_telemetry_1->Temperature, esc_telemetry_1->errorCount,
			esc_telemetry_1->errorState);
}

/**
 * @brief Send a esc_telemetry_1 message
 * @param chan MAVLink channel to send the message
 *
 * @param RPM  RPM. Range: -32768 - 32767
 * @param Voltage [cV] Voltage. Range: 0 - 65535. Range in Voltage: 0.0 - 655.3 V
 * @param Current [cA] Current. Range: -32768 - 32767. Range in Amps: -327.6 - 327.6 A
 * @param Temperature [c] Temperature. Range: 0-255 degC
 * @param errorCount  Error flags from ESC.
 * @param errorState  Error state from ESC.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_esc_telemetry_1_send(mavlink_channel_t chan, const int16_t *RPM, const uint16_t *Voltage,
		const int16_t *Current, const uint8_t *Temperature, const uint8_t *errorCount, const uint8_t *errorState)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ESC_TELEMETRY_1_LEN];

	_mav_put_int16_t_array(buf, 0, RPM, 8);
	_mav_put_uint16_t_array(buf, 16, Voltage, 8);
	_mav_put_int16_t_array(buf, 32, Current, 8);
	_mav_put_uint8_t_array(buf, 48, Temperature, 8);
	_mav_put_uint8_t_array(buf, 56, errorCount, 8);
	_mav_put_uint8_t_array(buf, 64, errorState, 8);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_TELEMETRY_1, buf, MAVLINK_MSG_ID_ESC_TELEMETRY_1_MIN_LEN,
					MAVLINK_MSG_ID_ESC_TELEMETRY_1_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_CRC);
#else
	mavlink_esc_telemetry_1_t packet;

	mav_array_memcpy(packet.RPM, RPM, sizeof(int16_t) * 8);
	mav_array_memcpy(packet.Voltage, Voltage, sizeof(uint16_t) * 8);
	mav_array_memcpy(packet.Current, Current, sizeof(int16_t) * 8);
	mav_array_memcpy(packet.Temperature, Temperature, sizeof(uint8_t) * 8);
	mav_array_memcpy(packet.errorCount, errorCount, sizeof(uint8_t) * 8);
	mav_array_memcpy(packet.errorState, errorState, sizeof(uint8_t) * 8);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_TELEMETRY_1, (const char *)&packet,
					MAVLINK_MSG_ID_ESC_TELEMETRY_1_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_CRC);
#endif
}

/**
 * @brief Send a esc_telemetry_1 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_esc_telemetry_1_send_struct(mavlink_channel_t chan,
		const mavlink_esc_telemetry_1_t *esc_telemetry_1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	mavlink_msg_esc_telemetry_1_send(chan, esc_telemetry_1->RPM, esc_telemetry_1->Voltage, esc_telemetry_1->Current,
					 esc_telemetry_1->Temperature, esc_telemetry_1->errorCount, esc_telemetry_1->errorState);
#else
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_TELEMETRY_1, (const char *)esc_telemetry_1,
					MAVLINK_MSG_ID_ESC_TELEMETRY_1_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_CRC);
#endif
}

#if MAVLINK_MSG_ID_ESC_TELEMETRY_1_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_esc_telemetry_1_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,
		const int16_t *RPM, const uint16_t *Voltage, const int16_t *Current, const uint8_t *Temperature,
		const uint8_t *errorCount, const uint8_t *errorState)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;

	_mav_put_int16_t_array(buf, 0, RPM, 8);
	_mav_put_uint16_t_array(buf, 16, Voltage, 8);
	_mav_put_int16_t_array(buf, 32, Current, 8);
	_mav_put_uint8_t_array(buf, 48, Temperature, 8);
	_mav_put_uint8_t_array(buf, 56, errorCount, 8);
	_mav_put_uint8_t_array(buf, 64, errorState, 8);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_TELEMETRY_1, buf, MAVLINK_MSG_ID_ESC_TELEMETRY_1_MIN_LEN,
					MAVLINK_MSG_ID_ESC_TELEMETRY_1_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_CRC);
#else
	mavlink_esc_telemetry_1_t *packet = (mavlink_esc_telemetry_1_t *)msgbuf;

	mav_array_memcpy(packet->RPM, RPM, sizeof(int16_t) * 8);
	mav_array_memcpy(packet->Voltage, Voltage, sizeof(uint16_t) * 8);
	mav_array_memcpy(packet->Current, Current, sizeof(int16_t) * 8);
	mav_array_memcpy(packet->Temperature, Temperature, sizeof(uint8_t) * 8);
	mav_array_memcpy(packet->errorCount, errorCount, sizeof(uint8_t) * 8);
	mav_array_memcpy(packet->errorState, errorState, sizeof(uint8_t) * 8);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ESC_TELEMETRY_1, (const char *)packet,
					MAVLINK_MSG_ID_ESC_TELEMETRY_1_MIN_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_LEN, MAVLINK_MSG_ID_ESC_TELEMETRY_1_CRC);
#endif
}
#endif

#endif

// MESSAGE ESC_TELEMETRY_1 UNPACKING


/**
 * @brief Get field RPM from esc_telemetry_1 message
 *
 * @return  RPM. Range: -32768 - 32767
 */
static inline uint16_t mavlink_msg_esc_telemetry_1_get_RPM(const mavlink_message_t *msg, int16_t *RPM)
{
	return _MAV_RETURN_int16_t_array(msg, RPM, 8,  0);
}

/**
 * @brief Get field Voltage from esc_telemetry_1 message
 *
 * @return [cV] Voltage. Range: 0 - 65535. Range in Voltage: 0.0 - 655.3 V
 */
static inline uint16_t mavlink_msg_esc_telemetry_1_get_Voltage(const mavlink_message_t *msg, uint16_t *Voltage)
{
	return _MAV_RETURN_uint16_t_array(msg, Voltage, 8,  16);
}

/**
 * @brief Get field Current from esc_telemetry_1 message
 *
 * @return [cA] Current. Range: -32768 - 32767. Range in Amps: -327.6 - 327.6 A
 */
static inline uint16_t mavlink_msg_esc_telemetry_1_get_Current(const mavlink_message_t *msg, int16_t *Current)
{
	return _MAV_RETURN_int16_t_array(msg, Current, 8,  32);
}

/**
 * @brief Get field Temperature from esc_telemetry_1 message
 *
 * @return [c] Temperature. Range: 0-255 degC
 */
static inline uint16_t mavlink_msg_esc_telemetry_1_get_Temperature(const mavlink_message_t *msg, uint8_t *Temperature)
{
	return _MAV_RETURN_uint8_t_array(msg, Temperature, 8,  48);
}

/**
 * @brief Get field errorCount from esc_telemetry_1 message
 *
 * @return  Error flags from ESC.
 */
static inline uint16_t mavlink_msg_esc_telemetry_1_get_errorCount(const mavlink_message_t *msg, uint8_t *errorCount)
{
	return _MAV_RETURN_uint8_t_array(msg, errorCount, 8,  56);
}

/**
 * @brief Get field errorState from esc_telemetry_1 message
 *
 * @return  Error state from ESC.
 */
static inline uint16_t mavlink_msg_esc_telemetry_1_get_errorState(const mavlink_message_t *msg, uint8_t *errorState)
{
	return _MAV_RETURN_uint8_t_array(msg, errorState, 8,  64);
}

/**
 * @brief Decode a esc_telemetry_1 message into a struct
 *
 * @param msg The message to decode
 * @param esc_telemetry_1 C-struct to decode the message contents into
 */
static inline void mavlink_msg_esc_telemetry_1_decode(const mavlink_message_t *msg,
		mavlink_esc_telemetry_1_t *esc_telemetry_1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	mavlink_msg_esc_telemetry_1_get_RPM(msg, esc_telemetry_1->RPM);
	mavlink_msg_esc_telemetry_1_get_Voltage(msg, esc_telemetry_1->Voltage);
	mavlink_msg_esc_telemetry_1_get_Current(msg, esc_telemetry_1->Current);
	mavlink_msg_esc_telemetry_1_get_Temperature(msg, esc_telemetry_1->Temperature);
	mavlink_msg_esc_telemetry_1_get_errorCount(msg, esc_telemetry_1->errorCount);
	mavlink_msg_esc_telemetry_1_get_errorState(msg, esc_telemetry_1->errorState);
#else
	uint8_t len = msg->len < MAVLINK_MSG_ID_ESC_TELEMETRY_1_LEN ? msg->len : MAVLINK_MSG_ID_ESC_TELEMETRY_1_LEN;
	memset(esc_telemetry_1, 0, MAVLINK_MSG_ID_ESC_TELEMETRY_1_LEN);
	memcpy(esc_telemetry_1, _MAV_PAYLOAD(msg), len);
#endif
}
