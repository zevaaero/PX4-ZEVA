/**
* Enable OSD display
*
* Set to display OSD overlay on video.
*
* @value 0 Disabled
* @value 1 Enabled
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_ENABLE, 1);

/**
* Set video type
*
* Sets which video format the FPV camera analog display is
*
* @value 0 PAL
* @value 1 NTSC
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_PAL_NTSC, 1);

/**
* OSD horizontal offset
*
* changes horizontal offset
*
* @min 0
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_HOR_OS, 32);

/**
* OSD vertical offset
*
* changes vertical offset
*
* @min 0
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_VER_OS, 16);

/**
* OSD left margin 
*
* changes left display margin 
*
* @min 0
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_LEFT_OS, 1);


/**
* OSD right margin
right
* changes left display margin
*
* @min 0
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_RIGHT_OS, 1);

/**
* OSD upper margin
*
* changes upper display margin
*
* @min 0
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_UPPER_OS, 1);

/**
* OSD lower margin
*
* changes lower display margin
*
* @min 0
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_LOWER_OS, 1);

/**
* Select display units
*
* Chooses which units to use, MKS or Imerial
*
* @value 0 MKS
* @value 1 IMPERIAL
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_UNITS, 0);


/**
* Show Height 
*
* Show height value
*
* @value 0 OFF
* @value 1 SMALL
* @value 2 LARGE
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_SHOW_HEIGHT, 1);


/**
* Show VARIO
*
* Show vario 
*
* @value 0 OFF
* @value 1 SMALL
* @value 2 LARGE
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_SHOW_VARIO, 1);

/**
* Show heading
*
* Show heading value
*
* @value 0 OFF
* @value 1 SMALL
* @value 2 LARGE
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_SHOW_HEADING, 2);

/**
* Show distance
*
* Show distance value
*
* @value 0 OFF
* @value 1 SMALL
* @value 2 LARGE
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_DISTANCE, 2);

/**
* Show Ground speed
*
* Show ground speed value
*
* @value 0 OFF
* @value 1 SMALL
* @value 2 LARGE
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_GROUNDSPEED, 1);
d

/**
* Show battery voltage
*
* Show battery voltage value
*
* @value 0 OFF
* @value 1 SMALL
* @value 2 LARGE
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_SHOW_BATTERY, 2);

/**
* Show flight clock
*
* Show flight clock
*
* @value 0 OFF
* @value 1 SMALL
* @value 2 LARGE
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_SHOW_TIME, 2);

/**
* Show GPS satellite count
*
* Show GPS satellite count
*
* @value 0 OFF
* @value 1 SMALL
* @value 2 LARGE
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_SHOW_GPS, 2);

/**
* Show GPS HACC
*
* Show GPS horizontal accuracy
*
* @value 0 OFF
* @value 1 ON
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_SHOW_GPSHACC, 1);

/**
* Show vario graphics
*
* Show vario iconography
*
* @value 0 OFF
* @value 1 ON
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_VARIOGFX, 1);

/**
* Show speed graphics
*
* Show speed iconography
*
* @value 0 OFF
* @value 1 ON
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_SPEEDGFX, 1);

/**
* Show sideslip graphics
*
* Show vario iconography
*
* @value 0 OFF
* @value 1 ON
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_SIDESLIPGFX, 1);

/**
* Show Attitude
*
* Show Attitude indicator
*
* @value 0 OFF
* @value 1 SMALL
* @value 2 LARGE
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_SHOW_ATT, 2);

/**
* Show heading arrow
*
* Show heading arrow
*
* @value 0 OFF
* @value 1 ON
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_HEADINGARROW, 1);

/**
* Show home arrow
*
* Show home arrow
*
* @value 0 OFF
* @value 1 ON
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_HOMEARROW, 1);

/**
* Show latitude and longitude
*
* Display position on screen
*
* @value 0 OFF
* @value 1 ON
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_SHOW_LATLON, 1);


/**
* Pitch scaling
*
* scale of pitch to motion on screen
*
* @min 1
* @max 200
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_PITCH_SCALE, 20);

/**
* Pitch interval
*
* interval for pitch markers
*
* @min 1
* @max 180
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_PITCH_IN, 10);


/**
* Roll Marker type
*
* Pick roll marker type
*
* @value 0 OFF
* @value 1 SMALL
* @value 2 LARGE
*
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_ROLL_MARKER, 0);


/**
* Roll scaling
*
* scaling for roll markers
*
* @min 1
* @max 200
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_ROLL_SCALING, 20);


/**
* Sky brightness
*
* brightness of top components
*
* @min 0
* @max 16
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_SKY_BRIGHT, 1);

/**
* Ground brightness
*
* brightness of bottom components
*
* @min 0
* @max 16
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_INT32(OSD_GND_BRIGHT, 1);

/**
* Battery alarm voltage
*
* cell voltage that triggers warning on screen
*
* @min 3.0
* @max 4.2
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_FLOAT(OSD_BAT_ALARM, 3.583f);

/**
* Range Alarm
*
* Beyond this range, range marker will flash to warn, meters
*
* @min 1.0
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_FLOAT(OSD_MAX_RANGE, 1000.0f);

/**
* Altitude Alarm
*
* Beyond this altitude, altitude marker will flash to warn, meters
*
* @min 1.0
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_FLOAT(OSD_MAX_ALTITUDE, 121.9f);

/**
* Velocity Alarm
*
* Beyond this speed, speed marker will flash to warn, m/s
*
* @min 1.0
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_FLOAT(OSD_MAX_VELOCITY, 20.0f);

/**
* Climb rate Alarm
*
* Beyond this climb rate, will flash to warn, m/s
*
* @min 0.0
* @reboot_required true
* @group OSD Display
*/
PARAM_DEFINE_FLOAT(OSD_MAX_CLIMB, 3.5f);

/**
* Boom LED brightness
*
* Overall brightness of all boom LEDs when armed
* 0-15, with 0 being dim and 15 being brightest
*
* @min 0

* @reboot_required false
* @group OSD Display
*/
PARAM_DEFINE_INT32(BOOM_BRT,10);

/**
* Boom 1 LED Color
*
* Color when disarmed
* 0 = OFF
* 1 = RED
* 2 = ORANGE
* 3 = YELLOW
* 4 = GREEN
* 5 = CYAN
* 6 = BLUE
* 7 = PURPLE
* 8 = WHITE
*
* @min 0
* @max 6
* @reboot_required false
* @group ALTA
*/
PARAM_DEFINE_INT32(BOOM1_COLOR,1);

/**
* Boom 2 LED Color
*
* Color when disarmed
* 0 = OFF
* 1 = RED
* 2 = ORANGE
* 3 = YELLOW
* 4 = GREEN
* 5 = CYAN
* 6 = BLUE
* 7 = PURPLE
* 8 = WHITE
*
* @min 0
* @max 6
* @reboot_required false
* @group ALTA
*/
PARAM_DEFINE_INT32(BOOM2_COLOR,0);

/**
* Boom 3 LED Color
*
* Color when disarmed
* 0 = OFF
* 1 = RED
* 2 = ORANGE
* 3 = YELLOW
* 4 = GREEN
* 5 = CYAN
* 6 = BLUE
* 7 = PURPLE
* 8 = WHITE
*
* @min 0
* @max 6
* @reboot_required false
* @group ALTA
*/
PARAM_DEFINE_INT32(BOOM3_COLOR,0);

/**
* Boom 4 LED Color
*
* Color when disarmed
* 0 = OFF
* 1 = RED
* 2 = ORANGE
* 3 = YELLOW
* 4 = GREEN
* 5 = CYAN
* 6 = BLUE
* 7 = PURPLE
* 8 = WHITE
*
* @min 0
* @max 6
* @reboot_required false
* @group ALTA
*/
PARAM_DEFINE_INT32(BOOM4_COLOR,4);

/**
* Boom 5 LED Color
*
* Color when disarmed
* 0 = OFF
* 1 = RED
* 2 = ORANGE
* 3 = YELLOW
* 4 = GREEN
* 5 = CYAN
* 6 = BLUE
* 7 = PURPLE
* 8 = WHITE
*
* @min 0
* @max 6
* @reboot_required false
* @group ALTA
*/
PARAM_DEFINE_INT32(BOOM5_COLOR,4);

/**
* Boom 6 LED Color
*
* Color when disarmed
* 0 = OFF
* 1 = RED
* 2 = ORANGE
* 3 = YELLOW
* 4 = GREEN
* 5 = CYAN
* 6 = BLUE
* 7 = PURPLE
* 8 = WHITE
*
* @min 0
* @max 6
* @reboot_required false
* @group ALTA
*/
PARAM_DEFINE_INT32(BOOM6_COLOR,0);

/**
* Boom 7 LED Color
*
* Color when disarmed
* 0 = OFF
* 1 = RED
* 2 = ORANGE
* 3 = YELLOW
* 4 = GREEN
* 5 = CYAN
* 6 = BLUE
* 7 = PURPLE
* 8 = WHITE
*
* @min 0
* @max 6
* @reboot_required false
* @group ALTA
*/
PARAM_DEFINE_INT32(BOOM7_COLOR,0);

/**
* Boom 8 LED Color
*
* Color when disarmed
* 0 = OFF
* 1 = RED
* 2 = ORANGE
* 3 = YELLOW
* 4 = GREEN
* 5 = CYAN
* 6 = BLUE
* 7 = PURPLE
* 8 = WHITE
*
* @min 0
* @max 6
* @reboot_required false
* @group ALTA
*/
PARAM_DEFINE_INT32(BOOM8_COLOR,1);
	BOOM_COLOR_OFF = 0,
	BOOM_COLOR_RED,
	BOOM_COLOR_ORANGE,
	BOOM_COLOR_YELLOW,
	BOOM_COLOR_GREEN,
	BOOM_COLOR_CYAN,
	BOOM_COLOR_BLUE,
	BOOM_COLOR_PURPLE,
	BOOM_COLOR_WHITE