/*
 * File:		UAVTalk.hpp
 * Author:		Alex Liao
 * Desc:		Implements UAVTalk telemetry communication with OpenPilot
 *				and the Pillar GCS
 * Etc:			Code based on code written by Joerg-D. Rothfuchs
*/

class UAVTalk
{

	public:

		// Read from the serial stream
		int uavtalk_read(void);


		int uavtalk_state(void);

	private:
		// Structure for the UAVTalk message
		typedef struct __uavtalk_message {
			uint8_t Sync;
			uint8_t MsgType;
			uint16_t Length;
			uint32_t ObjID;
			uint16_t InstID;
			uint8_t Data[255];
			uint8_t Crc;
		} uavtalk_message_t;

		// Enumeration options for field FlightMode 
		// From build/uavobject-synthetics/flight/flightstatus.h
		typedef enum { 
		  FLIGHTSTATUS_FLIGHTMODE_MANUAL=0, 
		  FLIGHTSTATUS_FLIGHTMODE_STABILIZED1=1, 
		  FLIGHTSTATUS_FLIGHTMODE_STABILIZED2=2, 
		  FLIGHTSTATUS_FLIGHTMODE_STABILIZED3=3, 
		  FLIGHTSTATUS_FLIGHTMODE_AUTOTUNE=4, 
		  FLIGHTSTATUS_FLIGHTMODE_ALTITUDEHOLD=5, 
		  FLIGHTSTATUS_FLIGHTMODE_VELOCITYCONTROL=6, 
		  FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD=7,
		  FLIGHTSTATUS_FLIGHTMODE_RETURNTOHOME=8,
		  FLIGHTSTATUS_FLIGHTMODE_PATHPLANNER=9
		} FlightStatusFlightModeOptions;

		// Enumeration options for field ControlSource 
		// From build/uavobject-synthetics/flight/flightstatus.h
		typedef enum { 
		  FLIGHTSTATUS_CONTROLSOURCE_GEOFENCE=0, 
		  FLIGHTSTATUS_CONTROLSOURCE_FAILSAFE=1, 
		  FLIGHTSTATUS_CONTROLSOURCE_TRANSMITTER=2, 
		  FLIGHTSTATUS_CONTROLSOURCE_TABLET=3 
		} FlightStatusControlSourceOptions;

		typedef enum {
			UAVTALK_PARSE_STATE_WAIT_SYNC = 0,
			UAVTALK_PARSE_STATE_GOT_SYNC,
			UAVTALK_PARSE_STATE_GOT_MSG_TYPE,
			UAVTALK_PARSE_STATE_GOT_LENGTH,
			UAVTALK_PARSE_STATE_GOT_OBJID,
			UAVTALK_PARSE_STATE_GOT_INSTID,
			UAVTALK_PARSE_STATE_GOT_DATA,
			UAVTALK_PARSE_STATE_GOT_CRC
		} uavtalk_parse_state_t;

		typedef enum {
			TELEMETRYSTATS_STATE_DISCONNECTED = 0,
			TELEMETRYSTATS_STATE_HANDSHAKEREQ,
			TELEMETRYSTATS_STATE_HANDSHAKEACK,
			TELEMETRYSTATS_STATE_CONNECTED
		} telemetrystats_state_t;

}
