/*!
	\file SIM68MD.h
	\brief Driver for controlling SIM68MD GPS module via UART interface.
	\authors Bliznets R.A. (r.bliznets@gmail.com)
	\version 1.5.0.0
	\date November 16, 2023
	\warning Requires ESP-IDF with FreeRTOS support and hardware UART
*/

#pragma once

#include "sdkconfig.h"
#include "CBaseTask.h"

#include "esp_pm.h"
#include "driver/uart.h"
#include <ctime>

#include "nmea.h"
#include "gpgga.h"
#include "gprmc.h"
#include "gpgsa.h"

// UART configuration parameters
#define GPS_TX_BUF (128 + 16)		   ///< UART transmit buffer size (bytes)
#define GPS_NMEA_BUF 256			   ///< Maximum NMEA string length (bytes)
#define GPS_RX_BUF (GPS_NMEA_BUF + 64) ///< UART receive buffer size (bytes)
#define GPS_EVEN_BUF 10				   ///< UART event queue depth
#define GPS_INAVALID_DELAY 3		   ///< Wait in sec for valid data from GPS	   

// GPS task control commands
#define MSG_GPS_ON 10  ///< Activate GPS with search parameters
#define MSG_GPS_OFF 11 ///< Deactivate GPS with mode selection

// GPS processing task parameters
#define GPSTASK_NAME "gps"				   ///< Task identifier for debugging
#define GPSTASK_STACKSIZE (2 * 1024 + 512) ///< Task stack size (bytes)
#define GPSTASK_LENGTH 10				   ///< Message queue capacity

/**
 * @brief GPS module operating modes
 */
enum class EGPSMode
{
	Unknown = 0, ///< Uninitialized state
	Run = 1,	 ///< Active mode with data transmission
	Sleep = 2,	 ///< Power saving mode 
	RTC = 3		 ///< Deep sleep with only real-time clock powered
};

/**
 * @brief Geographic coordinates in NMEA format
 */
struct SPosition
{
	int16_t degrees; ///< Integer part of coordinate in degrees
	float minutes;	 ///< Fractional part in minutes (0.0′ - 59.9999′)
	char cardinal;	 ///< Direction: 'N', 'S', 'E' or 'W'
};

/**
 * @brief Data received from GPS module
 */
struct SGPSData
{
	uint8_t valid;		 ///< Positioning status (0-invalid)
	uint8_t satellites;	 ///< Number of captured satellites (0-12)
	time_t time;		 ///< Time in UNIX format (UTC)
	SPosition longitude; ///< Longitude (from -180° to +180°)
	SPosition latitude;	 ///< Latitude (from -90° to +90°)
	float altitude;		 ///< Altitude above sea level (meters)
	float hdop;			 ///< Horizontal positioning error
};

/**
 * @brief Callback function for processing new GPS data
 * @param[out] gps  Pointer to structure with updated data
 * @param[in]  mode Current module operating mode (from EGPSMode)
 */
typedef void onGPS(SGPSData *gps, EGPSMode mode);

class SIM68MD;

/**
 * @brief Callback function when GPS data is missing
 * @param[in] device Pointer to SIM68MD driver instance
 */
typedef void onGPSFailed(SIM68MD *device);

/**
 * @brief Callback function when GPS return from sleep/rts mode
 * @param[in] eint_in eint_in pin
 * @param[in] eint0 eint0 pin
 */
typedef void onGPSSleep(bool eint_in, bool eint0);

/**
 * @brief GPS module driver configuration
 */
struct SGPSConfig
{
	onGPS *onDataRx = nullptr;		 ///< New data handler (called in task context)
	onGPSFailed *onFailed = nullptr; ///< Communication error handler
	onGPSSleep *onSleep = nullptr;   ///< External sleep pins control (expander for example)

	uint8_t cpu = 1;   ///< CPU core number (0 or 1)
	uint8_t prior = 2; ///< Task priority (0-25, where 25 is highest)

	uart_port_t port = UART_NUM_1; ///< Used UART port
	int baudrate = 115200;		   ///< Communication speed (bits/sec)

	int8_t pin_tx = 17;		 ///< GPIO for TX (UART)
	int8_t pin_rx = 18;		 ///< GPIO for RX (UART)
	int8_t pin_eint_in = 39; ///< GPIO for power control (active LOW)
	int8_t pin_eint0 = 42;	 ///< GPIO for wake interrupt
};

/**
 * @brief Class for working with SIM68MD (Singleton pattern implementation)
 */
class SIM68MD : public CBaseTask
{
protected:
	/** @brief Single class instance (Singleton pattern) */
	static SIM68MD *theSingleInstance;

	/** @brief Flag to track if this is the first start of the module */
	static bool firstStart;
	/** @brief Counter for invalid data delay */
	uint16_t mDelayNonValid = GPS_INAVALID_DELAY;

#if CONFIG_PM_ENABLE
	/** @brief CPU frequency reduction lock for UART operations */
	esp_pm_lock_handle_t mPMLock;
#endif

	/** @brief Configuration parameters for the GPS module */
	SGPSConfig mConfig;
	/** @brief Current GPS data received from the module */
	SGPSData mData;
	/** @brief Time structure used during parsing operations */
	std::tm mTime;
	/** @brief Flag indicating if an event needs to be sent to subscribers */
	bool mEventSend = false;
	/** @brief Flag indicating if GPS fix status has changed */
	bool mFixChanged = false;

	/** @brief Event queue set for managing multiple queues */
	QueueSetHandle_t mQueueSet;
	/** @brief UART event queue handle */
	QueueHandle_t m_uart_queue = nullptr;
	/** @brief Buffer for storing raw NMEA data received from the GPS module */
	char mBuf[GPS_NMEA_BUF];

	/** @brief Current operating mode of the GPS module */
	EGPSMode mRun = EGPSMode::Unknown;
	/** @brief Wait timer for timeout handling (in seconds) */
	uint32_t mWaitTime = 0;
	/** @brief Maximum satellite search time allowed */
	uint32_t mSearchTime = 0;
	/** @brief Start time for tracking search duration */
	time_t mStart_time;

	/** @brief Counter for successful NMEA parsing operations */
	int16_t mCount = 0;

	/**
	 * @brief Initialize UART and activate GPS module
	 * Sets up UART communication parameters and activates the GPS module
	 */
	void initUart();

	/**
	 * @brief Deactivate UART with power saving mode selection
	 * @param[in] rtc true - switch to RTC mode, false - normal sleep
	 * Properly shuts down UART communication and puts GPS module in selected power saving mode
	 */
	void deinitUart(bool rtc = false);

	/**
	 * @brief Main task function (inherited from CBaseTask)
	 * Contains the main processing loop that handles GPS data and commands
	 */
	virtual void run() override;

	/**
	 * @brief Parse NMEA string and update GPS data
	 * @param[in] start  Pointer to string start
	 * @param[in] length String length (bytes)
	 * @return true - data successfully updated, false - parsing failed
	 * Processes NMEA sentences and updates the internal GPS data structure
	 */
	bool gps_decode(char *start, size_t length);

	/**
	 * @brief Private constructor (Singleton pattern)
	 * @param[in] cfg GPS configuration structure
	 * Creates and initializes a new GPS module instance
	 */
	explicit SIM68MD(SGPSConfig *cfg);

	/**
	 * @brief Destructor with resource release
	 * Properly cleans up all allocated resources and stops GPS operations
	 */
	virtual ~SIM68MD();

	/** @brief Allow access to base class sendCmd method */
	using CBaseTask::sendCmd;

public:
	/**
	 * @brief Get class instance
	 * @return Pointer to the singleton instance
	 * Returns the single instance of the SIM68MD class
	 */
	static SIM68MD *Instance() { return theSingleInstance; }

	/**
	 * @brief Initialize driver
	 * @param[in] cfg Module configuration
	 * @return Pointer to created instance
	 * Creates and initializes the GPS driver instance with the given configuration
	 */
	static SIM68MD *init(SGPSConfig *cfg);

	/**
	 * @brief Deinitialize driver
	 * Stops GPS operations and frees all allocated resources
	 */
	static void free();

	/**
	 * @brief Check driver activity
	 * @return true if driver is running, false otherwise
	 * Checks if the GPS driver instance exists and is active
	 */
	static inline bool isRun() { return (theSingleInstance != nullptr); }

	/**
	 * @brief Start GPS with search timeout
	 * @param[in] search_time Max satellite search time (0 - no limit)
	 * @return true - command accepted to queue, false - command failed
	 * Activates the GPS module with optional search time limit
	 */
	inline bool start(uint32_t search_time = 0)
	{
		return sendCmd(MSG_GPS_ON, 0, search_time);
	}

	/**
	 * @brief Stop GPS with mode selection
	 * @param[in] rtc_mode true - deep sleep (RTC), false - normal
	 * @param[in] wake_after Time until auto-wake (0 - manual start)
	 * @return true - command accepted to queue, false - command failed
	 * Deactivates the GPS module and puts it in selected power saving mode
	 */
	inline bool stop(bool rtc_mode = false, uint32_t wake_after = 0)
	{
		return sendCmd(MSG_GPS_OFF, rtc_mode, wake_after);
	}

	/**
	 * @brief Get current GPS state
	 * @return Current EGPSMode state
	 * Returns the current operating mode of the GPS module
	 */
	inline EGPSMode getState(){return mRun;};
};