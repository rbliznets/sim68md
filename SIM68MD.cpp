/*!
	\file
	\brief Класс управления SIM68MD через UART.
	\authors Близнец Р.А. (r.bliznets@gmail.com)
	\version 1.5.0.0
	\date 16.11.2023
*/

#include "SIM68MD.h"
#include "CDateTimeSystem.h"
#include "CTrace.h"
#include "driver/gpio.h"
#include "esp_private/esp_gpio_reserve.h"
#include <cstring>
#include <sys/time.h>

// Task timeout setting for Watchdog Timer compatibility
#ifdef CONFIG_ESP_TASK_WDT
#if CONFIG_ESP_TASK_WDT_TIMEOUT_S > 1
#define TASK_MAX_BLOCK_TIME pdMS_TO_TICKS(1000)
#else
#define TASK_MAX_BLOCK_TIME pdMS_TO_TICKS((CONFIG_ESP_TASK_WDT_TIMEOUT_S - 1) * 1000 + 500)
#endif
#else
#define TASK_MAX_BLOCK_TIME portMAX_DELAY ///< Maximum task block time
#endif

// Tag for GPS event logging
static const char *TAG = "gps";
// Commands for controlling the SIM68MD module
#ifdef CONFIG_SIM68MD_PD_1
static const char *cmd_on = "$PAIR002*38\r\n"; // Power on command
static const char *cmd_hot_on = "$PAIR004*3E\r\n";
static const char *cmd_off = "$PAIR003*39\r\n";	  // Command to enter Standby mode
static const char *cmd_rtc = "$PAIR650,0*25\r\n"; // Command to enter Backup mode
static const char *cmd_auto_saving_enable = "$PAIR490,1*2A\r\n$PAIR510,1*23\r\n";
#else
static const char *cmd_on = "$PAIR002*38\r\n"; // Power on command
static const char *cmd_hot_on = "$PMTK101*32\r\n";
static const char *cmd_off = "$PMTK161,1*29\r\n";				   // Command to enter Standby mode
static const char *cmd_rtc = "$PMTK161,0*28\r\n";				   // Command to enter Backup mode
static const char *cmd_on1 = "$PMTK225,0*2B\r\n$PMTK225,8*23\r\n"; // Command to enter AlwaysLocate Standby mode
static const char *cmd_on2 = "$PMTK225,0*2B\r\n$PMTK225,9*22\r\n"; // Command to enter AlwaysLocate Backup mode
#endif

bool SIM68MD::firstStart = false;

// Singleton pattern - single instance of the class
SIM68MD *SIM68MD::theSingleInstance = nullptr;

/*!
	\brief Initialize single class instance
	\param cfg Pointer to GPS configuration
	\return Pointer to created instance
*/
SIM68MD *SIM68MD::init(SGPSConfig *cfg)
{
	if (theSingleInstance == nullptr)
	{
		// Create instance and initialize base task class
		theSingleInstance = new SIM68MD(cfg);
		theSingleInstance->CBaseTask::init(GPSTASK_NAME, GPSTASK_STACKSIZE, cfg->prior, GPSTASK_LENGTH, cfg->cpu);
		// Add task queue to queue set
		xQueueAddToSet(theSingleInstance->mTaskQueue, theSingleInstance->mQueueSet);
		// vTaskDelay(2);
	}
	return theSingleInstance;
}

/*!
	\brief Free resources and delete instance
*/
void SIM68MD::free()
{
	if (theSingleInstance != nullptr)
	{
		// Send task completion command
		theSingleInstance->sendCmd(MSG_END_TASK);
		// Wait for task completion
		do
		{
			vTaskDelay(1);
		}
#if (INCLUDE_vTaskDelete == 1)
		while (theSingleInstance->mTaskHandle != nullptr);
#else
		while (theSingleInstance->mTaskQueue != nullptr);
#endif
		vTaskDelay(1);
		// Delete instance
		delete theSingleInstance;
		theSingleInstance = nullptr;
	}
}

/*!
	\brief Class constructor
	\param cfg Pointer to GPS configuration
*/
SIM68MD::SIM68MD(SGPSConfig *cfg) : CBaseTask()
{
	// Copy configuration
	std::memcpy(&mConfig, cfg, sizeof(SGPSConfig));

	// Configure GPIO interrupt pins if specified
	if ((mConfig.pin_eint_in >= 0) && (mConfig.pin_eint0 >= 0))
	{
		gpio_config_t io_conf = {BIT64(mConfig.pin_eint_in) | BIT64(mConfig.pin_eint0), GPIO_MODE_OUTPUT, GPIO_PULLUP_ENABLE, GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE};
		gpio_config(&io_conf);
		gpio_set_level((gpio_num_t)mConfig.pin_eint_in, 1);
		gpio_set_level((gpio_num_t)mConfig.pin_eint0, 0);
	}
	else if (mConfig.onSleep != nullptr)
	{
		mConfig.onSleep(1, 0);
	}

	// Initialize data structures
	std::memset(&mData, 0, sizeof(SGPSData));
	mQueueSet = xQueueCreateSet(GPSTASK_LENGTH + GPS_EVEN_BUF);

#if CONFIG_PM_ENABLE
	// Create power management lock
	esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "gps", &mPMLock);
#endif
}

/*!
	\brief Class destructor
*/
SIM68MD::~SIM68MD()
{
	// Free queue and GPIO resources
	vQueueDelete(mQueueSet);
#if CONFIG_PM_ENABLE
	esp_pm_lock_delete(mPMLock);
#endif

	if ((mConfig.pin_eint_in >= 0) && (mConfig.pin_eint0 >= 0))
	{
		gpio_config_t io_conf = {BIT64(mConfig.pin_eint_in) | BIT64(mConfig.pin_eint0), GPIO_MODE_DISABLE, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE};
		gpio_config(&io_conf);
	}
	else if (mConfig.onSleep != nullptr)
	{
		mConfig.onSleep(1, 1);
	}
}

/*!
	\brief Initialize UART interface
*/
void SIM68MD::initUart()
{
	if (mRun != EGPSMode::Run)
	{
		// Configure UART parameters
		uart_config_t uart_config = {
			.baud_rate = mConfig.baudrate,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.rx_flow_ctrl_thresh = 0,
			.source_clk = UART_SCLK_DEFAULT,
			.flags = {0, 0}};
		int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
		intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

#if CONFIG_PM_ENABLE
		esp_pm_lock_acquire(mPMLock);
#endif
		// Install UART driver
		ESP_ERROR_CHECK(uart_driver_install(mConfig.port, GPS_RX_BUF, GPS_TX_BUF, GPS_EVEN_BUF, &m_uart_queue, intr_alloc_flags));
		xQueueAddToSet(m_uart_queue, mQueueSet);
		ESP_ERROR_CHECK(uart_param_config(mConfig.port, &uart_config));
		ESP_ERROR_CHECK(uart_set_pin(mConfig.port, mConfig.pin_tx, mConfig.pin_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

		// Configure end-of-line detection
		ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(mConfig.port, '\n', 1, 5, 0, 0));
		ESP_ERROR_CHECK(uart_pattern_queue_reset(mConfig.port, GPS_EVEN_BUF));
		ESP_ERROR_CHECK(uart_flush(mConfig.port));

		// Activate module via GPIO
		if ((mConfig.pin_eint_in >= 0) && (mConfig.pin_eint0 >= 0))
		{
			if ((mRun == EGPSMode::RTC) || (mRun == EGPSMode::Unknown))
			{
				gpio_set_level((gpio_num_t)mConfig.pin_eint0, 1);
				vTaskDelay(pdMS_TO_TICKS(15));
				gpio_set_level((gpio_num_t)mConfig.pin_eint0, 0);
			}
			if ((mRun == EGPSMode::Sleep) || (mRun == EGPSMode::Unknown))
			{
				gpio_set_level((gpio_num_t)mConfig.pin_eint_in, 0);
				vTaskDelay(pdMS_TO_TICKS(15));
				gpio_set_level((gpio_num_t)mConfig.pin_eint_in, 1);
#ifdef CONFIG_SIM68MD_PD_1
				if (mRun == EGPSMode::Sleep)
				{
					uart_write_bytes(mConfig.port, cmd_on, strlen(cmd_on));
					ESP_LOGD(TAG, "send %s", cmd_on);
					uart_wait_tx_done(mConfig.port, 50);
				}
#endif
			}
			if ((mRun == EGPSMode::RTC) || (mRun == EGPSMode::Unknown))
			{
				// Send power on command
				uart_write_bytes(mConfig.port, cmd_on, strlen(cmd_on));
				ESP_LOGD(TAG, "send %s", cmd_on);
				uart_wait_tx_done(mConfig.port, 50);
			}
		}
		else if (mConfig.onSleep != nullptr)
		{
			if ((mRun == EGPSMode::RTC) || (mRun == EGPSMode::Unknown))
			{
				mConfig.onSleep(1, 1);
				vTaskDelay(pdMS_TO_TICKS(15));
				mConfig.onSleep(1, 0);
			}
			if ((mRun == EGPSMode::Sleep) || (mRun == EGPSMode::Unknown))
			{
				mConfig.onSleep(0, 0);
				vTaskDelay(pdMS_TO_TICKS(15));
				mConfig.onSleep(1, 0);
			}
			if ((mRun == EGPSMode::RTC) || (mRun == EGPSMode::Unknown))
			{
				// Send power on command
				uart_write_bytes(mConfig.port, cmd_on, strlen(cmd_on));
				ESP_LOGD(TAG, "send %s", cmd_on);
				uart_wait_tx_done(mConfig.port, 50);
			}
		}
		else
		{
			// Send power on command
			uart_write_bytes(mConfig.port, cmd_on, strlen(cmd_on));
			ESP_LOGD(TAG, "send %s", cmd_on);
			uart_wait_tx_done(mConfig.port, 50);
		}
		mRun = EGPSMode::Run;
		if (firstStart)
		{
			uart_write_bytes(mConfig.port, cmd_hot_on, strlen(cmd_on));
			ESP_LOGW(TAG, "send %s", cmd_hot_on);
			firstStart = false;
		}
		// Reset data and flags
		mEventSend = false;
		ESP_LOGI(TAG, "Run");
	}
}

/*!
	\brief Deinitialize UART
	\param rtc Flag for RTC mode transition
*/
void SIM68MD::deinitUart(bool rtc)
{
	if (mRun == EGPSMode::Run)
	{
		xQueueRemoveFromSet(m_uart_queue, mQueueSet);
		// Wait for transmission completion
		ESP_ERROR_CHECK(uart_wait_tx_done(mConfig.port, pdMS_TO_TICKS(250)));
		if (rtc)
		{
			// Send RTC command
			uart_write_bytes(mConfig.port, cmd_rtc, strlen(cmd_rtc));
			ESP_LOGD(TAG, "send %s", cmd_rtc);
			mRun = EGPSMode::RTC;
			if (((mConfig.pin_eint_in >= 0) && (mConfig.pin_eint0 >= 0)) || (mConfig.onSleep != nullptr))
				ESP_LOGI(TAG, "RTC");
			else
				ESP_LOGW(TAG, "RTC without exit");
		}
		else
		{
			// Send power off command
			uart_write_bytes(mConfig.port, cmd_off, strlen(cmd_off));
			ESP_LOGD(TAG, "send %s", cmd_off);
			mRun = EGPSMode::Sleep;
			ESP_LOGI(TAG, "Sleep");
		}
		// Complete UART operation
		ESP_ERROR_CHECK(uart_wait_tx_done(mConfig.port, pdMS_TO_TICKS(150)));
		ESP_ERROR_CHECK(uart_driver_delete(mConfig.port));
		m_uart_queue = nullptr;
		esp_gpio_revoke(BIT64(mConfig.pin_tx) | BIT64(mConfig.pin_rx));
		vTaskDelay(pdMS_TO_TICKS(1));
#if CONFIG_PM_ENABLE
		esp_pm_lock_release(mPMLock);
#endif
	}
}

/*!
	\brief Main task processing loop
*/
void SIM68MD::run()
{
#ifndef CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE
	UBaseType_t m1 = uxTaskGetStackHighWaterMark2(nullptr);
#endif
	STaskMessage msg;
	QueueSetMemberHandle_t xActivatedMember;
	time_t count_time = 0;

	vTaskDelay(pdMS_TO_TICKS(125));

	for (;;)
	{
		// Handle search timeouts
		if (mWaitTime != 0)
		{
			time_t now;
			time(&now);
			if ((now - mStart_time) > mWaitTime)
			{
				mWaitTime = 0;
				if (mRun == EGPSMode::Run)
				{
					// Enter RTC mode when time expires
					if (((mConfig.pin_eint_in >= 0) && (mConfig.pin_eint0 >= 0)) || (mConfig.onSleep != nullptr))
						if (mData.valid > 0)
							deinitUart(false);
						else
							deinitUart(true);
					else
						deinitUart(false);
					if (mConfig.onDataRx != nullptr)
						mConfig.onDataRx(&mData, mRun);
				}
				else
				{
					// Reactivate search
					initUart();
					if (mSearchTime > 0)
					{
						time(&mStart_time);
						mWaitTime = mSearchTime;
					}
				}
			}
		}
		// Process events from queues
		if ((xActivatedMember = xQueueSelectFromSet(mQueueSet, TASK_MAX_BLOCK_TIME)) != nullptr)
		{
			// Process UART events
			if ((mRun == EGPSMode::Run) && (xActivatedMember == m_uart_queue))
			{
				uart_event_t event;
				int pos, read_len;
				while ((m_uart_queue != nullptr) && (xQueueReceive(m_uart_queue, &event, 0) == pdTRUE))
				{
					switch (event.type)
					{
					case UART_DATA:
						break;
					case UART_FIFO_OVF:
						ESP_LOGW(TAG, "HW FIFO Overflow");
						uart_flush(mConfig.port);
						xQueueReset(m_uart_queue);
						break;
					case UART_BUFFER_FULL:
						ESP_LOGW(TAG, "Ring Buffer Full");
						uart_flush(mConfig.port);
						xQueueReset(m_uart_queue);
						break;
					case UART_BREAK:
						ESP_LOGD(TAG, "Rx Break");
						break;
					case UART_PARITY_ERR:
						ESP_LOGE(TAG, "Parity Error");
						break;
					case UART_FRAME_ERR:
						ESP_LOGE(TAG, "Frame Error");
						break;
					case UART_PATTERN_DET:
						// Process end-of-line pattern
						pos = uart_pattern_pop_pos(mConfig.port);
						while (pos != -1)
						{
							if (pos < (GPS_NMEA_BUF - 2))
							{
								read_len = uart_read_bytes(mConfig.port, mBuf, pos + 1, 0);
								if (read_len > 0)
								{
									mBuf[read_len] = '\0';
									if (mBuf[1] != 'P')
									{
										ESP_LOGD(TAG, "%s", mBuf);
										// Decode NMEA message
										if (gps_decode(mBuf, read_len))
										{
											mCount++;
										}
									}
									else
									{
										ESP_LOGI(TAG, "%s", mBuf);
									}
								}
								else
								{
									uart_flush_input(mConfig.port);
									ESP_LOGE(TAG, "uart_read_bytes error");
								}
							}
							else
							{
								uart_flush_input(mConfig.port);
								ESP_LOGW(TAG, "Pattern Queue Size too small %d", pos);
								break;
							}
							pos = uart_pattern_pop_pos(mConfig.port);
						}
						break;
					default:
						ESP_LOGW(TAG, "unknown uart event type: %d", event.type);
						break;
					}
				}
			}
			// Process task messages
			else if (xActivatedMember == mTaskQueue)
			{
				while (getMessage(&msg))
				{
					switch (msg.msgID)
					{
					case MSG_GPS_ON:
						// Activate GPS with specified search time
						initUart();
						mSearchTime = msg.paramID;
						if (mSearchTime > 0)
						{
							time(&mStart_time);
							mWaitTime = 2 * mSearchTime;
						}
						else
						{
							mWaitTime = 0;
#ifdef CONFIG_SIM68MD_PD_2
							if (mConfig.pin_eint0 >= 0)
							{
								uart_write_bytes(mConfig.port, cmd_on2, strlen(cmd_on2));
								ESP_LOGD(TAG, "send %s", cmd_on2);
							}
							else
							{
								uart_write_bytes(mConfig.port, cmd_on1, strlen(cmd_on1));
								ESP_LOGD(TAG, "send %s", cmd_on1);
							}
							uart_wait_tx_done(mConfig.port, 10);
#endif
						}
						break;
					case MSG_GPS_OFF:
						// Deactivate GPS
						deinitUart(msg.shortParam != 0);
						if (msg.paramID > 0)
						{
							time(&mStart_time);
							mWaitTime = msg.paramID;
						}
						break;
					case MSG_END_TASK:
						goto endTask;
					default:
						ESP_LOGW(TAG, "unknown message %d", msg.msgID);
						break;
					}
				}
			}
		}
#ifndef CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE
		// Stack overflow control
		UBaseType_t m2 = uxTaskGetStackHighWaterMark2(nullptr);
		if (m2 != m1)
		{
			m1 = m2;
			TDEC("free gps stack", m2);
		}
#endif
		// Check GPS activity
		if (mRun == EGPSMode::Run)
		{
			time_t now;
			time(&now);
			if (count_time == 0)
			{
				count_time = now;
				mCount = 0;
			}
			else
			{
				if ((now - count_time) > CONFIG_SIM68MD_NMEA_TIMEOUT)
				{
					count_time = now;
					if (mCount == 0)
					{
						// Call handler when no data
						if (mConfig.onFailed != nullptr)
							mConfig.onFailed(this);
					}
					else if (mCount > 0)
						mCount = 0;
				}
			}
		}
		else
		{
			count_time = 0;
		}
	}
endTask:
	// Complete operation
	deinitUart(true);
	xQueueRemoveFromSet(mTaskQueue, mQueueSet);
}

/*!
	\brief Decode NMEA messages
	\param start Pointer to data buffer
	\param length Data length
	\return true - message processed successfully
*/
bool SIM68MD::gps_decode(char *start, size_t length)
{
	// std::string str(start,length);
	nmea_s *data = nmea_parse(start, length, 1);
	if (data == NULL)
	{
		return false;
	}
	else
	{
		if (data->errors != 0)
		{
			ESP_LOGW(TAG, "The sentence struct contains parse errors!");
		}
		else
		{
			// Process different NMEA message types
			if (NMEA_GPGGA == data->type)
			{
				// Process location data
				nmea_gpgga_s *gpgga = (nmea_gpgga_s *)data;
				if (gpgga->position_fix == 0)
				{
					if (mDelayNonValid != 0)
					{
						mDelayNonValid--;
						return true;
					}
				}
				else
				{
					mDelayNonValid = GPS_INAVALID_DELAY;
				}
				std::memcpy(&mTime, &gpgga->time, sizeof(tm));
				if (mData.valid != gpgga->position_fix)
				{
					mData.valid = gpgga->position_fix;
					mEventSend = true;
					mFixChanged = true;
				}
				// Update satellite and altitude information
				if (mData.satellites != gpgga->n_satellites)
				{
					mData.satellites = gpgga->n_satellites;
				}
				if (mData.altitude != gpgga->altitude)
				{
					mData.altitude = gpgga->altitude;
				}
				// Update coordinates
				if (mData.longitude.degrees != gpgga->longitude.degrees)
				{
					mData.longitude.degrees = gpgga->longitude.degrees;
					mEventSend = true;
				}
				if (mData.longitude.minutes != gpgga->longitude.minutes)
				{
					mData.longitude.minutes = gpgga->longitude.minutes;
					mEventSend = true;
				}
				if (mData.longitude.cardinal != gpgga->longitude.cardinal)
				{
					mData.longitude.cardinal = gpgga->longitude.cardinal;
					mEventSend = true;
				}
				if (mData.latitude.degrees != gpgga->latitude.degrees)
				{
					mData.latitude.degrees = gpgga->latitude.degrees;
					mEventSend = true;
				}
				if (mData.latitude.minutes != gpgga->latitude.minutes)
				{
					mData.latitude.minutes = gpgga->latitude.minutes;
					mEventSend = true;
				}
				if (mData.latitude.cardinal != gpgga->latitude.cardinal)
				{
					mData.latitude.cardinal = gpgga->latitude.cardinal;
					mEventSend = true;
				}
			}
			else if (NMEA_GPGSA == data->type)
			{
				// Process accuracy data
				nmea_gpgsa_s *gpgsa = (nmea_gpgsa_s *)data;
				mData.hdop = gpgsa->hdop;
			}
			else if (NMEA_GPRMC == data->type)
			{
				// Process time data
				nmea_gprmc_s *pos = (nmea_gprmc_s *)data;
				mTime.tm_mday = pos->date_time.tm_mday;
				mTime.tm_mon = pos->date_time.tm_mon;
				mTime.tm_year = pos->date_time.tm_year;
				mData.time = mktime(&mTime);
				if (mEventSend)
				{
					if (mFixChanged)
					{
						mFixChanged = false;
#if (CONFIG_SIM68MD_SYNC_TIME == 1)
						// Synchronize system time
						if (mData.valid != 0)
						{
							if (CDateTimeSystem::setDateTime(mData.time))
							{
								time(&mStart_time);
								mWaitTime = mSearchTime;
#ifndef CONFIG_SIM68MD_PD_2
								uart_write_bytes(mConfig.port, cmd_auto_saving_enable, strlen(cmd_auto_saving_enable));
								uart_wait_tx_done(mConfig.port, 10);
								ESP_LOGI(TAG, "send %s", cmd_auto_saving_enable);
#endif
							}
						}
#endif
					}
					// Call user data handler
					if (mEventSend)
					{
						mEventSend = false;
						if (mConfig.onDataRx != nullptr)
							mConfig.onDataRx(&mData, mRun);
					}
				}
			}
		}
		nmea_free(data);
	}
	return true;
}