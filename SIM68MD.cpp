/*!
	\file
	\brief Класс управления SIM68MD через UART.
	\authors Близнец Р.А. (r.bliznets@gmail.com)
	\version 1.2.0.0
	\date 16.11.2023
*/

#include "SIM68MD.h"
#include "CDateTimeSystem.h"
#include "CTrace.h"
#include "driver/gpio.h"
#include <cstring>
#include <sys/time.h>

static const char *GPS_TAG = "gps";
static const char *cmd_on = "$PAIR002*38\r\n";
static const char *cmd_off = "$PAIR003*39\r\n";
static const char *cmd_rtc = "$PAIR650,0*25\r\n";

SIM68MD *SIM68MD::theSingleInstance = nullptr;

SIM68MD *SIM68MD::init(SGPSConfig *cfg)
{
	if (theSingleInstance == nullptr)
	{
		theSingleInstance = new SIM68MD(cfg);
		theSingleInstance->CBaseTask::init(GPSTASK_NAME, GPSTASK_STACKSIZE, cfg->prior, GPSTASK_LENGTH, cfg->cpu);
		xQueueAddToSet(theSingleInstance->mTaskQueue, theSingleInstance->mQueueSet);
		vTaskDelay(2);
	}
	return theSingleInstance;
}

void SIM68MD::free()
{
	if (theSingleInstance != nullptr)
	{
		theSingleInstance->sendCmd(MSG_END_TASK);
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
		delete theSingleInstance;
		theSingleInstance = nullptr;
	}
}

SIM68MD::SIM68MD(SGPSConfig *cfg) : CBaseTask()
{
	std::memcpy(&mConfig, cfg, sizeof(SGPSConfig));

	if ((mConfig.pin_eint_in >= 0) && (mConfig.pin_eint0 >= 0))
	{
		gpio_iomux_in(mConfig.pin_eint_in, 1);
		gpio_iomux_out(mConfig.pin_eint_in, 1, false);
		gpio_set_direction((gpio_num_t)mConfig.pin_eint_in, GPIO_MODE_OUTPUT);
		gpio_set_pull_mode((gpio_num_t)mConfig.pin_eint_in, GPIO_PULLUP_ONLY);
		gpio_set_level((gpio_num_t)mConfig.pin_eint_in, 1);

		gpio_iomux_in(mConfig.pin_eint0, 1);
		gpio_iomux_out(mConfig.pin_eint0, 1, false);
		gpio_set_direction((gpio_num_t)mConfig.pin_eint0, GPIO_MODE_OUTPUT);
		gpio_set_pull_mode((gpio_num_t)mConfig.pin_eint0, GPIO_PULLDOWN_ONLY);
		gpio_set_level((gpio_num_t)mConfig.pin_eint0, 0);
	}

	gpio_iomux_in(mConfig.pin_tx, 1);
	gpio_iomux_out(mConfig.pin_tx, 1, false);
	gpio_iomux_in(mConfig.pin_rx, 1);
	gpio_iomux_out(mConfig.pin_rx, 1, false);

	std::memset(&mData, 0, sizeof(SGPSData));
	mQueueSet = xQueueCreateSet(GPSTASK_LENGTH + GPS_EVEN_BUF);

#if CONFIG_PM_ENABLE
	esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "gps", &mPMLock);
#endif
}

SIM68MD::~SIM68MD()
{
	vQueueDelete(mQueueSet);
#if CONFIG_PM_ENABLE
	esp_pm_lock_delete(mPMLock);
#endif

	if ((mConfig.pin_eint_in >= 0) && (mConfig.pin_eint0 >= 0))
	{
		gpio_set_pull_mode((gpio_num_t)mConfig.pin_eint_in, GPIO_FLOATING);
		gpio_set_direction((gpio_num_t)mConfig.pin_eint_in, GPIO_MODE_DISABLE);
		gpio_set_pull_mode((gpio_num_t)mConfig.pin_eint0, GPIO_FLOATING);
		gpio_set_direction((gpio_num_t)mConfig.pin_eint0, GPIO_MODE_DISABLE);
	}
}

void SIM68MD::initUart()
{
	if (mRun != EGPSMode::Run)
	{
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
		ESP_ERROR_CHECK(uart_driver_install(mConfig.port, GPS_RX_BUF, GPS_TX_BUF, GPS_EVEN_BUF, &m_uart_queue, intr_alloc_flags));
		xQueueAddToSet(m_uart_queue, mQueueSet);
		ESP_ERROR_CHECK(uart_param_config(mConfig.port, &uart_config));
		ESP_ERROR_CHECK(uart_set_pin(mConfig.port, mConfig.pin_tx, mConfig.pin_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

		/* Set pattern interrupt, used to detect the end of a line */
		ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(mConfig.port, '\n', 1, 5, 0, 0));
		/* Set pattern queue size */
		ESP_ERROR_CHECK(uart_pattern_queue_reset(mConfig.port, GPS_EVEN_BUF));
		ESP_ERROR_CHECK(uart_flush(mConfig.port));

		if ((mConfig.pin_eint_in >= 0) && (mConfig.pin_eint0 >= 0))
		{
			gpio_set_level((gpio_num_t)mConfig.pin_eint_in, 0);
			gpio_set_level((gpio_num_t)mConfig.pin_eint0, 1);
			vTaskDelay(pdMS_TO_TICKS(12));
			gpio_set_level((gpio_num_t)mConfig.pin_eint_in, 1);
			gpio_set_level((gpio_num_t)mConfig.pin_eint0, 0);
			vTaskDelay(pdMS_TO_TICKS(102));
		}
		uart_write_bytes(mConfig.port, cmd_on, strlen(cmd_on));
		mRun = EGPSMode::Run;

		mEventSend = false;
		std::memset(&mData, 0, sizeof(SGPSData));
		ESP_LOGI(GPS_TAG, "Run");
	}
}

void SIM68MD::deinitUart(bool rtc)
{
	if (mRun == EGPSMode::Run)
	{
		xQueueRemoveFromSet(m_uart_queue, mQueueSet);
		ESP_ERROR_CHECK(uart_wait_tx_done(mConfig.port, pdMS_TO_TICKS(250)));
		if (rtc)
		{
			uart_write_bytes(mConfig.port, cmd_rtc, strlen(cmd_rtc));
			mRun = EGPSMode::RTC;
			if ((mConfig.pin_eint_in >= 0) && (mConfig.pin_eint0 >= 0))
				ESP_LOGI(GPS_TAG, "RTC");
			else
				ESP_LOGW(GPS_TAG, "RTC without exit");
		}
		else
		{
			uart_write_bytes(mConfig.port, cmd_off, strlen(cmd_off));
			mRun = EGPSMode::Sleep;
			ESP_LOGI(GPS_TAG, "Sleep");
		}
		ESP_ERROR_CHECK(uart_wait_tx_done(mConfig.port, pdMS_TO_TICKS(150)));
		ESP_ERROR_CHECK(uart_driver_delete(mConfig.port));
		m_uart_queue = nullptr;
		vTaskDelay(pdMS_TO_TICKS(10));
#if CONFIG_PM_ENABLE
		esp_pm_lock_release(mPMLock);
#endif
	}
}

void SIM68MD::run()
{
#ifndef CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE
	UBaseType_t m1 = uxTaskGetStackHighWaterMark2(nullptr);
#endif
	STaskMessage msg;
	QueueSetMemberHandle_t xActivatedMember;
	time_t start_time;

	for (;;)
	{
		if (mWaitTime != 0)
		{
			time_t now;
			time(&now);
			if ((now - start_time) > mWaitTime)
			{
				mWaitTime = 0;
				if (mRun == EGPSMode::Run)
				{
					deinitUart(true);
					if (mConfig.onDataRx != nullptr)
						mConfig.onDataRx(&mData, mRun);
				}
				else
				{
					initUart();
					if (mSearchTime > 0)
					{
						time(&start_time);
						mWaitTime = mSearchTime;
					}
				}
			}
		}
		if ((xActivatedMember = xQueueSelectFromSet(mQueueSet, TASK_MAX_BLOCK_TIME)) != nullptr)
		{
			if ((mRun == EGPSMode::Run) && (xActivatedMember == m_uart_queue))
			{
				uart_event_t event;
				int pos, read_len;
				while ((m_uart_queue != nullptr) && (xQueueReceive(m_uart_queue, &event, 0) == pdTRUE))
				{
					if (mRun != EGPSMode::Run)
						continue;
					switch (event.type)
					{
					case UART_DATA:
						break;
					case UART_FIFO_OVF:
						ESP_LOGW(GPS_TAG, "HW FIFO Overflow");
						uart_flush(mConfig.port);
						xQueueReset(m_uart_queue);
						break;
					case UART_BUFFER_FULL:
						ESP_LOGW(GPS_TAG, "Ring Buffer Full");
						uart_flush(mConfig.port);
						xQueueReset(m_uart_queue);
						break;
					case UART_BREAK:
						ESP_LOGW(GPS_TAG, "Rx Break");
						break;
					case UART_PARITY_ERR:
						ESP_LOGE(GPS_TAG, "Parity Error");
						break;
					case UART_FRAME_ERR:
						ESP_LOGE(GPS_TAG, "Frame Error");
						break;
					case UART_PATTERN_DET:
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
										gps_decode(mBuf, read_len);
									}
									ESP_LOGD(GPS_TAG, "%s", mBuf);
								}
								else
								{
									uart_flush_input(mConfig.port);
									ESP_LOGE(GPS_TAG, "uart_read_bytes error");
								}
							}
							else
							{
								uart_flush_input(mConfig.port);
								ESP_LOGW(GPS_TAG, "Pattern Queue Size too small %d", pos);
								break;
							}
							pos = uart_pattern_pop_pos(mConfig.port);
						}
						break;
					default:
						ESP_LOGW(GPS_TAG, "unknown uart event type: %d", event.type);
						break;
					}
				}
			}
			else if (xActivatedMember == mTaskQueue)
			{
				while (getMessage(&msg))
				{
					// TDEC("gps",msg.msgID);
					switch (msg.msgID)
					{
					case MSG_GPS_ON:
						initUart();
						mSearchTime = msg.paramID;
						if (mSearchTime > 0)
						{
							time(&start_time);
							mWaitTime = mSearchTime;
						}
						else
							mWaitTime = 0;
						break;
					case MSG_GPS_OFF:
						deinitUart(msg.shortParam != 0);
						if (msg.paramID > 0)
						{
							time(&start_time);
							mWaitTime = msg.paramID;
						}
						break;
					case MSG_END_TASK:
						goto endTask;
					default:
						TRACE_WARNING("SIM68MD:unknown message", msg.msgID);
						break;
					}
				}
			}
		}
#ifndef CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE
		UBaseType_t m2 = uxTaskGetStackHighWaterMark2(nullptr);
		if (m2 != m1)
		{
			m1 = m2;
			TDEC("free gps stack", m2);
		}
#endif
	}
endTask:
	deinitUart(true);
	xQueueRemoveFromSet(mTaskQueue, mQueueSet);
}

void SIM68MD::gps_decode(char *start, size_t length)
{
	ESP_LOGD(GPS_TAG, "%s", start);
	nmea_s *data = nmea_parse(start, length, 1);
	if (data == NULL)
	{
		return;
	}
	else
	{
		if (data->errors != 0)
		{
			ESP_LOGW(GPS_TAG, "The sentence struct contains parse errors!");
		}
		else
		{
			if (NMEA_GPGGA == data->type)
			{
				nmea_gpgga_s *gpgga = (nmea_gpgga_s *)data;
				std::memcpy(&mTime, &gpgga->time, sizeof(tm));
				if (mData.valid != gpgga->position_fix)
				{
					mData.valid = gpgga->position_fix;
					mEventSend = true;
					mFixChanged = true;
					if (mData.valid > 0)
						mWaitTime = 0;
				}
				if (mData.satellites != gpgga->n_satellites)
				{
					mData.satellites = gpgga->n_satellites;
				}
				if (mData.altitude != gpgga->altitude)
				{
					mData.altitude = gpgga->altitude;
				}
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
				nmea_gpgsa_s *gpgsa = (nmea_gpgsa_s *)data;
				mData.hdop = gpgsa->hdop;
			}
			else if (NMEA_GPRMC == data->type)
			{
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
						if (mData.valid != 0)
						{
							CDateTimeSystem::setDateTime(mData.time);
						}
#endif
					}
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
}
