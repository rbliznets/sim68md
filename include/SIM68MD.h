/*!
	\file
	\brief Класс управления GPS-модулем SIM68MD через UART.
	\authors Близнец Р.А. (r.bliznets@gmail.com)
	\version 1.2.0.0
	\date 16.11.2023
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

// Конфигурационные параметры UART
#define GPS_TX_BUF (128 + 16)		   ///< Размер буфера UART на передачу (байт).
#define GPS_NMEA_BUF (256)			   ///< Максимальный размер строки NMEA (байт).
#define GPS_RX_BUF (GPS_NMEA_BUF + 64) ///< Размер буфера UART на прием (байт).
#define GPS_EVEN_BUF (10)			   ///< Размер очереди событий UART.

// Настройка максимального времени блокировки задачи для совместимости с Watchdog Timer
#ifdef CONFIG_ESP_TASK_WDT
#define TASK_MAX_BLOCK_TIME pdMS_TO_TICKS((CONFIG_ESP_TASK_WDT_TIMEOUT_S - 1) * 1000 + 500)
#else
#define TASK_MAX_BLOCK_TIME portMAX_DELAY
#endif

// Коды команд для управления задачей GPS
#define MSG_END_TASK (0) ///< Команда завершения работы задачи
#define MSG_GPS_ON (10)	 ///< Команда включения GPS-модуля
#define MSG_GPS_OFF (11) ///< Команда выключения GPS-модуля

// Параметры задачи обработки GPS
#define GPSTASK_NAME "gps"				   ///< Идентификатор задачи для отладки
#define GPSTASK_STACKSIZE (2 * 1024 + 512) ///< Размер стека задачи (байт)
#define GPSTASK_LENGTH (10)				   ///< Размер очереди сообщений задачи

/// Состояния работы GPS-модуля
enum class EGPSMode
{
	Unknown, ///< Начальное состояние после сброса
	Run,	 ///< Активный режим работы
	Sleep,	 ///< Режим пониженного энергопотребления
	RTC		 ///< Полное выключение с сохранением данных в RTC
};

/// Структура географической позиции
struct SPosition
{
	int16_t degrees; ///< Градусы координаты (целая часть)
	float minutes;	 ///< Минуты координаты (дробная часть)
	char cardinal;	 ///< Направление (N - север, S - юг, W - запад, E - восток)
};

/// Структура данных, получаемых от GPS-модуля
struct SGPSData
{
	uint8_t valid;		 ///< Тип позиционирования (0-8, см. документацию NMEA)
	uint8_t satellites;	 ///< Количество используемых спутников
	time_t time;		 ///< Временная метка (UTC)
	SPosition longitude; ///< Координата долготы
	SPosition latitude;	 ///< Координата широты
	float altitude;		 ///< Высота над уровнем моря (метры)
	float hdop;			 ///< Горизонтальная погрешность (HDOP)
};

/// Тип функции обратного вызова при обновлении GPS-данных
/*!
 * \param[in] gps Указатель на структуру с актуальными данными
 * \param[in] run Текущий режим работы модуля
 */
typedef void onGPS(SGPSData *gps, EGPSMode run);

/// Конфигурационные параметры драйвера SIM68MD
struct SGPSConfig
{
	onGPS *onDataRx = nullptr; ///< Callback-функция для обработки новых данных

	uint8_t cpu = 1;   ///< Ядро процессора для запуска задачи (0/1)
	uint8_t prior = 2; ///< Приоритет задачи (чем выше, тем приоритетнее)

	uart_port_t port = UART_NUM_1; ///< Номер UART-порта (UART_NUM_1 по умолчанию)
	int baudrate = 115200;		   ///< Скорость обмена данными (бод)

	int8_t pin_tx = 17;		 ///< GPIO для передачи данных (TX)
	int8_t pin_rx = 18;		 ///< GPIO для приема данных (RX)
	int8_t pin_eint_in = 42; ///< GPIO для внешнего прерывания (EINT_IN)
	int8_t pin_eint0 = 48;	 ///< GPIO для управления питанием (EINT0)
};

/// Класс для работы с GPS-модулем SIM68MD через UART-интерфейс
class SIM68MD : public CBaseTask
{
protected:
	static SIM68MD *theSingleInstance; ///< Указатель на единственный экземпляр класса (Singleton)
#if CONFIG_PM_ENABLE
	esp_pm_lock_handle_t mPMLock; ///< Блокировка управления питанием для стабильной работы UART
#endif
	SGPSConfig mConfig; ///< Конфигурация драйвера

	SGPSData mData;			  ///< Текущие данные GPS
	std::tm mTime;			  ///< Локальное время (с учетом временной зоны)
	bool mEventSend = false;  ///< Флаг необходимости вызова callback-функции
	bool mFixChanged = false; ///< Флаг изменения статуса позиционирования

	QueueSetHandle_t mQueueSet;			  ///< Набор очередей для обработки событий
	QueueHandle_t m_uart_queue = nullptr; ///< Очередь событий UART-драйвера
	char mBuf[GPS_NMEA_BUF];			  ///< Буфер для накопления NMEA-строк

	EGPSMode mRun = EGPSMode::Unknown; ///< Текущий режим работы модуля
	uint32_t mWaitTime = 0;			   ///< Время ожидания следующего события
	uint32_t mSearchTime = 0;		   ///< Максимальное время поиска спутников (мс)

	/// Инициализация UART и запуск модуля
	void initUart();

	/// Деинициализация UART и перевод модуля в режим сна
	/*!
	  \param[in] rtc Если true - режим глубокого сна (RTC), иначе - обычный сон
	*/
	void deinitUart(bool rtc = false);

	/// Основная функция задачи обработки GPS (переопределение метода CBaseTask)
	virtual void run() override;

	/// Разбор NMEA-строки и обновление данных
	/*!
	  \param[in] start Указатель на начало строки
	  \param[in] length Длина строки в байтах
	*/
	void gps_decode(char *start, size_t length);

	/// Приватный конструктор (реализация паттерна Singleton)
	/*!
	  \param[in] cfg Указатель на структуру конфигурации
	*/
	SIM68MD(SGPSConfig *cfg);

	/// Деструктор
	virtual ~SIM68MD();

	using CBaseTask::sendCmd;

public:
	/// Получение экземпляра класса (Singleton)
	/*!
	  \return Указатель на экземпляр класса или nullptr если не инициализирован
	*/
	static SIM68MD *Instance() { return theSingleInstance; };

	/// Инициализация экземпляра класса
	/*!
	  \param[in] cfg Указатель на структуру конфигурации
	  \return Указатель на созданный экземпляр
	*/
	static SIM68MD *init(SGPSConfig *cfg);

	/// Освобождение ресурсов экземпляра класса
	static void free();

	/// Проверка активности экземпляра
	static inline bool isRun() { return (theSingleInstance != nullptr); };

	/// Запуск работы GPS-модуля
	/*!
	  \param[in] sleep Максимальное время поиска спутников (мс). 0 - без ограничений
	  \return true - команда успешно отправлена
	*/
	inline bool start(uint32_t sleep = 0) { return sendCmd(MSG_GPS_ON, 0, sleep); };

	/// Остановка работы GPS-модуля
	/*!
	  \param[in] rtc Режим отключения (true - RTC, false - sleep)
	  \param[in] wakeUp Время до автоматического пробуждения (мс). 0 - отключение
	  \return true - команда успешно отправлена
	*/
	inline bool stop(bool rtc = false, uint32_t wakeUp = 0) { return sendCmd(MSG_GPS_OFF, rtc, wakeUp); };
};