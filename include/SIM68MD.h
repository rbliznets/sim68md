/*!
	\file SIM68MD.h
	\brief Драйвер для управления GPS-модулем SIM68MD через UART-интерфейс.
	\authors Близнец Р.А. (r.bliznets@gmail.com)
	\version 1.3.0.0
	\date 16 ноября 2023 г.
	\warning Для работы требуется ESP-IDF с поддержкой FreeRTOS и аппаратного UART
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
#define GPS_TX_BUF (128 + 16)		   ///< Размер передающего буфера UART (байт)
#define GPS_NMEA_BUF 256			   ///< Максимальная длина NMEA-строки (байт)
#define GPS_RX_BUF (GPS_NMEA_BUF + 64) ///< Размер приемного буфера UART (байт)
#define GPS_EVEN_BUF 10				   ///< Глубина очереди событий UART

// Настройка таймаута задачи для совместимости с Watchdog Timer
#ifdef CONFIG_ESP_TASK_WDT
#define TASK_MAX_BLOCK_TIME pdMS_TO_TICKS((CONFIG_ESP_TASK_WDT_TIMEOUT_S - 1) * 1000 + 500)
#else
#define TASK_MAX_BLOCK_TIME portMAX_DELAY ///< Максимальное время блокировки задачи
#endif

// Команды управления задачей GPS
#define MSG_END_TASK 0 ///< Принудительное завершение задачи
#define MSG_GPS_ON 10  ///< Активация GPS с параметрами поиска
#define MSG_GPS_OFF 11 ///< Деактивация GPS с выбором режима

// Параметры задачи обработки GPS
#define GPSTASK_NAME "gps"				   ///< Идентификатор задачи в отладке
#define GPSTASK_STACKSIZE (2 * 1024 + 512) ///< Размер стека задачи (байт)
#define GPSTASK_LENGTH 10				   ///< Вместимость очереди сообщений

/// Режимы работы GPS-модуля
enum class EGPSMode
{
	Unknown, ///< Неинициализированное состояние
	Run,	 ///< Активный режим с передачей данных
	Sleep,	 ///< Энергосберегающий режим с пробуждением по прерыванию
	RTC		 ///< Глубокий сон с питанием только часов реального времени
};

/// Географические координаты в формате NMEA
struct SPosition
{
	int16_t degrees; ///< Целая часть координаты в градусах
	float minutes;	 ///< Дробная часть в минутах (0.0′ - 59.9999′)
	char cardinal;	 ///< Направление: 'N', 'S', 'E' или 'W'
};

/// Данные, полученные от GPS-модуля
struct SGPSData
{
	uint8_t valid;		 ///< Статус позиционирования (0-недействительно)
	uint8_t satellites;	 ///< Число захваченных спутников (0-12)
	time_t time;		 ///< Время в формате UNIX (UTC)
	SPosition longitude; ///< Долгота (от -180° до +180°)
	SPosition latitude;	 ///< Широта (от -90° до +90°)
	float altitude;		 ///< Высота над уровнем моря (метры)
	float hdop;			 ///< Горизонтальная погрешность позиционирования
};

/// Callback-функция для обработки новых GPS-данных
/*!
 * \param[out] gps  Указатель на структуру с обновленными данными
 * \param[in]  mode Текущий режим работы модуля (из EGPSMode)
 */
typedef void onGPS(SGPSData *gps, EGPSMode mode);

class SIM68MD;

/// Callback-функция при отсутствии данных от GPS
/*!
 * \param[in] device Указатель на экземпляр драйвера SIM68MD
 */
typedef void onGPSFailed(SIM68MD *device);

/// Конфигурация драйвера GPS-модуля
struct SGPSConfig
{
	onGPS *onDataRx = nullptr;		 ///< Хендлер новых данных (вызывается в контексте задачи)
	onGPSFailed *onFailed = nullptr; ///< Хендлер ошибок связи

	uint8_t cpu = 1;   ///< Номер ядра процессора (0 или 1)
	uint8_t prior = 2; ///< Приоритет задачи (0-25, где 25 - наивысший)

	uart_port_t port = UART_NUM_1; ///< Используемый UART-порт
	int baudrate = 115200;		   ///< Скорость обмена (бит/с)

	int8_t pin_tx = 17;		 ///< GPIO для TX (UART)
	int8_t pin_rx = 18;		 ///< GPIO для RX (UART)
	int8_t pin_eint_in = 42; ///< GPIO для управления питанием (активный LOW)
	int8_t pin_eint0 = 48;	 ///< GPIO для прерывания пробуждения
};

/// Класс для работы с SIM68MD (реализация паттерна Singleton)
class SIM68MD : public CBaseTask
{
protected:
	static SIM68MD *theSingleInstance; ///< Единственный экземпляр класса

#if CONFIG_PM_ENABLE
	esp_pm_lock_handle_t mPMLock; ///< Блокировка снижения частоты CPU для UART
#endif

	SGPSConfig mConfig;		  ///< Конфигурационные параметры
	SGPSData mData;			  ///< Текущие GPS-данные
	std::tm mTime;			  ///< Временная структура для парсинга
	bool mEventSend = false;  ///< Флаг необходимости оповещения подписчиков
	bool mFixChanged = false; ///< Флаг изменения статуса фиксации

	QueueSetHandle_t mQueueSet;			  ///< Набор очередей событий
	QueueHandle_t m_uart_queue = nullptr; ///< Очередь событий UART
	char mBuf[GPS_NMEA_BUF];			  ///< Буфер для сырых NMEA-данных

	EGPSMode mRun = EGPSMode::Unknown; ///< Текущий режим работы
	uint32_t mWaitTime = 0;			   ///< Таймер ожидания (секунды)
	uint32_t mSearchTime = 0;		   ///< Макс. время поиска спутников
	uint16_t mCount = 0;			   ///< Счетчик успешных парсингов NMEA

	/// Инициализация UART и активация модуля
	void initUart();

	/// Деактивация UART с выбором режима энергосбережения
	/*!
	 * \param[in] rtc true - переход в RTC-режим, false - обычный сон
	 */
	void deinitUart(bool rtc = false);

	/// Основная функция задачи (наследование от CBaseTask)
	virtual void run() override;

	/// Парсинг NMEA-строки и обновление данных
	/*!
	 * \param[in] start  Указатель на начало строки
	 * \param[in] length Длина строки (байты)
	 * \return true - данные успешно обновлены
	 */
	bool gps_decode(char *start, size_t length);

	/// Приватный конструктор (Singleton)
	explicit SIM68MD(SGPSConfig *cfg);

	/// Деструктор с освобождением ресурсов
	virtual ~SIM68MD();

	using CBaseTask::sendCmd; // Разрешение доступа к базовому методу

public:
	/// Получение экземпляра класса
	static SIM68MD *Instance() { return theSingleInstance; }

	/// Инициализация драйвера
	/*!
	 * \param[in] cfg Конфигурация модуля
	 * \return Указатель на созданный экземпляр
	 */
	static SIM68MD *init(SGPSConfig *cfg);

	/// Деинициализация драйвера
	static void free();

	/// Проверка активности драйвера
	static inline bool isRun() { return (theSingleInstance != nullptr); }

	/// Запуск GPS с таймаутом поиска
	/*!
	 * \param[in] search_time Макс. время поиска спутников (0 - без ограничений)
	 * \return true - команда принята в очередь
	 */
	inline bool start(uint32_t search_time = 0)
	{
		return sendCmd(MSG_GPS_ON, 0, search_time);
	}

	/// Остановка GPS с выбором режима
	/*!
	 * \param[in] rtc_mode true - глубокий сон (RTC), false - обычный
	 * \param[in] wake_after Время до авто-пробуждения (0 - ручной старт)
	 * \return true - команда принята в очередь
	 */
	inline bool stop(bool rtc_mode = false, uint32_t wake_after = 0)
	{
		return sendCmd(MSG_GPS_OFF, rtc_mode, wake_after);
	}
};