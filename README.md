# SIM68MD
Для добавления в проект в папке компонентов из командной строки запустить:    
```
git submodule add https://github.com/rbliznets/sim68md SIM68MD
```
## Пример использования
```
#define GPS_TX_PIN (43)
#define GPS_RX_PIN (44)
#define GPS_EINT_IN_PIN (10) ///< Номер вывода EINT_IN.
#define GPS_EINT0_PIN (45)   ///< Номер вывода EINT0.

#define GPS_UART (UART_NUM_0) ///< Номер UART.
#define GPS_BAUDRATE (115200)   ///< Скорость обмена UART.

#define GPSTASK_PRIOR (2) ///< Приоритет задачи.
#define GPSTASK_CPU (1)   ///< Номер ядра процессора.

void onGPSEv(SGPSData *gps, EGPSMode run)
{
	printf("valid %d", gps->valid);
}

SGPSData *gps;
float lon, lat;
gcfg = {
    onGPSEv,
    GPSTASK_CPU, GPSTASK_PRIOR,
    GPS_UART, GPS_BAUDRATE,
    GPS_TX_PIN, GPS_RX_PIN, GPS_EINT_IN_PIN, GPS_EINT0_PIN};

SIM68MD::init(&gcfg)->start(0);
```