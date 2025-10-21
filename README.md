# SIM68MD
To add to a project in the components folder from the command line, run:    
```
git submodule add https://github.com/rbliznets/sim68md SIM68MD
```
## Example Usage
```
#define GPS_TX_PIN (43)
#define GPS_RX_PIN (44)
#define GPS_EINT_IN_PIN (10) ///< EINT_IN pin number.
#define GPS_EINT0_PIN (45)   ///< EINT0 pin number.

#define GPS_UART (UART_NUM_0) ///< UART number.
#define GPS_BAUDRATE (115200)   ///< UART baud rate.

#define GPSTASK_PRIOR (2) ///< Task priority.
#define GPSTASK_CPU (1)   ///< CPU core number.

void onGPSEv(SGPSData *gps, EGPSMode run)
{
    printf("valid %d", gps->valid);
}

SGPSData *gps;
float lon, lat;
gcfg = {
    onGPSEv,nullptr,nullptr,
    GPSTASK_CPU, GPSTASK_PRIOR,
    GPS_UART, GPS_BAUDRATE,
    GPS_TX_PIN, GPS_RX_PIN, GPS_EINT_IN_PIN, GPS_EINT0_PIN};

SIM68MD::init(&gcfg)->start(0);
```