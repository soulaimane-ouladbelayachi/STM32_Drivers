/*
 * gps.h
 *
 *  Created on: Sep 14, 2022
 *      Author: Soulaimane OuladBelayachi
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "stm32f4xx_hal.h"

#define GPSBUFFERSIZE 128

typedef struct{

    // calculated values
    float dec_longitude;
    float dec_latitude;
    float altitude_ft;

    // GGA - Global Positioning System Fixed Data
    float nmea_longitude;
    float nmea_latitude;
    float utc_time;
    char ns, ew;
    int lock;
    int satelites;

    float hdop;
    float msl_altitude;
    char msl_units;

    // RMC - Recommended Minimum Specific GNS Data
    char rmc_status;
    float speed_k;
    float course_d;
    int date;

    // GLL
    char gll_status;

    // VTG - Course over ground, ground speed
    float course_t; // ground speed true
    char course_t_unit;
    float course_m; // magnetic
    char course_m_unit;
    char speed_k_unit;
    float speed_km; // speed Km/h
    char speed_km_unit;
} GPS_t;

/*
 * Functions prototypes
 */
void GPS_Init(UART_HandleTypeDef *uart_handler);
void GPS_print_val(char *data, int value);
void GPS_UART_CallBack(UART_HandleTypeDef *uart_handler);
int GPS_validate(char *nmeastr);
int GPS_parse(char *GPSstrParse);
float GPS_nmea_to_dec(float deg_coord, char nsew);



#endif /* INC_GPS_H_ */
