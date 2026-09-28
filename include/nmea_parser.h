/*!
	\file nmea_parser.h
	\brief Локальный NMEA-парсер компонента SIM68MD (вынесен из libnmea).
	Содержит только типы и функции, реально используемые SIM68MD:
	разбор предложений GPGGA, GPRMC и GPGSA.
*/

#ifndef INC_SIM68MD_NMEA_PARSER_H
#define INC_SIM68MD_NMEA_PARSER_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

/* NMEA sentence types (только используемые) */
typedef enum {
	NMEA_UNKNOWN,
	NMEA_GPGGA,
	NMEA_GPGSA,
	NMEA_GPRMC
} nmea_t;

/* NMEA cardinal direction types */
typedef char nmea_cardinal_t;
#define NMEA_CARDINAL_DIR_NORTH		(nmea_cardinal_t) 'N'
#define NMEA_CARDINAL_DIR_EAST		(nmea_cardinal_t) 'E'
#define NMEA_CARDINAL_DIR_SOUTH		(nmea_cardinal_t) 'S'
#define NMEA_CARDINAL_DIR_WEST		(nmea_cardinal_t) 'W'
#define NMEA_CARDINAL_DIR_UNKNOWN	(nmea_cardinal_t) '\0'

/**
 * NMEA data base struct
 *
 * This struct will be extended by the parser data structs (ex: nmea_gpgga_s).
 */
typedef struct {
	nmea_t type;
	int errors;
} nmea_s;

/* GPS position struct */
typedef struct {
	double minutes;
	int degrees;
	nmea_cardinal_t cardinal;
} nmea_position;

/* NMEA sentence max length, including \r\n (chars) */
#define NMEA_MAX_LENGTH		82

/* NMEA sentence endings, should be \r\n according the NMEA 0183 standard */
#define NMEA_END_CHAR_1		'\r'
#define NMEA_END_CHAR_2		'\n'

/* NMEA sentence prefix length (num chars), Ex: GPGLL */
#define NMEA_PREFIX_LENGTH	5

/* ---- GPGGA sentence data ---- */
typedef struct {
	nmea_s base;
	struct tm time;
	nmea_position longitude;
	nmea_position latitude;
	int n_satellites;
	double altitude;
	char altitude_unit;
	double undulation;
	char undulation_unit;
	unsigned char position_fix;
} nmea_gpgga_s;

/* ---- GPRMC sentence data ---- */
typedef struct {
	nmea_s base;
	struct tm date_time;
	nmea_position longitude;
	nmea_position latitude;
	double gndspd_knots;
	double track_deg;
	double magvar_deg;
	nmea_cardinal_t magvar_cardinal;
	bool valid;
} nmea_gprmc_s;

/* ---- GPGSA sentence data ---- */
typedef struct {
	nmea_s base;
	char mode; /* M = manual (forced 2D or 3D), A = automatic */
	char fixtype; /* 1 = no fix, 2 = 2D, 3 = 3D */
	int satID_00;
	int satID_01;
	int satID_02;
	int satID_03;
	int satID_04;
	int satID_05;
	int satID_06;
	int satID_07;
	int satID_08;
	int satID_09;
	int satID_10;
	int satID_11;
	double pdop;
	double hdop;
	double vdop;
} nmea_gpgsa_s;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Free an nmea data struct.
 *
 * data should be a pointer to a struct of type nmea_s.
 */
void nmea_free(nmea_s *data);

/**
 * Parse an NMEA sentence string to a struct.
 *
 * sentence needs to be a validated NMEA sentence string.
 * length is the character length of the sentence string.
 * check_checksum, if 1 and there is a checksum, validate it.
 *
 * Returns a pointer to an NMEA data struct, or (nmea_s *) NULL if an error occurs.
 */
nmea_s *nmea_parse(char *sentence, size_t length, int check_checksum);

#ifdef __cplusplus
}
#endif

#endif  /* INC_SIM68MD_NMEA_PARSER_H */