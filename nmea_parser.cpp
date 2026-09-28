/*!
	\file nmea_parser.cpp
	\brief Локальный NMEA-парсер компонента SIM68MD (вынесен из libnmea).
	Содержит только используемые SIM68MD функции: nmea_parse(), nmea_free()
	и разбор предложений GPGGA, GPRMC, GPGSA.
*/

#include "nmea_parser.h"

#include <ctype.h>

/* ================= Параметры парсинга (из parse.c) ================= */

#define NMEA_TIME_FORMAT_LEN 6
#define TM_YEAR_START 1900
#define RMC_YEAR_START 2000

static int nmea_position_parse(char *s, nmea_position *pos);
static nmea_cardinal_t nmea_cardinal_direction_parse(char *s);
static int nmea_time_parse(char *s, struct tm *time);
static int nmea_date_parse(char *s, struct tm *date);

static int
nmea_position_parse(char *s, nmea_position *pos)
{
	char *cursor;

	pos->degrees = 0;
	pos->minutes = 0;

	if (s == NULL || *s == '\0') {
		return -1;
	}

	/* decimal minutes */
	if (NULL == (cursor = strchr(s, '.'))) {
		return -1;
	}

	/* minutes starts 2 digits before dot */
	cursor -= 2;
	pos->minutes = atof(cursor);
	*cursor = '\0';

	/* integer degrees */
	cursor = s;
	pos->degrees = atoi(cursor);

	return 0;
}

static nmea_cardinal_t
nmea_cardinal_direction_parse(char *s)
{
	if (NULL == s || '\0' == *s) {
		return NMEA_CARDINAL_DIR_UNKNOWN;
	}

	switch (*s) {
	case NMEA_CARDINAL_DIR_NORTH:
		return NMEA_CARDINAL_DIR_NORTH;
	case NMEA_CARDINAL_DIR_EAST:
		return NMEA_CARDINAL_DIR_EAST;
	case NMEA_CARDINAL_DIR_SOUTH:
		return NMEA_CARDINAL_DIR_SOUTH;
	case NMEA_CARDINAL_DIR_WEST:
		return NMEA_CARDINAL_DIR_WEST;
	default:
		break;
	}

	return NMEA_CARDINAL_DIR_UNKNOWN;
}

static int
nmea_time_parse(char *s, struct tm *time)
{
	char *rv;
	uint32_t x;

	if (s == NULL || *s == '\0') {
		return -1;
	}

	x = strtoul(s, &rv, 10);
	time->tm_hour = x / 10000;
	time->tm_min = (x % 10000) / 100;
	time->tm_sec = x % 100;
	if (time->tm_hour > 23 || time->tm_min > 59 || time->tm_sec > 59 || (int)(rv - s) < NMEA_TIME_FORMAT_LEN) {
		return -1;
	}
	if (*rv == '.') {
		/* TODO There is a sub-second field. */
	}

	return 0;
}

static int
nmea_date_parse(char *s, struct tm *date)
{
	char *rv;
	uint32_t x;

	if (s == NULL || *s == '\0') {
		return -1;
	}

	x = strtoul(s, &rv, 10);
	date->tm_mday = x / 10000;
	date->tm_mon = ((x % 10000) / 100) - 1;
	date->tm_year = x % 100;

	// Normalize tm_year according to C standard library
	if (date->tm_year > 1900) { // ZDA message case
		date->tm_year -= TM_YEAR_START;
	} else { // RMC message case
		date->tm_year += (RMC_YEAR_START - TM_YEAR_START);
	}

	return 0;
}

/* ============== Каркас парсера (из parser_types.h/parser.h) ============== */

typedef struct {
	nmea_t type;
	/* Для совместимости с более старыми версиями type_word также содержит
	 * идентификатор talker (например "GP"). Он не учитывается при поиске
	 * парсера: парсер с type_word="GPRMC" используется и для "GNRMC". */
	char type_word[NMEA_PREFIX_LENGTH];
	nmea_s *data;
} nmea_parser_s;

#define NMEA_PARSER_PREFIX(parser, type_prefix) memcpy((parser)->type_word, (type_prefix), NMEA_PREFIX_LENGTH)
#define NMEA_PARSER_TYPE(parser, nmea_type) (parser)->type = (nmea_type)

typedef int (*allocate_data_f)(nmea_parser_s *);
typedef int (*set_default_f)(nmea_parser_s *);
typedef int (*free_data_f)(nmea_s *);
typedef int (*parse_f)(nmea_parser_s *, char *, int);

typedef struct {
	nmea_parser_s parser;
	int errors;

	/* Functions */
	allocate_data_f allocate_data;
	set_default_f set_default;
	free_data_f free_data;
	parse_f parse;
} nmea_parser_module_s;

/* ================= Парсер GPGGA (из gpgga.c) ================= */

/* Value indexes */
#define NMEA_GPGGA_TIME			0
#define NMEA_GPGGA_LATITUDE		1
#define NMEA_GPGGA_LATITUDE_CARDINAL	2
#define NMEA_GPGGA_LONGITUDE		3
#define NMEA_GPGGA_LONGITUDE_CARDINAL	4
#define NMEA_GPGGA_POSITION_FIX		5
#define NMEA_GPGGA_N_SATELLITES		6
#define NMEA_GPGGA_ALTITUDE		8
#define NMEA_GPGGA_ALTITUDE_UNIT	9
#define NMEA_GPGGA_UNDULATION		10
#define NMEA_GPGGA_UNDULATION_UNIT	11

#define INVALID_UNDULATION -9999.999

static int nmea_gpgga_init(nmea_parser_s *parser);
static int nmea_gpgga_allocate_data(nmea_parser_s *parser);
static int nmea_gpgga_set_default(nmea_parser_s *parser);
static int nmea_gpgga_free_data(nmea_s *data);
static int nmea_gpgga_parse(nmea_parser_s *parser, char *value, int val_index);

static int
nmea_gpgga_init(nmea_parser_s *parser)
{
	/* Declare what sentence type to parse */
	NMEA_PARSER_TYPE(parser, NMEA_GPGGA);
	NMEA_PARSER_PREFIX(parser, "GPGGA");
	return 0;
}

static int
nmea_gpgga_allocate_data(nmea_parser_s *parser)
{
	parser->data = (nmea_s *)malloc(sizeof(nmea_gpgga_s));
	if (NULL == parser->data) {
		return -1;
	}

	return 0;
}

static int
nmea_gpgga_set_default(nmea_parser_s *parser)
{
	memset(parser->data, 0, sizeof(nmea_gpgga_s));
	// Set the default undulation to an invalid value
	nmea_gpgga_s *data = (nmea_gpgga_s *)parser->data;
	data->undulation = INVALID_UNDULATION;
	return 0;
}

static int
nmea_gpgga_free_data(nmea_s *data)
{
	free(data);
	return 0;
}

static int
nmea_gpgga_parse(nmea_parser_s *parser, char *value, int val_index)
{
	nmea_gpgga_s *data = (nmea_gpgga_s *)parser->data;

	switch (val_index) {
	case NMEA_GPGGA_TIME:
		/* Parse time */
		if (-1 == nmea_time_parse(value, &data->time)) {
			return -1;
		}
		break;

	case NMEA_GPGGA_LATITUDE:
		/* Parse latitude */
		if (-1 == nmea_position_parse(value, &data->latitude)) {
			return -1;
		}
		break;

	case NMEA_GPGGA_LATITUDE_CARDINAL:
		/* Parse cardinal direction */
		data->latitude.cardinal = nmea_cardinal_direction_parse(value);
		if (NMEA_CARDINAL_DIR_UNKNOWN == data->latitude.cardinal) {
			return -1;
		}
		break;

	case NMEA_GPGGA_LONGITUDE:
		/* Parse longitude */
		if (-1 == nmea_position_parse(value, &data->longitude)) {
			return -1;
		}
		break;

	case NMEA_GPGGA_LONGITUDE_CARDINAL:
		/* Parse cardinal direction */
		data->longitude.cardinal = nmea_cardinal_direction_parse(value);
		if (NMEA_CARDINAL_DIR_UNKNOWN == data->longitude.cardinal) {
			return -1;
		}
		break;

	case NMEA_GPGGA_POSITION_FIX:
		/* Parse position fix indicator */
		data->position_fix = atoi(value);
		break;

	case NMEA_GPGGA_N_SATELLITES:
		/* Parse number of satellies */
		data->n_satellites = atoi(value);
		break;

	case NMEA_GPGGA_ALTITUDE:
		/* Parse altitude */
		data->altitude = atof(value);
		break;

	case NMEA_GPGGA_ALTITUDE_UNIT:
		/* Parse altitude unit */
		data->altitude_unit = *value;
		break;

	case NMEA_GPGGA_UNDULATION:
		/* Parse undulation */
		data->undulation = atof(value);
		break;

	case NMEA_GPGGA_UNDULATION_UNIT:
		/* Parse undulation unit */
		data->undulation_unit = *value;
		break;

	default:
		break;
	}

	return 0;
}

/* ================= Парсер GPRMC (из gprmc.c) ================= */

/* Value indexes */
#define NMEA_GPRMC_TIME			0
#define NMEA_GPRMC_STATUS		1
#define NMEA_GPRMC_LATITUDE		2
#define NMEA_GPRMC_LATITUDE_CARDINAL	3
#define NMEA_GPRMC_LONGITUDE		4
#define NMEA_GPRMC_LONGITUDE_CARDINAL	5
#define NMEA_GPRMC_GNDSPD_KNOTS		6
#define NMEA_GPRMC_TRUECOURSE_DEG	7
#define NMEA_GPRMC_DATE			8
#define NMEA_GPRMC_MAGVAR_DEG		9
#define NMEA_GPRMC_MAGVAR_CARDINAL	10

static int nmea_gprmc_init(nmea_parser_s *parser);
static int nmea_gprmc_allocate_data(nmea_parser_s *parser);
static int nmea_gprmc_set_default(nmea_parser_s *parser);
static int nmea_gprmc_free_data(nmea_s *data);
static int nmea_gprmc_parse(nmea_parser_s *parser, char *value, int val_index);

static int
nmea_gprmc_init(nmea_parser_s *parser)
{
	/* Declare what sentence type to parse */
	NMEA_PARSER_TYPE(parser, NMEA_GPRMC);
	NMEA_PARSER_PREFIX(parser, "GPRMC");
	return 0;
}

static int
nmea_gprmc_allocate_data(nmea_parser_s *parser)
{
	parser->data = (nmea_s *)malloc(sizeof(nmea_gprmc_s));
	if (NULL == parser->data) {
		return -1;
	}

	return 0;
}

static int
nmea_gprmc_set_default(nmea_parser_s *parser)
{
	memset(parser->data, 0, sizeof(nmea_gprmc_s));
	return 0;
}

static int
nmea_gprmc_free_data(nmea_s *data)
{
	free(data);
	return 0;
}

static int
nmea_gprmc_parse(nmea_parser_s *parser, char *value, int val_index)
{
	nmea_gprmc_s *data = (nmea_gprmc_s *)parser->data;

	switch (val_index) {
	case NMEA_GPRMC_TIME:
		/* Parse time */
		if (-1 == nmea_time_parse(value, &data->date_time)) {
			return -1;
		}
		break;

	case NMEA_GPRMC_DATE:
		/* Parse date */
		if (-1 == nmea_date_parse(value, &data->date_time)) {
			return -1;
		}
		break;

	case NMEA_GPRMC_STATUS:
		/* Parse status */
		if (*value == 'A') {
			data->valid = true;
		} else if (*value == 'V') {
			data->valid = false;
		} else {
			return -1;
		}
		break;

	case NMEA_GPRMC_LATITUDE:
		/* Parse latitude */
		if (-1 == nmea_position_parse(value, &data->latitude)) {
			return -1;
		}
		break;

	case NMEA_GPRMC_LATITUDE_CARDINAL:
		/* Parse cardinal direction */
		data->latitude.cardinal = nmea_cardinal_direction_parse(value);
		if (NMEA_CARDINAL_DIR_UNKNOWN == data->latitude.cardinal) {
			return -1;
		}
		break;

	case NMEA_GPRMC_LONGITUDE:
		/* Parse longitude */
		if (-1 == nmea_position_parse(value, &data->longitude)) {
			return -1;
		}
		break;

	case NMEA_GPRMC_LONGITUDE_CARDINAL:
		/* Parse cardinal direction */
		data->longitude.cardinal = nmea_cardinal_direction_parse(value);
		if (NMEA_CARDINAL_DIR_UNKNOWN == data->longitude.cardinal) {
			return -1;
		}
		break;

	case NMEA_GPRMC_GNDSPD_KNOTS:
		/* Parse Ground speed, knots */
		data->gndspd_knots = strtod(value, NULL);
		break;

	case NMEA_GPRMC_TRUECOURSE_DEG:
		/* Parse the true course, degrees */
		data->track_deg = strtod(value, NULL);
		break;

	case NMEA_GPRMC_MAGVAR_DEG:
		/* Parse the Magnetic variation, degrees */
		data->magvar_deg = strtod(value, NULL);
		break;

	case NMEA_GPRMC_MAGVAR_CARDINAL:
		/* Parse the Magnetic variation direction, only E and W are valid */
		data->magvar_cardinal = nmea_cardinal_direction_parse(value);
		if (NMEA_CARDINAL_DIR_UNKNOWN == data->magvar_cardinal) {
			data->magvar_cardinal = NMEA_CARDINAL_DIR_UNKNOWN;
			return -1;
		}
		break;

	default:
		break;
	}

	return 0;
}

/* ================= Парсер GPGSA (из gpgsa.c) ================= */

/* Value indexes */
#define NMEA_GPGSA_MODE 0
#define NMEA_GPGSA_FIXTYPE 1
#define NMEA_GPGSA_SATID_00 2
#define NMEA_GPGSA_SATID_01 3
#define NMEA_GPGSA_SATID_02 4
#define NMEA_GPGSA_SATID_03 5
#define NMEA_GPGSA_SATID_04 6
#define NMEA_GPGSA_SATID_05 7
#define NMEA_GPGSA_SATID_06 8
#define NMEA_GPGSA_SATID_07 9
#define NMEA_GPGSA_SATID_08 10
#define NMEA_GPGSA_SATID_09 11
#define NMEA_GPGSA_SATID_10 12
#define NMEA_GPGSA_SATID_11 13
#define NMEA_GPGSA_PDOP 14
#define NMEA_GPGSA_HDOP 15
#define NMEA_GPGSA_VDOP 16

static int nmea_gpgsa_init(nmea_parser_s *parser);
static int nmea_gpgsa_allocate_data(nmea_parser_s *parser);
static int nmea_gpgsa_set_default(nmea_parser_s *parser);
static int nmea_gpgsa_free_data(nmea_s *data);
static int nmea_gpgsa_parse(nmea_parser_s *parser, char *value, int val_index);

static int
nmea_gpgsa_init(nmea_parser_s *parser)
{
	/* Declare what sentence type to parse */
	NMEA_PARSER_TYPE(parser, NMEA_GPGSA);
	NMEA_PARSER_PREFIX(parser, "GPGSA");
	return 0;
}

static int
nmea_gpgsa_allocate_data(nmea_parser_s *parser)
{
	parser->data = (nmea_s *)malloc(sizeof(nmea_gpgsa_s));
	if (NULL == parser->data) {
		return -1;
	}

	return 0;
}

static int
nmea_gpgsa_set_default(nmea_parser_s *parser)
{
	memset(parser->data, 0, sizeof(nmea_gpgsa_s));
	return 0;
}

static int
nmea_gpgsa_free_data(nmea_s *data)
{
	free(data);
	return 0;
}

static int
nmea_gpgsa_parse(nmea_parser_s *parser, char *value, int val_index)
{
	nmea_gpgsa_s *data = (nmea_gpgsa_s *)parser->data;

	switch (val_index) {
	case NMEA_GPGSA_MODE:
		data->mode = *value;
		if (('M' != toupper(data->mode)) && ('A' != toupper(data->mode))) {
			return -1;
		}
		break;
	case NMEA_GPGSA_FIXTYPE:
		data->fixtype = strtol(value, NULL, 10);
		if ((1 != data->fixtype) && (2 != data->fixtype) && (3 != data->fixtype)) {
			return -1;
		}
		break;
	case NMEA_GPGSA_SATID_00:
		data->satID_00 = strtol(value, NULL, 10);
		break;
	case NMEA_GPGSA_SATID_01:
		data->satID_01 = strtol(value, NULL, 10);
		break;
	case NMEA_GPGSA_SATID_02:
		data->satID_02 = strtol(value, NULL, 10);
		break;
	case NMEA_GPGSA_SATID_03:
		data->satID_03 = strtol(value, NULL, 10);
		break;
	case NMEA_GPGSA_SATID_04:
		data->satID_04 = strtol(value, NULL, 10);
		break;
	case NMEA_GPGSA_SATID_05:
		data->satID_05 = strtol(value, NULL, 10);
		break;
	case NMEA_GPGSA_SATID_06:
		data->satID_06 = strtol(value, NULL, 10);
		break;
	case NMEA_GPGSA_SATID_07:
		data->satID_07 = strtol(value, NULL, 10);
		break;
	case NMEA_GPGSA_SATID_08:
		data->satID_08 = strtol(value, NULL, 10);
		break;
	case NMEA_GPGSA_SATID_09:
		data->satID_09 = strtol(value, NULL, 10);
		break;
	case NMEA_GPGSA_SATID_10:
		data->satID_10 = strtol(value, NULL, 10);
		break;
	case NMEA_GPGSA_SATID_11:
		data->satID_11 = strtol(value, NULL, 10);
		break;
	case NMEA_GPGSA_PDOP:
		data->pdop = strtod(value, NULL);
		break;
	case NMEA_GPGSA_HDOP:
		data->hdop = strtod(value, NULL);
		break;
	case NMEA_GPGSA_VDOP:
		data->vdop = strtod(value, NULL);
		break;
	default:
		break;
	}

	return 0;
}

/* ============ Реестр парсеров (из parser_static.c, только 3 типа) ============ */

#define PARSER_COUNT 3

static nmea_parser_module_s parsers[PARSER_COUNT];

static void
nmea_load_parsers()
{
	int i = 0;
	nmea_parser_module_s *parser;

	parser = &(parsers[i]);
	parser->allocate_data = nmea_gpgga_allocate_data;
	parser->set_default = nmea_gpgga_set_default;
	parser->free_data = nmea_gpgga_free_data;
	parser->parse = nmea_gpgga_parse;
	nmea_gpgga_init((nmea_parser_s *)parser);
	i++;

	parser = &(parsers[i]);
	parser->allocate_data = nmea_gprmc_allocate_data;
	parser->set_default = nmea_gprmc_set_default;
	parser->free_data = nmea_gprmc_free_data;
	parser->parse = nmea_gprmc_parse;
	nmea_gprmc_init((nmea_parser_s *)parser);
	i++;

	parser = &(parsers[i]);
	parser->allocate_data = nmea_gpgsa_allocate_data;
	parser->set_default = nmea_gpgsa_set_default;
	parser->free_data = nmea_gpgsa_free_data;
	parser->parse = nmea_gpgsa_parse;
	nmea_gpgsa_init((nmea_parser_s *)parser);
}

static nmea_parser_module_s *
nmea_get_parser_by_type(nmea_t type)
{
	int i;

	for (i = 0; i < PARSER_COUNT; i++) {
		if (type == parsers[i].parser.type) {
			return &(parsers[i]);
		}
	}

	return (nmea_parser_module_s *)NULL;
}

static nmea_parser_module_s *
nmea_get_parser_by_sentence(const char *sentence)
{
	int i;

	for (i = 0; i < PARSER_COUNT; i++) {
		/* compare only the sentence ID, ignore the talker ID */
		if (0 == strncmp(sentence + 3, parsers[i].parser.type_word + 2, NMEA_PREFIX_LENGTH - 2)) {
			return &(parsers[i]);
		}
	}

	return (nmea_parser_module_s *)NULL;
}

static void
ensure_parsers_loaded()
{
	static bool loaded = false;

	if (!loaded) {
		nmea_load_parsers();
		loaded = true;
	}
}

/* =============== Основной код (из nmea.c) =============== */

#define ARRAY_LENGTH(a) (sizeof(a) / sizeof((a)[0]))

/**
 * Check if a value is not NULL and not empty.
 *
 * Returns 0 if set, otherwise -1.
 */
static int
_is_value_set(const char *value)
{
	if (NULL == value || '\0' == *value) {
		return -1;
	}

	return 0;
}

/**
 * Crop a sentence from the type word and checksum.
 *
 * The type word at the beginning along with the dollar sign ($) will be
 * removed. If there is a checksum, it will also be removed. The two end
 * characters (usually <CR><LF>) will not be included in the new string.
 *
 * sentence is a validated NMEA sentence string.
 * length is the char length of the sentence string.
 *
 * Returns pointer (char *) to the new string.
 */
static char *
_crop_sentence(char *sentence, size_t length)
{
	/* Skip type word, 7 characters (including $ and ,) */
	sentence += NMEA_PREFIX_LENGTH + 2;

	/* Null terminate before end of line/sentence, 2 characters */
	sentence[length - 9] = '\0';

	/* Remove checksum, if there is one */
	if ('*' == sentence[length - 12]) {
		sentence[length - 12] = '\0';
	}

	return sentence;
}

/**
 * Splits a string by comma.
 *
 * string is the string to split, will be manipulated. Needs to be
 *        null-terminated.
 * values is a char pointer array that will be filled with pointers to the
 *        splitted values in the string.
 * max_values is the maximum number of values to be parsed.
 *
 * Returns the number of values found in string.
 */
static int
_split_string_by_comma(char *string, char **values, int max_values)
{
	int i = 0;

	values[i++] = string;
	while (i < max_values && NULL != (string = strchr(string, ','))) {
		*string = '\0';
		values[i++] = ++string;
	}

	return i;
}

static nmea_t
nmea_get_type(const char *sentence)
{
	if (NULL == sentence) {
		return NMEA_UNKNOWN;
	}
	ensure_parsers_loaded();
	nmea_parser_module_s *parser = nmea_get_parser_by_sentence(sentence);
	if (NULL == parser) {
		return NMEA_UNKNOWN;
	}

	return parser->parser.type;
}

static uint8_t
nmea_get_checksum(const char *sentence)
{
	const char *n = sentence + 1;
	uint8_t chk = 0;

	/* While current char isn't '*' or sentence ending (newline) */
	while ('*' != *n && NMEA_END_CHAR_1 != *n && '\0' != *n) {
		chk ^= (uint8_t)*n;
		n++;
	}

	return chk;
}

static int
nmea_has_checksum(const char *sentence, size_t length)
{
	if ('*' == sentence[length - 5]) {
		return 0;
	}

	return -1;
}

static int
nmea_validate(const char *sentence, size_t length, int check_checksum)
{
	const char *n;

	/* should have atleast 9 characters */
	if (9 > length) {
		return -1;
	}

	/* should be less or equal to 82 characters */
	if (NMEA_MAX_LENGTH < length) {
		return -1;
	}

	/* should start with $ */
	if ('$' != *sentence) {
		return -1;
	}

	/* should end with \r\n, or other... */
	if (NMEA_END_CHAR_2 != sentence[length - 1] || NMEA_END_CHAR_1 != sentence[length - 2]) {
		return -1;
	}

	/* should have a 5 letter, uppercase word */
	n = sentence;
	while (++n < sentence + 6) {
		if (*n < 'A' || *n > 'Z') {
			/* not uppercase letter */
			return -1;
		}
	}

	/* should have a comma after the type word */
	if (',' != sentence[6]) {
		return -1;
	}

	/* check for checksum */
	if (1 == check_checksum && 0 == nmea_has_checksum(sentence, length)) {
		uint8_t actual_chk;
		uint8_t expected_chk;
		char checksum[3];

		checksum[0] = sentence[length - 4];
		checksum[1] = sentence[length - 3];
		checksum[2] = '\0';
		actual_chk = nmea_get_checksum(sentence);
		expected_chk = (uint8_t)strtol(checksum, NULL, 16);
		if (expected_chk != actual_chk) {
			return -1;
		}
	}

	return 0;
}

void
nmea_free(nmea_s *data)
{
	nmea_parser_module_s *parser;

	if (NULL == data) {
		return;
	}

	ensure_parsers_loaded();
	parser = nmea_get_parser_by_type(data->type);
	if (NULL == parser) {
		return;
	}

	parser->free_data(data);
}

nmea_s *
nmea_parse(char *sentence, size_t length, int check_checksum)
{
	unsigned int n_vals, val_index;
	char *value, *val_string;
	char *values[255];
	nmea_parser_module_s *parser;
	nmea_t type;

	/* Validate sentence string */
	if (-1 == nmea_validate(sentence, length, check_checksum)) {
		return (nmea_s *)NULL;
	}

	ensure_parsers_loaded();
	type = nmea_get_type(sentence);
	if (NMEA_UNKNOWN == type) {
		return (nmea_s *)NULL;
	}

	/* Crop sentence from type word and checksum */
	val_string = _crop_sentence(sentence, length);
	if (NULL == val_string) {
		return (nmea_s *)NULL;
	}

	/* Split the sentence into values */
	n_vals = _split_string_by_comma(val_string, values, ARRAY_LENGTH(values));
	if (0 == n_vals) {
		return (nmea_s *)NULL;
	}

	/* Get the right parser */
	parser = nmea_get_parser_by_type(type);
	if (NULL == parser) {
		return (nmea_s *)NULL;
	}

	/* Allocate memory for parsed data */
	parser->allocate_data((nmea_parser_s *)parser);
	if (NULL == parser->parser.data) {
		return (nmea_s *)NULL;
	}

	/* Set default values */
	parser->set_default((nmea_parser_s *)parser);
	parser->errors = 0;

	/* Loop through the values and parse them... */
	for (val_index = 0; val_index < n_vals; val_index++) {
		value = values[val_index];
		if (-1 == _is_value_set(value)) {
			continue;
		}

		if (-1 == parser->parse((nmea_parser_s *)parser, value, val_index)) {
			parser->errors++;
		}
	}

	parser->parser.data->type = type;
	parser->parser.data->errors = parser->errors;

	return parser->parser.data;
}