#ifndef NMEA_H
#define NMEA_H

#ifdef __cplusplus
extern "C" {
#endif


//------------------- DEFINES -----------------------------
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <time.h>
#include <math.h>

#define NMEA_MAX_LENGTH				256
#define NMEA_LEN					16
#define FREQ_LEN					14
#define BAUD_LEN					28


//------------------- VARIABLES ---------------------------
enum nmea_sentence_id {
	NMEA_INVALID = -1,
	NMEA_UNKNOWN = 0,
	NMEA_SENTENCE_RMC,
	NMEA_SENTENCE_GGA,
	NMEA_SENTENCE_GSA,
	NMEA_SENTENCE_GLL,
	NMEA_SENTENCE_GST,
	NMEA_SENTENCE_GSV,
	NMEA_SENTENCE_VTG,
	NMEA_SENTENCE_ZDA,
};

struct nmea_float {
	int_least32_t value;
	int_least32_t scale;
};

struct nmea_date {
	int day;
	int month;
	int year;
};

struct nmea_time {
	int hours;
	int minutes;
	int seconds;
	int microseconds;
};

struct nmea_sentence_rmc {
	struct nmea_time time;
	bool valid;
	struct nmea_float latitude;
	struct nmea_float longitude;
	struct nmea_float speed;
	struct nmea_float course;
	struct nmea_date date;
	struct nmea_float variation;
};

struct nmea_sentence_gga {
	struct nmea_time time;
	struct nmea_float latitude;
	struct nmea_float longitude;
	int fix_quality;
	int satellites_tracked;
	struct nmea_float hdop;
	struct nmea_float altitude; char altitude_units;
	struct nmea_float height; char height_units;
	struct nmea_float dgps_age;
};

enum nmea_gll_status {
	NMEA_GLL_STATUS_DATA_VALID = 'A',
	NMEA_GLL_STATUS_DATA_NOT_VALID = 'V',
};

enum nmea_faa_mode {
	NMEA_FAA_MODE_AUTONOMOUS = 'A',
	NMEA_FAA_MODE_DIFFERENTIAL = 'D',
	NMEA_FAA_MODE_ESTIMATED = 'E',
	NMEA_FAA_MODE_MANUAL = 'M',
	NMEA_FAA_MODE_SIMULATED = 'S',
	NMEA_FAA_MODE_NOT_VALID = 'N',
	NMEA_FAA_MODE_PRECISE = 'P',
};

struct nmea_sentence_gll {
	struct nmea_float latitude;
	struct nmea_float longitude;
	struct nmea_time time;
	char status;
	char mode;
};

struct nmea_sentence_gst {
	struct nmea_time time;
	struct nmea_float rms_deviation;
	struct nmea_float semi_major_deviation;
	struct nmea_float semi_minor_deviation;
	struct nmea_float semi_major_orientation;
	struct nmea_float latitude_error_deviation;
	struct nmea_float longitude_error_deviation;
	struct nmea_float altitude_error_deviation;
};

enum nmea_gsa_mode {
	NMEA_GPGSA_MODE_AUTO = 'A',
	NMEA_GPGSA_MODE_FORCED = 'M',
};

enum nmea_gsa_fix_type {
	NMEA_GPGSA_FIX_NONE = 1,
	NMEA_GPGSA_FIX_2D = 2,
	NMEA_GPGSA_FIX_3D = 3,
};

struct nmea_sentence_gsa {
	char mode;
	int fix_type;
	int sats[12];
	struct nmea_float pdop;
	struct nmea_float hdop;
	struct nmea_float vdop;
};

struct nmea_sat_info {
	int nr;
	int elevation;
	int azimuth;
	int snr;
};

struct nmea_sentence_gsv {
	int total_msgs;
	int msg_nr;
	int total_sats;
	struct nmea_sat_info sats[4];
};

struct nmea_sentence_vtg {
	struct nmea_float true_track_degrees;
	struct nmea_float magnetic_track_degrees;
	struct nmea_float speed_knots;
	struct nmea_float speed_kph;
	enum nmea_faa_mode faa_mode;
};

struct nmea_sentence_zda {
	struct nmea_time time;
	struct nmea_date date;
	int hour_offset;
	int minute_offset;
};

extern uint8_t turn_Off_GPGGA[NMEA_LEN];
extern uint8_t turn_Off_GPGLL[NMEA_LEN];
extern uint8_t turn_Off_GPGSA[NMEA_LEN];
extern uint8_t turn_Off_GPGLV[NMEA_LEN];
extern uint8_t turn_Off_GPRMC[NMEA_LEN];
extern uint8_t turn_Off_GPVTG[NMEA_LEN];
// <<---------------------------------------------------------------------------//on packets
extern uint8_t turn_On_GPGGA[NMEA_LEN];
extern uint8_t turn_On_GPGLL[NMEA_LEN];
extern uint8_t turn_On_GPGSA[NMEA_LEN];
extern uint8_t turn_On_GPGLV[NMEA_LEN];
extern uint8_t turn_On_GPRMC[NMEA_LEN];
extern uint8_t turn_On_GPVTG[NMEA_LEN];
extern uint8_t updateFreq[FREQ_LEN];
extern uint8_t changeBaud[BAUD_LEN];

//------------------- FUNCTIONS ---------------------------
/**
 * Подсчет контрольной суммы
 */
uint8_t nmea_checksum(const char *sentence);

/**
 * Проверяет контрольную сумму и корректнеость данных
 */
bool nmea_check(const char *sentence, bool strict);

/**
 * Определяет идентефикатор
 */
bool nmea_talker_id(char talker[3], const char *sentence);

/**
 * Определяет идентефикатор
 */
enum nmea_sentence_id nmea_sentence_id(const char *sentence, bool strict);

/**
 * Сканер данных NMEA. Поддерживаемые форматы:
 * c - символ (char *)
 * d - направление, возвращает as 1/-1, дефолт 0 (int *)
 * f - дробный, возвращает значение + размер (int *, int *)
 * i - десятичное, дефолт 0 (int *)
 * s - строка (char *)
 * t - идентефикатор и тип (char *)
 * T - дата/время (int *, int *, int *)
 * Возвращает true в случае успеха
 */
bool nmea_scan(const char *sentence, const char *format, ...);

/*
 * Парсинг типов данных. Возвращает true в случае успеха
 */
bool nmea_parse_rmc(struct nmea_sentence_rmc *frame, const char *sentence);
bool nmea_parse_gga(struct nmea_sentence_gga *frame, const char *sentence);
bool nmea_parse_gsa(struct nmea_sentence_gsa *frame, const char *sentence);
bool nmea_parse_gll(struct nmea_sentence_gll *frame, const char *sentence);
bool nmea_parse_gst(struct nmea_sentence_gst *frame, const char *sentence);
bool nmea_parse_gsv(struct nmea_sentence_gsv *frame, const char *sentence);
bool nmea_parse_vtg(struct nmea_sentence_vtg *frame, const char *sentence);
bool nmea_parse_zda(struct nmea_sentence_zda *frame, const char *sentence);

/**
 * Конвертер GPS UTC даты/времени в UNIX timestamp.
 */
int nmea_gettime(struct timespec *ts, const struct nmea_date *date, const struct nmea_time *time_);

/**
 * Меняет размер значения
 */
static inline int_least32_t nmea_rescale(struct nmea_float *f, int_least32_t new_scale)
{
	if (f->scale == 0)
		return 0;
	if (f->scale == new_scale)
		return f->value;
	if (f->scale > new_scale)
		return (f->value + ((f->value > 0) - (f->value < 0)) * f->scale/new_scale/2) / (f->scale/new_scale);
	else
		return f->value * (new_scale/f->scale);
}

/**
 * Конвертер чисел с фиксированной точкой в числа с плавающей
 * Возвращает NaN для "непонятных" значений.
 */
static inline float nmea_tofloat(struct nmea_float *f)
{
	if (f->scale == 0)
		return NAN;
	return (float) f->value / (float) f->scale;
}

/**
 * Конвертер чисел с фиксированной точкой в числа с плавающей
 * Возвращает NaN для "непонятных" значений.
 */
static inline void nmea_ftoa(struct nmea_float *f, char* buff)
{
	uint8_t r = 0;
	uint32_t scale = f->scale;
	if( scale > 0 ){
		while( scale > 10 ){
			scale /= 10;
			r++;
		}
		r++;
	}

	itoa( f->value, buff, 10 );

	if( r ){
		uint8_t len = strlen( buff );
		for( uint8_t i = len; i > 0; i-- ){
			buff[ i + 1 ] = buff[ i ];
			if( r > 0 ){
				r--;
			}else{
				buff[ i ] = '.';
				break;
			}
		}
	}
}

/**
 * Конвертер сырых координат в число с плавающей точкой DD.DDD....
 * Возвращает NaN для "непонятных" значений.
 */
static inline float nmea_tocoord(const struct nmea_float *f)
{
	if (f->scale == 0)
		return NAN;
	int_least32_t degrees = f->value / (f->scale * 100);
	int_least32_t minutes = f->value % (f->scale * 100);
	return (float) degrees + (float) minutes / (60 * f->scale);
}

#ifdef __cplusplus
}
#endif


#endif /* NMEA_H */

