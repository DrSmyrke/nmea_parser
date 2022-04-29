#include "nmea.h"



//------------------- DEFINES -----------------------------
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <time.h>

#define timegm _mkgmtime

/*		Example

char buff[NMEA_MAX_LENGTH];
readLine(buff, NMEA_MAX_LENGTH);
switch (nmea_sentence_id(buff, false)) {
	case NMEA_SENTENCE_GGA:
		m_sats.good = (nmea_parse_gga(&gga_frame, buff)) ? gga_frame.satellites_tracked : 0 ;
	break;
	case NMEA_SENTENCE_GSV:
		m_sats.view = (nmea_parse_gsv(&gsv_frame, buff)) ? gsv_frame.total_sats : 0;
	break;
	case NMEA_SENTENCE_RMC:
		if (nmea_parse_rmc(&rmc_frame, buff)) {
			std::string str = std::to_string(rmc_frame.time.hours) + ":" + std::to_string(rmc_frame.time.minutes) + ":" + std::to_string(rmc_frame.time.seconds);
			//��������� ������� ���������� � ���������
			SYSTEMTIME st;
			GetSystemTime(&st);
			st.wHour = rmc_frame.time.hours;
			st.wMinute = rmc_frame.time.minutes;
			st.wSecond = rmc_frame.time.seconds;
			st.wYear = 2000 + rmc_frame.date.year;
			st.wMonth = rmc_frame.date.month;
			st.wDay = rmc_frame.date.day;
			SetSystemTime(&st);
		}
	break;
}


*/


//------------------- FUNCTIONS ------------------------
static int hex2int(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

uint8_t nmea_checksum(const char *sentence)
{
    if (*sentence == '$')
        sentence++;

    uint8_t checksum = 0x00;

    while (*sentence && *sentence != '*')
        checksum ^= *sentence++;

    return checksum;
}

bool nmea_check(const char *sentence, bool strict)
{
    uint8_t checksum = 0x00;

    // ���� ������ ������ �������� ������
    if (strlen(sentence) > NMEA_MAX_LENGTH + 3)
        return false;

    // ���� ������ �� ���������� � "$".
    if (*sentence++ != '$')
        return false;

    while (*sentence && *sentence != '*' && isprint((unsigned char) *sentence))
        checksum ^= *sentence++;

    // ���� ������������ ����������� �����
    if (*sentence == '*') {
        sentence++;
        int upper = hex2int(*sentence++);
        if (upper == -1)
            return false;
        int lower = hex2int(*sentence++);
        if (lower == -1)
            return false;
        int expected = upper << 4 | lower;

        if (checksum != expected)
            return false;
    } else if (strict) {
        return false;
    }

    if (*sentence && strcmp(sentence, "\n") && strcmp(sentence, "\r\n"))
        return false;

    return true;
}

static inline bool nmea_isfield(char c) {
    return isprint((unsigned char) c) && c != ',' && c != '*';
}

bool nmea_scan(const char *sentence, const char *format, ...)
{
    bool result = false;
    bool optional = false;
    va_list ap;
    va_start(ap, format);

    const char *field = sentence;
#define next_field() \
    do { \
        while (nmea_isfield(*sentence)) \
            sentence++; \
        if (*sentence == ',') { \
            sentence++; \
            field = sentence; \
        } else { \
            field = NULL; \
        } \
    } while (0)

    while (*format) {
        char type = *format++;

        if (type == ';') {
            optional = true;
            continue;
        }

        if (!field && !optional) {
            goto parse_error;
        }

        switch (type) {
            case 'c': { // Single character field (char).
                char value = '\0';

                if (field && nmea_isfield(*field))
                    value = *field;

                *va_arg(ap, char *) = value;
            } break;

            case 'd': { // Single character direction field (int).
                int value = 0;

                if (field && nmea_isfield(*field)) {
                    switch (*field) {
                        case 'N':
                        case 'E':
                            value = 1;
                            break;
                        case 'S':
                        case 'W':
                            value = -1;
                            break;
                        default:
                            goto parse_error;
                    }
                }

                *va_arg(ap, int *) = value;
            } break;

            case 'f': { // Fractional value with scale (struct nmea_float).
                int sign = 0;
                int_least32_t value = -1;
                int_least32_t scale = 0;

                if (field) {
                    while (nmea_isfield(*field)) {
                        if (*field == '+' && !sign && value == -1) {
                            sign = 1;
                        } else if (*field == '-' && !sign && value == -1) {
                            sign = -1;
                        } else if (isdigit((unsigned char) *field)) {
                            int digit = *field - '0';
                            if (value == -1)
                                value = 0;
                            if (value > (INT_LEAST32_MAX-digit) / 10) {
                                if (scale) {
                                    break;
                                } else {
                                    goto parse_error;
                                }
                            }
                            value = (10 * value) + digit;
                            if (scale)
                                scale *= 10;
                        } else if (*field == '.' && scale == 0) {
                            scale = 1;
                        } else if (*field == ' ') {
                            if (sign != 0 || value != -1 || scale != 0)
                                goto parse_error;
                        } else {
                            goto parse_error;
                        }
                        field++;
                    }
                }

                if ((sign || scale) && value == -1)
                    goto parse_error;

                if (value == -1) {
                    value = 0;
                    scale = 0;
                } else if (scale == 0) {
                    scale = 1;
                }
                if (sign)
                    value *= sign;

				struct nmea_float test;
				test.value = value;
				test.scale = scale;
                //*va_arg(ap, struct nmea_float *) = (struct nmea_float) {value, scale};
                *va_arg(ap, struct nmea_float *) = test;
            } break;

            case 'i': { // Integer value, default 0 (int).
                int value = 0;

                if (field) {
                    char *endptr;
                    value = strtol(field, &endptr, 10);
                    if (nmea_isfield(*endptr))
                        goto parse_error;
                }

                *va_arg(ap, int *) = value;
            } break;

            case 's': { // String value (char *).
                char *buf = va_arg(ap, char *);

                if (field) {
                    while (nmea_isfield(*field))
                        *buf++ = *field++;
                }

                *buf = '\0';
            } break;

            case 't': { // NMEA talker+sentence identifier (char *).
                // ��� ���� �����������
                if (!field)
                    goto parse_error;

                if (field[0] != '$')
                    goto parse_error;
                for (int f=0; f<5; f++)
                    if (!nmea_isfield(field[1+f]))
                        goto parse_error;

                char *buf = va_arg(ap, char *);
                memcpy(buf, field+1, 5);
                buf[5] = '\0';
            } break;

            case 'D': { // Date (int, int, int), -1 if empty.
                struct nmea_date *date = va_arg(ap, struct nmea_date *);

                int d = -1, m = -1, y = -1;

                if (field && nmea_isfield(*field)) {
                    // ������ 6 ����.
                    for (int f=0; f<6; f++)
                        if (!isdigit((unsigned char) field[f]))
                            goto parse_error;

                    char dArr[] = {field[0], field[1], '\0'};
                    char mArr[] = {field[2], field[3], '\0'};
                    char yArr[] = {field[4], field[5], '\0'};
                    d = strtol(dArr, NULL, 10);
                    m = strtol(mArr, NULL, 10);
                    y = strtol(yArr, NULL, 10);
                }

                date->day = d;
                date->month = m;
                date->year = y;
            } break;

            case 'T': { // Time (int, int, int, int), -1 if empty.
                struct nmea_time *time_ = va_arg(ap, struct nmea_time *);

                int h = -1, i = -1, s = -1, u = -1;

                if (field && nmea_isfield(*field)) {
                    // ����������� ����������: ����� �����
                    for (int f=0; f<6; f++)
                        if (!isdigit((unsigned char) field[f]))
                            goto parse_error;

                    char hArr[] = {field[0], field[1], '\0'};
                    char iArr[] = {field[2], field[3], '\0'};
                    char sArr[] = {field[4], field[5], '\0'};
                    h = strtol(hArr, NULL, 10);
                    i = strtol(iArr, NULL, 10);
                    s = strtol(sArr, NULL, 10);
                    field += 6;

                    // ������� ����� ����������� ��� ������������
                    if (*field++ == '.') {
                        uint32_t value = 0;
                        uint32_t scale = 1000000LU;
                        while (isdigit((unsigned char) *field) && scale > 1) {
                            value = (value * 10) + (*field++ - '0');
                            scale /= 10;
                        }
                        u = value * scale;
                    } else {
                        u = 0;
                    }
                }

                time_->hours = h;
                time_->minutes = i;
                time_->seconds = s;
                time_->microseconds = u;
            } break;

            case '_': { // ������������� ����.
            } break;

            default: {
                goto parse_error;
            }
        }

        next_field();
    }

    result = true;

parse_error:
    va_end(ap);
    return result;
}

bool nmea_talker_id(char talker[3], const char *sentence)
{
    char type[6];
    if (!nmea_scan(sentence, "t", type))
        return false;

    talker[0] = type[0];
    talker[1] = type[1];
    talker[2] = '\0';

    return true;
}

enum nmea_sentence_id nmea_sentence_id(const char *sentence, bool strict)
{
    if (!nmea_check(sentence, strict))
        return NMEA_INVALID;

    char type[6];
    if (!nmea_scan(sentence, "t", type))
        return NMEA_INVALID;

    if (!strcmp(type+2, "RMC"))
        return NMEA_SENTENCE_RMC;
    if (!strcmp(type+2, "GGA"))
        return NMEA_SENTENCE_GGA;
    if (!strcmp(type+2, "GSA"))
        return NMEA_SENTENCE_GSA;
    if (!strcmp(type+2, "GLL"))
        return NMEA_SENTENCE_GLL;
    if (!strcmp(type+2, "GST"))
        return NMEA_SENTENCE_GST;
    if (!strcmp(type+2, "GSV"))
        return NMEA_SENTENCE_GSV;
    if (!strcmp(type+2, "VTG"))
        return NMEA_SENTENCE_VTG;
    if (!strcmp(type+2, "ZDA"))
        return NMEA_SENTENCE_ZDA;

    return NMEA_UNKNOWN;
}

bool nmea_parse_rmc(struct nmea_sentence_rmc *frame, const char *sentence)
{
    // $GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62
    char type[6];
    char validity;
    int latitude_direction;
    int longitude_direction;
    int variation_direction;
    if (!nmea_scan(sentence, "tTcfdfdffDfd",
            type,
            &frame->time,
            &validity,
            &frame->latitude, &latitude_direction,
            &frame->longitude, &longitude_direction,
            &frame->speed,
            &frame->course,
            &frame->date,
            &frame->variation, &variation_direction))
        return false;
    if (strcmp(type+2, "RMC"))
        return false;

    frame->valid = (validity == 'A');
    frame->latitude.value *= latitude_direction;
    frame->longitude.value *= longitude_direction;
    frame->variation.value *= variation_direction;

    return true;
}

bool nmea_parse_gga(struct nmea_sentence_gga *frame, const char *sentence)
{
    // $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    char type[6];
    int latitude_direction;
    int longitude_direction;

    if (!nmea_scan(sentence, "tTfdfdiiffcfcf_",
            type,
            &frame->time,
            &frame->latitude, &latitude_direction,
            &frame->longitude, &longitude_direction,
            &frame->fix_quality,
            &frame->satellites_tracked,
            &frame->hdop,
            &frame->altitude, &frame->altitude_units,
            &frame->height, &frame->height_units,
            &frame->dgps_age))
        return false;
    if (strcmp(type+2, "GGA"))
        return false;

    frame->latitude.value *= latitude_direction;
    frame->longitude.value *= longitude_direction;

    return true;
}

bool nmea_parse_gsa(struct nmea_sentence_gsa *frame, const char *sentence)
{
    // $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39
    char type[6];

    if (!nmea_scan(sentence, "tciiiiiiiiiiiiifff",
            type,
            &frame->mode,
            &frame->fix_type,
            &frame->sats[0],
            &frame->sats[1],
            &frame->sats[2],
            &frame->sats[3],
            &frame->sats[4],
            &frame->sats[5],
            &frame->sats[6],
            &frame->sats[7],
            &frame->sats[8],
            &frame->sats[9],
            &frame->sats[10],
            &frame->sats[11],
            &frame->pdop,
            &frame->hdop,
            &frame->vdop))
        return false;
    if (strcmp(type+2, "GSA"))
        return false;

    return true;
}

bool nmea_parse_gll(struct nmea_sentence_gll *frame, const char *sentence)
{
    // $GPGLL,3723.2475,N,12158.3416,W,161229.487,A,A*41$;
    char type[6];
    int latitude_direction;
    int longitude_direction;

    if (!nmea_scan(sentence, "tfdfdTc;c",
            type,
            &frame->latitude, &latitude_direction,
            &frame->longitude, &longitude_direction,
            &frame->time,
            &frame->status,
            &frame->mode))
        return false;
    if (strcmp(type+2, "GLL"))
        return false;

    frame->latitude.value *= latitude_direction;
    frame->longitude.value *= longitude_direction;

    return true;
}

bool nmea_parse_gst(struct nmea_sentence_gst *frame, const char *sentence)
{
    // $GPGST,024603.00,3.2,6.6,4.7,47.3,5.8,5.6,22.0*58
    char type[6];

    if (!nmea_scan(sentence, "tTfffffff",
            type,
            &frame->time,
            &frame->rms_deviation,
            &frame->semi_major_deviation,
            &frame->semi_minor_deviation,
            &frame->semi_major_orientation,
            &frame->latitude_error_deviation,
            &frame->longitude_error_deviation,
            &frame->altitude_error_deviation))
        return false;
    if (strcmp(type+2, "GST"))
        return false;

    return true;
}

bool nmea_parse_gsv(struct nmea_sentence_gsv *frame, const char *sentence)
{
    // $GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74
    // $GPGSV,3,3,11,22,42,067,42,24,14,311,43,27,05,244,00,,,,*4D
    // $GPGSV,4,2,11,08,51,203,30,09,45,215,28*75
    // $GPGSV,4,4,13,39,31,170,27*40
    // $GPGSV,4,4,13*7B
    char type[6];

    if (!nmea_scan(sentence, "tiii;iiiiiiiiiiiiiiii",
            type,
            &frame->total_msgs,
            &frame->msg_nr,
            &frame->total_sats,
            &frame->sats[0].nr,
            &frame->sats[0].elevation,
            &frame->sats[0].azimuth,
            &frame->sats[0].snr,
            &frame->sats[1].nr,
            &frame->sats[1].elevation,
            &frame->sats[1].azimuth,
            &frame->sats[1].snr,
            &frame->sats[2].nr,
            &frame->sats[2].elevation,
            &frame->sats[2].azimuth,
            &frame->sats[2].snr,
            &frame->sats[3].nr,
            &frame->sats[3].elevation,
            &frame->sats[3].azimuth,
            &frame->sats[3].snr
            )) {
        return false;
    }
    if (strcmp(type+2, "GSV"))
        return false;

    return true;
}

bool nmea_parse_vtg(struct nmea_sentence_vtg *frame, const char *sentence)
{
    // $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
    // $GPVTG,156.1,T,140.9,M,0.0,N,0.0,K*41
    // $GPVTG,096.5,T,083.5,M,0.0,N,0.0,K,D*22
    // $GPVTG,188.36,T,,M,0.820,N,1.519,K,A*3F
    char type[6];
    char c_true, c_magnetic, c_knots, c_kph, c_faa_mode;

    if (!nmea_scan(sentence, "tfcfcfcfc;c",
            type,
            &frame->true_track_degrees,
            &c_true,
            &frame->magnetic_track_degrees,
            &c_magnetic,
            &frame->speed_knots,
            &c_knots,
            &frame->speed_kph,
            &c_kph,
            &c_faa_mode))
        return false;
    if (strcmp(type+2, "VTG"))
        return false;
    // �������� ��������
    if (c_true != 'T' ||
        c_magnetic != 'M' ||
        c_knots != 'N' ||
        c_kph != 'K')
        return false;
    frame->faa_mode = (enum nmea_faa_mode)c_faa_mode;

    return true;
}

bool nmea_parse_zda(struct nmea_sentence_zda *frame, const char *sentence)
{
  // $GPZDA,201530.00,04,07,2002,00,00*60
  char type[6];

  if(!nmea_scan(sentence, "tTiiiii",
          type,
          &frame->time,
          &frame->date.day,
          &frame->date.month,
          &frame->date.year,
          &frame->hour_offset,
          &frame->minute_offset))
      return false;
  if (strcmp(type+2, "ZDA"))
      return false;

  // �������� ��������
  if (abs(frame->hour_offset) > 13 ||
      frame->minute_offset > 59 ||
      frame->minute_offset < 0)
      return false;

  return true;
}

int nmea_gettime(struct timespec *ts, const struct nmea_date *date, const struct nmea_time *time_)
{
    if (date->year == -1 || time_->hours == -1)
        return -1;

    struct tm tm;
    memset(&tm, 0, sizeof(tm));
    if (date->year < 80) {
        tm.tm_year = 2000 + date->year - 1900;
    } else if (date->year >= 1900) {
        tm.tm_year = date->year - 1900;
    } else {
        tm.tm_year = date->year;
    }
    tm.tm_mon = date->month - 1;
    tm.tm_mday = date->day;
    tm.tm_hour = time_->hours;
    tm.tm_min = time_->minutes;
    tm.tm_sec = time_->seconds;

    time_t timestamp = timegm(&tm);
    if (timestamp != (time_t)-1) {
        ts->tv_sec = timestamp;
        ts->tv_nsec = time_->microseconds * 1000;
        return 0;
    } else {
        return -1;
    }
}


