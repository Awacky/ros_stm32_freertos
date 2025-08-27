
#define LOG_TAG      "elog"

#include <log_handle.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#ifdef ELOG_COLOR_ENABLE
/**
 * CSI(Control Sequence Introducer/Initiator) sign
 * more information on https://en.wikipedia.org/wiki/ANSI_escape_code
 */
#define CSI_START                      "\033["
#define CSI_END                        "\033[0m"
/* output log front color */
#define F_BLACK                        "30;"
#define F_RED                          "31;"
#define F_GREEN                        "32;"
#define F_YELLOW                       "33;"
#define F_BLUE                         "34;"
#define F_MAGENTA                      "35;"
#define F_CYAN                         "36;"
#define F_WHITE                        "37;"
/* output log background color */
#define B_NULL
#define B_BLACK                        "40;"
#define B_RED                          "41;"
#define B_GREEN                        "42;"
#define B_YELLOW                       "43;"
#define B_BLUE                         "44;"
#define B_MAGENTA                      "45;"
#define B_CYAN                         "46;"
#define B_WHITE                        "47;"
/* output log fonts style */
#define S_BOLD                         "1m"
#define S_UNDERLINE                    "4m"
#define S_BLINK                        "5m"
#define S_NORMAL                       "22m"
/* output log default color definition: [front color] + [background color] + [show style] */
#ifndef ELOG_COLOR_ASSERT
#define ELOG_COLOR_ASSERT              (F_MAGENTA B_NULL S_NORMAL)
#endif
#ifndef ELOG_COLOR_ERROR
#define ELOG_COLOR_ERROR               (F_RED B_NULL S_NORMAL)
#endif
#ifndef ELOG_COLOR_WARN
#define ELOG_COLOR_WARN                (F_YELLOW B_NULL S_NORMAL)
#endif
#ifndef ELOG_COLOR_INFO
#define ELOG_COLOR_INFO                (F_CYAN B_NULL S_NORMAL)
#endif
#ifndef ELOG_COLOR_DEBUG
#define ELOG_COLOR_DEBUG               (F_GREEN B_NULL S_NORMAL)
#endif
#ifndef ELOG_COLOR_VERBOSE
#define ELOG_COLOR_VERBOSE             (F_BLUE B_NULL S_NORMAL)
#endif
#endif /* ELOG_COLOR_ENABLE */

static EasyLogger elog;
uint16_t buf_write_size = 0;
uint16_t currLength = 0;
char log_buf[ELOG_LINE_BUF_SIZE] = { 0 };
/* level output info */
static const char *level_output_info[] = {
        [ELOG_LVL_ASSERT]  = "A/",
        [ELOG_LVL_ERROR]   = "E/",
        [ELOG_LVL_WARN]    = "W/",
        [ELOG_LVL_INFO]    = "I/",
        [ELOG_LVL_DEBUG]   = "D/",
        [ELOG_LVL_VERBOSE] = "V/",
};

#ifdef ELOG_COLOR_ENABLE
/* color output info */
static const char *color_output_info[] = {
        [ELOG_LVL_ASSERT]  = ELOG_COLOR_ASSERT,
        [ELOG_LVL_ERROR]   = ELOG_COLOR_ERROR,
        [ELOG_LVL_WARN]    = ELOG_COLOR_WARN,
        [ELOG_LVL_INFO]    = ELOG_COLOR_INFO,
        [ELOG_LVL_DEBUG]   = ELOG_COLOR_DEBUG,
        [ELOG_LVL_VERBOSE] = ELOG_COLOR_VERBOSE,
};
#endif /* ELOG_COLOR_ENABLE */

static bool get_fmt_enabled(uint8_t level, uint16_t set);

static void elog_buf_output(const uint8_t *log, uint16_t len_t);
/* EasyLogger assert hook */
void (*elog_assert_p)(const char* expr, const char* func, uint16_t line) = NULL;
void (*elog_output_p)(uint8_t* buf_t, uint16_t length_t) = NULL;
void (*elog_lock_p)(void) = NULL;
void (*elog_unlock_p)(void) = NULL;
/**
 * Set a hook function to EasyLogger assert. It will run when the expression is false.
 *
 * @param hook the hook function
 */
void elog_assert_register(void (*fun_t)(const char* expr, const char* func, uint16_t line)) {
    elog_assert_p = fun_t;
}

void elog_output_register(void (*fun_t)(uint8_t* buf_t, uint16_t length_t)) {
    elog_output_p = fun_t;
}

void elog_lock_register(void (*fun_t)(void)) {
    elog_lock_p = fun_t;
}

void elog_unlock_register(void (*fun_t)(void)) {
    elog_unlock_p = fun_t;
}

void elog_assert_hook(const char* expr, const char* func, uint16_t line){
	if(elog_assert_p == NULL){
		elog_assert("elog", "(%s) has assert failed at %s:%ld.", expr, __FUNCTION__, __LINE__);
	}else{
		elog_assert_p(expr, __FUNCTION__, __LINE__);
	}
}
/**
 * another copy string function
 *
 * @param cur_len current copied log length, max size is ELOG_LINE_BUF_SIZE
 * @param dst destination
 * @param src source
 *
 * @return copied length
 */
uint16_t elog_strcpy(uint16_t cur_len, char *dst, const char *src) {
    const char *src_old = src;
    assert(dst);
    assert(src);
    while (*src != 0) {
        /* make sure destination has enough space */
        if (cur_len++ < ELOG_LINE_BUF_SIZE) {
            *dst++ = *src++;
        } else {
            break;
        }
    }
    return src - src_old;
}

/**
 * output buffered logs when buffer is full
 *
 * @param log will be buffered line's log
 * @param size log size
 */
static void elog_buf_output(const uint8_t *log, uint16_t len_t) {
    uint16_t write_size = 0, write_index = 0;
	if (buf_write_size + len_t > ELOG_LINE_BUF_SIZE) {
		currLength = buf_write_size;
		if(NULL!=elog_output_p){
			elog_output_p((uint8_t*)log_buf,currLength);
		}
		buf_write_size = 0;
		memcpy(log_buf, log, len_t);
		buf_write_size += len_t;
	} else {
		memcpy(log_buf + buf_write_size, log, len_t);
		buf_write_size += len_t;
	}
}

/**
 * EasyLogger start after initialize.
 */
void elog_start(void) {
    if (!elog.init_ok) {
        return ;
    }
	#ifdef ELOG_COLOR_ENABLE
    /* enable text color by default */
    elog_set_text_color_enabled(false);
	#endif
    /* enable output */
    elog_set_output_enabled(true);
    /* show version */
    elog_info(LOG_TAG,"EasyLogger V%s is initialize success.", ELOG_SW_VERSION);
}

/**
 * EasyLogger stop after initialize.
 */
void elog_stop(void) {
    if (!elog.init_ok) {
        return ;
    }
    /* disable output */
    elog_set_output_enabled(false);
    /* show version */
    elog_info(LOG_TAG,"EasyLogger V%s is deinitialize success.", ELOG_SW_VERSION);
}


/**
 * set output enable or disable
 *
 * @param enabled TRUE: enable FALSE: disable
 */
void elog_set_output_enabled(bool enabled) {
    ELOG_ASSERT((enabled == false) || (enabled == true));
    elog.output_enabled = enabled;
}

#ifdef ELOG_COLOR_ENABLE
/**
 * set log text color enable or disable
 * 
 * @param enabled TRUE: enable FALSE:disable
 */
void elog_set_text_color_enabled(bool enabled) {
    ELOG_ASSERT((enabled == false) || (enabled == true));
    elog.text_color_enabled = enabled;
}

/**
 * get log text color enable status
 *
 * @return enable or disable
 */
bool elog_get_text_color_enabled(void) {
    return elog.text_color_enabled;
}
#endif /* ELOG_COLOR_ENABLE */

/**
 * get output is enable or disable
 *
 * @return enable or disable
 */
bool elog_get_output_enabled(void) {
    return elog.output_enabled;
}

/**
 * set log output format. only enable or disable
 *
 * @param level level
 * @param set format set
 */
void elog_set_fmt(uint8_t level, uint16_t set) {
    ELOG_ASSERT(level <= ELOG_LVL_VERBOSE);
    elog.enabled_fmt_set[level] = set;
}

/**
 * output the log
 *
 * @param level level
 * @param tag tag
 * @param file file name
 * @param func function name
 * @param line line number
 * @param format output format
 * @param ... args
 *
 */
void elog_output(uint8_t level, const char *tag, const char *file, const char *func,
        const long line, const char *format, ...) {

    uint16_t tag_len = strlen(tag), log_len = 0, newline_len = strlen(ELOG_NEWLINE_SIGN);
    char line_num[ELOG_LINE_NUM_MAX_LEN + 1] = { 0 };
    char clog_buf[ELOG_LINE_BUF_SIZE] = { 0 };
    va_list args;
    int fmt_result;

    ELOG_ASSERT(level <= ELOG_LVL_VERBOSE);

    /* check output enabled */
    if (!elog.output_enabled) {
        return;
    }
	if(NULL != elog_lock_p){
		elog_lock_p();
	}
    /* args point to the first variable parameter */
    va_start(args, format);

#ifdef ELOG_COLOR_ENABLE
    /* add CSI start sign and color info */
    if (elog.text_color_enabled) {
        log_len += elog_strcpy(log_len, clog_buf + log_len, CSI_START);
        log_len += elog_strcpy(log_len, clog_buf + log_len, color_output_info[level]);
    }
#endif

    /* package level info */
    if (get_fmt_enabled(level, ELOG_FMT_LVL)) {
        log_len += elog_strcpy(log_len, clog_buf + log_len, level_output_info[level]);
    }
    /* package tag info */
    if (get_fmt_enabled(level, ELOG_FMT_TAG)) {
        log_len += elog_strcpy(log_len, clog_buf + log_len, tag);
        log_len += elog_strcpy(log_len, clog_buf + log_len, " ");
    }
//    /* package time, process and thread info */
//    if (get_fmt_enabled(level, ELOG_FMT_TIME | ELOG_FMT_P_INFO | ELOG_FMT_T_INFO)) {
//        log_len += elog_strcpy(log_len, clog_buf + log_len, "[");
//        /* package time info */
//        if (get_fmt_enabled(level, ELOG_FMT_TIME)) {
////            log_len += elog_strcpy(log_len, log_buf + log_len, elog_port_get_time());
//            if (get_fmt_enabled(level, ELOG_FMT_P_INFO | ELOG_FMT_T_INFO)) {
//                log_len += elog_strcpy(log_len, clog_buf + log_len, " ");
//            }
//        }
//        /* package process info */
//        if (get_fmt_enabled(level, ELOG_FMT_P_INFO)) {
////            log_len += elog_strcpy(log_len, log_buf + log_len, elog_port_get_p_info());
//            if (get_fmt_enabled(level, ELOG_FMT_T_INFO)) {
//                log_len += elog_strcpy(log_len, clog_buf + log_len, " ");
//            }
//        }
//        /* package thread info */
//        if (get_fmt_enabled(level, ELOG_FMT_T_INFO)) {
////            log_len += elog_strcpy(log_len, log_buf + log_len, elog_port_get_t_info());
//        }
//        log_len += elog_strcpy(log_len, clog_buf + log_len, "] ");
//    }
    /* package file directory and name, function name and line number info */
    if (get_fmt_enabled(level, ELOG_FMT_DIR | ELOG_FMT_FUNC | ELOG_FMT_LINE)) {
        log_len += elog_strcpy(log_len, clog_buf + log_len, "(");
        /* package file info */
        if (get_fmt_enabled(level, ELOG_FMT_DIR)) {
            log_len += elog_strcpy(log_len, clog_buf + log_len, file);
            if (get_fmt_enabled(level, ELOG_FMT_FUNC)) {
                log_len += elog_strcpy(log_len, clog_buf + log_len, ":");
            } else if (get_fmt_enabled(level, ELOG_FMT_LINE)) {
                log_len += elog_strcpy(log_len, clog_buf + log_len, " ");
            }
        }
        /* package line info */
        if (get_fmt_enabled(level, ELOG_FMT_LINE)) {
            snprintf(line_num, ELOG_LINE_NUM_MAX_LEN, "%ld", line);
            log_len += elog_strcpy(log_len, clog_buf + log_len, line_num);
            if (get_fmt_enabled(level, ELOG_FMT_FUNC)) {
                log_len += elog_strcpy(log_len, log_buf + log_len, " ");
            }
        }
        /* package func info */
        if (get_fmt_enabled(level, ELOG_FMT_FUNC)) {
            log_len += elog_strcpy(log_len, clog_buf + log_len, func);
            
        }
        log_len += elog_strcpy(log_len, clog_buf + log_len, ")");
    }
    /* package other log data to buffer. '\0' must be added in the end by vsnprintf. */
    fmt_result = vsnprintf(clog_buf + log_len, ELOG_LINE_BUF_SIZE - log_len, format, args);

    va_end(args);
    /* calculate log length */
    if ((log_len + fmt_result <= ELOG_LINE_BUF_SIZE) && (fmt_result > -1)) {
        log_len += fmt_result;
    } else {
        /* using max length */
        log_len = ELOG_LINE_BUF_SIZE;
    }
    /* overflow check and reserve some space for CSI end sign and newline sign */
#ifdef ELOG_COLOR_ENABLE
    if (log_len + (sizeof(CSI_END) - 1) + newline_len > ELOG_LINE_BUF_SIZE) {
        /* using max length */
        log_len = ELOG_LINE_BUF_SIZE;
        /* reserve some space for CSI end sign */
        log_len -= (sizeof(CSI_END) - 1);
#else
    if (log_len + newline_len > ELOG_LINE_BUF_SIZE) {
        /* using max length */
        log_len = ELOG_LINE_BUF_SIZE;
#endif /* ELOG_COLOR_ENABLE */
        /* reserve some space for newline sign */
        log_len -= newline_len;
    }

#ifdef ELOG_COLOR_ENABLE
    /* add CSI end sign */
    if (elog.text_color_enabled) {
        log_len += elog_strcpy(log_len, clog_buf + log_len, CSI_END);
    }
#endif
	
    log_len += elog_strcpy(log_len, clog_buf + log_len, ELOG_NEWLINE_SIGN);
	
	elog_buf_output((uint8_t *)clog_buf, log_len);
    if(NULL != elog_unlock_p){
		elog_unlock_p();
	}
}

/**
 * get format enabled
 *
 * @param level level
 * @param set format set
 *
 * @return enable or disable
 */
static bool get_fmt_enabled(uint8_t level, uint16_t set) {
    ELOG_ASSERT(level <= ELOG_LVL_VERBOSE);
    if (elog.enabled_fmt_set[level] & set) {
        return true;
    } else {
        return false;
    }
}
