#ifdef __cplusplus
extern "C" {
#endif

	#ifndef _LOG_HANDLE_H
	#define _LOG_HANDLE_H

	#include "hw_config.h"
		/*---------------------------------------------------------------------------*/
		//使能输出
		#define ELOG_OUTPUT_ENABLE
		//设置静态输出日志级别。范围:从ELOG_LVL_ASSERT到ELOG_LVL_VERBOSE
		#define ELOG_OUTPUT_LVL                          ELOG_LVL_VERBOSE
		//启用断言检查
		#define ELOG_ASSERT_ENABLE
		//每行日志的缓冲区大小
		#define ELOG_LINE_BUF_SIZE                       128
		//输出行号最大长度
		#define ELOG_LINE_NUM_MAX_LEN                    5
		//输出换行符
		#define ELOG_NEWLINE_SIGN                        "\n"
		/*---------------------------------------------------------------------------*/
		//启用日志颜色
		#define ELOG_COLOR_ENABLE
		//如果需要，可以将某些级别的日志更改为不默认的颜色
		#define ELOG_COLOR_ASSERT                        (F_MAGENTA B_NULL S_NORMAL)
		#define ELOG_COLOR_ERROR                         (F_RED B_NULL S_NORMAL)
		#define ELOG_COLOR_WARN                          (F_YELLOW B_NULL S_NORMAL)
		#define ELOG_COLOR_INFO                          (F_CYAN B_NULL S_NORMAL)
		#define ELOG_COLOR_DEBUG                         (F_GREEN B_NULL S_NORMAL)
		#define ELOG_COLOR_VERBOSE                       (F_BLUE B_NULL S_NORMAL)
		/*---------------------------------------------------------------------------*/
		//启用缓冲输出模式
		#define ELOG_BUF_OUTPUT_ENABLE
		//缓冲输出模式的缓冲区大小
		#define ELOG_BUF_OUTPUT_BUF_SIZE                 (ELOG_LINE_BUF_SIZE * 10)

		//输出日志的级别总数
		#define ELOG_LVL_TOTAL_NUM                   6

		//EasyLogger软件版本号
		#define ELOG_SW_VERSION                      "2.2.99"
		
		//所有格式的宏定义
		#define ELOG_FMT_ALL    (ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME|ELOG_FMT_P_INFO|ELOG_FMT_T_INFO| \
			ELOG_FMT_DIR|ELOG_FMT_FUNC|ELOG_FMT_LINE)
			/* all formats index */
		typedef enum {
			ELOG_FMT_LVL    = 1 << 0, /**< level 					（级别）*/
			ELOG_FMT_TAG    = 1 << 1, /**< tag 						（标签）*/
			ELOG_FMT_TIME   = 1 << 2, /**< current time 			（当前时间）*/
			ELOG_FMT_P_INFO = 1 << 3, /**< process info 			（进行信息）*/
			ELOG_FMT_T_INFO = 1 << 4, /**< thread info 				（线程信息）*/
			ELOG_FMT_DIR    = 1 << 5, /**< file directory and name 	（文件目录及名称）*/
			ELOG_FMT_FUNC   = 1 << 6, /**< function name 			（函数名称）*/
			ELOG_FMT_LINE   = 1 << 7, /**< line number 				（代码行号）*/
		} ElogFmtIndex;
		/* easy logger */
		typedef struct {
			uint16_t enabled_fmt_set[ELOG_LVL_TOTAL_NUM];
			bool init_ok;
			bool output_enabled;
		#ifdef ELOG_COLOR_ENABLE
			bool text_color_enabled;
		#endif
		}EasyLogger, *EasyLogger_t;
		
		//EasyLogger断言为开发人员
		#ifdef ELOG_ASSERT_ENABLE
			#define ELOG_ASSERT(EXPR)                                                 \
			if (!(EXPR))                                                              \
			{                                                                         \
					elog_assert_hook(#EXPR, __FUNCTION__, __LINE__);                  \
			}
		#else
			#define ELOG_ASSERT(EXPR)                    ((void)0);
		#endif
				
		void elog_start(void);
		void elog_stop(void);
		
		void elog_set_fmt(uint8_t level, uint16_t set);
		void elog_set_text_color_enabled(bool enabled); 
		bool elog_get_text_color_enabled(void);
		void elog_set_output_enabled(bool enabled);
		bool elog_get_output_enabled(void);
		
		void elog_assert_register(void (*fun_t)(const char* expr, const char* func, uint16_t line));
		void elog_output_register(void (*fun_t)(uint8_t* buf_t, uint16_t length_t));
		void elog_lock_register(void (*fun_t)(void));
		void elog_unlock_register(void (*fun_t)(void));
		
	#endif
	
#ifdef __cplusplus
}
#endif