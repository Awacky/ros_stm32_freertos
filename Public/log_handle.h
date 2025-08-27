#ifdef __cplusplus
extern "C" {
#endif

	#ifndef _LOG_HANDLE_H
	#define _LOG_HANDLE_H

	#include "hw_config.h"
		/*---------------------------------------------------------------------------*/
		//ʹ�����
		#define ELOG_OUTPUT_ENABLE
		//���þ�̬�����־���𡣷�Χ:��ELOG_LVL_ASSERT��ELOG_LVL_VERBOSE
		#define ELOG_OUTPUT_LVL                          ELOG_LVL_VERBOSE
		//���ö��Լ��
		#define ELOG_ASSERT_ENABLE
		//ÿ����־�Ļ�������С
		#define ELOG_LINE_BUF_SIZE                       128
		//����к���󳤶�
		#define ELOG_LINE_NUM_MAX_LEN                    5
		//������з�
		#define ELOG_NEWLINE_SIGN                        "\n"
		/*---------------------------------------------------------------------------*/
		//������־��ɫ
		#define ELOG_COLOR_ENABLE
		//�����Ҫ�����Խ�ĳЩ�������־����Ϊ��Ĭ�ϵ���ɫ
		#define ELOG_COLOR_ASSERT                        (F_MAGENTA B_NULL S_NORMAL)
		#define ELOG_COLOR_ERROR                         (F_RED B_NULL S_NORMAL)
		#define ELOG_COLOR_WARN                          (F_YELLOW B_NULL S_NORMAL)
		#define ELOG_COLOR_INFO                          (F_CYAN B_NULL S_NORMAL)
		#define ELOG_COLOR_DEBUG                         (F_GREEN B_NULL S_NORMAL)
		#define ELOG_COLOR_VERBOSE                       (F_BLUE B_NULL S_NORMAL)
		/*---------------------------------------------------------------------------*/
		//���û������ģʽ
		#define ELOG_BUF_OUTPUT_ENABLE
		//�������ģʽ�Ļ�������С
		#define ELOG_BUF_OUTPUT_BUF_SIZE                 (ELOG_LINE_BUF_SIZE * 10)

		//�����־�ļ�������
		#define ELOG_LVL_TOTAL_NUM                   6

		//EasyLogger����汾��
		#define ELOG_SW_VERSION                      "2.2.99"
		
		//���и�ʽ�ĺ궨��
		#define ELOG_FMT_ALL    (ELOG_FMT_LVL|ELOG_FMT_TAG|ELOG_FMT_TIME|ELOG_FMT_P_INFO|ELOG_FMT_T_INFO| \
			ELOG_FMT_DIR|ELOG_FMT_FUNC|ELOG_FMT_LINE)
			/* all formats index */
		typedef enum {
			ELOG_FMT_LVL    = 1 << 0, /**< level 					������*/
			ELOG_FMT_TAG    = 1 << 1, /**< tag 						����ǩ��*/
			ELOG_FMT_TIME   = 1 << 2, /**< current time 			����ǰʱ�䣩*/
			ELOG_FMT_P_INFO = 1 << 3, /**< process info 			��������Ϣ��*/
			ELOG_FMT_T_INFO = 1 << 4, /**< thread info 				���߳���Ϣ��*/
			ELOG_FMT_DIR    = 1 << 5, /**< file directory and name 	���ļ�Ŀ¼�����ƣ�*/
			ELOG_FMT_FUNC   = 1 << 6, /**< function name 			���������ƣ�*/
			ELOG_FMT_LINE   = 1 << 7, /**< line number 				�������кţ�*/
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
		
		//EasyLogger����Ϊ������Ա
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