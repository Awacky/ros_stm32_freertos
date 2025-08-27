#ifdef __cplusplus
extern "C" {
#endif

#ifndef __ROS_BSP_H_
#define __ROS_BSP_H_	

	#include "stdint.h"
	#include "datatypes.h"
	
	void ros_serial_init(uint32_t baudRate);
	void rosSetuart1Taskfunc(void (* const send)(void));
	void rosSetDmaRead_func(void(*dmaRead_func_t)(stcATBuff *getStcABuff));
	void ros_serial_write_length(uint8_t *data, uint16_t length);
	int  ros_serial_read_char(void);
	int  ros2_serial_read_buff(uint8_t *data,uint32_t getLength);
	uint32_t ros_get_system_time(void);
	void ros_dealy_ms(uint16_t ms);	
	void ros_communication_bsp_init(void);

#endif /* __ROS_BSP_H_ */	
	
	
#ifdef __cplusplus
}
#endif

