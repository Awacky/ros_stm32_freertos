#include "STM_custom_transport.h"
#include <rmw_microxrcedds_c/config.h>

#include "FreeRTOS.h"
#include "task.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "ros_bsp.h"

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

// --- micro-ROS Transports ---

bool dev_transport_open(struct uxrCustomTransport * transport){
	(void)transport;
    return true;
}

bool dev_transport_close(struct uxrCustomTransport * transport){
	(void)transport;
    return true;
}

size_t dev_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err){
	(void)transport;
	(void)err;
	ros_serial_write_length(buf,len);
    return len;
}

size_t dev_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
	size_t wrote;
    (void)transport;
	(void)timeout;
	(void)err;
	wrote = ros2_serial_read_buff(buf,len);
    return wrote;
}

#endif //RMW_UXRCE_TRANSPORT_CUSTOM


