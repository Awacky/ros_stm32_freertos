#ifndef __STM_CUSTOM_TRANSPORT_H__
#define __STM_CUSTOM_TRANSPORT_H__

#include <uxr/client/transport.h>

bool dev_transport_open(struct uxrCustomTransport * transport);
bool dev_transport_close(struct uxrCustomTransport * transport);

size_t dev_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err);
size_t dev_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

#endif /* INC_IMU_H_ */


