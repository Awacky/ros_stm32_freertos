/******************************************************************************
 * Copyright 2020 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <string.h>
#include "ublox.h"

#define FNV1_32_INIT  ((uint32_t)0x811c9dc5) // init value for FNV1 hash algorithm
#define FNV1_32_PRIME ((uint32_t)0x01000193) // magic prime for FNV1 hash algorithm
#define MIN(x, y)     (x < y ? x : y)

static void (*ubxDevSend)(uint8_t *src_t,uint16_t srcLen)=NULL;
static uint8_t ubxBuff_t[128];
static uint32_t _fnv1_32_str(uint8_t* str, uint32_t hval)
{
    uint8_t* s = str;

    /*
	 * FNV-1 hash each octet in the buffer
	 */
    while (*s) {

        /* multiply by the 32 bit FNV magic prime mod 2^32 */
#if defined(NO_FNV_GCC_OPTIMIZATION)
        hval *= FNV1_32_PRIME;
#else
        hval += (hval << 1) + (hval << 4) + (hval << 7) + (hval << 8) + (hval << 24);
#endif

        /* xor the bottom with the current octet */
        hval ^= (uint32_t)*s++;
    }

    /* return our new hash value */
    return hval;
}

static void _calc_ubx_checksum(const uint8_t* buffer, const uint16_t length, ubx_checksum_t* checksum)
{
    for (uint16_t i = 0; i < length; i++) {
        checksum->ck_a = checksum->ck_a + buffer[i];
        checksum->ck_b = checksum->ck_b + checksum->ck_a;
    }
}

static void _add_byte_to_checksum(ubx_decoder_t* ubx_decoder, const uint8_t b)
{
    ubx_decoder->rx_ck_a += b;
    ubx_decoder->rx_ck_b += ubx_decoder->rx_ck_a;
}

// -1 = error, 0 = ok, 1 = payload completed
//把接收的数据添加到buf中，并进行处理
static int _payload_rx_add_nav_svinfo(ubx_decoder_t* ubx_decoder, const uint8_t b)
{
    int ret = 0;
    if (ubx_decoder->rx_payload_index < sizeof(ubx_payload_rx_nav_svinfo_part1_t)) {
        // Fill Part 1 buffer
        ubx_decoder->buf.raw[ubx_decoder->rx_payload_index] = b;
    } else {
        if (ubx_decoder->rx_payload_index == sizeof(ubx_payload_rx_nav_svinfo_part1_t)) {
            // Part 1 complete: decode Part 1 buffer
            ubx_decoder->satellite_info.count = MIN(ubx_decoder->buf.payload_rx_nav_svinfo_part1.numCh, SAT_INFO_MAX_SATELLITES);
        }

        if (ubx_decoder->rx_payload_index < 
			sizeof(ubx_payload_rx_nav_svinfo_part1_t) + 
			ubx_decoder->satellite_info.count * sizeof(ubx_payload_rx_nav_svinfo_part2_t)) {//8+n*12
            // Still room in _satellite_info: fill Part 2 buffer
            unsigned buf_index = (ubx_decoder->rx_payload_index - sizeof(ubx_payload_rx_nav_svinfo_part1_t)) % sizeof(ubx_payload_rx_nav_svinfo_part2_t);
            ubx_decoder->buf.raw[buf_index] = b;

            if (buf_index == sizeof(ubx_payload_rx_nav_svinfo_part2_t) - 1) {//12-1
                // Part 2 complete: decode Part 2 buffer
                unsigned sat_index = (ubx_decoder->rx_payload_index - sizeof(ubx_payload_rx_nav_svinfo_part1_t)) / sizeof(ubx_payload_rx_nav_svinfo_part2_t);
                ubx_decoder->satellite_info.used[sat_index] = (uint8_t)(ubx_decoder->buf.payload_rx_nav_svinfo_part2.flags & 0x01);
                ubx_decoder->satellite_info.snr[sat_index] = (uint8_t)(ubx_decoder->buf.payload_rx_nav_svinfo_part2.cno);
                ubx_decoder->satellite_info.elevation[sat_index] = (uint8_t)(ubx_decoder->buf.payload_rx_nav_svinfo_part2.elev);
                ubx_decoder->satellite_info.azimuth[sat_index] = (uint8_t)((float)ubx_decoder->buf.payload_rx_nav_svinfo_part2.azim * 255.0f / 360.0f);
                ubx_decoder->satellite_info.svid[sat_index] = (uint8_t)(ubx_decoder->buf.payload_rx_nav_svinfo_part2.svid);
            }
        }
    }
    if (++ubx_decoder->rx_payload_index >= ubx_decoder->rx_payload_length) {
        ret = 1; // payload received completely
    }
    return ret;
}

// -1 = error, 0 = ok, 1 = payload completed
//把接收的数据添加到版本信息buf中，并进行处理
static int _payload_rx_add_mon_ver(ubx_decoder_t* ubx_decoder, const uint8_t b)
{
    int ret = 0;
    if (ubx_decoder->rx_payload_index < sizeof(ubx_payload_rx_mon_ver_part1_t)) {
        // Fill Part 1 buffer
        ubx_decoder->buf.raw[ubx_decoder->rx_payload_index] = b;
    } else {
        if (ubx_decoder->rx_payload_index == sizeof(ubx_payload_rx_mon_ver_part1_t)) {
            // Part 1 complete: decode Part 1 buffer and calculate hash for SW&HW version strings
            ubx_decoder->ubx_version = _fnv1_32_str(ubx_decoder->buf.payload_rx_mon_ver_part1.swVersion, FNV1_32_INIT);
            ubx_decoder->ubx_version = _fnv1_32_str(ubx_decoder->buf.payload_rx_mon_ver_part1.hwVersion, ubx_decoder->ubx_version);
        }
        // fill Part 2 buffer
        unsigned buf_index = (ubx_decoder->rx_payload_index - sizeof(ubx_payload_rx_mon_ver_part1_t)) % sizeof(ubx_payload_rx_mon_ver_part2_t);
        ubx_decoder->buf.raw[buf_index] = b;
        if (buf_index == sizeof(ubx_payload_rx_mon_ver_part2_t) - 1) {
            // Part 2 complete: decode Part 2 buffer
            //console_printf("VER ext \" %30s\"\r\n", ubx_decoder->buf.payload_rx_mon_ver_part2.extension);
        }
    }
    if (++ubx_decoder->rx_payload_index >= ubx_decoder->rx_payload_length) {
        ret = 1; // payload received completely
    }
    return ret;
}

// -1 = abort, 0 = continue
//接收数据初始化
static int _payload_rx_init(ubx_decoder_t* ubx_decoder)
{
    int ret = 0;
    ubx_decoder->rx_state = UBX_RXMSG_HANDLE; // handle by default
    switch (ubx_decoder->rx_msg) {
		case UBX_MSG_NAV_PVT:{
			if ((ubx_decoder->rx_payload_length != UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX7)       /* u-blox 7 msg format */
				&& (ubx_decoder->rx_payload_length != UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX8)) { /* u-blox 8+ msg format */
				ubx_decoder->rx_state = UBX_RXMSG_ERROR_LENGTH;
			} else if (!ubx_decoder->configured) {
				ubx_decoder->rx_state = UBX_RXMSG_IGNORE; // ignore if not ubx_decoder->configured
			} else if (!ubx_decoder->use_nav_pvt) {
				ubx_decoder->rx_state = UBX_RXMSG_DISABLE; // disable if not using NAV-PVT
			}
		}break;
		case UBX_MSG_NAV_POSLLH:{
			if (ubx_decoder->rx_payload_length != sizeof(ubx_payload_rx_nav_posllh_t)) {
				ubx_decoder->rx_state = UBX_RXMSG_ERROR_LENGTH;
			} else if (!ubx_decoder->configured) {
				ubx_decoder->rx_state = UBX_RXMSG_IGNORE; // ignore if not ubx_decoder->configured
			} else if (ubx_decoder->use_nav_pvt) {
				ubx_decoder->rx_state = UBX_RXMSG_DISABLE; // disable if using NAV-PVT instead
			}
		}break;
		case UBX_MSG_NAV_SOL:{
			if (ubx_decoder->rx_payload_length != sizeof(ubx_payload_rx_nav_sol_t)) {
				ubx_decoder->rx_state = UBX_RXMSG_ERROR_LENGTH;
			} else if (!ubx_decoder->configured) {
				ubx_decoder->rx_state = UBX_RXMSG_IGNORE; // ignore if not ubx_decoder->configured
			} else if (ubx_decoder->use_nav_pvt) {
				ubx_decoder->rx_state = UBX_RXMSG_DISABLE; // disable if using NAV-PVT instead
			}
		}break;

		case UBX_MSG_NAV_DOP:{
			if (ubx_decoder->rx_payload_length != sizeof(ubx_payload_rx_nav_dop_t)) {
				ubx_decoder->rx_state = UBX_RXMSG_ERROR_LENGTH;
			} else if (!ubx_decoder->configured) {
				ubx_decoder->rx_state = UBX_RXMSG_IGNORE; // ignore if not ubx_decoder->configured
			}
		}break;
		case UBX_MSG_NAV_TIMEUTC:{
			if (ubx_decoder->rx_payload_length != sizeof(ubx_payload_rx_nav_timeutc_t)) {
				ubx_decoder->rx_state = UBX_RXMSG_ERROR_LENGTH;
			} else if (!ubx_decoder->configured) {
				ubx_decoder->rx_state = UBX_RXMSG_IGNORE; // ignore if not ubx_decoder->configured
			} else if (ubx_decoder->use_nav_pvt) {
				ubx_decoder->rx_state = UBX_RXMSG_DISABLE; // disable if using NAV-PVT instead
			}
		}break;
		case UBX_MSG_NAV_SVINFO:{
			if (!ubx_decoder->configured) {
				ubx_decoder->rx_state = UBX_RXMSG_IGNORE; // ignore if not ubx_decoder->configured
			} else {
				memset(&ubx_decoder->satellite_info, 0, sizeof(ubx_decoder->satellite_info)); // initialize sat info
			}
		}break;

		case UBX_MSG_NAV_VELNED:{
			if (ubx_decoder->rx_payload_length != sizeof(ubx_payload_rx_nav_velned_t)) {
				ubx_decoder->rx_state = UBX_RXMSG_ERROR_LENGTH;
			} else if (!ubx_decoder->configured) {
				ubx_decoder->rx_state = UBX_RXMSG_IGNORE; // ignore if not ubx_decoder->configured
			} else if (ubx_decoder->use_nav_pvt) {
				ubx_decoder->rx_state = UBX_RXMSG_DISABLE; // disable if using NAV-PVT instead
			}
		}break;
		case UBX_MSG_MON_VER:{
		}break; // unconditionally handle this message
		case UBX_MSG_MON_HW:{
			if ((ubx_decoder->rx_payload_length != sizeof(ubx_payload_rx_mon_hw_ubx6_t))       /* u-blox 6 msg format */
				&& (ubx_decoder->rx_payload_length != sizeof(ubx_payload_rx_mon_hw_ubx7_t))) { /* u-blox 7+ msg format */
				ubx_decoder->rx_state = UBX_RXMSG_ERROR_LENGTH;
			} else if (!ubx_decoder->configured) {
				ubx_decoder->rx_state = UBX_RXMSG_IGNORE; // ignore if not ubx_decoder->configured
			}
		}break;
		case UBX_MSG_ACK_ACK:{
			if (ubx_decoder->rx_payload_length != sizeof(ubx_payload_rx_ack_ack_t)) {
				ubx_decoder->rx_state = UBX_RXMSG_ERROR_LENGTH;
			} else if (ubx_decoder->configured) {
				ubx_decoder->rx_state = UBX_RXMSG_IGNORE; // ignore if ubx_decoder->configured
			}
		}break;
		case UBX_MSG_ACK_NAK:{
			if (ubx_decoder->rx_payload_length != sizeof(ubx_payload_rx_ack_nak_t)) {
				ubx_decoder->rx_state = UBX_RXMSG_ERROR_LENGTH;
			} else if (ubx_decoder->configured) {
				ubx_decoder->rx_state = UBX_RXMSG_IGNORE; // ignore if ubx_decoder->configured
			}
		}break;

		default:{
			ubx_decoder->rx_state = UBX_RXMSG_DISABLE; // disable all other messages
		}break;
    }

    switch (ubx_decoder->rx_state) {
		case UBX_RXMSG_HANDLE: 			// handle message
		case UBX_RXMSG_IGNORE:{ 		// ignore message but don't report error
			ret = 0;
		}break;
		case UBX_RXMSG_DISABLE:{ 		// disable unexpected messages
			ret = -1;           		// return error, abort handling this message
		}break;
		case UBX_RXMSG_ERROR_LENGTH:{ 	// error: invalid length
			ret = -1;                	// return error, abort handling this message
		}break;

		default:{      					// invalid message state
			ret = -1; 					// return error, abort handling this message
		}break;
    }

    return ret;
}

// -1 = error, 0 = ok, 1 = payload completed
//把接收的数据添加到buf中
static int _payload_rx_add(ubx_decoder_t* ubx_decoder, const uint8_t b)
{
    int ret = 0;
    ubx_decoder->buf.raw[ubx_decoder->rx_payload_index] = b;
    if (++ubx_decoder->rx_payload_index >= ubx_decoder->rx_payload_length) {
        ret = 1; // payload received completely 有效载荷已完全接收
    }
    return ret;
}

// 0 = decoding, 1 = message handled, 2 = sat info message handled
//ubx格式数据解析处理函数
int parse_ubx_char(ubx_decoder_t* ubx_decoder, const uint8_t c)
{
    int ret = 0;
    switch (ubx_decoder->decode_state) {	//解码状态
		case UBX_DECODE_SYNC1: {			//接收到包头低位0xB5
			if (c == UBX_SYNC1) {			
				ubx_decoder->decode_state = UBX_DECODE_SYNC2;
			}
		} break;
		case UBX_DECODE_SYNC2: {			//接收到包头高位0x62
			if (c == UBX_SYNC2) { // Sync2 found --> expecting Class 找到Sync2-->需要Class
				ubx_decoder->decode_state = UBX_DECODE_CLASS;
			} else { // Sync1 not followed by Sync2: reset parser	Sync1后面未跟Sync2：重置解析程序
				reset_ubx_decoder(ubx_decoder);
			}
		} break;
		case UBX_DECODE_CLASS: {			//接收到数据类
			_add_byte_to_checksum(ubx_decoder, c); 
			// checksum is calculated for everything except Sync and Checksum bytes
			//除了同步字节和校验和字节之外，所有字节都计算校验和
			ubx_decoder->rx_msg = c;
			ubx_decoder->decode_state = UBX_DECODE_ID;
		} break;
		case UBX_DECODE_ID: {				//接收到数据ID
			_add_byte_to_checksum(ubx_decoder, c);
			ubx_decoder->rx_msg |= c << 8;
			ubx_decoder->decode_state = UBX_DECODE_LENGTH1;
		} break;
		case UBX_DECODE_LENGTH1: {			//接收到数据长度低位
			_add_byte_to_checksum(ubx_decoder, c);
			ubx_decoder->rx_payload_length = c;
			ubx_decoder->decode_state = UBX_DECODE_LENGTH2;
		} break;
		case UBX_DECODE_LENGTH2: {			//接收到数据长度高位
			_add_byte_to_checksum(ubx_decoder, c);
			ubx_decoder->rx_payload_length |= c << 8; // calculate payload size
			if (_payload_rx_init(ubx_decoder) != 0) { // start payload reception
				// payload will not be handled, discard message
				reset_ubx_decoder(ubx_decoder);
			} else {
				ubx_decoder->decode_state = (ubx_decoder->rx_payload_length > 0) ? UBX_DECODE_PAYLOAD : UBX_DECODE_CHKSUM1;
			}

			//console_printf("len:%d\r\n" , ubx_decoder->rx_payload_length);
		} break;
		case UBX_DECODE_PAYLOAD: {			//接收到数据报文
			_add_byte_to_checksum(ubx_decoder, c);
			switch (ubx_decoder->rx_msg) {
				case UBX_MSG_NAV_SVINFO: {								//0x3001
					ret = _payload_rx_add_nav_svinfo(ubx_decoder, c); 	// add a NAV-SVINFO payload byte
				} break;
				case UBX_MSG_MON_VER: {									//0x040A 版本信息(0x0A 0x04)
					ret = _payload_rx_add_mon_ver(ubx_decoder, c); 		// add a MON-VER payload byte
				} break;
				default: {
					ret = _payload_rx_add(ubx_decoder, c); // add a payload byte
				} break;
			}

			//console_printf("payload ret:%d\r\n" , ret);
			if (ret < 0) {
				// payload not handled, discard message
				reset_ubx_decoder(ubx_decoder);
			} else if (ret > 0) {
				// payload complete, expecting checksum
				ubx_decoder->decode_state = UBX_DECODE_CHKSUM1;
			} else {
				// expecting more payload, stay in state UBX_DECODE_PAYLOAD
			}
			ret = 0;
		} break;
		case UBX_DECODE_CHKSUM1: {			//接收到数据CRC低位
			if (ubx_decoder->rx_ck_a != c) {
				reset_ubx_decoder(ubx_decoder);
			} else {
				ubx_decoder->decode_state = UBX_DECODE_CHKSUM2;
			}
		} break;
		case UBX_DECODE_CHKSUM2: {			//接收到数据CRC高位
			if (ubx_decoder->rx_ck_b != c) {
				//DRV_DBG("ubx checksum2 err\r\n");
			} else {
				ret = ubx_decoder->ubx_rx_handle(); // finish payload processing
			}
			reset_ubx_decoder(ubx_decoder);
		} break;
    }
    return ret;
}

void reset_ubx_decoder(ubx_decoder_t* ubx_decoder)
{
    uint8_t c;
    uint16_t ret;
    ubx_decoder->decode_state = UBX_DECODE_SYNC1;
    ubx_decoder->rx_ck_a = 0;
    ubx_decoder->rx_ck_b = 0;
    ubx_decoder->rx_payload_length = 0;
    ubx_decoder->rx_payload_index = 0;
}

uint8_t init_ubx_decoder(ubx_decoder_t* ubx_decoder, ubx_rx_handle_ptr ubx_rx_handle)
{
    if (ubx_rx_handle == NULL) {
        return FMT_EEMPTY;
    }
    ubx_decoder->ack_state = UBX_ACK_IDLE;
    ubx_decoder->ack_waiting_msg = 0;
    ubx_decoder->got_posllh = false;
    ubx_decoder->got_velned = false;
    ubx_decoder->got_svinfo = false;
    ubx_decoder->configured = false;
    ubx_decoder->use_nav_pvt = false;
    ubx_decoder->ubx_rx_handle = ubx_rx_handle;
    reset_ubx_decoder(ubx_decoder);
    return FMT_EOK;
}

uint8_t send_ubx_msg(ubx_decoder_t* ubx_decoder, const uint16_t msg, const uint8_t* payload, const uint16_t length)
{	
    ubx_header_t header = { UBX_SYNC1, UBX_SYNC2 };
	uint8_t wIndex=0;
    ubx_checksum_t checksum = { 0, 0 };
	memset(ubxBuff_t,0,128);
    // Populate header
    header.msg = msg;
    header.length = length;
    // Calculate checksum
    _calc_ubx_checksum(((uint8_t*)&header) + 2, sizeof(header) - 2, &checksum); // skip 2 sync bytes
    if (payload != NULL) {
        _calc_ubx_checksum(payload, length, &checksum);
    }
	wIndex = sizeof(ubx_header_t);
	memcpy(ubxBuff_t,(uint8_t*)&header,wIndex);
	memcpy(&ubxBuff_t[wIndex],payload,length);
	wIndex += length;
	ubxBuff_t[wIndex] = checksum.ck_a;wIndex++;
	ubxBuff_t[wIndex] = checksum.ck_b;wIndex++;
	if(ubxDevSend!=NULL){
		ubxDevSend(ubxBuff_t,wIndex);
		return 1;
	}
    return 0;
}

/* configure message rates */
/* the last argument is divisor for measurement rate (set by CFG RATE), i.e. 1 means 10Hz 
   配置消息速率
   最后一个参数是测量速率的除数（由CFG rate设置），即1表示10Hz*/
uint8_t configure_ubx_msg_rate(ubx_decoder_t* ubx_decoder, const uint16_t msg, const uint8_t rate)
{
    ubx_payload_tx_cfg_msg_t cfg_msg; // don't use _buf (allow interleaved operation)
    cfg_msg.msg = msg;
    cfg_msg.rate = rate;
    return send_ubx_msg(ubx_decoder, UBX_MSG_CFG_MSG, (uint8_t*)&cfg_msg, sizeof(cfg_msg));
}

void registeUbxSend(void(*Send_t)(uint8_t *src_t,uint16_t srcLen)){
	if(Send_t!=NULL){
		ubxDevSend = Send_t;
	}
}