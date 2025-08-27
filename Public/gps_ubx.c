#include "ublox.h"
#include "hw_config.h"

//static struct gps_device gps_device;
static ubx_decoder_t     ubx_decoder;
//static gps_report_t      gps_report;
uint8_t gpsReadHandle(stcATBuff *SrcBuff){
	uint8_t index_v=0;
	for(index_v=0;index_v<SrcBuff->length_t;index_v++){
		if(parse_ubx_char(&ubx_decoder,SrcBuff->DataBuff[index_v])){//解析失败
			return 0;												//则返回0
		}
	}
	return 1;
}

static int ubx_rx_handle(void)	//ubx数据解析数据处理
{
    int ret = 0;
    // return if no message handled 如果未处理消息，则返回
    if (ubx_decoder.rx_state != UBX_RXMSG_HANDLE) {
        return ret;
    }

    // handle message 处理消息
    switch (ubx_decoder.rx_msg) {
		case UBX_MSG_NAV_PVT: {
			// struct gps_tm timeinfo;
			if ((ubx_decoder.buf.payload_rx_nav_pvt.flags & UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK) == 1) {
	//            gps_report.fix_type      = ubx_decoder.buf.payload_rx_nav_pvt.fixType;
	//            gps_report.vel_ned_valid = 1;

			} else {
	//            gps_report.fix_type      = 0;
	//            gps_report.vel_ned_valid = 0;
			}

//			gps_report.satellites_used = ubx_decoder.buf.payload_rx_nav_pvt.numSV;

//			gps_report.lat = ubx_decoder.buf.payload_rx_nav_pvt.lat;
//			gps_report.lon = ubx_decoder.buf.payload_rx_nav_pvt.lon;
//			gps_report.alt = ubx_decoder.buf.payload_rx_nav_pvt.hMSL;

//			gps_report.eph            = (float)ubx_decoder.buf.payload_rx_nav_pvt.hAcc * 1e-3f;
//			gps_report.epv            = (float)ubx_decoder.buf.payload_rx_nav_pvt.vAcc * 1e-3f;
//			gps_report.s_variance_m_s = (float)ubx_decoder.buf.payload_rx_nav_pvt.sAcc * 1e-3f;

//			gps_report.vel_m_s = (float)ubx_decoder.buf.payload_rx_nav_pvt.gSpeed * 1e-3f;

//			gps_report.vel_n_m_s = (float)ubx_decoder.buf.payload_rx_nav_pvt.velN * 1e-3f;
//			gps_report.vel_e_m_s = (float)ubx_decoder.buf.payload_rx_nav_pvt.velE * 1e-3f;
//			gps_report.vel_d_m_s = (float)ubx_decoder.buf.payload_rx_nav_pvt.velD * 1e-3f;

//			gps_report.cog_rad        = (float)ubx_decoder.buf.payload_rx_nav_pvt.headMot * M_DEG_TO_RAD_F * 1e-5f;
//			gps_report.c_variance_rad = (float)ubx_decoder.buf.payload_rx_nav_pvt.headAcc * M_DEG_TO_RAD_F * 1e-5f;

			// Check if time and date fix flags are good
			if ((ubx_decoder.buf.payload_rx_nav_pvt.valid & UBX_RX_NAV_PVT_VALID_VALIDDATE)
				&& (ubx_decoder.buf.payload_rx_nav_pvt.valid & UBX_RX_NAV_PVT_VALID_VALIDTIME)
				&& (ubx_decoder.buf.payload_rx_nav_pvt.valid & UBX_RX_NAV_PVT_VALID_FULLYRESOLVED)) {
				/* convert to unix timestamp */
				// timeinfo.tm_year	= ubx_decoder.buf.payload_rx_nav_pvt.year - 1900;
				// timeinfo.tm_mon		= ubx_decoder.buf.payload_rx_nav_pvt.month - 1;
				// timeinfo.tm_mday	= ubx_decoder.buf.payload_rx_nav_pvt.day;
				// timeinfo.tm_hour	= ubx_decoder.buf.payload_rx_nav_pvt.hour;
				// timeinfo.tm_min		= ubx_decoder.buf.payload_rx_nav_pvt.min;
				// timeinfo.tm_sec		= ubx_decoder.buf.payload_rx_nav_pvt.sec;

				// gps_report.time_utc_usec = 0;
			}
	//        gps_report.timestamp_time     = systime_now_ms();
	//        gps_report.timestamp_velocity = systime_now_ms();
	//        gps_report.timestamp_variance = systime_now_ms();
	//        gps_report.timestamp_position = systime_now_ms();
			ubx_decoder.got_posllh = true;
			ubx_decoder.got_velned = true;
			ret = 1;
		} break;
/*
		case UBX_MSG_NAV_POSLLH: {
			// console_printf("Rx NAV-POSLLH\r\n");

			gps_report.lat           = ubx_decoder.buf.payload_rx_nav_posllh.lat;
			gps_report.lon           = ubx_decoder.buf.payload_rx_nav_posllh.lon;
			gps_report.alt           = ubx_decoder.buf.payload_rx_nav_posllh.hMSL;
			gps_report.eph           = (float)ubx_decoder.buf.payload_rx_nav_posllh.hAcc * 1e-3f; // from mm to m
			gps_report.epv           = (float)ubx_decoder.buf.payload_rx_nav_posllh.vAcc * 1e-3f; // from mm to m
			gps_report.alt_ellipsoid = ubx_decoder.buf.payload_rx_nav_posllh.height;
			gps_report.timestamp_position = systime_now_ms();
			ubx_decoder.got_posllh = true;
			// console_printf("alt:%d lat:%d lon:%d\r\n" , gps_report.alt, gps_report.lat,gps_report.lon);
			ret = 1;
		} break;

		case UBX_MSG_NAV_SOL: {
			// console_printf("Rx NAV-SOL\r\n");

			gps_report.fix_type        = ubx_decoder.buf.payload_rx_nav_sol.gpsFix;
			gps_report.s_variance_m_s  = (float)ubx_decoder.buf.payload_rx_nav_sol.sAcc * 1e-2f; // from cm to m
			gps_report.satellites_used = ubx_decoder.buf.payload_rx_nav_sol.numSV;

			gps_report.timestamp_variance = systime_now_ms();
		} break;

		case UBX_MSG_NAV_DOP: {
			// console_printf("Rx NAV-DOP\r\n");

			gps_report.hdop = ubx_decoder.buf.payload_rx_nav_dop.hDOP * 0.01f; // from cm to m
			gps_report.vdop = ubx_decoder.buf.payload_rx_nav_dop.vDOP * 0.01f; // from cm to m
			gps_report.tdop = ubx_decoder.buf.payload_rx_nav_dop.tDOP * 0.01f; // from cm to m
			gps_report.ndop = ubx_decoder.buf.payload_rx_nav_dop.nDOP * 0.01f; // from cm to m
			gps_report.edop = ubx_decoder.buf.payload_rx_nav_dop.eDOP * 0.01f; // from cm to m

			gps_report.timestamp_variance = systime_now_ms();

			ret = 1;
		} break;

		case UBX_MSG_NAV_TIMEUTC: {
			// console_printf("Rx NAV-TIMEUTC\r\n");

			// if (ubx_decoder.buf.payload_rx_nav_timeutc.valid & UBX_RX_NAV_TIMEUTC_VALID_VALIDUTC) {
			//     convert to unix timestamp
			//     struct gps_tm timeinfo;
			//     timeinfo.tm_year	= ubx_decoder.buf.payload_rx_nav_timeutc.year - 1900;
			//     timeinfo.tm_mon		= ubx_decoder.buf.payload_rx_nav_timeutc.month - 1;
			//     timeinfo.tm_mday	= ubx_decoder.buf.payload_rx_nav_timeutc.day;
			//     timeinfo.tm_hour	= ubx_decoder.buf.payload_rx_nav_timeutc.hour;
			//     timeinfo.tm_min		= ubx_decoder.buf.payload_rx_nav_timeutc.min;
			//     timeinfo.tm_sec		= ubx_decoder.buf.payload_rx_nav_timeutc.sec;

			//     gps_report.time_utc_usec = 0;
			//     console_printf("%d-%d-%d %d:%d:%d\r\n" , timeinfo.tm_year,timeinfo.tm_mon,timeinfo.tm_mday,timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec);
			// }

			// gps_report.timestamp_time = systime_now_ms();

			ret = 1;
		} break;

		case UBX_MSG_NAV_SVINFO: {
			// console_printf("Rx NAV-SVINFO\r\n");
			//  _satellite_info already populated by payload_rx_add_svinfo(), just add a timestamp
			ubx_decoder.satellite_info.timestamp = systime_now_us();

			ubx_decoder.got_svinfo = true;

			ret = 2;
		} break;

		case UBX_MSG_NAV_VELNED: {
			gps_report.vel_m_s        = (float)ubx_decoder.buf.payload_rx_nav_velned.speed * 1e-2f;
			gps_report.vel_n_m_s      = (float)ubx_decoder.buf.payload_rx_nav_velned.velN * 1e-2f; // NED NORTH velocity 
			gps_report.vel_e_m_s      = (float)ubx_decoder.buf.payload_rx_nav_velned.velE * 1e-2f; // NED EAST velocity
			gps_report.vel_d_m_s      = (float)ubx_decoder.buf.payload_rx_nav_velned.velD * 1e-2f; // NED DOWN velocity
			gps_report.cog_rad        = (float)ubx_decoder.buf.payload_rx_nav_velned.heading * M_DEG_TO_RAD_F * 1e-5f;
			gps_report.c_variance_rad = (float)ubx_decoder.buf.payload_rx_nav_velned.cAcc * M_DEG_TO_RAD_F * 1e-5f;
			gps_report.vel_ned_valid  = 1;

			gps_report.timestamp_velocity = systime_now_ms();

			ubx_decoder.got_velned = true;

			// console_printf("nV:%.2f eV:%.2f dV:%.2f" , gps_report.vel_n_m_s,gps_report.vel_e_m_s,gps_report.vel_d_m_s);

			ret = 1;
		} break;

		case UBX_MSG_MON_VER: {
			// console_printf("Rx MON-VER\r\n");
		} break;

		case UBX_MSG_MON_HW: {
			// console_printf("Rx MON-HW\r\n");

			switch (ubx_decoder.rx_payload_length) {
			case sizeof(ubx_payload_rx_mon_hw_ubx6_t): // u-blox 6 msg format 
				gps_report.noise_per_ms      = ubx_decoder.buf.payload_rx_mon_hw_ubx6.noisePerMS;
				gps_report.jamming_indicator = ubx_decoder.buf.payload_rx_mon_hw_ubx6.jamInd;

				ret = 1;
				break;

			case sizeof(ubx_payload_rx_mon_hw_ubx7_t): // u-blox 7+ msg format
				gps_report.noise_per_ms      = ubx_decoder.buf.payload_rx_mon_hw_ubx7.noisePerMS;
				gps_report.jamming_indicator = ubx_decoder.buf.payload_rx_mon_hw_ubx7.jamInd;

				ret = 1;
				break;

			default:     // unexpected payload size:
				ret = 0; // don't handle message
				break;
			}
		} break;

		case UBX_MSG_ACK_ACK: {
			// DRV_DBG("Rx ACK-ACK\r\n");

			if ((ubx_decoder.ack_state == UBX_ACK_WAITING) && (ubx_decoder.buf.payload_rx_ack_ack.msg == ubx_decoder.ack_waiting_msg)) {
				ubx_decoder.ack_state = UBX_ACK_GOT_ACK;
			}

			ret = 1;
		} break;

		case UBX_MSG_ACK_NAK: {
        // DRV_DBG("Rx ACK-NAK\r\n");

        if ((ubx_decoder.ack_state == UBX_ACK_WAITING) && (ubx_decoder.buf.payload_rx_ack_ack.msg == ubx_decoder.ack_waiting_msg)) {
            ubx_decoder.ack_state = UBX_ACK_GOT_NAK;
        }

        ret = 1;
    } break;
	*/
		default:
			break;
    }
    return ret;
}