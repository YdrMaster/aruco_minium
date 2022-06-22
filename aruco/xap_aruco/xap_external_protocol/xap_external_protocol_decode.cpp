#include "xap_external_protocol_decode.h"
#include "stdio.h"

int xapExternalProtocolParseChar(xap_external_protocol_decoder_t *decoder, const uint8_t b)
{
    //printf("%x\n",b);
    int ret = 0;

    switch (decoder->decodeState)
    {
    case XAP_EXTERNAL_PROTOCOL_DECODE_SYNC1:
        if (b == XAP_EXTERNAL_SYNC1)
        {
            //printf("SYNC1\n");
            decoder->decodeState = XAP_EXTERNAL_PROTOCOL_DECODE_SYNC2;
        }
        break;

    case XAP_EXTERNAL_PROTOCOL_DECODE_SYNC2:
        if (b == XAP_EXTERNAL_SYNC2)
        {
            //printf("SYNC2\n");
            decoder->decodeState = XAP_EXTERNAL_PROTOCOL_DECODE_MSG_ID;
            xap_external_crc_init(&decoder->crc);
        }
        else
        {
            xapExternalProtocolDecodeInit(decoder);
        }
        break;

    case XAP_EXTERNAL_PROTOCOL_DECODE_MSG_ID:
        //printf("ID\n");
        decoder->crc = xap_external_crc_table_accumulate_byte(b, decoder->crc);
        decoder->msg_id = b;
        decoder->decodeState = XAP_EXTERNAL_PROTOCOL_DECODE_MSG_CLASS;
        break;

    case XAP_EXTERNAL_PROTOCOL_DECODE_MSG_CLASS:
        //printf("CLASS\n");
        decoder->crc = xap_external_crc_table_accumulate_byte(b, decoder->crc);
        decoder->msg_class = b;
        decoder->decodeState = XAP_EXTERNAL_PROTOCOL_DECODE_LENGTH1;
        break;

    case XAP_EXTERNAL_PROTOCOL_DECODE_LENGTH1:
        //printf("LENGTH1\n");
        decoder->crc = xap_external_crc_table_accumulate_byte(b, decoder->crc);
        decoder->rx_payload_length = b;
        decoder->decodeState = XAP_EXTERNAL_PROTOCOL_DECODE_LENGTH2;
        break;

    case XAP_EXTERNAL_PROTOCOL_DECODE_LENGTH2:
        //printf("LENGTH2\n");
        decoder->crc = xap_external_crc_table_accumulate_byte(b, decoder->crc);
        decoder->rx_payload_length |= b << 8;
        if (xapExternalProtocolPayloadRxInit(decoder) != 0)
        {
            xapExternalProtocolDecodeInit(decoder);
        }
        else
        {
            decoder->decodeState = (decoder->rx_payload_length > 0) ? XAP_EXTERNAL_PROTOCOL_DECODE_PAYLOAD : XAP_EXTERNAL_PROTOCOL_DECODE_CRC1;
        }
        break;

    case XAP_EXTERNAL_PROTOCOL_DECODE_PAYLOAD:
        //printf("PAYLOAD\n");
        decoder->crc = xap_external_crc_table_accumulate_byte(b, decoder->crc);
        ret = xapExternalProtocolPayloadRxAdd(decoder, b);

        if (ret < 0)
        {
            // payload not handled, discard message
            xapExternalProtocolDecodeInit(decoder);
        }
        else if (ret > 0)
        {
            // payload complete, expecting checksum
            decoder->decodeState = XAP_EXTERNAL_PROTOCOL_DECODE_CRC1;
        }
        else
        {
            // expecting more payload, stay in state UBX_DECODE_PAYLOAD
        }

        ret = 0;
        break;

    case XAP_EXTERNAL_PROTOCOL_DECODE_CRC1:
        //printf("CRC1\n");
        decoder->msg_crc = b;
        decoder->decodeState = XAP_EXTERNAL_PROTOCOL_DECODE_CRC2;
        break;

    case XAP_EXTERNAL_PROTOCOL_DECODE_CRC2:
        //printf("CRC2\n");
        decoder->msg_crc |= b << 8;
        if (decoder->msg_crc != decoder->crc)
        {
            xapExternalProtocolDecodeInit(decoder);
        }
        else
        {
            decoder->decodeState = XAP_EXTERNAL_PROTOCOL_DECODE_END;
        }
        break;

    case XAP_EXTERNAL_PROTOCOL_DECODE_END:
        //printf("END1\n");

        if (b != XAP_EXTERNAL_END)
        {
        }
        else
        {
            //printf("END2\n");
            ret = xapExternalProtocolPayloadRxDone(decoder);
        }
        //printf("END3\n");
        xapExternalProtocolDecodeInit(decoder);
        //printf("END4\n");
        break;
    }

    return ret;
}

void xapExternalProtocolDecodeInit(xap_external_protocol_decoder_t *decoder)
{
    decoder->decodeState = XAP_EXTERNAL_PROTOCOL_DECODE_SYNC1;
    decoder->rx_payload_length = 0;
    decoder->rx_payload_index = 0;
    xap_external_crc_init(&decoder->crc);
}

int xapExternalProtocolPayloadRxInit(xap_external_protocol_decoder_t *decoder)
{
    int ret = 0;

    decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_HANDLE;

    switch (decoder->msg_class)
    {
    case XAP_EXTERNAL_CLASS_HIL_SBG:
    {
        switch (decoder->msg_id)
        {
        case XAP_EXTERNAL_HIL_SBG_IMU_DATA:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_sbg_imu_data_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_HIL_SBG_EKF_EULER:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_sbg_ekf_euler_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_HIL_SBG_EKF_QUAT:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_sbg_ekf_quat_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_HIL_SBG_EKF_NAV:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_sbg_ekf_nav_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_HIL_SBG_GPS1_VEL:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_sbg_gps_vel_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_HIL_SBG_GPS1_POS:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_sbg_gps_pos_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_HIL_SBG_GPS1_HDT:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_sbg_gps_hdt_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_HIL_SBG_GPS2_VEL:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_sbg_gps_vel_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_HIL_SBG_GPS2_POS:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_sbg_gps_pos_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_HIL_SBG_GPS2_HDT:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_sbg_gps_hdt_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_HIL_SBG_AIR_DATA:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_sbg_air_data_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        default:
            decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_DISABLE;
            break;
        }
    }
    break;

    case XAP_EXTERNAL_CLASS_HIL_PX4:
    {
        switch (decoder->msg_id)
        {
        case XAP_EXTERNAL_HIL_PX4_GYRO:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_px4_gyro_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_HIL_PX4_ACCEL:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_px4_accel_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_HIL_PX4_MAG:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_px4_mag_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_HIL_PX4_BARO:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_px4_baro_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_HIL_PX4_GPS:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_px4_gps_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_HIL_PX4_AIRSPEED:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_px4_airspeed_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        default:
            decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_DISABLE;
            break;
        }
    }
    break;

    case XAP_EXTERNAL_CLASS_HIL_XAP:
    {
        switch (decoder->msg_id)
        {
        case XAP_EXTERNAL_HIL_XAP_GYRO:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_xap_gyro_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_HIL_XAP_ACCEL:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_xap_accel_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_HIL_XAP_MAG:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_xap_mag_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_HIL_XAP_BARO:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_xap_baro_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_HIL_XAP_GPS:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_xap_gps_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_HIL_XAP_AIRSPEED:
            if (decoder->rx_payload_length != sizeof(xap_external_hil_xap_airspeed_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        default:
            decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_DISABLE;
            break;
        }
    }
    break;

    case XAP_EXTERNAL_CLASS_FC_STATUS:
    {
        switch (decoder->msg_id)
        {
        case XAP_EXTERNAL_FC_STATUS_ATT:
            if (decoder->rx_payload_length != sizeof(xap_external_fc_status_att_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        default:
            decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_DISABLE;
            break;
        }
    }
    break;

    case XAP_EXTERNAL_CLASS_FC_CMD:
    {
        switch (decoder->msg_id)
        {
        case XAP_EXTERNAL_FC_CMD_MPWM:
            if (decoder->rx_payload_length != sizeof(xap_external_fc_cmd_mpwm_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_FC_CMD_APWM:
            if (decoder->rx_payload_length != sizeof(xap_external_fc_cmd_apwm_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        default:
            decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_DISABLE;
            break;
        }
    }
    break;

    case XAP_EXTERNAL_CLASS_MCP_STATUS:
    {
        switch (decoder->msg_id)
        {

        default:
            decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_DISABLE;
            break;
        }
    }
    break;

    case XAP_EXTERNAL_CLASS_MCP_CMD:
    {
        switch (decoder->msg_id)
        {

        default:
            decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_DISABLE;
            break;
        }
    }
    break;

    case XAP_EXTERNAL_CLASS_MCP_NAV:
    {
        switch (decoder->msg_id)
        {
        case XAP_EXTERNAL_MCP_NAV_ARUCO:
            if (decoder->rx_payload_length != sizeof(xap_external_mcp_nav_aruco_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        default:
            decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_DISABLE;
            break;
        }
    }
    break;

    case XAP_EXTERNAL_CLASS_SENS_CAMERA:
    {
        switch (decoder->msg_id)
        {
        case XAP_EXTERNAL_SENS_CAMERA_INFO:
            if (decoder->rx_payload_length != sizeof(xap_external_sens_camera_info_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_SENS_CAMERA_ROI:
            if (decoder->rx_payload_length != sizeof(xap_external_sens_camera_roi_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        case XAP_EXTERNAL_SENS_CAMERA_TARGET_INFO:
            if (decoder->rx_payload_length != sizeof(xap_external_sens_camera_target_info_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        default:
            decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_DISABLE;
            break;
        }
    }
    break;

    case XAP_EXTERNAL_CLASS_COMMON_TIME:
    {
        switch (decoder->msg_id)
        {
        case XAP_EXTERNAL_COMMON_TIME_SYNC:
            if (decoder->rx_payload_length != sizeof(xap_external_common_time_sync_t))
                decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
            break;

        default:
            decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_DISABLE;
            break;
        }
    }
    break;

    default:
        decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_DISABLE;
        break;
    }

    if(decoder->rx_payload_length > sizeof(decoder->rx_buffer))
    {
        decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH;
    }

    switch (decoder->rxState)
    {
    case XAP_EXTERNAL_PROTOCOL_RXMSG_HANDLE:
    case XAP_EXTERNAL_PROTOCOL_RXMSG_IGNORE:
        ret = 0;
        break;

    case XAP_EXTERNAL_PROTOCOL_RXMSG_DISABLE:
        ret = -1;
        break;

    case XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH:
        ret = -1;
        break;

    default:
        ret = -1;
        break;
    }

    return ret;
}

int xapExternalProtocolPayloadRxAdd(xap_external_protocol_decoder_t *decoder, const uint8_t b)
{
    int ret = 0;

    if(decoder->rx_payload_length >= sizeof(decoder->rx_buffer))
    {
        // 避免消息长度超过接收缓存引起内存溢出
        return -1;
    }

    uint8_t *p_buf = (uint8_t *)&decoder->rx_buffer;

    p_buf[decoder->rx_payload_index] = b;

    if (++decoder->rx_payload_index >= decoder->rx_payload_length)
    {
        ret = 1; // payload received completely
    }

    return ret;
}

int xapExternalProtocolPayloadRxDone(xap_external_protocol_decoder_t *decoder)
{
    int ret = 0;

    // return if no message handled
    if (decoder->rxState != XAP_EXTERNAL_PROTOCOL_RXMSG_HANDLE)
    {
        return ret;
    }

    // handle message
    switch (decoder->msg_class)
    {
    case XAP_EXTERNAL_CLASS_HIL_SBG:
    {
        switch (decoder->msg_id)
        {
        case XAP_EXTERNAL_HIL_SBG_IMU_DATA:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        case XAP_EXTERNAL_HIL_SBG_EKF_EULER:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        case XAP_EXTERNAL_HIL_SBG_EKF_QUAT:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        case XAP_EXTERNAL_HIL_SBG_EKF_NAV:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        case XAP_EXTERNAL_HIL_SBG_GPS1_VEL:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        case XAP_EXTERNAL_HIL_SBG_GPS1_POS:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        case XAP_EXTERNAL_HIL_SBG_GPS1_HDT:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        case XAP_EXTERNAL_HIL_SBG_GPS2_VEL:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        case XAP_EXTERNAL_HIL_SBG_GPS2_POS:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        case XAP_EXTERNAL_HIL_SBG_GPS2_HDT:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        case XAP_EXTERNAL_HIL_SBG_AIR_DATA:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        default:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;
        }
    }
    break;

    case XAP_EXTERNAL_CLASS_HIL_PX4:
    {
        switch (decoder->msg_id)
        {

        case XAP_EXTERNAL_HIL_PX4_GYRO:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        case XAP_EXTERNAL_HIL_PX4_ACCEL:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        case XAP_EXTERNAL_HIL_PX4_MAG:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        case XAP_EXTERNAL_HIL_PX4_BARO:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        case XAP_EXTERNAL_HIL_PX4_GPS:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        case XAP_EXTERNAL_HIL_PX4_AIRSPEED:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        default:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;
        }
    }
    break;

    case XAP_EXTERNAL_CLASS_FC_STATUS:
    {
        switch (decoder->msg_id)
        {
        case XAP_EXTERNAL_FC_STATUS_ATT:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        default:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;
        }
    }
    break;

    case XAP_EXTERNAL_CLASS_FC_CMD:
    {
        switch (decoder->msg_id)
        {
        case XAP_EXTERNAL_FC_CMD_MPWM:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        case XAP_EXTERNAL_FC_CMD_APWM:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        default:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;
        }
    }
    break;

    case XAP_EXTERNAL_CLASS_MCP_STATUS:
    {
        switch (decoder->msg_id)
        {

        default:
            decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_DISABLE;
            break;
        }
    }
    break;

    case XAP_EXTERNAL_CLASS_MCP_CMD:
    {
        switch (decoder->msg_id)
        {

        default:
            decoder->rxState = XAP_EXTERNAL_PROTOCOL_RXMSG_DISABLE;
            break;
        }
    }
    break;

    case XAP_EXTERNAL_CLASS_MCP_NAV:
    {
        switch (decoder->msg_id)
        {
        case XAP_EXTERNAL_MCP_NAV_ARUCO:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        default:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;
        }
    }
    break;

    case XAP_EXTERNAL_CLASS_SENS_CAMERA:
    {
        switch (decoder->msg_id)
        {
        case XAP_EXTERNAL_SENS_CAMERA_INFO:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        case XAP_EXTERNAL_SENS_CAMERA_ROI:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        case XAP_EXTERNAL_SENS_CAMERA_TARGET_INFO:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        default:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;
        }
    }
    break;

    case XAP_EXTERNAL_CLASS_COMMON_TIME:
    {
        switch (decoder->msg_id)
        {
        case XAP_EXTERNAL_COMMON_TIME_SYNC:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;

        default:
            ret = decoder->msg_class << 8 | decoder->msg_id;
            break;
        }
    }
    break;

    default:
        ret = decoder->msg_class << 8 | decoder->msg_id;
        break;
    }

    return ret;
}