
#ifndef _XAP_EXTERNAL_PROTOCOL_DECODE_H_
#define _XAP_EXTERNAL_PROTOCOL_DECODE_H_

#include <stdint.h>
#include <xap_external_protocol.h>

/*** xap external protocol binary message and payload definitions ***/
#pragma pack(push, 1)

/* Decoder received message buffer struct */
typedef union
{
    /* XAP_EXTERNAL_CLASS_HIL_SBG */
    xap_external_hil_sbg_imu_data_t xap_external_hil_sbg_imu_data;
    xap_external_hil_sbg_ekf_euler_t xap_external_hil_sbg_ekf_euler;
    xap_external_hil_sbg_ekf_quat_t xap_external_hil_sbg_ekf_quat;
    xap_external_hil_sbg_ekf_nav_t xap_external_hil_sbg_ekf_nav;
    xap_external_hil_sbg_gps_vel_t xap_external_hil_sbg_gps_vel;
    xap_external_hil_sbg_gps_pos_t xap_external_hil_sbg_gps_pos;
    xap_external_hil_sbg_gps_hdt_t xap_external_hil_sbg_gps_hdt;
    xap_external_hil_sbg_air_data_t xap_external_hil_sbg_air_data;

    /* XAP_EXTERNAL_CLASS_HIL_PX4 */
    xap_external_hil_px4_gyro_t xap_external_hil_px4_gyro;
    xap_external_hil_px4_accel_t xap_external_hil_px4_accel;
    xap_external_hil_px4_mag_t xap_external_hil_px4_mag;
    xap_external_hil_px4_baro_t xap_external_hil_px4_baro;
    xap_external_hil_px4_gps_t xap_external_hil_px4_gps;
    xap_external_hil_px4_airspeed_t xap_external_hil_px4_airspeed;

    /* XAP_EXTERNAL_CLASS_HIL_XAP */
    xap_external_hil_xap_gyro_t xap_external_hil_xap_gyro;
    xap_external_hil_xap_accel_t xap_external_hil_xap_accel;
    xap_external_hil_xap_mag_t xap_external_hil_xap_mag;
    xap_external_hil_xap_baro_t xap_external_hil_xap_baro;
    xap_external_hil_xap_gps_t xap_external_hil_xap_gps;
    xap_external_hil_xap_airspeed_t xap_external_hil_xap_airspeed;

    /* XAP_EXTERNAL_CLASS_FC_STATUS */
    xap_external_fc_status_att_t xap_external_fc_status_att;

    /* XAP_EXTERNAL_CLASS_FC_CMD */
    xap_external_fc_cmd_mpwm_t xap_external_fc_cmd_mpwm;
    xap_external_fc_cmd_apwm_t xap_external_fc_cmd_apwm;

    /* XAP_EXTERNAL_CLASS_MCP_NAV */
    xap_external_mcp_nav_aruco_t xap_external_mcp_nav_aruco;

    /* XAP_EXTERNAL_CLASS_SENS_CAMERA */
    xap_external_sens_camera_roi_t xap_external_sens_camera_roi;
    xap_external_sens_camera_info_t xap_external_sens_camera_info;
    xap_external_sens_camera_target_info_t xap_external_sens_camera_target_info;

    /* XAP_EXTERNAL_CLASS_COMMON_TIME */
    xap_external_common_time_sync_t xap_external_common_time_sync;
} xap_external_protocol_rx_payload_buf_t;

/* Decoder send message buffer struct */
typedef union
{
    /* XAP_EXTERNAL_CLASS_HIL_SBG */
    xap_external_hil_sbg_imu_data_t xap_external_hil_sbg_imu_data;
    xap_external_hil_sbg_ekf_euler_t xap_external_hil_sbg_ekf_euler;
    xap_external_hil_sbg_ekf_quat_t xap_external_hil_sbg_ekf_quat;
    xap_external_hil_sbg_ekf_nav_t xap_external_hil_sbg_ekf_nav;
    xap_external_hil_sbg_gps_vel_t xap_external_hil_sbg_gps_vel;
    xap_external_hil_sbg_gps_pos_t xap_external_hil_sbg_gps_pos;
    xap_external_hil_sbg_gps_hdt_t xap_external_hil_sbg_gps_hdt;
    xap_external_hil_sbg_air_data_t xap_external_hil_sbg_air_data;

    /* XAP_EXTERNAL_CLASS_HIL_PX4 */
    xap_external_hil_px4_gyro_t xap_external_hil_px4_gyro;
    xap_external_hil_px4_accel_t xap_external_hil_px4_accel;
    xap_external_hil_px4_mag_t xap_external_hil_px4_mag;
    xap_external_hil_px4_baro_t xap_external_hil_px4_baro;
    xap_external_hil_px4_gps_t xap_external_hil_px4_gps;
    xap_external_hil_px4_airspeed_t xap_external_hil_px4_airspeed;

    /* XAP_EXTERNAL_CLASS_HIL_XAP */
    xap_external_hil_xap_gyro_t xap_external_hil_xap_gyro;
    xap_external_hil_xap_accel_t xap_external_hil_xap_accel;
    xap_external_hil_xap_mag_t xap_external_hil_xap_mag;
    xap_external_hil_xap_baro_t xap_external_hil_xap_baro;
    xap_external_hil_xap_gps_t xap_external_hil_xap_gps;
    xap_external_hil_xap_airspeed_t xap_external_hil_xap_airspeed;

    /* XAP_EXTERNAL_CLASS_FC_STATUS */
    xap_external_fc_status_att_t xap_external_fc_status_att;

    /* XAP_EXTERNAL_CLASS_FC_CMD */
    xap_external_fc_cmd_mpwm_t xap_external_fc_cmd_mpwm;
    xap_external_fc_cmd_apwm_t xap_external_fc_cmd_apwm;

    /* XAP_EXTERNAL_CLASS_MCP_NAV */
    xap_external_mcp_nav_aruco_t xap_external_mcp_nav_aruco;

    /* XAP_EXTERNAL_CLASS_SENS_CAMERA */
    xap_external_sens_camera_roi_t xap_external_sens_camera_roi;
    xap_external_sens_camera_info_t xap_external_sens_camera_info;
    xap_external_sens_camera_target_info_t xap_external_sens_camera_target_info;

    /* XAP_EXTERNAL_CLASS_COMMON_TIME */
    xap_external_common_time_sync_t xap_external_common_time_sync;
} xap_external_protocol_tx_payload_buf_t;

#pragma pack(pop)
/*** END OF xap external protocol binary message and payload definitions ***/

/* Decoder state */
typedef enum
{
    XAP_EXTERNAL_PROTOCOL_DECODE_SYNC1 = 0,
    XAP_EXTERNAL_PROTOCOL_DECODE_SYNC2,
    XAP_EXTERNAL_PROTOCOL_DECODE_MSG_ID,
    XAP_EXTERNAL_PROTOCOL_DECODE_MSG_CLASS,
    XAP_EXTERNAL_PROTOCOL_DECODE_LENGTH1,
    XAP_EXTERNAL_PROTOCOL_DECODE_LENGTH2,
    XAP_EXTERNAL_PROTOCOL_DECODE_PAYLOAD,
    XAP_EXTERNAL_PROTOCOL_DECODE_CRC1,
    XAP_EXTERNAL_PROTOCOL_DECODE_CRC2,
    XAP_EXTERNAL_PROTOCOL_DECODE_END,
} xap_external_protocol_decode_state_t;

/* Rx message state */
typedef enum
{
    XAP_EXTERNAL_PROTOCOL_RXMSG_IGNORE = 0,
    XAP_EXTERNAL_PROTOCOL_RXMSG_HANDLE,
    XAP_EXTERNAL_PROTOCOL_RXMSG_DISABLE,
    XAP_EXTERNAL_PROTOCOL_RXMSG_ERROR_LENGTH
} xap_external_protocol_rxmsg_state_t;

typedef struct
{
    uint8_t decodeState;
    uint8_t rxState;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t crc;
    uint16_t msg_crc;
    uint16_t rx_payload_length;
    uint16_t rx_payload_index;
    xap_external_protocol_rx_payload_buf_t rx_buffer;
} xap_external_protocol_decoder_t;

int xapExternalProtocolParseChar(xap_external_protocol_decoder_t *decoder, const uint8_t b);

void xapExternalProtocolDecodeInit(xap_external_protocol_decoder_t *decoder);

int xapExternalProtocolPayloadRxInit(xap_external_protocol_decoder_t *decoder);

int xapExternalProtocolPayloadRxAdd(xap_external_protocol_decoder_t *decoder, const uint8_t b);

int xapExternalProtocolPayloadRxDone(xap_external_protocol_decoder_t *decoder);

#endif /* _XAP_EXTERNAL_PROTOCOL_DECODE_H_ */
