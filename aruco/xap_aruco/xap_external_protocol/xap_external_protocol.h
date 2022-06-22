#ifndef __XAP_EXTERNAL_PROTOCOL__
#define __XAP_EXTERNAL_PROTOCOL__

#include <stdint.h>
#include <stdbool.h>

#define XAP_EXTERNAL_SYNC1 0xFF
#define XAP_EXTERNAL_SYNC2 0x5A

#define XAP_EXTERNAL_END 0x33

/* Classes */
#define XAP_EXTERNAL_CLASS_HIL_SBG      0x00            /* HIL SBG模拟数据消息类    发送:HIL */
#define XAP_EXTERNAL_CLASS_HIL_PX4      0x01            /* HIL PX4模拟数据消息类    发送:HIL */
#define XAP_EXTERNAL_CLASS_HIL_XAP      0x05            /* HIL XAP模拟数据消息类    发送:HIL */

#define XAP_EXTERNAL_CLASS_FC_STATUS    0x0A            /* FC 飞控状态消息类       发送:飞控 */
#define XAP_EXTERNAL_CLASS_FC_CMD       0x0C            /* FC 飞控指令消息类       发送:飞控 */

#define XAP_EXTERNAL_CLASS_MCP_STATUS   0x10            /* 任务机 任务机状态消息类   发送:任务机 接收:飞控 */
#define XAP_EXTERNAL_CLASS_MCP_CMD      0x11            /* 任务机 任务机指令消息类   发送:任务机 接收:飞控 */
#define XAP_EXTERNAL_CLASS_MCP_NAV      0x12            /* 任务机 任务机导航消息类   发送:任务机 接收:飞控 */

#define XAP_EXTERNAL_CLASS_SENS_CAMERA  0x20            /* 传感器 相机消息类 发送:相机 */

#define XAP_EXTERNAL_CLASS_COMMON_TIME  0xF0            /* 通用 时间消息类 */

/* IDs */
/* HIL SBG消息ID */
#define XAP_EXTERNAL_HIL_SBG_IMU_DATA             	03  /*	Includes IMU status, acc., gyro, temp delta speeds and delta angles values */
#define XAP_EXTERNAL_HIL_SBG_EKF_EULER            	06  /*	Includes roll, pitch, yaw and their accuracies on each axis */
#define XAP_EXTERNAL_HIL_SBG_EKF_QUAT             	07  /*	Includes the 4 quaternions values */
#define XAP_EXTERNAL_HIL_SBG_EKF_NAV 			  	0x08  /*	Position and velocities in NED coordinates with the accuracies on each axis */
#define XAP_EXTERNAL_HIL_SBG_GPS1_VEL 				13  /*	GNSS velocity from primary receiver */
#define XAP_EXTERNAL_HIL_SBG_GPS1_POS 				14  /*	GNSS position from primary receiver */
#define XAP_EXTERNAL_HIL_SBG_GPS1_HDT 				15  /*	GNSS true heading from primary receiver */
#define XAP_EXTERNAL_HIL_SBG_GPS2_VEL 				16  /*	GNSS velocity from secondary receiver */
#define XAP_EXTERNAL_HIL_SBG_GPS2_POS 				17  /*	GNSS position from secondary receiver */
#define XAP_EXTERNAL_HIL_SBG_GPS2_HDT 				18  /*	GNSS true heading from secondary receiver */
#define XAP_EXTERNAL_HIL_SBG_AIR_DATA 				36  /*	Air data output */

/* HIL PX4消息ID */
#define XAP_EXTERNAL_HIL_PX4_GYRO                 01  /* PX4 gyro */
#define XAP_EXTERNAL_HIL_PX4_ACCEL                02  /* PX4 accel */
#define XAP_EXTERNAL_HIL_PX4_MAG                  03  /* PX4 mag */
#define XAP_EXTERNAL_HIL_PX4_BARO                 04  /* PX4 baro */
#define XAP_EXTERNAL_HIL_PX4_GPS                  05  /* PX4 gps */
#define XAP_EXTERNAL_HIL_PX4_AIRSPEED             06  /* PX4 airspeed */

/* HIL XAP消息ID */
#define XAP_EXTERNAL_HIL_XAP_GYRO                 01  /* XAP gyro */
#define XAP_EXTERNAL_HIL_XAP_ACCEL                02  /* XAP accel */
#define XAP_EXTERNAL_HIL_XAP_MAG                  03  /* XAP mag */
#define XAP_EXTERNAL_HIL_XAP_BARO                 04  /* XAP baro */
#define XAP_EXTERNAL_HIL_XAP_GPS                  05  /* XAP gps */
#define XAP_EXTERNAL_HIL_XAP_AIRSPEED             06  /* XAP airspeed */

/* FC STATUS消息ID */
#define XAP_EXTERNAL_FC_STATUS_ATT               01  /* Autopilot attitude */

/* FC CMD消息ID */
#define XAP_EXTERNAL_FC_CMD_MPWM                 01  /* Main pwm values */
#define XAP_EXTERNAL_FC_CMD_APWM                 02  /* Aux pwm values */

/* MCP NAV消息ID */
#define XAP_EXTERNAL_MCP_NAV_ARUCO               01  /* Aruco detect marker's position */

/* SENS CAMERA消息ID */
#define XAP_EXTERNAL_SENS_CAMERA_INFO            01  /* Carmer's information */
#define XAP_EXTERNAL_SENS_CAMERA_ROI             02  /* Carmer region of interest information */
#define XAP_EXTERNAL_SENS_CAMERA_TARGET_INFO     03  /* Carmer recognize target information */

/* COMMON TIME消息ID */
#define XAP_EXTERNAL_COMMON_TIME_SYNC            01  /* Time sync message */

#pragma pack(push, 1)

/* General: Header */
typedef struct {
	uint8_t sync1;
	uint8_t sync2;
	uint8_t msgID;
	uint8_t msgClass;
	uint16_t length;
} xap_external_header_t;

/* General: Checksum & end */
typedef struct {
	uint16_t crc;
	uint8_t end;
} xap_external_crc_end_t;

/* HIL SBG消息 */
/* XAP_EXTERNAL_HIL_SBG_IMU_DATA (00 | 03)*/
typedef struct {
	uint32_t time_stamp; 		/* Time since sensor is powered up, µs */
	uint16_t imu_status; 		/* IMU Status bitmask */
	float accel_x; 				/* Filtered Accelerometer – X axis, m/s2 */
	float accel_y; 				/* Filtered Accelerometer – Y axis, m/s2 */
	float accel_z; 				/* Filtered Accelerometer – Z axis, m/s2 */
	float gyro_x; 				/* Filtered Gyroscope – X axis, rad/s */
	float gyro_y; 				/* Filtered Gyroscope – Y axis, rad/s */
	float gyro_z; 				/* Filtered Gyroscope – Z axis, rad/s */
	float temp;					/* Internal Temperature, oC */
	float delta_vel_x;			/* Sculling output - X axis, m/s2 */
	float delta_vel_y;			/* Sculling output - Y axis, m/s2 */
	float delta_vel_z;			/* Sculling output - Z axis, m/s2 */
	float delta_angle_x;		/* Coning output - X axis, rad/s */
	float delta_angle_y;		/* Coning output - Y axis, rad/s */
	float delta_angle_z;		/* Coning output - Z axis, rad/s */
} xap_external_hil_sbg_imu_data_t;

/* XAP_EXTERNAL_HIL_SBG_EKF_EULER (00 | 06) */
typedef struct {
	uint32_t time_stamp; 		/* Time since sensor is powered up, µs */
	float roll;					/* Roll angle, rad */
	float pitch;				/* Pitch angle, rad */
	float yaw;					/* Yaw angle (heading), rad */
	float roll_acc;				/* 1b Roll angle accuracy, rad */
	float pitch_acc;			/* 1b Pitch angle accuracy, rad */
	float yaw_acc;				/* 1b Yaw angle accuracy, rad */
	uint32_t solution_status;	/* Global solution status. See SOLUTION_STATUS definition for more details. */
} xap_external_hil_sbg_ekf_euler_t;

/* XAP_EXTERNAL_HIL_SBG_EKF_QUAT (00 | 07) */
typedef struct {
	uint32_t time_stamp; 		/* Time since sensor is powered up, µs */
	float q0;					/* First quaternion parameter (W) */
	float q1;					/* Second quaternion parameter (X) */
	float q2;					/* Third quaternion parameter (Y) */
	float q3;					/* Forth quaternion parameter (Z) */
	float roll_acc;				/* 1b Roll angle accuracy, rad */
	float pitch_acc;			/* 1b Pitch angle accuracy, rad */
	float yaw_acc;				/* 1b Yaw angle accuracy, rad */
	uint32_t solution_status;	/* Global solution status. See SOLUTION_STATUS definition for more details. */
} xap_external_hil_sbg_ekf_quat_t;

/* XAP_EXTERNAL_HIL_SBG_EKF_NAV (00 | 08) */
typedef struct {
	uint32_t time_stamp; 		/* Time since sensor is powered up, µs */
	float velpcity_n;			/* Velocity in North direction, m/s */
	float velpcity_e;			/* Velocity in East direction, m/s */
	float velpcity_d;			/* Velocity in Down direction, m/s */
	float velpcity_n_acc;		/* 1b Velocity in North direction accuracy, m/s */
	float velpcity_e_acc;		/* 1b Velocity in East direction accuracy, m/s */
	float velpcity_d_acc;		/* 1b Velocity in Down direction accuracy, m/s */
	double latitude;			/* Latitude, deg */
	double longitude;			/* longitude, deg */
	double altitude;			/* Altitude above Mean Sea Level, m */
	float undulation;			/* Altitude difference between the geoid and the Ellipsoid. (WGS-84 Altitude = MSL Altitude + undulation), m */
	float latitude_acc;			/* 1b Latitude accuracy, deg */
	float longitude_acc;		/* 1b Longitude accuracy, deg */
	float altitude_acc;			/* 1b Vertical Position accuracy, m */
	uint32_t solution_status;	/* Global solution status. See SOLUTION_STATUS definition for more details. */
} xap_external_hil_sbg_ekf_nav_t;

/* XAP_EXTERNAL_HIL_SBG_GPS1_VEL (00 | 13), XAP_EXTERNAL_HIL_SBG_GPS2_VEL (00 | 16) */
typedef struct {
	uint32_t time_stamp; 		/* Time since sensor is powered up, µs */
	uint32_t gps_vel_status;	/* GPS velocity fix and status bitmask */
	uint32_t gps_tow;			/* GPS Time of Week, ms */
	float vel_n;				/* Velocity in North direction, m/s */
	float vel_e;				/* Velocity in East direction, m/s */
	float vel_d;				/* Velocity in Down direction, m/s */
	float vel_acc_n;			/* 1b Accuracy in North direction, m/s */
	float vel_acc_e;			/* 1b Accuracy in East direction, m/s */
	float vel_acc_d;			/* 1b Accuracy in Down direction, m/s */
	float course;				/* True direction of motion over ground (0 to 360 deg), deg */
	float course_acc;			/* 1b course accuracy (0 to 360 deg), deg */
} xap_external_hil_sbg_gps_vel_t;

/* XAP_EXTERNAL_HIL_SBG_GPS1_POS (00 | 14) / XAP_EXTERNAL_HIL_SBG_GPS2_POS (00 | 17) */
typedef struct {
	uint32_t time_stamp; 		/* Time since sensor is powered up, µs */
	uint32_t gps_pos_status;	/* GPS velocity fix and status bitmask */
	uint32_t gps_tow;			/* GPS Time of Week, ms */
	double lat;					/* Latitude, positive North, deg */
	double longit;				/* Longitude, positive East, deg */
	double alt;					/* Altitude Above Mean Sea Level, m */
	float undulation;			/* Altitude difference between the geoid and the Ellipsoid. (WGS-84 Altitude = MSL Altitude + undulation), m */
	float pos_acc_lat;			/* 1b Latitude Accuracy, m */
	float pos_acc_longit;		/* 1b Longitude Accuracy, m */
	float pos_acc_alt;			/* 1b Altitude Accuracy, m */
	uint8_t num_sv_used;		/* Number of space vehicles used in GNSS solution */
	uint8_t base_station_id;	/* ID of the DGPS/RTK base station in use */
	uint16_t diff_age;			/* Differential data age */
} xap_external_hil_sbg_gps_pos_t;

/* XAP_EXTERNAL_HIL_SBG_GPS1_HDT (00 | 15) / XAP_EXTERNAL_HIL_SBG_GPS2_HDT (00 | 18) */
typedef struct {
	uint32_t time_stamp; 			/* Time since sensor is powered up, µs */
	uint16_t gps_hdt_status;		/* GPS True Heading status. */
	uint32_t gps_tow;				/* GPS Time of Week, ms */
	float gps_true_heading;			/* True heading angle (0 to 360°). deg */
	float gps_true_heading_acc;		/* 1b True heading estimated accuracy (0 to 360°). deg */
	float gps_pitch;				/* Pitch angle from the master to the rover, deg */
	float gps_pitch_acc;			/* 1b pitch estimated accuracy, deg */
} xap_external_hil_sbg_gps_hdt_t;

/* XAP_EXTERNAL_HIL_SBG_AIR_DATA (00 | 36) */
typedef struct {
	uint32_t time_stamp;	  /* Time since sensor is powered up, µs */
	uint16_t airdata_status;  /* Air data status */
	float pressureAbs;		  /* Pressure measured by the sensor, Pa */
	float altitude;			  /* Altitude computed from barometric almtimeter, m */
	float pressureDiff;		  /* Pressure diff, Pa */
	float trueAirspeed;		  /* True air speed, m */
	float airTemperature;	  /* Air temperature, K */
} xap_external_hil_sbg_air_data_t;

/* HIL PX4消息 */
/* XAP_EXTERNAL_HIL_PX4_GYRO (01 | 01) */
typedef struct {
	uint64_t time_stamp;  /* Time since sensor is powered up, µs */
	uint32_t device_id;   /* Unique device ID for the sensor */
	uint64_t integral_dt; /* Integration time, us */
	float x;              /* Filtered angular velocity in x axis, rad / s */
	float y;              /* Filtered angular velocity in y axis, rad / s */
	float z;              /* Filtered angular velocity in z axis, rad / s */
	float x_integral;     /* delta angle - x axis in the integration time, rad */
	float y_integral;     /* delta angle - y axis in the integration time, rad */
	float z_integral;     /* delta angle - z axis in the integration time, rad */
} xap_external_hil_px4_gyro_t;

/* XAP_EXTERNAL_HIL_PX4_ACCEL (01 | 02) */
typedef struct {
	uint64_t time_stamp;  /* Time since sensor is powered up, µs */
	uint32_t device_id;   /*unique device ID for the sensor */
	uint64_t integral_dt; /* integration time, us */
	float x;              /* Filtered acceleration in x axis, m/s/s */
	float y;              /* Filtered acceleration in y axis, m/s/s */
	float z;              /* Filtered acceleration in z axis, m/s/s */
	float x_integral;     /* velocity -x axis in the integration time, m/s */
	float y_integral;     /* velocity -y axis in the integration time, m/s */
	float z_integral;     /* velocity -z axis in the integration time, m/s */
} xap_external_hil_px4_accel_t;

/* XAP_EXTERNAL_HIL_PX4_MAG (01 | 03) */
typedef struct {
	uint64_t timestamp; /* time since system start (microseconds),  µs */
	uint32_t device_id; /* unique device ID for the sensor */
	float x;            /* magnetic field in x axis	Gauss */
	float y;            /* magnetic field in y axis	Gauss */
	float z;            /* magnetic field in z axis	Gauss */
} xap_external_hil_px4_mag_t;

/* XAP_EXTERNAL_HIL_PX4_BARO (01 | 04) */
typedef struct {
	uint64_t timestamp; /* time since system start (microseconds),  µs */
	uint32_t device_id; /* unique device ID for the sensor */
	float pressure;     /* measurement in millibar millibar */
	float temp;         /* Temperature in degrees celsius C */
} xap_external_hil_px4_baro_t;

/* XAP_EXTERNAL_HIL_PX4_GPS (01 | 05) */
typedef struct {
	uint64_t timestamp; 	  /* time since system start (microseconds),  µs */
	uint32_t device_id;       /* unique device ID for the sensor */
	int32_t lat;              /* Latitude in 1E-7 degrees 1E-7deg */
	int32_t lon;              /* Longitude in 1E-7 degrees 1E-7deg */
	int32_t alt;              /* Altitude in 1E-3 meters above MSL mm */
	float s_variance_m_s;     /* GPS speed accuracy estimate m / s */
	uint8_t fix_type;         /* GPS position fixing type */
	float eph;                /* GPS horizontal position accuracy m */
	float epv;                /*GPS vertical position accuracy m */
	float hdop;               /* Horizontal dilution of precision */
	float vdop;               /* Vertical dilution of precision */
	float vel_m_s;            /* GPS ground speed m / s */
	float vel_n_m_s;          /* GPS North velocity m / s */
	float vel_e_m_s;          /*GPS East velocity m / s */
	float vel_d_m_s;          /* GPS Down velocity m / s */
	float yaw;                /* dual antennas yaw rad */
	uint8_t heading_fix_type; /* dual antennas fixing type rad */
	uint8_t satellites_used;  /* Number of satellites used */
} xap_external_hil_px4_gps_t;

/* XAP_EXTERNAL_HIL_PX4_AIRSPEED (01 | 06) */
typedef struct {
	uint64_t timestamp; /* time since system start (microseconds),  µs */
	uint32_t device_id; /* unique device ID for the sensor */
	float tas;          /* true airspeed m/s */
	float cas;          /* calibrated airspee m/s */
	float temp;         /* air temperature in degrees celsius */
} xap_external_hil_px4_airspeed_t;

/* HIL XAP消息 */
/* XAP_EXTERNAL_HIL_XAP_GYRO (05 | 01) Length = 20 TransferTime = 0.315ms(B921600-8N1 Include header and tailer) */
typedef struct {
	uint32_t device_id;   /* Unique device ID for the sensor */
	float x;              /* Filtered angular velocity in x axis, rad / s */
	float y;              /* Filtered angular velocity in y axis, rad / s */
	float z;              /* Filtered angular velocity in z axis, rad / s */
	float temperature;  /* Temperature in degrees celsius C */
} xap_external_hil_xap_gyro_t;

/* XAP_EXTERNAL_HIL_XAP_ACCEL (05 | 02) Length = 20 TransferTime = 0.315ms(B921600-8N1 Include header and tailer) */
typedef struct {
	uint32_t device_id;   /*unique device ID for the sensor */
	float x;              /* Filtered acceleration in x axis, m/s/s */
	float y;              /* Filtered acceleration in y axis, m/s/s */
	float z;              /* Filtered acceleration in z axis, m/s/s */
	float temperature;  /* Temperature in degrees celsius C */
} xap_external_hil_xap_accel_t;

/* XAP_EXTERNAL_HIL_XAP_MAG (05 | 03) Length = 20 TransferTime = 0.315ms(B921600-8N1 Include header and tailer) */
typedef struct {
	uint32_t device_id; /* unique device ID for the sensor */
	float x;            /* magnetic field in x axis	Gauss */
	float y;            /* magnetic field in y axis	Gauss */
	float z;            /* magnetic field in z axis	Gauss */
	float temperature;  /* Temperature in degrees celsius C */
} xap_external_hil_xap_mag_t;

/* XAP_EXTERNAL_HIL_XAP_BARO (05 | 04) Length = 12 TransferTime = 0.228ms(B921600-8N1 Include header and tailer) */
typedef struct {
	uint32_t device_id; /* unique device ID for the sensor */
	float pressure;     /* measurement in millibar millibar */
	float temperature;  /* Temperature in degrees celsius C */
} xap_external_hil_xap_baro_t;

/* XAP_EXTERNAL_HIL_XAP_GPS (05 | 05) Length = 59 TransferTime = 0.738ms(B921600-8N1 Include header and tailer) */
typedef struct {
	uint32_t device_id;       /* unique device ID for the sensor */
	int32_t lat;              /* Latitude in 1E-7 degrees 1E-7deg */
	int32_t lon;              /* Longitude in 1E-7 degrees 1E-7deg */
	int32_t alt;              /* Altitude in 1E-3 meters above MSL mm */
	float s_variance_m_s;     /* GPS speed accuracy estimate m / s */
	uint8_t fix_type;         /* GPS position fixing type */
	float eph;                /* GPS horizontal position accuracy m */
	float epv;                /*GPS vertical position accuracy m */
	float hdop;               /* Horizontal dilution of precision */
	float vdop;               /* Vertical dilution of precision */
	float vel_m_s;            /* GPS ground speed m / s */
	float vel_n_m_s;          /* GPS North velocity m / s */
	float vel_e_m_s;          /*GPS East velocity m / s */
	float vel_d_m_s;          /* GPS Down velocity m / s */
	float yaw;                /* dual antennas yaw rad */
	uint8_t heading_fix_type; /* dual antennas fixing type rad */
	uint8_t satellites_used;  /* Number of satellites used */
} xap_external_hil_xap_gps_t;

/* XAP_EXTERNAL_HIL_XAP_AIRSPEED (05 | 06) */
typedef struct {
	uint32_t device_id; /* unique device ID for the sensor */
	float tas;          /* true airspeed m/s */
	float cas;          /* calibrated airspee m/s */
	float temp;         /* air temperature in degrees celsius */
} xap_external_hil_xap_airspeed_t;

/* FC STATUS消息 */
/* XAP_EXTERNAL_FC_STATUS_ATT (0A | 01) */
typedef struct {
	float phi;
	float theta;
	float psi;
} xap_external_fc_status_att_t;

/* FC CMD消息 */
/* XAP_EXTERNAL_FC_CMD_MPWM (0C | 01) */
typedef struct {
	float main_pwm1;
	float main_pwm2;
	float main_pwm3;
	float main_pwm4;
	float main_pwm5;
	float main_pwm6;
	float main_pwm7;
	float main_pwm8;
} xap_external_fc_cmd_mpwm_t;

/* XAP_EXTERNAL_FC_CMD_APWM (0C | 02) */
typedef struct {
	float aux_pwm1;
	float aux_pwm2;
	float aux_pwm3;
	float aux_pwm4;
	float aux_pwm5;
	float aux_pwm6;
	float aux_pwm7;
	float aux_pwm8;
} xap_external_fc_cmd_apwm_t;

/* MCP NAV消息 */
/* XAP_EXTERNAL_MCP_NAV_ARUCO (11 | 01) */
typedef struct {
	uint8_t markerID;       /* 二维码ID */
	float x;                /* 二维码相对坐标x */
	float y;                /* 二维码相对坐标y */
	float z;                /* 二维码相对坐标z */
	float q0;               /* 二维码相对姿态四元数q0 */
	float q1;               /* 二维码相对姿态四元数q1 */
	float q2;               /* 二维码相对姿态四元数q2 */
	float q3;               /* 二维码相对姿态四元数q3 */
} xap_external_mcp_nav_aruco_t;

/* SENS CAMERA消息 */
/* XAP_EXTERNAL_SENS_CAMERA_ROI (20 | 02) */
typedef struct {
	uint32_t x_offset; /*
                        * Leftmost pixel of the ROI
                        * (0 if the ROI includes the left edge of the image)
                        */

	uint32_t y_offset; /*
                        * Topmost pixel of the ROI
                        * (0 if the ROI includes the top edge of the image)
                        */

	uint32_t height;   // Height of ROI
	uint32_t width;    // Width of ROI

	/*
	 * True if a distinct rectified ROI should be calculated from the "raw"
	 * ROI in this message. Typically this should be False if the full image
	 * is captured (ROI not used), and True if a subwindow is captured (ROI
	 * used).
	*/
	uint8_t do_rectify;
} xap_external_sens_camera_roi_t;

/* 相机图像畸变模型 */
#define DISTORTION_MODEL_PLUMB_BOB              0
#define DISTORTION_MODEL_RATIONAL_POLYNOMIAL    1
#define DISTORTION_MODEL_EQUIDISTANT            2

/* XAP_EXTERNAL_SENS_CAMERA_INFO (20 | 01) */
typedef struct {
	uint32_t height;                        /* 相机像素高 */
	uint32_t width;                         /* 相机像素宽 */
	uint8_t distortion_model;               /* 相机图像畸变模型 DISTORTION_MODEL_** */
	double D[5];                            /* 相机图像畸变模型参数 */
	double K[9];                            /* 相机固有矩阵 */
	double R[9];                            /* 相机校正矩阵 */
	double P[9];                            /* 相机投影矩阵 */
	uint32_t binning_x;                     /* 分辨率缩放系数-X */
	uint32_t binning_y;                     /* 分辨率缩放系数-Y */
	xap_external_sens_camera_roi_t roi;     /* 感兴趣区域参数 */
} xap_external_sens_camera_info_t;

/* XAP_EXTERNAL_SENS_CAMERA_TARGET_INFO (20 | 03) */
typedef struct {
	uint8_t targetID;                      /* 目标ID */

	uint32_t x_offset;                      /* 目标最左侧像素位置 */
	uint32_t y_offset;                      /* 目标最上侧像素位置 */

	uint32_t height;                        /* 目标像素高 */
	uint32_t width;                         /* 目标像素宽 */

	uint8_t do_rectify;                        /* 位置信息基于原始图像尺寸 */
} xap_external_sens_camera_target_info_t;

/* XAP_EXTERNAL_COMMON_TIME_SYNC (F0 | 01) */
typedef struct {
	uint32_t ort_timestamp;                      /* 发起源时间戳 ms */
	uint32_t rec_timestamp;                      /* 接收时间戳 ms */
	uint32_t xmt_timestamp;                      /* 发送时间戳 ms */

} xap_external_common_time_sync_t;

#pragma pack(pop)

const static uint16_t X8408_INT_CRC = 0x0000;

/*!< CRC table used to compute a 16 bit CRC with the polynom 0x8408. */
const static uint16_t crc16LookupTable[256] = {
	0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF, 0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
	0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E, 0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
	0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD, 0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
	0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C, 0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
	0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB, 0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
	0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A, 0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
	0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9, 0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
	0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738, 0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
	0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7, 0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
	0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036, 0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
	0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5, 0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
	0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134, 0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
	0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3, 0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
	0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232, 0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
	0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1, 0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
	0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330, 0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78
};

static inline uint16_t xap_external_crc_table_accumulate_byte(uint8_t b, uint16_t crc)
{
	uint8_t index = (uint8_t)((b ^ crc) & 0xFF);
	return (uint16_t)(crc16LookupTable[index] ^ (crc >> 8));
}

static inline uint16_t xap_external_crc_table_accumulate_buffer(const uint8_t *buffer, uint16_t start, uint16_t length,
		uint16_t crc)
{
	uint16_t crcTmp = crc;

	for (int i = start; i < start + length; i++) {
		crcTmp = xap_external_crc_table_accumulate_byte(buffer[i], crcTmp);
	}

	return crcTmp;
}

static inline uint16_t xap_external_crc_table_calculate(uint8_t *buffer, uint16_t start, uint16_t length)
{
	uint16_t crcTmp = X8408_INT_CRC;

	for (int i = start; i < start + length; i++) {
		crcTmp = xap_external_crc_table_accumulate_byte(buffer[i], crcTmp);
	}

	return crcTmp;
}

/*
* Compute a CRC for a specified buffer.
* \param[in] pBuffer Read only buffer to compute the CRC on.
* \param[in] bufferSize Buffer size in bytes.
* \return The computed 16 bit CRC.
*/
static inline uint16_t xap_external_crc_bit_calculate_buffer(const void *pBuffer, uint16_t bufferSize)
{
	const uint8_t *pBytesArray = (const uint8_t *)pBuffer;
	uint16_t poly = 0x8408;
	uint16_t crc = 0;
	uint8_t carry;
	uint8_t i_bits;
	uint16_t j;

	for (j = 0; j < bufferSize; j++) {
		crc = crc ^ pBytesArray[j];

		for (i_bits = 0; i_bits < 8; i_bits++) {
			carry = crc & 1;
			crc = crc / 2;

			if (carry) {
				crc = crc ^ poly;
			}
		}
	}

	return crc;
}

static inline void xap_external_crc_bit_calculate_accumulate_buffer(const void *pBuffer, uint16_t bufferSize,
		uint16_t *crc)
{
	const uint8_t *pBytesArray = (const uint8_t *)pBuffer;
	uint16_t poly = 0x8408;
	uint8_t carry;
	uint8_t i_bits;
	uint16_t j;

	for (j = 0; j < bufferSize; j++) {
		*crc = *crc ^ pBytesArray[j];

		for (i_bits = 0; i_bits < 8; i_bits++) {
			carry = *crc & 1;
			*crc = *crc / 2;

			if (carry) {
				*crc = *crc ^ poly;
			}
		}
	}
}

static inline void xap_external_crc_bit_accumulate_byte(const uint8_t data, uint16_t *crc)
{
	uint16_t poly = 0x8408;
	uint8_t carry;
	uint8_t i_bits;
	*crc = *crc ^ data;

	for (i_bits = 0; i_bits < 8; i_bits++) {
		carry = *crc & 1;
		*crc = *crc / 2;

		if (carry) {
			*crc = *crc ^ poly;
		}
	}
}

static inline void xap_external_crc_init(uint16_t *crc_value)
{
	*crc_value = X8408_INT_CRC;
}

#endif /* __XAP_EXTERNAL_PROTOCOL__ */
