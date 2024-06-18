/*
  Author: Brett Lopez
  Contact: btlopez@ucla.edu
  Date: Oct 3, 2022
*/

#define HEADER 0xFF

#define PACKETID_ARM 0x01
#define PACKETID_CMD 0x02
#define PACKETID_MEAS 0x03
#define PACKETID_AHRS 0x04
#define PACKETID_IMU 0x05
#define PACKETID_GAIN 0x06
#define PACKETID_MTR 0x07
#define PACKETID_STATE 0x08
#define PACKETID_BARO 0x09

namespace vectr {

struct armPkt {
  uint8_t header;
  uint8_t id;
  char arm;
  char zero;
  uint8_t checksum;
} __attribute__((packed));

struct cmdPkt {
  uint8_t header;
  uint8_t id;
  float qw_d;
  float qx_d;
  float qy_d;
  float qz_d;
  float wx_d;
  float wy_d;
  float wz_d;
  float throttle;
  uint8_t att_status;
  uint8_t checksum;
} __attribute__((packed));

struct measPkt {
  uint8_t header;
  uint8_t id;
  float qw_m;
  float qx_m;
  float qy_m;
  float qz_m;
  float px_m;
  float py_m;
  float pz_m;
  uint8_t checksum;
} __attribute__((packed));

struct ahrsPkt {
  uint8_t header;
  uint8_t id;
  float qw;
  float qx;
  float qy;
  float qz;
  float wx;
  float wy;
  float wz;
  float bx;
  float by;
  float bz;
  uint8_t checksum;
} __attribute__((packed));

struct imuPkt {
  uint8_t header;
  uint8_t id;
  uint8_t seq;
  uint32_t timestamp;
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  uint8_t checksum;
} __attribute__((packed));

struct baroPkt {
  uint8_t header;
  uint8_t id;
  uint8_t seq;
  uint32_t timestamp;
  float temp;
  float pres;
  uint8_t checksum;
} __attribute__((packed));

struct gainPkt {
  uint8_t header;
  uint8_t id;
  float Kp_r;
  float Kp_p;
  float Kp_y;
  float Kd_r;
  float Kd_p;
  float Kd_y;
  float Ki_r;
  float Ki_p;
  float Ki_y;
  float Lq;
  float Lbg;
  float Lp;
  float Lv;
  float Lba;
  float acc_lpf;
  float gyro_lpf;
  uint8_t checksum;
} __attribute__((packed));

struct motorPkt {
  uint8_t header;
  uint8_t id;
  float m1;
  float m2;
  float m3;
  float m4;
  uint8_t checksum;
} __attribute__((packed));

struct statePkt {
  uint8_t header;
  uint8_t id;
  float qw;
  float qx;
  float qy;
  float qz;
  float wx;
  float wy;
  float wz;
  float bgx;
  float bgy;
  float bgz;
  float px;
  float py;
  float pz;
  float vx;
  float vy;
  float vz;
  float bax;
  float bay;
  float baz;
  uint8_t checksum;
} __attribute__((packed));

}; // namespace vectr
