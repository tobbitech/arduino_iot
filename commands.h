#ifndef COMMANDS_H
#define COMMANDS_H

#define REBOOT 1
#define STATUS_REPORT 2
#define SET_STATUS_INTERVAL 3 // argument: "interval_s" <- seconds between each slowloop
#define ENABLE_OTA 5
#define GET_IP 6
#define GET_MAC 7
#define SHOW_NVS_LOG 8
#define SET_TELEMETRY_INTERVAL 9
#define SCAN_I2C_BUS 14
#define INIT_I2C 15

// Non default commands
#define GET_HAN_MESSAGE 100
#define SET_DEEPSLEEP_TIME 200


// NVS commands, not implemented
#define NVS_READ 530
#define NVS_WRITE 531
#define NVS_INIT_ERASE 533
#define NVS_WRITE_CLIENT_NAME 534
#define NVS_WRITE_ROOTCA 535
#define NVS_WRITE_CERT 536
#define NVS_WRITE_KEY 537
#define NVS_READ_CONFIG 538
#define NVS_READ_CA 539
#define NVS_READ_CERT 540
#define NVS_READ_KEY 541

#endif