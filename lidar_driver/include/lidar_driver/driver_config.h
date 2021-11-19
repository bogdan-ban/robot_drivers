// Do NOT change these paramters, they are used to communicate with the lidar
#define COMMAND_START_BYTE 0xA5
#define COMMAND_START_SCAN_BYTE 0x60
#define COMMAND_STOP_SCAN_BYTE 0x65
#define COMMAND_RESET_BYTE 0x80

// Highly sensitive paramters, change with care
#define ANGLE_INCREMENT 0.615
#define ANGLE_INCREMENT_TOLERANCE 0.05
#define MAX_FRAME_LENGTH 90

// Can be tuned
#define TOPIC_BUFFER_SIZE 100
#define PI 3.1415
