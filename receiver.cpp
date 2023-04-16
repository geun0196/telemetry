#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <telemetry/common/mavlink.h>
#include <telemetry/mavlink_helpers.h>

int main()
{
    int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (fd == -1) {
        std::cerr << "Error opening serial port." << std::endl;
        return 1;
    }

    struct termios options = {0};
    if (tcgetattr(fd, &options) != 0) {
        std::cerr << "Error getting serial port attributes." << std::endl;
        close(fd);
        return 1;
    }
    cfsetispeed(&options, B57600);
    cfsetospeed(&options, B57600);
    options.c_cflag = (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag = CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 50;
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        std::cerr << "Error setting serial port attributes." << std::endl;
        close(fd);
        return 1;
    }

    //Create a Mavlink instance
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_status_t status;
    mavlink_channel_t channel = MAVLINK_COMM_0;

    //Wait for messages
    while (true) {
        uint8_t byte;
        std::string message = "";
        while (read(fd, &byte, 1) == 1) {
            //Try to parse the message
            if (mavlink_parse_char(channel, byte, &msg, &status)) {           
                //If a message is received, check if it's a string message
                if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
                  mavlink_statustext_t text;
                  mavlink_msg_statustext_decode(&msg, &text);
                  message = std::string(reinterpret_cast<char*>(text.text));
                  std::cout << "Received id: " << text.id << std::endl;
                  std::cout << "Received: " << message << std::endl;
                  printf("=======================================================================\n");
                  break;
                }
            }
        }
    }

    //Close the serial port
    close(fd);
    return 0;
}

