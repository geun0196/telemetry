#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <telemetry/common/mavlink.h>
#include <telemetry/mavlink_helpers.h>

int serialSetting(int serial_port){
	termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error getting serial port attributes." << std::endl;
        close(serial_port);
        return 1;
    }
    cfsetospeed(&tty, B57600);
    cfsetispeed(&tty, B57600);
    tty.c_cflag &= ~PARENB; // no parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8; // 8 data bits
    tty.c_cflag &= ~CRTSCTS; // no hardware flow control
    tty.c_cc[VMIN] = 1; // read() blocks until at least 1 byte is available
    tty.c_cc[VTIME] = 0; // read() blocks indefinitely
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting serial port attributes." << std::endl;
        close(serial_port);
        return 1;
    }
    return 0;
}

int main()
{
    int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (fd == -1) {
        std::cerr << "Error opening serial port." << std::endl;
        return 1;
    }
    
    serialSetting(fd);

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

