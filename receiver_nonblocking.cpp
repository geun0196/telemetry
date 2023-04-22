#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <ctime>
#include <telemetry/common/mavlink.h>
#include <telemetry/mavlink_helpers.h>

int sender_serialSetting(int serial_port){
	termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error getting serial port attributes." << std::endl;
        close(serial_port);
        return 1;
    }
    
    cfsetospeed(&tty, B57600);
    cfsetispeed(&tty, B57600);
    
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting serial port attributes." << std::endl;
        close(serial_port);
        return 1;
    }
    return 0;
}

int receiver_serialSetting(int serial_port){   
    struct termios options = {0};
    if (tcgetattr(serial_port, &options) != 0) {
        std::cerr << "Error getting serial port attributes." << std::endl;
        close(serial_port);
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
    options.c_cc[VTIME] = 20;
    if (tcsetattr(serial_port, TCSANOW, &options) != 0) {
        std::cerr << "Error setting serial port attributes." << std::endl;
        close(serial_port);
        return 1;
    }
    return 0;
}

bool receiveMessage(int fd, mavlink_message_t& msg, mavlink_status_t& status, mavlink_channel_t channel, std::string& message) {
    uint8_t byte;
    bool message_received = false;
    
    while (1) { 
        fd_set set;
        FD_ZERO(&set);
        FD_SET(fd, &set);
        
        struct timeval timeout;
        timeout.tv_sec = 2; // wait 2sec for select function input
        timeout.tv_usec = 0;
        
        int rv = select(fd + 1, &set, NULL, NULL, &timeout);
        if (rv == -1) {
            std::cerr << "Error during select." << std::endl;
            break;
        } else if (rv == 0) {
            break;
        } else {
            read(fd, &byte, 1);
            if (mavlink_parse_char(channel, byte, &msg, &status)) {
                if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
                    mavlink_statustext_t text;
                    mavlink_msg_statustext_decode(&msg, &text);
                    message = std::string(reinterpret_cast<char*>(text.text));
                    message_received = true;
                    break;
                }
            }
        }
    }
    return message_received;
}

int main()
{
    int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (fd == -1) {
        std::cerr << "Error opening serial port." << std::endl;
        return 1;
    }
    sender_serialSetting(fd);
    receiver_serialSetting(fd);

    //Create a Mavlink instance
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_status_t status;
    mavlink_channel_t channel = MAVLINK_COMM_0;
    std::string message;
    
    //Wait for messages
    while (true) {
        printf("start while\n");
        
        bool message_received = receiveMessage(fd, msg, status, channel, message);
        
        if (message_received) {
            printf("%s\n",message.c_str());
        } else {
            message = "ERR";
            printf("%s\n",message.c_str());
            printf("timeout occurred\n");
        }
    }
    return 0;
}
