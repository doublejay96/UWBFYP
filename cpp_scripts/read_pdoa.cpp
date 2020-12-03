#include <iostream>
#include <string>
//linux serial port headers
#include <termios.h> //Terminal Control
#include <fcntl.h> //File Control (open function)
#include <errno.h> //Error functions
#include <unistd.h> //read, write, close functions

int main() {
    std::cout << "Hello world!" << std::endl;
    int serial_port = open("/dev/ttyACM0", O_RDWR); //O_RDWR for both read and write, O_RDONLY for read only access
    if (serial_port < 0) {
        std::cout << "Error opening serial port, returned " << serial_port << std::endl;
    } else {
        std::cout << "Serial port opened successfully" << std::endl;
    }
    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {//get parameters associated with the object in serial_port, stores in termios struct, returns 0 on success
        std::cout << "Error reading atributes from file descriptor to termios struct" << std::endl;
    }
    tty.c_lflag &= ICANON; //set canonical mode, input becomes available line-by-line. Default should be canonical alr, but just make sure.
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {//get parameters associated with the object in serial_port, stores in termios struct, returns 0 on success
        std::cout << "Error setting attributes from file descriptor to termios struct" << std::endl;
    }
    char read_buf [256];
    int n = 0;
    while (n = read(serial_port, &read_buf, sizeof(read_buf))) {
        read_buf[n-1] = '\0'; //replace the final character, newline, with null
        std::cout << std::string(read_buf) << std::endl; //convert it into a cpp string (ends at the null)
    }
    close(serial_port);
    return 0;
}
