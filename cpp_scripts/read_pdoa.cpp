#include <iostream>
#include <cstring>
#include <string>
//linux serial port headers
#include <termios.h> //Terminal Control
#include <fcntl.h> //File Control (open function)
#include <errno.h> //Error functions
#include <unistd.h> //read, write, close functions

//define a struct to store the important info from the UWB node
typedef struct PDoA_info {
    int D, P, Xcm, Ycm;
} PDoA_info;

//this function reads the JSON frame (in string form) and writes the important info (D,P,Xcm,Ycm) into the PDoA_info struct
void readPDoAFrame(char* read_buf, PDoA_info* output_info, char* D_field, char* P_field, char* Xcm_field, char* Ycm_field) {//pass in all these to avoid re-declaring
    char* p; //pointer for general use
    p = strstr(read_buf, D_field) + 4; //find first occurence of "D", +4 to get to the start of the value, store pointer to this char in p
    output_info->D = atoi(p); //convert the remaining array (substring) pointed to by p to an integer, giving the D value
    p = strstr(p, P_field) + 4; //continue from the previous pointer, find first occurence of "P" in the remaining substring pointed to by p
    output_info->P = atoi(p);
    p = strstr(p, Xcm_field) + 6;
    output_info->Xcm = atoi(p);
    p = strstr(p, Ycm_field) + 6;
    output_info->Ycm = atoi(p);
    return;
}

int main() {
    std::cout << "Hello world!" << std::endl;
    int serial_port = open("/dev/ttyACM0", O_RDWR); //returns the file descriptor (usually 3), set O_RDWR for both read and write, O_RDONLY for read only access
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
    char read_buf[256];
    int n = 0;
    n = read(serial_port, &read_buf, sizeof(read_buf)); //read the first (usually bugged) line to discard
    char D_field[] = "\"D\"", P_field[] = "\"P\"", Xcm_field[] = "\"Xcm\"", Ycm_field[] = "\"Ycm\""; //store all these strings to be searched for later
    PDoA_info pdoa_Info; //declare a struct to store the PDoA information
    while (n = read(serial_port, &read_buf, sizeof(read_buf))) {
        if (n == 1) continue; //if it is the second of a double newline, continue
        read_buf[n-1] = '\0'; //replace the final character, newline, with null
        readPDoAFrame(read_buf, &pdoa_Info, D_field, P_field, Xcm_field, Ycm_field);
        std::cout << "PDoA distance:" << pdoa_Info.D << ", phase:" << pdoa_Info.P << ", Xcm:" << pdoa_Info.Xcm << ", Ycm:" << pdoa_Info.Ycm << std::endl;
        std::cout << std::string(read_buf) << std::endl; //convert it into a cpp string (ends at the null)
    }
    close(serial_port);
    return 0;
}

