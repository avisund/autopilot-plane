// baudrate = 9600
// port = /dev/cu.usbmoden1431

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

int main() {
    std::system("/bin/stty -f /dev/cu.wchusbserial1430 sane raw pass8 -echo -hupcl clocal 9600");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "opened stty" << std::endl;
    std::fstream file("/dev/cu.wchusbserial1430");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "opened fstream" << std::endl;
    std::string response;
    char buf[64];  // set to the size of one packet
    file.rdbuf()->pubsetbuf(buf, 16);

    while (file.is_open()) {
        getline(file, response);
        std::cout << "got: " << response << std::endl;

        char c[] = "ab";
        file.write(c, sizeof(c));

        // if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() % 10 == 0) {
        // std::cout << "writing abcd\\n" << std::endl;

        // }
    }
    file.close();
    return 0;
}