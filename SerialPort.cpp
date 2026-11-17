#include "SerialPort.h"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cerrno>
#include <chrono>
#include <poll.h>

SerialPort::~SerialPort() {
    if(fileDescriptor >= 0) {
        close(fileDescriptor);
    }
}

bool SerialPort::openPort(const std::string& devicePath) {
    if (fileDescriptor >= 0) {
        return false;
    }

    fileDescriptor = open(
        devicePath.c_str(),
        O_RDWR | O_NOCTTY | O_NONBLOCK
    );

    if (fileDescriptor < 0) {
        return false;
    }

    termios settings{};

    if (tcgetattr(fileDescriptor, &settings) != 0) {
        close(fileDescriptor);
        fileDescriptor = -1;
        return false;
    }

    cfmakeraw(&settings);

    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= ~(CRTSCTS | CSTOPB);

    if (cfsetispeed(&settings, B115200) != 0 ||
        cfsetospeed(&settings, B115200) != 0 ||
        tcsetattr(fileDescriptor, TCSANOW, &settings) != 0) {
        close(fileDescriptor);
        fileDescriptor = -1;
        return false;
    }

    return true;
}

bool SerialPort::readLine(std::string& line) {
    if (fileDescriptor < 0) {
        return false;
    }

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);

    while (true) {
        const auto remaining =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()
            ).count();

        if (remaining <= 0) {
            return false;
        }

        pollfd device{};
        device.fd = fileDescriptor;
        device.events = POLLIN;

        int ready = poll(&device, 1, static_cast<int>(remaining));

        if (ready < 0 && errno == EINTR) {
            continue;
        }

        if (ready <= 0 ||
            (device.revents & (POLLERR | POLLHUP | POLLNVAL))) {
            return false;
        }

        char character{};
        ssize_t received = read(fileDescriptor, &character, 1);

        if (received < 0 &&
            (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            continue;
        }

        if (received <= 0) {
            return false;
        }

        if (character == '\n') {
            if(discardingLine) {
                line.clear();
                discardingLine = false;
            }
            else {
                line = pendingLine;
            }

            pendingLine.clear();
            return true;
        }

        if(discardingLine) {
            continue;
        }

        constexpr std::size_t maxLineLenght = 256;

        if(pendingLine.size() >= maxLineLenght) {
            pendingLine.clear();
            discardingLine = true;
            continue;
        }

        pendingLine += character;
    }
}