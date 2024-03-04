// #include <stdio.h>
// #include <fcntl.h>
// #include <unistd.h>
// #include <sys/ioctl.h>
// #include <linux/i2c-dev.h>

// #define ADXL345_IOCTL_START_MEASUREMENT _IOW('A', 1, char)
// #define ADXL345_IOCTL_STOP_MEASUREMENT _IOW('B', 2, char)

// int main() {
//     int fd = open("/dev/adxl345-0", O_RDWR);
//     if (fd < 0) {
//         perror("Failed to open the device");
//         return 1;
//     }

//     // Start measurement
//     if (ioctl(fd, ADXL345_IOCTL_START_MEASUREMENT, NULL) < 0) {
//         perror("Failed to start measurement");
//         close(fd);
//         return 1;
//     }

//     printf("Measurement mode started\n");

//     // Sleep for a while to allow data to be collected
//     sleep(5);

//     // Read data from the driver
//     struct {
//         short x;
//         short y;
//         short z;
//     } data;

//     ssize_t bytesRead = read(fd, &data, sizeof(data));
//     if (bytesRead < 0) {
//         perror("Failed to read data from the device");
//         close(fd);
//         return 1;
//     }

//     printf("Accelerometer Data: X=%d, Y=%d, Z=%d\n", data.x, data.y, data.z);

//     // Stop measurement
//     if (ioctl(fd, ADXL345_IOCTL_STOP_MEASUREMENT, NULL) < 0) {
//         perror("Failed to stop measurement");
//         close(fd);
//         return 1;
//     }

//     printf("Measurement mode stopped\n");

//     close(fd);
//     return 0;
// }


#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>

int main() {
    int fd = open("/dev/adxl345-0", O_RDWR);
    if (fd < 0) {
        perror("Failed to open the device");
        return 1;
    }

    // Read data from the driver
    struct {
        uint8_t data[6];
    } data;

    ssize_t bytesRead = read(fd, &data, sizeof(data));
    if (bytesRead < 0) {
        perror("Failed to read data from the device");
        close(fd);
        return 1;
    }

    printf("Accelerometer Data: X=%d, Y=%d, Z=%d\n", data.data[0] | (data.data[1] << 8), data.data[2] | (data.data[3] << 8), data.data[4] | (data.data[5] << 8));

    close(fd);
    return 0;
}

