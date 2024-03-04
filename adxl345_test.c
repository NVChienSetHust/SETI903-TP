#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define ADXL_IOCTL_SET_AXIS_X _IO('X', 0)
#define ADXL_IOCTL_SET_AXIS_Y _IO('Y', 1)
#define ADXL_IOCTL_SET_AXIS_Z _IO('Z', 2)

#define DEVICE_PATH "/dev/adxl345-0"

int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <axis>\n", argv[0]);
        fprintf(stderr, "  axis: X, Y, or Z\n");
        return EXIT_FAILURE;
    }

    int fd = open(DEVICE_PATH, O_RDWR);
    if (fd == -1) {
        perror("Failed to open the device");
        return EXIT_FAILURE;
    }

    char axis = argv[1][0];

    switch (axis) {
        case 'X':
            ioctl(fd, ADXL_IOCTL_SET_AXIS_X, 0);
            break;
        case 'Y':
            ioctl(fd, ADXL_IOCTL_SET_AXIS_Y, 1);
            break;
        case 'Z':
            ioctl(fd, ADXL_IOCTL_SET_AXIS_Z, 2);
            break;
        default:
            fprintf(stderr, "Invalid axis. Use X, Y, or Z\n");
            close(fd);
            return EXIT_FAILURE;
    }

    // Read data from the accelerometer
    short accel_data;
    ssize_t ret = read(fd, &accel_data, sizeof(accel_data));

    if (ret == -1) {
        perror("Error reading from device file");
        close(fd);
        return EXIT_FAILURE;
    }

    printf("Accelerometer Data (Axis %c): %hx\n", axis, accel_data);

    close(fd);
    return 0;
}