#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>

#define DEVICE_PATH "/dev/adxl345-0"  // Update with the correct device path

int main() {
    int fd;
    short accel_data;

    // Open the character device file
    fd = open(DEVICE_PATH, O_RDONLY);
    if (fd == -1) {
        perror("Error opening device file");
        return EXIT_FAILURE;
    }

    // Read data from the X-axis
    ssize_t bytesRead = read(fd, &accel_data, sizeof(accel_data));
    if (bytesRead == -1) {
        perror("Error reading from device");
        close(fd);
        return EXIT_FAILURE;
    }

    // Display the read accelerometer data
    printf("X-axis Accelerometer Data: %hx\n", accel_data);

    // Close the device file
    close(fd);

    return EXIT_SUCCESS;
}