#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>

#define ADXL345_I2C_ADDRESS 0x53

// ADXL345 register addresses
#define ADXL345_BW_RATE_REG 0x2C
#define ADXL345_INT_ENABLE_REG 0x2E
#define ADXL345_DATA_FORMAT_REG 0x31
#define ADXL345_FIFO_CTL_REG 0x38
#define ADXL345_POWER_CTL_REG 0x2D

// ADXL345 register values for configuration
#define ADXL345_OUTPUT_DATA_RATE_100HZ 0x0A
#define ADXL345_INT_DISABLE_ALL 0x00
#define ADXL345_DATA_FORMAT_DEFAULT 0x00
#define ADXL345_FIFO_BYPASS_MODE 0x00


#define ADXL345_MEASURE_MODE 0x08
#define ADXL345_STANDBY_MODE 0x00

// Data
#define ADXL345_OFSX       0x1E
#define ADXL345_OFSY       0x1F
#define ADXL345_OFSZ       0x20

#define ADXL345_DATAX0     0x32
#define ADXL345_DATAX1     0x33
#define ADXL345_DATAY0     0x34
#define ADXL345_DATAY1     0x35
#define ADXL345_DATAZ0     0x36
#define ADXL345_DATAZ1     0x37

#define ADXL345_MAX_X      4095
#define ADXL345_MAX_Y      4095
#define ADXL345_MAX_Z      4095

// Custom IOCTL commands
#define ADXL_IOCTL_SET_AXIS_X _IO('X', 0)
#define ADXL_IOCTL_SET_AXIS_Y _IO('Y', 1)
#define ADXL_IOCTL_SET_AXIS_Z _IO('Z', 2)

static int current_axis = ADXL_IOCTL_SET_AXIS_X;  // Default axis is X

static long adxl345_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    switch (cmd) {
        case ADXL_IOCTL_SET_AXIS_X:
        case ADXL_IOCTL_SET_AXIS_Y:
        case ADXL_IOCTL_SET_AXIS_Z:
            current_axis = cmd;
            return 0;

        default:
            return -ENOTTY;  // Not a valid ioctl command
    }
}

struct adxl345_device {
    struct miscdevice misc_device;
};

static int accelerometer_count = 0; // Global variable to count the number of accelerometers

// Function to read data from the accelerometer
static ssize_t adxl345_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    struct adxl345_device *adxl_dev;
    struct i2c_client *client;
    u8 reg_addr[2];
    u8 reg_data[2];
    s16 accel_data;  // Assuming accelerometer data is signed 16 bits
    ssize_t ret;

    // // Retrieve the instance of the struct adxl345_device
    // adxl_dev = container_of(file->private_data, struct adxl345_device, misc_device);

    // // Retrieve the instance of the struct i2c_client
    // client = to_i2c_client(adxl_dev->misc_device.parent);

    // // Read accelerometer data from the appropriate registers
    // reg_data[0] = current_axis;  // Assuming sequential registers for X, Y, Z data
    // i2c_master_send(client, reg_data, 1);
    // i2c_master_recv(client, &reg_data[1], 1);

    // // Combine the high and low bytes into a signed 16-bit value
    // accel_data = (s16)((reg_data[1] << 8) | reg_data[0]);

    // // Pass all or part of this sample to the application
    // if (count >= sizeof(accel_data)) {
    //     // The application requests more bytes than the size of our sample
    //     count = sizeof(accel_data);
    // }

    // if (copy_to_user(buf, &accel_data, count)) {
    //     pr_err("Failed to copy data to user\n");
    //     ret = -EFAULT;
    // } else {
    //     ret = count;
    // }


    // Retrieve the instance of the struct adxl345_device
    adxl_dev = container_of(file->private_data, struct adxl345_device, misc_device);

    // Retrieve the instance of the struct i2c_client
    client = to_i2c_client(adxl_dev->misc_device.parent);

    // Set register based on the current axis
    switch (current_axis) {
        case ADXL_IOCTL_SET_AXIS_X:
            reg_addr[0] = ADXL345_DATAX0;
            reg_addr[1] = ADXL345_DATAX1;
            break;
        case ADXL_IOCTL_SET_AXIS_Y:
            reg_addr[0] = ADXL345_DATAY0;
            reg_addr[1] = ADXL345_DATAY1;
            break;
        case ADXL_IOCTL_SET_AXIS_Z:
            reg_addr[0] = ADXL345_DATAZ0;
            reg_addr[1] = ADXL345_DATAZ1;
            break;
        default:
            return -EINVAL;  // Invalid argument
    }

    i2c_master_send(client, &reg_addr[1], 1);
    i2c_master_recv(client, &reg_data[1], 1);

    i2c_master_send(client, &reg_addr[0], 1);
    i2c_master_recv(client, &reg_data[0], 1);

    accel_data = (s16)((reg_data[1] << 8) | reg_data[0]);

    if (count >= sizeof(accel_data)) {
        count = sizeof(accel_data);
    }

    if (copy_to_user(buf, &accel_data, count)) {
        pr_err("Failed to copy data to user\n");
        ret = -EFAULT;
    } else {
        ret = count;
    }

    return ret;
}

// Define file operations structure with read callback
static const struct file_operations adxl345_fops = {
    .owner = THIS_MODULE,
    .read = adxl345_read,
    .unlocked_ioctl = adxl345_ioctl,
};

static int adxl345_probe(struct i2c_client *client)
{
    struct i2c_adapter *adapter = client->adapter;
    struct i2c_msg msgs[2];
    u8 reg_data[2];
    u8 reg = 0x00;  // DEVID register address
    u8 data;
    int ret;

    // Print a message indicating that the probe function is running
    pr_info("ADXL345 probe function called\n");


    // Set the BW_RATE register for Output Data Rate to 100 Hz
    reg_data[0] = ADXL345_BW_RATE_REG;
    reg_data[1] = ADXL345_OUTPUT_DATA_RATE_100HZ;
    i2c_master_send(client, reg_data, 2);

    // Disable all interrupts (INT_ENABLE register)
    reg_data[0] = ADXL345_INT_ENABLE_REG;
    reg_data[1] = ADXL345_INT_DISABLE_ALL; // All interrupts disabled
    i2c_master_send(client, reg_data, 2);

    // Set default data format (DATA_FORMAT register)
    reg_data[0] = ADXL345_DATA_FORMAT_REG;
    reg_data[1] = ADXL345_DATA_FORMAT_DEFAULT; // Default data format
    i2c_master_send(client, reg_data, 2);

    // Set FIFO Bypass mode (FIFO_CTL register)
    reg_data[0] = ADXL345_FIFO_CTL_REG;
    reg_data[1] = ADXL345_FIFO_BYPASS_MODE;
    i2c_master_send(client, reg_data, 2);

    // Activate Measurement mode (POWER_CTL register)
    reg_data[0] = ADXL345_POWER_CTL_REG;
    reg_data[1] = ADXL345_MEASURE_MODE;
    i2c_master_send(client, reg_data, 2);


    // Step 1: Send a write command to the device to set the register pointer
    msgs[0].addr = ADXL345_I2C_ADDRESS;
    msgs[0].flags = 0; // Write operation
    msgs[0].buf = &reg;
    msgs[0].len = sizeof(reg);

    // Step 2: Read the content of the DEVID register
    msgs[1].addr = ADXL345_I2C_ADDRESS;
    msgs[1].flags = I2C_M_RD; // Read operation
    msgs[1].buf = &data;
    msgs[1].len = sizeof(data);

    ret = i2c_transfer(adapter, msgs, 2);
    if (ret != 2) {
        pr_err("I2C transfer failed with error: %d\n", ret);
        return ret;
    }

    // Print the content of the DEVID register
    pr_info("DEVID register value: 0x%02x\n", data);

    // Dynamically allocate memory for an instance of the struct adxl345_device
    struct adxl345_device *adxl_dev = kzalloc(sizeof(struct adxl345_device), GFP_KERNEL);
    if (!adxl_dev) {
        pr_err("Failed to allocate memory for adxl345_device\n");
        return -ENOMEM;
    }

    // Associate this instance with the struct i2c_client
    adxl_dev->misc_device.parent = &client->dev;

    // Generate a unique name for the accelerometer
    char dev_name[20];
    snprintf(dev_name, sizeof(dev_name), "adxl345-%d", accelerometer_count);

    // Fill the content of the struct miscdevice structure
    adxl_dev->misc_device.minor = MISC_DYNAMIC_MINOR;
    adxl_dev->misc_device.name = dev_name;
    adxl_dev->misc_device.fops = &adxl345_fops;

    // Register with the misc framework
    ret = misc_register(&adxl_dev->misc_device);
    if (ret) {
        pr_err("Failed to register misc device: %d\n", ret);
        kfree(adxl_dev);
        return ret;
    }

    // Set the client data
    i2c_set_clientdata(client, adxl_dev);
    
    // Increment the accelerometer count
    accelerometer_count++;

    return 0;
}

static void adxl345_remove(struct i2c_client *client)
{
    u8 reg_data[2];
    int ret;

    pr_info("Entering adxl345_remove\n");

    // Switch off the accelerometer in the remove function
    reg_data[0] = ADXL345_POWER_CTL_REG;
    reg_data[1] = ADXL345_STANDBY_MODE;

    ret = i2c_master_send(client, reg_data, 2);

    if (ret != 1) {
        pr_err("Failed to switch off accelerometer: %d\n", ret);
    }

    pr_info("Accelerometer switched off\n");

    // Retrieve the instance of the struct adxl345_device
    struct adxl345_device *adxl_dev = i2c_get_clientdata(client);
    if (!adxl_dev) {
        pr_err("Failed to get client data\n");
        return;
    }

    pr_info("Before misc_deregister\n");
    // Unregister from the misc framework
    misc_deregister(&adxl_dev->misc_device);

    pr_info("Before kfree\n");
    // Free the memory allocated for adxl345_device
    kfree(adxl_dev);

    // Decrement the accelerometer count
    accelerometer_count--;

    pr_info("Exiting adxl345_remove\n");
}
/* The following list allows the association between a device and its driver
driver in the case of a static initialization without using
device tree.
Each entry contains a string used to make the association
association and an integer that can be used by the driver to
driver to perform different treatments depending on the physical
the physical device detected (case of a driver that can manage
different device models).*/
static struct i2c_device_id adxl345_idtable[] = {
{ "adxl345", 0 },
{ }
};
MODULE_DEVICE_TABLE(i2c, adxl345_idtable);
#ifdef CONFIG_OF
/* If device tree support is available, the following list
allows to make the association using the device tree.
Each entry contains a structure of type of_device_id. The field
compatible field is a string that is used to make the association
with the compatible fields in the device tree. The data field is
a void* pointer that can be used by the driver to perform different
perform different treatments depending on the physical device detected.
device detected.*/
static const struct of_device_id adxl345_of_match[] = {
{ .compatible = "qemu,adxl345",
.data = NULL },
{}
};

MODULE_DEVICE_TABLE(of, adxl345_of_match);
#endif
static struct i2c_driver adxl345_driver = {
.driver = {
/* The name field must correspond to the name of the module
and must not contain spaces. */
.name = "adxl345",
.of_match_table = of_match_ptr(adxl345_of_match),
},
.id_table = adxl345_idtable,
.probe = adxl345_probe,
.remove = adxl345_remove,
};
module_i2c_driver(adxl345_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("adxl345 driver");
MODULE_AUTHOR("Van-Chien NGUYEN");

 