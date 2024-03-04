#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/kfifo.h>
#include <linux/delay.h>

#include <linux/fs.h>

#include <linux/mutex.h>

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
#define ADXL345_FIFO_STREAM_MODE 0x80
#define ADXL345_FIFO_WATERMARK_LEVEL 0x14 // Set the watermark level to 20 samples
#define ADXL345_FIFO_WATERMARK_INT 0x02


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

#define DRIVER_FIFO_SIZE 64

#define ADXL345_FIFO_STATUS_REG 0x39
#define ADXL345_FIFO_STATUS_ENTRIES 0x1F


// Define a structure to represent a sample
struct fifo_element {
    u8 data[6]; // Array of 6 bytes for X, Y, Z data
};

struct adxl345_device {
    struct miscdevice misc_device;
    DECLARE_KFIFO(driver_fifo, struct fifo_element, DRIVER_FIFO_SIZE);  // Driver's FIFO
    wait_queue_head_t fifo_queue;  // Wait queue for processes waiting for data
    struct mutex lock;  // Mutex for protecting critical sections
    spinlock_t irq_lock;
};

static int accelerometer_count = 0; // Global variable to count the number of accelerometers


// Function to read data from the accelerometer
static ssize_t adxl345_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    struct adxl345_device *adxl_dev;
    struct i2c_client *client;
    ssize_t ret;
    
    // Retrieve the instance of the struct adxl345_device
    adxl_dev = container_of(file->private_data, struct adxl345_device, misc_device);

    // Retrieve the instance of the struct i2c_client
    client = to_i2c_client(adxl_dev->misc_device.parent);

    // Check if data is available in the FIFO
    if (kfifo_is_empty(&adxl_dev->driver_fifo)) {
        // Put the process to sleep until data is available
        wait_event_interruptible(adxl_dev->fifo_queue, !kfifo_is_empty(&adxl_dev->driver_fifo));
    }

    struct fifo_element sample;

    
    // Retrieve data from the FIFO
    // if (kfifo_get(&adxl_dev->driver_fifo, &sample) != sizeof(struct fifo_element)) {
    //     pr_err("Failed to get sample from fifo\n");
    //     return -EIO;
    // }

    mutex_lock(&adxl_dev->lock);
    pr_info("mutex lock before getting fifo data\n");
    // Retrieve data from the FIFO
    ret = kfifo_get(&adxl_dev->driver_fifo, &sample);

    mutex_unlock(&adxl_dev->lock);

    pr_info("mutex unlock after getting fifo data\n");

    pr_info("Number of fifo bytes obtained from device fifo: %d", ret);
    if (count >= sizeof(struct fifo_element)) {
        count = sizeof(struct fifo_element);
    }

    if (copy_to_user(buf, &sample, count)) {
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
};

// Function to flush accelerometer's FIFO and store data in driver's FIFO
static void adxl345_flush_fifo(struct adxl345_device *adxl_dev, int num_samples)
{
    struct i2c_client *client = to_i2c_client(adxl_dev->misc_device.parent);
    struct fifo_element sample;

    while (num_samples > 0) {
        u8 buffer[6];  // Temporary buffer to hold 6 bytes of data (2 bytes for each axis)
        u8 reg_addr[2];

        // Read X-axis data (2 bytes)
        reg_addr[0] = ADXL345_DATAX0;
        reg_addr[1] = ADXL345_DATAX1;
        if (i2c_master_send(client, &reg_addr[0], 1) != 1 ||
            i2c_master_recv(client, &buffer[0], 1) != 1 ||
            i2c_master_send(client, &reg_addr[1], 1) != 1 ||
            i2c_master_recv(client, &buffer[1], 1) != 1) {
            pr_err("Failed to read X-axis data\n");
        }

        // Read Y-axis data (2 bytes)
        reg_addr[0] = ADXL345_DATAY0;
        reg_addr[1] = ADXL345_DATAY1;
        if (i2c_master_send(client, &reg_addr[0], 1) != 1 ||
            i2c_master_recv(client, &buffer[2], 1) != 1 ||
            i2c_master_send(client, &reg_addr[1], 1) != 1 ||
            i2c_master_recv(client, &buffer[3], 1) != 1) {
            pr_err("Failed to read Y-axis data\n");
        }

        // Read Z-axis data (2 bytes)
        reg_addr[0] = ADXL345_DATAZ0;
        reg_addr[1] = ADXL345_DATAZ1;
        if (i2c_master_send(client, &reg_addr[0], 1) != 1 ||
            i2c_master_recv(client, &buffer[4], 1) != 1 ||
            i2c_master_send(client, &reg_addr[1], 1) != 1 ||
            i2c_master_recv(client, &buffer[5], 1) != 1) {
            pr_err("Failed to read Z-axis data\n");
        }

        // Copy the 6 bytes of data to the sample array
        memcpy(sample.data, buffer, sizeof(sample.data));

        // Store the sample in internal FIFO
        kfifo_put(&adxl_dev->driver_fifo, sample);

        num_samples--;
    }

    pr_info("adxl345_flush_fifo: Data added to FIFO\n");

    // Wake up any processes waiting for data
    wake_up_interruptible(&adxl_dev->fifo_queue);
}

// Implement the bottom half function for the interrupt
static irqreturn_t adxl345_int(int irq, void *dev_id)
{
    struct adxl345_device *adxl_dev = dev_id;
    struct i2c_client *client = to_i2c_client(adxl_dev->misc_device.parent);

    pr_info("adxl345_int: Interrupt received\n");

    u8 reg_addr[2];
    u8 reg_data;
    
    // Create a buffer with the register address
    reg_addr[0] = ADXL345_FIFO_STATUS_REG;

    // Retrieve the number of samples available in the accelerometer FIFO
    if (i2c_master_send(client, &reg_addr[0], 1) != 1) {
        pr_err("Failed to send FIFO status register request\n");
        return IRQ_HANDLED;
    }

    if (i2c_master_recv(client, &reg_data, 1) != 1) {
        pr_err("Failed to receive FIFO status register data\n");
        return IRQ_HANDLED;
    }

    int num_samples = reg_data & ADXL345_FIFO_STATUS_ENTRIES;
    pr_info("Number of samples available in FIFO: %d\n", num_samples);

    spin_lock(&adxl_dev->irq_lock);
    pr_info("spin lock while tranfering fifo data\n");
    // Call the flush function to handle data transfer
    adxl345_flush_fifo(adxl_dev, num_samples);

    spin_unlock(&adxl_dev->irq_lock);

    pr_info("spin unlock after tranfering fifo data\n");

    pr_info("adxl345_int: Process waking up\n");

    // Add a delay
    msleep(10000);
    return IRQ_HANDLED;
}


static int adxl345_probe(struct i2c_client *client)
{
    u8 reg_data[2];
    int ret;

    // Print a message indicating that the probe function is running
    pr_info("ADXL345 probe function called\n");

    // Set the BW_RATE register for Output Data Rate to 100 Hz
    reg_data[0] = ADXL345_BW_RATE_REG;
    reg_data[1] = ADXL345_OUTPUT_DATA_RATE_100HZ;
    ret = i2c_master_send(client, reg_data, 2);
    if (ret != 2) {
        pr_err("Failed to set BW_RATE register: %d\n", ret);
        return ret;
    }

    // Enable watermark interrupt (INT_ENABLE register)
    reg_data[0] = ADXL345_INT_ENABLE_REG;
    reg_data[1] = ADXL345_FIFO_WATERMARK_INT;
    ret = i2c_master_send(client, reg_data, 2);
    if (ret != 2) {
        pr_err("Failed to set INT_ENABLE register: %d\n", ret);
        return ret;
    }

    // Set default data format (DATA_FORMAT register)
    reg_data[0] = ADXL345_DATA_FORMAT_REG;
    reg_data[1] = ADXL345_DATA_FORMAT_DEFAULT; // Default data format
    ret = i2c_master_send(client, reg_data, 2);
    if (ret != 2) {
        pr_err("Failed to set DATA_FORMAT register: %d\n", ret);
        return ret;
    }

    // Set FIFO Stream mode (FIFO_CTL register)
    reg_data[0] = ADXL345_FIFO_CTL_REG;
    reg_data[1] = ADXL345_FIFO_STREAM_MODE | ADXL345_FIFO_WATERMARK_LEVEL;
    ret = i2c_master_send(client, reg_data, 2);
    if (ret != 2) {
        pr_err("Failed to set FIFO_CTL register: %d\n", ret);
        return ret;
    }

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
    // Initialize the mutex
    mutex_init(&adxl_dev->lock);

    // Initialize the spinlock
    spin_lock_init(&adxl_dev->irq_lock);

    mutex_lock(&adxl_dev->lock);
    pr_info("mutex lock before setting client data\n");
    // Set the client data
    i2c_set_clientdata(client, adxl_dev);
    
    mutex_unlock(&adxl_dev->lock);
    pr_info("mutex unlock after setting client data\n");

    // Increment the accelerometer count
    accelerometer_count++;

    pr_info("Successfully register with the misc framework\n");

    // Initialize the accelerometer FIFO and wait queue
    INIT_KFIFO(adxl_dev->driver_fifo);
    init_waitqueue_head(&adxl_dev->fifo_queue);

    // Register a threaded IRQ
    ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
                                    adxl345_int, IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
                                    "adxl345_irq", adxl_dev);

    if (ret) {
        pr_err("Failed to request IRQ: %d\n", ret);
        return ret;
    }

    pr_info("IRQ number: %d\n", client->irq);

    // Activate Measurement mode (POWER_CTL register)
    reg_data[0] = ADXL345_POWER_CTL_REG;
    reg_data[1] = ADXL345_MEASURE_MODE;
    ret = i2c_master_send(client, reg_data, 2);
    if (ret != 2) {
        pr_err("Failed to set POWER_CTL register: %d\n", ret);
        return ret;
    }

    pr_info("Successfully probe and init fifo\n");

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

    if (ret != 2) {
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

 