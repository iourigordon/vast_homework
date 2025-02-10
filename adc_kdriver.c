#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/spi/spi.h>
#include <linux/ioctl.h>

/*
PRU shared memory base starts at offset 0x10000
*/
#define PRU_SHARED_MEM  0x4A310000
#define BUFFER_SIZE     1024
#define DEVICE_NAME     "pru_adc"
#define PRU_IRQ         50  // IRQ number mapped in device tree
#define PRU_IRQ_CMD     51  // IRQ number mapped in device tree

#define IOCTL_SET_REGS  _IOW('a', 1, int16_t*)  // Set ADC registers
#define IOCTL_GET_REGS  _IOR('a', 2, int8_t*)  // Get ADC registers

static void __iomem *pru_shared_mem;
static struct cdev pru_adc_cdev;
static dev_t dev_num;
static struct class *pru_adc_class = NULL;
static wait_queue_head_t adc_wait_queue;
static wait_queue_head_t adc_wait_cmd_queue;
static int data_available = 0;
static struct spi_device *spi_dev = NULL;  // SPI device for ADC

int8_t ioctl_val;
int16_t ioctl_wr_val;

typedef struct {
    uint16_t data[BUFFER_SIZE];
    uint32_t head;
    uint32_t tail;

    uint8_t  command; /* only read and write register cmd and addr*/
    uint8_t  value;
} CircularBuffer;

static irqreturn_t pru_adc_irq_handler(int irq, void *dev_id) {
    CircularBuffer *buffer = (CircularBuffer *) pru_shared_mem;

    if (ioread32(&buffer->head) != ioread32(&buffer->tail)) {
        data_available = 1;
        wake_up_interruptible(&adc_wait_queue);  // Wake up waiting processes
    }

    return IRQ_HANDLED;
}

static irqreturn_t pru_adc_cmd_irq_handler(int irq, void *dev_id) {

    wake_up_interruptible(&adc_wait_cmd_queue);  // Wake up waiting processes
    return IRQ_HANDLED;
}

static long pru_adc_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
	CircularBuffer *buffer = (CircularBuffer *) pru_shared_mem;

    switch (cmd) {
        case IOCTL_SET_REGS:
            if (copy_from_user(&ioctl_wr_val, (u16 *)arg, sizeof(u16)))
                return -EFAULT;

			iowrite16(ioctl_wr_val,&buffer->command);
			wake_up_interruptible(&adc_wait_cmd_queue);  // Wake up waiting processes
            return 0;

        case IOCTL_GET_REGS:
			if (copy_from_user(&ioctl_val,(uint8_t *)arg, sizeof(uint8_t)))
				return -EFAULT;

			iowrite8(ioctl_val,&buffer-command);

			wake_up_interruptible(&adc_wait_cmd_queue);  // Wake up waiting processes

			ioctl_val = ioread8(&buffer->value);
			if (copy_to_user((uint8_t *)arg, &ioctl_val, sizeof(uint8_t)))
                return -EFAULT;
            return 0;

        default:
            return -EINVAL;
    }
}

// Read ADC Data from PRU Shared Memory
static ssize_t pru_adc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos) {
    CircularBuffer *buffer = (CircularBuffer *) pru_shared_mem;

    if (ioread32(&buffer->tail) == ioread32(&buffer->head)) {
        wait_event_interruptible(adc_wait_queue, data_available);  // Wait for data
        data_available = 0;
    }

    uint32_t tail = ioread32(&buffer->tail);
    uint16_t adc_value = ioread16(&buffer->data[tail]);
    iowrite32((tail + 1) % BUFFER_SIZE, &buffer->tail); // Update tail pointer

    if (copy_to_user(buf, &adc_value, sizeof(adc_value)))
        return -EFAULT;

    return sizeof(adc_value);
}

static const struct file_operations pru_adc_fops = {
    .owner = THIS_MODULE,
    .read = pru_adc_read,
    .unlocked_ioctl = pru_adc_ioctl,
};

static int __init pru_adc_init(void) {
    if (alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME)) {
        pr_err("Failed to allocate char device\n");
        return -1;
    }

    cdev_init(&pru_adc_cdev, &pru_adc_fops);
    if (cdev_add(&pru_adc_cdev, dev_num, 1)) {
        pr_err("Failed to add cdev\n");
        unregister_chrdev_region(dev_num, 1);
        return -1;
    }

    pru_adc_class = class_create(THIS_MODULE, DEVICE_NAME);
    if (!pru_adc_class) {
        cdev_del(&pru_adc_cdev);
        unregister_chrdev_region(dev_num, 1);
        return -1;
    }

    device_create(pru_adc_class, NULL, dev_num, NULL, DEVICE_NAME);

    pru_shared_mem = ioremap(PRU_SHARED_MEM, sizeof(CircularBuffer));
    if (!pru_shared_mem) {
        pr_err("Failed to map PRU shared memory\n");
        device_destroy(pru_adc_class, dev_num);
        class_destroy(pru_adc_class);
        cdev_del(&pru_adc_cdev);
        unregister_chrdev_region(dev_num, 1);
        return -1;
    }

    init_waitqueue_head(&adc_wait_queue);
	init_waitqueue_head(&adc_wait_cmd_queue);

    ret = request_irq(PRU_IRQ_DATA, pru_adc_irq_handler, IRQF_SHARED, "pru_adc_irq", (void *)pru_adc_irq_handler);
    if (ret) {
        pr_err("Failed to request IRQ %d\n", PRU_IRQ);
        iounmap(pru_shared_mem);
        device_destroy(pru_adc_class, dev_num);
        class_destroy(pru_adc_class);
        cdev_del(&pru_adc_cdev);
        unregister_chrdev_region(dev_num, 1);
        return -1;
    }

    ret = request_irq(PRU_IRQ_CMD, pru_adc_cmd_irq_handler, IRQF_SHARED,"pru_adc_cmd_irq", (void *)pru_adc_cmd_irq_handler);
    if (ret) {
		free_irq(PRU_IRQ, (void *)pru_adc_irq_handler);
        pr_err("Failed to request IRQ %d\n", PRU_IRQ);
        iounmap(pru_shared_mem);
        device_destroy(pru_adc_class, dev_num);
        class_destroy(pru_adc_class);
        cdev_del(&pru_adc_cdev);
        unregister_chrdev_region(dev_num, 1);
        return -1;
    }

    pr_info("PRU ADC driver with ioctl initialized\n");
    return 0;
}

static void __exit pru_adc_exit(void) {
	free_irq(PRU_IRQ, (void *)pru_adc_irq_handler);
	free_irq(PRU_IRQ_CMD, (void *)pru_adc_cmd_irq_handler);
    iounmap(pru_shared_mem);
    device_destroy(pru_adc_class, dev_num);
    class_destroy(pru_adc_class);
    cdev_del(&pru_adc_cdev);
    unregister_chrdev_region(dev_num, 1);
    pr_info("PRU ADC driver removed\n");
}

module_init(pru_adc_init);
module_exit(pru_adc_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("pru adc driver");

