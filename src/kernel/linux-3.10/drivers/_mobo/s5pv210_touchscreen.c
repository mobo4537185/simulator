/*
 * Samsung s5pv210 touchscreen driver
 *
 * author:mobo
 * copyright @ 20141015
 */
#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <plat/gpio-cfg.h>
#include <linux/interrupt.h>
#include <linux/slab.h>

//const static unsigned short normal_address[] = {0x5d, I2C_CLIENT_END};
//static unsigned FT5306DE4_int;
static struct input_dev *ts_input;
static struct workqueue_struct *wq;
static struct work_struct work;

static struct i2c_client * this_client = NULL;

#define EVENT_PRESS 200
#define SCREEN_MAX_X 499
#define SCREEN_MAX_Y 799

static unsigned int status = 0;

struct point_position {
	int count;
	short x[5];
	short y[5];
};

static struct point_position *points = NULL;
static int i2c_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
{
    int ret=-1;
    struct i2c_msg msgs[] =
    {
        {
            .addr   = this_client->addr,
            .flags  = I2C_M_RD,
            .len    = len,
            .buf    = buf,
        },
    };


    ret=i2c_transfer(client->adapter,msgs, 1);
    return ret;
}

static int i2c_write_bytes(struct i2c_client *client,uint8_t *data,int len)
{
    struct i2c_msg msg;
    int ret=-1;

    msg.flags=!I2C_M_RD;
    msg.addr=client->addr;
    msg.len=len;
    msg.buf=data;

    ret=i2c_transfer(client->adapter,&msg, 1);
    return ret;
}

static const struct i2c_device_id ts_id[] = {
    { "FT5306DE4-ts", 0 },
    { }
};

static irqreturn_t FT5306_int_handler(int irq, void *devid){
    disable_irq_nosync(this_client->irq);
    queue_work(wq, &work);
    return IRQ_RETVAL(IRQ_HANDLED);

}

static void ft5x0x_ts_report(struct point_position *points) {
	int i =0;
 //  printk("point count: %d\n", points->count);
	for (i = 0; i < points->count; i++) {
        printk("stat: %d, x: %d, y: %d\n", i, points->x[i], points->y[i]);
#if 0
        // 多触点驱动支持
        input_report_abs(ts_input, ABS_MT_POSITION_X, points->x[i]);
        input_report_abs(ts_input, ABS_MT_POSITION_Y, points->y[i]);

        input_report_abs(ts_input, ABS_MT_PRESSURE, EVENT_PRESS);
        input_report_abs(ts_input, ABS_MT_TOUCH_MAJOR, EVENT_PRESS);
        input_report_abs(ts_input, ABS_MT_TRACKING_ID, i);

        input_mt_sync(ts_input);
#endif
        //单触点支持
        input_report_abs(ts_input, ABS_X, points->x[i]);
        input_report_abs(ts_input, ABS_Y, points->y[i]);
        input_report_abs(ts_input, ABS_PRESSURE, 1);
        input_report_key(ts_input, BTN_TOUCH, 1);
        input_mt_sync(ts_input);
        break;
	}

    input_sync(ts_input);
}


static void ts_work_func(struct work_struct* work){
    int ret;
    unsigned char point_data[32] = {0};

    points = (struct point_position *)kmalloc(sizeof(struct point_position), GFP_KERNEL);
    if (points == NULL) {
    	return ;
    }

    ret=i2c_read_bytes(this_client, point_data, sizeof(point_data)/sizeof(point_data[0]));
    if(ret <= 0){
        printk("Failed\n");
        kfree (points);
        return;
    }
/*
    int i = 0;
    for (i = 0; i < 19; i++) {
    	printk("function:ts_work_func point_data[%d]=%d\n",i,point_data[i]);
    }
*/
	points->count = point_data[2];

    if(point_data[2]&0x1){
        status = 1;

        switch (point_data[2]) {
            case 5:
            	points->x[4] = (s16)(point_data[0x1b] & 0x0F)<<8 | (s16)point_data[0x1c];
            	points->y[4] = (s16)(point_data[0x1d] & 0x0F)<<8 | (s16)point_data[0x1e];
            case 4:
            	points->x[3] = (s16)(point_data[0x15] & 0x0F)<<8 | (s16)point_data[0x16];
            	points->y[3] = (s16)(point_data[0x17] & 0x0F)<<8 | (s16)point_data[0x18];
            case 3:
            	points->x[2] = (s16)(point_data[0x0f] & 0x0F)<<8 | (s16)point_data[0x10];
            	points->y[2] = (s16)(point_data[0x11] & 0x0F)<<8 | (s16)point_data[0x12];
            case 2:
            	points->x[1] = (s16)(point_data[0x09] & 0x0F)<<8 | (s16)point_data[0x0a];
            	points->y[1] = (s16)(point_data[0x0b] & 0x0F)<<8 | (s16)point_data[0x0c];
            case 1:
            	points->x[0] = (s16)(point_data[0x03] & 0x0F)<<8 | (s16)point_data[0x04];
            	points->y[0] = (s16)(point_data[0x05] & 0x0F)<<8 | (s16)point_data[0x06];

                break;
            default:
                printk("%s: invalid touch data, %d\n", __func__, point_data[2]);
                kfree (points);
                return ;
        }
    }

    // 发送event事件
    ft5x0x_ts_report(points);

    enable_irq(this_client->irq);
    kfree (points);
}

static int ts_probe(struct i2c_client *client, const struct i2c_device_id *id){
    int retry, ret;
    char test_data;

    printk("ts_probe\n");

    test_data = 0;

    gpio_request(S5PV210_GPH1(6), "ts_int");
    s3c_gpio_cfgpin(S5PV210_GPH1(6), S3C_GPIO_SFN(0xf));
    gpio_free(S5PV210_GPH1(6));

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        dev_err(&client->dev, "Must have I2C_FUNC_I2C.\n");
        return -ENODEV;
    }

    dev_info(&client->dev,"address %d\n",client->addr);
    for(retry=0;retry < 5; retry++)
    {
        ret =i2c_write_bytes(client, &test_data, 1);
        if (ret > 0)
            break;

        dev_info(&client->dev, "FT5306 I2C TEST FAILED!Please check the HARDWARE connect\n");
    }

    if(ret <= 0)
    {
        dev_err(&client->dev, "Warnning: I2C communication might be ERROR!\n");
        return -ENODEV;
    }

    dev_info(&client->dev, "I2C check successed\n");

    this_client = client;

    ts_input = input_allocate_device();
    if(IS_ERR(ts_input)){
        printk("FT5306 allocate ts input device failed!\n");
        return -ENOMEM;
    }
    dev_info(&client->dev, "ts_input alloc successed\n");


    set_bit(ABS_MT_TOUCH_MAJOR, ts_input->absbit);
    set_bit(ABS_MT_POSITION_X, ts_input->absbit);
    set_bit(ABS_MT_POSITION_Y, ts_input->absbit);
    set_bit(ABS_MT_WIDTH_MAJOR, ts_input->absbit);

    ts_input->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
    ts_input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
    ts_input->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);

    // 单触点支持
    input_set_abs_params(ts_input, ABS_X, 0, 799, 0, 0);
    input_set_abs_params(ts_input, ABS_Y, 0, 479, 0, 0);
    input_set_abs_params(ts_input, ABS_PRESSURE, 0, 255, 0, 0);

#if 0
    // 多触点支持
    input_set_abs_params(ts_input,
                 ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
    input_set_abs_params(ts_input,
                 ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
    input_set_abs_params(ts_input,
                 ABS_MT_TOUCH_MAJOR, 0, 200, 0, 0);
    input_set_abs_params(ts_input,
                 ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
    input_set_abs_params(ts_input,
                 ABS_MT_TRACKING_ID, 0, 5, 0, 0);
#endif

    ts_input->name = "mobo210-ts";
    ts_input->phys = "input/ts";
    ts_input->id.bustype = BUS_I2C;
    ts_input->id.product = 0xBEEF;
    ts_input->id.vendor  =0xDEAD;


    ret = input_register_device(ts_input);
    if(ret < 0){
        printk("Unable register %s input device!\n", ts_input->name);
        input_free_device(ts_input);
        return -ENOMEM;
    }
    dev_info(&client->dev, "ts_input register successed\n");

    client->irq = IRQ_EINT(14);

    wq = create_workqueue("ts_handle_thread");
    if(wq == NULL){
        printk(KERN_ALERT "crete workqueue failed!\n");
        dev_info(&client->dev, "crete workqueue failed!\n");
        input_unregister_device(ts_input);
        input_free_device(ts_input);
        free_irq(IRQ_EINT(14), NULL);
        return -ENOMEM;
    }
    dev_info(&client->dev, "workqueue register successed\n");
    INIT_WORK(&work, ts_work_func);
    dev_info(&client->dev, "workqueue init successed\n");

    if(request_irq(IRQ_EINT(14), FT5306_int_handler, IRQF_TRIGGER_FALLING, "FT5306DE4-ts", NULL) < 0){
        printk("Request irq for FT5306 failed!\n");
        input_unregister_device(ts_input);
        input_free_device(ts_input);
        return -ENOMEM;
    }
    dev_info(&client->dev, "irq register successed\n");

    return 0;
}

static int ts_remove(struct i2c_client *client){

    free_irq(IRQ_EINT(14), NULL);
    enable_irq(client->irq);
    flush_workqueue(wq);
    destroy_workqueue(wq);

    input_unregister_device(ts_input);
    input_free_device(ts_input);

    return 0;
}

static struct i2c_driver ts_driver = {
    .driver = {
        .name = "FT5306DE4-ts",
        .owner = THIS_MODULE,
    },

    .probe = ts_probe,
    .remove = ts_remove,
    .id_table = ts_id,
//    .address_list = normal_address,
};

static int ts_init(void){
    printk("init\n");
    i2c_add_driver(&ts_driver);
    return 0;
}

static void ts_exit(void){
    i2c_del_driver(&ts_driver);
    printk("exit\n");
}

module_init(ts_init);
module_exit(ts_exit);
MODULE_LICENSE("GPL");
