/* drivers/input/touchscreen/ektf3k.c - ELAN EKTF3K FIFO verions of driver
 *
 * Copyright (C) 2011 Elan Microelectronics Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>

#include <linux/proc_fs.h>
#include <linux/wakelock.h>

#include <linux/i2c/ektf3k.h>

#define PACKET_SIZE		40
#define NEW_PACKET_SIZE	55
#define FINGER_NUM		10

#define PWR_STATE_DEEP_SLEEP	0
#define PWR_STATE_NORMAL		1

#define CMD_S_PKT		0x52
#define CMD_R_PKT		0x53
#define CMD_W_PKT		0x54

#define NORMAL_PKT		0x63
#define NEW_NOMARL_PKT	0x66

#define IDX_FINGER			3
#define MAX_FINGER_SIZE		31
#define MAX_FINGER_PRESSURE	255

#define ABS_MT_POSITION		0x2a /* Group a set of X and Y */

static uint8_t firmware_recovery = 0x00;
static uint8_t work_lock = 0;
static uint8_t power_source = 0;

struct ektf3k_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *elan_wq;
	struct work_struct work;
	int intr_gpio;
	int fw_ver;
	int fw_id;
	int x_resolution;
	int y_resolution;
	int abs_x_max;
	int abs_y_max;
	int rst_gpio;
	struct wake_lock wakelock;
};

static struct ektf3k_ts_data *private_ts = NULL;
static int ektf3k_ts_hw_reset(struct i2c_client *client);
static struct semaphore pSem;
static uint16_t mTouchStatus[FINGER_NUM] = {0};

static int ektf3k_ts_poll(struct i2c_client *client)
{
	struct ektf3k_ts_data *ts = i2c_get_clientdata(client);
	int status = 0, retry = 150;

	do {
		status = gpio_get_value(ts->intr_gpio);
		dev_dbg(&client->dev, "%s: status = %d\n", __func__, status);
		--retry;
		msleep(20);
	} while (status == 1 && retry > 0);

	dev_dbg(&client->dev, "%s: poll interrupt status %s\n",
			__func__, status == 1 ? "high" : "low");
	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int ektf3k_ts_read_command(struct i2c_client *client, u8 *cmd,
		u16 cmd_length, u16 value_length) {
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];
	int length = 0;

	msg[0].addr = client->addr;
	msg[0].flags = 0x00;
	msg[0].len = cmd_length;
	msg[0].buf = cmd;

	down(&pSem);
	length = i2c_transfer(adapter, msg, 1);
	up(&pSem);
	
	if (length == 1) // only send on packet
		return value_length;
	else
		return -EIO;
}

static int ektf3k_i2c_read_packet(struct i2c_client *client, 
		u8 *value, u16 value_length) {
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[1];
	int length = 0;


	msg[0].addr = client->addr;
	msg[0].flags = I2C_M_RD;
	msg[0].len = value_length;
	msg[0].buf = (u8 *) value;
	down(&pSem);
	length = i2c_transfer(adapter, msg, 1);
	up(&pSem);
	
	if (length == 1) // only send on packet
		return value_length;
	else
		return -EIO;
}

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[4] = { 0 };

	rc = ektf3k_ts_poll(client);
	if (rc < 0) {
		dev_err(&client->dev, "%s: IRQ is not low!\n", __func__);
		firmware_recovery = 1;
	}

	rc = i2c_master_recv(client, buf_recv, 4);
	if (rc < 0) {
		dev_err(&client->dev, "I2C message no Ack!\n");
			 return -EINVAL;
	}

	if (!(buf_recv[0] == 0x55 && buf_recv[1] == 0x55 &&
		buf_recv[2] == 0x55 && buf_recv[3] == 0x55)) {
		firmware_recovery = 0x80;
		return firmware_recovery;
	}

	return 0;
}

static int wait_for_IRQ_Low(struct i2c_client *client, int utime) {
	struct ektf3k_ts_data *ts = i2c_get_clientdata(client);
	int retry_times = 10;
	do {
		usleep_range(utime,utime + 500);
		if (gpio_get_value(ts->intr_gpio) == 0)
			return 0;
		--retry_times;
	} while(retry_times > 0);

	dev_err(&client->dev, "Wait IRQ timeout\n");
	return -1;
}

static int __fw_packet_handler(struct i2c_client *client, int immediate)
{
	struct ektf3k_ts_data *ts = i2c_get_clientdata(client);
	int rc;
	int major, minor;
	uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01};
	uint8_t cmd_x[] = {0x53, 0x60, 0x00, 0x00}; /*Get x resolution*/
	uint8_t cmd_y[] = {0x53, 0x63, 0x00, 0x00}; /*Get y resolution*/
	uint8_t cmd_id[] = {0x53, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
	uint8_t buf_recv[4] = {0};

	// Firmware version
	rc = ektf3k_ts_read_command(client, cmd, 4, 4);
	if (rc < 0)
		return rc;

	if (immediate) {
		wait_for_IRQ_Low(client, 1000);
		ektf3k_i2c_read_packet(client, buf_recv, 4);
		major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
		minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
		ts->fw_ver = major << 8 | minor;
		dev_dbg(&client->dev, "%s: firmware version: 0x%4.4x\n",
			__func__, ts->fw_ver);
	}

	// X Resolution
	rc = ektf3k_ts_read_command(client, cmd_x, 4, 4);
	if (rc < 0)
		return rc;
	
	if (immediate) {
		wait_for_IRQ_Low(client, 1000);
		ektf3k_i2c_read_packet(client, buf_recv, 4);
		minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
		ts->x_resolution =minor;
		dev_dbg(&client->dev, "%s: X resolution: 0x%4.4x\n",
			__func__, ts->x_resolution);
	}

	// Y Resolution	
	rc = ektf3k_ts_read_command(client, cmd_y, 4, 4);
	if (rc < 0)
		return rc;
	
	if (immediate) {
		wait_for_IRQ_Low(client, 1000);
		ektf3k_i2c_read_packet(client, buf_recv, 4);
		minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
		ts->y_resolution =minor;
		dev_dbg(&client->dev, "%s: Y resolution: 0x%4.4x\n",
			__func__, ts->y_resolution);
	}

	// Firmware ID
	rc = ektf3k_ts_read_command(client, cmd_id, 4, 4);
	if (rc < 0)
		return rc;
	
	if (immediate) {
		wait_for_IRQ_Low(client, 1000);
		ektf3k_i2c_read_packet(client, buf_recv, 4);
		major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
		minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
		ts->fw_id = major << 8 | minor;
		dev_dbg(&client->dev, "%s: firmware id: 0x%4.4x\n",
			__func__, ts->fw_id);
	}

	return 0;
}

static inline int ektf3k_ts_parse_xy(uint8_t *data, uint16_t *x, uint16_t *y)
{
	*x = *y = 0;

	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	return 0;
}

static int ektf3k_ts_setup(struct i2c_client *client)
{
	int rc, count = 10;

retry:	
	ektf3k_ts_hw_reset(client);
	
	rc = __hello_packet_handler(client);
	dev_dbg(&client->dev, "%s: hello packet's rc = %d\n", __func__, rc);
	if (rc < 0) { 
		if (rc == -ETIME && count > 0) {
			--count;
			dev_err(&client->dev, "wait main hello timeout, reset\n");
			goto retry;
		} else
			goto hand_shake_failed;
	}

	dev_dbg(&client->dev, "%s: hello packet received\n", __func__);

	msleep(200);

	rc = __fw_packet_handler(client, 1);
	if (rc < 0)
		goto hand_shake_failed;

	dev_dbg(&client->dev, "%s: firmware checking done\n", __func__);

hand_shake_failed:
	return rc;
}

static int ektf3k_ts_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};
	int length;

	dev_dbg(&client->dev, "%s: enter\n", __func__);

	cmd[1] |= (state << 3);

	dev_dbg(&client->dev,
		"dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);

	down(&pSem);
	length = i2c_master_send(client, cmd, sizeof(cmd));
	up(&pSem);
	if (length != sizeof(cmd)) {
		dev_err(&client->dev,
			"%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int ektf3k_ts_hw_reset(struct i2c_client *client)
{
	struct ektf3k_ts_data *ts = i2c_get_clientdata(client);
	dev_dbg(&client->dev, "hardware reset\n");
	gpio_direction_output(ts->rst_gpio, 0);
	usleep_range(1000, 1500);
	gpio_direction_output(ts->rst_gpio, 1);
	msleep(250);
	return 0;
}

static int ektf3k_ts_set_power_source(struct i2c_client *client, u8 state)
{
	struct ektf3k_ts_data *ts = i2c_get_clientdata(client);
	uint8_t cmd[] = {CMD_W_PKT, 0x40, 0x00, 0x01};
	int length = 0;

	dev_dbg(&client->dev, "%s: enter\n", __func__);

	// 0x52 0x40 0x00 0x01 => Battery Mode
	// 0x52 0x41 0x00 0x01 => USB and AC Adapter Mode
	cmd[1] |= state & 0x0F;

	dev_dbg(&client->dev,
		"dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);
	dev_info(&ts->client->dev, "setting power source to %d\n", state);	
	down(&pSem);
	length = i2c_master_send(client, cmd, sizeof(cmd));
	up(&pSem);
	if (length != sizeof(cmd)) {
		dev_err(&client->dev,
			"%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static void update_power_source(void) {
	// set power source to 1 if a charger is connected
	if (private_ts != NULL && !work_lock)
		ektf3k_ts_set_power_source(private_ts->client, power_source != 0);
}

void touch_callback(uint8_t cable_status) { 
	power_source = cable_status;
	update_power_source();
}

static int ektf3k_ts_recv_data(struct i2c_client *client, uint8_t *buf, int size)
{
	int rc;

	if (buf == NULL)
		return -EINVAL;

	memset(buf, 0, size);
	rc = i2c_master_recv(client, buf, size);

	if (rc != size) {
		dev_err(&client->dev,
			"%s: i2c_master_recv error?! \n", __func__);
		rc = i2c_master_recv(client, buf, size);
		return -EINVAL;
	}

	return rc;
}

static void ektf3k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct ektf3k_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *idev = ts->input_dev;
	uint16_t x, y, touch_size, pressure_size;
	uint16_t fbits, checksum = 0;
	uint8_t i, num;
	static uint8_t size_index[10] = {35, 35, 36, 36, 37, 37, 38, 38, 39, 39};
	uint16_t active;
	uint8_t idx = IDX_FINGER;

	num = buf[2] & 0xf; 
	for (i=0; i<34;i++)
		checksum +=buf[i];
	
	if ((num < 3) || ((checksum & 0x00ff) == buf[34])) {
		fbits = buf[2] & 0x30;	
		fbits = (fbits << 4) | buf[1]; 
		for (i = 0; i < FINGER_NUM; i++) {
			active = fbits & 0x1;
			if (active || mTouchStatus[i]) {
				input_mt_slot(ts->input_dev, i);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, active);
				if (active) {
					ektf3k_ts_parse_xy(&buf[idx], &x, &y);
#ifdef CONFIG_MACH_TRANSFORMER
                    y = ts->abs_y_max - y;
#else
					x = x > ts->abs_x_max ? 0 : ts->abs_x_max - x;
					y = y > ts->abs_y_max ? ts->abs_y_max : y; 
#endif
					touch_size = ((i & 0x01) ? buf[size_index[i]] : (buf[size_index[i]] >> 4)) & 0x0F;
					pressure_size = touch_size << 4; // max pressure value is 255
					input_report_abs(idev, ABS_MT_TOUCH_MAJOR, touch_size);
					input_report_abs(idev, ABS_MT_PRESSURE, pressure_size);
#ifdef CONFIG_MACH_TRANSFORMER
                    input_report_abs(idev, ABS_MT_POSITION_X, x);
                    input_report_abs(idev, ABS_MT_POSITION_Y, y);
#else
					input_report_abs(idev, ABS_MT_POSITION_X, y);
					input_report_abs(idev, ABS_MT_POSITION_Y, x);
#endif
				}
			}
			mTouchStatus[i] = active;
			fbits = fbits >> 1;
			idx += 3;
		}
		input_sync(idev);
	} else
		dev_err(&client->dev, "Checksum Error, byte[2]=%X\n", buf[2]);
}

static void ektf3k_ts_report_data2(struct i2c_client *client, uint8_t *buf)
{
	struct ektf3k_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *idev = ts->input_dev;
	uint16_t x, y, touch_size, pressure_size;
	uint16_t fbits, checksum=0;
	uint8_t i, num;
	uint16_t active = 0; 
	uint8_t idx = IDX_FINGER;

	num = buf[2] & 0xf;
	for (i=0; i<34;i++)
		checksum +=buf[i];
	
	if ((num < 3) || ((checksum & 0x00ff) == buf[34])) {
		fbits = buf[2] & 0x30;	
		fbits = (fbits << 4) | buf[1]; 
		for (i = 0; i < FINGER_NUM; i++) {
			active = fbits & 0x1;
			if (active || mTouchStatus[i]) {
			 input_mt_slot(ts->input_dev, i);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, active);
				if (active) {
				 ektf3k_ts_parse_xy(&buf[idx], &x, &y);
					x = x > ts->abs_x_max ? 0 : ts->abs_x_max - x;
			 y = y > ts->abs_y_max ? ts->abs_y_max : y;
			 touch_size = buf[35 + i];
			 pressure_size = buf[45 + i];	 
			 input_report_abs(idev, ABS_MT_TOUCH_MAJOR, touch_size);
			 input_report_abs(idev, ABS_MT_PRESSURE, pressure_size);
			 input_report_abs(idev, ABS_MT_POSITION_X, y);
			 input_report_abs(idev, ABS_MT_POSITION_Y, x);
			 }
		 }
		 mTouchStatus[i] = active;
			fbits = fbits >> 1;
			idx += 3;
		}
		input_sync(idev);
	} else {
		dev_err(&client->dev, "Checksum Error, byte[2]=%X\n", buf[2]);
	}
}

static void process_resp_message(struct ektf3k_ts_data *ts,
		const unsigned char *buf, unsigned int size) {
	int major, minor;

	if (buf == NULL) return;
	switch (buf[1] & 0xF0) {
	case 0x00:// firmware version
		major = ((buf[1] & 0x0f) << 4) | ((buf[2] & 0xf0) >> 4);
		minor = ((buf[2] & 0x0f) << 4) | ((buf[3] & 0xf0) >> 4);
		ts->fw_ver = major << 8 | minor;
		break;
	case 0x20: // Rough calibrate response
		major = buf[2] & 0x0F;		
		dev_info(&ts->client->dev, "Get the Rough Calibrate result:%d\n", major);
		break;
	case 0x40: // the power source
		major = buf[1] & 0x0f;
		dev_info(&ts->client->dev, "Get the power source:%d\n", major);
	case 0x50: // the power state
		major = buf[1] & 0x0f;
		dev_info(&ts->client->dev, "Get the power mode:%d\n", major);
		break;
	case 0xF0: // firmware id
		major = ((buf[1] & 0x0f) << 4) | ((buf[2] & 0xf0) >> 4);
		minor = ((buf[2] & 0x0f) << 4) | ((buf[3] & 0xf0) >> 4);
		ts->fw_id = major << 8 | minor;
		break;
	default: 
		dev_info(&ts->client->dev,
			"Get unknow packet {0x%02X, 0x%02X, 0x%02X, 0x%02X}\n",
			buf[0], buf[1], buf[2], buf[3]);	
	}
}

static void ektf3k_ts_work_func(struct work_struct *work)
{
	int rc;
	struct ektf3k_ts_data *ts = container_of(work, struct ektf3k_ts_data, work);
	uint8_t buf[NEW_PACKET_SIZE + 4] = { 0 };
	uint8_t buf1[NEW_PACKET_SIZE] = { 0 };
	uint8_t buf2[NEW_PACKET_SIZE] = { 0 };

	if (work_lock) {
		dev_info(&ts->client->dev, "Firmware update during touch event handling\n");
		enable_irq(ts->client->irq);
		return;
	}

	down(&pSem);

	// read the first four bytes
	rc = ektf3k_ts_recv_data(ts->client, buf, 4);

	if (rc < 0) {
		up(&pSem);
		enable_irq(ts->client->irq);
		return;
	}

	switch (buf[0]) {
	case NORMAL_PKT:
		// read the finger report packet
		rc = ektf3k_ts_recv_data(ts->client, buf + 4, 40);
		up(&pSem);
		ektf3k_ts_report_data(ts->client, buf + 4);
		 // Second package
	 	if ((buf[1] == 2) || (buf[1] == 3)) {
			rc = ektf3k_ts_recv_data(ts->client, buf1, PACKET_SIZE);
			if (rc < 0) {
				enable_irq(ts->client->irq);
				return;
			}
			ektf3k_ts_report_data(ts->client, buf1);
		}
		
		// Final package
		if (buf[1] == 3) {
			rc = ektf3k_ts_recv_data(ts->client, buf2, PACKET_SIZE);
			if (rc < 0) {
				enable_irq(ts->client->irq);
				return;
			}
			ektf3k_ts_report_data(ts->client, buf2);
		}
		break;
	case NEW_NOMARL_PKT:
		// read the finger report packet.
		rc = ektf3k_ts_recv_data(ts->client, buf+4, NEW_PACKET_SIZE);
		up(&pSem);
		ektf3k_ts_report_data2(ts->client, buf+4);
		// Second package
	 	if ((buf[1] == 2) || (buf[1] == 3)) {
			rc = ektf3k_ts_recv_data(ts->client, buf1,NEW_PACKET_SIZE);
			if (rc < 0) {
				enable_irq(ts->client->irq);
				return;
			}
			ektf3k_ts_report_data2(ts->client, buf1);
		}

		// Final package
		if (buf[1] == 3) {
			rc = ektf3k_ts_recv_data(ts->client, buf2, NEW_PACKET_SIZE);
			if (rc < 0) {
				enable_irq(ts->client->irq);
				return;
			}
			ektf3k_ts_report_data2(ts->client, buf2);
		}
		break;
	case CMD_S_PKT:
		up(&pSem);
		process_resp_message(ts, buf, 4);
		break;
	default:
		up(&pSem);	
		dev_info(&ts->client->dev, "Get unknow packet {0x%02X, 0x%02X, 0x%02X, 0x%02X}\n",
			buf[0], buf[1], buf[2], buf[3]);
	}

	enable_irq(ts->client->irq);
}

static irqreturn_t ektf3k_ts_irq_handler(int irq, void *dev_id)
{
	struct ektf3k_ts_data *ts = dev_id;
	struct i2c_client *client = ts->client;

	dev_dbg(&client->dev, "%s\n", __func__);
	disable_irq_nosync(ts->client->irq);
	queue_work(ts->elan_wq, &ts->work);

	return IRQ_HANDLED;
}

static int ektf3k_ts_register_interrupt(struct i2c_client *client)
{
	struct ektf3k_ts_data *ts = i2c_get_clientdata(client);
	int err = 0;

	err = request_irq(client->irq, ektf3k_ts_irq_handler,
			IRQF_TRIGGER_LOW, client->name, ts);
	if (err)
		dev_err(&client->dev, "%s: request_irq %d failed\n",
				__func__, client->irq);

	return err;
}

static int ektf3k_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;
	struct elan_ktf3k_i2c_platform_data *pdata;
	struct ektf3k_ts_data *ts;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct ektf3k_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		dev_info(&client->dev, "%s: allocate ektf3k_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->elan_wq = create_singlethread_workqueue("elan_wq");
	if (!ts->elan_wq) {
		dev_err(&client->dev, "%s: create workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_wq_failed;
	}

	INIT_WORK(&ts->work, ektf3k_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;

	if (likely(pdata != NULL)) {
		ts->intr_gpio = pdata->intr_gpio;
		ts->rst_gpio = pdata->rst_gpio;
	}

	sema_init(&pSem, 1);
	err = ektf3k_ts_setup(client);
	if (err < 0) {
		dev_err(&client->dev, "Main code fail\n");
		firmware_recovery = 1;
		err = 0;
	}
	
	wake_lock_init(&ts->wakelock, WAKE_LOCK_SUSPEND, "elan_touch");
	if (err==0x80)
		dev_info(&client->dev, "Touch is in boot mode!\n");

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev, "Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "elan-touchscreen";

	ts->abs_x_max = pdata->abs_x_max;
	ts->abs_y_max = pdata->abs_y_max;

	input_mt_init_slots(ts->input_dev, FINGER_NUM);
#ifdef CONFIG_MACH_TRANSFORMER
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, pdata->abs_x_min, pdata->abs_x_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, pdata->abs_y_min, pdata->abs_y_max, 0, 0);
#else
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, pdata->abs_y_min, pdata->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, pdata->abs_x_min, pdata->abs_x_max, 0, 0);
#endif
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, MAX_FINGER_SIZE, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, MAX_FINGER_PRESSURE, 0, 0);

	__set_bit(EV_ABS, ts->input_dev->evbit);
	__set_bit(EV_SYN, ts->input_dev->evbit);
	__set_bit(EV_KEY, ts->input_dev->evbit);
	
	err = input_register_device(ts->input_dev);
	if (err) {
		dev_err(&client->dev,
			"%s: unable to register %s input device\n",
			__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	ektf3k_ts_register_interrupt(ts->client);

	if (gpio_get_value(ts->intr_gpio) == 0) {
		dev_info(&client->dev, "%s: handle missed interrupt\n", __func__);
		ektf3k_ts_irq_handler(client->irq, ts);
	}

	private_ts = ts;

	update_power_source();

	dev_info(&client->dev, "probed\n");

	return 0;

err_input_register_device_failed:
	if (ts->input_dev)
		input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);

err_create_wq_failed:
	kfree(ts);

err_alloc_data_failed:
err_check_functionality_failed:

	return err;
}

static int ektf3k_ts_remove(struct i2c_client *client)
{
	struct ektf3k_ts_data *ts = i2c_get_clientdata(client);

	free_irq(client->irq, ts);

	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);

	input_unregister_device(ts->input_dev);

	wake_lock_destroy(&ts->wakelock);

	kfree(ts);

	return 0;
}

void force_release_pos(struct i2c_client *client)
{
	struct ektf3k_ts_data *ts = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < FINGER_NUM; i++) {
		if (mTouchStatus[i] == 0) continue;
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
		mTouchStatus[i] = 0;
	}

	input_sync(ts->input_dev);
}

#ifdef CONFIG_PM
static int ektf3k_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ektf3k_ts_data *ts = i2c_get_clientdata(client);
	int rc;

	disable_irq(client->irq);

	force_release_pos(client);

	rc = cancel_work_sync(&ts->work);
	if (rc)
		enable_irq(client->irq);

	if (!work_lock)
		ektf3k_ts_set_power_state(client, PWR_STATE_DEEP_SLEEP);

	return 0;
}

static int ektf3k_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (!work_lock)
		ektf3k_ts_set_power_state(client, PWR_STATE_NORMAL);

	enable_irq(client->irq);	

	return 0;
}

static SIMPLE_DEV_PM_OPS(ektf3k_ts_pm, ektf3k_ts_suspend, ektf3k_ts_resume);
#endif

static const struct i2c_device_id ektf3k_ts_id[] = {
	{ ELAN_KTF3K_NAME, 0 },
	{ }
};

static struct i2c_driver ektf3k_ts_driver = {
	.probe		= ektf3k_ts_probe,
	.remove		= ektf3k_ts_remove,
	.id_table	= ektf3k_ts_id,
	.driver		= {
		.name = ELAN_KTF3K_NAME,
#ifdef CONFIG_PM
		.pm = &ektf3k_ts_pm,
#endif
	},
};

static int __devinit ektf3k_ts_init(void)
{
	return i2c_add_driver(&ektf3k_ts_driver);
}

static void __exit ektf3k_ts_exit(void)
{
	i2c_del_driver(&ektf3k_ts_driver);
}

module_init(ektf3k_ts_init);
module_exit(ektf3k_ts_exit);

MODULE_DESCRIPTION("ELAN KTF3K Touchscreen Driver");
MODULE_LICENSE("GPL");
