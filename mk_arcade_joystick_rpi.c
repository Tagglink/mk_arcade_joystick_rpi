/*
*  Arcade Joystick Driver for RaspberryPi
*
*  Copyright (c) 2014 Matthieu Proucelle
*
*  Based on the gamecon driver by Vojtech Pavlik, and Markus Hiienkari
*/


/*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
* MA 02110-1301, USA.
*/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include <linux/i2c-dev.h>
#include <linux/ioport.h>
#include <asm/io.h>


MODULE_AUTHOR("Matthieu Proucelle");
MODULE_DESCRIPTION("GPIO, MCP23017 and Teensy Arcade Joystick Driver");
MODULE_LICENSE("GPL");

#define MK_MAX_DEVICES		9

#ifdef RPI2
#define PERI_BASE        0x3F000000
#else
#define PERI_BASE        0x20000000
#endif

#define GPIO_BASE                (PERI_BASE + 0x200000) /* GPIO controller */

#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define GPIO_READ(g)  *(gpio + 13) &= (1<<(g))

#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)
#define GPIO_CLR *(gpio+10)

#define BSC1_BASE		(PERI_BASE + 0x804000)


/*
* MCP23017 Defines
*/
#define MPC23017_GPIOA_MODE		0x00
#define MPC23017_GPIOB_MODE		0x01
#define MPC23017_GPIOA_PULLUPS_MODE	0x0c
#define MPC23017_GPIOB_PULLUPS_MODE	0x0d
#define MPC23017_GPIOA_READ             0x12
#define MPC23017_GPIOB_READ             0x13

/*
 * Teensy i2c gamepad defines
 */
#define TEENSY_READ_INPUT      0x10
#define TEENSY_BOUNCE_INTERVAL 0x20
#define TEENSY_CALIB_JS1X_MAX  0x30
#define TEENSY_CALIB_JS1Y_MAX  0x40
#define TEENSY_CALIB_JS1X_MIN  0x50
#define TEENSY_CALIB_JS1Y_MIN  0x60
#define TEENSY_CALIB_JS2X_MAX  0x70
#define TEENSY_CALIB_JS2Y_MAX  0x80
#define TEENSY_CALIB_JS2X_MIN  0x90
#define TEENSY_CALIB_JS2Y_MIN  0xA0
#define TEENSY_I2C_CLOCKRATE   0xB0

/*
* Defines for I2C peripheral (aka BSC, or Broadcom Serial Controller)
*/

#define BSC1_C		*(bsc1 + 0x00)
#define BSC1_S		*(bsc1 + 0x01)
#define BSC1_DLEN	*(bsc1 + 0x02)
#define BSC1_A		*(bsc1 + 0x03)
#define BSC1_FIFO	*(bsc1 + 0x04)

#define BSC_C_I2CEN	(1 << 15)
#define BSC_C_INTR	(1 << 10)
#define BSC_C_INTT	(1 << 9)
#define BSC_C_INTD	(1 << 8)
#define BSC_C_ST	(1 << 7)
#define BSC_C_CLEAR	(1 << 4)
#define BSC_C_READ	1

#define START_READ	BSC_C_I2CEN|BSC_C_ST|BSC_C_CLEAR|BSC_C_READ
#define START_WRITE	BSC_C_I2CEN|BSC_C_ST

#define BSC_S_CLKT	(1 << 9)
#define BSC_S_ERR	(1 << 8)
#define BSC_S_RXF	(1 << 7)
#define BSC_S_TXE	(1 << 6)
#define BSC_S_RXD	(1 << 5)
#define BSC_S_TXD	(1 << 4)
#define BSC_S_RXR	(1 << 3)
#define BSC_S_TXW	(1 << 2)
#define BSC_S_DONE	(1 << 1)
#define BSC_S_TA	1

#define CLEAR_STATUS	BSC_S_CLKT|BSC_S_ERR|BSC_S_DONE

static volatile unsigned *gpio;
static volatile unsigned *bsc1;

struct mk_config {
	int args[MK_MAX_DEVICES];
	unsigned int nargs;
};

static struct mk_config mk_cfg __initdata;

module_param_array_named(map, mk_cfg.args, int, &(mk_cfg.nargs), 0);
MODULE_PARM_DESC(map, "Enable or disable GPIO, MCP23017, TFT, Custom Arcade Joystick and Teensy");

struct gpio_config {
	int mk_arcade_gpio_maps_custom[12];
	unsigned int nargs;
};

static struct gpio_config gpio_cfg __initdata;

module_param_array_named(gpio, gpio_cfg.mk_arcade_gpio_maps_custom, int, &(gpio_cfg.nargs), 0);
MODULE_PARM_DESC(gpio, "Numbers of custom GPIO for Arcade Joystick");

struct i2c_config {
	int i2cAddresses[MK_MAX_DEVICES];
	unsigned int nargs;
};

static struct i2c_config i2c_cfg __initdata;

module_param_array_named(i2c, i2c_cfg.i2cAddresses, int, &(i2c_cfg.nargs), 0);
MODULE_PARM_DESC(i2c, "I2C addresses for the gamepads, in order of entry.");

enum mk_type {
	MK_NONE = 0,
	MK_ARCADE_GPIO,
	MK_ARCADE_GPIO_BPLUS,
	MK_ARCADE_MCP23017,
	MK_ARCADE_GPIO_TFT,
	MK_ARCADE_GPIO_CUSTOM,
	MK_TEENSY,
	MK_MAX
};

#define MK_REFRESH_TIME	HZ/100

struct mk_pad {
	struct input_dev *dev;
	enum mk_type type;
	char phys[32];
	int i2caddr;
	int gpio_maps[12];
};

struct mk_nin_gpio {
	unsigned pad_id;
	unsigned cmd_setinputs;
	unsigned cmd_setoutputs;
	unsigned valid_bits;
	unsigned request;
	unsigned request_len;
	unsigned response_len;
	unsigned response_bufsize;
};

struct mk {
	struct mk_pad pads[MK_MAX_DEVICES];
	struct timer_list timer;
	int pad_count[MK_MAX];
	int used;
	struct mutex mutex;
};

struct mk_subdev {
	unsigned int idx;
};

static struct mk *mk_base;

static const int mk_max_arcade_buttons = 12;

static const int mk_teensy_axis_count = 4;
static const int mk_teensy_button_count = 16;
static const int mk_teensy_input_bytes = 20;

static const int mk_i2c_timeout_cycles = 50;

// Map of the gpios :                     up, down, left, right, start, select, a,  b,  tr, y,  x,  tl
static const int mk_arcade_gpio_maps[] = { 4,  17,    27,  22,    10,    9,      25, 24, 23, 18, 15, 14 };
// 2nd joystick on the b+ GPIOS                 up, down, left, right, start, select, a,  b,  tr, y,  x,  tl
static const int mk_arcade_gpio_maps_bplus[] = { 11, 5,    6,    13,    19,    26,     21, 20, 16, 12, 7,  8 };
// Map of the mcp23017 on GPIOA            up, down, left, right, start, select
static const int mk_arcade_gpioa_maps[] = { 0,  1,    2,    3,     4,     5 };

// Map of the mcp23017 on GPIOB            a, b, tr, y, x, tl
static const int mk_arcade_gpiob_maps[] = { 0, 1, 2,  3, 4, 5 };

// Map joystick on the b+ GPIOS with TFT      up, down, left, right, start, select, a,  b,  tr, y,  x,  tl
static const int mk_arcade_gpio_maps_tft[] = { 21, 13,    26,    19,    5,    6,     22, 4, 20, 17, 27,  16 };

static const short mk_arcade_gpio_btn[] = {
	BTN_START, BTN_SELECT, BTN_A, BTN_B, BTN_TR, BTN_Y, BTN_X, BTN_TL
};

// Teensy axes (4): L-Stick X, L-Stick Y, R-Stick X, R-Stick Y
//                  ABS_X,     ABS_Y,     ABS_RX,    ABS_RY

// Teensy buttons (16): A, B, X, Y, L, R, Select, Start, L-Stick press, R-Stick press, D-Pad Left, D-Pad Right, D-Pad Up, D-Pad Down, Custom1, Custom2
static const short mk_teensy_buttons[] = { 
	BTN_A, BTN_B, BTN_X, BTN_Y, BTN_TL, BTN_TR, BTN_SELECT, BTN_START, BTN_THUMBL, BTN_THUMBR, BTN_DPAD_LEFT, BTN_DPAD_RIGHT, BTN_DPAD_UP, BTN_DPAD_DOWN, BTN_MODE, BTN_TRIGGER_HAPPY1
};

static const char *mk_names[] = {
	NULL, "GPIO Controller 1", "GPIO Controller 2", "MCP23017 Controller", "GPIO Controller 1" , "GPIO Controller 1", "Teensy Controller 1"
};

/* GPIO UTILS */
static void setGpioPullUps(int pullUps) {
	*(gpio + 37) = 0x02;
	udelay(10);
	*(gpio + 38) = pullUps;
	udelay(10);
	*(gpio + 37) = 0x00;
	*(gpio + 38) = 0x00;
}

static void setGpioAsInput(int gpioNum) {
	INP_GPIO(gpioNum);
}

static int getPullUpMask(int gpioMap[]) {
	int mask = 0x0000000;
	int i;
	for (i = 0; i<12; i++) {
		if (gpioMap[i] != -1) {   // to avoid unused pins
			int pin_mask = 1 << gpioMap[i];
			mask = mask | pin_mask;
		}
	}
	return mask;
}

/* I2C UTILS */
static void i2c_init(void) {
	INP_GPIO(2);
	SET_GPIO_ALT(2, 0);
	INP_GPIO(3);
	SET_GPIO_ALT(3, 0);
}

// timeout becomes 1 if timeout was encountered
static void wait_i2c_done(int* timeout) {
	int cycles = mk_i2c_timeout_cycles;

	while ((!((BSC1_S)& BSC_S_DONE)) && --cycles) {
		usleep_range(1000, 1100);
	}
	if (cycles == 0)
		*(timeout) = 1;
}

// Function to write data to an I2C device via the FIFO.  This doesn't refill the FIFO, so writes are limited to 16 bytes
// including the register address. len specifies the number of bytes in the buffer.
// timeout becomes 1 if timeout was encountered

static void i2c_write(char dev_addr, char reg_addr, char *buf, unsigned short len, int* timeout) {

	int idx;

	BSC1_A = dev_addr;
	BSC1_DLEN = len + 1; // one byte for the register address, plus the buffer length

	BSC1_FIFO = reg_addr; // start register address
	for (idx = 0; idx < len; idx++)
		BSC1_FIFO = buf[idx];

	BSC1_S = CLEAR_STATUS; // Reset status bits (see #define)
	BSC1_C = START_WRITE; // Start Write (see #define)

	wait_i2c_done(timeout);
}

// Function to read a number of bytes into a  buffer from the FIFO of the I2C controller

static void i2c_read(char dev_addr, char reg_addr, char *buf, unsigned short len) {
	int timeout = 0;
	
	do {
		i2c_write(dev_addr, reg_addr, NULL, 0, &timeout);
	} while (timeout > 0);

	unsigned short bufidx;
	bufidx = 0;

	memset(buf, 0, len); // clear the buffer

	BSC1_DLEN = len;
	BSC1_S = CLEAR_STATUS; // Reset status bits (see #define)
	BSC1_C = START_READ; // Start Read after clearing FIFO (see #define)

	do {
		// Wait for some data to appear in the FIFO
		while ((BSC1_S & BSC_S_TA) && !(BSC1_S & BSC_S_RXD));

		// Consume the FIFO
		while ((BSC1_S & BSC_S_RXD) && (bufidx < len)) {
			buf[bufidx++] = BSC1_FIFO;
		}
	} while ((!(BSC1_S & BSC_S_DONE)));
}

/*  ------------------------------------------------------------------------------- */

static void mk_mcp23017_read_packet(struct mk_pad * pad, unsigned char *data) {
	int i;
	char resultA, resultB;
	i2c_read(pad->i2caddr, MPC23017_GPIOA_READ, &resultA, 1);
	i2c_read(pad->i2caddr, MPC23017_GPIOB_READ, &resultB, 1);

	// read direction
	for (i = 0; i < 4; i++) {
		data[i] = !((resultA >> mk_arcade_gpioa_maps[i]) & 0x1);
	}
	// read buttons on gpioa
	for (i = 4; i < 6; i++) {
		data[i] = !((resultA >> mk_arcade_gpioa_maps[i]) & 0x1);
	}
	// read buttons on gpiob
	for (i = 6; i < 12; i++) {
		data[i] = !((resultB >> (mk_arcade_gpiob_maps[i - 6])) & 0x1);
	}
}

static void mk_gpio_read_packet(struct mk_pad * pad, unsigned char *data) {
	int i;

	for (i = 0; i < mk_max_arcade_buttons; i++) {
		if (pad->gpio_maps[i] != -1) {    // to avoid unused buttons
			int read = GPIO_READ(pad->gpio_maps[i]);
			if (read == 0) data[i] = 1;
			else data[i] = 0;
		}
		else data[i] = 0;
	}
}

// this function needs work
static void mk_teensy_read_packet(struct mk_pad * pad, unsigned char *data) {
	int i;

	/* 
	 * byte 0: L-Stick X
	 * byte 1: L-Stick Y
	 * byte 2: R-Stick X
	 * byte 3: R-Stick Y
	 * byte 4: A, B, X, Y, L, R, Select, Start
	 * byte 5: L-Stick, R-Stick, D-Pad Left, D-Pad Right, D-Pad Up, D-Pad Down, Custom1, Custom2
	 */
	char result[6];

	i2c_read(pad->i2caddr, TEENSY_READ_INPUT, result, 6);

	// read the first four bytes as axes
	for (i = 0; i < 4; i++) {
		data[i] = result[i];
	}

	// read 8 buttons in the 5th byte
	for (i = 4; i < 12; i++) {
		data[i] = (result[4] >> (i - 4)) & 0x1;
	}

	// read 8 buttons in the 6th byte
	for (i = 12; i < 20; i++) {
		data[i] = (result[5] >> (i - 12)) & 0x1;
	}
}

static void mk_input_report(struct mk_pad * pad, unsigned char * data) {
	struct input_dev * dev = pad->dev;
	int j;
	input_report_abs(dev, ABS_Y, !data[0] - !data[1]);
	input_report_abs(dev, ABS_X, !data[2] - !data[3]);
	for (j = 4; j < mk_max_arcade_buttons; j++) {
		input_report_key(dev, mk_arcade_gpio_btn[j - 4], data[j]);
	}
	input_sync(dev);
}

static void mk_teensy_input_report(struct mk_pad * pad, unsigned char * data) {
	struct input_dev * dev = pad->dev;
	int j;
	// send joystick data to input device
	input_report_abs(dev, ABS_X, data[0]);
	input_report_abs(dev, ABS_Y, data[1]);
	input_report_abs(dev, ABS_RX, data[2]);
	input_report_abs(dev, ABS_RY, data[3]);

	// send button data to input device
	for (j = 4; j < mk_teensy_input_bytes; j++) {
		input_report_key(dev, mk_teensy_buttons[j - 4], data[j]);
	}
	input_sync(dev);
}

static void mk_process_packet(struct mk *mk) {

	unsigned char data[mk_max_arcade_buttons];
	unsigned char teensy_data[mk_teensy_input_bytes];
	struct mk_pad *pad;
	int i;

	for (i = 0; i < MK_MAX_DEVICES; i++) {
		pad = &mk->pads[i];
		if (pad->type == MK_ARCADE_GPIO || pad->type == MK_ARCADE_GPIO_BPLUS || pad->type == MK_ARCADE_GPIO_TFT || pad->type == MK_ARCADE_GPIO_CUSTOM) {
			mk_gpio_read_packet(pad, data);
			mk_input_report(pad, data);
		}
		if (pad->type == MK_ARCADE_MCP23017) {
			mk_mcp23017_read_packet(pad, data);
			mk_input_report(pad, data);
		}
		if (pad->type == MK_TEENSY) {
			mk_teensy_read_packet(pad, teensy_data);
			mk_teensy_input_report(pad, teensy_data);
		}
	}

}

/*
* mk_timer() initiates reads of console pads data.
*/

static void mk_timer(unsigned long private) {
	struct mk *mk = (void *) private;
	mk_process_packet(mk);
	mod_timer(&mk->timer, jiffies + MK_REFRESH_TIME);
}

static int mk_open(struct input_dev *dev) {
	struct mk *mk = input_get_drvdata(dev);
	int err;

	err = mutex_lock_interruptible(&mk->mutex);
	if (err)
		return err;

	if (!mk->used++)
		mod_timer(&mk->timer, jiffies + MK_REFRESH_TIME);

	mutex_unlock(&mk->mutex);
	return 0;
}

static void mk_close(struct input_dev *dev) {
	struct mk *mk = input_get_drvdata(dev);

	mutex_lock(&mk->mutex);
	if (!--mk->used) {
		del_timer_sync(&mk->timer);
	}
	mutex_unlock(&mk->mutex);
}

static int __init mk_setup_pad(struct mk *mk, int idx, int pad_type_arg) {
	struct mk_pad *pad = &mk->pads[idx];
	struct input_dev *input_dev;
	int i, pad_type;
	int err;
	char FF = 0xFF;
	pr_err("pad type : %d\n", pad_type_arg);

	pad_type = pad_type_arg;

	if (pad_type < 1 || pad_type >= MK_MAX) {
		pr_err("Pad type %d unknown\n", pad_type);
		return -EINVAL;
	}

	if (pad_type == MK_ARCADE_GPIO_CUSTOM) {

		// if the device is custom, be sure to get correct pins
		if (gpio_cfg.nargs < 1) {
			pr_err("Custom device needs gpio argument\n");
			return -EINVAL;
		}
		else if (gpio_cfg.nargs != 12) {
			pr_err("Invalid gpio argument\n", pad_type);
			return -EINVAL;
		}
	}
	else if (pad_type == MK_ARCADE_MCP23017 || pad_type == MK_TEENSY) {
		if (i2c_cfg.nargs < 1) {
			pr_err("I2C device needs i2c argument\n");
			return -EINVAL;
		}
		else if (i2c_cfg.nargs < idx + 1) {
			pr_err("Number of i2c arguments does not match number of gamepads\n");
			return -EINVAL;
		}
	}

	pr_err("pad type : %d\n", pad_type);
	pad->dev = input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("Not enough memory for input device\n");
		return -ENOMEM;
	}

	pad->type = pad_type;
	pad->i2caddr = i2c_cfg.i2cAddresses[idx];
	snprintf(pad->phys, sizeof(pad->phys),
		"input%d", idx);

	input_dev->name = mk_names[pad_type];
	input_dev->phys = pad->phys;
	input_dev->id.bustype = BUS_PARPORT;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = pad_type;
	input_dev->id.version = 0x0100;

	input_set_drvdata(input_dev, mk);

	input_dev->open = mk_open;
	input_dev->close = mk_close;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	if (pad_type != MK_TEENSY) {
		for (i = 0; i < 2; i++)
			input_set_abs_params(input_dev, ABS_X + i, -1, 1, 0, 0);
		for (i = 0; i < mk_max_arcade_buttons - 4; i++)
			__set_bit(mk_arcade_gpio_btn[i], input_dev->keybit);
	}
	else { // if teensy, we are using two joysticks and more buttons
		// setup left stick axes
		for (i = 0; i < 2; i++)
			input_set_abs_params(input_dev, ABS_X + i, 0, 255, 4, 8);

		// setup right stick axes
		for (i = 0; i < 2; i++)
			input_set_abs_params(input_dev, ABS_RX + i, 0, 255, 4, 8);

		// setup buttons
		for (i = 0; i < mk_teensy_button_count; i++) {
			__set_bit(mk_teensy_buttons[i], input_dev->keybit);
		}
	}

	mk->pad_count[pad_type]++;

	// asign gpio pins
	switch (pad_type) {
	case MK_ARCADE_GPIO:
		memcpy(pad->gpio_maps, mk_arcade_gpio_maps, 12 * sizeof(int));
		break;
	case MK_ARCADE_GPIO_BPLUS:
		memcpy(pad->gpio_maps, mk_arcade_gpio_maps_bplus, 12 * sizeof(int));
		break;
	case MK_ARCADE_GPIO_TFT:
		memcpy(pad->gpio_maps, mk_arcade_gpio_maps_tft, 12 * sizeof(int));
		break;
	case MK_ARCADE_GPIO_CUSTOM:
		memcpy(pad->gpio_maps, gpio_cfg.mk_arcade_gpio_maps_custom, 12 * sizeof(int));
		break;
	case MK_ARCADE_MCP23017:
		// nothing to asign if MCP23017 is used
		break;
	case MK_TEENSY:
		// nothing to assign if teensy is used
		break;
	}

	// initialize gpio if not MCP23017 or teensy, else initialize i2c
	if (pad_type != MK_ARCADE_MCP23017 && pad_type != MK_TEENSY) {
		for (i = 0; i < mk_max_arcade_buttons; i++) {
			printk("GPIO = %d\n", pad->gpio_maps[i]);
			if (pad->gpio_maps[i] != -1) {    // to avoid unused buttons
				setGpioAsInput(pad->gpio_maps[i]);
			}
		}
		setGpioPullUps(getPullUpMask(pad->gpio_maps));
		printk("GPIO configured for pad%d\n", idx);
	}
	else if (pad_type == MK_ARCADE_MCP23017) {
		i2c_init();
		udelay(1000);
		// Put all GPIOA inputs on MCP23017 in INPUT mode
		i2c_write(pad->i2caddr, MPC23017_GPIOA_MODE, &FF, 1);
		udelay(1000);
		// Put all inputs on MCP23017 in pullup mode
		i2c_write(pad->i2caddr, MPC23017_GPIOA_PULLUPS_MODE, &FF, 1);
		udelay(1000);
		// Put all GPIOB inputs on MCP23017 in INPUT mode
		i2c_write(pad->i2caddr, MPC23017_GPIOB_MODE, &FF, 1);
		udelay(1000);
		// Put all inputs on MCP23017 in pullup mode
		i2c_write(pad->i2caddr, MPC23017_GPIOB_PULLUPS_MODE, &FF, 1);
		udelay(1000);
		// Put all inputs on MCP23017 in pullup mode a second time
		// Known bug : if you remove this line, you will not have pullups on GPIOB 
		i2c_write(pad->i2caddr, MPC23017_GPIOB_PULLUPS_MODE, &FF, 1);
		udelay(1000);
	}
	else { // if teensy, the pin setup is already done in the teensy
		i2c_init();
		udelay(1000);
		// will come back to this if there are any setup parameters I might
		// want to send to the Teensy from this driver
	}

	err = input_register_device(pad->dev);
	if (err)
		goto err_free_dev;

	return 0;

err_free_dev:
	input_free_device(pad->dev);
	pad->dev = NULL;
	return err;
}

static struct mk __init *mk_probe(int *pads, int n_pads) {
	struct mk *mk;
	int i;
	int count = 0;
	int err;

	mk = kzalloc(sizeof(struct mk), GFP_KERNEL);
	if (!mk) {
		pr_err("Not enough memory\n");
		err = -ENOMEM;
		goto err_out;
	}

	mutex_init(&mk->mutex);
	setup_timer(&mk->timer, mk_timer, (long)mk);

	for (i = 0; i < n_pads && i < MK_MAX_DEVICES; i++) {
		if (!pads[i])
			continue;

		err = mk_setup_pad(mk, i, pads[i]);
		if (err)
			goto err_unreg_devs;

		count++;
	}

	if (count == 0) {
		pr_err("No valid devices specified\n");
		err = -EINVAL;
		goto err_free_mk;
	}

	return mk;

err_unreg_devs:
	while (--i >= 0)
		if (mk->pads[i].dev)
			input_unregister_device(mk->pads[i].dev);
err_free_mk:
	kfree(mk);
err_out:
	return ERR_PTR(err);
}

static void mk_remove(struct mk *mk) {
	int i;

	for (i = 0; i < MK_MAX_DEVICES; i++)
		if (mk->pads[i].dev)
			input_unregister_device(mk->pads[i].dev);
	kfree(mk);
}

static int __init mk_init(void) {
	/* Set up gpio pointer for direct register access */
	if ((gpio = ioremap(GPIO_BASE, 0xB0)) == NULL) {
		pr_err("io remap failed\n");
		return -EBUSY;
	}
	/* Set up i2c pointer for direct register access */
	if ((bsc1 = ioremap(BSC1_BASE, 0xB0)) == NULL) {
		pr_err("io remap failed\n");
		return -EBUSY;
	}
	if (mk_cfg.nargs < 1) {
		pr_err("at least one device must be specified\n");
		return -EINVAL;
	}
	else {
		mk_base = mk_probe(mk_cfg.args, mk_cfg.nargs);
		if (IS_ERR(mk_base))
			return -ENODEV;
	}
	return 0;
}

static void __exit mk_exit(void) {
	if (mk_base)
		mk_remove(mk_base);

	iounmap(gpio);
	iounmap(bsc1);
}

module_init(mk_init);
module_exit(mk_exit);
