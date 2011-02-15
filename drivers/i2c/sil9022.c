/*
 * drivers/video/sil9022.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * SIL9022
 *
 */

/***********************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <mach/sil9022.h>

u16 current_descriptor_addrs;

static struct video_timings hdmi_timings = {
	.x_res          = HDMI_XRES,
	.y_res          = HDMI_YRES,
	.pixel_clock    = HDMI_PIXCLOCK_MAX,
	.hfp            = 110,
	.hbp            = 220,
	.hsw            = 40,
	.vfp            = 5,
	.vbp            = 20,
	.vsw            = 5,
};

static struct hdmi_reg_data  hdmi_tpi_audio_config_data[] = {
	/* Transmitter is brought to Full operation when value of power
	 * state register is 0x0 */
	{ HDMI_TPI_POWER_STATE_CTRL_REG, TPI_AVI_POWER_STATE_D0		 },
	/* TMDS output lines active. bit 3 1:TMDS inactive, 0: TMDS active */
	{ HDMI_SYS_CTRL_DATA_REG,  0x01 				 },
	/*HDCP Enable - Disable */
	{ HDMI_TPI_HDCP_CONTROLDATA_REG, 0 				 },
	/* I2S mode , Mute Enabled , PCM */
	{ HDMI_TPI_AUDIO_CONFIG_BYTE2_REG, TPI_AUDIO_INTERFACE_I2S |
					    TPI_AUDIO_MUTE_ENABLE |
					    TPI_AUDIO_CODING_PCM	 },
	/* I2S Input configuration register */
	{ HDMI_TPI_I2S_INPUT_CONFIG_REG, TPI_I2S_SCK_EDGE_RISING |
					TPI_I2S_MCLK_MULTIPLIER_256 |
					TPI_I2S_WS_POLARITY_HIGH |
					TPI_I2S_SD_JUSTIFY_LEFT |
					TPI_I2S_SD_DIRECTION_MSB_FIRST |
					TPI_I2S_FIRST_BIT_SHIFT_YES	 },
	/* I2S Enable ad Mapping Register */
	{ HDMI_TPI_I2S_ENABLE_MAPPING_REG, TPI_I2S_SD_CHANNEL_ENABLE |
					    TPI_I2S_SD_FIFO_0 |
					    TPI_I2S_DOWNSAMPLE_DISABLE |
					    TPI_I2S_LF_RT_SWAP_NO |
					    TPI_I2S_SD_CONFIG_SELECT_SD0 },
	{ HDMI_TPI_I2S_ENABLE_MAPPING_REG, TPI_I2S_SD_CHANNEL_DISABLE |
					    TPI_I2S_SD_FIFO_1 |
					    TPI_I2S_DOWNSAMPLE_DISABLE |
					    TPI_I2S_LF_RT_SWAP_NO |
					    TPI_I2S_SD_CONFIG_SELECT_SD1 },
	{ HDMI_TPI_I2S_ENABLE_MAPPING_REG, TPI_I2S_SD_CHANNEL_DISABLE |
					    TPI_I2S_SD_FIFO_2 |
					    TPI_I2S_DOWNSAMPLE_DISABLE |
					    TPI_I2S_LF_RT_SWAP_NO |
					    TPI_I2S_SD_CONFIG_SELECT_SD2 },
	{ HDMI_TPI_I2S_ENABLE_MAPPING_REG, TPI_I2S_SD_CHANNEL_DISABLE |
					    TPI_I2S_SD_FIFO_3 |
					    TPI_I2S_DOWNSAMPLE_DISABLE |
					    TPI_I2S_LF_RT_SWAP_NO |
					    TPI_I2S_SD_CONFIG_SELECT_SD3 },
	{ HDMI_TPI_AUDIO_CONFIG_BYTE3_REG, TPI_AUDIO_SAMPLE_SIZE_16 |
					     TPI_AUDIO_FREQ_44KHZ |
					     TPI_AUDIO_2_CHANNEL	 },
	/* Speaker Configuration  refer CEA Specification*/
	{ HDMI_TPI_AUDIO_CONFIG_BYTE4_REG, (0x0 << 0)},
	/* Stream Header Settings */
	{ HDMI_TPI_I2S_STRM_HDR_0_REG, I2S_CHAN_STATUS_MODE		 },
	{ HDMI_TPI_I2S_STRM_HDR_1_REG, I2S_CHAN_STATUS_CAT_CODE	},
	{ HDMI_TPI_I2S_STRM_HDR_2_REG, I2S_CHAN_SOURCE_CHANNEL_NUM	 },
	{ HDMI_TPI_I2S_STRM_HDR_3_REG, I2S_CHAN_ACCURACY_N_44_SAMPLING_FS },
	{ HDMI_TPI_I2S_STRM_HDR_4_REG, I2S_CHAN_ORIGIN_FS_N_SAMP_LENGTH  },
	/*     Infoframe data Select  */
	{ HDMI_CPI_MISC_IF_SELECT_REG, HDMI_INFOFRAME_TX_ENABLE |
					HDMI_INFOFRAME_TX_REPEAT |
					HDMI_AUDIO_INFOFRAME		 },
};

static u8 misc_audio_info_frame_data[] = {
	MISC_INFOFRAME_TYPE | MISC_INFOFRAME_ALWAYS_SET,
	MISC_INFOFRAME_VERSION,
	MISC_INFOFRAME_LENGTH,
	0,				/* Checksum byte*/
	HDMI_SH_PCM | HDMI_SH_TWO_CHANNELS,
	HDMI_SH_44KHz | HDMI_SH_16BIT,	/* 44.1 KHz*/
	0x0,   /* Default 0*/
	HDMI_SH_SPKR_FLFR,
	HDMI_SH_0dB_ATUN | 0x1,		/* 0 dB  Attenuation*/
	0x0,
	0x0,
	0x0,
	0x0,
	0x0
};

static u8 avi_info_frame_data[] = {
	0x00,
	0x00,
	0xA8,
	0x00,
	0x04,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00
};

void get_horz_vert_timing_info(struct i2c_client *client, u8 *edid)
{
	/*HORIZONTAL FRONT PORCH */
	hdmi_timings.hfp = edid[current_descriptor_addrs + 8];
	/*HORIZONTAL SYNC WIDTH */
	hdmi_timings.hsw = edid[current_descriptor_addrs + 9];
	/*HORIZONTAL BACK PORCH */
	hdmi_timings.hbp = (((edid[current_descriptor_addrs + 4]
					  & 0x0F) << 8) |
					edid[current_descriptor_addrs + 3]) -
		(hdmi_timings.hfp + hdmi_timings.hsw);
	/*VERTICAL FRONT PORCH */
	hdmi_timings.vfp = ((edid[current_descriptor_addrs + 10] &
				       0xF0) >> 4);
	/*VERTICAL SYNC WIDTH */
	hdmi_timings.vsw = (edid[current_descriptor_addrs + 10] &
				      0x0F);
	/*VERTICAL BACK PORCH */
	hdmi_timings.vbp = (((edid[current_descriptor_addrs + 7] &
					0x0F) << 8) |
				      edid[current_descriptor_addrs + 6]) -
		(hdmi_timings.vfp + hdmi_timings.vsw);

	dev_dbg(&client->dev, "<%s> \n"
				       "hfp			= %d\n"
				       "hsw			= %d\n"
				       "hbp			= %d\n"
				       "vfp			= %d\n"
				       "vsw			= %d\n"
				       "vbp			= %d\n",
		 __func__,
		 hdmi_timings.hfp,
		 hdmi_timings.hsw,
		 hdmi_timings.hbp,
		 hdmi_timings.vfp,
		 hdmi_timings.vsw,
		 hdmi_timings.vbp
		 );

}

void get_edid_timing_data(struct i2c_client *client, u8 *edid, u16 *pixel_clk,
			  u16 *horizontal_res, u16 *vertical_res)
{
	u8 offset, effective_addrs;
	u8 count;
	u8 i;
	u8 flag = false;
	/*check for 720P timing in block0 */
	for (count = 0; count < EDID_SIZE_BLOCK0_TIMING_DESCRIPTOR; count++) {
		current_descriptor_addrs =
			EDID_DESCRIPTOR_BLOCK0_ADDRESS +
			count * EDID_TIMING_DESCRIPTOR_SIZE;
		*horizontal_res =
			(((edid[EDID_DESCRIPTOR_BLOCK0_ADDRESS + 4 +
			   count * EDID_TIMING_DESCRIPTOR_SIZE] & 0xF0) << 4) |
			 edid[EDID_DESCRIPTOR_BLOCK0_ADDRESS + 2 +
			 count * EDID_TIMING_DESCRIPTOR_SIZE]);
		*vertical_res =
			(((edid[EDID_DESCRIPTOR_BLOCK0_ADDRESS + 7 +
			   count * EDID_TIMING_DESCRIPTOR_SIZE] & 0xF0) << 4) |
			 edid[EDID_DESCRIPTOR_BLOCK0_ADDRESS + 5 +
			 count * EDID_TIMING_DESCRIPTOR_SIZE]);

		dev_dbg(&client->dev,
			"<%s> ***Block-0-Timing-descriptor[%d]***\n",
			__func__, count);
		for (i = current_descriptor_addrs;
		      i <
		      (current_descriptor_addrs+EDID_TIMING_DESCRIPTOR_SIZE);
		      i++)
			dev_dbg(&client->dev,
				"%x ==>		%x\n", i, edid[i]);

			dev_dbg(&client->dev,
				 "<%s>\n"
				 "E-EDID Buffer Index	= %d\n"
				 "horizontal_res       	= %d\n"
				 "vertical_res		= %d\n",
				 __func__,
				 current_descriptor_addrs,
				 *horizontal_res,
				 *vertical_res
				 );

		if (*horizontal_res == HDMI_XRES &&
		    *vertical_res == HDMI_YRES) {
			dev_info(&client->dev, "<%s>\nFound EDID Data "
						       "for %d x %dp\n",
				 __func__, *horizontal_res, *vertical_res);
			flag = true;
			break;
			}
	}

	/*check for the Timing in block1 */
	if (flag != true) {
		offset = edid[EDID_DESCRIPTOR_BLOCK1_ADDRESS + 2];
		if (offset != 0) {
			effective_addrs = EDID_DESCRIPTOR_BLOCK1_ADDRESS
				+ offset;
			/*to determine the number of descriptor blocks */
			for (count = 0;
			      count < EDID_SIZE_BLOCK1_TIMING_DESCRIPTOR;
			      count++) {
				current_descriptor_addrs = effective_addrs +
					count * EDID_TIMING_DESCRIPTOR_SIZE;
				*horizontal_res =
					(((edid[effective_addrs + 4 +
					   count*EDID_TIMING_DESCRIPTOR_SIZE] &
					   0xF0) << 4) |
					 edid[effective_addrs + 2 +
					 count * EDID_TIMING_DESCRIPTOR_SIZE]);
				*vertical_res =
					(((edid[effective_addrs + 7 +
					   count*EDID_TIMING_DESCRIPTOR_SIZE] &
					   0xF0) << 4) |
					 edid[effective_addrs + 5 +
					 count * EDID_TIMING_DESCRIPTOR_SIZE]);

				dev_dbg(&client->dev,
					 "<%s> Block1-Timing-descriptor[%d]\n",
					 __func__, count);

				for (i = current_descriptor_addrs;
				      i < (current_descriptor_addrs+
					   EDID_TIMING_DESCRIPTOR_SIZE); i++)
					dev_dbg(&client->dev,
						"%x ==> 	%x\n",
						   i, edid[i]);

				dev_dbg(&client->dev, "<%s>\n"
						"current_descriptor	= %d\n"
						"horizontal_res		= %d\n"
						"vertical_res 		= %d\n",
					 __func__, current_descriptor_addrs,
					 *horizontal_res, *vertical_res);

				if (*horizontal_res == HDMI_XRES &&
				    *vertical_res == HDMI_YRES) {
					dev_info(&client->dev,
						 "<%s> Found EDID Data for "
						 "%d x %dp\n",
						 __func__,
						 *horizontal_res,
						 *vertical_res
						 );
					flag = true;
					break;
					}
			}
		}
	}

	if (flag == true) {
		*pixel_clk = ((edid[current_descriptor_addrs + 1] << 8) |
			     edid[current_descriptor_addrs]);

		hdmi_timings.x_res = *horizontal_res;
		hdmi_timings.y_res = *vertical_res;
		hdmi_timings.pixel_clock = *pixel_clk*10;
		dev_dbg(&client->dev,
			 "EDID TIMING DATA supported by zoom2 FOUND\n"
			 "EDID DTD block address	= %d\n"
			 "pixel_clk 			= %d\n"
			 "horizontal res		= %d\n"
			 "vertical res			= %d\n",
			 current_descriptor_addrs,
			 hdmi_timings.pixel_clock,
			 hdmi_timings.x_res,
			 hdmi_timings.y_res
			 );

		get_horz_vert_timing_info(client, edid);
	} else {

		dev_info(&client->dev,
			 "<%s>\n"
			 "EDID TIMING DATA supported by zoom2 NOT FOUND\n"
			 "setting default timing values for 720p\n"
			 "pixel_clk 		= %d\n"
			 "horizontal res	= %d\n"
			 "vertical res		= %d\n",
			 __func__,
			 hdmi_timings.pixel_clock,
			 hdmi_timings.x_res,
			 hdmi_timings.y_res
			 );

		*pixel_clk = hdmi_timings.pixel_clock;
		*horizontal_res = hdmi_timings.x_res;
		*vertical_res = hdmi_timings.y_res;
	}


}

/* Write a value to a register in sil9022 device.
 * @client: i2c driver client structure.
 * @reg: Address of the register to read value from.
 * @val: Value to be written to a specific register.
 * Returns zero if successful, or non-zero otherwise.
 */
static int
sil9022_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int err = 0;
	struct i2c_msg msg[1];
	u8 data[2];
	int retries = 0;

	if (!client->adapter) {
		dev_err(&client->dev, "<%s> ERROR: No HDMI Device\n", __func__);
		return -ENODEV;
	}

retry:
	msg->addr = client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 2;
	msg->buf = data;

	data[0] = reg;
	data[1] = val;

	err = i2c_transfer(client->adapter, msg, 1);
	dev_dbg(&client->dev, "<%s> i2c write at=%x "
			       "val=%x flags=%d err=%d\n",
		__func__, data[0], data[1], msg->flags, err);
	udelay(50);

	if (err >= 0)
		return 0;

	dev_err(&client->dev, "<%s> ERROR: i2c write at=%x "
			       "val=%x flags=%d err=%d\n",
		__func__, data[0], data[1], msg->flags, err);
	if (retries <= 5) {
		dev_info(&client->dev, "Retrying I2C... %d\n", retries);
		retries++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
		goto retry;
	}
	return err;
}

/*
 * Read a value from a register in sil9022 device.
 * The value is returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */
static int sil9022_read_reg(struct i2c_client *client, u16 data_length, u8 reg, u8 *val)
{
	int err = 0;
	struct i2c_msg msg[1];
	u8 data[2];

	if (!client->adapter) {
		dev_err(&client->dev, "<%s> ERROR: No HDMI Device\n", __func__);
		return -ENODEV;
	}

	msg->addr = client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);
	dev_dbg(&client->dev, "<%s> i2c Read1 reg=%x val=%d "
			       "flags=%d err=%d\n",
		__func__, reg, data[1], msg->flags, err);

	if (err >= 0) {
		mdelay(3);
		msg->flags = I2C_M_RD;
		msg->len = data_length;
		err = i2c_transfer(client->adapter, msg, 1);
	}

	if (err >= 0) {
		*val = 0;
		if (data_length == 1)
			*val = data[0];
		else if (data_length == 2)
			*val = data[1] + (data[0] << 8);
		dev_dbg(&client->dev, "<%s> i2c Read2 at 0x%x, *val=%d "
				       "flags=%d err=%d\n",
			 __func__, reg, *val, msg->flags, err);
		return 0;
	}

	dev_err(&client->dev, "<%s> ERROR: i2c Read at 0x%x, "
			      "*val=%d flags=%d err=%d\n",
		__func__, reg, *val, msg->flags, err);
	return err;
}

static int sil9022_blockwrite_reg(struct i2c_client *client, u8 reg, u16 alength, u8 *val, u16 *out_len)
{
	int err = 0, i;
	struct i2c_msg msg[1];
	u8 data[2];

	if (!client->adapter) {
		dev_err(&client->dev, "<%s> ERROR: No HDMI Device\n", __func__);
		return -ENODEV;
	}

	msg->addr = client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 2;
	msg->buf = data;

	/* high byte goes out first */
	data[0] = reg >> 8;

	for (i = 0; i < alength - 1; i++) {
		data[1] = val[i];
		err = i2c_transfer(client->adapter, msg, 1);
		udelay(50);
		dev_dbg(&client->dev, "<%s> i2c Block write at 0x%x, "
				      "*val=%d flags=%d byte[%d] err=%d\n",
			__func__, data[0], data[1], msg->flags, i, err);
		if (err < 0)
			break;
	}
	/* set the number of bytes written*/
	*out_len = i;

	if (err < 0) {
		dev_err(&client->dev, "<%s> ERROR:  i2c Block Write at 0x%x, "
				      "*val=%d flags=%d bytes written=%d "
				      "err=%d\n",
			__func__, data[0], data[1], msg->flags, i, err);
		return err;
	}
	return 0;
}

static int sil9022_blockread_reg(struct i2c_client *client, u16 data_length, u16 alength, u8 reg, u8 *val, u16 *out_len)
{
	int err = 0, i;
	struct i2c_msg msg[1];
	u8 data[2];

	if (!client->adapter) {
		dev_err(&client->dev, "<%s> ERROR: No HDMI Device\n", __func__);
		return -ENODEV;
	}

	msg->addr = client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 1;
	msg->buf = data;

	/* High byte goes out first */
	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);
	dev_dbg(&client->dev, "<%s> i2c Block Read1 at 0x%x, "
			       "*val=%d flags=%d err=%d\n",
		 __func__, data[0], data[1], msg->flags, err);

	for (i = 0; i < alength; i++) {
		if (err >= 0) {
			mdelay(3);
			msg->flags = I2C_M_RD;
			msg->len = data_length;
			err = i2c_transfer(client->adapter, msg, 1);
		} else
			break;
		if (err >= 0) {
			val[i] = 0;
			/* High byte comes first */
			if (data_length == 1)
				val[i] = data[0];
			else if (data_length == 2)
				val[i] = data[1] + (data[0] << 8);
			dev_dbg(&client->dev, "<%s> i2c Block Read2 at 0x%x, "
					       "*val=%d flags=%d byte=%d "
					       "err=%d\n",
				 __func__, reg, val[i], msg->flags, i, err);
		} else
			break;
	}
	*out_len = i;
	dev_dbg(&client->dev, "<%s> i2c Block Read at 0x%x, bytes read = %d\n",
		__func__, client->addr, *out_len);

	if (err < 0) {
		dev_err(&client->dev, "<%s> ERROR:  i2c Read at 0x%x, "
				      "*val=%d flags=%d bytes read=%d err=%d\n",
			__func__, reg, *val, msg->flags, i, err);
		return err;
	}
	return 0;
}

static int hdmi_read_edid(struct i2c_client *client, u16 len, char *p_buffer, u16 *out_len)
{
	int err =  0;
	u8 val = 0;
	int retries = 0;
	int i = 0;
	int k = 0;

	len = (len < HDMI_EDID_MAX_LENGTH) ? len : HDMI_EDID_MAX_LENGTH;

	/* Request DDC bus access to read EDID info from HDTV */
	dev_info(&client->dev, "<%s> Reading HDMI EDID\n", __func__);

	/* Bring transmitter to low-Power state */
	val = TPI_AVI_POWER_STATE_D2;
	err = sil9022_write_reg(client, HDMI_TPI_DEVICE_POWER_STATE_DATA, val);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: Failed during bring power state - low.\n",
			 __func__);
		return err;
	}

	/* Disable TMDS clock */
	val = 0x11;
	err = sil9022_write_reg(client, HDMI_SYS_CTRL_DATA_REG, val);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: Failed during bring power state - low.\n",
			 __func__);
		return err;
	}

	val = 0;
	/* Read TPI system control register*/
	err = sil9022_read_reg(client, 1, HDMI_SYS_CTRL_DATA_REG, &val);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: Reading DDC BUS REQUEST\n", __func__);
		return err;
	}

	/* The host writes 0x1A[2]=1 to request the
	 * DDC(Display Data Channel) bus
	 */
	val |= TPI_SYS_CTRL_DDC_BUS_REQUEST;
	err = sil9022_write_reg(client, HDMI_SYS_CTRL_DATA_REG, val);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: Writing DDC BUS REQUEST\n", __func__);
		return err;
	}

	 /*  Poll for bus DDC Bus control to be granted */
	dev_info(&client->dev, "<%s> Poll for DDC bus access\n", __func__);
	val = 0;
	do {
		err = sil9022_read_reg(client, 1, HDMI_SYS_CTRL_DATA_REG, &val);
		if (retries++ > 100)
			return err;

	} while ((val & TPI_SYS_CTRL_DDC_BUS_GRANTED) == 0);

	/*  Close the switch to the DDC */
	val |= TPI_SYS_CTRL_DDC_BUS_REQUEST | TPI_SYS_CTRL_DDC_BUS_GRANTED;
	err = sil9022_write_reg(client, HDMI_SYS_CTRL_DATA_REG, val);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: Close switch to DDC BUS REQUEST\n",
			__func__);
		return err;
	}

	memset(p_buffer, 0, len);
	/* change I2C SetSlaveAddress to HDMI_I2C_MONITOR_ADDRESS */
	/*  Read the EDID structure from the monitor I2C address  */
	client->addr = HDMI_I2C_MONITOR_ADDRESS;
	err = sil9022_blockread_reg(client, 1, len,
				    0x00, p_buffer, out_len);
	if (err < 0 || *out_len <= 0) {
		dev_err(&client->dev,
			"<%s> ERROR: Reading EDID from "
			"HDMI_I2C_MONITOR_ADDRESS\n", __func__);
		return err;
	}

	for (i = 0; i < *out_len; i++) {
		if ((i / 18) < 3) {
			dev_dbg(&client->dev, "byte->%02x	%x\n",
				i, p_buffer[i]);
			continue;
		}
		if ((i/18 >= 3 && i/18 <= 6) && (i%18 == 0))
			dev_dbg(&client->dev, "\n DTD Block %d\n", k++);

		if ((i/18 == 7) && (i%18 == 0))
			dev_dbg(&client->dev, "\n");

		dev_dbg(&client->dev, "byte->%02x	%x\n", i, p_buffer[i]);
	}

	/* Release DDC bus access */
	client->addr = SI9022_I2CSLAVEADDRESS;
	val &= ~(TPI_SYS_CTRL_DDC_BUS_REQUEST | TPI_SYS_CTRL_DDC_BUS_GRANTED);
	err = sil9022_write_reg(client, HDMI_SYS_CTRL_DATA_REG, val);

	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: Releasing DDC  Bus Access\n",
			__func__);
		return err;
	}

	/*  Success */
	return 0;
}

static int hdmi_enable_audio(struct i2c_client *client)
{
	int err = 0;
	u8  val = 0;
	u8  crc = 0;
	u32 count = 0;
	int index = 0;

	for (index = 0;
	      index < sizeof(hdmi_tpi_audio_config_data) /
	      sizeof(struct hdmi_reg_data);
	      index++) {
		err = sil9022_write_reg(
			client,
			hdmi_tpi_audio_config_data[index].reg_offset,
			hdmi_tpi_audio_config_data[index].value);
		if (err != 0) {
			dev_err(&client->dev,
				"<%s> ERROR: Writing "
				"tpi_audio_config_data[%d]={ %d, %d }\n",
				__func__, index,
				hdmi_tpi_audio_config_data[index].reg_offset,
				hdmi_tpi_audio_config_data[index].value);
			return err;
			}
		}

	/* Fill the checksum byte for Infoframe data*/
	count = 0;
	while (count < MISC_INFOFRAME_SIZE_MEMORY) {
		crc += misc_audio_info_frame_data[count];
		count++;
	}
	crc = 0x100 - crc;

	/* Fill CRC Byte*/
	misc_audio_info_frame_data[0x3] = crc;

	for (count = 0; count < MISC_INFOFRAME_SIZE_MEMORY; count++) {
		err = sil9022_write_reg(client,
					(HDMI_CPI_MISC_IF_OFFSET + count),
					misc_audio_info_frame_data[count]);
		if (err < 0) {
			dev_err(&client->dev,
				"<%s> ERROR: writing audio info frame"
				" CRC data: %d\n", __func__, count);
			return err;
		}
	}

	/* Decode Level 0 Packets */
	val = 0x2;
	sil9022_write_reg(client, 0xBC, val);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: writing level 0 packets to 0xBC\n",
			__func__);
		return err;
	}

	val = 0x24;
	err = sil9022_write_reg(client, 0xBD, val);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: writing level 0 packets to 0xBD\n",
			__func__);
		return err;
	}

	val = 0x2;
	err = sil9022_write_reg(client, 0xBE, val);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: writing level 0 packets to 0xBE\n",
			__func__);
		return err;
	}

	/* Disable Mute */
	val = TPI_AUDIO_INTERFACE_I2S |
		  TPI_AUDIO_MUTE_DISABLE |
		  TPI_AUDIO_CODING_PCM;
	err = sil9022_write_reg(client, HDMI_TPI_AUDIO_CONFIG_BYTE2_REG, val);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: Disabling mute\n",
			__func__);
		return err;
	}

	dev_info(&client->dev, "<%s> hdmi audio enabled\n",
		__func__);
	return 0;

}

static int hdmi_disable_audio(struct i2c_client *client)
{
	u8 val = 0;
	int err = 0;
	/* Disable Audio */
	val = TPI_AUDIO_INTERFACE_DISABLE;
	err = sil9022_write_reg(client, HDMI_TPI_AUDIO_CONFIG_BYTE2_REG, val);
	if (err < 0)
		dev_err(&client->dev,
			"<%s> ERROR: Disisable audio interface", __func__);

	dev_info(&client->dev, "<%s> hdmi audio disabled\n", __func__);
	return err;
}

static int hdmi_enable(struct i2c_client *client)
{
	int		err;
	u8		val, vals[14];
	int		i;
	u16		out_len = 0;
	u8		edid[HDMI_EDID_MAX_LENGTH];
	u16		horizontal_res;
	u16		vertical_res;
	u16		pixel_clk;

	memset(edid, 0, HDMI_EDID_MAX_LENGTH);
	memset(vals, 0, 14);

	err = hdmi_read_edid(client, HDMI_EDID_MAX_LENGTH, edid, &out_len);
	if (err < 0 || out_len == 0) {
		dev_err(&client->dev,
			"<%s> Unable to read EDID for monitor\n", __func__);
		return err;
	}

	get_edid_timing_data(client,
			     edid,
			     &pixel_clk,
			     &horizontal_res,
			     &vertical_res
			     );

	/*  Fill the TPI Video Mode Data structure */
	vals[0] = (pixel_clk & 0xFF);                  /* Pixel clock */
	vals[1] = ((pixel_clk & 0xFF00) >> 8);
	vals[2] = VERTICAL_FREQ;                    /* Vertical freq */
	vals[3] = 0x00;
	vals[4] = (horizontal_res & 0xFF);         /* Horizontal pixels*/
	vals[5] = ((horizontal_res & 0xFF00) >> 8);
	vals[6] = (vertical_res & 0xFF);           /* Vertical pixels */
	vals[7] = ((vertical_res & 0xFF00) >> 8);


	dev_info(&client->dev, "<%s>\nHDMI Monitor E-EDID Timing Data\n"
				       "horizontal_res 	= %d\n"
				       "vertical_res	= %d\n"
				       "pixel_clk	= %d\n"
				       "hfp 		= %d\n"
				       "hsw 		= %d\n"
				       "hbp 		= %d\n"
				       "vfp 		= %d\n"
				       "vsw 		= %d\n"
				       "vbp 		= %d\n",
		 __func__,
		 hdmi_timings.x_res,
		 hdmi_timings.y_res,
		 hdmi_timings.pixel_clock,
		 hdmi_timings.hfp,
		 hdmi_timings.hsw,
		 hdmi_timings.hbp,
		 hdmi_timings.vfp,
		 hdmi_timings.vsw,
		 hdmi_timings.vbp
		 );

	/*  Write out the TPI Video Mode Data */
	out_len = 0;
	err = sil9022_blockwrite_reg(client,
				     HDMI_TPI_VIDEO_DATA_BASE_REG,
				     8, vals, &out_len);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: writing TPI video mode data\n", __func__);
		return err;
	}

	/* Write out the TPI Pixel Repetition Data (24 bit wide bus,
	falling edge, no pixel replication) */
	val = TPI_AVI_PIXEL_REP_BUS_24BIT |
		TPI_AVI_PIXEL_REP_FALLING_EDGE |
		TPI_AVI_PIXEL_REP_NONE;
	err = sil9022_write_reg(client,
				HDMI_TPI_PIXEL_REPETITION_REG,
				val);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: writing TPI pixel repetition data\n",
			__func__);
		return err;
	}

	 /*  Write out the TPI AVI Input Format */
	val = TPI_AVI_INPUT_BITMODE_8BIT |
		TPI_AVI_INPUT_RANGE_AUTO |
		TPI_AVI_INPUT_COLORSPACE_RGB;
	err = sil9022_write_reg(client,
				HDMI_TPI_AVI_IN_FORMAT_REG,
				val);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: writing TPI AVI Input format\n", __func__);
		return err;
	}

	/*  Write out the TPI AVI Output Format */
	val = TPI_AVI_OUTPUT_CONV_BT709 |
		TPI_AVI_OUTPUT_RANGE_AUTO |
		TPI_AVI_OUTPUT_COLORSPACE_RGBHDMI;
	err = sil9022_write_reg(client,
				HDMI_TPI_AVI_OUT_FORMAT_REG, val);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: writing TPI AVI output format\n",
			__func__);
		return err;
	}

	/* Write out the TPI System Control Data to power down */
	val = TPI_SYS_CTRL_POWER_DOWN;
	err = sil9022_write_reg(client, HDMI_SYS_CTRL_DATA_REG, val);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: writing TPI power down control data\n",
			__func__);
		return err;
	}

	/* Write out the TPI AVI InfoFrame Data (all defaults) */
	/* Compute CRC*/
	val = 0x82 + 0x02 + 13;

	for (i = 0; i < sizeof(avi_info_frame_data); i++)
		val += avi_info_frame_data[i];

	avi_info_frame_data[0] = 0x100 - val;

	out_len = 0;
	err = sil9022_blockwrite_reg(client,
				     HDMI_TPI_AVI_DBYTE_BASE_REG,
				     sizeof(avi_info_frame_data),
				     avi_info_frame_data, &out_len);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: writing TPI AVI infoframe data\n",
			__func__);
		return err;
	}

	/*  Audio Configuration  */
	err = hdmi_enable_audio(client);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: Unable set audio configuration\n",
			__func__);
		return err;
	}

	/*  Write out the TPI Device Power State (D0) */
	val = TPI_AVI_POWER_STATE_D0;
	err = sil9022_write_reg(client,
				HDMI_TPI_POWER_STATE_CTRL_REG, val);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: Setting device power state to D0\n",
			__func__);
		return err;
	}

	/* Write out the TPI System Control Data to power up and
	 * select output mode
	 */
	val = TPI_SYS_CTRL_POWER_ACTIVE | TPI_SYS_CTRL_OUTPUT_MODE_HDMI;
	err = sil9022_write_reg(client, HDMI_SYS_CTRL_DATA_REG, val);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: Writing system control data\n", __func__);
		return err;
	}

	/*  Read back TPI System Control Data to latch settings */
	msleep(10);
	err = sil9022_read_reg(client, 1, HDMI_SYS_CTRL_DATA_REG, &val);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: Writing system control data\n",
			__func__);
		return err;
	}

	/* HDCP Enable - Disable */
	val = 0;
	err = sil9022_write_reg(client,
				HDMI_TPI_HDCP_CONTROLDATA_REG, val);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: Enable (1) / Disable (0) => HDCP: %d\n",
			__func__, val);
		return err;
	}

	dev_info(&client->dev, "<%s> hdmi enabled\n", __func__);

	return 0;

}

static int hdmi_disable(struct i2c_client *client)
{
	u8 val = 0;
	int err = 0;

	err = hdmi_disable_audio(client);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: failed to disable audio\n", __func__);
		return err;
	}

	/*  Write out the TPI System Control Data to power down  */
	val = TPI_SYS_CTRL_POWER_DOWN;
	err = sil9022_write_reg(client, HDMI_SYS_CTRL_DATA_REG, val);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: writing control data - power down\n",
			__func__);
		return err;
	}

	/*  Write out the TPI Device Power State (D2) */
	val = TPI_AVI_POWER_STATE_D2;
	err = sil9022_write_reg(client,
			  HDMI_TPI_DEVICE_POWER_STATE_DATA, val);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: Setting device power state to D2\n",
			__func__);
		return err;
	}

	/*  Read back TPI System Control Data to latch settings */
	mdelay(10);
	err = sil9022_read_reg(client, 1, HDMI_SYS_CTRL_DATA_REG, &val);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR:  Reading System control data "
			"- latch settings\n", __func__);
		return err;
	}

	dev_info(&client->dev, "<%s> hdmi disabled\n", __func__);

	return 0;

}

static int sil9022_set_cm_clkout_ctrl(struct i2c_client *client)
{
	int err;
	u8 ver;
	// u32 clkout_ctrl = 0;

	/* from TRM*/
	/* intitialize the CM_CLKOUT_CTRL register*/
	/* 	clkout_ctrl = CLKOUT2_EN |	/\* sys_clkout2 is enabled*\/ */
	/* 		      CLKOUT2_DIV |	/\* sys_clkout2 / 16*\/ */
	/* 		      CLKOUT2SOURCE;	/\* CM_96M_FCLK *\/ */

	// omap_writel(clkout_ctrl, CM_CLKOUT_CTRL); /*CM_CLKOUT_CTRL*/

	/* probe for sil9022 chip version*/
	err = sil9022_write_reg(client, SI9022_REG_TPI_RQB, 0x00);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: Writing HDMI configuration to "
			"reg - SI9022_REG_TPI_RQB\n", __func__);
		err = -ENODEV;
		goto out;
	}

	err = sil9022_read_reg(client, 1, SI9022_REG_CHIPID0, &ver);
	if (err < 0) {
		dev_err(&client->dev,
			"<%s> ERROR: Reading HDMI version Id\n", __func__);
		err = -ENODEV;
		goto out;
	} else if (ver != SI9022_CHIPID_902x) {
		dev_err(&client->dev,
			"<%s> Not a valid verId: 0x%x \n", __func__, ver);
		err = -ENODEV;
		goto out;
	} else
		dev_info(&client->dev,
			 "<%s> sil9022 HDMI Chip version = %x \n",
			 __func__, ver);

out:
	return err;
}

static int __devinit sil9022_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;

	ret = sil9022_set_cm_clkout_ctrl(client);
	if (ret)
		goto out;

	/* Enable sil9022.  */
	ret = hdmi_enable(client);
	if (ret)
		goto out;

	/* wait couple of vsyncs until enabling the LCD */
	msleep(50);

out:
	return ret;
}

static int sil9022_remove(struct i2c_client *client)
{
	if (!client->adapter) {
		dev_err(&client->dev, "<%s> No HDMI Device \n",
			__func__);
		return -ENODEV;
	}

	hdmi_disable(client);

	return 0;
}

static const struct i2c_device_id sil9022_id[] = {
	{ SIL9022_DRV_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sil9022_id);

static struct i2c_driver sil9022_driver = {
	.driver = {
		.name  = SIL9022_DRV_NAME,
		.owner = THIS_MODULE,
		},
	.probe		= sil9022_probe,
	.remove		= __devexit_p(sil9022_remove),
	.id_table 	= sil9022_id,
};

static int __init sil9022_init(void)
{
	if (i2c_add_driver(&sil9022_driver) < 0) {
		printk(KERN_ERR "<%s> Driver registration failed\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static void __exit sil9022_exit(void)
{
	i2c_del_driver(&sil9022_driver);
}

late_initcall(sil9022_init);
module_exit(sil9022_exit);

MODULE_AUTHOR("Giuseppe Calderaro - <giuseppe.calderaro@arm.com>");
MODULE_DESCRIPTION("SIL9022 HDMI Driver");
MODULE_LICENSE("GPL");
