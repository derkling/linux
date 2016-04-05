/*
 *  stmvl53l0_module.c - Linux kernel modules for STM VL53L0 FlightSense TOF
 *						 sensor
 *
 *  Copyright (C) 2016 STMicroelectronics Imaging Division.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/kobject.h>
/*
 * API includes
 */
#include "vl53l0_api.h"
#include "vl53l010_api.h"
/*
#include "vl53l0_def.h"
#include "vl53l0_platform.h"
#include "stmvl53l0-i2c.h"
#include "stmvl53l0-cci.h"
#include "stmvl53l0.h"
*/

#define USE_INT

#define HTC_MODIFY
#define HTC
#ifdef HTC
#define OFFSET_CALI_TARGET_DISTANCE	     100 // Target: 100 mm
#define RANGE_MEASUREMENT_TIMES		     10
#define RANGE_MEASUREMENT_RETRY_TIMES	     13
#define RANGE_MEASUREMENT_OVERFLOW	     8100
#define VL53L0_MAGIC 			     'A'
#define VL53L0_IOCTL_GET_DATA		     _IOR(VL53L0_MAGIC, 0x01, \
						VL53L0_RangingMeasurementData_t)
#define VL53L0_IOCTL_OFFSET_CALI	     _IOR(VL53L0_MAGIC, 0x02, int)
#define VL53L0_IOCTL_XTALK_CALI		     _IOR(VL53L0_MAGIC, 0x03, int)
#define VL53L0_IOCTL_SET_XTALK_CALI_DISTANCE _IOR(VL53L0_MAGIC, 0x04, int)

static int g_offsetMicroMeter = 0;
static FixPoint1616_t g_XTalkCompensationRateMegaCps = 0;
struct timeval start_tv, stop_tv;
#endif

/*
 * Global data
 */
#ifdef CAMERA_CCI
static struct stmvl53l0_module_fn_t stmvl53l0_module_func_tbl = {
	.init = stmvl53l0_init_cci,
	.deinit = stmvl53l0_exit_cci,
	.power_up = stmvl53l0_power_up_cci,
	.power_down = stmvl53l0_power_down_cci,
};
#else
static struct stmvl53l0_module_fn_t stmvl53l0_module_func_tbl = {
	.init = stmvl53l0_init_i2c,
	.deinit = stmvl53l0_exit_i2c,
	.power_up = stmvl53l0_power_up_i2c,
	.power_down = stmvl53l0_power_down_i2c,
};
#endif
struct stmvl53l0_module_fn_t *pmodule_func_tbl;

struct stmvl53l0_api_fn_t {
	int8_t (*GetVersion)(VL53L0_Version_t *pVersion);
	int8_t (*GetPalSpecVersion)(VL53L0_Version_t *pPalSpecVersion);

	int8_t (*GetProductRevision)(VL53L0_DEV Dev,
					uint8_t *pProductRevisionMajor,
					uint8_t *pProductRevisionMinor);
	int8_t (*GetDeviceInfo)(VL53L0_DEV Dev,
				VL53L0_DeviceInfo_t *pVL53L0_DeviceInfo);
	int8_t (*GetDeviceErrorStatus)(VL53L0_DEV Dev,
				VL53L0_DeviceError *pDeviceErrorStatus);
	int8_t (*GetRangeStatusString)(uint8_t RangeStatus,
				char *pRangeStatusString);
	int8_t (*GetDeviceErrorString)(VL53L0_DeviceError ErrorCode,
				char *pDeviceErrorString);
	int8_t (*GetPalErrorString)(VL53L0_Error PalErrorCode,
				char *pPalErrorString);
	int8_t (*GetPalStateString)(VL53L0_State PalStateCode,
				char *pPalStateString);
	int8_t (*GetPalState)(VL53L0_DEV Dev,	VL53L0_State *pPalState);
	int8_t (*SetPowerMode)(VL53L0_DEV Dev,
				VL53L0_PowerModes PowerMode);
	int8_t (*GetPowerMode)(VL53L0_DEV Dev,
				VL53L0_PowerModes *pPowerMode);
	int8_t (*SetOffsetCalibrationDataMicroMeter)(VL53L0_DEV Dev,
				int32_t OffsetCalibrationDataMicroMeter);
	int8_t (*GetOffsetCalibrationDataMicroMeter)(VL53L0_DEV Dev,
				int32_t *pOffsetCalibrationDataMicroMeter);
	int8_t (*SetLinearityCorrectiveGain)(VL53L0_DEV Dev,
				int16_t LinearityCorrectiveGain);
	int8_t (*GetLinearityCorrectiveGain)(VL53L0_DEV Dev,
				uint16_t *pLinearityCorrectiveGain);
	int8_t (*SetGroupParamHold)(VL53L0_DEV Dev,
				uint8_t GroupParamHold);
	int8_t (*GetUpperLimitMilliMeter)(VL53L0_DEV Dev,
				uint16_t *pUpperLimitMilliMeter);
	int8_t (*SetDeviceAddress)(VL53L0_DEV Dev,
				uint8_t DeviceAddress);
	int8_t (*DataInit)(VL53L0_DEV Dev);
	int8_t (*SetTuningSettingBuffer)(VL53L0_DEV Dev,
				uint8_t *pTuningSettingBuffer,
				uint8_t UseInternalTuningSettings);
	int8_t (*GetTuningSettingBuffer)(VL53L0_DEV Dev,
				uint8_t **pTuningSettingBuffer,
				uint8_t *pUseInternalTuningSettings);
	int8_t (*StaticInit)(VL53L0_DEV Dev);
	int8_t (*WaitDeviceBooted)(VL53L0_DEV Dev);
	int8_t (*ResetDevice)(VL53L0_DEV Dev);
	int8_t (*SetDeviceParameters)(VL53L0_DEV Dev,
				const VL53L0_DeviceParameters_t *pDeviceParameters);
	int8_t (*GetDeviceParameters)(VL53L0_DEV Dev,
				VL53L0_DeviceParameters_t *pDeviceParameters);
	int8_t (*SetDeviceMode)(VL53L0_DEV Dev,
				VL53L0_DeviceModes DeviceMode);
	int8_t (*GetDeviceMode)(VL53L0_DEV Dev,
				VL53L0_DeviceModes *pDeviceMode);
	int8_t (*SetHistogramMode)(VL53L0_DEV Dev,
				VL53L0_HistogramModes HistogramMode);
	int8_t (*GetHistogramMode)(VL53L0_DEV Dev,
				VL53L0_HistogramModes *pHistogramMode);
	int8_t (*SetMeasurementTimingBudgetMicroSeconds)(VL53L0_DEV Dev,
				uint32_t  MeasurementTimingBudgetMicroSeconds);
	int8_t (*GetMeasurementTimingBudgetMicroSeconds)(
				VL53L0_DEV Dev,
				uint32_t *pMeasurementTimingBudgetMicroSeconds);
	int8_t (*GetVcselPulsePeriod)(VL53L0_DEV Dev,
				VL53L0_VcselPeriod VcselPeriodType,
				uint8_t	*pVCSELPulsePeriod);
	int8_t (*SetVcselPulsePeriod)(VL53L0_DEV Dev,
				VL53L0_VcselPeriod VcselPeriodType,
				uint8_t VCSELPulsePeriod);
	int8_t (*SetSequenceStepEnable)(VL53L0_DEV Dev,
				VL53L0_SequenceStepId SequenceStepId,
				uint8_t SequenceStepEnabled);
	int8_t (*GetSequenceStepEnable)(VL53L0_DEV Dev,
				VL53L0_SequenceStepId SequenceStepId,
				uint8_t *pSequenceStepEnabled);
	int8_t (*GetSequenceStepEnables)(VL53L0_DEV Dev,
				VL53L0_SchedulerSequenceSteps_t *pSchedulerSequenceSteps);
	int8_t (*SetSequenceStepTimeout)(VL53L0_DEV Dev,
				VL53L0_SequenceStepId SequenceStepId,
				FixPoint1616_t TimeOutMilliSecs);
	int8_t (*GetSequenceStepTimeout)(VL53L0_DEV Dev,
				VL53L0_SequenceStepId SequenceStepId,
				FixPoint1616_t *pTimeOutMilliSecs);
	int8_t (*GetNumberOfSequenceSteps)(VL53L0_DEV Dev,
				uint8_t *pNumberOfSequenceSteps);
	int8_t (*GetSequenceStepsInfo)(
				VL53L0_SequenceStepId SequenceStepId,
				char *pSequenceStepsString);
	int8_t (*SetInterMeasurementPeriodMilliSeconds)(
				VL53L0_DEV Dev,
				uint32_t InterMeasurementPeriodMilliSeconds);
	int8_t (*GetInterMeasurementPeriodMilliSeconds)(
				VL53L0_DEV Dev,
				uint32_t *pInterMeasurementPeriodMilliSeconds);
	int8_t (*SetXTalkCompensationEnable)(VL53L0_DEV Dev,
				uint8_t XTalkCompensationEnable);
	int8_t (*GetXTalkCompensationEnable)(VL53L0_DEV Dev,
				uint8_t *pXTalkCompensationEnable);
	int8_t (*SetXTalkCompensationRateMegaCps)(
				VL53L0_DEV Dev,
				FixPoint1616_t XTalkCompensationRateMegaCps);
	int8_t (*GetXTalkCompensationRateMegaCps)(
				VL53L0_DEV Dev,
				FixPoint1616_t *pXTalkCompensationRateMegaCps);
	int8_t (*GetNumberOfLimitCheck)(
				uint16_t *pNumberOfLimitCheck);
	int8_t (*GetLimitCheckInfo)(VL53L0_DEV Dev,
				uint16_t LimitCheckId, char *pLimitCheckString);
	int8_t (*SetLimitCheckEnable)(VL53L0_DEV Dev,
				uint16_t LimitCheckId,
				uint8_t LimitCheckEnable);
	int8_t (*GetLimitCheckEnable)(VL53L0_DEV Dev,
				uint16_t LimitCheckId, uint8_t *pLimitCheckEnable);
	int8_t (*SetLimitCheckValue)(VL53L0_DEV Dev,
				uint16_t LimitCheckId,
				FixPoint1616_t LimitCheckValue);
	int8_t (*GetLimitCheckValue)(VL53L0_DEV Dev,
				uint16_t LimitCheckId,
				FixPoint1616_t *pLimitCheckValue);
	int8_t (*GetLimitCheckCurrent)(VL53L0_DEV Dev,
				uint16_t LimitCheckId, FixPoint1616_t *pLimitCheckCurrent);
	int8_t (*SetWrapAroundCheckEnable)(VL53L0_DEV Dev,
				uint8_t WrapAroundCheckEnable);
	int8_t (*GetWrapAroundCheckEnable)(VL53L0_DEV Dev,
				uint8_t *pWrapAroundCheckEnable);
	int8_t (*PerformSingleMeasurement)(VL53L0_DEV Dev);
	int8_t (*PerformRefCalibration)(VL53L0_DEV Dev,
				uint8_t *pVhvSettings, uint8_t *pPhaseCal);
	int8_t (*PerformXTalkCalibration)(VL53L0_DEV Dev,
				FixPoint1616_t XTalkCalDistance,
				FixPoint1616_t *pXTalkCompensationRateMegaCps);
	int8_t (*PerformOffsetCalibration)(VL53L0_DEV Dev,
				FixPoint1616_t CalDistanceMilliMeter,
				int32_t *pOffsetMicroMeter);
	int8_t (*StartMeasurement)(VL53L0_DEV Dev);
	int8_t (*StopMeasurement)(VL53L0_DEV Dev);
	int8_t (*GetMeasurementDataReady)(VL53L0_DEV Dev,
				uint8_t *pMeasurementDataReady);
	int8_t (*WaitDeviceReadyForNewMeasurement)(VL53L0_DEV Dev,
				uint32_t MaxLoop);
	int8_t (*GetRangingMeasurementData)(VL53L0_DEV Dev,
				VL53L0_RangingMeasurementData_t *pRangingMeasurementData);
	int8_t (*GetHistogramMeasurementData)(VL53L0_DEV Dev,
				VL53L0_HistogramMeasurementData_t *pHistogramMeasurementData);
	int8_t (*PerformSingleRangingMeasurement)(VL53L0_DEV Dev,
				VL53L0_RangingMeasurementData_t *pRangingMeasurementData);
	int8_t (*PerformSingleHistogramMeasurement)(VL53L0_DEV Dev,
				VL53L0_HistogramMeasurementData_t *pHistogramMeasurementData);
	int8_t (*SetNumberOfROIZones)(VL53L0_DEV Dev,
				uint8_t NumberOfROIZones);
	int8_t (*GetNumberOfROIZones)(VL53L0_DEV Dev,
				uint8_t *pNumberOfROIZones);
	int8_t (*GetMaxNumberOfROIZones)(VL53L0_DEV Dev,
				uint8_t *pMaxNumberOfROIZones);
	int8_t (*SetGpioConfig)(VL53L0_DEV Dev,
				uint8_t Pin,
				VL53L0_DeviceModes DeviceMode,
				VL53L0_GpioFunctionality Functionality,
				VL53L0_InterruptPolarity Polarity);
	int8_t (*GetGpioConfig)(VL53L0_DEV Dev,
				uint8_t Pin,
				VL53L0_DeviceModes *pDeviceMode,
				VL53L0_GpioFunctionality *pFunctionality,
				VL53L0_InterruptPolarity *pPolarity);
	int8_t (*SetInterruptThresholds)(VL53L0_DEV Dev,
				VL53L0_DeviceModes DeviceMode,
				FixPoint1616_t ThresholdLow,
				FixPoint1616_t ThresholdHigh);
	int8_t (*GetInterruptThresholds)(VL53L0_DEV Dev,
				VL53L0_DeviceModes DeviceMode,
				FixPoint1616_t *pThresholdLow,
				FixPoint1616_t *pThresholdHigh);
	int8_t (*ClearInterruptMask)(VL53L0_DEV Dev,
				uint32_t InterruptMask);
	int8_t (*GetInterruptMaskStatus)(VL53L0_DEV Dev,
				uint32_t *pInterruptMaskStatus);
	int8_t (*EnableInterruptMask)(VL53L0_DEV Dev, uint32_t InterruptMask);
	int8_t (*SetSpadAmbientDamperThreshold)(VL53L0_DEV Dev,
				uint16_t SpadAmbientDamperThreshold);
	int8_t (*GetSpadAmbientDamperThreshold)(VL53L0_DEV Dev,
				uint16_t *pSpadAmbientDamperThreshold);
	int8_t (*SetSpadAmbientDamperFactor)(VL53L0_DEV Dev,
				uint16_t SpadAmbientDamperFactor);
	int8_t (*GetSpadAmbientDamperFactor)(VL53L0_DEV Dev,
				uint16_t *pSpadAmbientDamperFactor);
	int8_t (*PerformRefSpadManagement)(VL53L0_DEV Dev,
				uint32_t *refSpadCount, uint8_t *isApertureSpads);
};

static struct stmvl53l0_api_fn_t stmvl53l0_api_func_tbl = {
	.GetVersion = VL53L0_GetVersion,
	.GetPalSpecVersion = VL53L0_GetPalSpecVersion,
	.GetProductRevision = VL53L0_GetProductRevision,
	.GetDeviceInfo = VL53L0_GetDeviceInfo,
	.GetDeviceErrorStatus = VL53L0_GetDeviceErrorStatus,
	.GetRangeStatusString = VL53L0_GetRangeStatusString,
	.GetDeviceErrorString = VL53L0_GetDeviceErrorString,
	.GetPalErrorString = VL53L0_GetPalErrorString,
	.GetPalState = VL53L0_GetPalState,
	.SetPowerMode = VL53L0_SetPowerMode,
	.GetPowerMode = VL53L0_GetPowerMode,
	.SetOffsetCalibrationDataMicroMeter = VL53L0_SetOffsetCalibrationDataMicroMeter,
	.SetLinearityCorrectiveGain = VL53L0_SetLinearityCorrectiveGain,
	//.GetLinearityCorrectiveGain = VL53L0_GetLinearityCorrectiveGain,
	.GetOffsetCalibrationDataMicroMeter = VL53L0_GetOffsetCalibrationDataMicroMeter,
	.SetGroupParamHold = VL53L0_SetGroupParamHold,
	.GetUpperLimitMilliMeter = VL53L0_GetUpperLimitMilliMeter,
	.SetDeviceAddress = VL53L0_SetDeviceAddress,
	.DataInit = VL53L0_DataInit,
	.SetTuningSettingBuffer = VL53L0_SetTuningSettingBuffer,
	//.GetTuningSettingBuffer = VL53L0_GetTuningSettingBuffer,
	.StaticInit = VL53L0_StaticInit,
	.WaitDeviceBooted = VL53L0_WaitDeviceBooted,
	.ResetDevice = VL53L0_ResetDevice,
	.SetDeviceParameters = VL53L0_SetDeviceParameters,
	.SetDeviceMode = VL53L0_SetDeviceMode,
	.GetDeviceMode = VL53L0_GetDeviceMode,
	.SetHistogramMode = VL53L0_SetHistogramMode,
	.GetHistogramMode = VL53L0_GetHistogramMode,
	.SetMeasurementTimingBudgetMicroSeconds = VL53L0_SetMeasurementTimingBudgetMicroSeconds,
	.GetMeasurementTimingBudgetMicroSeconds = VL53L0_GetMeasurementTimingBudgetMicroSeconds,
	.GetVcselPulsePeriod = VL53L0_GetVcselPulsePeriod,
	.SetVcselPulsePeriod = VL53L0_SetVcselPulsePeriod,
	.SetSequenceStepEnable = VL53L0_SetSequenceStepEnable,
	.GetSequenceStepEnable = VL53L0_GetSequenceStepEnable,
	.GetSequenceStepEnables = VL53L0_GetSequenceStepEnables,
	.SetSequenceStepTimeout = VL53L0_SetSequenceStepTimeout,
	.GetSequenceStepTimeout = VL53L0_GetSequenceStepTimeout,
	.GetNumberOfSequenceSteps = VL53L0_GetNumberOfSequenceSteps,
	.GetSequenceStepsInfo = VL53L0_GetSequenceStepsInfo,
	.SetInterMeasurementPeriodMilliSeconds = VL53L0_SetInterMeasurementPeriodMilliSeconds,
	.GetInterMeasurementPeriodMilliSeconds = VL53L0_GetInterMeasurementPeriodMilliSeconds,
	.SetXTalkCompensationEnable = VL53L0_SetXTalkCompensationEnable,
	.GetXTalkCompensationEnable = VL53L0_GetXTalkCompensationEnable,
	.SetXTalkCompensationRateMegaCps = VL53L0_SetXTalkCompensationRateMegaCps,
	.GetXTalkCompensationRateMegaCps = VL53L0_GetXTalkCompensationRateMegaCps,
	.GetNumberOfLimitCheck = VL53L0_GetNumberOfLimitCheck,
	.GetLimitCheckInfo = VL53L0_GetLimitCheckInfo,
	.SetLimitCheckEnable = VL53L0_SetLimitCheckEnable,
	.GetLimitCheckEnable = VL53L0_GetLimitCheckEnable,
	.SetLimitCheckValue = VL53L0_SetLimitCheckValue,
	.GetLimitCheckValue = VL53L0_GetLimitCheckValue,
	.GetLimitCheckCurrent = VL53L0_GetLimitCheckCurrent,
	.SetWrapAroundCheckEnable = VL53L0_SetWrapAroundCheckEnable,
	.GetWrapAroundCheckEnable = VL53L0_GetWrapAroundCheckEnable,
	.PerformSingleMeasurement = VL53L0_PerformSingleMeasurement,
	.PerformRefCalibration = VL53L0_PerformRefCalibration,
	.PerformXTalkCalibration = VL53L0_PerformXTalkCalibration,
	.PerformOffsetCalibration = VL53L0_PerformOffsetCalibration,
	.StartMeasurement = VL53L0_StartMeasurement,
	.StopMeasurement = VL53L0_StopMeasurement,
	.GetMeasurementDataReady = VL53L0_GetMeasurementDataReady,
	.WaitDeviceReadyForNewMeasurement = VL53L0_WaitDeviceReadyForNewMeasurement,
	.GetRangingMeasurementData = VL53L0_GetRangingMeasurementData,
	.GetHistogramMeasurementData = VL53L0_GetHistogramMeasurementData,
	.PerformSingleRangingMeasurement = VL53L0_PerformSingleRangingMeasurement,
	.PerformSingleHistogramMeasurement = VL53L0_PerformSingleHistogramMeasurement,
	.SetNumberOfROIZones = VL53L0_SetNumberOfROIZones,
	.GetNumberOfROIZones = VL53L0_GetNumberOfROIZones,
	.GetMaxNumberOfROIZones = VL53L0_GetMaxNumberOfROIZones,
	.SetGpioConfig = VL53L0_SetGpioConfig,
	.GetGpioConfig = VL53L0_GetGpioConfig,
	.SetInterruptThresholds = VL53L0_SetInterruptThresholds,
	.GetInterruptThresholds = VL53L0_GetInterruptThresholds,
	.ClearInterruptMask = VL53L0_ClearInterruptMask,
	.GetInterruptMaskStatus = VL53L0_GetInterruptMaskStatus,
	.EnableInterruptMask = VL53L0_EnableInterruptMask,
	.SetSpadAmbientDamperThreshold = VL53L0_SetSpadAmbientDamperThreshold,
	.GetSpadAmbientDamperThreshold = VL53L0_GetSpadAmbientDamperThreshold,
	.SetSpadAmbientDamperFactor = VL53L0_SetSpadAmbientDamperFactor,
	.GetSpadAmbientDamperFactor = VL53L0_GetSpadAmbientDamperFactor,
	.PerformRefSpadManagement = VL53L0_PerformRefSpadManagement,
};
struct stmvl53l0_api_fn_t *papi_func_tbl;

/*
 * IOCTL definitions
 */
#define VL53L0_IOCTL_INIT			_IO('p', 0x01)
#define VL53L0_IOCTL_XTALKCALB		_IOW('p', 0x02, unsigned int)
#define VL53L0_IOCTL_OFFCALB		_IOW('p', 0x03, unsigned int)
#define VL53L0_IOCTL_STOP			_IO('p', 0x05)
#define VL53L0_IOCTL_SETXTALK		_IOW('p', 0x06, unsigned int)
#define VL53L0_IOCTL_SETOFFSET		_IOW('p', 0x07, int8_t)
#define VL53L0_IOCTL_GETDATAS \
			_IOR('p', 0x0b, VL53L0_RangingMeasurementData_t)
#define VL53L0_IOCTL_REGISTER \
			_IOWR('p', 0x0c, struct stmvl53l0_register)
#define VL53L0_IOCTL_PARAMETER \
			_IOWR('p', 0x0d, struct stmvl53l0_parameter)

#define VL53L0_IOCTL_GET_DATA                  _IOR(VL53L0_MAGIC,      0x01, VL53L0_RangingMeasurementData_t)
#define VL53L0_IOCTL_OFFSET_CALI               _IOR(VL53L0_MAGIC,      0x02, int)
#define VL53L0_IOCTL_XTALK_CALI                _IOR(VL53L0_MAGIC,      0x03, int)
#define VL53L0_IOCTL_SET_XTALK_CALI_DISTANCE   _IOR(VL53L0_MAGIC,      0x04, int)



//#define CALIBRATION_FILE 1
#ifdef CALIBRATION_FILE
int8_t offset_calib;
int16_t xtalk_calib;
#endif

static long stmvl53l0_ioctl(struct file *file,
				unsigned int cmd, unsigned long arg);
/*static int stmvl53l0_flush(struct file *file, fl_owner_t id);*/
static int stmvl53l0_open(struct inode *inode, struct file *file);
static int stmvl53l0_init_client(struct stmvl53l0_data *data);
static int stmvl53l0_start(struct stmvl53l0_data *data, uint8_t scaling,
			init_mode_e mode);
static int stmvl53l0_stop(struct stmvl53l0_data *data);

#ifdef CALIBRATION_FILE
static void stmvl53l0_read_calibration_file(struct stmvl53l0_data *data)
{
	struct file *f;
	char buf[8];
	mm_segment_t fs;
	int i, is_sign = 0;

	f = filp_open("/data/calibration/offset", O_RDONLY, 0);
	if (f != NULL && !IS_ERR(f) && f->f_dentry != NULL) {
		fs = get_fs();
		set_fs(get_ds());
		/* init the buffer with 0 */
		for (i = 0; i < 8; i++)
			buf[i] = 0;
		f->f_op->read(f, buf, 8, &f->f_pos);
		set_fs(fs);
		vl53l0_dbgmsg("offset as:%s, buf[0]:%c\n", buf, buf[0]);
		offset_calib = 0;
		for (i = 0; i < 8; i++) {
			if (i == 0 && buf[0] == '-')
				is_sign = 1;
			else if (buf[i] >= '0' && buf[i] <= '9')
				offset_calib = offset_calib * 10 +
					(buf[i] - '0');
			else
				break;
		}
		if (is_sign == 1)
			offset_calib = -offset_calib;
		vl53l0_dbgmsg("offset_calib as %d\n", offset_calib);
/*later
		VL6180x_SetOffsetCalibrationData(vl53l0_dev, offset_calib);
*/
		filp_close(f, NULL);
	} else {
		vl53l0_errmsg("no offset calibration file exist!\n");
	}

	is_sign = 0;
	f = filp_open("/data/calibration/xtalk", O_RDONLY, 0);
	if (f != NULL && !IS_ERR(f) && f->f_dentry != NULL) {
		fs = get_fs();
		set_fs(get_ds());
		/* init the buffer with 0 */
		for (i = 0; i < 8; i++)
			buf[i] = 0;
		f->f_op->read(f, buf, 8, &f->f_pos);
		set_fs(fs);
		vl53l0_dbgmsg("xtalk as:%s, buf[0]:%c\n", buf, buf[0]);
		xtalk_calib = 0;
		for (i = 0; i < 8; i++) {
			if (i == 0 && buf[0] == '-')
				is_sign = 1;
			else if (buf[i] >= '0' && buf[i] <= '9')
				xtalk_calib = xtalk_calib * 10 + (buf[i] - '0');
			else
				break;
		}
		if (is_sign == 1)
			xtalk_calib = -xtalk_calib;
		vl53l0_dbgmsg("xtalk_calib as %d\n", xtalk_calib);
/* later
		VL6180x_SetXTalkCompensationRate(vl53l0_dev, xtalk_calib);
*/
		filp_close(f, NULL);
	} else {
		vl53l0_errmsg("no xtalk calibration file exist!\n");
	}

}

static void stmvl53l0_write_offset_calibration_file(void)
{
	struct file *f;
	char buf[8];
	mm_segment_t fs;

	f = filp_open("/data/calibration/offset", O_WRONLY|O_CREAT, 0644);
	if (f != NULL) {
		fs = get_fs();
		set_fs(get_ds());
		snprintf(buf, 5, "%d", offset_calib);
		vl53l0_dbgmsg("write offset as:%s, buf[0]:%c\n", buf, buf[0]);
		f->f_op->write(f, buf, 8, &f->f_pos);
		set_fs(fs);
	}
	filp_close(f, NULL);

}

static void stmvl53l0_write_xtalk_calibration_file(void)
{
	struct file *f;
	char buf[8];
	mm_segment_t fs;

	f = filp_open("/data/calibration/xtalk", O_WRONLY|O_CREAT, 0644);
	if (f != NULL) {
		fs = get_fs();
		set_fs(get_ds());
		snprintf(buf, 4, "%d", xtalk_calib);
		vl53l0_dbgmsg("write xtalk as:%s, buf[0]:%c\n", buf, buf[0]);
		f->f_op->write(f, buf, 8, &f->f_pos);
		set_fs(fs);
	}
	filp_close(f, NULL);

}
#endif

#ifdef HTC
static void stmvl53l0_DebugTimeGet(struct timeval *ptv)
{
	do_gettimeofday(ptv);
}

static void stmvl53l0_DebugTimeDuration(struct timeval *pstart_tv,
			struct timeval *pstop_tv)
{
	long total_sec, total_msec;
	total_sec = pstop_tv->tv_sec - pstart_tv->tv_sec;
	total_msec = (pstop_tv->tv_usec - pstart_tv->tv_usec)/1000;
	total_msec += total_sec * 1000;
	timing_dbgmsg("elapsedTime:%ld\n", total_msec);
}
#endif

static void stmvl53l0_setupAPIFunctions(struct stmvl53l0_data *data)
{
	uint8_t revision = 0;
	VL53L0_DEV vl53l0_dev = data;

	/* Read Revision ID */
	VL53L0_RdByte(vl53l0_dev, VL53L0_REG_IDENTIFICATION_REVISION_ID, &revision);
	vl53l0_errmsg("read REVISION_ID: 0x%x\n", revision);
	revision = (revision & 0xF0) >> 4;
	if (revision == 1) {
		/*cut 1.1*/
		vl53l0_errmsg("to setup API cut 1.1\n");
		papi_func_tbl->GetVersion = VL53L0_GetVersion;
		papi_func_tbl->GetPalSpecVersion = VL53L0_GetPalSpecVersion;
		papi_func_tbl->GetProductRevision = VL53L0_GetProductRevision;
		papi_func_tbl->GetDeviceInfo = VL53L0_GetDeviceInfo;
		papi_func_tbl->GetDeviceErrorStatus = VL53L0_GetDeviceErrorStatus;
		papi_func_tbl->GetRangeStatusString = VL53L0_GetRangeStatusString;
		papi_func_tbl->GetDeviceErrorString = VL53L0_GetDeviceErrorString;
		papi_func_tbl->GetPalErrorString = VL53L0_GetPalErrorString;
		papi_func_tbl->GetPalState = VL53L0_GetPalState;
		papi_func_tbl->SetPowerMode = VL53L0_SetPowerMode;
		papi_func_tbl->GetPowerMode = VL53L0_GetPowerMode;
		papi_func_tbl->SetOffsetCalibrationDataMicroMeter = VL53L0_SetOffsetCalibrationDataMicroMeter;
		papi_func_tbl->GetOffsetCalibrationDataMicroMeter = VL53L0_GetOffsetCalibrationDataMicroMeter;
		papi_func_tbl->SetLinearityCorrectiveGain = VL53L0_SetLinearityCorrectiveGain;
		//papi_func_tbl->GetLinearityCorrectiveGain = VL53L0_GetLinearityCorrectiveGain;
		papi_func_tbl->SetGroupParamHold = VL53L0_SetGroupParamHold;
		papi_func_tbl->GetUpperLimitMilliMeter = VL53L0_GetUpperLimitMilliMeter;
		papi_func_tbl->SetDeviceAddress = VL53L0_SetDeviceAddress;
		papi_func_tbl->DataInit = VL53L0_DataInit;
		papi_func_tbl->SetTuningSettingBuffer = VL53L0_SetTuningSettingBuffer;
//		papi_func_tbl->GetTuningSettingBuffer = VL53L0_GetTuningSettingBuffer;
		papi_func_tbl->StaticInit = VL53L0_StaticInit;
		papi_func_tbl->WaitDeviceBooted = VL53L0_WaitDeviceBooted;
		papi_func_tbl->ResetDevice = VL53L0_ResetDevice;
		papi_func_tbl->SetDeviceParameters = VL53L0_SetDeviceParameters;
		papi_func_tbl->SetDeviceMode = VL53L0_SetDeviceMode;
		papi_func_tbl->GetDeviceMode = VL53L0_GetDeviceMode;
		papi_func_tbl->SetHistogramMode = VL53L0_SetHistogramMode;
		papi_func_tbl->GetHistogramMode = VL53L0_GetHistogramMode;
		papi_func_tbl->SetMeasurementTimingBudgetMicroSeconds = VL53L0_SetMeasurementTimingBudgetMicroSeconds;
		papi_func_tbl->GetMeasurementTimingBudgetMicroSeconds = VL53L0_GetMeasurementTimingBudgetMicroSeconds;
		papi_func_tbl->GetVcselPulsePeriod = VL53L0_GetVcselPulsePeriod;
		papi_func_tbl->SetVcselPulsePeriod = VL53L0_SetVcselPulsePeriod;
		papi_func_tbl->SetSequenceStepEnable = VL53L0_SetSequenceStepEnable;
		papi_func_tbl->GetSequenceStepEnable = VL53L0_GetSequenceStepEnable;
		papi_func_tbl->GetSequenceStepEnables = VL53L0_GetSequenceStepEnables;
		papi_func_tbl->SetSequenceStepTimeout = VL53L0_SetSequenceStepTimeout;
		papi_func_tbl->GetSequenceStepTimeout = VL53L0_GetSequenceStepTimeout;
		papi_func_tbl->GetNumberOfSequenceSteps = VL53L0_GetNumberOfSequenceSteps;
		papi_func_tbl->GetSequenceStepsInfo = VL53L0_GetSequenceStepsInfo;
		papi_func_tbl->SetInterMeasurementPeriodMilliSeconds = VL53L0_SetInterMeasurementPeriodMilliSeconds;
		papi_func_tbl->GetInterMeasurementPeriodMilliSeconds = VL53L0_GetInterMeasurementPeriodMilliSeconds;
		papi_func_tbl->SetXTalkCompensationEnable = VL53L0_SetXTalkCompensationEnable;
		papi_func_tbl->GetXTalkCompensationEnable = VL53L0_GetXTalkCompensationEnable;
		papi_func_tbl->SetXTalkCompensationRateMegaCps = VL53L0_SetXTalkCompensationRateMegaCps;
		papi_func_tbl->GetXTalkCompensationRateMegaCps = VL53L0_GetXTalkCompensationRateMegaCps;
		papi_func_tbl->GetNumberOfLimitCheck = VL53L0_GetNumberOfLimitCheck;
		papi_func_tbl->GetLimitCheckInfo = VL53L0_GetLimitCheckInfo;
		papi_func_tbl->SetLimitCheckEnable = VL53L0_SetLimitCheckEnable;
		papi_func_tbl->GetLimitCheckEnable = VL53L0_GetLimitCheckEnable;
		papi_func_tbl->SetLimitCheckValue = VL53L0_SetLimitCheckValue;
		papi_func_tbl->GetLimitCheckValue = VL53L0_GetLimitCheckValue;
		papi_func_tbl->GetLimitCheckCurrent = VL53L0_GetLimitCheckCurrent;
		papi_func_tbl->SetWrapAroundCheckEnable = VL53L0_SetWrapAroundCheckEnable;
		papi_func_tbl->GetWrapAroundCheckEnable = VL53L0_GetWrapAroundCheckEnable;
		papi_func_tbl->PerformSingleMeasurement = VL53L0_PerformSingleMeasurement;
		papi_func_tbl->PerformRefCalibration = VL53L0_PerformRefCalibration;
		papi_func_tbl->PerformXTalkCalibration = VL53L0_PerformXTalkCalibration;
		papi_func_tbl->PerformOffsetCalibration = VL53L0_PerformOffsetCalibration;
		papi_func_tbl->StartMeasurement = VL53L0_StartMeasurement;
		papi_func_tbl->StopMeasurement = VL53L0_StopMeasurement;
		papi_func_tbl->GetMeasurementDataReady = VL53L0_GetMeasurementDataReady;
		papi_func_tbl->WaitDeviceReadyForNewMeasurement =
			VL53L0_WaitDeviceReadyForNewMeasurement;
		papi_func_tbl->GetRangingMeasurementData =
			VL53L0_GetRangingMeasurementData;
		papi_func_tbl->GetHistogramMeasurementData =
			VL53L0_GetHistogramMeasurementData;
		papi_func_tbl->PerformSingleRangingMeasurement =
			VL53L0_PerformSingleRangingMeasurement;
		papi_func_tbl->PerformSingleHistogramMeasurement =
			VL53L0_PerformSingleHistogramMeasurement;
		papi_func_tbl->SetNumberOfROIZones = VL53L0_SetNumberOfROIZones;
		papi_func_tbl->GetNumberOfROIZones = VL53L0_GetNumberOfROIZones;
		papi_func_tbl->GetMaxNumberOfROIZones = VL53L0_GetMaxNumberOfROIZones;
		papi_func_tbl->SetGpioConfig = VL53L0_SetGpioConfig;
		papi_func_tbl->GetGpioConfig = VL53L0_GetGpioConfig;
		papi_func_tbl->SetInterruptThresholds = VL53L0_SetInterruptThresholds;
		papi_func_tbl->GetInterruptThresholds = VL53L0_GetInterruptThresholds;
		papi_func_tbl->ClearInterruptMask = VL53L0_ClearInterruptMask;
		papi_func_tbl->GetInterruptMaskStatus = VL53L0_GetInterruptMaskStatus;
		papi_func_tbl->EnableInterruptMask = VL53L0_EnableInterruptMask;
		papi_func_tbl->SetSpadAmbientDamperThreshold =
			VL53L0_SetSpadAmbientDamperThreshold;
		papi_func_tbl->GetSpadAmbientDamperThreshold =
			VL53L0_GetSpadAmbientDamperThreshold;
		papi_func_tbl->SetSpadAmbientDamperFactor =
			VL53L0_SetSpadAmbientDamperFactor;
		papi_func_tbl->GetSpadAmbientDamperFactor =
			VL53L0_GetSpadAmbientDamperFactor;
		papi_func_tbl->PerformRefSpadManagement = VL53L0_PerformRefSpadManagement;


	} else if (revision == 0) {
		/*cut 1.0*/
		vl53l0_errmsg("to setup API cut 1.0\n");
		papi_func_tbl->GetVersion = VL53L010_GetVersion;
		papi_func_tbl->GetPalSpecVersion = VL53L010_GetPalSpecVersion;
		//papi_func_tbl->GetProductRevision = NULL;
		papi_func_tbl->GetDeviceInfo = VL53L010_GetDeviceInfo;
		papi_func_tbl->GetDeviceErrorStatus = VL53L010_GetDeviceErrorStatus;
		papi_func_tbl->GetDeviceErrorString = VL53L010_GetDeviceErrorString;
		papi_func_tbl->GetPalErrorString = VL53L010_GetPalErrorString;
		papi_func_tbl->GetPalState = VL53L010_GetPalState;
		papi_func_tbl->SetPowerMode = VL53L010_SetPowerMode;
		papi_func_tbl->GetPowerMode = VL53L010_GetPowerMode;
		papi_func_tbl->SetOffsetCalibrationDataMicroMeter =
			VL53L010_SetOffsetCalibrationDataMicroMeter;
		papi_func_tbl->GetOffsetCalibrationDataMicroMeter =
			VL53L010_GetOffsetCalibrationDataMicroMeter;
		papi_func_tbl->SetGroupParamHold = VL53L010_SetGroupParamHold;
		papi_func_tbl->GetUpperLimitMilliMeter = VL53L010_GetUpperLimitMilliMeter;
		papi_func_tbl->SetDeviceAddress = VL53L010_SetDeviceAddress;
		papi_func_tbl->DataInit = VL53L010_DataInit;
		/*
		papi_func_tbl->SetTuningSettingBuffer = NULL;
		papi_func_tbl->GetTuningSettingBuffer = NULL;
		*/
		papi_func_tbl->StaticInit = VL53L010_StaticInit;
		papi_func_tbl->WaitDeviceBooted = VL53L010_WaitDeviceBooted;
		papi_func_tbl->ResetDevice = VL53L010_ResetDevice;
		papi_func_tbl->SetDeviceParameters = VL53L010_SetDeviceParameters;
		papi_func_tbl->SetDeviceMode = VL53L010_SetDeviceMode;
		papi_func_tbl->GetDeviceMode = VL53L010_GetDeviceMode;
		papi_func_tbl->SetHistogramMode = VL53L010_SetHistogramMode;
		papi_func_tbl->GetHistogramMode = VL53L010_GetHistogramMode;
		papi_func_tbl->SetMeasurementTimingBudgetMicroSeconds =
			VL53L010_SetMeasurementTimingBudgetMicroSeconds;
		papi_func_tbl->GetMeasurementTimingBudgetMicroSeconds =
			VL53L010_GetMeasurementTimingBudgetMicroSeconds;
		/*
		papi_func_tbl->GetVcselPulsePeriod = NULL;
		papi_func_tbl->SetVcselPulsePeriod = NULL;
		papi_func_tbl->SetSequenceStepEnable = NULL;
		papi_func_tbl->GetSequenceStepEnable = NULL;
		papi_func_tbl->GetSequenceStepEnables = NULL;
		papi_func_tbl->SetSequenceStepTimeout = NULL;
		papi_func_tbl->GetSequenceStepTimeout = NULL;
		papi_func_tbl->GetNumberOfSequenceSteps =NULL;
		papi_func_tbl->GetSequenceStepsInfo = NULL;
		*/
		papi_func_tbl->SetInterMeasurementPeriodMilliSeconds =
			VL53L010_SetInterMeasurementPeriodMilliSeconds;
		papi_func_tbl->GetInterMeasurementPeriodMilliSeconds =
			VL53L010_GetInterMeasurementPeriodMilliSeconds;
		papi_func_tbl->SetXTalkCompensationEnable =
			VL53L010_SetXTalkCompensationEnable;
		papi_func_tbl->GetXTalkCompensationEnable =
			VL53L010_GetXTalkCompensationEnable;
		papi_func_tbl->SetXTalkCompensationRateMegaCps =
			VL53L010_SetXTalkCompensationRateMegaCps;
		papi_func_tbl->GetXTalkCompensationRateMegaCps =
			VL53L010_GetXTalkCompensationRateMegaCps;
		papi_func_tbl->GetNumberOfLimitCheck = VL53L010_GetNumberOfLimitCheck;
		papi_func_tbl->GetLimitCheckInfo = VL53L010_GetLimitCheckInfo;
		papi_func_tbl->SetLimitCheckEnable = VL53L010_SetLimitCheckEnable;
		papi_func_tbl->GetLimitCheckEnable = VL53L010_GetLimitCheckEnable;
		papi_func_tbl->SetLimitCheckValue = VL53L010_SetLimitCheckValue;
		papi_func_tbl->GetLimitCheckValue = VL53L010_GetLimitCheckValue;
		papi_func_tbl->GetLimitCheckCurrent = VL53L010_GetLimitCheckCurrent;
		papi_func_tbl->SetWrapAroundCheckEnable =
			VL53L010_SetWrapAroundCheckEnable;
		papi_func_tbl->GetWrapAroundCheckEnable =
			VL53L010_GetWrapAroundCheckEnable;
		papi_func_tbl->PerformSingleMeasurement =
			VL53L010_PerformSingleMeasurement;
		//papi_func_tbl->PerformRefCalibration = VL53L010_PerformRefCalibration;
		papi_func_tbl->PerformRefCalibration = NULL;
		papi_func_tbl->PerformXTalkCalibration = VL53L010_PerformXTalkCalibration;
		papi_func_tbl->PerformOffsetCalibration =
			VL53L010_PerformOffsetCalibration;
		papi_func_tbl->StartMeasurement = VL53L010_StartMeasurement;
		papi_func_tbl->StopMeasurement = VL53L010_StopMeasurement;
		papi_func_tbl->GetMeasurementDataReady = VL53L010_GetMeasurementDataReady;
		papi_func_tbl->WaitDeviceReadyForNewMeasurement =
			VL53L010_WaitDeviceReadyForNewMeasurement;
		papi_func_tbl->GetRangingMeasurementData =
			VL53L010_GetRangingMeasurementData;
		papi_func_tbl->GetHistogramMeasurementData =
			VL53L010_GetHistogramMeasurementData;
		papi_func_tbl->PerformSingleRangingMeasurement =
			VL53L010_PerformSingleRangingMeasurement;
		papi_func_tbl->PerformSingleHistogramMeasurement =
			VL53L010_PerformSingleHistogramMeasurement;
		papi_func_tbl->SetNumberOfROIZones = VL53L010_SetNumberOfROIZones;
		papi_func_tbl->GetNumberOfROIZones = VL53L010_GetNumberOfROIZones;
		papi_func_tbl->GetMaxNumberOfROIZones = VL53L010_GetMaxNumberOfROIZones;
		papi_func_tbl->SetGpioConfig = VL53L010_SetGpioConfig;
		papi_func_tbl->GetGpioConfig = VL53L010_GetGpioConfig;
		papi_func_tbl->SetInterruptThresholds = VL53L010_SetInterruptThresholds;
		papi_func_tbl->GetInterruptThresholds = VL53L010_GetInterruptThresholds;
		papi_func_tbl->ClearInterruptMask = VL53L010_ClearInterruptMask;
		papi_func_tbl->GetInterruptMaskStatus = VL53L010_GetInterruptMaskStatus;
		papi_func_tbl->EnableInterruptMask = VL53L010_EnableInterruptMask;
		papi_func_tbl->SetSpadAmbientDamperThreshold =
			VL53L010_SetSpadAmbientDamperThreshold;
		papi_func_tbl->GetSpadAmbientDamperThreshold =
			VL53L010_GetSpadAmbientDamperThreshold;
		papi_func_tbl->SetSpadAmbientDamperFactor =
			VL53L010_SetSpadAmbientDamperFactor;
		papi_func_tbl->GetSpadAmbientDamperFactor =
			VL53L010_GetSpadAmbientDamperFactor;
		papi_func_tbl->PerformRefSpadManagement =NULL;

	}

}

static void stmvl53l0_ps_read_measurement(struct stmvl53l0_data *data)
{
	struct timeval tv;

	do_gettimeofday(&tv);

	data->ps_data = data->rangeData.RangeMilliMeter;
	input_report_abs(data->input_dev_ps, ABS_DISTANCE, (int)(data->ps_data + 5) / 10);
	input_report_abs(data->input_dev_ps, ABS_HAT0X, tv.tv_sec);
	input_report_abs(data->input_dev_ps, ABS_HAT0Y, tv.tv_usec);
	input_report_abs(data->input_dev_ps, ABS_HAT1X, data->rangeData.RangeMilliMeter);
	input_report_abs(data->input_dev_ps, ABS_HAT1Y,	data->rangeData.RangeStatus);
	input_report_abs(data->input_dev_ps, ABS_HAT2X,	data->rangeData.SignalRateRtnMegaCps);
	input_report_abs(data->input_dev_ps, ABS_HAT2Y,	data->rangeData.AmbientRateRtnMegaCps);
	input_report_abs(data->input_dev_ps, ABS_HAT3X,	data->rangeData.MeasurementTimeUsec);
	input_report_abs(data->input_dev_ps, ABS_HAT3Y,	data->rangeData.RangeDMaxMilliMeter);
	input_sync(data->input_dev_ps);

	if (data->enableDebug)
		vl53l0_errmsg("range:%d, RtnRateMcps:%d,err:0x%x,Dmax:%d,rtnambr:%d,time:%d\n",
					data->rangeData.RangeMilliMeter,
					data->rangeData.SignalRateRtnMegaCps,
					data->rangeData.RangeStatus,
					data->rangeData.RangeDMaxMilliMeter,
					data->rangeData.AmbientRateRtnMegaCps,
					data->rangeData.MeasurementTimeUsec);
}

static void stmvl53l0_cancel_handler(struct stmvl53l0_data *data)
{
	unsigned long flags;
	bool ret;

	spin_lock_irqsave(&data->update_lock.wait_lock, flags);
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	ret = cancel_delayed_work(&data->dwork);
	if (ret == 0)
		vl53l0_errmsg("cancel_delayed_work return FALSE\n");

	spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);

}

static void stmvl53l0_schedule_handler(struct stmvl53l0_data *data)
{
	unsigned long flags;

	spin_lock_irqsave(&data->update_lock.wait_lock, flags);
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	cancel_delayed_work(&data->dwork);
	schedule_delayed_work(&data->dwork, msecs_to_jiffies(data->delay_ms));
	spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);

}


#ifdef USE_INT
static irqreturn_t stmvl53l0_interrupt_handler(int vec, void *info)
{

	struct stmvl53l0_data *data = (struct stmvl53l0_data *)info;
	vl53l0_dbgmsg("enter\n");
	if (data->irq == vec) {
		data->interrupt_received = 1;
		schedule_delayed_work(&data->dwork, 0);
	}
	return IRQ_HANDLED;
}
#endif

/* work handler */
static void stmvl53l0_work_handler(struct work_struct *work)
{
	struct stmvl53l0_data *data = container_of(work,
				struct stmvl53l0_data, dwork.work);
	VL53L0_DEV vl53l0_dev = data;
//	uint8_t val;
	uint32_t interruptStatus = 0;
	VL53L0_Error Status = VL53L0_ERROR_NONE;

	vl53l0_dbgmsg("enter\n");

	mutex_lock(&data->work_mutex);

	if (data->enable_ps_sensor == 1) {
		/* vl53l0_dbgmsg("Enter\n"); */
#ifdef HTC
		if (data->enableTimingDebug) {
			stmvl53l0_DebugTimeGet(&stop_tv);
			stmvl53l0_DebugTimeDuration(&start_tv, &stop_tv);
		}
#endif
//		Status = VL53L0_RdByte(vl53l0_dev, VL53L0_REG_RESULT_RANGE_STATUS, &val);
//		timing_dbgmsg("RangeStatus: 0x%x ===============================\n", val);

		Status = VL53L0_GetInterruptMaskStatus(vl53l0_dev, &interruptStatus);
		if (data->enableDebug)
			timing_dbgmsg("interruptStatus:0x%x, interrupt_received:%d\n",
						interruptStatus, data->interrupt_received);
		data->interrupt_received = 0;
//		if (Status == VL53L0_ERROR_NONE &&
//		    interruptStatus == data->gpio_function) {
		Status = papi_func_tbl->ClearInterruptMask(vl53l0_dev, 0);
		Status = papi_func_tbl->GetRangingMeasurementData(
					vl53l0_dev, &(data->rangeData));
				/* to push the measurement */
		if (Status == VL53L0_ERROR_NONE)
			stmvl53l0_ps_read_measurement(data);
#ifdef HTC
		if (data->enableTimingDebug)
			timing_dbgmsg("Measured range:%d\n", data->rangeData.RangeMilliMeter);
#endif
//		}
#ifdef HTC
		if (data->enableTimingDebug)
			stmvl53l0_DebugTimeGet(&start_tv);
#endif
		if (data->deviceMode == VL53L0_DEVICEMODE_SINGLE_RANGING) {
			Status = papi_func_tbl->StartMeasurement(vl53l0_dev);
		}

		/* enable work handler */
		/* if interrupt is trigger, the work-handler will kick out immediately*/
		stmvl53l0_schedule_handler(data);
	}
	mutex_unlock(&data->work_mutex);
}

#ifdef HTC
static int Laser_xtalk_calibrate(FixPoint1616_t *XTalkCompensationRateMegaCps,
				 struct stmvl53l0_data *data)
{
    VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DEV vl53l0_dev = data;

    Status = VL53L0_SetXTalkCompensationRateMegaCps(vl53l0_dev, 0);
    if (Status == VL53L0_ERROR_NONE)
    {
        if (data->cali_distance != 0)
        {
            Status = VL53L0_PerformXTalkCalibration(vl53l0_dev,
			 data->cali_distance, XTalkCompensationRateMegaCps);
            if (Status == VL53L0_ERROR_NONE)
            {
                vl53l0_dbgmsg("Xtalk calibration finished. "
			      "XTalkCompensationRateMegaCps = 0x%X "
			      "with distance = %d mm\n",
					*XTalkCompensationRateMegaCps,
					(data->cali_distance>>16));
                // Update to xtalk_kvalue
                data->xtalk_kvalue = *XTalkCompensationRateMegaCps;
                return 0;
            }
            else
            {
                vl53l0_errmsg("Failed in xtalk calibration\n");
                return -1;
            }
        }
        else
        {
            vl53l0_errmsg("Please set distance before xtalk calibration\n");
            return -1;
        }
    }
    else
    {
        vl53l0_errmsg("Set default xtalk value failed\n");
        return -1;
    }

    return 0;
}


static int Laser_offset_calibrate(int32_t *offset, struct stmvl53l0_data *data)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
    VL53L0_DEV vl53l0_dev = data;

	Status = VL53L0_SetOffsetCalibrationDataMicroMeter(vl53l0_dev, 0);
	Status |= VL53L0_SetXTalkCompensationRateMegaCps(vl53l0_dev, 0);
	if (Status == VL53L0_ERROR_NONE)
	{
		Status = VL53L0_PerformOffsetCalibration(vl53l0_dev, (OFFSET_CALI_TARGET_DISTANCE << 16), offset);
		if (Status == VL53L0_ERROR_NONE) {
			// Update to offset_kvalue
			*offset = *offset / 1000;
			data->offset_kvalue = *offset;
			return 0;
		} else {
			vl53l0_errmsg("%s: Failed in offset calibration\n", __func__);
			return -1;
		}
	}
	else
	{
		vl53l0_errmsg("%s: Set default offset/xtalk value failed\n", __func__);
		return -1;
	}

	return 0;
}

static ssize_t laser_power_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stmvl53l0_data *data = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "laser_power = %d\n", data->enable_ps_sensor);
							       //data->laser_power);
}

static ssize_t laser_power_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct stmvl53l0_data *data = dev_get_drvdata(dev);
	unsigned long value = 0;
	int err = 0;

	vl53l0_dbgmsg("Enter++\n");

	err = kstrtoul(buf, 10, &value);
	if (err) {
		vl53l0_errmsg("kstrtoul fails, error = %d\n", err);
		return err;
	}
	if (value == 1)
	        err = stmvl53l0_start(data, 3, NORMAL_MODE);
	else
	        err = stmvl53l0_stop(data);
	if (err)
		return -1;

	vl53l0_dbgmsg("End--\n");
	return count;
}

static ssize_t laser_hwid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	u8 model_id = 0;
	u8 revisin_id = 0;
	u8 module_id = 0;
	u16 rangeA_timeout = 0;
	u16 rangeB1_timeout = 0;
	struct stmvl53l0_data *data = dev_get_drvdata(dev);
	VL53L0_DEV vl53l0_dev = data;


	ret = VL53L0_RdByte(vl53l0_dev, VL53L0_REG_IDENTIFICATION_MODEL_ID, &model_id);
	ret += VL53L0_RdByte(vl53l0_dev, VL53L0_REG_IDENTIFICATION_REVISION_ID, &revisin_id);
	ret += VL53L0_RdByte(vl53l0_dev, VL53L0_REG_IDENTIFICATION_MODEL_ID, &module_id);

	if (ret == 0)
		return scnprintf(buf, PAGE_SIZE, "0x%X 0x%X 0x%X 0x%X 0x%X\n",
			 model_id, revisin_id, module_id, rangeA_timeout, rangeB1_timeout);
	else
		return scnprintf(buf, PAGE_SIZE, "0 0 0 0 0\n");
}

static ssize_t laser_range_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	VL53L0_RangingMeasurementData_t RangingMeasurementData;
	struct stmvl53l0_data *data = dev_get_drvdata(dev);
	VL53L0_DEV vl53l0_dev = data;

	if (data->enable_ps_sensor == 1) {
		//Status = VL53L0_PerformSingleRangingMeasurement(vl53l0_dev,
		//					 &RangingMeasurementData);
		Status = papi_func_tbl->PerformSingleRangingMeasurement(vl53l0_dev,
							 &RangingMeasurementData);
		if (Status == VL53L0_ERROR_NONE) {
			vl53l0_dbgmsg("RangeMilliMeter = %d,  "
				      "SignalRateRtnMegaCps = 0x%x\n",
					RangingMeasurementData.RangeMilliMeter,
					RangingMeasurementData.SignalRateRtnMegaCps);
		}
		return scnprintf(buf, PAGE_SIZE, "RangeMilliMeter = %d,  SignalRateRtnMegaCps = 0x%x\n",
			 RangingMeasurementData.RangeMilliMeter, RangingMeasurementData.SignalRateRtnMegaCps);
	} else {
		return scnprintf(buf, PAGE_SIZE, "RangeMilliMeter = 0,  SignalRateRtnMegaCps = 0\n");
	}
}

static ssize_t laser_compensation_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	FixPoint1616_t compensation_enable = 0;
	struct stmvl53l0_data *data = dev_get_drvdata(dev);
	VL53L0_DEV vl53l0_dev = data;

	//Status = VL53L0_GetXTalkCompensationEnable(vl53l0_dev, &compensation_enable);
	Status = papi_func_tbl->GetXTalkCompensationRateMegaCps(vl53l0_dev,
								&compensation_enable);
	if (Status == VL53L0_ERROR_NONE)
	{
		return scnprintf(buf, PAGE_SIZE, "compensation_enable = %d \n", compensation_enable);
	}
	else
	{
		return scnprintf(buf, PAGE_SIZE, "Failed in VL53L0_GetXTalkCompensationEnable\n");
	}
}

static ssize_t laser_compensation_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	unsigned long value = 0;
	int err = 0;
	struct stmvl53l0_data *data = dev_get_drvdata(dev);
	VL53L0_DEV vl53l0_dev = data;

	err = kstrtoul(buf, 10, &value);
	if (err) {
		vl53l0_errmsg("%s: kstrtoul fails, error = %d\n", __func__, err);
		return err;
	}

	//Status = VL53L0_SetXTalkCompensationEnable(vl53l0_dev, value);
	Status = papi_func_tbl->SetXTalkCompensationEnable(vl53l0_dev, value);
	if (Status == VL53L0_ERROR_NONE)
	{
		vl53l0_dbgmsg("%s: Set compensation_enable = %lu\n", __func__, value);
	}
	else
	{
		vl53l0_errmsg("%s: Failed in VL53L0_SetXTalkCompensationEnable\n", __func__);
		return -1;
	}

	return count;
}

static ssize_t laser_offset_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	int offset_value = 0;
	struct stmvl53l0_data *data = dev_get_drvdata(dev);
	VL53L0_DEV vl53l0_dev = data;

	//Status = VL53L0_GetOffsetCalibrationDataMicroMeter(vl53l0_dev, &offset_value);
	Status = papi_func_tbl->GetOffsetCalibrationDataMicroMeter(vl53l0_dev, &offset_value);
	offset_value = (offset_value / 1000);
	if (Status == VL53L0_ERROR_NONE)
	{
		return scnprintf(buf, PAGE_SIZE, "Offset = %d mm\n", offset_value);
	}
	else
	{
		return scnprintf(buf, PAGE_SIZE, "Failed in VL53L0_GetOffsetCalibrationDataMicroMeter\n");
	}
}

static ssize_t laser_offset_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	int value = 0;
	int err = 0;
	struct stmvl53l0_data *data = dev_get_drvdata(dev);
	VL53L0_DEV vl53l0_dev = data;

	err = kstrtoint(buf, 10, &value);
	if (err) {
		vl53l0_errmsg("%s: kstrtoint fails, error = %d\n", __func__, err);
		return err;
	}

	//Status = VL53L0_SetOffsetCalibrationDataMicroMeter(vl53l0_dev, (value*1000));
	Status = papi_func_tbl->SetOffsetCalibrationDataMicroMeter(vl53l0_dev, (value*1000));
	if(Status == VL53L0_ERROR_NONE)
	{
		vl53l0_dbgmsg("%s: Set offset = %d mm\n", __func__, value);
		// Update to offset_kvalue
		data->offset_kvalue = value;
	}
	else
	{
		vl53l0_errmsg("%s: Failed in VL53L0_SetOffsetCalibrationDataMicroMeter\n", __func__);
		return -1;
	}

	return count;
}

static ssize_t laser_xtalk_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	FixPoint1616_t XTalkCompensationRateMegaCps;
	struct stmvl53l0_data *data = dev_get_drvdata(dev);
	VL53L0_DEV vl53l0_dev = data;

	//Status = VL53L0_GetXTalkCompensationRateMegaCps(vl53l0_dev, &XTalkCompensationRateMegaCps);
	Status = papi_func_tbl->GetXTalkCompensationRateMegaCps(vl53l0_dev, &XTalkCompensationRateMegaCps);
	if (Status == VL53L0_ERROR_NONE)
	{
		return scnprintf(buf, PAGE_SIZE, "Xtalk = 0x%X\n", XTalkCompensationRateMegaCps);
	}
	else
	{
		return scnprintf(buf, PAGE_SIZE, "Failed in VL53L0_GetXTalkCompensationRateMegaCps\n");
	}
}

static ssize_t laser_xtalk_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	unsigned long value = 0;
	int err = 0;
	struct stmvl53l0_data *data = dev_get_drvdata(dev);
	VL53L0_DEV vl53l0_dev = data;

	err = kstrtoul(buf, 10, &value);
	if (err) {
		vl53l0_errmsg("%s: kstrtoul fails, error = %d\n", __func__, err);
		return err;
	}

	//Status = VL53L0_SetXTalkCompensationRateMegaCps(vl53l0_dev, value);
	Status = papi_func_tbl->SetXTalkCompensationRateMegaCps(vl53l0_dev, value);
	if(Status == VL53L0_ERROR_NONE)
	{
		vl53l0_dbgmsg("%s: Set xtalk = %lu\n", __func__, value);
		// Update to xtalk_kvalue
		data->xtalk_kvalue = value;
	}
	else
	{
		vl53l0_errmsg("%s: Failed in VL53L0_SetXTalkCompensationRateMegaCps\n", __func__);
		return -1;
	}

	return count;
}

static ssize_t laser_offset_calibrate_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int offset_value = 0;
	struct stmvl53l0_data *data = dev_get_drvdata(dev);

	ret = Laser_offset_calibrate(&offset_value, data);
	if (ret == 0)
	{
		return scnprintf(buf, PAGE_SIZE, "Offset calibration finished. "
							"Offset = %d mm\n", offset_value);
	}
	else
	{
		return scnprintf(buf, PAGE_SIZE, "Failed in offset calibration\n");
	}

}

static ssize_t laser_xtalk_calibrate_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned long value = 0;
	int err = 0;
    struct stmvl53l0_data *data = dev_get_drvdata(dev);

	err = kstrtoul(buf, 10, &value);
	if (err) {
		vl53l0_errmsg("%s: kstrtoul fails, error = %d\n", __func__, err);
		return err;
	}

	data->cali_distance = value << 16;

	return count;
}

static ssize_t laser_xtalk_calibrate_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	FixPoint1616_t XTalkCompensationRateMegaCps = 0;
    struct stmvl53l0_data *data = dev_get_drvdata(dev);

	if (data->cali_distance != 0)
	{
		ret = Laser_xtalk_calibrate(&XTalkCompensationRateMegaCps, data);
		if (ret == 0)
		{
			return scnprintf(buf, PAGE_SIZE, "Xtalk calibration finished. XTalkCompensationRateMegaCps = 0x%X with distance = %d mm\n",
					 XTalkCompensationRateMegaCps,
					 (data->cali_distance>>16));
		}
		else
		{
			return scnprintf(buf, PAGE_SIZE, "Failed in xtalk calibration\n");
		}
	}
	else
	{
		return scnprintf(buf, PAGE_SIZE, "Please set distance before xtalk calibration\n");
	}
}

static ssize_t laser_cali_status_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct stmvl53l0_data *data = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "MFG calibration status = 0x%x\n",data->cali_status);
}
#endif

/*
 * SysFS support
 */
static ssize_t stmvl53l0_show_enable_ps_sensor(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct stmvl53l0_data *data = dev_get_drvdata(dev);

	//return snprintf(buf, 5, "%d\n", data->enable_ps_sensor);
	return scnprintf(buf, PAGE_SIZE, "%d\n", data->enable_ps_sensor);
}

static ssize_t stmvl53l0_store_enable_ps_sensor(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct stmvl53l0_data *data = dev_get_drvdata(dev);

	unsigned long val = simple_strtoul(buf, NULL, 10);

	if ((val != 0) && (val != 1)) {
		vl53l0_errmsg("store unvalid value=%ld\n", val);
		return count;
	}
	mutex_lock(&data->work_mutex);
	vl53l0_dbgmsg("Enter, enable_ps_sensor flag:%d\n",
					data->enable_ps_sensor);
	vl53l0_dbgmsg("enable ps senosr (%ld)\n", val);

	if (val == 1) {
		if (data->enable_ps_sensor == 0) {
			/* turn on tof sensor */
			stmvl53l0_start(data, 3, NORMAL_MODE);
		} else {
			vl53l0_errmsg("Already enabled. Skip !");
		}
	} else {
		/* turn off tof sensor */
		if (data->enable_ps_sensor == 1) {
			/* turn off tof sensor */
			stmvl53l0_stop(data);
		}
	}
	vl53l0_dbgmsg("End\n");
	mutex_unlock(&data->work_mutex);

	return count;
}

#if 0
static DEVICE_ATTR(enable_ps_sensor, 0664/*S_IWUGO | S_IRUGO*/,
		   	stmvl53l0_show_enable_ps_sensor,
			stmvl53l0_store_enable_ps_sensor);
#endif

/* for debug */
static ssize_t stmvl53l0_show_enable_debug(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stmvl53l0_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", data->enableDebug);
}

static ssize_t stmvl53l0_store_enable_debug(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct stmvl53l0_data *data = dev_get_drvdata(dev);
	long on = simple_strtoul(buf, NULL, 10);

	if ((on != 0) && (on != 1)) {
		vl53l0_errmsg("set debug=%ld\n", on);
		return count;
	}
	data->enableDebug = on;

	return count;
}

#if 0
/* DEVICE_ATTR(name,mode,show,store) */
static DEVICE_ATTR(enable_debug, 0660/*S_IWUSR | S_IRUGO*/,
				stmvl53l0_show_enable_debug,
				stmvl53l0_store_enable_debug);
#endif

/* for timing debug */
static ssize_t stmvl53l0_show_enable_timing_debug(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stmvl53l0_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", data->enableTimingDebug);
}

static ssize_t stmvl53l0_store_enable_timing_debug(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct stmvl53l0_data *data = dev_get_drvdata(dev);
	long on = simple_strtoul(buf, NULL, 10);

	if ((on != 0) && (on != 1)) {
		vl53l0_errmsg("set timing_debug=%ld\n", on);
		return count;
	}
	data->enableTimingDebug = on;

	return count;
}

#if 0
/* DEVICE_ATTR(name,mode,show,store) */
static DEVICE_ATTR(enable_timing_debug, 0660/*S_IWUSR | S_IRUGO*/,
				stmvl53l0_show_enable_timing_debug,
				stmvl53l0_store_enable_timing_debug);
#endif

static ssize_t stmvl53l0_show_set_delay_ms(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stmvl53l0_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", data->delay_ms);
}

/* for work handler scheduler time */
static ssize_t stmvl53l0_store_set_delay_ms(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct stmvl53l0_data *data = dev_get_drvdata(dev);
	long delay_ms = simple_strtoul(buf, NULL, 10);

	if (delay_ms == 0) {
		vl53l0_errmsg("set delay_ms=%ld\n", delay_ms);
		return count;
	}
	mutex_lock(&data->work_mutex);
	data->delay_ms = delay_ms;
	mutex_unlock(&data->work_mutex);

	return count;
}

#if 0
/* DEVICE_ATTR(name,mode,show,store) */
static DEVICE_ATTR(set_delay_ms, 0660/*S_IWUGO | S_IRUGO*/,
				   stmvl53l0_show_set_delay_ms,
					stmvl53l0_store_set_delay_ms);

static struct attribute *stmvl53l0_attributes[] = {
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_enable_debug.attr,
	&dev_attr_enable_timing_debug.attr,
	&dev_attr_set_delay_ms.attr,
	NULL,
};

static const struct attribute_group stmvl53l0_attr_group = {
	.attrs = stmvl53l0_attributes,
};
#endif

#ifdef HTC
static struct device_attribute attributes[] = {
    __ATTR(laser_power, 0660, laser_power_show, laser_power_store),
    __ATTR(laser_hwid, 0440, laser_hwid_show, NULL),
    __ATTR(laser_range, 0440, laser_range_show, NULL),
    __ATTR(laser_compensation, 0660, laser_compensation_show, laser_compensation_store),
    __ATTR(laser_offset, 0660, laser_offset_show, laser_offset_store),
    __ATTR(laser_xtalk, 0660, laser_xtalk_show, laser_xtalk_store),
    __ATTR(laser_offset_calibrate, 0440, laser_offset_calibrate_show, NULL),
    __ATTR(laser_xtalk_calibrate, 0660, laser_xtalk_calibrate_show, laser_xtalk_calibrate_store),
    __ATTR(laser_cali_status, 0440, laser_cali_status_show, NULL),
#ifdef HTC_MODIFY
    __ATTR(enable_ps_sensor, 0664, stmvl53l0_show_enable_ps_sensor, stmvl53l0_store_enable_ps_sensor),
    __ATTR(enable_debug, 0660, stmvl53l0_show_enable_debug, stmvl53l0_store_enable_debug),
    __ATTR(enable_timing_debug, 0660, stmvl53l0_show_enable_timing_debug, stmvl53l0_store_enable_timing_debug),
    __ATTR(set_delay_ms, 0660, stmvl53l0_show_set_delay_ms, stmvl53l0_store_set_delay_ms),
#endif
};
#endif

/*
 * misc device file operation functions
 */
static int stmvl53l0_ioctl_handler(struct file *file,
			unsigned int cmd, unsigned long arg,
			void __user *p)
{
	int rc = 0;
	unsigned int xtalkint = 0;
	unsigned int targetDistance = 0;
	int8_t offsetint = 0;
	struct stmvl53l0_data *data =
			container_of(file->private_data,
				struct stmvl53l0_data, miscdev);
	struct stmvl53l0_register reg;
	struct stmvl53l0_parameter parameter;
	VL53L0_DEV vl53l0_dev = data;
	VL53L0_DeviceModes deviceMode;
	uint8_t page_num = 0;

	if (!data)
		return -EINVAL;

	vl53l0_dbgmsg("Enter enable_ps_sensor:%d\n", data->enable_ps_sensor);
	switch (cmd) {
	/* enable */
	case VL53L0_IOCTL_INIT:
		vl53l0_dbgmsg("VL53L0_IOCTL_INIT\n");
		/* turn on tof sensor only if it's not enabled by other client */
		if (data->enable_ps_sensor == 0) {
			/* to start */
			stmvl53l0_start(data, 3, NORMAL_MODE);
		} else
			rc = -EINVAL;
		break;
	/* crosstalk calibration */
	case VL53L0_IOCTL_XTALKCALB:
		vl53l0_dbgmsg("VL53L0_IOCTL_XTALKCALB\n");
		data->xtalkCalDistance = 100;
		if (copy_from_user(&targetDistance, (unsigned int *)p,
			sizeof(unsigned int))) {
			vl53l0_errmsg("%d, fail\n", __LINE__);
			return -EFAULT;
		}
		data->xtalkCalDistance = targetDistance;

		/* turn on tof sensor only if it's not enabled by other	client */
		if (data->enable_ps_sensor == 0) {
			/* to start */
			stmvl53l0_start(data, 3, XTALKCALIB_MODE);
		} else
			rc = -EINVAL;
		break;
	/* set up Xtalk value */
	case VL53L0_IOCTL_SETXTALK:
		vl53l0_dbgmsg("VL53L0_IOCTL_SETXTALK\n");
		if (copy_from_user(&xtalkint, (unsigned int *)p,
			sizeof(unsigned int))) {
			vl53l0_errmsg("%d, fail\n", __LINE__);
			return -EFAULT;
		}
		vl53l0_dbgmsg("SETXTALK as 0x%x\n", xtalkint);
#ifdef CALIBRATION_FILE
		xtalk_calib = xtalkint;
		stmvl53l0_write_xtalk_calibration_file();
#endif
/* later
		VL6180x_SetXTalkCompensationRate(vl53l0_dev, xtalkint);
*/
		break;
	/* offset calibration */
	case VL53L0_IOCTL_OFFCALB:
		vl53l0_dbgmsg("VL53L0_IOCTL_OFFCALB\n");
		data->offsetCalDistance = 50;
		if (copy_from_user(&targetDistance, (unsigned int *)p,
			sizeof(unsigned int))) {
			vl53l0_errmsg("%d, fail\n", __LINE__);
			return -EFAULT;
		}
		data->offsetCalDistance = targetDistance;
		if (data->enable_ps_sensor == 0) {
			/* to start */
			stmvl53l0_start(data, 3, OFFSETCALIB_MODE);
		} else
			rc = -EINVAL;
		break;
	/* set up offset value */
	case VL53L0_IOCTL_SETOFFSET:
		vl53l0_dbgmsg("VL53L0_IOCTL_SETOFFSET\n");
		if (copy_from_user(&offsetint, (int8_t *)p, sizeof(int8_t))) {
			vl53l0_errmsg("%d, fail\n", __LINE__);
			return -EFAULT;
		}
		vl53l0_dbgmsg("SETOFFSET as %d\n", offsetint);
#ifdef CALIBRATION_FILE
		offset_calib = offsetint;
		stmvl53l0_write_offset_calibration_file();
#endif
/* later
		VL6180x_SetOffsetCalibrationData(vl53l0_dev, offsetint);
*/
		break;
	/* disable */
	case VL53L0_IOCTL_STOP:
		vl53l0_dbgmsg("VL53L0_IOCTL_STOP\n");
		/* turn off tof sensor only if it's enabled by other client */
		if (data->enable_ps_sensor == 1) {
			data->enable_ps_sensor = 0;
			/* to stop */
			stmvl53l0_stop(data);
		}
		break;
	/* Get all range data */
	case VL53L0_IOCTL_GETDATAS:
		vl53l0_dbgmsg("VL53L0_IOCTL_GETDATAS\n");
		if (copy_to_user((VL53L0_RangingMeasurementData_t *)p,
			&(data->rangeData),
			sizeof(VL53L0_RangingMeasurementData_t))) {
			vl53l0_errmsg("%d, fail\n", __LINE__);
			return -EFAULT;
		}
		break;
	/* Register tool */
	case VL53L0_IOCTL_REGISTER:
		vl53l0_dbgmsg("VL53L0_IOCTL_REGISTER\n");
		if (copy_from_user(&reg, (struct stmvl53l0_register *)p,
			sizeof(struct stmvl53l0_register))) {
			vl53l0_errmsg("%d, fail\n", __LINE__);
			return -EFAULT;
		}
		reg.status = 0;
		page_num = (uint8_t)((reg.reg_index & 0x0000ff00) >> 8);
		vl53l0_dbgmsg(
"VL53L0_IOCTL_REGISTER,	page number:%d\n", page_num);
		if (page_num != 0)
			reg.status = VL53L0_WrByte(vl53l0_dev, 0xFF, page_num);

		switch (reg.reg_bytes) {
		case(4):
			if (reg.is_read)
				reg.status = VL53L0_RdDWord(vl53l0_dev,
					(uint8_t)reg.reg_index,
					&reg.reg_data);
			else
				reg.status = VL53L0_WrDWord(vl53l0_dev,
					(uint8_t)reg.reg_index,
					reg.reg_data);
			break;
		case(2):
			if (reg.is_read)
				reg.status = VL53L0_RdWord(vl53l0_dev,
					(uint8_t)reg.reg_index,
					(uint16_t *)&reg.reg_data);
			else
				reg.status = VL53L0_WrWord(vl53l0_dev,
					(uint8_t)reg.reg_index,
					(uint16_t)reg.reg_data);
			break;
		case(1):
			if (reg.is_read)
				reg.status = VL53L0_RdByte(vl53l0_dev,
					(uint8_t)reg.reg_index,
					(uint8_t *)&reg.reg_data);
			else
				reg.status = VL53L0_WrByte(vl53l0_dev,
					(uint8_t)reg.reg_index,
					(uint8_t)reg.reg_data);
			break;
		default:
			reg.status = -1;

		}
		if (page_num != 0)
			reg.status = VL53L0_WrByte(vl53l0_dev, 0xFF, 0);


		if (copy_to_user((struct stmvl53l0_register *)p, &reg,
				sizeof(struct stmvl53l0_register))) {
			vl53l0_errmsg("%d, fail\n", __LINE__);
			return -EFAULT;
		}
		break;
	/* parameter access */
	case VL53L0_IOCTL_PARAMETER:
		vl53l0_dbgmsg("VL53L0_IOCTL_PARAMETER\n");
		if (copy_from_user(&parameter, (struct stmvl53l0_parameter *)p,
				sizeof(struct stmvl53l0_parameter))) {
			vl53l0_errmsg("%d, fail\n", __LINE__);
			return -EFAULT;
		}
		parameter.status = 0;
		switch (parameter.name) {
		case (OFFSET_PAR):
			if (parameter.is_read)
				parameter.status =
					papi_func_tbl->GetOffsetCalibrationDataMicroMeter(
						vl53l0_dev, &parameter.value);
			else
				parameter.status =
					papi_func_tbl->SetOffsetCalibrationDataMicroMeter(
						vl53l0_dev, parameter.value);
			vl53l0_dbgmsg("get parameter value as %d\n", parameter.value);
			break;
		case (XTALKRATE_PAR):
			if (parameter.is_read)
				parameter.status =
					papi_func_tbl->GetXTalkCompensationRateMegaCps(
						vl53l0_dev, (FixPoint1616_t *)
						&parameter.value);
			else
				parameter.status =
					papi_func_tbl->SetXTalkCompensationRateMegaCps(
						vl53l0_dev,
						(FixPoint1616_t)
							parameter.value);

			break;
		case (XTALKENABLE_PAR):
			if (parameter.is_read)
				parameter.status =
					papi_func_tbl->GetXTalkCompensationEnable(
						vl53l0_dev,
						(uint8_t *) &parameter.value);
			else
				parameter.status =
					papi_func_tbl->SetXTalkCompensationEnable(
						vl53l0_dev,
						(uint8_t) parameter.value);
			break;
		case (GPIOFUNC_PAR):
			if (parameter.is_read) {
				parameter.status =
					papi_func_tbl->GetGpioConfig(vl53l0_dev, 0, &deviceMode,
						&data->gpio_function,
						&data->gpio_polarity);
				parameter.value = data->gpio_function;
			} else {
				data->gpio_function = parameter.value;
				parameter.status =
					papi_func_tbl->SetGpioConfig(vl53l0_dev, 0, 0,
						data->gpio_function,
						data->gpio_polarity);
			}
			break;
		case (LOWTHRESH_PAR):
			if (parameter.is_read) {
				parameter.status =
					papi_func_tbl->GetInterruptThresholds(vl53l0_dev, 0,
					&(data->low_threshold), &(data->high_threshold));
				parameter.value = data->low_threshold >> 16;
			} else {
				data->low_threshold = parameter.value << 16;
				parameter.status =
					papi_func_tbl->SetInterruptThresholds(vl53l0_dev, 0,
					data->low_threshold, data->high_threshold);
			}
			break;
		case (HIGHTHRESH_PAR):
			if (parameter.is_read) {
				parameter.status =
					papi_func_tbl->GetInterruptThresholds(vl53l0_dev, 0,
					&(data->low_threshold), &(data->high_threshold));
				parameter.value = data->high_threshold >> 16;
			} else {
				data->high_threshold = parameter.value << 16;
				parameter.status =
					papi_func_tbl->SetInterruptThresholds(vl53l0_dev, 0,
					data->low_threshold, data->high_threshold);
			}
			break;
		case (DEVICEMODE_PAR):
			if (parameter.is_read) {
				parameter.status =
					papi_func_tbl->GetDeviceMode(vl53l0_dev,
						(VL53L0_DeviceModes *)&(parameter.value));
			} else {
				parameter.status =
					papi_func_tbl->SetDeviceMode(vl53l0_dev,
						(VL53L0_DeviceModes)(parameter.value));
				data->deviceMode = (VL53L0_DeviceModes)(parameter.value);
			}
			break;



		case (INTERMEASUREMENT_PAR):
			if (parameter.is_read) {
				parameter.status =
					papi_func_tbl->GetInterMeasurementPeriodMilliSeconds(vl53l0_dev,
						(uint32_t *)&(parameter.value));
			} else {
				parameter.status =
					papi_func_tbl->SetInterMeasurementPeriodMilliSeconds(vl53l0_dev,
						(uint32_t)(parameter.value));
				data->interMeasurems = parameter.value;
			}
			break;


		}

		if (copy_to_user((struct stmvl53l0_parameter *)p, &parameter,
				sizeof(struct stmvl53l0_register))) {
			vl53l0_errmsg("%d, fail\n", __LINE__);
			return -EFAULT;
		}
		break;
#ifdef HTC
	case VL53L0_IOCTL_GET_DATA:
		{
			VL53L0_RangingMeasurementData_t RangingMeasurementData;
			VL53L0_Error Status = VL53L0_ERROR_NONE;
			uint32_t RangeSum = 0;
			uint32_t RateSum = 0;
			int i = 0;
			int measure_times = 0;

			for (i = 0; i < RANGE_MEASUREMENT_TIMES;)
			{
				Status = papi_func_tbl->PerformSingleRangingMeasurement(vl53l0_dev, &RangingMeasurementData);
				//Status = VL53L0_PerformSingleRangingMeasurement(&MyDevice, &RangingMeasurementData);
				if (Status == VL53L0_ERROR_NONE && RangingMeasurementData.RangeMilliMeter < RANGE_MEASUREMENT_OVERFLOW) {
					i++;
					RangeSum += RangingMeasurementData.RangeMilliMeter;
					RateSum += RangingMeasurementData.SignalRateRtnMegaCps;
				}
				measure_times++;
				if (measure_times > RANGE_MEASUREMENT_RETRY_TIMES) {
					vl53l0_errmsg("Failed to get measurement data\n");
					return -1;
				}
			}

			// Return avg, data
			vl53l0_dbgmsg("Sum of measurement data = (0x%X , 0x%X)", RangeSum, RateSum);
			RangingMeasurementData.RangeMilliMeter = RangeSum / RANGE_MEASUREMENT_TIMES;
			RangingMeasurementData.SignalRateRtnMegaCps = RateSum / RANGE_MEASUREMENT_TIMES;

			if (copy_to_user(p , &RangingMeasurementData , sizeof(VL53L0_RangingMeasurementData_t)))
			{
				vl53l0_errmsg("copy to user failed in VL53L0_IOCTL_GET_DATA\n");
				return -1;
			}
		}
		break;

	case VL53L0_IOCTL_OFFSET_CALI:
		{
			vl53l0_dbgmsg("VL53L0_IOCTL_OFFSET_CALI\n");

			data->offsetCalDistance = OFFSET_CALI_TARGET_DISTANCE;
			if (data->enable_ps_sensor == 0) {
				/* to start */
				stmvl53l0_start(data, 3, OFFSETCALIB_MODE);
			} else
				rc = -EINVAL;
#if 0
			ret = Laser_offset_calibrate(&offset_value, data);
			if (ret == 0)
			{
				vl53l0_dbgmsg("Offset calibration finished. Offset = %d mm\n", offset_value);
			}
			else
			{
				vl53l0_errmsg("Failed in offset calibration\n");
				return -1;
			}
#endif
			vl53l0_dbgmsg("Offset calibration finished. Offset = %d mm\n",
									g_offsetMicroMeter);

			if(copy_to_user(p, &g_offsetMicroMeter, sizeof(int))) {
				vl53l0_errmsg("copy to user failed in VL53L0_IOCTL_OFFSET_CALI\n");
				return -1;
			}
		}
		break;
	case VL53L0_IOCTL_XTALK_CALI:
		{
			VL53L0_Error Status = VL53L0_ERROR_NONE;
			FixPoint1616_t XTalkCompensationRateMegaCps = 0;
			vl53l0_dbgmsg("VL53L0_IOCTL_XTALK_CALI\n");

	                if (data->enable_ps_sensor == 0) {
        	                /* to start */
                	        stmvl53l0_start(data, 3, XTALKCALIB_MODE);
	                } else
        	                rc = -EINVAL;
#if 0
			ret = Laser_xtalk_calibrate(&XTalkCompensationRateMegaCps, data);
			if (ret == 0)
			{
				vl53l0_dbgmsg("Xtalk calibration finished. "
					      "XTalkCompensationRateMegaCps = 0x%X with distance = %d mm\n",
						XTalkCompensationRateMegaCps, (data->cali_distance>>16));
			}
			else
			{
				vl53l0_errmsg("Failed in xtalk calibration\n");
				return -1;
			}
#endif
			// read out xtalk value and return
			Status = papi_func_tbl->GetXTalkCompensationRateMegaCps(vl53l0_dev,
								&XTalkCompensationRateMegaCps);
			if (Status != VL53L0_ERROR_NONE) {
				vl53l0_errmsg("Failed in VL53L0_GetXTalkCompensationRateMegaCps\n");
				return -1;
			}
			vl53l0_dbgmsg("Xtalk calibration finished. "
				      "XTalkCompensationRateMegaCps = 0x%X(0x%X) with distance = %d mm\n",
						XTalkCompensationRateMegaCps, g_XTalkCompensationRateMegaCps,
						data->xtalkCalDistance);

			if(copy_to_user(p, &XTalkCompensationRateMegaCps, sizeof(int))) {
				vl53l0_errmsg("copy to user failed in VL53L0_IOCTL_XTALK_CALI\n");
				return -1;
			}
		}
		break;
	case VL53L0_IOCTL_SET_XTALK_CALI_DISTANCE:
		{
			int distance_mm = 0;
			if(copy_from_user(&distance_mm , (int *)p , sizeof(int))) {
				vl53l0_errmsg("copy from user failed in VL53L0_IOCTL_SET_XTALK_DISTANCE\n");
				return -1;
			}

			data->xtalkCalDistance = distance_mm;
			vl53l0_dbgmsg("Set distance for xtalk calibration to %d mm\n", distance_mm);
		}
		break;
#endif
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int stmvl53l0_open(struct inode *inode, struct file *file)
{
	return 0;
}

#if 0
static int stmvl53l0_flush(struct file *file, fl_owner_t id)
{
	struct stmvl53l0_data *data = container_of(file->private_data,
					struct stmvl53l0_data, miscdev);
	(void) file;
	(void) id;

	if (data) {
		if (data->enable_ps_sensor == 1) {
			/* turn off tof sensor if it's enabled */
			data->enable_ps_sensor = 0;
			/* to stop */
			stmvl53l0_stop(data);
		}
	}
	return 0;
}
#endif
static long stmvl53l0_ioctl(struct file *file,
				unsigned int cmd, unsigned long arg)
{
	long ret;
	struct stmvl53l0_data *data = container_of(file->private_data,
					struct stmvl53l0_data, miscdev);
	mutex_lock(&data->work_mutex);
	ret = stmvl53l0_ioctl_handler(file, cmd, arg, (void __user *)arg);
	mutex_unlock(&data->work_mutex);

	return ret;
}

/*
 * Initialization function
 */
static int stmvl53l0_init_client(struct stmvl53l0_data *data)
{
	uint8_t id = 0, type = 0;
	uint8_t revision = 0, module_id = 0;
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	VL53L0_DeviceInfo_t DeviceInfo;
	VL53L0_DEV vl53l0_dev = data;
	FixPoint1616_t	LimitValue;
	uint8_t LimitEnable;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;

	vl53l0_dbgmsg("Enter\n");

	/* Read Model ID */
	VL53L0_RdByte(vl53l0_dev, VL53L0_REG_IDENTIFICATION_MODEL_ID, &id);
	vl53l0_errmsg("read MODLE_ID: 0x%x\n", id);
	if (id == 0xee) {
		vl53l0_errmsg("STM VL53L0 Found\n");
	} else if (id == 0) {
		vl53l0_errmsg("Not found STM VL53L0\n");
		return -EIO;
	}
	VL53L0_RdByte(vl53l0_dev, 0xc1, &id);
	vl53l0_errmsg("read 0xc1: 0x%x\n", id);
	VL53L0_RdByte(vl53l0_dev, 0xc2, &id);
	vl53l0_errmsg("read 0xc2: 0x%x\n", id);
	VL53L0_RdByte(vl53l0_dev, 0xc3, &id);
	vl53l0_errmsg("read 0xc3: 0x%x\n", id);

	/* Read Model Version */
	VL53L0_RdByte(vl53l0_dev, 0xC0, &type);
	VL53L0_RdByte(vl53l0_dev, VL53L0_REG_IDENTIFICATION_REVISION_ID,
		&revision);
	VL53L0_RdByte(vl53l0_dev, 0xc3,
		&module_id);
	vl53l0_errmsg("STM VL53L0 Model type : %d. rev:%d. module:%d\n", type,
		revision, module_id);

	vl53l0_dev->I2cDevAddr      = 0x52;
	vl53l0_dev->comms_type      =  1;
	vl53l0_dev->comms_speed_khz =  400;

	/* Setup API functions based on revision */
	stmvl53l0_setupAPIFunctions(data);

	if (Status == VL53L0_ERROR_NONE && data->reset) {
		pr_err("Call of VL53L0_DataInit\n");
		Status = papi_func_tbl->DataInit(vl53l0_dev); /* Data initialization */
		//data->reset = 0;
	}

	if (Status == VL53L0_ERROR_NONE) {
		pr_err("VL53L0_GetDeviceInfo:\n");
		Status = papi_func_tbl->GetDeviceInfo(vl53l0_dev, &DeviceInfo);
		if (Status == VL53L0_ERROR_NONE) {
			pr_err("Device Name : %s\n", DeviceInfo.Name);
			pr_err("Device Type : %s\n", DeviceInfo.Type);
			pr_err("Device ID : %s\n", DeviceInfo.ProductId);
			pr_err("Product type: %d\n", DeviceInfo.ProductType);
			pr_err("ProductRevisionMajor : %d\n",
				DeviceInfo.ProductRevisionMajor);
			pr_err("ProductRevisionMinor : %d\n",
				DeviceInfo.ProductRevisionMinor);
		}
	}

	if (Status == VL53L0_ERROR_NONE) {
		pr_err("Call of VL53L0_StaticInit\n");
		Status = papi_func_tbl->StaticInit(vl53l0_dev);
		/* Device Initialization */
	}

	if (Status == VL53L0_ERROR_NONE && data->reset) {
		if (papi_func_tbl->PerformRefCalibration != NULL) {
		pr_err("Call of VL53L0_PerformRefCalibration\n");
		Status = papi_func_tbl->PerformRefCalibration(vl53l0_dev,
				&VhvSettings, &PhaseCal); /* Ref calibration */
		}
	}

	if (Status == VL53L0_ERROR_NONE && data->reset) {
		if (papi_func_tbl->PerformRefSpadManagement != NULL) {
		pr_err("Call of VL53L0_PerformRefSpadManagement\n");
		Status = papi_func_tbl->PerformRefSpadManagement(vl53l0_dev,
				&refSpadCount, &isApertureSpads); /* Ref Spad Management */
		}
		data->reset = 0;
	}

	if (Status == VL53L0_ERROR_NONE) {
		/* Setup in single ranging mode */
		pr_err("Call of VL53L0_SetDeviceMode\n");
		Status = papi_func_tbl->SetDeviceMode(vl53l0_dev,
					VL53L0_DEVICEMODE_SINGLE_RANGING);
	}
	if (Status == VL53L0_ERROR_NONE) {
		Status = papi_func_tbl->GetLimitCheckValue(vl53l0_dev,
					VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE,
					&LimitValue);
		Status = papi_func_tbl->GetLimitCheckEnable(vl53l0_dev,
					VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE,
					&LimitEnable);
		pr_err("Get LimitCheckValue SIGMA_FINAL_RANGE as:%d,Enable:%d\n",
				(LimitValue>>16), LimitEnable);
	}

	if (Status == VL53L0_ERROR_NONE) {
		Status = papi_func_tbl->GetLimitCheckValue(vl53l0_dev,
					VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
					&LimitValue);
		Status = papi_func_tbl->GetLimitCheckEnable(vl53l0_dev,
					VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
					&LimitEnable);
		pr_err("Get LimitCheckValue SIGNAL_FINAL_RANGE as:%d(Fix1616),Eanble:%d\n",
				(LimitValue), LimitEnable);
	}

	if (Status == VL53L0_ERROR_NONE) {
		Status = papi_func_tbl->GetLimitCheckValue(vl53l0_dev,
					VL53L0_CHECKENABLE_SIGNAL_REF_CLIP,
					&LimitValue);
		Status = papi_func_tbl->GetLimitCheckEnable(vl53l0_dev,
					VL53L0_CHECKENABLE_SIGNAL_REF_CLIP,
					&LimitEnable);
		pr_err("Get LimitCheckValue SIGNAL_REF_CLIP as:%d(fix1616),Enable:%d\n",
				(LimitValue), LimitEnable);
	}

	if (Status == VL53L0_ERROR_NONE) {
		Status = papi_func_tbl->GetLimitCheckValue(vl53l0_dev,
					VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
					&LimitValue);
		Status = papi_func_tbl->GetLimitCheckEnable(vl53l0_dev,
					VL53L0_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
					&LimitEnable);
		pr_err("Get LimitCheckValue RANGE_IGNORE_THRESHOLD as:%d(fix1616),Enable:%d\n",
				(LimitValue), LimitEnable);
	}
#if 0
	if (Status == VL53L0_ERROR_NONE) {
		pr_err("set LimitCheckValue SIGMA_FINAL_RANGE\n");
		LimitValue = 30 << 16;
		Status = papi_func_tbl->SetLimitCheckValue(vl53l0_dev,
					VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE,
					LimitValue);
	}

	if (Status == VL53L0_ERROR_NONE) {
		pr_err("set LimitCheckValue SIGNAL_RATE_FINAL_RANGE\n");
		//SigmaLimitValue = 94743; /* 1.44567500 * 65536 */
		LimitValue = 553495; /* 8.44567500 * 65536 */
		Status = papi_func_tbl->SetLimitCheckValue(vl53l0_dev,
					VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
					LimitValue);
	}

	/*  Enable/Disable Sigma and Signal check */
	if (Status == VL53L0_ERROR_NONE)
		Status = papi_func_tbl->SetLimitCheckEnable(vl53l0_dev,
					VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE, 1);

	if (Status == VL53L0_ERROR_NONE)
		Status = papi_func_tbl->SetLimitCheckEnable(vl53l0_dev,
					VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 0);
#endif
	if (Status == VL53L0_ERROR_NONE)
		Status = papi_func_tbl->SetWrapAroundCheckEnable(vl53l0_dev, 1);

#ifdef CALIBRATION_FILE
	/*stmvl53l0_read_calibration_file(data);*/
#endif

	vl53l0_dbgmsg("End\n");

	return 0;
}

static int stmvl53l0_start(struct stmvl53l0_data *data,
			   uint8_t scaling, init_mode_e mode)
{
	int rc = 0;
	VL53L0_DEV vl53l0_dev = data;

	vl53l0_dbgmsg("Enter++\n");

	/* Power up */
	rc = pmodule_func_tbl->power_up(data->client_object, &data->reset);
	if (rc) {
		vl53l0_errmsg("%d,error rc %d\n", __LINE__, rc);
		return rc;
	}
#ifdef HTC
	/* Pull high the power-down pin */
        rc = gpio_direction_output(data->pwdn_gpio, 1);
        if (rc) {
                vl53l0_errmsg("Failed to pull up pwdn_gpio\n");
                return rc;
        }
        msleep(2);
#endif
	/* init */
	rc = stmvl53l0_init_client(data);
	if (rc) {
		vl53l0_errmsg("%d, error rc %d\n", __LINE__, rc);
		pmodule_func_tbl->power_down(data->client_object);
#ifdef HTC
        	rc = gpio_direction_output(data->pwdn_gpio, 0);
		if (rc) {
			vl53l0_errmsg("Failed to pull low pwdn_gpio\n");
			return rc;
		}
#endif
		return -EINVAL;
	}
	/* check mode */
	if (mode != NORMAL_MODE)
		papi_func_tbl->SetXTalkCompensationEnable(vl53l0_dev, 0);

	if (mode == OFFSETCALIB_MODE) {
		//VL53L0_SetOffsetCalibrationDataMicroMeter(vl53l0_dev, 0);
		int32_t OffsetMicroMeter = 0;
		papi_func_tbl->PerformOffsetCalibration(vl53l0_dev,
						(data->offsetCalDistance<<16),
						&OffsetMicroMeter);
		g_offsetMicroMeter = OffsetMicroMeter/1000;
		data->offset_kvalue = g_offsetMicroMeter;
		vl53l0_dbgmsg("Offset calibration:%u\n", OffsetMicroMeter);
		return rc;
	} else if (mode == XTALKCALIB_MODE) {
		FixPoint1616_t XTalkCompensationRateMegaCps = 0;
		/*
		 * caltarget distance : 100mm and convert to
		 * fixed point 16 16 format
		 */
		papi_func_tbl->PerformXTalkCalibration(vl53l0_dev,
						(data->xtalkCalDistance<<16),
						&XTalkCompensationRateMegaCps);
		g_XTalkCompensationRateMegaCps = XTalkCompensationRateMegaCps;
		data->xtalk_kvalue = g_XTalkCompensationRateMegaCps;
		vl53l0_dbgmsg("Xtalk calibration:%u\n", XTalkCompensationRateMegaCps);
		return rc;
	}
	/* set up device parameters */
	data->gpio_polarity = VL53L0_INTERRUPTPOLARITY_LOW;
	papi_func_tbl->SetGpioConfig(vl53l0_dev, 0, 0, data->gpio_function,
							VL53L0_INTERRUPTPOLARITY_LOW);
	papi_func_tbl->SetInterruptThresholds(vl53l0_dev, 0, data->low_threshold,
							data->high_threshold);
	papi_func_tbl->SetInterMeasurementPeriodMilliSeconds(vl53l0_dev,
							data->interMeasurems);
	vl53l0_dbgmsg("DeviceMode:0x%x, interMeasurems:%d==\n",
					data->deviceMode, data->interMeasurems);
	papi_func_tbl->SetDeviceMode(vl53l0_dev, data->deviceMode);
	papi_func_tbl->ClearInterruptMask(vl53l0_dev, 0);

	/* start the ranging */
	papi_func_tbl->StartMeasurement(vl53l0_dev);
	data->enable_ps_sensor = 1;
#ifdef HTC
	enable_irq(data->irq);
#endif
	/* enable work handler */
	stmvl53l0_schedule_handler(data);
	vl53l0_dbgmsg("End--\n");

	return rc;
}

static int stmvl53l0_stop(struct stmvl53l0_data *data)
{
	int rc = 0;
	VL53L0_DEV vl53l0_dev = data;

	vl53l0_dbgmsg("Enter++\n");

#ifdef HTC
	disable_irq(data->irq);
#endif
	/* stop - if continuous mode */
	if (data->deviceMode == VL53L0_DEVICEMODE_CONTINUOUS_RANGING ||
	    data->deviceMode == VL53L0_DEVICEMODE_CONTINUOUS_TIMED_RANGING) {
	    	papi_func_tbl->StopMeasurement(vl53l0_dev);
	}
	/* clean interrupt */
	papi_func_tbl->ClearInterruptMask(vl53l0_dev, 0);

	/* cancel work handler */
	stmvl53l0_cancel_handler(data);

	/* power down */
	rc = pmodule_func_tbl->power_down(data->client_object);
	if (rc) {
		vl53l0_errmsg("%d, error rc %d\n", __LINE__, rc);
		return rc;
	}
#ifdef HTC
	/* pull low the power-down pin */
        rc = gpio_direction_output(data->pwdn_gpio, 0);
        if (rc) {
                vl53l0_errmsg("Failed to pull up pwdn_gpio\n");
                return rc;
        }
#endif
	data->enable_ps_sensor = 0;
	vl53l0_dbgmsg("End--\n");

	return rc;
}

/*
 * I2C init/probing/exit functions
 */
static const struct file_operations stmvl53l0_ranging_fops = {
	.owner 		=	THIS_MODULE,
	.unlocked_ioctl =	stmvl53l0_ioctl,
	.open 		=	stmvl53l0_open,
	//.flush 	=	stmvl53l0_flush,
};

/*
static struct miscdevice stmvl53l0_ranging_dev = {
	.minor =	MISC_DYNAMIC_MINOR,
	.name =		"stmvl53l0_ranging",
	.fops =		&stmvl53l0_ranging_fops
};
*/

int stmvl53l0_setup(struct stmvl53l0_data *data)
{
	int rc = 0;
#ifdef HTC
	int attr_cnt;
#endif
	vl53l0_dbgmsg("Enter++\n");

	/* init mutex */
	mutex_init(&data->update_lock);
	mutex_init(&data->work_mutex);

#ifdef USE_INT
	/* init interrupt */
	if (gpio_is_valid(data->laser_irq_gpio)) {
		rc = gpio_request(data->laser_irq_gpio, "vl53l0_gpio_int");
		if (rc) {
			vl53l0_errmsg("request irq gpio fail\n");
		}
		gpio_direction_input(data->laser_irq_gpio);
	} else {
		E("irq_gpio is not valid\n");
	}
	data->irq = gpio_to_irq(data->laser_irq_gpio);
	if (data->irq < 0) {
		vl53l0_errmsg("filed to map GPIO: %d to interrupt: %d\n",
					data->laser_irq_gpio, data->irq);
	} else {
		vl53l0_dbgmsg("register_irq: %d\n", data->irq);
		/* IRQF_TRIGGER_FALLING	- poliarity:0
 		 * IRQF_TRIGGER_RISNG	- poliarity:1 */
		rc = request_threaded_irq(data->irq, NULL, stmvl53l0_interrupt_handler,
					  //IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					  IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					  "vl53l0_interrupt", (void *)data);
		if (rc) {
			vl53l0_errmsg("%d, Could not allocate STMVL53L0_INT"
				      " ! result:%d\n", __LINE__, rc);
			free_irq(data->irq, data);
			goto exit_free_irq;
		}
	}
	vl53l0_errmsg("interrupt is hooked\n");
#ifdef HTC
	disable_irq(data->irq);
#endif
#endif
	/* init work handler */
	INIT_DELAYED_WORK(&data->dwork, stmvl53l0_work_handler);

	/* Register to Input Device */
	data->input_dev_ps = input_allocate_device();
	if (!data->input_dev_ps) {
		rc = -ENOMEM;
		vl53l0_errmsg("%d error:%d\n", __LINE__, rc);
		goto exit_free_irq;
	}
	set_bit(EV_ABS, data->input_dev_ps->evbit);
	/* range in cm*/
	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 76, 0, 0);
	/* tv_sec */
	input_set_abs_params(data->input_dev_ps, ABS_HAT0X, 0, 0xffffffff, 0, 0);
	/* tv_usec */
	input_set_abs_params(data->input_dev_ps, ABS_HAT0Y, 0, 0xffffffff, 0, 0);
	/* range in_mm */
	input_set_abs_params(data->input_dev_ps, ABS_HAT1X, 0, 765, 0, 0);
	/* error code change maximum to 0xff for more flexibility */
	input_set_abs_params(data->input_dev_ps, ABS_HAT1Y, 0, 0xff, 0, 0);
	/* rtnRate */
	input_set_abs_params(data->input_dev_ps, ABS_HAT2X, 0, 0xffffffff, 0, 0);
	/* rtn_amb_rate */
	input_set_abs_params(data->input_dev_ps, ABS_HAT2Y, 0, 0xffffffff, 0, 0);
	/* rtn_conv_time */
	input_set_abs_params(data->input_dev_ps, ABS_HAT3X, 0, 0xffffffff, 0, 0);
	/* dmax */
	input_set_abs_params(data->input_dev_ps, ABS_HAT3Y, 0, 0xffffffff, 0, 0);

	data->input_dev_ps->name = "STM VL53L0 proximity sensor";

	rc = input_register_device(data->input_dev_ps);
	if (rc) {
		rc = -ENOMEM;
		vl53l0_errmsg("%d error:%d\n", __LINE__, rc);
		goto exit_free_dev_ps;
	}
	/* setup drv data */
	input_set_drvdata(data->input_dev_ps, data);

	/* Register sysfs hooks */
	data->range_kobj = kobject_create_and_add("range", kernel_kobj);
	if (!data->range_kobj) {
		rc = -ENOMEM;
		vl53l0_errmsg("%d error:%d\n", __LINE__, rc);
		goto exit_unregister_dev_ps;
	}
#if 0
	rc = sysfs_create_group(&data->input_dev_ps->dev.kobj,
			&stmvl53l0_attr_group);
	if (rc) {
		rc = -ENOMEM;
		vl53l0_errmsg("%d error:%d\n", __LINE__, rc);
		goto exit_unregister_dev_ps_1;
	}
#endif
	/* to register as a misc device */
	data->miscdev.minor = MISC_DYNAMIC_MINOR;
#ifdef HTC
	data->miscdev.name = "laser_stmvl53l0";
#endif
	data->miscdev.fops = &stmvl53l0_ranging_fops;
	vl53l0_errmsg("Misc device registration name:%s\n", data->dev_name);
	if (misc_register(&data->miscdev) != 0)
		vl53l0_errmsg("Could not register misc. dev for stmvl53l0 ranging\n");
#ifdef HTC
	data->laser_class = class_create(THIS_MODULE, "htc_laser");
	if (IS_ERR(data->laser_class)) {
		goto exit_unregister_dev_ps_1;
	}

	data->laser_dev = device_create(data->laser_class, NULL, 0, "%s", "laser");
	if (IS_ERR(data->laser_dev)) {
		goto exit_unregister_dev_ps_1;
	}
	dev_set_drvdata(data->laser_dev, data);

	for (attr_cnt = 0; attr_cnt < ARRAY_SIZE(attributes); attr_cnt++) {
		if (device_create_file(data->laser_dev, attributes + attr_cnt))
			goto exit_err_file_create;
	}
	/* init default device parameter value */
	data->enableTimingDebug = 0;
#endif
	data->enable_ps_sensor = 0;
	data->reset = 1;
	data->delay_ms = 30;	/* delay time to 30ms */
	data->enableDebug = 0;
	data->gpio_polarity = VL53L0_INTERRUPTPOLARITY_LOW;
	data->gpio_function = VL53L0_GPIOFUNCTIONALITY_NEW_MEASURE_READY;
	data->low_threshold = 60;
	data->high_threshold = 200;
	data->deviceMode = VL53L0_DEVICEMODE_SINGLE_RANGING;
	data->interMeasurems = 30;

	vl53l0_dbgmsg("support ver. %s enabled\n", DRIVER_VERSION);
	vl53l0_dbgmsg("End--");

	return 0;
#ifdef HTC
exit_err_file_create:
	while (--attr_cnt >= 0)
		device_remove_file(data->laser_dev, attributes + attr_cnt);
	device_unregister(data->laser_dev);
#endif
exit_unregister_dev_ps_1:
	kobject_put(data->range_kobj);
exit_unregister_dev_ps:
	input_unregister_device(data->input_dev_ps);
exit_free_dev_ps:
	input_free_device(data->input_dev_ps);
exit_free_irq:
#ifdef USE_INT
	free_irq(data->irq, data);
#endif
	kfree(data);
	return rc;
}

static int __init stmvl53l0_init(void)
{
	int ret = -1;

	vl53l0_dbgmsg("Enter\n");
	/* assign function table */
	pmodule_func_tbl = &stmvl53l0_module_func_tbl;
	papi_func_tbl = &stmvl53l0_api_func_tbl;

	/* client specific init function */
	ret = pmodule_func_tbl->init();

	if (ret)
		vl53l0_errmsg("%d failed with %d\n", __LINE__, ret);

	vl53l0_dbgmsg("End\n");

	return ret;
}

static void __exit stmvl53l0_exit(void)
{
	vl53l0_dbgmsg("Enter\n");

	vl53l0_dbgmsg("End\n");
}


MODULE_AUTHOR("STMicroelectronics Imaging Division");
MODULE_DESCRIPTION("ST FlightSense Time-of-Flight sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(stmvl53l0_init);
module_exit(stmvl53l0_exit);

