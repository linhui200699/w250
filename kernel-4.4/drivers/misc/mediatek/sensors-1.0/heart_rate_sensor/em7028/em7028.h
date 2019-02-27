/* 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * Definitions for em3071 als/ps sensor chip.
 */
#ifndef __EM7028_H__
#define __EM7028_H__

#include <linux/ioctl.h>

#define EM7028_CMM_ID					0X00
#define EM7028_CMM_ENABLE 				0X01
#define EM7028_CMM_STATUS				0x02
#define EM7028_CMM_INT_PS_LB			0X03
#define EM7028_CMM_INT_PS_HB  			0X04
#define EM7028_CMM_OFFSET				0X08
#define EM7028_CMM_HRS2_CTRL 			0X09
#define EM7028_CMM_GAIN 				0X0A
#define EM7028_CMM_CTR					0x0d
#define EM7028_CMM_INT_CTRL				0x0e

#define EM7028_CMM_HRS2DATAL			0x20
#define EM7028_CMM_HRS2DATAH			0x21

#define EM7028_CMM_HRSDATAL				0x30
#define EM7028_CMM_HRSDATAH				0x31
#define EM7028_SUCCESS					0
#define EM7028_ERR_I2C					-1

extern  int em70xx_bpm_dynamic(int RECEIVED_BYTE, int g_sensor_x, int g_sensor_y, int g_sensor_z);
extern  int em70xx_reset(int i);
#endif
