/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/

#ifndef __VOW_API_H__
#define __VOW_API_H__
int VowDrv_ChangeStatus(void);
int VowDrv_EnableHW(int status);
void VowDrv_SetDmicLowPower(bool enable);
void VowDrv_SetSmartDevice(bool enable);
void VowDrv_SetSmartDevice_GPIO(bool enable);
void VowDrv_SetPeriodicEnable(bool enable);
void VowDrv_SetMtkifType(unsigned int type);
#endif
