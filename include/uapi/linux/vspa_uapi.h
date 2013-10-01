/*
 * include/uapi/linux/vspa_uapi.h
 * VSPA device driver
 * Author: Vineet Sharma b44341@freeescale.com
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __VSPA_UAPI_H_
#define __VSPA_UAPI_H_

#define VSP_MAGIC_NUM 'V'
#define IOCTL_REQ_IRQ   _IOR(VSP_MAGIC_NUM, 1, int)
#define IOCTL_REQ_PDN   _IOR(VSP_MAGIC_NUM, 2, int)
#define IOCTL_REQ_PUP   _IOR(VSP_MAGIC_NUM, 3, int)

/* mmap offset argument for vspa regsiters */
#define VSPA_REG_OFFSET	0

/* mmap offset argument for dbg regsiter */
#define VSPA_DBG_OFFSET	4096

/* mmap offset argument for vspa device context memory */
#define VSPA_DS_OFFSET	8192

#endif
