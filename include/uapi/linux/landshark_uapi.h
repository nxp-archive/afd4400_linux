/* include/uapi/linux/landshark_uapi.h
 *
 * Landshark Driver
 * This driver is designed for support to user space Landshark memory
 * allocator Library.
 *
 * Author: Arpit Goel
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 *  */

#ifndef __LANDSHARK_UAPI_H
#define __LANDSHARK_UAPI_H


#define LANDSHARK_MAGIC_NUM 'M'
#define LANDSHARK_MEM_INFO_GET _IOW(LANDSHARK_MAGIC_NUM, 1, int)

/* Get if the Ls persitant memory has been intialized
 * by the user process
*/
#define LANDSHARK_INIT_INFO_GET _IOW(LANDSHARK_MAGIC_NUM, 2, int)

/* Allows process to notify to the driver about the
 * presistant memory intialization
*/
#define LANDSHARK_INIT_INFO_SET _IOW(LANDSHARK_MAGIC_NUM, 3, int)

/* Maintains all the information about the metadata memory */
struct pinit_info {

	/* Memory buffer for LS Meta data */
	unsigned int nondma_memory;

	/* Metadata intialization done by the application ? */
	unsigned int pinit_done;

	/* Size of the Metadata memory as required by the application */
	unsigned int pinit_size;
};

#endif /* __LANDSHARK_UAPI_H */
