/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


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
