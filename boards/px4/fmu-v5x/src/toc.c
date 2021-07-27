/****************************************************************************
 *
 *   Copyright (C) 2020 Technology Innovation Institute. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include <image_toc.h>

/* (Maximum) size of the signature */
#define SIGNATURE_SIZE 64

/* Boot image starts at _vectors and ends at
 * the beginning of signature
*/

extern uint32_t _vectors[];
extern const int *_boot_signature;

#define APP_ADDR_START _vectors
#define APP_ADDR_END ((const void *)&_boot_signature)

/* Boot signature start and end are defined by the
 * signature definition below
*/

#define MSTR_SIG_ADDR_START ((const void *)APP_ADDR_END)
#define MSTR_SIG_ADDR_END ((const void *)((const uint8_t *)MSTR_SIG_ADDR_START+SIGNATURE_SIZE))

#define OEM_SIG_ADDR_START ((const void *)((const uint8_t *)MSTR_SIG_ADDR_END))
#define OEM_SIG_ADDR_END ((const void *)((const uint8_t *)OEM_SIG_ADDR_START+SIGNATURE_SIZE))


/* RD certifcate may follow boot signature */

#define RDCT_ADDR_START OEM_SIG_ADDR_END
#define RDCT_ADDR_END ((const void *)((const uint8_t*)RDCT_ADDR_START+sizeof(image_cert_t)))



/* The table of contents */

IMAGE_MAIN_TOC(6) = {
	{TOC_START_MAGIC, TOC_VERSION},
	{
		{"MSTR", APP_ADDR_START, APP_ADDR_END, 0, 1, 0, 0, TOC_FLAG1_BOOT | TOC_FLAG1_CHECK_SIGNATURE},
		{"SIG1", MSTR_SIG_ADDR_START, MSTR_SIG_ADDR_END, 0, 0, 0, 0, 0},
		{"OEM",  APP_ADDR_START, APP_ADDR_END, 0, 3, 0, 0, TOC_FLAG1_BOOT | TOC_FLAG1_CHECK_SIGNATURE},
		{"SIG2", OEM_SIG_ADDR_START, OEM_SIG_ADDR_END, 0, 0, 0, 0, 0},
		{"RDCT", APP_ADDR_START, APP_ADDR_END, 0, 5, 0, 0, TOC_FLAG1_RDCT | TOC_FLAG1_BOOT},
		{"RDSG", RDCT_ADDR_START, RDCT_ADDR_END, 0, 0, 0, 0, 0},
	},
	TOC_END_MAGIC
};


//  image_toc_entry_t master_signature = {
// 	.name="MSTR",			/* Name of the section */
// 	.crypto_section_start = APP_ADDR_START,   	/* Start address of the section in flash */
// 	.crypto_section_len= (MSTR_SIG_ADDR_END + SIGNATURE_SIZE - MSTR_SIG_ADDR_START),    	/* Length of the section in bytes*/
// 	.target_address = NULL,           	/* Copy target address of the section */
// 	.key_index = 0,  			/* Key index for the crypto operation */
// 	.sign_start = MSTR_SIG_ADDR_START,     		/* Start address of the section in flash */
// 	.sign_len= SIGNATURE_SIZE,        		/* End of the section */

// 	.flags1=  TOC_FLAG1_BOOT | TOC_FLAG1_CHECK_SIGNATURE,         		/* Flags */
// 	.reserved=0;      		/* e.g. for more flags */

// };
