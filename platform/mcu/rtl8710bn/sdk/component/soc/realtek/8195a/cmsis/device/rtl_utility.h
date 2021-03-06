/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 ******************************************************************************/
#ifndef __RTL_UTILITY_H_
#define __RTL_UTILITY_H_

VOID RtlMemcpy(VOID* dec, VOID* sour, u32 sz);
u32 RtlMemcmp(VOID *dst, VOID *src, u32 sz);
VOID RtlMemset(VOID *pbuf, u32 c, u32 sz);

s8 *
RtlStrncpy(
    IN  s8 *dest, 
    IN  const s8 *src, 
    IN  SIZE_T count
);

s8 *
RtlStrcpy(
    IN  s8 *dest, 
    IN  const s8 *src
);


SIZE_T
RtlStrlen(
    IN  const s8 *s
);


SIZE_T
RtlStrnlen(
    IN  const s8 *s, 
    IN  SIZE_T count
);


int 
RtlStrcmp(
    IN  const s8 *cs, 
    IN  const s8 *ct

);

int
RtlStrncmp(
    IN  const s8 *cs, 
    IN  const s8 *ct, 
    IN  SIZE_T count
);

#endif


