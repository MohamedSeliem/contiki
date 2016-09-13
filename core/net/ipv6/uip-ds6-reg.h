/*
 * Copyright (c) 2013, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *
 */

/**
 * \addtogroup uip6
 * @{
 */

/**
 * \file
 *    IPv6 address registration list
 * \author Mohamed Seliem  <mseliem11@gmail.com>
 *
 */
#ifndef UIP_DS6_REG_H_
#define UIP_DS6_REG_H_

#include "net/ip/uip.h"
#include "net/nbr-table.h"
#include "sys/stimer.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uip-ds6-nbr.h"
#include "net/ipv6/uip-ds6-route.h"

#if UIP_CONF_IPV6_QUEUE_PKT
#include "net/ip/uip-packetqueue.h"
#endif                          /*UIP_CONF_QUEUE_PKT */

/* The host SHOULD start sending Router Solicitations "well before the
 * minimum of those lifetimes" (across all the prefixes and all the
 * contexts) expire. RFC 6775. We define thus a threshold
 * value to start sending RS messages (in seconds).*/
#ifdef  UIP_DS6_CONF_LIFETIME_THRESHOLD
#define UIP_DS6_LIFETIME_THRESHOLD UIP_DS6_CONF_LIFETIME_THRESHOLD
#else
#define UIP_DS6_LIFETIME_THRESHOLD 60
#endif

/* 6lowpan-nd default lifetimes (in seconds)*/
#ifdef UIP_DS6_GARBAGE_COLLECTIBLE_REG_LIFETIME
#define UIP_DS6_GARBAGE_COLLECTIBLE_REG_LIFETIME UIP_DS6_GARBAGE_COLLECTIBLE_REG_LIFETIME
#else
#define UIP_DS6_GARBAGE_COLLECTIBLE_REG_LIFETIME 20
#endif
#ifdef UIP_DS6_TENTATIVE_REG_LIFETIME
#define UIP_DS6_TENTATIVE_REG_LIFETIME UIP_DS6_TENTATIVE_REG_LIFETIME
#else
#define UIP_DS6_TENTATIVE_REG_LIFETIME 20 /* Default value in RFC 6775*/
#endif

/** \brief Possible states for the nbr cache entries,
 * if 6lowpan-nd is used, new states are defined (new states are
 * orthogonal to those defined in rfc4861) */
#define  REG_GARBAGE_COLLECTIBLE 0
#define  REG_TENTATIVE 1
#define  REG_REGISTERED 2
#define  REG_TO_BE_UNREGISTERED 3 /* Auxiliary registration entry state */


/* Structure to handle 6lowpan-nd registrations */
typedef struct uip_ds6_reg {
  uint8_t isused;
  uint8_t state;
  uip_ds6_addr_t* addr;
  uip_ds6_defrt_t* defrt;
  struct stimer reg_lifetime;
  struct timer registration_timer;
  uint8_t reg_count;
} uip_ds6_reg_t;

uip_ds6_reg_t* uip_ds6_reg_add(uip_ds6_addr_t* addr, uip_ds6_defrt_t* defrt, uint8_t state, uint16_t lifetime);
void uip_ds6_reg_rm(uip_ds6_reg_t* reg);
uip_ds6_reg_t *uip_ds6_reg_lookup(uip_ds6_addr_t* addr, uip_ds6_defrt_t* defrt);
uip_ds6_reg_update(uip_ds6_addr_t* addr, uip_ds6_defrt_t* defrt, uint16_t lifetime);
void uip_ds6_reg_cleanup_defrt(uip_ds6_defrt_t* defrt);
void uip_ds6_reg_cleanup_addr(uip_ds6_addr_t* addr);
uint8_t uip_ds6_get_registrations(uip_ds6_defrt_t *defrt);
uip_ds6_defrt_t* uip_ds6_defrt_choose_min_reg(uip_ds6_addr_t* addr);

#endif /* UIP_DS6_REG_H_ */
/** @} */
