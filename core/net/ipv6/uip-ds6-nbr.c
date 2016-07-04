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
 *    IPv6 Neighbor cache (link-layer/IPv6 address mapping)
 * \author Mathilde Durvy <mdurvy@cisco.com>
 * \author Julien Abeille <jabeille@cisco.com>
 * \author Simon Duquennoy <simonduq@sics.se>
 *
 */

#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include "lib/list.h"
#include "net/link-stats.h"
#include "net/linkaddr.h"
#include "net/packetbuf.h"
#include "net/ipv6/uip-ds6-nbr.h"
#include "net/ipv6/uip-ds6.h"

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

#ifdef UIP_CONF_DS6_NEIGHBOR_STATE_CHANGED
#define NEIGHBOR_STATE_CHANGED(n) UIP_CONF_DS6_NEIGHBOR_STATE_CHANGED(n)
void NEIGHBOR_STATE_CHANGED(uip_ds6_nbr_t *n);
#else
#define NEIGHBOR_STATE_CHANGED(n)
#endif /* UIP_DS6_CONF_NEIGHBOR_STATE_CHANGED */

#ifdef UIP_CONF_DS6_LINK_NEIGHBOR_CALLBACK
#define LINK_NEIGHBOR_CALLBACK(addr, status, numtx) UIP_CONF_DS6_LINK_NEIGHBOR_CALLBACK(addr, status, numtx)
void LINK_NEIGHBOR_CALLBACK(const linkaddr_t *addr, int status, int numtx);
#else
#define LINK_NEIGHBOR_CALLBACK(addr, status, numtx)
#endif /* UIP_CONF_DS6_LINK_NEIGHBOR_CALLBACK */
uip_ds6_reg_t uip_ds6_reg_list[UIP_DS6_REG_LIST_SIZE]; /** \brief Registrations list */
uip_ds6_defrt_t uip_ds6_defrt_list[UIP_DS6_DEFRT_NB];             /** \brief Default rt list */

NBR_TABLE_GLOBAL(uip_ds6_nbr_t, ds6_neighbors);

/* Pointers used in this file */
static uip_ds6_defrt_t *locdefrt;
#if UIP_ND6_SEND_NA
static uip_ds6_reg_t *locreg;
static uip_ds6_defrt_t *min_defrt; /* default router with minimum lifetime */
static unsigned long min_lifetime; /* minimum lifetime */
static uip_ds6_nbr_t *locnbr;
#endif

/*---------------------------------------------------------------------------*/
void
uip_ds6_neighbors_init(void)
{
  link_stats_init();
  nbr_table_register(ds6_neighbors, (nbr_table_callback *)uip_ds6_nbr_rm);
}
/*---------------------------------------------------------------------------*/
uip_ds6_nbr_t *
uip_ds6_nbr_add(const uip_ipaddr_t *ipaddr, const uip_lladdr_t *lladdr,
                uint8_t isrouter, uint8_t state, nbr_table_reason_t reason,
                void *data)
{
  uip_ds6_nbr_t *nbr = nbr_table_add_lladdr(ds6_neighbors, (linkaddr_t*)lladdr
                                            , reason, data);
  if(nbr) {
    uip_ipaddr_copy(&nbr->ipaddr, ipaddr);
#if UIP_ND6_SEND_NA || UIP_ND6_SEND_RA || !UIP_CONF_ROUTER
    nbr->isrouter = isrouter;
#endif /* UIP_ND6_SEND_NA || UIP_ND6_SEND_RA || !UIP_CONF_ROUTER */
    nbr->state = state;
#if UIP_CONF_IPV6_QUEUE_PKT
    uip_packetqueue_new(&nbr->packethandle);
#endif /* UIP_CONF_IPV6_QUEUE_PKT */
#if UIP_ND6_SEND_NA
    /* timers are set separately, for now we put them in expired state */
    stimer_set(&nbr->reachable, 0);
    stimer_set(&nbr->sendns, 0);
    nbr->nscount = 0;
#endif /* UIP_ND6_SEND_NA */
    PRINTF("Adding neighbor with ip addr ");
    PRINT6ADDR(ipaddr);
    PRINTF(" link addr ");
    PRINTLLADDR(lladdr);
    PRINTF(" state %u\n", state);
    NEIGHBOR_STATE_CHANGED(nbr);
    return nbr;
  } else {
    PRINTF("uip_ds6_nbr_add drop ip addr ");
    PRINT6ADDR(ipaddr);
    PRINTF(" link addr (%p) ", lladdr);
    PRINTLLADDR(lladdr);
    PRINTF(" state %u\n", state);
    return NULL;
  }
}

/*---------------------------------------------------------------------------*/
int
uip_ds6_nbr_rm(uip_ds6_nbr_t *nbr)
{
  if(nbr != NULL) {
#if UIP_CONF_IPV6_QUEUE_PKT
    uip_packetqueue_free(&nbr->packethandle);
#endif /* UIP_CONF_IPV6_QUEUE_PKT */
    NEIGHBOR_STATE_CHANGED(nbr);
    return nbr_table_remove(ds6_neighbors, nbr);
  }
  return 0;
}

/*---------------------------------------------------------------------------*/
const uip_ipaddr_t *
uip_ds6_nbr_get_ipaddr(const uip_ds6_nbr_t *nbr)
{
  return (nbr != NULL) ? &nbr->ipaddr : NULL;
}

/*---------------------------------------------------------------------------*/
const uip_lladdr_t *
uip_ds6_nbr_get_ll(const uip_ds6_nbr_t *nbr)
{
  return (const uip_lladdr_t *)nbr_table_get_lladdr(ds6_neighbors, nbr);
}
/*---------------------------------------------------------------------------*/
int
uip_ds6_nbr_num(void)
{
  uip_ds6_nbr_t *nbr;
  int num;

  num = 0;
  for(nbr = nbr_table_head(ds6_neighbors);
      nbr != NULL;
      nbr = nbr_table_next(ds6_neighbors, nbr)) {
    num++;
  }
  return num;
}
/*---------------------------------------------------------------------------*/
uip_ds6_nbr_t *
uip_ds6_nbr_lookup(const uip_ipaddr_t *ipaddr)
{
  uip_ds6_nbr_t *nbr = nbr_table_head(ds6_neighbors);
  if(ipaddr != NULL) {
    while(nbr != NULL) {
      if(uip_ipaddr_cmp(&nbr->ipaddr, ipaddr)) {
        return nbr;
      }
      nbr = nbr_table_next(ds6_neighbors, nbr);
    }
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
uip_ds6_nbr_t *
uip_ds6_nbr_ll_lookup(const uip_lladdr_t *lladdr)
{
  return nbr_table_get_from_lladdr(ds6_neighbors, (linkaddr_t*)lladdr);
}

/*---------------------------------------------------------------------------*/
uip_ipaddr_t *
uip_ds6_nbr_ipaddr_from_lladdr(const uip_lladdr_t *lladdr)
{
  uip_ds6_nbr_t *nbr = uip_ds6_nbr_ll_lookup(lladdr);
  return nbr ? &nbr->ipaddr : NULL;
}

/*---------------------------------------------------------------------------*/
const uip_lladdr_t *
uip_ds6_nbr_lladdr_from_ipaddr(const uip_ipaddr_t *ipaddr)
{
  uip_ds6_nbr_t *nbr = uip_ds6_nbr_lookup(ipaddr);
  return nbr ? uip_ds6_nbr_get_ll(nbr) : NULL;
}
/*---------------------------------------------------------------------------*/
void
uip_ds6_link_neighbor_callback(int status, int numtx)
{
  const linkaddr_t *dest = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
  if(linkaddr_cmp(dest, &linkaddr_null)) {
    return;
  }

  /* Update neighbor link statistics */
  link_stats_packet_sent(dest, status, numtx);
  /* Call upper-layer callback (e.g. RPL) */
  LINK_NEIGHBOR_CALLBACK(dest, status, numtx);

#if UIP_DS6_LL_NUD
  /* From RFC4861, page 72, last paragraph of section 7.3.3:
   *
   *         "In some cases, link-specific information may indicate that a path to
   *         a neighbor has failed (e.g., the resetting of a virtual circuit). In
   *         such cases, link-specific information may be used to purge Neighbor
   *         Cache entries before the Neighbor Unreachability Detection would do
   *         so. However, link-specific information MUST NOT be used to confirm
   *         the reachability of a neighbor; such information does not provide
   *         end-to-end confirmation between neighboring IP layers."
   *
   * However, we assume that receiving a link layer ack ensures the delivery
   * of the transmitted packed to the IP stack of the neighbour. This is a
   * fair assumption and allows battery powered nodes save some battery by
   * not re-testing the state of a neighbour periodically if it
   * acknowledges link packets. */
  if(status == MAC_TX_OK) {
    uip_ds6_nbr_t *nbr;
    nbr = uip_ds6_nbr_ll_lookup((uip_lladdr_t *)dest);
    if(nbr != NULL && nbr->state != NBR_INCOMPLETE) {
      nbr->state = NBR_REACHABLE;
      stimer_set(&nbr->reachable, UIP_ND6_REACHABLE_TIME / 1000);
      PRINTF("uip-ds6-neighbor : received a link layer ACK : ");
      PRINTLLADDR((uip_lladdr_t *)dest);
      PRINTF(" is reachable.\n");
    }
  }
#endif /* UIP_DS6_LL_NUD */

}
#if UIP_ND6_SEND_NA
/*---------------------------------------------------------------------------*/
/** Periodic processing on neighbors */
void
uip_ds6_neighbor_periodic(void)
{
  /* This flag signals whether we allow or not to send a packet in the current
   * invocation. */
  uint8_t allow_output = 1;

  /* minimum lifetime */
  min_lifetime = 0xFFFFFFFF;
  /* router with minimum lifetime */
  min_defrt = NULL;

  /* Periodic processing on registrations */
  for(locreg = uip_ds6_reg_list;
      locreg < uip_ds6_reg_list + UIP_DS6_REG_LIST_SIZE; locreg++) {
    if(locreg->isused) {
      if(stimer_expired(&locreg->reg_lifetime)) {
        uip_ds6_reg_rm(locreg);
      } else if(allow_output) {
        /* If no output is allowed, it is pointless to enter here in this invocation */
        if(uip_ds6_if.registration_in_progress) {
          /* There is a registration in progress */
          if((locreg == uip_ds6_if.registration_in_progress) &&
             (timer_expired(&locreg->registration_timer))) {
            /* We already sent a NS message for this address but there has been no response */
            if(locreg->reg_count >= UIP_ND6_MAX_UNICAST_SOLICIT) {
              /* NUD failed. Signal the need for next-hop determination by deleting the
               * NCE (RFC 4861) */
              uip_ds6_reg_rm(locreg);
              /* And then, delete neighbor and corresponding router (as hosts only keep
               * NCEs for routers in 6lowpan-nd) */
              locnbr = uip_ds6_nbr_lookup(&locreg->defrt->ipaddr);
              uip_ds6_nbr_rm(locnbr);
              uip_ds6_defrt_rm(locreg->defrt);
              /* Since we are deleting a default router, we must delete also all
               * registrations with that router.
               * Be careful here, uip_ds6_reg_cleanup_defrt() modifies the value of locreg!*/
              uip_ds6_reg_cleanup_defrt(locreg->defrt);
              /* We will also need to start sending RS,
               * for NUD failure case */
#if !UIP_CONF_ROUTER
              uip_ds6_send_rs(NULL);
#endif
              uip_ds6_if.registration_in_progress = NULL;
            } else {
              locreg->reg_count++;
              timer_restart(&locreg->registration_timer);
              uip_nd6_ns_output(&locreg->addr->ipaddr, &locreg->defrt->ipaddr,
                                &locreg->defrt->ipaddr, 1, UIP_ND6_REGISTRATION_LIFETIME);
            }
            allow_output = 0;                               /* Prevent this invocation from sending anything else */
          }
        } else
        /* There are no registrations in progress, let's see this entry needs (re)registration
         * or deletion */
        if((locreg->state == REG_GARBAGE_COLLECTIBLE) ||
           (locreg->state == REG_TO_BE_UNREGISTERED) ||
           ((locreg->state == REG_REGISTERED) &&
            (stimer_remaining(&locreg->reg_lifetime) < stimer_elapsed(&locreg->reg_lifetime)))) {
          /* Issue (re)registration */
          uip_ds6_if.registration_in_progress = locreg;
          locreg->reg_count++;
          timer_set(&locreg->registration_timer, (uip_ds6_if.retrans_timer / 1000) * CLOCK_SECOND);
          if(locreg->state == REG_TO_BE_UNREGISTERED) {
            uip_nd6_ns_output(&locreg->addr->ipaddr, &locreg->defrt->ipaddr,
                              &locreg->defrt->ipaddr, 1, 0);
          } else {
            uip_nd6_ns_output(&locreg->addr->ipaddr, &locreg->defrt->ipaddr,
                              &locreg->defrt->ipaddr, 1, UIP_ND6_REGISTRATION_LIFETIME);
          } allow_output = 0;                     /* Prevent this invocation from sending anything else */
        }
      }
    }
  }
  uip_ds6_nbr_t *nbr = nbr_table_head(ds6_neighbors);
  while(nbr != NULL) {
    switch(nbr->state) {
    case NBR_REACHABLE:
      if(stimer_expired(&nbr->reachable)) {
#if UIP_CONF_IPV6_RPL
        /* when a neighbor leave its REACHABLE state and is a default router,
           instead of going to STALE state it enters DELAY state in order to
           force a NUD on it. Otherwise, if there is no upward traffic, the
           node never knows if the default router is still reachable. This
           mimics the 6LoWPAN-ND behavior.
         */
        if(uip_ds6_defrt_lookup(&nbr->ipaddr) != NULL) {
          PRINTF("REACHABLE: defrt moving to DELAY (");
          PRINT6ADDR(&nbr->ipaddr);
          PRINTF(")\n");
          nbr->state = NBR_DELAY;
          stimer_set(&nbr->reachable, UIP_ND6_DELAY_FIRST_PROBE_TIME);
          nbr->nscount = 0;
        } else {
          PRINTF("REACHABLE: moving to STALE (");
          PRINT6ADDR(&nbr->ipaddr);
          PRINTF(")\n");
          nbr->state = NBR_STALE;
        }
#else /* UIP_CONF_IPV6_RPL */
        PRINTF("REACHABLE: moving to STALE (");
        PRINT6ADDR(&nbr->ipaddr);
        PRINTF(")\n");
        nbr->state = NBR_STALE;
#endif /* UIP_CONF_IPV6_RPL */
      }
      break;
    case NBR_INCOMPLETE:
      if(nbr->nscount >= UIP_ND6_MAX_MULTICAST_SOLICIT) {
        uip_ds6_nbr_rm(nbr);
      } else if(stimer_expired(&nbr->sendns) && (uip_len == 0)) {
        nbr->nscount++;
        PRINTF("NBR_INCOMPLETE: NS %u\n", nbr->nscount);
        uip_nd6_ns_output(NULL, NULL, &nbr->ipaddr,0,0);
        stimer_set(&nbr->sendns, uip_ds6_if.retrans_timer / 1000);
      }
      break;
    case NBR_DELAY:
      if(stimer_expired(&nbr->reachable)) {
        nbr->state = NBR_PROBE;
        nbr->nscount = 0;
        PRINTF("DELAY: moving to PROBE\n");
        stimer_set(&nbr->sendns, 0);
      }
      break;
    case NBR_PROBE:
      if(nbr->nscount >= UIP_ND6_MAX_UNICAST_SOLICIT) {
        uip_ds6_defrt_t *locdefrt;
        PRINTF("PROBE END\n");
        if((locdefrt = uip_ds6_defrt_lookup(&nbr->ipaddr)) != NULL) {
          if (!locdefrt->isinfinite) {
            uip_ds6_defrt_rm(locdefrt);
          }
        }
        uip_ds6_nbr_rm(nbr);
      } else if(stimer_expired(&nbr->sendns) && (uip_len == 0)) {
        nbr->nscount++;
        PRINTF("PROBE: NS %u\n", nbr->nscount);
        uip_nd6_ns_output(NULL, &nbr->ipaddr, &nbr->ipaddr,0,0);
        stimer_set(&nbr->sendns, uip_ds6_if.retrans_timer / 1000);
      }
      break;
    default:
      break;
     }
    nbr = nbr_table_next(ds6_neighbors, nbr);
  }
}
/*---------------------------------------------------------------------------*/
uip_ds6_nbr_t *
uip_ds6_get_least_lifetime_neighbor(void)
{
  uip_ds6_nbr_t *nbr = nbr_table_head(ds6_neighbors);
  uip_ds6_nbr_t *nbr_expiring = NULL;
  while(nbr != NULL) {
    if(nbr_expiring != NULL) {
      clock_time_t curr = stimer_remaining(&nbr->reachable);
      if(curr < stimer_remaining(&nbr->reachable)) {
        nbr_expiring = nbr;
      }
    } else {
      nbr_expiring = nbr;
    }
    nbr = nbr_table_next(ds6_neighbors, nbr);
  }
  return nbr_expiring;
}
#endif /* UIP_ND6_SEND_NA */
/*---------------------------------------------------------------------------*/
/**
 * \brief Removes a registration from the registrations list. It also
 * decreases the value of the number of registrations of
 * the corresponding default router
 *
 * \param reg  registration to be deleted
 */

void
uip_ds6_reg_rm(uip_ds6_reg_t *reg)
{

  reg->defrt->registrations--;
  reg->isused = 0;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Looks for a registration in the registrations list
 * \param addr  address whose registration we are looking for
 * \param defrt  default router with which the address is registered
 *
 * \returns reg  registration matching the search
 * NULL if there are no matches.
 */

uip_ds6_reg_t *
uip_ds6_reg_lookup(uip_ds6_addr_t *addr, uip_ds6_defrt_t *defrt)
{

  uip_ds6_reg_t *reg;

  for(reg = uip_ds6_reg_list;
      reg < uip_ds6_reg_list + UIP_DS6_REG_LIST_SIZE; reg++) {
    if((reg->isused) && (reg->addr == addr) && (reg->defrt == defrt)) {
      return reg;
    }
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Removes all registrations with defrt from the registration
 * list.
 *
 * \param defrt  router whose registrations we want to remove
 *
 */

void
uip_ds6_reg_cleanup_defrt(uip_ds6_defrt_t *defrt)
{

  uip_ds6_reg_t *reg;

  for(reg = uip_ds6_reg_list;
      reg < uip_ds6_reg_list + UIP_DS6_REG_LIST_SIZE; reg++) {
    if((reg->isused) && (reg->defrt == defrt)) {
      uip_ds6_reg_rm(reg);
    }
  }
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Removes all resgitrations of address addr from the
 * registration list. If the registration is in REGISTERED
 * state, we can not just delete it, but we MUST first send
 * a NS with ARO lifetime = 0. As there may be more than one,
 * we mark it as TO_BE_UNREGISTERED so uip_ds6_periodic can
 * process them properly.
 *
 * \param addr  address whose registrationes we want to remove
 *
 */

void
uip_ds6_reg_cleanup_addr(uip_ds6_addr_t *addr)
{

  uip_ds6_reg_t *reg;

  for(reg = uip_ds6_reg_list;
      reg < uip_ds6_reg_list + UIP_DS6_REG_LIST_SIZE; reg++) {
    if((reg->isused) && (reg->addr == addr)) {
      if(reg->state != REG_REGISTERED) {
        uip_ds6_reg_rm(reg);
      } else {
        /* Mark it as TO_BE_UNREGISTERED */
        reg->state = REG_TO_BE_UNREGISTERED;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns the number of addresses that are registered (or
 *                                                              pending to be registered) with a router
 *
 * \param defrt  router whose number of registrations we want to check
 *
 * \returns The number of addresses registered (or pending to be
 *                                                              registered) with defrt
 */

uint8_t
uip_ds6_get_registrations(uip_ds6_defrt_t *defrt)
{

  if(!((defrt == NULL) || (!defrt->isused))) {
     return (uint8_t) defrt->registrations;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Checks whether a NCE can be garbage-collected or not.
 *
 * \param nbr  NCE we want to check
 *
 * \returns Returns 1 if nbr can be garbage-collected, 0 otherwise.
 */

uint8_t
uip_ds6_is_nbr_garbage_collectible(uip_ds6_nbr_t *nbr)
{

  uip_ds6_reg_t *reg;
  uip_ds6_defrt_t *defrt;

  defrt =  uip_ds6_defrt_lookup(&nbr->ipaddr);

  for(reg = uip_ds6_reg_list;
      reg < uip_ds6_reg_list + UIP_DS6_REG_LIST_SIZE; reg++) {
    if((reg->isused) &&
       (reg->defrt == defrt) &&
       (reg->state != REG_GARBAGE_COLLECTIBLE)) {
      return 0;
    }
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns a default router that meets:
 *    -has the minimum number of registrations
 *    - addr is not registered with it
 *
 */
uip_ds6_defrt_t *
uip_ds6_defrt_choose_min_reg(uip_ds6_addr_t *addr)
{
  uint8_t min = 0;
  uip_ds6_defrt_t *min_defrt = NULL;

  for(locdefrt = uip_ds6_defrt_list;
      locdefrt < uip_ds6_defrt_list + UIP_DS6_DEFRT_NB; locdefrt++) {
    if(locdefrt->isused) {
      if(NULL == uip_ds6_reg_lookup(addr, locdefrt)) {
        if((min_defrt == NULL) ||
           ((min_defrt != NULL) && (uip_ds6_get_registrations(locdefrt) < min))) {
          min_defrt = locdefrt;
          min = uip_ds6_get_registrations(locdefrt);
          if(min == 0) {
            /* We are not going to find a better candidate */
            return min_defrt;
          }
        }
      }
    }
  }
  return min_defrt;
}
/*---------------------------------------------------------------------------*/



/** @} */
