#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// This is the crucial line to fix the error.
// It tells lwIP to use the system's timeval struct, not its own.
#define LWIP_TIMEVAL_PRIVATE 0

// --- General options ---
#define NO_SYS                      0
#define LWIP_SOCKET                 1
#define LWIP_NETCONN                0

// --- Memory options ---
#define MEM_LIBC_MALLOC             1
#define MEMP_MEM_MALLOC             1

// --- DHCP options ---
#define LWIP_DHCP                   1

// --- TCP options ---
#define LWIP_TCP                    1
#define TCP_MSS                     1460
#define TCP_WND                     (8 * TCP_MSS)
#define TCP_SND_BUF                 (8 * TCP_MSS)

// --- DNS options ---
#define LWIP_DNS                    1

// --- Other options ---
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1

#endif
