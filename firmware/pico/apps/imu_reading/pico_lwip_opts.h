#ifndef PICO_LWIP_OPTS_H
#define PICO_LWIP_OPTS_H

// Minimal lwIP opts to enable httpd and SSI when using Pico SDK's lwIP port
// We avoid enabling features that significantly increase flash/RAM unless needed.

// Use lightweight DHCP client if STA mode joins networks using DHCP
#ifndef LWIP_DHCP
#define LWIP_DHCP 1
#endif

// Enable IPv4 only for simplicity
#ifndef LWIP_IPV4
#define LWIP_IPV4 1
#endif
#ifndef LWIP_IPV6
#define LWIP_IPV6 0
#endif

// HTTPD + SSI
#ifndef LWIP_HTTPD
#define LWIP_HTTPD 1
#endif
#ifndef LWIP_HTTPD_SSI
#define LWIP_HTTPD_SSI 1
#endif
#ifndef LWIP_HTTPD_DYNAMIC_HEADERS
#define LWIP_HTTPD_DYNAMIC_HEADERS 1
#endif

// Enable BSD Sockets API
#ifndef LWIP_SOCKET
#define LWIP_SOCKET 1
#endif

// Use system struct timeval to avoid redefinition with lwIP
#ifndef LWIP_TIMEVAL_PRIVATE
#define LWIP_TIMEVAL_PRIVATE 0
#endif

// Optional: Reduce default TCP MSS/window to save RAM
#ifndef TCP_MSS
#define TCP_MSS 1460
#endif
#ifndef TCP_WND
#define TCP_WND (4 * TCP_MSS)
#endif
#ifndef TCP_SND_BUF
#define TCP_SND_BUF (2 * TCP_MSS)
#endif

#endif // PICO_LWIP_OPTS_H
