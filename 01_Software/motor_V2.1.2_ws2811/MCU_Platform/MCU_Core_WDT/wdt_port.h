#ifndef WDT_PORT_H
#define WDT_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

void wdt_port_init(void);
void wdt_port_enable(void);
void wdt_port_feed(void);

#ifdef __cplusplus
}
#endif

#endif /* WDT_PORT_H */
