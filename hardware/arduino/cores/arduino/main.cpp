
#if 0 //(!defined(CONTIKI) && !defined(IDUINO))
#include <Arduino.h>
#include <avr/wdt.h>
#include <util/delay.h>
int main(void)
{
	//wdt_enable(WDTO_8S);
	//_delay_ms(1000);
	//wdt_disable();

	init();

#if defined(USBCON)
	USBDevice.attach();
#endif
	
	setup();
    
	for (;;) {
		loop();
		if (serialEventRun) serialEventRun();
	}
        
	return 0;
}
#elif 1 //defined(IDUINO)
#include "iduino.h"

static char initialized = 0;

void
initialize(void)
{
  if (initialized)
    return;
  initialized = 1;
  watchdog_init();
  watchdog_start();
  clock_init();

  process_init();
  process_start(&etimer_process, NULL);
}
int
main(void)
{
#if ARDUINO
  setup();
#endif
  initialize();
 while(1) {
    process_run();
    watchdog_periodic();
    loop();
  }
  return 0;
}

#endif

