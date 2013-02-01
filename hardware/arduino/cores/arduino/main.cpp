#include <Arduino.h>
#include <avr/wdt.h>
#include <util/delay.h>

int main(void)
{

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


