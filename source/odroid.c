/*
*  Odroid specific code borrowed from Hardkernel's wiringPi port
*/

// License and info rom Hardkernel's original file:
/*
 * wiringPi:
 *	Arduino compatable (ish) Wiring library for the Raspberry Pi
 *	Copyright (c) 2012 Gordon Henderson
 *	Additional code for pwmSetClock by Chris Hall <chris@kchall.plus.com>
 *
 *	Thanks to code samples from Gert Jan van Loo and the
 *	BCM2835 ARM Peripherals manual, however it's missing
 *	the clock section /grr/mutter/
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

// Revisions:
//	19 Jul 2012:
//		Moved to the LGPL
//		Added an abstraction layer to the main routines to save a tiny
//		bit of run-time and make the clode a little cleaner (if a little
//		larger)
//		Added waitForInterrupt code
//		Added piHiPri code
//
//	 9 Jul 2012:
//		Added in support to use the /sys/class/gpio interface.
//	 2 Jul 2012:
//		Fixed a few more bugs to do with range-checking when in GPIO mode.
//	11 Jun 2012:
//		Fixed some typos.
//		Added c++ support for the .h file
//		Added a new function to allow for using my "pin" numbers, or native
//			GPIO pin numbers.
//		Removed my busy-loop delay and replaced it with a call to delayMicroseconds
//
//	02 May 2012:
//		Added in the 2 UART pins
//		Change maxPins to numPins to more accurately reflect purpose

#ifdef DEFINE_ODROID_CODE
#include <stdarg.h>
#include <errno.h>
#include <unistd.h>
#include <sys/utsname.h>

int wiringPiFailure (int fatal, const char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  if (!fatal && wiringPiReturnCodes)
    return -1 ;

  va_start (argp, message) ;
    vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  fprintf (stderr, "%s", buffer) ;
  exit (EXIT_FAILURE) ;

  return 0 ;
}

static int gpioToPin(int gpio)
{
	int pin;

	if (pinToGpio == NULL) {
		(void)wiringPiFailure (WPI_FATAL, "%s: wiringPi is not initialized yet\n", __func__);
		return -1;
	}

	for (pin = 0; pin < pin_array_count; ++pin) {
		if (pinToGpio[pin] == gpio)
			return pin;
	}

	(void)wiringPiFailure (WPI_FATAL, "%s: could not find the pin of %d gpio\n", __func__, gpio);
	return -1;
}

//
// sysfs FD offset
//
static int  gpioFdOffsetXU34(int pin)
{
    int offset = -1;

    switch(pin) {
        case  GPIO_X1_START...GPIO_X1_END:  offset = (pin - GPIO_X1_START) + 0;     break;
        case  GPIO_X2_START...GPIO_X2_END:  offset = (pin - GPIO_X2_START) + 8;     break;
        case  GPIO_X3_START...GPIO_X3_END:  offset = (pin - GPIO_X3_START) + 16;    break;
        case  GPIO_A0_START...GPIO_A0_END:  offset = (pin - GPIO_A0_START) + 24;    break;
        case  GPIO_A2_START...GPIO_A2_END:  offset = (pin - GPIO_A2_START) + 32;    break;
        case  GPIO_B3_START...GPIO_B3_END:  offset = (pin - GPIO_B3_START) + 40;    break;
        default :                           offset = -1;                            break;
    }
    return  offset;
}

//
// offset to the GPIO Set regsiter
//
static int  gpioToGPSETReg (int pin)
{
    if(piModel == PI_MODEL_ODROIDXU_34) {
        switch(pin) {
            case    GPIO_X1_START...GPIO_X1_END:
                return  (GPIO_X1_DAT_OFFSET >> 2);
            case    GPIO_X2_START...GPIO_X2_END:
                return  (GPIO_X2_DAT_OFFSET >> 2);
            case    GPIO_X3_START...GPIO_X3_END:
                return  (GPIO_X3_DAT_OFFSET >> 2);
            case    GPIO_A0_START...GPIO_A0_END:
                return  (GPIO_A0_DAT_OFFSET >> 2);
            case    GPIO_A2_START...GPIO_A2_END:
                return  (GPIO_A2_DAT_OFFSET >> 2);
            case    GPIO_B3_START...GPIO_B3_END:
                return  (GPIO_B3_DAT_OFFSET >> 2);
            default:
                break;
        }
    }
    else if (piModel == PI_MODEL_ODROIDC2)	{
        if(pin >= C2_GPIOX_PIN_START && pin <= C2_GPIOX_PIN_END)
		return  C2_GPIOX_OUTP_REG_OFFSET;
        if(pin >= C2_GPIOY_PIN_START && pin <= C2_GPIOY_PIN_END)
		return  C2_GPIOY_OUTP_REG_OFFSET;
    }
    else    {
        if(pin >= GPIOX_PIN_START && pin <= GPIOX_PIN_END)
		return  GPIOX_OUTP_REG_OFFSET;
        if(pin >= GPIOY_PIN_START && pin <= GPIOY_PIN_END)
		return  GPIOY_OUTP_REG_OFFSET;
    }

    return  -1;
}

//
// offset to the GPIO Input regsiter
//
static int  gpioToGPLEVReg (int pin)
{
    if(piModel == PI_MODEL_ODROIDXU_34) {
        switch(pin) {
            case    GPIO_X1_START...GPIO_X1_END:
                return  (GPIO_X1_DAT_OFFSET >> 2);
            case    GPIO_X2_START...GPIO_X2_END:
                return  (GPIO_X2_DAT_OFFSET >> 2);
            case    GPIO_X3_START...GPIO_X3_END:
                return  (GPIO_X3_DAT_OFFSET >> 2);
            case    GPIO_A0_START...GPIO_A0_END:
                return  (GPIO_A0_DAT_OFFSET >> 2);
            case    GPIO_A2_START...GPIO_A2_END:
                return  (GPIO_A2_DAT_OFFSET >> 2);
            case    GPIO_B3_START...GPIO_B3_END:
                return  (GPIO_B3_DAT_OFFSET >> 2);
            default:
                break;
        }
    }
    else if (piModel == PI_MODEL_ODROIDC2)	{
        if(pin >= C2_GPIOX_PIN_START && pin <= C2_GPIOX_PIN_END)
		return  C2_GPIOX_INP_REG_OFFSET;
        if(pin >= C2_GPIOY_PIN_START && pin <= C2_GPIOY_PIN_END)
		return  C2_GPIOY_INP_REG_OFFSET;
    }
    else    {
        if(pin >= GPIOX_PIN_START && pin <= GPIOX_PIN_END)
		return  GPIOX_INP_REG_OFFSET;
        if(pin >= GPIOY_PIN_START && pin <= GPIOY_PIN_END)
		return  GPIOY_INP_REG_OFFSET;
    }

    return  -1;
}

//
// offset to the GPIO Pull up/down enable regsiter
//
static int  gpioToPUENReg (int pin)
{
	if(piModel == PI_MODEL_ODROIDC2)	{
		if(pin >= C2_GPIOX_PIN_START && pin <= C2_GPIOX_PIN_END)
			return  C2_GPIOX_PUEN_REG_OFFSET;
		if(pin >= C2_GPIOY_PIN_START && pin <= C2_GPIOY_PIN_END)
			return  C2_GPIOY_PUEN_REG_OFFSET;
	}
	else	{
		if(pin >= GPIOX_PIN_START && pin <= GPIOX_PIN_END)
			return  GPIOX_PUEN_REG_OFFSET;
		if(pin >= GPIOY_PIN_START && pin <= GPIOY_PIN_END)
			return  GPIOY_PUEN_REG_OFFSET;
	}

    return  -1;
}

//
// offset to the GPIO Pull up/down regsiter
//
static int  gpioToPUPDReg (int pin)
{
    if(piModel == PI_MODEL_ODROIDXU_34) {
        switch(pin) {
            case    GPIO_X1_START...GPIO_X1_END:
                return  (GPIO_X1_PUD_OFFSET >> 2);
            case    GPIO_X2_START...GPIO_X2_END:
                return  (GPIO_X2_PUD_OFFSET >> 2);
            case    GPIO_X3_START...GPIO_X3_END:
                return  (GPIO_X3_PUD_OFFSET >> 2);
            case    GPIO_A0_START...GPIO_A0_END:
                return  (GPIO_A0_PUD_OFFSET >> 2);
            case    GPIO_A2_START...GPIO_A2_END:
                return  (GPIO_A2_PUD_OFFSET >> 2);
            case    GPIO_B3_START...GPIO_B3_END:
                return  (GPIO_B3_PUD_OFFSET >> 2);
            default:
                break;
        }
    }
    else if (piModel == PI_MODEL_ODROIDC2)	{
        if(pin >= C2_GPIOX_PIN_START && pin <= C2_GPIOX_PIN_END)
		return	C2_GPIOX_PUPD_REG_OFFSET;
        if(pin >= C2_GPIOY_PIN_START && pin <= C2_GPIOY_PIN_END)
		return  C2_GPIOY_PUPD_REG_OFFSET;
    }
    else    {
        if(pin >= GPIOX_PIN_START && pin <= GPIOX_PIN_END)
		return  GPIOX_PUPD_REG_OFFSET;
        if(pin >= GPIOY_PIN_START && pin <= GPIOY_PIN_END)
		return  GPIOY_PUPD_REG_OFFSET;
    }

    return  -1;
}

//
// offset to the GPIO bit
//
static int  gpioToShiftReg (int pin)
{
    if(piModel == PI_MODEL_ODROIDXU_34) {
        switch(pin) {
            case    GPIO_X1_START...GPIO_X1_END:
                return  (pin - GPIO_X1_START);
            case    GPIO_X2_START...GPIO_X2_END:
                return  (pin - GPIO_X2_START);
            case    GPIO_X3_START...GPIO_X3_END:
                return  (pin - GPIO_X3_START);
            case    GPIO_A0_START...GPIO_A0_END:
                return  (pin - GPIO_A0_START);
            case    GPIO_A2_START...GPIO_A2_END:
                return  (pin - GPIO_A2_START);
            case    GPIO_B3_START...GPIO_B3_END:
                return  (pin - GPIO_B3_START);
            default:
                break;
        }
    }
    else if (piModel == PI_MODEL_ODROIDC2)	{
        if(pin >= C2_GPIOX_PIN_START && pin <= C2_GPIOX_PIN_END)
		return  pin - C2_GPIOX_PIN_START;
        if(pin >= C2_GPIOY_PIN_START && pin <= C2_GPIOY_PIN_END)
		return  pin - C2_GPIOY_PIN_START;
    }
    else    {
        if(pin >= GPIOX_PIN_START && pin <= GPIOX_PIN_END)
		return  pin - GPIOX_PIN_START;
        if(pin >= GPIOY_PIN_START && pin <= GPIOY_PIN_END)
		return  pin - GPIOY_PIN_START;
    }

    return  -1;
}

//
// offset to the GPIO Function register
//
static int  gpioToGPFSELReg (int pin)
{
    if(piModel == PI_MODEL_ODROIDXU_34) {
        switch(pin) {
            case    GPIO_X1_START...GPIO_X1_END:
                return  (GPIO_X1_CON_OFFSET >> 2);
            case    GPIO_X2_START...GPIO_X2_END:
                return  (GPIO_X2_CON_OFFSET >> 2);
            case    GPIO_X3_START...GPIO_X3_END:
                return  (GPIO_X3_CON_OFFSET >> 2);
            case    GPIO_A0_START...GPIO_A0_END:
                return  (GPIO_A0_CON_OFFSET >> 2);
            case    GPIO_A2_START...GPIO_A2_END:
                return  (GPIO_A2_CON_OFFSET >> 2);
            case    GPIO_B3_START...GPIO_B3_END:
                return  (GPIO_B3_CON_OFFSET >> 2);
            default:
                break;
        }
    }
    else if (piModel == PI_MODEL_ODROIDC2)	{
        if(pin >= C2_GPIOX_PIN_START && pin <= C2_GPIOX_PIN_END)
		return  C2_GPIOX_FSEL_REG_OFFSET;
        if(pin >= C2_GPIOY_PIN_START && pin <= C2_GPIOY_PIN_END)
		return  C2_GPIOY_FSEL_REG_OFFSET;
    }
    else    {
        if(pin >= GPIOX_PIN_START && pin <= GPIOX_PIN_END)
		return  GPIOX_FSEL_REG_OFFSET;
        if(pin >= GPIOY_PIN_START && pin <= GPIOY_PIN_END)
		return  GPIOY_FSEL_REG_OFFSET;
    }

    return  -1;
}

/*
* Note: Unlike the above code, this is not copied directly from wiringPi
* Much of the code is identical, but un-necessary part are deleted
*/

int wiringPiSetupOdroid (void)
{
    int fd;

    // Open the master /dev/memory device
    if (access("/dev/gpiomem", 0) == 0)
    {
        if ((fd = open("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0)
            return wiringPiFailure(WPI_ALMOST, "wiringPiSetup: Unable to open /dev/gpiomem: %s\n", strerror(errno));
    }
    else
    {
        if (geteuid() != 0)
            (void)wiringPiFailure(WPI_FATAL, "wiringPiSetup: Must be root. (Did you forget sudo?)\n");

        if ((fd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0)
            return wiringPiFailure(WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror(errno));
    }

    //  piBoardId (&model, &rev, &mem, &maker, &overVolted) ;

    //wiringPi pin numbers are unused in rPI.GPIO
    pinToGpio = NULL;
    pin_array_count = 0;
    //physToGPIO replaced by pin_to_gpio rPI.GPIO
    physToGpio = NULL;

    if (piModel == PI_MODEL_ODROIDC)
    {

        // GPIO:

        gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, ODROID_GPIO_BASE);
        if ((int32_t)gpio == -1)
            return wiringPiFailure(WPI_ALMOST, "wiringPiSetup: mmap (GPIO) failed: %s\n", strerror(errno));
        gpio1 = gpio;

        // ADC
        // ADC sysfs open (/sys/class/saradc/saradc_ch0, ch1)
        adcFds[0] = open(piAinNode0, O_RDONLY);
        adcFds[1] = open(piAinNode1, O_RDONLY);
    }
    else if (piModel == PI_MODEL_ODROIDC2)
    {

        // GPIO:
        gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, ODROIDC2_GPIO_BASE);
        if ((int32_t)gpio == -1)
            return wiringPiFailure(WPI_ALMOST, "wiringPiSetup: mmap (GPIO) failed: %s\n", strerror(errno));
        gpio1 = gpio;

        // ADC
        // ADC sysfs open (/sys/class/saradc/saradc_ch0, ch1)
        adcFds[0] = open(C2_piAinNode0, O_RDONLY);
        adcFds[1] = open(C2_piAinNode1, O_RDONLY);
    }
    else if (piModel == PI_MODEL_ODROIDXU_34)
    {
        // Check the kernel version and then set the ADC files
        struct utsname uname_buf;

        uname(&uname_buf);
        if (strncmp(uname_buf.release, "4.14", 4) == 0)
        {
            piAinNode0_xu = "/sys/devices/platform/soc/12d10000.adc/iio:device0/in_voltage0_raw";
            piAinNode1_xu = "/sys/devices/platform/soc/12d10000.adc/iio:device0/in_voltage3_raw";
        }
        else if (strncmp(uname_buf.release, "4.9", 3) == 0)
        {
            piAinNode0_xu = "/sys/devices/platform/soc:/12d10000.adc:/iio:device0/in_voltage0_raw";
            piAinNode1_xu = "/sys/devices/platform/soc:/12d10000.adc:/iio:device0/in_voltage3_raw";
        }
        else
        { // 3.10 kernel
            piAinNode0_xu = "/sys/devices/12d10000.adc/iio:device0/in_voltage0_raw";
            piAinNode1_xu = "/sys/devices/12d10000.adc/iio:device0/in_voltage3_raw";
        }

        // GPIO:
        //#define ODROIDXU_GPX_BASE   0x13400000  // GPX0,1,2,3
        gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, ODROIDXU_GPX_BASE);
        if ((int32_t)gpio == -1)
            return wiringPiFailure(WPI_ALMOST, "wiringPiSetup: mmap (GPIO) failed: %s\n", strerror(errno));

        //#define ODROIDXU_GPA_BASE   0x14010000  // GPA0,1,2, GPB0,1,2,3,4
        gpio1 = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, ODROIDXU_GPA_BASE);
        if ((int32_t)gpio1 == -1)
            return wiringPiFailure(WPI_ALMOST, "wiringPiSetup: mmap (GPIO) failed: %s\n", strerror(errno));

        // ADC
        // ADC Fds[0] = ("/sys/devices/12d10000.adc/iio:device0/in_voltage0_raw")
        // ADC Fds[1] = ("/sys/devices/12d10000.adc/iio:device0/in_voltage3_raw")
        adcFds[0] = open(piAinNode0_xu, O_RDONLY);
        adcFds[1] = open(piAinNode1_xu, O_RDONLY);
    }

    return 0;
}

#else /* DEFINE_ODROID_CODE */
#endif /* DEFINE_ODROID_CODE */