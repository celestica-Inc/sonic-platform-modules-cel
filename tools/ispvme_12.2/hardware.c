/*********************************************************************************
* Lattice Semiconductor Corp. Copyright 2000-2008
* 
* This is the hardware.c of ispVME V12.1 for JTAG programmable devices.
* All the functions requiring customization are organized into this file for 
* the convinience of porting. 
*********************************************************************************/
/*********************************************************************************
 * Revision History:
 * 
 * 09/11/07 NN Type cast mismatch variables
 * 09/24/07 NN Added calibration function.
 *             Calibration will help to determine the system clock frequency
 *             and the count value for one micro-second delay of the target 
 *             specific hardware.
 *             Modified the ispVMDelay function
 *             Removed Delay Percent support
 *             Moved the sclock() function from ivm_core.c to hardware.c
 *********************************************************************************/
#include "vmopcode.h"
#include <sys/ioctl.h>
#include <sys/io.h>
#include <fcntl.h>      //for open()
#include <unistd.h>     //for close()
#include <time.h>
#include <sys/mman.h>
#include <stdio.h>

/********************************************************************************
* Declaration of global variables
*
*********************************************************************************/
static int devmem_c2, devmem_c5;
static void *portC2, *portC5;

/**
 * NOTE: Using only core GPIO here
 * FIXME: This should move to config section and support both CORE and SUS region.
 */

// unsigned long  g_siIspPins        = 0x00000000;          /*Keeper of JTAG pin state*/
unsigned short g_usCpu_Frequency  = CPU_FREQ_MH_CONFIG; /*Enter your CPU frequency here, unit in MHz.*/

/*********************************************************************************
* This is the definition of the bit locations of each respective
* signal in the global variable g_siIspPins.
*
* NOTE: Users must add their own implementation here to define
*       the bit location of the signal to target their hardware.
*       The example below is for the Lattice download cable on
*       on the parallel port.
*
*********************************************************************************/


unsigned long g_ucPinTDI          = GPIO_TDI_CONFIG;    /* Pin nummber of TDI */
unsigned long g_ucPinTCK          = GPIO_TCK_CONFIG;    /* Pin nummber of TCK */
unsigned long g_ucPinTMS          = GPIO_TMS_CONFIG;    /* Pin nummber of TMS */
unsigned long g_ucPinENABLE       = GPIO_ENABLE_CONFIG; /* Pin nummber of ENABLE */
unsigned long g_ucPinTRST         = GPIO_TRST_CONFIG;   /* Pin nummber of TRST */
unsigned long g_ucPinTDO          = GPIO_TDO_CONFIG;    /* Pin nummber of TDO */
unsigned long g_ucInPort          = GP_LVL;             /* All TCI,TDO,TMS,TCK are on same register */
unsigned long g_ucOutPort         = GP_LVL;             /* All TCI,TDO,TMS,TCK are on same register */

/* For Denverton CPU */
// const unsigned long g_ucPinTDI          = DNV_GPIO_TDI_CONFIG;
// const unsigned long g_ucPinTCK          = DNV_GPIO_TCK_CONFIG;
// const unsigned long g_ucPinTMS          = DNV_GPIO_TMS_CONFIG;
// const unsigned long g_ucPinTDO          = DNV_GPIO_TDO_CONFIG;

/***************************************************************
*
* Functions declared in hardware.c module.
*
***************************************************************/
void writePort( unsigned long a_ucPins, unsigned char a_ucValue );
unsigned char readPort();
void sclock();
void ispVMDelay( unsigned short a_usTimeDelay );
void calibration(void);

/********************************************************************************
* writePort
* To apply the specified value to the pins indicated. This routine will
* be modified for specific systems. 
* As an example, this code uses the IBM-PC standard Parallel port, along with the
* schematic shown in Lattice documentation, to apply the signals to the
* JTAG pins.
*
* PC Parallel port pin    Signal name           Port bit address
*        2                g_ucPinTDI             1
*        3                g_ucPinTCK             2
*        4                g_ucPinTMS             4
*        5                g_ucPinENABLE          8
*        6                g_ucPinTRST            16
*        10               g_ucPinTDO             64
*
*  Parameters:
*   - a_ucPins, which is actually a set of bit flags (defined above)
*     that correspond to the bits of the data port. Each of the I/O port
*     bits that drives an isp programming pin is assigned a flag 
*     (through a #define) corresponding to the signal it drives. To 
*     change the value of more than one pin at once, the flags are added 
*     together, much like file access flags are.
*
*     The bit flags are only set if the pin is to be changed. Bits that 
*     do not have their flags set do not have their levels changed. The 
*     state of the port is always manintained in the static global 
*     variable g_siIspPins, so that each pin can be addressed individually 
*     without disturbing the others.
*
*   - a_ucValue, which is either HIGH (0x01 ) or LOW (0x00 ). Only these two
*     values are valid. Any non-zero number sets the pin(s) high.
*
*********************************************************************************/

void writePort( unsigned long a_ucPins, unsigned char a_ucValue )
{

    unsigned long siIspPins = 0;

    /* For Denverton */
    // isp_dnv_gpio_write(a_ucPins, (unsigned int) a_ucValue);

    /* TODO: Convert to bit read/write function */
    siIspPins = inl_p( g_ucOutPort );
    if( a_ucValue ){
        siIspPins |= (1U << a_ucPins);
    }else{
        siIspPins &= ~(1U << a_ucPins);
    }
    outl_p(siIspPins, g_ucOutPort);
}

/*********************************************************************************
*
* readPort
*
* Returns the value of the TDO from the device.
*
**********************************************************************************/
unsigned char readPort()
{
    unsigned char ucRet = 0;

    /* For Denverton */
    // if ( isp_dnv_gpio_read(DNV_GPIO_TDO_CONFIG) ) {
    //  ucRet = 0x01;
    // }
    // else {
    //  ucRet = 0x00;
    // }

    /* TODO: Convert to bit read/write function */
    if ( inl_p( g_ucInPort ) & (1U << g_ucPinTDO)) {
        ucRet = 0x01;
    }
    else {
        ucRet = 0x00;
    }
    return ( ucRet );
}

/*********************************************************************************
* sclock
*
* Apply a pulse to TCK.
*
* This function is located here so that users can modify to slow down TCK if
* it is too fast (> 25MHZ). Users can change the IdleTime assignment from 0 to 
* 1, 2... to effectively slowing down TCK by half, quarter...
*
*********************************************************************************/
void sclock()
{
    unsigned short IdleTime    = 0; //change to > 0 if need to slow down TCK
    unsigned short usIdleIndex = 0;
    IdleTime++;
    for ( usIdleIndex = 0; usIdleIndex < IdleTime; usIdleIndex++ ) {
        writePort( g_ucPinTCK, 0x01 );
    }
    for ( usIdleIndex = 0; usIdleIndex < IdleTime; usIdleIndex++ ) { 
        writePort( g_ucPinTCK, 0x00 );
    }
}
/********************************************************************************
*
* ispVMDelay
*
*
* Users must implement a delay to observe a_usTimeDelay, where
* bit 15 of the a_usTimeDelay defines the unit.
*      1 = milliseconds
*      0 = microseconds
* Example:
*      a_usTimeDelay = 0x0001 = 1 microsecond delay.
*      a_usTimeDelay = 0x8001 = 1 millisecond delay.
*
* This subroutine is called upon to provide a delay from 1 millisecond to a few 
* hundreds milliseconds each time. 
* It is understood that due to a_usTimeDelay is defined as unsigned short, a 16 bits
* integer, this function is restricted to produce a delay to 64000 micro-seconds 
* or 32000 milli-second maximum. The VME file will never pass on to this function
* a delay time > those maximum number. If it needs more than those maximum, the VME
* file will launch the delay function several times to realize a larger delay time
* cummulatively.
* It is perfectly alright to provide a longer delay than required. It is not 
* acceptable if the delay is shorter.
*
* Delay function example--using the machine clock signal of the native CPU------
* When porting ispVME to a native CPU environment, the speed of CPU or 
* the system clock that drives the CPU is usually known. 
* The speed or the time it takes for the native CPU to execute one for loop 
* then can be calculated as follows:
*       The for loop usually is compiled into the ASSEMBLY code as shown below:
*       LOOP: DEC RA;
*             JNZ LOOP;
*       If each line of assembly code needs 4 machine cycles to execute, 
*       the total number of machine cycles to execute the loop is 2 x 4 = 8.
*       Usually system clock = machine clock (the internal CPU clock). 
*       Note: Some CPU has a clock multiplier to double the system clock for 
              the machine clock.
*
*       Let the machine clock frequency of the CPU be F, or 1 machine cycle = 1/F.
*       The time it takes to execute one for loop = (1/F ) x 8.
*       Or one micro-second = F(MHz)/8;
*
* Example: The CPU internal clock is set to 100Mhz, then one micro-second = 100/8 = 12
*
* The C code shown below can be used to create the milli-second accuracy. 
* Users only need to enter the speed of the cpu.
*
**********************************************************************************/
void ispVMDelay( unsigned short a_usTimeDelay )
{
    unsigned short loop_index     = 0;
    unsigned short ms_index       = 0;
    unsigned short us_index       = 0;

    if ( a_usTimeDelay & 0x8000 ) /*Test for unit*/
    {
        a_usTimeDelay &= ~0x8000; /*unit in milliseconds*/
    }
    else { /*unit in microseconds*/
        a_usTimeDelay = (unsigned short) (a_usTimeDelay/1000); /*convert to milliseconds*/
        if ( a_usTimeDelay <= 0 ) {
             a_usTimeDelay = 1; /*delay is 1 millisecond minimum*/
        }
    }
    /*Users can replace the following section of code by their own*/
    for( ms_index = 0; ms_index < a_usTimeDelay; ms_index++)
    {
        /*Loop 1000 times to produce the milliseconds delay*/
        for (us_index = 0; us_index < 1000; us_index++)
        { /*each loop should delay for 1 microsecond or more.*/
            loop_index = 0;
            do {
                /*The NOP fakes the optimizer out so that it doesn't toss out the loop code entirely*/
                asm("nop");
            }while (loop_index++ < ((g_usCpu_Frequency/8)+(+ ((g_usCpu_Frequency % 8) ? 1 : 0))));/*use do loop to force at least one loop*/
        }
    }
}

/*********************************************************************************
*
* calibration
*
* It is important to confirm if the delay function is indeed providing 
* the accuracy required. Also one other important parameter needed 
* checking is the clock frequency. 
* Calibration will help to determine the system clock frequency 
* and the loop_per_micro value for one micro-second delay of the target 
* specific hardware.
*
**********************************************************************************/
void calibration(void)
{
    /*Apply 2 pulses to TCK.*/
    writePort( g_ucPinTCK, 0x00 );
    writePort( g_ucPinTCK, 0x01 );
    writePort( g_ucPinTCK, 0x00 );
    writePort( g_ucPinTCK, 0x01 );
    writePort( g_ucPinTCK, 0x00 );

    /*Delay for 1 millisecond. Pass on 1000 or 0x8001 both = 1ms delay.*/
    ispVMDelay(0x8001);

    /*Apply 2 pulses to TCK*/
    writePort( g_ucPinTCK, 0x01 );
    writePort( g_ucPinTCK, 0x00 );
    writePort( g_ucPinTCK, 0x01 );
    writePort( g_ucPinTCK, 0x00 );

    ispVMDelay(0x8001);
}

void port_test(void)
{
    int siRetCode;
    unsigned char cbit;

    printf("TDI set HIGH.\n");
    if(scanf("%d",&siRetCode)){}
    writePort( g_ucPinTDI, 0x01);
    printf("TDI set LOW.\n");
    if(scanf("%d",&siRetCode)){}
    writePort( g_ucPinTDI, 0x00);
    printf("TMS set HIGH.\n");
    if(scanf("%d",&siRetCode)){}
    writePort(g_ucPinTMS, 0x01);
    printf("TMS set LOW.\n");
    if(scanf("%d",&siRetCode)){}
    writePort(g_ucPinTMS, 0x00);
    printf("TCK set HIGH.\n");
    if(scanf("%d",&siRetCode)){}
    writePort(g_ucPinTCK, 0x01);
    printf("TCK set LOW.\n");
    if(scanf("%d",&siRetCode)){}
    writePort(g_ucPinTCK, 0x00);
    printf("write finished.read begin:\n");
    if(scanf("%d",&siRetCode)){}
    cbit = readPort();
    printf("Read date is %d\n", cbit);
    printf("read begin:\n");
    if(scanf("%d",&siRetCode)){}
    cbit = readPort();
    printf("Read date is %d\n", cbit);
    printf("read finished.\n");
    if(scanf("%d",&siRetCode)){}
}


void isp_dnv_gpio_config(unsigned int gpio, unsigned int dir)
{
    volatile unsigned int *buffer;
    // Select community
    if(GET_PORT(gpio) == 0xC5){
        buffer = (volatile unsigned int *)(portC5 + OFFSET_ADDR(gpio));
    }else{
        buffer = (volatile unsigned int *)(portC2 + OFFSET_ADDR(gpio));
    }
    // set mode to GPIO, set pin direction.
    *buffer &= (~((unsigned int)7)) << 10; // clear [12:10]
    *buffer &= (~((unsigned int)3)) << 8;  // clear [9:8]
    *buffer |= ((unsigned int)dir & 0x3) << 8; // set [9:8]
}

void isp_dnv_gpio_write(unsigned int gpio, unsigned int value)
{
    volatile unsigned char *buffer;
    // Select community
    if(GET_PORT(gpio) == 0xC5){
        buffer = (volatile unsigned char *)(portC5 + OFFSET_ADDR(gpio));
    }else{
        buffer = (volatile unsigned char *)(portC2 + OFFSET_ADDR(gpio));
    }
    if(value) {
        *buffer = DNV_GPIO_LVL_HIGH;
    } else {
        *buffer = DNV_GPIO_LVL_LOW;
    }
}

int isp_dnv_gpio_read(unsigned int gpio)
{
    volatile unsigned int *buffer;
    // Select community
    if(GET_PORT(gpio) == 0xC5){
        buffer = (volatile unsigned int *)(portC5 + OFFSET_ADDR(gpio));
    }else{
        buffer = (volatile unsigned int *)(portC2 + OFFSET_ADDR(gpio));
    }
    return (int)((*buffer & 0x2) >> 1);
}


void isp_dnv_gpio_init(void){

    devmem_c2 = open("/dev/mem", O_RDWR | O_SYNC);
    if (devmem_c2 == -1){
        perror("Can't open /dev/mem.");
        return;
    }

    devmem_c5 = open("/dev/mem", O_RDWR | O_SYNC);
    if (devmem_c5 == -1){
        perror("Can't open /dev/mem.");
        return;
    }

    portC2 = mmap(NULL, MAP_SIZE(g_ucPinTCK) , PROT_READ | PROT_WRITE, MAP_SHARED, devmem_c2, g_ucPinTCK & ~MAP_MASK);
    if (portC2 == MAP_FAILED) {
        perror("Can't map memory: ");
        return;
    }
    portC5 = mmap(NULL, MAP_SIZE(g_ucPinTDO) , PROT_READ | PROT_WRITE, MAP_SHARED, devmem_c5, g_ucPinTDO & ~MAP_MASK);
    if (portC2 == MAP_FAILED) {
        perror("Can't map memory: ");
        return;
    }
}

void isp_dnv_gpio_deinit(void){
    munmap(portC2, MAP_SIZE(g_ucPinTCK));
    munmap(portC5, MAP_SIZE(g_ucPinTDO));
    close(devmem_c2);
    close(devmem_c5);
}
