
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/lm3s1968.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"

#define CLOCK_MHZ 50
#define SWIM_CLK_MHZ 8

#define IN() (HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (GPIO_PIN_6 << 2))) != 0)

unsigned long systick_mark, last_mark;
volatile int rx = 0, rx_bit;


static inline unsigned long current()
{
    return (last_mark = HWREG(NVIC_ST_CURRENT));
}

static unsigned long diff_us(unsigned long a, unsigned long b)
{
    return (a-b)/CLOCK_MHZ;
}

static int elapsed(unsigned long us)
{
    return systick_mark-current() > us*CLOCK_MHZ;
}

static void
delay_us(unsigned long us)
{
    SysCtlDelay(CLOCK_MHZ*us/3);
}

static inline void
out(int high)
{
    HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (GPIO_PIN_5 << 2))) = high ? GPIO_PIN_5 : 0;
}


static inline void
trigger(int high)
{
    HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (GPIO_PIN_2 << 2))) = high ? GPIO_PIN_2 : 0;
}


static inline void
opto(int high)
{
    HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + (GPIO_PIN_1 << 2))) = high ? GPIO_PIN_1 : 0;
}



static void
delay_swim(unsigned long clks)
{
    unsigned long start;

    for(start=current();start-current() < clks*CLOCK_MHZ/SWIM_CLK_MHZ;)
        ;
}

static inline void
listen()
{
    GPIOPinIntClear(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinIntEnable(GPIO_PORTB_BASE, GPIO_PIN_6);
    rx_bit = 0;
}

static inline int
send_bit(int b, int last)
{
    unsigned long start = last_mark;

    while(!IN());
    // use PWM here?
    //start = current();

    // disable interrupt
    HWREG(GPIO_PORTB_BASE + GPIO_O_IM) &= ~(GPIO_PIN_6);
    out(0);

    for(;start-current() < 2*CLOCK_MHZ/SWIM_CLK_MHZ;)
        ;

    if(b) {
        out(1);
        listen();
    } else {
        listen();
        for(;start-current() < 20*CLOCK_MHZ/SWIM_CLK_MHZ;)
            ;
        out(1);
    }

    for(;start-current() < 22*CLOCK_MHZ/SWIM_CLK_MHZ;)
        ;

    return b;
}

static inline int
recv_bit(int i) 
{
    unsigned long start;

    //if(rx)
    //    trigger(1);

    /*
    for(start = current();!rx_bit;)
        if(start-current() > 80*CLOCK_MHZ/SWIM_CLK_MHZ) {
            UARTprintf("timeout %d\r\n", i);
            return -1;
        }
        */
    for(start = 0;!rx_bit;start++)
        if(start > 8000) {
            UARTprintf("timeout %d\r\n", i);
            return -1;
        }
    
    rx_bit = 0;

    //trigger(0);
    return IN();
}

static int
send_cmd(int b)
{
    int p = 0;
    current();

    send_bit(0, 0);
    p ^= (send_bit(b & 4, 0) != 0);
    p ^= (send_bit(b & 2, 0) != 0);
    p ^= (send_bit(b & 1, 0) != 0);
    send_bit(p, 1);

    return recv_bit(100);
}

static int
send_byte(int b)
{
    int i, p = 0;
    
    b &= 0xff;

    for(i=b;i;i>>=1)
        p ^= i & 1;

    current();
    send_bit(0, 0);
    send_bit(b & 0x80, 0);
    send_bit(b & 0x40, 0);
    send_bit(b & 0x20, 0);
    send_bit(b & 0x10, 0);
    send_bit(b & 0x08, 0);
    send_bit(b & 0x04, 0);
    send_bit(b & 0x02, 0);
    send_bit(b & 0x01, 0);
    send_bit(p, 1);

    return recv_bit(101);
}

static int
recv_byte()
{
    int i = 0, rc = 0, p = 0;

    if(recv_bit(102) != 1) {
        UARTprintf("unexpected header bit\r\n");
        return -1;
    }

    for(;i<8;i++) {
        int bit = recv_bit(i);
        
        if(bit < 0) 
            return -1;

        rc <<= 1;
        rc |= bit;
        p ^= bit;
    }

    if(recv_bit(103) != p) {
        UARTprintf("parity error\r\n");
        return -1;
    }

    delay_swim(2);
    //ack
    send_bit(1, 0);

    return rc;
}

static int
rotf(int addr, unsigned char *buf, int len)
{
    int i;

    if(send_cmd(1) != 1) {
        UARTprintf("!cmd\r\n");
        return -1;
    }

    if(send_byte(len) != 1) {
        UARTprintf("!len\r\n");
        return -1;
    }

    if(send_byte(addr >> 16) != 1) {
        UARTprintf("!e\r\n");
        return -1;
    }

    //rx=1;
    
    if(send_byte(addr >> 8) != 1) {
        UARTprintf("!h\r\n");
        return -1;
    }


    if(send_byte(addr) != 1) {
        UARTprintf("!l\r\n");
        return -1;
    }

    for(i=0;i<len;i++) {
        int b = recv_byte();
        if(b < 0)
            return -1;
        buf[i] = b;
    }
    //rx=0;

    return len;
}

static int
wotf(int addr, const unsigned char *buf, int len)
{
    int i;

    if(send_cmd(2) != 1) {
        UARTprintf("!cmd\r\n");
        return -1;
    }

    if(send_byte(len) != 1) {
        UARTprintf("!len\r\n");
        return -1;
    }

    if(send_byte(addr >> 16) != 1) {
        UARTprintf("!e\r\n");
        return -1;
    }

    if(send_byte(addr >> 8) != 1) {
        UARTprintf("!h\r\n");
        return -1;
    }

    if(send_byte(addr) != 1) {
        UARTprintf("!l\r\n");
        return -1;
    }

    for(i=0;i<len;i++) {
        if(send_byte(buf[i]) != 1) {
            UARTprintf("!v\r\n");
            return -1;
        }
    }

    return len;
}

void GPIOHandler()
{
    GPIOPinIntClear(GPIO_PORTB_BASE, GPIO_PIN_6);
    rx_bit = 1;
}

static int 
wait_sync()
{
    unsigned long sync_timeout = systick_mark = last_mark;

    // wait for the sync pattern
    for(;diff_us(sync_timeout, last_mark) < 1000;) {
        systick_mark = current();

        // measure the sync pattern
        for(;!IN() && !elapsed(200);)
            ;

        if(last_mark == systick_mark) {
            // sync was not detected
            continue;
        }

        UARTprintf(">> found sync after %d ns, sync %d swim clks\r\n", 
                (sync_timeout-systick_mark)*1000/CLOCK_MHZ, 
                (systick_mark-last_mark)*SWIM_CLK_MHZ/CLOCK_MHZ);

        delay_us(16);

        return 1;
    }

    return -1;
}

static int
init()
{
    unsigned char buf[1];
    int i;

    systick_mark = current();
    // RESET
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0);
    delay_us(16);
    UARTprintf("systick diff %d\r\n", systick_mark-current());

    // low
    out(0);
    delay_us(16);
    for(i=0;i<4;i++) {
        out(1);
        delay_us(500);
        out(0);
        delay_us(500);
    }

    for(i=0;i<4;i++) {
        out(1);
        delay_us(250);
        out(0);
        delay_us(250);
    }
    // release SWIM
    systick_mark = current();
    out(1);
    // wait for the line to be released
    for(;!IN();)
        ;

    if(wait_sync() < 0)
        return -1;

    buf[0] = 0xa0; // enable debugger unit
    if(wotf(0x7f80, buf, 1) < 0) {
        UARTprintf("0x7f80 <- 0xa0 failed\r\n");
        return -1;
    }
    delay_us(16);
    
    // release the reset line
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_PIN_4);
    delay_us(500);

    // set clocks to default
    buf[0] = 0x00;
    if(wotf(0x50c6, buf, 1) < 0)
        UARTprintf("0x50c6 <- 0x00 failed\r\n");

    // sreset
    send_cmd(0);

    return 1;
}

static void
send_opto(unsigned char b)
{
    int i;

    for(i=0;i<8;i++,b <<= 1) {
        systick_mark = last_mark;
        opto(0);
        if(b & 0x80) {
            while(!elapsed(100));
            opto(1);
        } else {
            while(!elapsed(900));
            opto(1);
        }
        while(!elapsed(1000));
    }
}

int
main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_4| SYSCTL_USE_PLL | SYSCTL_XTAL_8MHZ |SYSCTL_OSC_MAIN); // 50MHz
    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    SysTickPeriodSet(CLOCK_MHZ*1000*1000);
    SysTickEnable();

    // UART
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    // LED
    GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    GPIODirModeSet(GPIO_PORTG_BASE, GPIO_PIN_2, GPIO_DIR_MODE_OUT);

    // trigger
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_DIR_MODE_OUT);

    // OptoIN
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    opto(1);
    GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_DIR_MODE_OUT);

    // #RESET
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_DIR_MODE_OUT);

    // SWIM OUT
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_OD);
    GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_DIR_MODE_OUT);

    // SWIM IN
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_DIR_MODE_IN);
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_FALLING_EDGE);
    //GPIOPinIntEnable(GPIO_PORTB_BASE, GPIO_PIN_6);
    IntEnable(INT_GPIOB);

    /*
    // SWIM CCP0
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_DIR_MODE_HW);
*/

    // SWIM, #RESET -> high
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_4 | GPIO_PIN_5);
    delay_us(16);

    /*
    TimerConfigure(TIMER0_BASE, TIMER_CFG_16_BIT_PAIR | TIMER_CFG_A_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 0xffff);
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);

    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_CAPA_EVENT);
    TimerEnable(TIMER0_BASE, TIMER_A);
*/

    IntMasterEnable();
    
    UARTStdioInit(1);
    UARTConfigSetExpClk(UART0_BASE, CLOCK_MHZ*1000*1000, 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));


    UARTprintf("Hi there!\r\n");

    for(;;) {
        unsigned char buf[255];
        int i, len, addr, cmd = UARTCharGet(UART0_BASE);

        switch(cmd) {
            case 0:
                if(send_cmd(0) > 0) {
                    UARTCharPut(UART0_BASE, 0);
                } else {
                    UARTCharPut(UART0_BASE, 0xff);
                }
                break;
            case 1:
                len = UARTCharGet(UART0_BASE);
                addr = UARTCharGet(UART0_BASE);
                addr <<= 8;
                addr |= UARTCharGet(UART0_BASE);
                addr <<= 8;
                addr |= UARTCharGet(UART0_BASE);
                if(len > 0 && rotf(addr, buf, len) > 0) {
                    UARTCharPut(UART0_BASE, 1);
                    for(i=0;i<len;i++) {
                        UARTCharPut(UART0_BASE, buf[i]);
                    }
                } else {
                    UARTCharPut(UART0_BASE, 0xff);
                }
                break;
            case 2:
                len = UARTCharGet(UART0_BASE);
                addr = UARTCharGet(UART0_BASE);
                addr <<= 8;
                addr |= UARTCharGet(UART0_BASE);
                addr <<= 8;
                addr |= UARTCharGet(UART0_BASE);
                for(i=0;i<len;i++) {
                    buf[i] = UARTCharGet(UART0_BASE);
                }

                if(len > 0 && wotf(addr, buf, len) > 0) {
                    UARTCharPut(UART0_BASE, 2);
                } else {
                    UARTCharPut(UART0_BASE, 0xff);
                }
                break;
            case '>':
                len = UARTCharGet(UART0_BASE);
                for(i=0;i<len;i++) {
                    buf[i] = UARTCharGet(UART0_BASE);
                }
                current();
                for(i=0;i<len;i++) {
                    send_opto(buf[i]);
                }
                UARTCharPut(UART0_BASE, '>');
                break;

            case '!':
                if(init() < 0) {
                    UARTCharPut(UART0_BASE, 0xff);
                } else {
                    UARTCharPut(UART0_BASE, '!');
                }
                break;

            default:
                UARTCharPut(UART0_BASE, 0xff);
        }
    }
    /*
    while(1)
    {
        GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, GPIO_PIN_2);
        delay_us(500000);
        UARTprintf("tick...\r\n");

        GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, 0);
        delay_us(500000);
        UARTprintf("tock...\r\n");
    }
    */
}
