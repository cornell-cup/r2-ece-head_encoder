#include "config.h"
#include "pt_cornell_1_2.h"
#include <math.h>
#include "tft_master.h"
#include "tft_gfx.h"
#include <time.h>
#include <stdlib.h>
#include <stdint.h>


// === SPI setup ========================================================
volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
volatile int spiClkDiv = 64 ; // 20 MHz max speed for this RAM

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_spiWrite, pt_spiReadPos, pt_spiZeroPt, pt_UART;

unsigned int cmd_NOP;
unsigned int cmd_rdpos;
unsigned int cmd_zeroPt;
unsigned int pos_MSB = 0;
unsigned int pos_LSB = 0;
unsigned int enc_read;

//// UART parameters
//#define BAUDRATE 9600 // must match PC end
//#define PB_DIVISOR (1 << OSCCONbits.PBDIV) // read the peripheral bus divider, FPBDIV
//#define PB_FREQ SYS_FREQ/PB_DIVISOR // periperhal bus frequency
//
//// useful ASCII/VT100 macros for PuTTY
//#define clrscr() printf( "\x1b[2J")
//#define home()   printf( "\x1b[H")
//#define pcr()    printf( '\r')
//#define crlf     putchar(0x0a); putchar(0x0d);
//#define backspace 0x08
//#define str_buffer_size 20
//#define max_chars 32 // for input buffer
//#define timer2rate 625000 //ticks per 1sec
//#define max_pwm 40000 //PWM

 //UART Initializations
//int count, items, num_char;
//static char rxchar='0'; 		//received character
//int num;
//char str_buffer[str_buffer_size];


static PT_THREAD (protothread_UART(struct pt *pt))
{
    PT_BEGIN(pt);
      while(1) {
        //printf("this is working");
        PT_YIELD_TIME_msec(1000);
        //PT_SPAWN(pt, &pt_output, PutSerialBuffer(&pt_output) );
        }
        // never exit while
        
       // END WHILE(1)
  PT_END(pt);
  }
static PT_THREAD (protothread_spiWrite(struct pt *pt))
{
    printf("encoder is 0x10\r\n");
    PT_BEGIN(pt);
    cmd_NOP = 0; //0x00
    while(1){
        tft_spiwrite(cmd_NOP);
        enc_read = ReadSPI2();
        PT_YIELD_TIME_msec(500);
        //return enc_read;
    }
    PT_END(pt);
}

static PT_THREAD (protothread_spiZeroPt(struct pt *pt))
{
    PT_BEGIN(pt);
    cmd_zeroPt = 112; //0x70
    printf("encoder is 0x01\r\n");
    while(1){
        PT_YIELD_TIME_msec(500);
//        enc_read = tft_spiwrite(cmd_zeroPt);
//        while (enc_read == 128){ //0x80
//            enc_read = tft_spiwrite(cmd_NOP);
//        }
//        if (enc_read == 128) { //0x80
//            return enc_read;
//        }
        }
    PT_END(pt);
}

static PT_THREAD (protothread_spiReadPos(struct pt *pt))
{
    PT_BEGIN(pt);
    cmd_rdpos = 16; //0x10
    cmd_NOP = 0;
    while(1){
        printf("encoder is 0xa5\r\n");
        tft_spiwrite(cmd_rdpos);
        enc_read = ReadSPI2(); // Step 1: Master sends rd_pos command. Encoder responds with idle character.
        while (enc_read == 0xA5){ //convert hex to int 0xa5   165
            printf("encoder must be 0xa5\r\n");
            tft_spiwrite(cmd_NOP); // Step 2: Continue sending nop_a5 command while encoder response is 0xA5
            enc_read = ReadSPI2();
        }
        if(enc_read == 0x10){ //0x10    16
            // Step 3: If response was 0x10 (rd_pos), send nop_a5 and receive MSB position
            // (lower 4 bits of this byte are the upper 4 of the 12-bit position)
            printf("enc_read is finally 0x10\r\n");
            tft_spiwrite(cmd_NOP);
            pos_MSB = ReadSPI2();
            tft_spiwrite(cmd_NOP); // Step 4: Send second nop_a5 command and receive LSB position (lower 8 bits of 12-bit positon)
            pos_LSB = ReadSPI2();
//            sprintf(pos_MSB,pos_LSB);
        }
        //printf(&enc_read);
        PT_YIELD_TIME_msec(500);
        //return pos_MSB, pos_LSB;
    }
    PT_END(pt);
}
// === Main ======================================================
void main(void) {
    __XC_UART = 2;

    /* 	Initialize PPS */
    // specify PPS group, signal, logical pin name
    //PPSInput (2, U2RX, RPB11); // Assign U2RX to pin RPB11 -- Physical pin 22 on 28 PDIP
    //PPSOutput(4, RPB10, U2TX); // Assign U2TX to pin RPB10 -- Physical pin 21 on 28 PDIP
    PPSInput (3, SDI2, RPA4); // Assign U2RX to pin RB0 -- Physical pin 12 on 28 PDIP. This is MISO
    PPSOutput(2, RPB1, SDO2); // Assign U2TX to pin RB1 -- Physical pin 5 on 28 PDIP. This is MOSI
    mPORTBSetBits(BIT_2); // RPB2 for CSB -- Physical pin 6 on 28 PDIP.
    mPORTBSetPinsDigitalOut(BIT_3); // RPB3 for SCK -- Physical pin 7 on 28 PDIP.
    
    //SYSTEMConfigPerformance(PBCLK);
    ANSELA = 0; ANSELB = 0; 
    // === config threads ==========
    // turns OFF UART support and debugger pin, unless defines are set
    PT_setup();
   
    UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART2, PB_FREQ, BAUDRATE);
    UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
    
    // PuTTY
    clrscr();  //clear PuTTY screen
    home();
    // By default, MPLAB XC32's libraries use UART2 for STDOUT.
    // This means that formatted output functions such as printf()
    // will send their output to UART2
    printf("UART Interface Initialized\n\r");
    // === setup system wide interrupts  ========
    INTEnableSystemMultiVectoredInt();

      // === set up SPI ===================
  // SCK2 is pin 26 
  // SDO2 (MOSI) is in PPS output group 2, RPB1 pin 5
   PPSOutput(2, RPB1, SDO2);
  // SDI2 (MISO) is PPS output group 3, RPA4 pin 12
   PPSInput(3,SDI2,RPA4);

  // control CS for RAM
   mPORTBSetPinsDigitalOut(BIT_9);
  //PORTSetPinsDigitalOut(IOPORT_B, BIT_0); //IOPORT_PIN_0
   mPORTBSetBits(BIT_9);
        
  // divide Fpb by 2, configure the I/O ports. Not using SS in this example
  // 8 bit transfer CKP=1 CKE=1
  // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
  // For any given peripherial, you will need to match these
   SpiChnOpen(spiChn, SPI_OPEN_ON | SPI_OPEN_MODE8 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , spiClkDiv);
    
	CloseADC10();	// ensure the ADC is off before setting the configuration
    
      // init the threads
    //PT_INIT(&pt_UART);
   PT_INIT(&pt_spiReadPos);
    PT_INIT(&pt_spiWrite);
   PT_INIT(&pt_spiZeroPt);
    
    // round-robin scheduler for threads
    while (1){
        PT_SCHEDULE(protothread_spiWrite(&pt_spiWrite));
        PT_SCHEDULE(protothread_spiReadPos(&pt_spiReadPos));
        PT_SCHEDULE(protothread_spiZeroPt(&pt_spiZeroPt));
        
    }
} // main
