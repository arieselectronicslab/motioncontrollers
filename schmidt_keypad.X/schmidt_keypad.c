/* 
 * File:   schmidt_keypad.c
 * Author: mr_ts
 *
 * Created on 24 November, 2021, 3:49 PM
 */


// DSPIC30F3014 Configuration Bit Settings

// 'C' source line config statements

#define FCY 2500000ULL //Fcy = (Fosc/4) Fosc = 10MHz

#pragma config FOSFPR = HS              // Oscillator (HS)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_OFF         // POR Timer Value (Timer Disabled)
#pragma config BODENV = BORV42          // Brown Out Voltage (4.2V)
#pragma config BOREN = PBOR_OFF         // PBOR Enable (Disabled)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>


#include <stdio.h>
#include <stdlib.h>

/*
 * 
 */

#define SWITCH_FOCUS_UP PORTAbits.RA11 
#define SWITCH_FOCUS_DOWN PORTDbits.RD3 
#define SWITCH_FOCUS_JOY PORTBbits.RB0 

//#define SETTIMEOUT {timeout =0; T3CONbits.TON = 0;T3CONbits.TCKPS = 2 ;IFS0bits.T3IF = 0;PR3=2499;TMR3 = 0 ;T3CONbits.TON = 1;}
#define SETTIMEOUT      {PR3=12000; timeout =0; T3CONbits.TON = 0;T3CONbits.TCKPS = 2 ;IFS0bits.T3IF = 0;TMR3 = 0 ;T3CONbits.TON = 1;}
#define UART_SETTIMEOUT {PR3=60000; timeout =0; T3CONbits.TON = 0;T3CONbits.TCKPS = 2 ;IFS0bits.T3IF = 0;TMR3 = 0 ;T3CONbits.TON = 1;}
#define WIND_SETTIMEOUT {PR3=60000; timeout =0; T3CONbits.TON = 0;T3CONbits.TCKPS = 1 ;IFS0bits.T3IF = 0;TMR3 = 0 ;T3CONbits.TON = 1;}
#define CHECKTIMEOUT {if(IFS0bits.T3IF){timeout =1;}}
#define TIMEOUT timeout

void init(void);
void rs232_init(void) ;
void i2c_init(void);
void setint(void) ;
void  adc_init(void);
void  adc_sample(void);
void bin_io_1(unsigned char*, unsigned char);
void bin_io_2(unsigned char*, unsigned char);
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void);
void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt(void);

void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt(void);
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt(void);

void init_lcd(void);
void write_char_lcd(unsigned char);
void write_str_lcd(unsigned char*, unsigned char);
void write_com_lcd(unsigned char);
void bin_io_lcd(void);



//--------------------------------------------------------------------------------------------------------------------

//ADC buffer--------
unsigned int adc_value[12];
//------------------

//UART_buffers------
unsigned char tx_1_data[25], tx_1_count,tx_2_data[25], tx_2_count;
unsigned char rx_1_data[25],rec_1_count,rx_2_data[25],rec_2_count;
unsigned char rec1_count, rec2_count ;

//------------------

unsigned int timer_1_counter = 0 ;
unsigned char timer_2_counter = 0 ;
unsigned char timer_3_counter = 0 ;
unsigned char timer_4_counter = 0 ;

long int_count_t1, int_count_t2, int_count_t4 ; 



char i2c_success = 0 ;
unsigned char screen= 0 ;
unsigned char lcd_write_counter=0;

struct
{
    union 
    {
        unsigned char status ;// Button status 
        struct
        {
            unsigned due:1;
            unsigned sent:1;
            unsigned :1;
            unsigned :1;
            unsigned :1;
            unsigned :1;
            unsigned :1;
            unsigned :1;   
        };
    };
    
} command;
struct
{
    int jog_focus;
      
    int jog_zero_preset, max_step_rate;
            
    int jog_zero_neg, jog_zero_pos;
           
    unsigned char jog_focus_x_low ;// Button status 
    unsigned char jog_focus_x_high ;// Button status 
    
    int pot_focus;
    unsigned char pot_focus_low ;// Button status 
    unsigned char pot_focus_high ;// Button status 

    unsigned int jog_axes;
    unsigned char jog_axes_x_low ;// Button status 
    unsigned char jog_axes_y_high ;// Button status
    
    unsigned int pot_axes;
    unsigned char pot_axes_low ;// Button status 
    unsigned char pot_axes_high ;// Button status 

    unsigned int pot_dome;
    unsigned char pot_dome_low ;// Button status 
    unsigned char pot_dome_high ;// Button status 
    
    char focus_step; 
    int focus_velocity, focus_old_velocity;
    int abs_focus_velocity;
    
    union
    {
        unsigned char button_state_buffer1 ;// Button status 
        struct
        {
            unsigned on_off:1;
            unsigned dome_left:1;
            unsigned dome_right:1;
            unsigned focus_joy:1;
            unsigned focus_up:1;
            unsigned focus_down:1;
            unsigned focus_velocity_cmd:1;
            unsigned focus_position_cmd:1;   
        };
    };
        
    union
    {
        unsigned char button_state_buffer2 ;// Button status 
        struct
        {
            unsigned axes_joy:1;
            unsigned axes_fwd:1;
            unsigned axes_rev:1;
            unsigned axes_select:1;
            unsigned :1;
            unsigned :1;
            unsigned :1;
            unsigned :1;   
        };
    };
    
} keypad;

//--


int main(int argc, char** argv) 
{

    init();
    rs232_init();
    i2c_init();
    setint();
    adc_init();
    init_lcd();

    I2CCONbits.I2CEN = 1;
 

    T1CONbits.TON = 1;
    T2CONbits.TON = 1;
    ADCON1bits.ADON=1;  // turn ADC ON

    while(1)
    {
        U2STAbits.OERR = 0 ;
        U1STAbits.OERR = 0 ;
    }

    return (EXIT_SUCCESS);
}

void init()
{
    keypad.max_step_rate= 1000;
    keypad.jog_zero_preset=2048;
    keypad.jog_zero_neg =-100;
    keypad.jog_zero_pos = 100;
    
    TRISBbits.TRISB0 = 1 ;
        
    TRISAbits.TRISA11 = 1;
    TRISDbits.TRISD2 = 1;
    TRISDbits.TRISD3 = 1;
    TRISDbits.TRISD8 = 1;
    TRISDbits.TRISD9 = 1;
    
    
    //TxCON: Type A Time Base Register
    //bit 15 TON: Timer On Control bit
    //  1 = Starts the timer
    //  0 = Stops the timer
    T2CONbits.TON = 0;
    //bit 14 Unimplemented: Read as ?0?
    //bit 13 TSIDL: Stop in Idle Mode bit
    //  1 = Discontinue timer operation when device enters Idle mode
    //  0 = Continue timer operation in Idle mode
    T2CONbits.TSIDL = 0 ;
    //bit 12-7 Unimplemented: Read as ?0?
    //bit 6 TGATE: Timer Gated Time Accumulation Enable bit
    //  1 = Gated time accumulation enabled
    //  0 = Gated time accumulation disabled
    //  (TCS must be set to ?0? when TGATE = 1. Reads as ?0? if TCS = 1)
    T2CONbits.TGATE = 0;
    //bit 5-4 TCKPS<1:0>: Timer Input Clock Prescale Select bits
    //  11 = 1:256 prescale value
    //  10 = 1:64 prescale value
    //  01 = 1:8 prescale value
    //  00 = 1:1 prescale value
    T2CONbits.TCKPS = 1 ;
    //bit 3 Unimplemented: Read as ?0?
    //bit 2 TSYNC: Timer External Clock Input Synchronization Select bit
    //When TCS = 1:
    //1 = Synchronize external clock input
    //0 = Do not synchronize external clock input
    //When TCS = 0:
    //This bit is ignored. Read as ?0?. Timer1 uses the internal clock when TCS = 0.
    //bit 1 TCS: Timer Clock Source Select bit
    //1 = External clock from pin TxCK
    //0 = Internal clock (FOSC/4)
    T2CONbits.TCS = 0;

    //serial send timer
    TMR2 = 0 ;      
    //PR2=62500;
    PR2=31250;
    //bit 0 Unimplemented: Read as ?0?
     //Timer1 interrupt set to (inverse of 2500000/8/62500) = 1/200 ms
    //OFFSET by 3036 = 65536-62500 i.e. 0xBDC, so that interrupt is raised at 200 ms
    //Timer1 is used for serial transmit every .2 s
    //transmit interval time is increased in multiple of .2 s by using additional user counter
    //ex. for 10 sec sampling time use counter value 10/0.2 = 50.

    //control system timer
    T1CONbits.TON = 0;
    T1CONbits.TSIDL = 0 ;
    T1CONbits.TGATE = 0;
    T1CONbits.TCKPS = 0 ;
    T1CONbits.TCS = 0;

    TMR1 = 0 ; 
    PR1 = 25000;
    //Timer interrupt set to (inverse of 2500000/8/(65536-3036)) = 200 ms
    //Timer interrupt set to (inverse of 2500000/1/2500) = 1 ms
    //OFFSET by 63036 = 65536-2500 i.e. 0xf63c, so that interrupt is raised at 1 ms
    //Timer2 is used for sampling the data every .4 s
    //sampling time is increased in multiple of .4 s by using additional user counter
    //ex. for 10 sec sampling time use counter value 10/0.4 = 25.
    
    //communication timeout timer
    T3CONbits.TON = 0;
    T3CONbits.TSIDL = 0 ;
    T3CONbits.TGATE = 0;
    T3CONbits.TCKPS = 3 ;
    T3CONbits.TCS = 0;

    TMR3 = 0 ; 
    PR3 = 25000;
    //Timer interrupt set to (inverse of 2500000/1/2500) = 1 ms
    //OFFSET by 63036 = 65536-2500 i.e. 0xf63c, so that interrupt is raised at 1 ms
    //Timer2 is used for sampling the data every .4 s
    //sampling time is increased in multiple of .4 s by using additional user counter
    //ex. for 10 sec sampling time use counter value 10/0.4 = 25.
    
    


}

void    rs232_init()
{
    //------------UART 1

    //UXMODE: UARTX Mode Register
    //bit 15 UARTEN: UART Enable bit
    //1 = UART is enabled. UART pins are controlled by UART as defined by UEN<1:0> and UTXEN control bits.
    //0 = UART is disabled. UART pins are controlled by corresponding PORT, LAT, and TRIS bits.
    //bit 14 Unimplemented: Read as ?0?
    //bit 13 USIDL: Stop in Idle Mode bit
    //1 = Discontinue operation when device enters Idle mode
    //0 = Continue operation in Idle mode
    //bit 12 Unimplemented: Read as ?0?
    //bit 11 Reserved: Write ?0? to this location
    //bit 10 ALTIO: UART Alternate I/O Selection bit
    //1 = UART communicates using UxATX and UxARX I/O pins
    //0 = UART communicates using UxTX and UxRX I/O pins
    //Note: The alternate UART I/O pins are not available on all devices. See device data sheet for details.
    //bit 9-8 Reserved: Write ?0? to these locations
    //bit 7 WAKE: Enable Wake-up on Start bit Detect During Sleep Mode bit
    //1 = Wake-up enabled
    //0 = Wake-up disabled
    //bit 6 LPBACK: UART Loopback Mode Select bit
    //1 = Enable Loopback mode
    //0 = Loopback mode is disabled
    //bit 5 ABAUD: Auto Baud Enable bit
    //1 = Input to Capture module from UxRX pin
    //0 = Input to Capture module from ICx pin
    //bit 4-3 Unimplemented: Read as ?0?
    //bit 2-1 PDSEL<1:0>: Parity and Data Selection bits
    //11 = 9-bit data, no parity
    //10 = 8-bit data, odd parity
    //01 = 8-bit data, even parity
    //00 = 8-bit data, no parity
    //bit 0 STSEL: Stop Selection bit
    //1 = 2 Stop bits
    //0 = 1 Stop bit

    U1MODEbits.UARTEN = 1 ;
    U1MODEbits.USIDL = 0 ;
    U1MODEbits.ALTIO = 1;   //If ALTIO = 1, then alternate pins UxATX and UxARX are used
    U1MODEbits.PDSEL = 0;   //
    U1MODEbits.STSEL = 0 ;

    //Baud Rate = FCY/16/(UxBRG + 1)
    U1BRG = 7;
    U1STAbits.UTXEN = 0;
    //UXSTA: UARTX Status and Control Register
    //bit 15 UTXISEL: Transmission Interrupt Mode Selection bit
        //1 = Interrupt when a character is transferred to the Transmit Shift register and as result, the transmit buffer becomes empty
        //0 = Interrupt when a character is transferred to the Transmit Shift register (this implies that there is at least
        //one character open in the transmit buffer)
    //bit 14-12 Unimplemented: Read as ?0?
    //bit 11 UTXBRK: Transmit Break bit
        //1 = UxTX pin is driven low, regardless of transmitter state
        //0 = UxTX pin operates normally
    //bit 10 UTXEN: Transmit Enable bit
        //1 = UART transmitter enabled, UxTX pin controlled by UART (if UARTEN = 1)
        //0 = UART transmitter disabled, any pending transmission is aborted and buffer is reset. UxTX pin controlled by PORT.
    //bit 9 UTXBF: Transmit Buffer Full Status bit (Read Only)
        //1 = Transmit buffer is full
        //0 = Transmit buffer is not full, at least one more data word can be written
    //bit 8 TRMT: Transmit Shift Register is Empty bit (Read Only)
        //1 = Transmit shift register is empty and transmit buffer is empty (the last transmission has completed)
        //0 = Transmit shift register is not empty, a transmission is in progress or queued in the transmit buffer
    //bit 7-6 URXISEL<1:0>: Receive Interrupt Mode Selection bit
        //11 =Interrupt flag bit is set when Receive Buffer is full (i.e., has 4 data characters)
        //10 =Interrupt flag bit is set when Receive Buffer is 3/4 full (i.e., has 3 data characters)
        //0x =Interrupt flag bit is set when a character is received
    //bit 5 ADDEN: Address Character Detect (bit 8 of received data = 1)
        //1 = Address Detect mode enabled. If 9-bit mode is not selected, this control bit has no effect.
        //0 = Address Detect mode disabled
    //bit 4 RIDLE: Receiver Idle bit (Read Only)
        //1 = Receiver is Idle
        //0 = Data is being received
    //bit 3 PERR: Parity Error Status bit (Read Only)
        //1 = Parity error has been detected for the current character
        //0 = Parity error has not been detected
    //bit 2 FERR: Framing Error Status bit (Read Only)
        //1 = Framing Error has been detected for the current character
        //0 = Framing Error has not been detected
    //bit 1 OERR: Receive Buffer Overrun Error Status bit (Read/Clear Only)
        //1 = Receive buffer has overflowed
        //0 = Receive buffer has not overflowed
    //bit 0 URXDA: Receive Buffer Data Available bit (Read Only)
        //1 = Receive buffer has data, at least one more character can be read
        //0 = Receive buffer is empty
    //---------------UART2
    //UXMODE: UARTX Mode Register
    //Upper Byte:
    //UARTEN ? USIDL ? reserved ALTIO reserved reserved
    //Lower Byte:
    //WAKE LPBACK ABAUD ? ? PDSEL<1:0> STSEL
    U2MODEbits.UARTEN = 1 ;
    U2MODEbits.USIDL = 0 ;
    U2MODEbits.PDSEL = 0;
    U2MODEbits.STSEL = 0 ;
    U2BRG = 7;
    U2STAbits.UTXEN = 1;
    U2STAbits.URXISEL = 0 ;
    IFS1bits.U2TXIF = 0 ;




    //UXSTA: UARTX Status and Control Register
    //Upper Byte:
    //UTXISEL ? ? ? UTXBRK UTXEN UTXBF TRMT
    //URXISEL<1:0> ADDEN RIDLE PERR FERR OERR URXDA



}


void    setint()
{

    
    INTCON1bits.NSTDIS = 0;

    //reserved 0x7
    IPC2bits.U1RXIP = 0x04; //serial 4
    IPC6bits.U2RXIP = 0x04; //serial 4
    IPC0bits.T1IP = 0x3;    //control
    //reserved 0x2
    IPC1bits.T2IP = 0x1;    //send
     
    IEC0bits.T1IE = 1 ;
    IFS0bits.T1IF = 0;

    IEC0bits.T2IE = 1 ;
    IFS0bits.T2IF = 0;
    
    //disabled
    IEC0bits.T3IE = 0 ;
    IFS0bits.T3IF = 0;

    IEC0bits.U1RXIE = 1;
    IFS0bits.U1RXIF = 0 ;

    IEC1bits.U2RXIE = 1;
    IFS1bits.U2RXIF = 0 ;
 
    IFS0bits.U1TXIF = 0 ;
        
}
void  adc_init()
{
    TRISBbits.TRISB1 = 1 ;  //AN0/RB0 pin input     //focus_joy_x
    TRISBbits.TRISB2 = 1 ;  //AN1/RB1 pin input     //focus_joy_y
    TRISBbits.TRISB8 = 1 ;  //AN2/RB2 pin input     //focus_pot
    TRISBbits.TRISB9 = 1 ;  //AN3/RB3 pin input     //axes_joy_x
    TRISBbits.TRISB10 = 1 ;  //AN/RB10 pin input     //axes_joy_y
    TRISBbits.TRISB11 = 1 ;  //AN/RB11 pin input     //axes_pot    
    TRISBbits.TRISB12 = 1 ;  //AN/RB12 pin input     //dome_pot
     
    //ADCON1: A/D Control Register 1
    //bit 15 ADON: A/D Operating Mode bit
    //  1 = A/D converter module is operating
    //  0 = A/D converter is off
    ADCON1bits.ADON=0;
    //bit 14 Unimplemented: Read as ?0?
    //bit 13 ADSIDL: Stop in Idle Mode bit
    //  1 = Discontinue module operation when device enters Idle mode
    //  0 = Continue module operation in Idle mode
    ADCON1bits.ADSIDL = 0;
    //bit 12-10 Unimplemented: Read as ?0?
    //bit 9-8 FORM<1:0>: Data Output Format bits
    //  11 = Signed fractional (DOUT = sddd dddd dddd 0000)
    //  10 = Fractional (DOUT = dddd dddd dddd 0000)
    //  01 = Signed integer (DOUT = ssss sddd dddd dddd)
    //  00 = Integer (DOUT = 0000 dddd dddd dddd)
    ADCON1bits.FORM = 0 ;
    //bit 7-5 SSRC<2:0>: Conversion Trigger Source Select bits
    //    111 = Internal counter ends sampling and starts conversion (auto convert)
    //    110 = Reserved
    //    101 = Reserved
    //    100 = Reserved
    //    011 = Motor Control PWM interval ends sampling and starts conversion
    //    010 = General purpose Timer3 compare ends sampling and starts conversion
    //    001 = Active transition on INT0 pin ends sampling and starts conversion
    //    000 = Clearing SAMP bit ends sampling and starts conversion
    ADCON1bits.SSRC=7;
    //bit 4-3 Unimplemented: Read as ?0?
    //bit 2 ASAM: A/D Sample Auto-Start bit
    //    1 = Sampling begins immediately after last conversion completes. SAMP bit is auto set.
    //    0 = Sampling begins when SAMP bit set
    ADCON1bits.ASAM = 1 ;
    //bit 1 SAMP: A/D Sample Enable bit
    //    1 = At least one A/D sample/hold amplifier is sampling
    //    0 = A/D sample/hold amplifiers are holding
    //    When ASAM = 0, writing ?1? to this bit will start sampling.
    //    When SSRC = 000, writing ?0? to this bit will end sampling and start conversion.
    ADCON1bits.SAMP = 0 ;
    //bit 0 DONE: A/D Conversion Status bit
    //    1 = A/D conversion is done
    //    0 = A/D conversion is not done
    //    Clearing this bit will not effect any operation in progress.
    //    Cleared by software or start of a new conversion.

    
    //ADCON2: A/D Control Register 2
    //bit 15-13 VCFG<2:0>: Voltage Reference Configuration bits
    //000 AVDD AVSS
    //001 External VREF+ pin AVSS
    //010 AVDD External VREF- pin
    //011 External VREF+ pin External VREF- pin
    //1xx AVDD AVSS
    ADCON2bits.VCFG = 0;
    
    ADCON2bits.CSCNA = 1;

    ADCON2bits.SMPI = 2;
            
    //bit 12 Reserved: User should write ?0? to this location
    //bit 11 Unimplemented: Read as ?0?
    //bit 10 CSCNA: Scan Input Selections for CH0+ S/H Input for MUX A Input Multiplexer Setting bit
    //1 = Scan inputs
    //0 = Do not scan inputs
    //bit 9-8 Unimplemented: Read as ?0?
    //bit 7 BUFS: Buffer Fill Status bit
    //Only valid when BUFM = 1 (ADRES split into 2 x 8-word buffers)
    //1 = A/D is currently filling buffer 0x8-0xF, user should access data in 0x0-0x7
    //0 = A/D is currently filling buffer 0x0-0x7, user should access data in 0x8-0xF
    //bit 6 Unimplemented: Read as ?0?
    //bit 5-2 SMPI<3:0>: Sample/Convert Sequences Per Interrupt Selection bits
    //1111 = Interrupts at the completion of conversion for each 16th sample/convert sequence
    //1110 = Interrupts at the completion of conversion for each 15th sample/convert sequence
    //.....
    //0001 = Interrupts at the completion of conversion for each 2nd sample/convert sequence
    //0000 = Interrupts at the completion of conversion for each sample/convert sequence
    //bit 1 BUFM: Buffer Mode Select bit
    //1 = Buffer configured as two 8-word buffers ADCBUF(15...8), ADCBUF(7...0)
    //0 = Buffer configured as one 16-word buffer ADCBUF(15...0)
    ADCON2bits.BUFM = 0;
    
    //bit 0 ALTS: Alternate Input Sample Mode Select bit
    //1 = Uses MUX A input multiplexer settings for first sample, then alternate between MUX B and MUX A input
    //multiplexer settings for all subsequent samples
    //0 = Always use MUX A input multiplexer settings
    ADCON2bits.ALTS = 0;

    //ADCON3: A/D Control Register 3
    //bit 15-13 Unimplemented: Read as ?0?
    //bit 12-8 SAMC<4:0>: Auto Sample Time bits
    //11111 = 31 TAD
    //·····
    //00001 = 1 TAD
    //00000 = 0 TAD
    ADCON3bits.SAMC = 14;   //1 TAD per bit + 1 TAD use 14 TAD for 12 bits
    //bit 7 ADRC: A/D Conversion Clock Source bit
    //1 = A/D internal RC clock
    //0 = Clock derived from system clock
    ADCON3bits.ADRC =  0;
    //bit 6 Unimplemented: Read as ?0?
    //bit 5-0 ADCS<5:0>: A/D Conversion Clock Select bits
    //111111 = TCY/2 * (ADCS<5:0> + 1) = 32 ? TCY
    //······
    //000001 = TCY/2 ? (ADCS<5:0> + 1) = TCY
    //000000 = TCY/2 ? (ADCS<5:0> + 1) = TCY/2
    ADCON3bits.ADCS = 4 ; //TAD =TCY(ADCS + 1)/2  ; minimum TAD time of 333.33 nsec ; use 1 us
                          // for 1 us ADCS = 4, therefore TAD = .4us*5/2 = 1 us
    //ADCHS: A/D Input Select Register
    //ADCHS = 0;
    //ADCHSbits.CH0NA = 0 ; 
    //ADCHSbits.CH0NB = 0 ;
    //ADCHSbits.CH0SA = 0;
    //ADCHSbits.CH0SB = 2;
    //ADCHSbits.CH0SA0= 0 ;
    //ADCHSbits.CH0SA1 = 1 ;
    //ADCHSbits.CH0SA2 = 1 ;
    //ADCHSbits.CH0SA3 = 0 ;
    ADCHS = 0;
    ADCHSbits.CH0SA = 2;

            
    //ADPCFG: A/D Port Configuration Register
    //bit 15-0 PCFG<15:0>: Analog Input Pin Configuration Control bits
    //  1 = Analog input pin in Digital mode, port read input enabled, A/D input multiplexer input connected to AVSS
    //  0 = Analog input pin in Analog mode, port read input disabled, A/D samples pin voltage
    //ADPCFG = 0xff;
    ADPCFGbits.PCFG0 = 1 ;
    ADPCFGbits.PCFG1 = 0 ;
    ADPCFGbits.PCFG2 = 0 ;
    ADPCFGbits.PCFG8 = 0;
    ADPCFGbits.PCFG9 = 0 ;
    ADPCFGbits.PCFG10 = 0 ;
    ADPCFGbits.PCFG11 = 0 ;
    ADPCFGbits.PCFG12 = 0 ;
    
    //ADCSSL: A/D Input Scan Select Register
    //CSSL<15:0>: A/D Input Pin Scan Selection bits
    //1 = Select ANx for input scan
    //0 = Skip ANx for input scan
    ADCSSL = 0;
    ADCSSLbits.CSSL0 = 0 ;
    ADCSSLbits.CSSL1 = 1 ;
    ADCSSLbits.CSSL2 = 1 ;
    ADCSSLbits.CSSL8 = 1 ;
    ADCSSLbits.CSSL9 = 0 ;
    ADCSSLbits.CSSL10 = 0 ;
    ADCSSLbits.CSSL11 = 0 ;
    ADCSSLbits.CSSL12 = 0 ;

}

void init_lcd()
{
    

}

void i2c_init()
{
    //six registers
    //Control Register (I2CCON): This register allows control of the I2C operation.
    //Status Register (I2CSTAT): This register contains status flags indicating the module state during I2C operation.
    //Receive Buffer Register (I2CRCV): This is the buffer register from which data bytes can be read. The I2CRCV register is a read only register.
    //Transmit Register (I2CTRN): This is the transmit register; bytes are written to this register during a transmit operation. The I2CTRN register is a read/write register.
    //Address Register (I2CADD): The I2CADD register holds the slave device address.
    //Baud Rate Generator Reload Register (I2CBRG): Holds the baud rate generator reload value for the I2C module baud rate generator.


    //1) I2CCON
    I2CCONbits.A10M = 0;        //0=7 bit address else 10 bit
    I2CCONbits.I2CEN = 0;
    //5) I2C Address
    I2CADD = 0x20;
    //6) BAUDRATE (Fcy/Fscl - Fcy/1111111) -1
    I2CBRG = 21;

 }




void  adc_sample()
{
}


char i2c_write(char i2c_add, char i2c_data)
{

     /*----Write operation----
     * StartBit--SlaveAddress--W--ACK--Address--ACK--Data--ACK--StopBit
     * 1) Master issues start bit (S) followed by slave address (0X40) and then W bit (0) indicating write operation
     * 2) Slave acknowledges (A) by pulling SDA line low for high duration of the ninth SCL cycle
     * 3) Master issues register address where to write the data
     * 4) Slave acknowledges (A)
     * 5) Master issues data to be written in the register
     * 6) Slave acknowledges (A)
     * 7) Master issues stop bit (P)
    */

   // I2CCONbits.RCEN    = 0 ;           //receive enable

    /*
    IFS0bits.MI2CIF = 0;
    I2CCONbits.RCEN    = 0 ;       //receive enable

    //start
    I2CCONbits.SEN     = 1 ;       //Start initiated (1)
    
    SETTIMEOUT
    while(I2CCONbits.SEN)
    {
        CHECKTIMEOUT
        if(TIMEOUT)
        {
            I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
            return 0;
        }
    }

    SETTIMEOUT
    while(!IFS0bits.MI2CIF)
    {
        CHECKTIMEOUT
        if(TIMEOUT)
        {
            I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
            return 0;
        }
    }
    IFS0bits.MI2CIF = 0;

    //send address
    I2CTRN  = 0x27;//i2c_add ;     //slave address (1)
    SETTIMEOUT
    while(!IFS0bits.MI2CIF)
    {
          CHECKTIMEOUT
          if(TIMEOUT)
           {
                I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
                return 0;
           }
    }
    IFS0bits.MI2CIF = 0;

    //send data
    I2CTRN  = 0x00 ;     //slave address (1)
    SETTIMEOUT
    while(!IFS0bits.MI2CIF)
    {
        CHECKTIMEOUT
        if(TIMEOUT)
        {
            I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
            return 0;
        }
    }
    IFS0bits.MI2CIF = 0;

    //start
    I2CCONbits.RSEN    = 1 ;       //Start initiated (1)
    SETTIMEOUT
    while(I2CCONbits.RSEN)
    {
        CHECKTIMEOUT
        if(TIMEOUT)
        {
            I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
            return 0;
        }
    }

    SETTIMEOUT
    while(!IFS0bits.MI2CIF)
    {
        CHECKTIMEOUT
        if(TIMEOUT)
        {
            I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
            return 0;
        }
    }
    IFS0bits.MI2CIF = 0;

    //send address
    I2CTRN  = 0x51 ;     //slave address (1)
    SETTIMEOUT
    while(!IFS0bits.MI2CIF)
    {
        CHECKTIMEOUT
        if(TIMEOUT)
        {
            I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
            return 0;
        }
    }
        
    IFS0bits.MI2CIF = 0;

    //stop
    if(!I2CSTATbits.ACKSTAT)
    {
        I2CCONbits.RCEN    = 1 ;           //receive enable
        SETTIMEOUT
        while(!IFS0bits.MI2CIF)
        {
            CHECKTIMEOUT
            if(TIMEOUT)
            {
                I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
                return 0;
            }
        }
        IFS0bits.MI2CIF = 0;

        SETTIMEOUT
        while(!I2CSTATbits.RBF)
        {
            CHECKTIMEOUT
            if(TIMEOUT)
            {
                I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
                return 0;
            }
        }

        rec_hum_h=I2CRCV;

        I2CCONbits.ACKDT=0 ;
        I2CCONbits.ACKEN=1 ;
        SETTIMEOUT
        while(!IFS0bits.MI2CIF)
        {
            CHECKTIMEOUT
            if(TIMEOUT)
            {
                I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
                return 0;
            }
        }
        IFS0bits.MI2CIF = 0;

        I2CCONbits.RCEN    = 1 ;           //receive enable
        SETTIMEOUT
        while(!IFS0bits.MI2CIF)
        {
            CHECKTIMEOUT
            if(TIMEOUT)
            {
                I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
                return 0;
            }
        }
        IFS0bits.MI2CIF = 0;

        SETTIMEOUT
        while(!I2CSTATbits.RBF)
        {
            CHECKTIMEOUT
            if(TIMEOUT)
            {
                I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
                return 0;
            }
        }

        rec_hum_l=I2CRCV;

        I2CCONbits.ACKDT=0 ;
        I2CCONbits.ACKEN=1 ;
        SETTIMEOUT
        while(!IFS0bits.MI2CIF)
        {
            CHECKTIMEOUT
            if(TIMEOUT)
            {
                I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
                return 0;
            }
        }
        IFS0bits.MI2CIF = 0;

        I2CCONbits.RCEN    = 1 ;           //receive enable
        SETTIMEOUT
        while(!IFS0bits.MI2CIF)
        {
            CHECKTIMEOUT
            if(TIMEOUT)
            {
                I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
                return 0;
            }
        }
        IFS0bits.MI2CIF = 0;

        SETTIMEOUT
        while(!I2CSTATbits.RBF)
        {
            CHECKTIMEOUT
            if(TIMEOUT)
            {
                I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
                return 0;
            }
        }

        rec_temp_h=I2CRCV;

        I2CCONbits.ACKDT=0 ;
        I2CCONbits.ACKEN=1 ;
        SETTIMEOUT
        while(!IFS0bits.MI2CIF)
        {
            CHECKTIMEOUT
            if(TIMEOUT)
            {
                I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
                return 0;
            }
        }
        IFS0bits.MI2CIF = 0;

        I2CCONbits.RCEN    = 1 ;           //receive enable
        SETTIMEOUT
        while(!IFS0bits.MI2CIF)
        {
            CHECKTIMEOUT
            if(TIMEOUT)
            {
                I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
                return 0;
            }
        }
        IFS0bits.MI2CIF = 0;

        SETTIMEOUT
        while(!I2CSTATbits.RBF)
        {
            CHECKTIMEOUT
            if(TIMEOUT)
            {
                I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
                return 0;
            }
        }

        rec_temp_l=I2CRCV;

        I2CCONbits.ACKDT=1 ;
        I2CCONbits.ACKEN=1 ;
        SETTIMEOUT
        while(!IFS0bits.MI2CIF)
        {
            CHECKTIMEOUT
            if(TIMEOUT)
            {
                I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
                return 0;
            }
        }
        IFS0bits.MI2CIF = 0;

        I2CCONbits.PEN     = 1 ;           //Stop initiate

        SETTIMEOUT
        while(I2CCONbits.PEN)
        {
            CHECKTIMEOUT
            if(TIMEOUT)
            {
                I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
                return 0;
            }
        }
        
        SETTIMEOUT
        while(!IFS0bits.MI2CIF)
        {
            CHECKTIMEOUT
            if(TIMEOUT)
            {
                I2CCONbits.PEN     = 1 ;       //Stop initiate (7)
                return 0;
            }
        }
        IFS0bits.MI2CIF = 0;


        i2c_success = 1 ;

         __delay_us(500);
    }
    else
    {
        i2c_success = 0 ;
        __delay_us(500);

    }


    return 1;

*/
}


void  bin_io_1(unsigned char *ptr_tx_data, unsigned char local_tx_count)
{
    //UART1
    static char for_count;
    for(for_count=0;for_count<local_tx_count;for_count++)
    {
         U1TXREG=  *(ptr_tx_data+for_count) ;
         if(!U1STAbits.UTXEN)
         {
             U1STAbits.UTXEN = 1;
         }
         while(!IFS0bits.U1TXIF);
         IFS0bits.U1TXIF= 0;
    }
}

void  bin_io_2(unsigned char *ptr_tx_data, unsigned char local_tx_count)
{
    //UART 2
    static char for_count;
    for(for_count=0;for_count<local_tx_count;for_count++)
    {
        U2TXREG= *(ptr_tx_data+for_count) ;
        while(U2STAbits.UTXBF);
       // while(!IFS1bits.U2TXIF);
       // IFS1bits.U2TXIF= 0;
    }
}



void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void)
{
    /* Scan buttons and pots
     */
    IFS0bits.T1IF = 0;
    
    timer_1_counter++;
    
    if(timer_1_counter == 10)
    {
        keypad.focus_down = SWITCH_FOCUS_DOWN ;
        keypad.focus_up = SWITCH_FOCUS_UP ;
        keypad.focus_joy = SWITCH_FOCUS_JOY ;

       // ADCON1bits.SAMP = 1;
       // while(!ADCON1bits.DONE);
        while(!IFS0bits.ADIF);
        keypad.jog_focus = ADCBUF0;     //focus_jog_y
        adc_value[1] = ADCBUF1;     //focus_jog_x
        keypad.pot_focus = ADCBUF2;     //focus_pot
        IFS0bits.ADIF = 0 ;

        if(!command.due)
        {
            if(keypad.focus_down)
            {
                keypad.focus_step = -1;
                keypad.focus_position_cmd = 1; 
            }
            if(keypad.focus_up)
            {
                keypad.focus_step = 1;
                keypad.focus_position_cmd =  1; 
            }
            if(!keypad.focus_up & !keypad.focus_down)
            {
                keypad.focus_step = 0 ;
                keypad.focus_position_cmd = 0 ;
            }

            
            //adc value - zero position normalized by max speed
            keypad.focus_velocity = (keypad.jog_focus-keypad.jog_zero_preset)*1.0/keypad.jog_zero_preset/4096*keypad.pot_focus*keypad.max_step_rate;
                       
            //set error band for zero position
            if((keypad.focus_velocity>keypad.jog_zero_neg) && (keypad.focus_velocity < keypad.jog_zero_pos))
            {
                keypad.focus_velocity = 0;
                keypad.focus_velocity_cmd =  0;
                
            }
            
            if(keypad.focus_old_velocity  != keypad.focus_velocity)
            {
                keypad.focus_old_velocity = keypad.focus_velocity;
                keypad.focus_velocity_cmd = 1;
                keypad.focus_position_cmd = 0 ; 
            }
            
            if(keypad.focus_velocity_cmd|keypad.focus_position_cmd)
                command.due =1;
            
        }
        
        timer_1_counter = 0;
    }
}

void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt(void)
{

    IFS0bits.T2IF = 0;
    
    int_count_t2++;
        if(int_count_t2>99999)
            int_count_t2 = 0;
        
    timer_2_counter++;
  
    //to make step command fast and velocity command slow OR it with focus_step
    if(timer_2_counter == 2 || keypad.focus_position_cmd)    //200 ms timer x 5 = 1000 ms
    {
        if(command.due)
        {
            tx_1_data[0] = 'B' ;
            if(keypad.focus_velocity_cmd)
            {
                if(!keypad.focus_velocity)
                {
                    tx_1_data[1] = '2';
                    tx_1_data[2] = 'C';
                
                    tx_1_data[3] = 'S';


                    tx_1_count = 4; 

                    keypad.focus_velocity_cmd = 0;
                }
            
                
                else
                {
                    tx_1_data[1] = '7';
                    tx_1_data[2] = 'C';

                    keypad.abs_focus_velocity = abs(keypad.focus_velocity);
                    if(keypad.focus_velocity>=0)
                        tx_1_data[3] = 'v';
                    else 
                        tx_1_data[3] = 'V';

                    tx_1_data[4] = keypad.abs_focus_velocity/10000%10+48;
                    tx_1_data[5] = keypad.abs_focus_velocity/1000%10+48;
                    tx_1_data[6] = keypad.abs_focus_velocity/100%10+48;
                    tx_1_data[7] = keypad.abs_focus_velocity/10%10+48;
                    tx_1_data[8] = keypad.abs_focus_velocity%10+48;

                    tx_1_count = 9; 
                    keypad.focus_velocity_cmd = 0;
                }
            }
            if(keypad.focus_position_cmd)
            {
                tx_1_data[1] = '2';
                tx_1_data[2] = 'C';
                
                if(keypad.focus_step == 1)
                    tx_1_data[3] = 'P';
                else 
                    tx_1_data[3] = 'N';

               
                tx_1_count = 4; 
                
                keypad.focus_position_cmd = 0;
            }
        
            command.due = 0;
            bin_io_1(tx_1_data,tx_1_count);   
  
        }
        
        

        timer_2_counter =0 ; 
        
    }
    
     
}



void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt(void)
{
    
     
    IFS1bits.U2RXIF = 0 ;
} 

void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt(void)
{
    IFS0bits.U1RXIF = 0 ;
    
} 