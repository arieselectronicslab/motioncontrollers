/* 
 * File:   schmidt_dspic30f4011_focus.c
 * Author: mr_ts
 *
 * Created on 14 November, 2021, 8:17 PM
 */
// DSPIC30F4011 Configuration Bit Settings

// 'C' source line config statements

// FOSC
#pragma config FPR = HS                 // Primary Oscillator Mode (HS)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_OFF         // PBOR Enable (Disabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN       // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)
//#pragma config ICS = ICS_PGD1           // Comm Channel Select (Use EMUC1 and EMUD1)
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <string.h>
#include <libpic30.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define FCY 2500000ULL //Fcy = (Fosc/4) Fosc = 10MHz



//-------------- 10kHz---------------------------
//PTPER settings
//T_pwm = (PTPER+1)*prescaler/FOSC_4
//ex. T_pwm = (249+1)/2500000 = 0.1 ms or F_osc = 10 kHz
//PDC settings
//14 bit duty cycle register; last 2 bits are for selecting Q1, Q2, Q3, Q4 duty cycle match time
// for Q3 rising edge select last two bits as 10; useful for having 50% duty cycle when time period is odd
// and divide by 2 is not an integer
// Duty cycle 0 and 1 registers 249*2 = 498 in decimal or 0x01f2 in hex
#define STEP_MODE3_OUT      OVDCONbits.POUT1L
#define DIRECTION_MODE4_OUT OVDCONbits.POUT1H 
#define MODE1_OUT           OVDCONbits.POUT2L 
#define MODE2_OUT           OVDCONbits.POUT2H 
#define NOT_ENABLE_OUT      OVDCONbits.POUT3L 
#define NOT_STANDBY_OUT     OVDCONbits.POUT3H 

#define pwm_10kHz   {PTPER = 249 ;\
                    PDC1 = 249 ;}        
                    

//2499 or 0x09C3
#define pwm_1kHz   {PTPER = 2499 ;\  
                    PDC1 = 2499;}
                                

                    
//Stepper motor controller
#define step_cw {OVDCONbits.POVD1L = 1;\
                 DIRECTION_MODE4_OUT=1;\
                 IFS2bits.PWMIF = 0 ;\
                 PTCONbits.PTEN = 1 ;\           
                 while(!IFS2bits.PWMIF);}

#define step_ccw {OVDCONbits.POVD1L = 1;\ 
                 DIRECTION_MODE4_OUT=0;\
                 IFS2bits.PWMIF = 0 ;\
                 PTCONbits.PTEN = 1 ;\          
                 while(!IFS2bits.PWMIF);}



#define FULL_STEP_MODE { NOT_STANDBY_OUT=0;\
                        OVDCONbits.POVD1L = 0;\
                        MODE1_OUT           =0 ;\
                        MODE2_OUT           =0 ;\
                        STEP_MODE3_OUT      =0 ;\
                        DIRECTION_MODE4_OUT =0 ;\
                        NOT_STANDBY_OUT=1;}
                 
#define HALF_STEP_MODE { NOT_STANDBY_OUT=0;\
                        OVDCONbits.POVD1L = 0;\
                        MODE1_OUT=1;\
                        MODE2_OUT=0;\
                        STEP_MODE3_OUT = 1 ;\
                        DIRECTION_MODE4_OUT = 0 ;\
                        NOT_STANDBY_OUT=1;}

#define QTR_STEP_MODE { NOT_STANDBY_OUT=0;\
                        MODE1_OUT=0;\
                        MODE2_OUT=1;\
                        OVDCONbits.POVD1L = 0;\
                        STEP_MODE3_OUT = 0 ;\
                        DIRECTION_MODE4_OUT = 1 ;\
                        NOT_STANDBY_OUT=1;}

#define ONEBYEIGHT_STEP_MODE { NOT_STANDBY_OUT=0;\
                        MODE1_OUT=1;\
                        MODE2_OUT=1;\
                        OVDCONbits.POVD1L = 0;\
                        STEP_MODE3_OUT = 1 ;\
                        DIRECTION_MODE4_OUT = 0 ;\
                        NOT_STANDBY_OUT=1;}                 

#define ONEBY16_STEP_MODE { NOT_STANDBY_OUT=0;\
                        MODE1_OUT=1;\
                        MODE2_OUT=1;\
                        OVDCONbits.POVD1L = 0;\
                        STEP_MODE3_OUT = 1;\
                        DIRECTION_MODE4_OUT = 1 ;\
                        NOT_STANDBY_OUT=1;}                 
/*
 * 
 */

#define ASCII 'A' 
#define BINARY 'B'
#define COMMAND 'C'
#define STATUS 'S'
#define MOVE 'M'
#define STOP 'S'
#define ENABLE 'E'
#define DISABLE 'D'
#define PARAMETER 'P'
        
//ERROR AND WARNING CODES                 
#define  DRIVE_DISABLED  1             
#define  LIMIT_ERROR     2
                 

#define limit_pos PORTDbits.RD0
#define limit_neg PORTDbits.RD2
#define limit_home PORTDbits.RD1
                 
                 
void init(void);
void rs232_init(void) ;
void can_init(void);
void i2c_init(void);
void setint(void) ;
void adc_init(void);
void adc_sample(void);
void encoder_init(void);
void pwm_init(void);
void bin_io_1(unsigned char*, unsigned char);
void bin_io_2(unsigned char*, unsigned char);
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void);
void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt(void);
void __attribute__((__interrupt__, __auto_psv__)) _T4Interrupt(void);

void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt(void);
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt(void);

void __attribute__((__interrupt__, __auto_psv__)) _PWMInterrupt(void);


//--------------------------------------------------------------------------------------------------------------------

//-global variables

//ADC buffer--------
unsigned int adc_1_value, adc_2_value;
float voltage_adc_2;
unsigned char adc_2_value_h,adc_2_value_m,adc_2_value_l,adc_2_value_f;
unsigned char voltage_adc_2_h,voltage_adc_2_m,voltage_adc_2_l,voltage_adc_2_f1,voltage_adc_2_f2,voltage_adc_2_f3;
//------------------

//UART_buffers------
unsigned char tx_1_data[100], tx_1_count,tx_2_data[100], tx_2_count;
unsigned char rx_1_data[100],rec_1_count,rx_2_data[100],rec_2_count;
unsigned char rec1_count, rec2_count ;

//------------------

unsigned int timer_1_counter = 0 ;
unsigned char timer_2_counter = 0 ;
unsigned char timer_3_counter = 0 ;
unsigned char timer_4_counter = 0 ;

long int_count_t1, int_count_t2, int_count_t4 ; 



  
struct 
{
    long encoder_pos ; 
    char encoder_index;
    long position  ;   // um
    long abs_position  ;
    long position_error ;    
    long abs_position_error  ;
    long steps_to_home ;
    long counts    ;   // steps 
    long command   ;   //set point
    long offset    ;   //offset after homing
    long home_pos  ;   //home position (index pulse after limit)
    long plim_pos  ;   //Positive limit position
    long nlim_pos  ;   //Negative limit position
    long soft_plim;     //software positive limit < plim_pos
    long soft_nlim;     //software negative limit > nlim_pos
    unsigned long steps_per_output_motion;  //user defined one unit of output movement. ex. one rotation or 1 mm linear motion
    unsigned int steps_per_motor_rot;   //steps per motor rotation
    unsigned long speed;         // steps/sec
    unsigned long speed_preset; 
    unsigned int reduction;     //motor:output generally = or > 1
    unsigned int device_address;//Device address suffixed in messages
    
    unsigned char error_code;
    
    union
    {
        unsigned int status1;
        struct
        {
            unsigned enable:1;
            unsigned emergency:1;
            unsigned direction:1;
            unsigned plim:1;
            unsigned nlim:1;
            unsigned sw_plim:1;
            unsigned sw_nlim:1;
            unsigned up_down_test:1;
            
            unsigned ok_to_move_pos:1;
            unsigned ok_to_move_neg:1;
            unsigned maint_mode:1;
            unsigned expert:1;
            unsigned homing:1;
            unsigned moving:1 ;
            unsigned homing_completed:1 ;
            unsigned move_done:1;
            
        };
    };
    
    union
    {
        unsigned int status2;
        struct
        {
            unsigned error:1;
            unsigned index_error:1;
            unsigned plim_error:1;
            unsigned nlim_error:1;
            unsigned move_nlim:1;
            unsigned move_plim:1;
            unsigned move_command:1;
            unsigned stop_command:1;
            
            unsigned clockwise_is_positive:1;
            unsigned use_limit:1;
            unsigned use_index_neg:1;   //index after homing
            unsigned use_index_pos:1;   //index after homing
            unsigned home_direction:1;  //homing direction
            unsigned manual_command:1;
            unsigned warning:1;
            unsigned homing_error:1;
            
        };
    };
        
    union
    {
        unsigned int status3;
        struct
        {
            unsigned overvoltage:1;
            unsigned overcurrent:1;
            unsigned over_temp1:1;
            unsigned over_temp2:1;
            unsigned configured:1;
            unsigned error_ack:1;
            unsigned move_ack:1;
            unsigned start_motion:1;
            
            unsigned incomp_rec1_command:1;
            unsigned bad_rec1_command:1;
            unsigned incomp_rec2_command:1;
            unsigned bad_rec2_command:1;
            unsigned invalid_req_position:1;
            unsigned invalid_req_speed:1;
            unsigned :1;
            unsigned homelim:1;
            
        };
    };
    
    union
    {
        unsigned int status4;
        struct
        {
            unsigned plim_sw:1;
            unsigned nlim_sw:1;
            unsigned home_command:1;
            unsigned tem_sensor1_open:1;
            unsigned tem_sensor2_open:1;
            unsigned tem_sensor1_short:1;
            unsigned tem_sensor2_short:1;
            unsigned :1;
            
            unsigned :1;
            unsigned :1;
            unsigned :1;
            unsigned :1;
            unsigned :1;
            unsigned :1;
            unsigned :1;
            unsigned :1;
            
        };
    };
      
    float motor_tem_sensor1 , motor_tem_sensor2;
    unsigned int RT0_1 ;  // NTC RT0 at T0 ?
    float T0_1; //Kelvin
    float RT_1; //current resistance
    unsigned int B1 ;      // K
    unsigned int R1 ;  //R=10K?

    unsigned int RT0_2  ;
    float T0_2  ; //Kelvin
    float RT_2; //current resistance
    unsigned int B2  ;      // K
    unsigned int R2  ; //R=10K?
    float VDD   ; //Supply voltage

} focus;


int main(int argc, char** argv) 
{

    init();
    rs232_init();
    can_init();
    i2c_init();
    setint();
    adc_init();
    encoder_init();
    pwm_init();

    //PWM mode
    //pwm_1kHz
    //FULL_STEP_MODE
    pwm_1kHz
    HALF_STEP_MODE
    //pwm_1kHz
    //QTR_STEP_MODE
    
    //pwm_10kHz
    //ONEBYEIGHT_STEP_MODE
    
     //pwm_10kHz
     //ONEBY16_STEP_MODE
    
    //Enable Timer
    T1CONbits.TON = 1;
    T2CONbits.TON = 1;
    T4CONbits.TON = 1;
   
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
    focus.steps_to_home = 2000;
    focus.RT0_1 = 10000 ;  // NTC RT0 at T0 ?
    focus.T0_1 = 25+273.15 ; //Kelvin
    focus.B1 = 3977 ;      // K
    focus.R1 =  10000 ; //R=10K?

    focus.RT0_2 = 10000 ;  // NTC RT0 at T0 ?
    focus.T0_2 = 25+273.15 ; //Kelvin
    focus.B2 = 3977 ;      // K
    focus.R2 =  10000 ; //R=10K?
    
    focus.VDD = 5.0  ;  //Supply voltage
    
    //limits
    TRISDbits.TRISD0 = 1; 
    TRISDbits.TRISD1 = 1; 
    TRISDbits.TRISD2 = 1; 
   
     

    //constants
    focus.status1=0;
    focus.status2=0;
    focus.position =0  ;  
    focus.counts = 0;
    focus.position_error = 0 ; 
    focus.command = 0   ;    
    focus.offset = 0    ;    
    focus.home_pos = 0  ;    
    focus.plim_pos = 1250  ;    
    focus.nlim_pos = 0  ;    
    focus.soft_plim = 1200;      
    focus.soft_nlim = 50;      
    focus.steps_per_output_motion = 400; 
    focus.steps_per_motor_rot = 400;   
    focus.speed = 250;          
    focus.reduction = 2;      
    focus.device_address = 0X30; 
    focus.speed_preset = 1000; 
    focus.enable = 1;
    
    tx_1_data[0] = 'N';
    tx_1_data[1] = 'o';
    tx_1_data[2] = 't';
    tx_1_data[3] = ' ';
    tx_1_data[4] = 'C';
    tx_1_data[5] = 'o';
    tx_1_data[6] = 'n';
    tx_1_data[7] = 'f';
    tx_1_data[8] = 'i';
    tx_1_data[9] = 'g';
    tx_1_data[10] = 'u';
    tx_1_data[11] = 'r';
    tx_1_data[12] = 'e';
    tx_1_data[13] = 'd';
    tx_1_data[14] = ' ';
    tx_1_data[15] = '\r';
    tx_1_data[16] = '\n';
    
    
 
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
    PR2=62500;
    //PR2=31250;
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
    PR1 = 2500;
    //Timer interrupt set to (inverse of 2500000/8/(65536-3036)) = 200 ms
    //Timer interrupt set to (inverse of 2500000/1/2500) = 1 ms
    //OFFSET by 63036 = 65536-2500 i.e. 0xf63c, so that interrupt is raised at 1 ms
    //Timer2 is used for sampling the data every .4 s
    //sampling time is increased in multiple of .4 s by using additional user counter
    //ex. for 10 sec sampling time use counter value 10/0.4 = 25.
    
    //control system timer
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
    
    //control system timer
    T4CONbits.TON = 0;
    T4CONbits.TSIDL = 0 ;
    T4CONbits.TGATE = 0;
    T4CONbits.TCKPS = 0 ;
    T4CONbits.TCS = 0;

    TMR4 = 0 ; 
    PR4 = 2500;
    //Timer interrupt set to (inverse of 2500000/1/250) = 0.1 ms
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
    



    //UXSTA: UARTX Status and Control Register
    //Upper Byte:
    //UTXISEL ? ? ? UTXBRK UTXEN UTXBF TRMT
    //URXISEL<1:0> ADDEN RIDLE PERR FERR OERR URXDA



}

void pwm_init()
{
    //PORTB RB7 RB6 RB5 RB4 RB3 RB2 RB1 RB0
    //RB0/PWM0, RB1/PWM1, RB2/PWM2, RB3/PWM3, RB4/PWM4, RB5/PWM5
    TRISEbits.TRISE0= 0 ;      //set as output  //PWM1 L
    TRISEbits.TRISE1= 0 ;      //set as output  //PWM1 H 
    TRISEbits.TRISE2= 0 ;      //set as output  //PWM2 L
    TRISEbits.TRISE3= 0 ;      //set as output  //PWM2 H
    TRISEbits.TRISE4= 0 ;      //set as output  //PWM3 L
    TRISEbits.TRISE5= 0 ;      //set as output  //PWM3 H

    //PTCON0: PWM TIMER CONTROL REGISTER 0
    


    //bit 15 PTEN: PWM Time Base Timer Enable bit
    //      1 = PWM time base is on
    //      0 = PWM time base is off

    //bit 14 Unimplemented: Read as ?0
    //BIT 13 PTSIDL: PWM Time Base Stop in Idle Mode bit
        //1 = PWM time base halts in CPU Idle mode
        //0 = PWM time base runs in CPU Idle mode
    //bit 12-8 Unimplemented: Read as ?0?

    //bit 7-4 PTOPS<3:0>: PWM Time Base Output Postscale Select bits
    //    0000 = 1:1 Postscale
    //    0001 = 1:2 Postscale
    //    1111 = 1:16 Postscale
    //bit 3-2 PTCKPS<1:0>: PWM Time Base Input Clock Prescale Select bits
    //      00 = PWM time base input clock is FOSC/4 (1:1 prescale)
    //      01 = PWM time base input clock is FOSC/16 (1:4 prescale)
    //      10 = PWM time base input clock is FOSC/64 (1:16 prescale)
    //      11 = PWM time base input clock is FOSC/256 (1:64 prescale)
    //bit 1-0 PTMOD<1:0>: PWM Time Base Mode Select bits
    //      11 = PWM time base operates in a Continuous Up/Down Count mode with interrupts for double PWM updates
    //      10 = PWM time base operates in a Continuous Up/Down Count mode
    //      01 = PWM time base configured for Single-Shot mode
    //      00 = PWM time base operates in a Free-Running mode

    PTCON = 0X01;  //PWM single shot
    PTCONbits.PTCKPS = 1;
   
    //PTMR: PWM Time Base Register
    //bit 15 PTDIR: PWM Time Base Count Direction Status bit (Read Only)
                //1 = PWM time base is counting down
                //0 = PWM time base is counting up
    //bit 14-0 PTMR <14:0>: PWM Timebase Register Count Value

    PTMRbits.PTDIR = 0;

    //PWMCON1: PWM CONTROL REGISTER 1
    //bit 15-12 Unimplemented: Read as ?0?
    //bit 11-8 PMOD4:PMOD1: PWM I/O Pair Mode bits
        //1 = PWM I/O pin pair is in the independent output
        //0 = PWM I/O pin pair is in the complementary output mode
    //bit 7-4 PEN4H-PEN1H: PWMxH I/O Enable bits(1)
        //1 = PWMxH pin is enabled for PWM output
        //0 = PWMxH pin disabled. I/O pin becomes general purpose I/O
    //bit 3-0 PEN4L-PEN1L: PWMxL I/O Enable bits(1)
        //1 = PWMxL pin is enabled for PWM output
        //0 = PWMxL pin disabled. I/O pin becomes general purpose I/O

  
    PWMCON1bits.PEN1H =1 ;      //0 mean general I/O, 1 means PWM
    PWMCON1bits.PEN1L =1 ;      
    PWMCON1bits.PEN2H =1 ;      //0 mean general I/O, 1 means PWM
    PWMCON1bits.PEN2L =1 ;      
    PWMCON1bits.PEN3H =1 ;      //0 mean general I/O, 1 means PWM
    PWMCON1bits.PEN3L =1 ;      
    
    PWMCON1bits.PTMOD3 = 1;     //1 mean L and H are independent
    PWMCON1bits.PTMOD2 = 1;
    PWMCON1bits.PTMOD1 = 1;
    
    OVDCONbits.POVD3L = 0;      //1 mean controlled by PWM pin, 0 mean controlled by POUT
    OVDCONbits.POVD3H = 0;      
    OVDCONbits.POVD2L = 0;
    OVDCONbits.POVD2H = 0;
    OVDCONbits.POVD1L = 0;
    OVDCONbits.POVD1H = 0;
           
    OVDCONbits.POUT1L = 0;
    OVDCONbits.POUT1H = 0;
    OVDCONbits.POUT2L = 0;
    OVDCONbits.POUT2H = 0;
    OVDCONbits.POUT3L = 0;
    OVDCONbits.POUT3H = 0;
   
    //PWMCON2: PWM CONTROL REGISTER 2
    //bit 15-12 Unimplemented: Read as ?0?
    //bit 11-8 SEVOPS<3:0>: PWM Special Event Trigger Output Postscale Select bits
            //1111 = 1:16 Postscale
            //?
            //?
            //0001 = 1:2 Postscale
            //0000 = 1:1 Postscale
    //bit 7-2 Unimplemented: Read as ?0?
    //bit 2 IUE: Immediate Update Enable bit(1)
            //1 = Updates to the active PDC registers are immediate
            //0 = Updates to the active PDC registers are synchronized to the PWM time base
    //bit 1 OSYNC: Output Override Synchronization bit
            //1 = Output overrides via the OVDCON register are synchronized to the PWM time base
            //0 = Output overrides via the OVDCON register occur on next TCY boundary
    //bit 0 UDIS: PWM Update Disable bit
            //1 = Updates from duty cycle and period buffer registers are disabled

    PWMCON2 = 0 ;

    //time base register
    pwm_1kHz

    //DTCON: DEAD-TIME CONTROL REGISTER
    //R/W-0 R/W-0 R/W-0 R/W-0 R/W-0 R/W-0 R/W-0 R/W-0
    //DTPS1 DTPS0 DT5 DT4 DT3 DT2 DT1 DT0
    //bit 7-6 DTPS<1:0>: Dead-Time Unit A Prescale Select bits
    //      11 = Clock source for dead-time unit is FOSC/16
    //      10 = Clock source for dead-time unit is FOSC/8
    //      01 = Clock source for dead-time unit is FOSC/4
    //      00 = Clock source for dead-time unit is FOSC/2
    //bit 5-0 DT<5:0>: Unsigned 6-Bit Dead-Time Value for Dead-Time Unit bits
    DTCON1 = 0 ;

    //OVDCOND: OUTPUT OVERRIDE CONTROL REGISTER
    //R/W-1 R/W-1 R/W-1 R/W-1 R/W-1 R/W-1 R/W-1 R/W-1
    //POVD7(1) POVD6(1) POVD5 POVD4 POVD3 POVD2 POVD1 POVD0
    //bit 7-0 POVD<7:0>: PWM Output Override bits
    //      1 = Output on PWM I/O pin is controlled by the value in the Duty Cycle register and the PWM time base
    //      0 = Output on PWM I/O pin is controlled by the value in the corresponding POUT bit

    //OVDCONS: OUTPUT STATE REGISTER(1,2)
    //R/W-0 R/W-0 R/W-0 R/W-0 R/W-0 R/W-0 R/W-0 R/W-0
    //POUT7(1) POUT6(1) POUT5 POUT4 POUT3 POUT2 POUT1 POUT0
    //bit 7-0 POUT<7:0>: PWM Manual Output bits
    //      1 = Output on PWM I/O pin is active when the corresponding PWM output override bit is cleared
    //      0 = Output on PWM I/O pin is inactive when the corresponding PWM output override bit is cleared

    FLTACON = 0 ;
 
    SEVTCMP = 0 ;
    
}

void encoder_init()
{
    ADPCFGbits.PCFG3 = 1 ;	//1 for Digital I/Os and 0 for analog input
	ADPCFGbits.PCFG4 = 1 ;	
    ADPCFGbits.PCFG5 = 1 ;	
     
    TRISBbits.TRISB3 = 1 ;  //index
    TRISBbits.TRISB4 = 1 ;  //QEA
    TRISBbits.TRISB5 = 1 ;  //QEB


    //QEICON: QEI Control Register
    //bit 15 CNTERR: Count Error Status Flag bit
    //1 = Position count error has occurred
    //0 = No position count error has occurred
    //bit 14 Unimplemented: Read as ?0?
    //bit 13 QEISIDL: Stop in Idle Mode bit
        //1 = Discontinue module operation when device enters Idle mode
        //0 = Continue module operation in Idle mode
    //bit 12 INDEX: Index Pin State Status bit (Read Only)
        //1 = Index pin is High
        //0 = Index pin is Low
    //bit 11 UPDN: Position Counter Direction Status bit
        //1 = Position Counter Direction is positive (+)
        //0 = Position Counter Direction is negative (-)
    //bit 10-8 QEIM<2:0>: Quadrature Encoder Interface Mode Select bits
        //111 = Quadrature Encoder Interface enabled (x4 mode) with position counter reset by match (MAXCNT)
        //110 = Quadrature Encoder Interface enabled (x4 mode) with Index Pulse reset of position counter
        //101 = Quadrature Encoder Interface enabled (x2 mode) with position counter reset by match (MAXCNT)
        //100 = Quadrature Encoder Interface enabled (x2 mode) with Index Pulse reset of position counter
        //011 = Unused (Module disabled)
        //010 = Unused (Module disabled)
        //001 = Starts 16-bit Timer
        //000 = Quadrature Encoder Interface/Timer off
    //bit 7 SWPAB: Phase A and Phase B Input Swap Select bit
        //1 = Phase A and Phase B inputs swapped
        //0 = Phase A and Phase B inputs not swapped
    //bit 6 PCDOUT: Position Counter Direction State Output Enable bit
        //1 = Position Counter Direction Status Output Enable (QEI logic controls state of I/O pin)
        //0 = Position Counter Direction Status Output Disabled (Normal I/O pin operation)
    //bit 5 TQGATE: Timer Gated Time Accumulation Enable bit
        //1 = Timer gated time accumulation enabled
        //0 = Timer gated time accumulation disabled
    //bit 4-3 TQCKPS<1:0>: Timer Input Clock Prescale Select bits
        //11 = 1:256 prescale value
        //10 = 1:64 prescale value
        //01 = 1:8 prescale value
        //00 = 1:1 prescale value
    //bit 2 POSRES: Position Counter Reset Enable bit
        //1 = Index Pulse resets Position Counter
        //0 = Index Pulse does not reset Position Counter
        //(Bit only applies when QEIM<2:0> = 100 or 110)
    //bit 1 TQCS: Timer Clock Source Select bit
        //1 = External clock from pin QEA (on the rising edge)
        //0 = Internal clock (TCY)
    //bit 0 UDSRC: Position Counter Direction Selection Control bit
        //1 = QEB pin State Defines Position Counter Direction
        //0 = Control/Status bit, UPDN (QEICON<11>), Defines Timer Counter (POSCNT)
  	//QUADRATURE ENCODER INTERFACE CONTROL REGISTER
	QEICONbits.CNTERR = 0 ; 	//0 = No position count error has occurred
	QEICONbits.QEISIDL = 0 ;	//0 = Continue module operation in Idle mode
	QEICONbits.INDX = 0 ;  		//0 = Index pin is Low
	QEICONbits.UPDN = 1 ; 		//0 = Position Counter Direction is negative (-)
	QEICONbits.QEIM = 0x7 ; 	//111 = Quadrature Encoder Interface enabled (x4 mode) with position counter reset by match (MAXCNT)
	QEICONbits.SWPAB = 0 ;  	//0 = Phase A and Phase B inputs not swapped
	QEICONbits.TQGATE = 0 ;  	//0 = Timer gated time accumulation disabled
	QEICONbits.TQCKPS = 0 ; 	//Timer Input Clock Prescale Select bits
	QEICONbits.POSRES = 0 ;		//0 = Index Pulse does not reset Position Counter
	QEICONbits.TQCS = 0 ;		//0 = Internal clock (TCY)
	QEICONbits.UPDN_SRC= 0 ;	//1 = QEB pin State Defines Position Counter Direction
    
    MAXCNT = 2000;
}

void can_init()
{
    TRISFbits.TRISF0 = 1;  //CAN RX input
    TRISFbits.TRISF1 = 0;  //CAN TX output
    
    
    //CiCTRL: CAN Module Control and Status Register
    //bit 15 CANCAP: CAN Message Receive Capture Enable bit
        //1 = Enable CAN capture
        //0 = Disable CAN capture
        //Note: CANCAP is always writable, regardless of CAN module Operating mode.
    //bit 14 Unimplemented: Read as ?0?
    //bit 13 CSIDL: Stop in Idle Mode bit
        //1 = Discontinue CAN module operation when device enters Idle mode
        //0 = Continue CAN module operation in Idle mode
    //bit 12 ABAT: Abort All Pending Transmissions bit
        //1 = Abort pending transmissions in all Transmit Buffers
        //0 = No effect
    //Note: Module will clear this bit when all transmissions aborted.
    //bit 11 CANCKS: CAN Master Clock Select bit
        //1 = FCAN clock is FCY
        //0 = FCAN clock is 4 FCY
    //bit 10-8 REQOP<2:0>: Request Operation Mode bits
        //111 = Set Listen All Messages mode
        //110 = Reserved
        //101 = Reserved
        //100 = Set Configuration mode
        //011 = Set Listen Only mode
        //010 = Set Loopback mode
        //001 = Set Disable mode
        //000 = Set Normal Operation mode
    //bit 7-5 OPMODE<2:0>: Operation Mode bits
        //Note: These bits indicate the current Operating mode of the CAN module. See description for REQOP
    //bits (CiCTRL<10:8>).
    //bit 4 Unimplemented: Read as ?0?
    //bit 3-1 ICODE<2:0>: Interrupt Flag Code bits
        //111 = Wake-up interrupt
        //110 = RXB0 interrupt
        //101 = RXB1 interrupt
        //100 = TXB0 interrupt
        //011 = TXB1 interrupt
        //010 = TXB2 interrupt
        //001 = Error interrupt
        //000 = No interrupt
    //bit 0 Unimplemented: Read as ?0?
    C1CTRLbits.REQOP = 4;
    while(C1CTRLbits.OPMODE !=4);
    
    C1CTRLbits.CANCAP = 1 ;
    C1CTRLbits.CANCKS = 0 ;
    //CiTXnCON: Transmit Buffer Status and Control Register
    C1TX0CON = 0x00000;

    C1TX0SID = 0x00f0;
    C1TX0EID = 0;
    C1TX0DLC = 0x0008;
    
    C1RX0CON=0x0000;
    C1RX1CON =0;
    
    
    
    //CiCFG1: Baud Rate Configuration Register 1
    //bit 15-8 Unimplemented: Read as ?0?
    //bit 7-6 SJW<1:0>: Synchronized Jump Width bits
        //11 = Synchronized jump width time is 4 x TQ
        //10 = Synchronized jump width time is 3 x TQ
        //01 = Synchronized jump width time is 2 x TQ
        //00 = Synchronized jump width time is 1 x TQ
    //bit 5-0 BRP<5:0>: Baud Rate Prescaler bits
        //11 1111 = TQ = 2 x (BRP + 1)/FCAN = 128/FCAN
        //11 1110 = TQ = 2 x (BRP + 1)/FCAN = 126/FCAN
        //.
        //00 0001 = TQ = 2 x (BRP + 1)/FCAN = 4/FCAN
        //00 0000 = TQ = 2 x (BRP + 1)/FCAN = 2/FCAN
    //Note: FCAN is FCY or 4 FCY, depending on the CANCKS bit setting.
    C1CFG1bits.BRP = 3;
    
    //CiCFG2: Baud Rate Configuration Register 2
    //bit 15 Unimplemented: Read as ?0?
    //bit 14 WAKFIL: Select CAN bus Line Filter for Wake-up bit
        //1 = Use CAN bus line filter for wake-up
        //0 = CAN bus line filter is not used for wake-up
    //bit 13-11 Unimplemented: Read as ?0?
    //bit 10-8 SEG2PH<2:0>: Phase Buffer Segment 2 bits
        //111 = length is 8 x TQ
        //.
        //000 = length is 1 x TQ
    //bit 7 SEG2PHTS: Phase Segment 2 Time Select bit
        //1 = Freely programmable
        //0 = Maximum of SEG1PH or information processing time (3 TQ?s), whichever is greater
    //bit 6 SAM: Sample of the CAN bus Line bit
        //1 = Bus line is sampled three times at the sample point
        //0 = Bus line is sampled once at the sample point
    //bit 5-3 SEG1PH<2:0>: Phase Buffer Segment 1 bits
        //111 = length is 8 x TQ
        //.
        //000 = length is 1 x TQ
    //bit 2-0 PRSEG<2:0>: Propagation Time Segment bits
        //111 = length is 8 x TQ
        //.
        //000 = length is 1 x TQ
    C1CFG2 = 0x0018;
    
    C1CTRLbits.REQOP = 0;
    while(C1CTRLbits.OPMODE !=0);
    
}


void    i2c_init()
{
}

void    setint()
{

    INTCON1bits.NSTDIS = 0;

    //reserved 0x7
    IPC5bits.T4IP = 0x6;    //safety
    //can receive 5
    IPC2bits.U1RXIP = 0x04; //serial 4
    IPC6bits.U2RXIP = 0x04; //serial 4
    IPC9bits.PWMIP = 0X03;
    IPC0bits.T1IP = 0x1;    //control
    //reserved 0x2
    IPC1bits.T2IP = 0x2;    //send
     
    IEC0bits.T1IE = 1 ;
    IFS0bits.T1IF = 0;

    IEC0bits.T2IE = 1 ;
    IFS0bits.T2IF = 0;
    
    //disabled
    IEC0bits.T3IE = 0 ;
    IFS0bits.T3IF = 0;

    IEC1bits.T4IE = 1 ;
    IFS1bits.T4IF = 0;

    IEC2bits.PWMIE = 0; 
    IFS2bits.PWMIF = 0;
    
    IEC0bits.U1RXIE = 1;
    IFS0bits.U1RXIF = 0 ;

    IEC1bits.U2RXIE = 1;
    IFS1bits.U2RXIF = 0 ;
  

    IFS0bits.U1TXIF = 0 ;
    
   
}
void  adc_init()
{
    TRISBbits.TRISB2 = 1 ;  //AN2/RB2 pin input     //T sensor 
    TRISBbits.TRISB6 = 1 ;  //AN2/RB6 pin input     //T sensor 

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
    ADCON1bits.ASAM = 1;
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
    //bit 12 Reserved: User should write ?0? to this location
    //bit 11 Unimplemented: Read as ?0?
   
     //bit 10 CSCNA: Scan Input Selections for CH0+ S/H Input for MUX A Input Multiplexer Setting bit
    //1 = Scan inputs
    //0 = Do not scan inputs
   ADCON2bits.CSCNA = 1;

   
    ADCON2bits.CHPS = 0; 
   
    //bit 5-2 SMPI<3:0>: Sample/Convert Sequences Per Interrupt Selection bits
    //1111 = Interrupts at the completion of conversion for each 16th sample/convert sequence
    //1110 = Interrupts at the completion of conversion for each 15th sample/convert sequence
    //.....
    //0001 = Interrupts at the completion of conversion for each 2nd sample/convert sequence
    //0000 = Interrupts at the completion of conversion for each sample/convert sequence
    
   
   ADCON2bits.SMPI = 2;
    
    //bit 9-8 Unimplemented: Read as ?0?
    //bit 7 BUFS: Buffer Fill Status bit
    //Only valid when BUFM = 1 (ADRES split into 2 x 8-word buffers)
    //1 = A/D is currently filling buffer 0x8-0xF, user should access data in 0x0-0x7
    //0 = A/D is currently filling buffer 0x0-0x7, user should access data in 0x8-0xF
    //bit 6 Unimplemented: Read as ?0?
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
    ADCHS = 0;
    ADCHSbits.CH0SA = 2;
    //ADPCFG: A/D Port Configuration Register
    //bit 15-0 PCFG<15:0>: Analog Input Pin Configuration Control bits
    //  1 = Analog input pin in Digital mode, port read input enabled, A/D input multiplexer input connected to AVSS
    //  0 = Analog input pin in Analog mode, port read input disabled, A/D samples pin voltage
    ADPCFGbits.PCFG0 = 1 ;
    ADPCFGbits.PCFG1 = 1 ;
    ADPCFGbits.PCFG2 = 0 ;
    ADPCFGbits.PCFG3 = 1 ;
    ADPCFGbits.PCFG4 = 1 ;
    ADPCFGbits.PCFG5 = 1 ;
    ADPCFGbits.PCFG6 = 0 ;
    ADPCFGbits.PCFG7 = 1 ;
    ADPCFGbits.PCFG8 = 1 ;
    //ADCSSL: A/D Input Scan Select Register
    //CSSL<15:0>: A/D Input Pin Scan Selection bits
    //1 = Select ANx for input scan
    //0 = Skip ANx for input scan
    ADCSSL = 0 ;
    ADCSSLbits.CSSL0 = 0 ;
    ADCSSLbits.CSSL1 = 0 ;
    ADCSSLbits.CSSL2 = 1 ;
    ADCSSLbits.CSSL3 = 0 ;
    ADCSSLbits.CSSL4 = 0 ;
    ADCSSLbits.CSSL5 = 0 ;
    ADCSSLbits.CSSL6 = 1 ;
    ADCSSLbits.CSSL7 = 0 ;
    ADCSSLbits.CSSL8 = 0 ;
}

void  adc_sample()
{
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

//safety loop
void __attribute__((__interrupt__, __auto_psv__)) _T4Interrupt(void)
{

    IFS1bits.T4IF = 0;
    
    int_count_t4++;
        if(int_count_t4>99999)
            int_count_t4 = 0;
    
    while(!IFS0bits.ADIF);
    adc_1_value = ADCBUF0;     //focus_jog_y
    adc_2_value = ADCBUF1;     //focus_jog_x
    IFS0bits.ADIF = 0 ;
    
    if(adc_1_value>1000)
    {
        focus.tem_sensor1_open = 1;
        adc_1_value = 1000;
    }
    else if(adc_1_value<100)
    {
        focus.tem_sensor1_short = 1;
        adc_1_value = 100;
    }
    else 
    {
        focus.tem_sensor1_open = 0;
        focus.tem_sensor1_short = 0;
    }
    
    if(adc_2_value>1000)
    {
        focus.tem_sensor2_open = 1;
        adc_2_value = 1000;
    }
    else if(adc_2_value<100)
    {
        focus.tem_sensor2_short = 1;
        adc_2_value = 100;
    }
    else 
    {
        focus.tem_sensor2_open = 0;
        focus.tem_sensor2_short = 0;
    }
    
    focus.plim_sw = !(limit_pos) ;
    focus.nlim_sw = !(limit_neg) ;
    focus.homelim = !(limit_home) ;
    
    if(focus.error_ack)
    {
     
        focus.incomp_rec1_command = 0 ; 
        focus.bad_rec1_command = 0 ;
        focus.incomp_rec2_command = 0 ;
        focus.bad_rec2_command = 0 ; 
        focus.invalid_req_position = 0 ; 
        focus.invalid_req_speed = 0 ;
        focus.over_temp1 = 0;
        focus.over_temp2 = 0 ;
        focus.overcurrent = 0;
        focus.overvoltage = 0 ;
        focus.plim_error = 0 ;
        focus.nlim_error = 0 ;
        focus.index_error = 0 ;
        focus.error = 0 ;
        
        focus.warning = 0;
        focus.error_ack = 0;
    }

    if(!focus.homing_completed | !focus.configured | focus.homing_error | focus.index_error| focus.nlim_error| focus.plim_error)
    {
        focus.warning = 1;
    }
    
    if((focus.soft_nlim<=focus.nlim_pos) | (focus.soft_plim>=focus.plim_pos))
    {
        focus.warning = 1;
    }
    
    if(focus.position<=focus.soft_nlim)  
    {
        focus.sw_nlim = 1;
    }
    else
    {
        focus.sw_nlim = 0;
    }
    
    if(focus.position>=focus.soft_plim)  
    {
        focus.sw_plim = 1;
     }
    else
    {
        focus.sw_plim = 0;
    }
    
    if(focus.plim_sw|focus.sw_plim)
    {
        focus.plim = 1 ;
    }
    else 
    {
        focus.plim = 0 ;
    }
    if(focus.nlim_sw|focus.sw_nlim)
    {
        focus.nlim = 1 ;
    }
    else 
    {
        focus.nlim = 0 ;
    }
    
    
    if(focus.enable & !focus.emergency & !focus.error) 
    {
        if(!focus.plim)
            focus.ok_to_move_pos = 1 ; 
        else
            focus.ok_to_move_pos = 0 ;
        
        if(!focus.nlim)
            focus.ok_to_move_neg = 1 ; 
        else
            focus.ok_to_move_neg = 0 ;
                    
    }
    else
    {
         focus.ok_to_move_pos = 0 ;
         focus.ok_to_move_neg = 0 ;
         focus.command = focus.position ;
         focus.error = 0 ;
         focus.stop_command =1;
         focus.move_command = 0;
         focus.moving = 0 ; 
       
    }
}

void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void)
{
    /* The focus mechanism slides on a precise THK Ball Screw arrangement which 
     * travels 5 mm/revolution. 
     * The mechanism is coupled to a Superior Electric KM062 bipolar stepper motor 
     * using a timing belt and pulleys offering a reduction of 1:2. 
     * It has an incremental shaft encoder for position measurement and for initial 
     * referencing a PCA-116 series LVDT is used. 
     * The motor and the on-shaft encoder undergo four full rotations generating 
     * four index pulses in the range ±5 mm. 
     * At 0.9 degree per half step the housing can be positioned with a resolution of 6.25 µm.
     * 400 1/2 steps per one motor rotation
     * 800 1/2 steps per one shaft rotation
     * 800 1/2 steps per 5mm travel
     * Total range -/+ 5 mm. or 1600 steps or 4 rotations.
     * Resolution 5/800 mm = 6.25 um
     */
    static long i; 
    
    IFS0bits.T1IF = 0;
    //timer_1_counter++;
   
    
    
  //  ADCON1bits.SAMP = 1;
   // while(!ADCON1bits.DONE);
    //adc_2_value = ADCBUF0;
    if(focus.home_command&(focus.ok_to_move_pos||focus.ok_to_move_neg))
    {
        if(focus.speed == 0)
            PTPER = 0 ; 
        else 
            PTPER = 1.0*2500000/focus.speed - 1;
        
        PDC1 = PTPER ; 
        
        focus.homing_completed = 0;
        focus.home_command = 0;
        focus.counts = 0;
        focus.homing = 1 ; 
        focus.moving = 1; 
        
        
        if(focus.home_direction)
        {
            while(1)
            {
                if(focus.stop_command || focus.move_command || focus.home_command)
                {
                    break;
                }
                                
                if(focus.plim_sw)
                {
                    focus.plim = 1;
                    
                    focus.position = focus.plim_pos; 
                    focus.command = focus.position;
                    focus.position_error = 0;
                    focus.homing = 0 ; 
                    focus.moving = 0; 
                    focus.homing_completed = 1;
                    break;
                }
                
                if(focus.counts==focus.steps_to_home)
                {
                    focus.homing = 0 ; 
                    focus.moving = 0; 
                    focus.homing_error ;
                    
                    break;
                }
                
                step_cw
                focus.counts++;        
                focus.position++;   
             }
        }
        else
        {
            while(1)
            {
                if(focus.stop_command || focus.move_command || focus.home_command)
                {
                    break;                    
                }
                
                if(focus.nlim_sw)
                {
                    focus.nlim = 1;
                    
                    focus.position = focus.nlim_pos;
                    focus.command = focus.position;
                    focus.position_error = 0;
                    
                     focus.homing = 0 ; 
                     focus.moving = 0; 
       
                            
                    break;
                }
                if(focus.counts==focus.steps_to_home)
                {
                    focus.homing = 0 ; 
                    focus.moving = 0; 
                    focus.homing_error ;
                    
                    break;
                }
                
                step_ccw
                focus.counts++;
                focus.position--;
            }
        }
    }
    
    if(focus.move_command&(focus.ok_to_move_pos||focus.ok_to_move_neg))
    {
        if(focus.speed == 0)
            PTPER = 0 ; 
        else 
            PTPER = 1.0*2500000/focus.speed - 1;
        
        PDC1 = PTPER ; 
        focus.move_command = 0;
        focus.moving = 1 ; 
        
        focus.position_error = focus.command - focus.position;

        if(focus.position_error>=0)
        {    
            focus.direction = 1;
            while((focus.position!=focus.command))
            {
                if(focus.stop_command || focus.move_command || focus.home_command)
                {
                    break;
                }
                                
                if(focus.position >= focus.soft_plim)
                {
                     focus.sw_plim = 1;
                    
                    focus.command = focus.position;
                    focus.position_error = 0;
                    break;
                }
                
                step_cw
                focus.position++;   
                focus.position_error = focus.command - focus.position ; 
                
            }

        }
        else
        {
            focus.direction = 0;
            while((focus.position!=focus.command))
            {
                if(focus.stop_command || focus.move_command || focus.home_command)
                {
                    break;                    
                }
                
                if(focus.position <= focus.soft_nlim)
                {
                    focus.sw_nlim = 1;
                    
                    focus.command = focus.position;
                    focus.position_error = 0;
                            
                    break;
                }
                
                step_ccw
                focus.position--;
                focus.position_error = focus.command - focus.position ; 
                

            }
        }
    
        
        focus.moving = 0 ; 
        focus.move_done = 1;
      
    }
    
    C1TX0DLCbits.DLC = 8;
    C1TX0B1 = 1 ;
    C1TX0B2 = 2 ;
    C1TX0B3 = 3 ;
    C1TX0B4 = 4 ;
    C1TX0CONbits.TXREQ =1;
    
}


void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt(void)
{

    IFS0bits.T2IF = 0;
    
    int_count_t2++;
        if(int_count_t2>99999)
            int_count_t2 = 0;
        
    timer_2_counter++;
   
    
    
    if(timer_2_counter == 2)    //200 ms timer x 5 = 1000 ms
    {
        //module
        /* Formula 
         * RT_1 = VDD*R1/VADC - R1 
         * VADC = VDD*ADC_counts/1023 
         * therefore RT_1/R1 = 1023/ADC_counts - 1
         * T = 1/(ln(RT/R)+1/T0)-273.15        
        */ 
        //focus.RT_1 = (1023.0/adc_1_value - 1); //ratio of RT1/R1
        focus.motor_tem_sensor1 = 1/(log(1023.0/adc_1_value - 1)/focus.B1 + 1/focus.T0_1)-273.15;
        
        //focus.RT_2 = (1023.0/adc_2_value - 1); //ratio of RT1/R1
        focus.motor_tem_sensor2 = 1/(log(1023.0/adc_2_value - 1)/focus.B2 + 1/focus.T0_2)-273.15;
                
        
        
        tx_1_data[0] = 'B';
        tx_1_data[1] = focus.device_address/10+48;
        tx_1_data[2] = focus.device_address%10+48;
        tx_1_data[3] = ',';
        
        focus.abs_position = abs(focus.position) ;
        focus.abs_position_error = abs(focus.position_error);
        
        if(focus.position<0)
            tx_1_data[4] = '-';
        else 
            tx_1_data[4] = '+';
        
        
        tx_1_data[5] =  (focus.abs_position)/10000%10+48;
        tx_1_data[6] =  (focus.abs_position)/1000%10+48;
        tx_1_data[7] =  (focus.abs_position)/100%10+48;
        tx_1_data[8] =  (focus.abs_position)/10%10+48;
        tx_1_data[9] =  (focus.abs_position)%10+48;
        tx_1_data[10] =  ',';
        tx_1_data[11] =  (focus.abs_position_error)/10000%10+48;
        tx_1_data[12] =  (focus.abs_position_error)/1000%10+48;
        tx_1_data[13] =  (focus.abs_position_error)/100%10+48;
        tx_1_data[14] =  (focus.abs_position_error)/10%10+48;
        tx_1_data[15] =  (focus.abs_position_error)%10+48;
        tx_1_data[16] =  ','; 
        tx_1_data[17] =  (POSCNT)/10000%10+48;
        tx_1_data[18] =  (POSCNT)/1000%10+48;
        tx_1_data[19] =  (POSCNT)/100%10+48;
        tx_1_data[20] =  (POSCNT)/10%10+48;
        tx_1_data[21] =  (POSCNT)%10+48;
        tx_1_data[22] =  ',';
        if(focus.motor_tem_sensor1<0)
            tx_1_data[23] = '-';
        else 
            tx_1_data[23] = '+';    
        tx_1_data[24] =  (int)abs(focus.motor_tem_sensor1)/100%10+48;
        tx_1_data[25] =  (int)abs(focus.motor_tem_sensor1)/10%10+48;
        tx_1_data[26] =  (int)abs(focus.motor_tem_sensor1)%10+48;
        tx_1_data[27] =  '.'; 
        tx_1_data[28] =  (int)abs(focus.motor_tem_sensor1*10)%10+48;
        tx_1_data[29] =  ',';
        if(focus.motor_tem_sensor2<0)
            tx_1_data[30] = '-';
        else 
            tx_1_data[30] = '+';    
        
        tx_1_data[31] =  (int)abs(focus.motor_tem_sensor2)/100%10+48;
        tx_1_data[32] =  (int)abs(focus.motor_tem_sensor2)/10%10+48;
        tx_1_data[33] =  (int)abs(focus.motor_tem_sensor2)%10+48;
        tx_1_data[34] =  '.'; 
        tx_1_data[35] =  (int)abs(focus.motor_tem_sensor2*10)%10+48;
        tx_1_data[36] =  ','; 
        
        if(focus.soft_nlim<0)
            tx_1_data[37] =  '-';
        else 
            tx_1_data[37] =  '+';
        /*tx_1_data[38] =  adc_1_value/10000%10+48;//abs(focus.soft_nlim)/10000%10+48;
        tx_1_data[39] =  adc_1_value/1000%10+48;//abs(focus.soft_nlim)/1000%10+48;
        tx_1_data[40] =  adc_1_value/100%10+48;//abs(focus.soft_nlim)/100%10+48;
        tx_1_data[41] =  adc_1_value/10%10+48;//abs(focus.soft_nlim)/10%10+48;
        tx_1_data[42] =  adc_1_value%10+48;//abs(focus.soft_nlim)%10+48;
        tx_1_data[43] =  ',';
        */
        tx_1_data[38] =  abs(focus.soft_nlim)/10000%10+48;
        tx_1_data[39] =  abs(focus.soft_nlim)/1000%10+48;
        tx_1_data[40] =  abs(focus.soft_nlim)/100%10+48;
        tx_1_data[41] =  abs(focus.soft_nlim)/10%10+48;
        tx_1_data[42] =  abs(focus.soft_nlim)%10+48;
        tx_1_data[43] =  ',';
         if(focus.soft_plim<0)
            tx_1_data[44] =  '-';
        else 
            tx_1_data[44] =  '+';
        /*tx_1_data[45] =  adc_2_value/10000%10+48;//abs(focus.soft_plim)/10000%10+48;
        tx_1_data[46] =  adc_2_value/1000%10+48;//abs(focus.soft_plim)/1000%10+48;
        tx_1_data[47] = adc_2_value/100%10+48;// abs(focus.soft_plim)/100%10+48;
        tx_1_data[48] = adc_2_value/10%10+48;// abs(focus.soft_plim)/10%10+48;
        tx_1_data[49] =  adc_2_value%10+48;//abs(focus.soft_plim)%10+48;
        tx_1_data[50] =  ',';
        */
        tx_1_data[45] =  abs(focus.soft_plim)/10000%10+48;
        tx_1_data[46] =  abs(focus.soft_plim)/1000%10+48;
        tx_1_data[47] = abs(focus.soft_plim)/100%10+48;
        tx_1_data[48] = abs(focus.soft_plim)/10%10+48;
        tx_1_data[49] =  abs(focus.soft_plim)%10+48;
        tx_1_data[50] =  ',';
                
        tx_1_data[51] =  focus.status1>>8;
        tx_1_data[52] =  focus.status1;
        tx_1_data[53] =  focus.status2>>8;
        tx_1_data[54] =  focus.status2;
        tx_1_data[55] =  focus.status3>>8;
        tx_1_data[56] =  focus.status3;
        tx_1_data[57] =  focus.status4>>8;
        tx_1_data[58] =  focus.status4;
       
        tx_1_data[59] =  '\n';
        tx_1_data[60] =  '\r';


        tx_1_count  =  61;
        tx_2_count  =  61;


        bin_io_1(tx_1_data,tx_1_count);   
        
        //rs232
        //bin_io_2(tx_1_data,tx_2_count);
        timer_2_counter =0 ; 
    }
    
     
}



void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt(void)
{
    IFS1bits.U2RXIF = 0 ;
    while(!U2STAbits.URXDA);
    rx_2_data[0]= U2RXREG;
    if(rx_2_data[0]=='R')
    {
        while(1);
    }
     
    IFS1bits.U2RXIF = 0 ;
} 

void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt(void)
{
    static char i ; 

    for (i=0;i<25;i++)
        rx_1_data[i] = '0';
            
    IFS0bits.U1RXIF = 0 ;
    rx_1_data[0]= U1RXREG;
    
           
    if(rx_1_data[0]=='B')
    {
        
        T3CONbits.TON=0;
        TMR3=0;
        IFS0bits.T3IF = 0 ;
        T3CONbits.TON=1;
        while(!U1STAbits.URXDA)
        {
              
            if(IFS0bits.T3IF)
            {
                T3CONbits.TON =0;
                focus.incomp_rec1_command = 1; 
                return;
                
            }
        }
        
        IFS0bits.U1RXIF = 0 ;
        rx_1_data[1]= U1RXREG-48;
        
       
        for(rec1_count=0;rec1_count<rx_1_data[1];rec1_count++)
        {
            T3CONbits.TON=0;
            TMR3=0;
            IFS0bits.T3IF = 0 ;
            T3CONbits.TON=1;
            while(!U1STAbits.URXDA)
            {
                if(IFS0bits.T3IF)
                {
                    T3CONbits.TON =0;
                    focus.incomp_rec1_command = 1; 
                    return;
                }
            }
            
            IFS0bits.U1RXIF = 0 ;
            rx_1_data[rec1_count+2] = U1RXREG;
        }
        
        T3CONbits.TON=0;
            
        switch(rx_1_data[2])
        {
            case COMMAND:   
                            { 
                                if(rx_1_data[3]== 'H')
                                {
                                    focus.stop_command = 0; 
                                    focus.home_command = 1;
                                    focus.speed = focus.speed_preset;
                                    
                                }
                                
                                if(rx_1_data[3]== 'V'||rx_1_data[3]== 'v')
                                {
                                   if((rx_1_data[4]>= '0'&& rx_1_data[4]<= '9')&(rx_1_data[5]>= '0'&& rx_1_data[5]<= '9')&(rx_1_data[6]>= '0'&& rx_1_data[6]<= '9')&(rx_1_data[7]>= '0'&& rx_1_data[7]<= '9')&(rx_1_data[8]>= '0'&& rx_1_data[8]<= '9'))
                                    {
                                        focus.speed = (rx_1_data[4]-48)*10000+(rx_1_data[5]-48)*1000+(rx_1_data[6]-48)*100+(rx_1_data[7]-48)*10+rx_1_data[8]-48;
                                        
                                        
                                        if(!focus.speed)
                                        {
                                            focus.command = focus.position ; 
                                            focus.move_command = 1;
                                            focus.speed = focus.speed_preset;
                                        }
                                        else 
                                        {
                                            focus.command = 99999;
                                            if(rx_1_data[3]== 'v')
                                                focus.command = -1*focus.command;
                                        
                                            focus.stop_command = 0; 
                                            focus.move_command = 1;
                                        }
                                        
                                       
                                    }
                                    else 
                                    {
                                        focus.invalid_req_position = 1; 
                                        return;
                                    } 
                                     
                                }
                                if(rx_1_data[3]== 'S')
                                {
                                    //focus.stop_command = 1; 
                                    focus.command = focus.position ; 
                                    focus.move_command = 1;
                                    focus.speed = focus.speed_preset;
                                   
                                }
                                if(rx_1_data[3]== '-' || rx_1_data[3]== '+')
                                {   
                                    if((rx_1_data[4]>= '0'&& rx_1_data[4]<= '9')&(rx_1_data[5]>= '0'&& rx_1_data[5]<= '9')&(rx_1_data[6]>= '0'&& rx_1_data[6]<= '9')&(rx_1_data[7]>= '0'&& rx_1_data[7]<= '9')&(rx_1_data[8]>= '0'&& rx_1_data[8]<= '9'))
                                    {
                                        focus.command = (rx_1_data[4]-48)*10000+(rx_1_data[5]-48)*1000+(rx_1_data[6]-48)*100+(rx_1_data[7]-48)*10+rx_1_data[8]-48;
                                        if(rx_1_data[3]== '-')
                                            focus.command = -1*focus.command;
                                        focus.stop_command = 0; 
                                        focus.move_command = 1;
                                        focus.speed = focus.speed_preset;
                                    }
                                    else 
                                    {
                                        focus.invalid_req_position = 1; 
                                        return;
                                    }
                                    
                                }
                                if(rx_1_data[3]== 'P' || rx_1_data[3]== 'N')
                                {                                    
                                    if(rx_1_data[3]== 'N')
                                        focus.command = focus.position -1;
                                    else 
                                        focus.command = focus.position +1;
                                    
                                    focus.stop_command = 0; 
                                    focus.move_command = 1;
                                    focus.speed = focus.speed_preset;
                                }
                                
                                else 
                                {
                                    focus.invalid_req_position = 1; 
                                    focus.stop_command = 0; 
                                    
                                    return;
                                }
                                    
                                break;
                            }

            case STATUS:    {

                                break;  
                            }
            case PARAMETER:
                            {
                                if(rx_1_data[3]== 'W')
                                {
                                    if(rx_1_data[4]== 'O')
                                    {
                                        if((rx_1_data[5]=='-'||rx_1_data[5]=='+')&(rx_1_data[6]>= '0'&& rx_1_data[6]<= '9')&(rx_1_data[7]>= '0'&& rx_1_data[7]<= '9')&(rx_1_data[8]>= '0'&& rx_1_data[8]<= '9')&(rx_1_data[9]>= '0'&& rx_1_data[9]<= '9')&(rx_1_data[10]>= '0'&& rx_1_data[10]<= '9'))
                                        {
                                            focus.offset = (rx_1_data[6]-48)*10000+(rx_1_data[7]-48)*1000+(rx_1_data[8]-48)*100+(rx_1_data[9]-48)*10+rx_1_data[10]-48;
                                        
                                            if(rx_1_data[5]== '-')
                                                focus.offset = -1*focus.offset;
                                        }
                                        else 
                                        {
                                            focus.invalid_req_position = 1; 
                                            return;
                                        }
                                    }
                                
                                    if(rx_1_data[4]== 'P')      //limit sw posi
                                    {
                                        if((rx_1_data[5]=='-'||rx_1_data[5]=='+')&(rx_1_data[6]>= '0'&& rx_1_data[6]<= '9')&(rx_1_data[7]>= '0'&& rx_1_data[7]<= '9')&(rx_1_data[8]>= '0'&& rx_1_data[8]<= '9')&(rx_1_data[9]>= '0'&& rx_1_data[9]<= '9')&(rx_1_data[10]>= '0'&& rx_1_data[10]<= '9'))
                                        {
                                            focus.plim_pos = (rx_1_data[6]-48)*10000+(rx_1_data[7]-48)*1000+(rx_1_data[8]-48)*100+(rx_1_data[9]-48)*10+rx_1_data[10]-48;
                                         
                                            if(rx_1_data[5]== '-')
                                                focus.plim_pos = -1*focus.plim_pos;
                                        }
                                        else 
                                        {
                                            focus.invalid_req_position = 1; 
                                            return;
                                        }
                                    }
                                    
                                    if(rx_1_data[4]== 'N')      //limit sw posi
                                    {
                                        if((rx_1_data[5]=='-'||rx_1_data[5]=='+')&(rx_1_data[6]>= '0'&& rx_1_data[6]<= '9')&(rx_1_data[7]>= '0'&& rx_1_data[7]<= '9')&(rx_1_data[8]>= '0'&& rx_1_data[8]<= '9')&(rx_1_data[9]>= '0'&& rx_1_data[9]<= '9')&(rx_1_data[10]>= '0'&& rx_1_data[10]<= '9'))
                                        {
                                            focus.nlim_pos = (rx_1_data[6]-48)*10000+(rx_1_data[7]-48)*1000+(rx_1_data[8]-48)*100+(rx_1_data[9]-48)*10+rx_1_data[10]-48;
                                            
                                            if(rx_1_data[5]== '-')
                                                focus.nlim_pos = -1*focus.nlim_pos;
                                        }
                                        else 
                                        {
                                            focus.invalid_req_position = 1; 
                                            return;
                                        }
                                        
                                    }
                                    
                                    if(rx_1_data[4]== 'p')      //soft limit
                                    {
                                        if((rx_1_data[5]=='-'||rx_1_data[5]=='+')&(rx_1_data[6]>= '0'&& rx_1_data[6]<= '9')&(rx_1_data[7]>= '0'&& rx_1_data[7]<= '9')&(rx_1_data[8]>= '0'&& rx_1_data[8]<= '9')&(rx_1_data[9]>= '0'&& rx_1_data[9]<= '9')&(rx_1_data[10]>= '0'&& rx_1_data[10]<= '9'))
                                        {
                                            focus.soft_plim = (rx_1_data[6]-48)*10000+(rx_1_data[7]-48)*1000+(rx_1_data[8]-48)*100+(rx_1_data[9]-48)*10+rx_1_data[10]-48;
                                        
                                            if(rx_1_data[5]== '-')
                                                focus.soft_plim = -1*focus.soft_plim;
                                        }
                                        else 
                                        {
                                            focus.invalid_req_position = 1; 
                                            return;
                                        }
                                    }
                                    
                                    if(rx_1_data[4]== 'n')      //soft limit
                                    {
                                        if((rx_1_data[5]=='-'||rx_1_data[5]=='+')&(rx_1_data[6]>= '0'&& rx_1_data[6]<= '9')&(rx_1_data[7]>= '0'&& rx_1_data[7]<= '9')&(rx_1_data[8]>= '0'&& rx_1_data[8]<= '9')&(rx_1_data[9]>= '0'&& rx_1_data[9]<= '9')&(rx_1_data[10]>= '0'&& rx_1_data[10]<= '9'))
                                        {
                                            focus.soft_nlim = (rx_1_data[6]-48)*10000+(rx_1_data[7]-48)*1000+(rx_1_data[8]-48)*100+(rx_1_data[9]-48)*10+rx_1_data[10]-48;
                                        
                                            if(rx_1_data[5]== '-')
                                                focus.soft_nlim = -1*focus.soft_nlim;
                                        }
                                        else 
                                        {
                                            focus.invalid_req_position = 1; 
                                            return;
                                        }
                                    }
                                    
                                    if(rx_1_data[4]== 'S')  //steps per rotation
                                    {
                                        if((rx_1_data[6]>= '0'&& rx_1_data[6]<= '9')&(rx_1_data[7]>= '0'&& rx_1_data[7]<= '9')&(rx_1_data[8]>= '0'&& rx_1_data[8]<= '9')&(rx_1_data[9]>= '0'&& rx_1_data[9]<= '9')&(rx_1_data[10]>= '0'&& rx_1_data[10]<= '9'))
                                        {
                                            focus.steps_per_motor_rot = (rx_1_data[6]-48)*10000+(rx_1_data[7]-48)*1000+(rx_1_data[8]-48)*100+(rx_1_data[9]-48)*10+rx_1_data[10]-48;
                                        }
                                        else 
                                        {
                                            focus.invalid_req_position = 1; 
                                            return;
                                        }
                                    }
                                    
                                    if(rx_1_data[4]== 'R')  //reduction
                                    {
                                        if((rx_1_data[6]>= '0'&& rx_1_data[6]<= '9')&(rx_1_data[7]>= '0'&& rx_1_data[7]<= '9')&(rx_1_data[8]>= '0'&& rx_1_data[8]<= '9')&(rx_1_data[9]>= '0'&& rx_1_data[9]<= '9')&(rx_1_data[10]>= '0'&& rx_1_data[10]<= '9'))
                                        {
                                            focus.reduction = (rx_1_data[6]-48)*10000+(rx_1_data[7]-48)*1000+(rx_1_data[8]-48)*100+(rx_1_data[9]-48)*10+rx_1_data[10]-48;
                                        }
                                        else 
                                        {
                                            focus.invalid_req_position = 1; 
                                            return;
                                        }
                                    }
                                    
                                    if(rx_1_data[4]== 'M')  //steps per op motion
                                    {
                                        if((rx_1_data[6]>= '0'&& rx_1_data[6]<= '9')&(rx_1_data[7]>= '0'&& rx_1_data[7]<= '9')&(rx_1_data[8]>= '0'&& rx_1_data[8]<= '9')&(rx_1_data[9]>= '0'&& rx_1_data[9]<= '9')&(rx_1_data[10]>= '0'&& rx_1_data[10]<= '9'))
                                        {
                                            focus.steps_per_output_motion = (rx_1_data[6]-48)*10000+(rx_1_data[7]-48)*1000+(rx_1_data[8]-48)*100+(rx_1_data[9]-48)*10+rx_1_data[10]-48;
                                        }
                                        else 
                                        {
                                            focus.invalid_req_position = 1; 
                                            return;
                                        }
                                    }
                                    
                                    if(rx_1_data[4]== 'V') //preset speed
                                    {
                                        if((rx_1_data[6]>= '0'&& rx_1_data[6]<= '9')&(rx_1_data[7]>= '0'&& rx_1_data[7]<= '9')&(rx_1_data[8]>= '0'&& rx_1_data[8]<= '9')&(rx_1_data[9]>= '0'&& rx_1_data[9]<= '9')&(rx_1_data[10]>= '0'&& rx_1_data[10]<= '9'))
                                        {
                                            focus.speed_preset = (rx_1_data[6]-48)*10000+(rx_1_data[7]-48)*1000+(rx_1_data[8]-48)*100+(rx_1_data[9]-48)*10+rx_1_data[10]-48;
                                        }
                                        else 
                                        {
                                            focus.invalid_req_position = 1; 
                                            return;
                                        }
                                    }
                                    
                                    if(rx_1_data[4]== 'I') //Device address 
                                    {
                                        if((rx_1_data[6]>= '0'&& rx_1_data[6]<= '9')&(rx_1_data[7]>= '0'&& rx_1_data[7]<= '9')&(rx_1_data[8]>= '0'&& rx_1_data[8]<= '9')&(rx_1_data[9]>= '0'&& rx_1_data[9]<= '9')&(rx_1_data[10]>= '0'&& rx_1_data[10]<= '9'))
                                        {
                                            focus.device_address = (rx_1_data[6]-48)*10000+(rx_1_data[7]-48)*1000+(rx_1_data[8]-48)*100+(rx_1_data[9]-48)*10+rx_1_data[10]-48;
                                        }
                                        else 
                                        {
                                            focus.invalid_req_position = 1; 
                                            return;
                                        }
                                    }
                                    
                                }
                                
                                if(rx_1_data[3]== 'S')
                                {
                                    if(rx_1_data[4]== 'E') //Enable drive
                                    {
                                        NOT_ENABLE_OUT = 0;
                                        focus.enable = 1;
                                    }
                                    if(rx_1_data[4]== 'D') //Disable drive
                                    {
                                        NOT_ENABLE_OUT = 1;
                                        focus.enable = 0;
                                    }
                                    if(rx_1_data[4]== 'M')  //emergency ON
                                    {
                                        focus.emergency = 1;
                                    }
                                    if(rx_1_data[4]== 'C')  //emergency clear
                                    {
                                        focus.emergency = 0; 
                                    }
                                    
                                    if(rx_1_data[4]== 'U')  //User mode
                                    {
                                        focus.expert = 0;
                                        focus.maint_mode = 0;
                                    }
                                    if(rx_1_data[4]== 'X')  //expert mode
                                    {
                                        focus.expert = 1;
                                        focus.maint_mode = 0; 
                                    }
                                    
                                    if(rx_1_data[4]== 'N')  //Engineering mode
                                    {
                                        focus.expert = 0;
                                        focus.maint_mode = 1; 
                                    }
                                    
                                    if(rx_1_data[4]== 'P')  //Director bit
                                    {
                                        focus.clockwise_is_positive = 1; 
                                    }
                                    
                                    if(rx_1_data[4]== 'p')  //Director bit
                                    {
                                        focus.clockwise_is_positive = 0; 
                                    }
                                    
                                    if(rx_1_data[4]== 'H')  //Director bit
                                    {
                                        focus.home_direction = 1; 
                                    }
                                    
                                    if(rx_1_data[4]== 'h')  //Director bit
                                    {
                                        focus.home_direction = 0; 
                                    }
                                    
                                    
                                    if(rx_1_data[4]== 'A')  //Error ack  
                                    {                                         
                                        focus.error_ack = 1; 
                                    }
                                }
                                break;
                            }
            default:
                            focus.bad_rec1_command =1 ;
                            break ;
        }   
    }
    
    
} 


void __attribute__((__interrupt__, __auto_psv__)) _PWMInterrupt(void)
{
    
}