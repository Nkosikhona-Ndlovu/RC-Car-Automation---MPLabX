// FDEVOPTF
#pragma config SOSCHP = OFF             // Secondary Oscillator High Power Enable bit (SOSC oprerates in normal power mode.)
#pragma config ALTI2C = ON              // Alternate I2C1 Pins Location Enable bit (Alternate I2C1 pins are used)
#pragma config FUSBIDIO = ON            // USBID pin control (USBID pin is controlled by the port function)
#pragma config FVBUSIO = ON             // VBUS Pin Control (VBUS pin is controlled by port function)
#pragma config USERID = 0xFFFF          // User ID bits (User ID bits)

// FICD
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)
#pragma config ICS = PGx2               // ICE/ICD Communication Channel Selection bits (Communicate on PGEC2/PGED2)

// FPOR
#pragma config BOREN = BOR3             // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware; SBOREN bit disabled)
#pragma config RETVR = OFF              // Retention Voltage Regulator Enable bit (Retention regulator is disabled)
#pragma config LPBOREN = ON             // Downside Voltage Protection Enable bit (Low power BOR is enabled, when main BOR is disabled)

// FWDT
#pragma config SWDTPS = PS1048576       // Sleep Mode Watchdog Timer Postscale Selection bits (1:1048576)
#pragma config FWDTWINSZ = PS25_0       // Watchdog Timer Window Size bits (Watchdog timer window size is 25%)
#pragma config WINDIS = OFF             // Windowed Watchdog Timer Disable bit (Watchdog timer is in non-window mode)
#pragma config RWDTPS = PS1048576       // Run Mode Watchdog Timer Postscale Selection bits (1:1048576)
#pragma config RCLKSEL = LPRC           // Run Mode Watchdog Timer Clock Source Selection bits (Clock source is LPRC (same as for sleep mode))
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (WDT is disabled)

// FOSCSEL
#pragma config FNOSC = FRCDIV           // Oscillator Selection bits (Fast RC oscillator (FRC) with divide-by-N)
#pragma config PLLSRC = FRC             // System PLL Input Clock Selection bit (FRC oscillator is selected as PLL reference input on device reset)
#pragma config SOSCEN = ON              // Secondary Oscillator Enable bit (Secondary oscillator (SOSC) is enabled)
#pragma config IESO = OFF               // Two Speed Startup Enable bit (Two speed startup is disabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Selection bit (Primary oscillator is disabled)
#pragma config OSCIOFNC = OFF           // System Clock on CLKO Pin Enable bit (OSCO pin operates as a normal I/O)
#pragma config SOSCSEL = OFF            // Secondary Oscillator External Clock Enable bit (Crystal is used (RA4 and RB4 are controlled by SOSC))
#pragma config FCKSM = CSECME           // Clock Switching and Fail-Safe Clock Monitor Enable bits (Clock switching is enabled; Fail-safe clock monitor is enabled)

// FSEC
#pragma config CP = OFF                 // Code Protection Enable bit (Code protection is disabled)

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/attribs.h>

int i;

//multi vectored mode
void __ISR(_ADC_VECTOR, IPL5SOFT) _ADC_HANDLER(void) {
    // LATDbits.LATD3 ^= 1;
    int ADC14Val = ADC1BUF0;    //save ADC value
    if (ADC14Val > 2048)        //if ADC > 2048=3.3/2
        LATCbits.LATC13 = 1;    //set RC13 = HIGH
    else
        LATCbits.LATC13 = 0;    //set RC13 = LOW
    IFS1bits.AD1IF = 0;         //reset interrupt flag
I2C1TRN = 0x23;

    // user code
}

//single vectored mode
void __ISR_SINGLE() _SingleVectorHandler(void){
    if(IFS1bits.AD1IF){//timer 2 interrupt
        int ADC14Val = ADC1BUF0;    //save ADC value
        if (ADC14Val > 2048)        //if ADC > 2048=3.3/2
            LATCbits.LATC13 = 1;    //set RC13 = HIGH
        else
            LATCbits.LATC13 = 0;    //set RC13 = LOW

        IFS1bits.AD1IF = 0;         //reset interrupt flag
    }
    if(IFS1bits.AD1IF){//timer 2 interrupt
        
    }
}

int main(void) {

    //Unlock sequence for SPLLCON and OSCCON registers.
    SYSKEY = 0; // force lock
    SYSKEY = 0xAA996655; // unlock sequence
    SYSKEY = 0x556699AA; // lock sequence

    //Temporarily switch to 8MHz FRC (without PLL), so we can safely change the PLL settings,
    //in case we had previously been already running from the PLL.
    OSCCON = OSCCON & 0xF8FFF87E; //FRC configured for 8MHz output, and set NOSC to run from FRC with divider but without PLL.
    if (OSCCONbits.COSC != OSCCONbits.NOSC) {
        //Initiate clock switching operation.
        OSCCONbits.OSWEN = 1;
        while (OSCCONbits.OSWEN == 1); //Wait for switching complete.
    }

    //Configure the PLL to run from the FRC, and output 24MHz for the CPU + Peripheral Bus (and 48MHz for the USB module)
    SPLLCON = 0x02050080; //PLLODIV = /4, PLLMULT = 12x, PLL source = FRC, so: 8MHz FRC * 12x / 4 = 24MHz CPU and peripheral bus frequency.

    //Now switch to the PLL source.
    OSCCON = OSCCON | 0x00000101; //NOSC = SPLL, initiate clock switch (OSWEN = 1)


    //Wait for PLL startup/lock and clock switching operation to complete.
    for (i = 0; i < 100000; i++) {
        if ((CLKSTATbits.SPLLRDY == 1) && (OSCCONbits.OSWEN == 0))
            break;
    }


    //Enable USB active clock tuning for the FRC
    OSCTUN = 0x00009000; //active clock tuning enabled, source = USB
    SYSKEY = 0; //Re-lock oscillator registers  



    //set all ports as digital
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    ANSELD = 0; //set all as digital
    TRISCbits.TRISC13 = 0;

    //disable interrupts
    __builtin_disable_interrupts();

    //##############################################################
    //ADC

    AD1CON1bits.ON = 0;         //Switch off ADC
    AD1CON1bits.FORM = 0b000;   //Integer 16-bit (DOUT = 0000 0000 0000 0000 0000 00dd dddd dddd)
    AD1CON1bits.MODE12 = 1;     // set 12bit mode

   
    AD1CON1bits.ASAM = 0;       // Sampling start when Timer1 overflows (Auto Off)        
    AD1CON1bits.SSRC = 0b0101;  // TIMER1 starts sampling      
    AD1CON2bits.CSCNA = 1;      // Enable scan      
    AD1CON2bits.SMPI = 2 - 1;   // Set number conversion per interrupt to 2
    AD1CON2bits.BUFM = 0;       //16bit buffer mode    

    // Clock from peripheral clock TPB
   
    AD1CHSbits.CH0NA = 0b000;   // Negative input is VrefL = AVSS
    AD1CON2bits.VCFG = 0b000;   //AVdd AVss

    
    // ADC clock TAD = 8 peripheral clock TPB   
    AD1CON3bits.ADRC = 0;       //Use system clock 24Mhz
    //Need at least 280ns see Table 29-35
    //ADCS = (Tad/(2*Tsrc))-1 = (280ns/(2*1/24e6))-1 = 2.36 so make 3
    AD1CON3bits.ADCS = 3;       //makes Tad > 280ns  
    AD1CON3bits.SAMC = 2;       //sample time SAMC*Tad > 0 2=560ns
    
    
     // Set RB2/AN4 as an analog input
    ANSELBbits.ANSB2 = 1;
    TRISBbits.TRISB2 = 1;

    // Set RC8/AN14 as an analog input
    ANSELCbits.ANSC8 = 1;
    TRISCbits.TRISC8 = 1;
    
    // Add 2 inputs above to the scan list
    AD1CSS = 0; //make sure all channels are deselected first
    AD1CSS =  (1 << 14 | 1<<4); //read channels 4 and 14
   
    ////OR
    //AD1CSSbits.CSS14 = 1;   //scan channel 14
    //AD1CSSbits.CSS4 = 1;    //scan channel 4
    ////          
 
    AD1CON1bits.ON = 1;         // Turn on the ADC
    AD1CON5bits.ASEN = 1;       //auto scan mode
    
    //SET ADC INTERRUPT
    //set interrupt priority and enable peripheral interrupt
    IPC8bits.AD1IP = 5;         // set ADC interrupt priority to 5
    IPC8bits.AD1IS = 0;         // set ADC sub-priority to 0
    IEC1bits.AD1IE = 1;         // enable ADC int   
    IFS1bits.AD1IF = 0;         // Reset ADC interrupt flag
    
    //SET TIMER1 to start sampling
    T1CONbits.ON = 0;           //switch off timer 1
    T1CONbits.TCS = 0;          //internal clock 
    T1CONbits.TCKPS = 0b000;    //1:1    
    TMR1 = 0;                   // Set initial timer value 
    PR1 = 4799;               // Set timer overflow value
    // Enable TIMER 1
    T1CONbits.ON = 1;


    INTCONbits.MVEC = 0;            //multi-vecotred mode
    __builtin_enable_interrupts();  //enable interrupts
    
    
    while (1) {
        Nop();      //do nothing
    }

}


