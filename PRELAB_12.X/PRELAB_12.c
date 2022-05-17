/* 
 * File:   PRELAB_12.c
 * Author: ALBA RODAS
 *
 * Created on 15 de mayo de 2022, 17:09 PM
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h> 

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 1000000
uint8_t address = 0, cont = 0;
/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
uint8_t read_EEPROM(uint8_t address);
void write_EEPROM(uint8_t address, uint8_t data);
/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    
    if(RBIF)
    {
        if(!PORTBbits.RB0)
            PORTEbits.RE0 = 0;
        INTCONbits.RBIF = 0;
        
    }
    if(PIR1bits.ADIF){
        if(ADCON0bits.CHS == 0){
           PORTD = ADRESH;
           ADRESH = address;
        }
        PIR1bits.ADIF = 0;
        
     }
    return;
}
/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){

    ANSEL = 0b00000001;             // ENTRADA --> AN0
    ANSELH = 0;                     // I/O digitales
    
    TRISAbits.TRISA0 = 1;           // AN0 --> PLACE EL POT
    TRISEbits.TRISE0 = 0;           //     
    TRISEbits.TRISE1 = 0;           // 
    TRISEbits.TRISE2 = 0;           // ESTOY EN MODO GUARDAR EN EEPROM
    
    TRISE = 0;
    TRISA = 0b00000001;             // SS ; RA0 --> INPUTS    
    TRISC = 0;
    
    TRISBbits.TRISB1 = 1;           // RB1 --> INPUT PARA PUSH
    TRISBbits.TRISB0 = 1;           // RB0 --> INPUT PARA PUSH
    TRISBbits.TRISB4 = 1;
    OPTION_REGbits.nRBPU = 0;       // ACTIVO RESISTENCIAS INTERNAS PARA EL PORTB
    WPUBbits.WPUB1 = 1;             // PULLUP PARA --> RB1
    WPUBbits.WPUB0 = 1;  
    WPUBbits.WPUB4 = 1;
    
    TRISD = 0;
    PORTA = 0;
    PORTB = 0;                     // LIMPIEZA DE PUERTOS
    PORTE = 0;                     
    PORTD = 0;
    PORTC = 0;
    
    //CONFIGURACI�N DE OSCILADOR
    OSCCONbits.IRCF = 0b100;        //1MHz
    OSCCONbits.SCS = 1;             //Reloj interno

    //CONFIGURACI�N DEL ADC
    ADCON1bits.ADFM = 0;            //Justificado a la izquierda
    ADCON1bits.VCFG0 = 0;           //VDD *Referencias internas
    ADCON1bits.VCFG1 = 0;           //VSS *Referencias internas
    
    ADCON0bits.ADCS = 0b01;         //FOSC/2
    ADCON0bits.CHS = 0;             //AN0 --> POT
    ADCON0bits.ADON = 1;            // on M�dulo ADC
    __delay_us(50);
    
    //CONFIGURACI�N INTERUPCCIONES
    //IOCBbits.IOCB1 = 1;
    IOCBbits.IOCB0 = 1;
    INTCONbits.RBIE = 1;
    INTCONbits.GIE = 1;       //Se habilita las banderas globales
    INTCONbits.PEIE = 1;      //Habilitar interrupciones de perif�ricos
    PIR1bits.ADIF = 0;        //Limpiamos bandera de interrupci�n de ADC
    PIE1bits.ADIE = 1;        //Habilitamos interrupci�n de ADC
}
/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){
    
        PORTC = read_EEPROM(0);
        
        if(ADCON0bits.GO == 0){
            ADCON0bits.GO = 1;
        }
        __delay_ms(10);
        
        if(!PORTBbits.RB1)
        {
            while(!RB1);
            PORTEbits.RE0 = 1;
            SLEEP();
        }
        if (!PORTBbits.RB4){
            while(!RB4);
            write_EEPROM(0, PORTD);
        }

    }
    return;
}
// VISTO EN CLASE:
uint8_t read_EEPROM(uint8_t address){
    EEADR = address;
    EECON1bits.EEPGD = 0;       // Lectura a la EEPROM
    EECON1bits.RD = 1;          // Obtenemos dato de la EEPROM
    return EEDAT;               // Regresamos dato 
}
void write_EEPROM(uint8_t address, uint8_t data){
    EEADR = address;
    EEDAT = data;
    EECON1bits.EEPGD = 0;       // Escritura a la EEPROM
    EECON1bits.WREN = 1;        // Habilitamos escritura en la EEPROM
    
    INTCONbits.GIE = 0;         // Deshabilitamos interrupciones
    EECON2 = 0x55;      
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;          // Iniciamos escritura
    
    EECON1bits.WREN = 0;        // Deshabilitamos escritura en la EEPROM
    INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1;         // Habilitamos interrupciones
}