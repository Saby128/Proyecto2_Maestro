/* 
 * File:  Proyecto 2 - Pic Maestro
 * Author: 
 * Saby Andrade 20882
 * Pablo Fuentes 20888
 *
 * Pic Maestro: Control de entradas analogicas, cambio de estados, control
 *              USART por medio de interfaz en computadoras
 * 
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT    // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF               // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF              // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF              // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF                 // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF                // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF              // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF               // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF              // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF                // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V           // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF                // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 1000000        // Frecuencia de oscilador en 1 MHz
#define FLAG_SPI 0xFF             // Variable bandera para lectura del esclavo
#define IN_MIN 0                  // Valor minimo de entrada del potenciometro
#define IN_MAX 255                // Valor m?ximo de entrada del potenciometro
#define IN_MAX2 63                // Valor m?ximo de entrada del potenciometro (Para entradas de USART))
#define OUT_MIN 0                 // Valor minimo de ancho de pulso de se?al PWM
#define OUT_MAX 125               // Valor m?ximo de ancho de pulso de se?al PWM
#define OUT_MAX2 127              // Valor m?ximo de entrada del potenciometro (Para contolar motores SPI))
#define OUT_MAX3 80               // Mapeo para garra a 45?
#define STATE PORTBbits.RB0       //Botones a utilizar
#define ACTION PORTBbits.RB1
#define ACTION2 PORTBbits.RB2
/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
uint8_t LECTURA_POT, var1, var2, var3, var4;        //variables de grabado EEPROM
uint8_t LEC1, LEC2, LEC3, LEC4;                     //variables de reproducci?n
uint8_t CHECK, RECIV;                               //Temp. Verificacion de servo
unsigned short CCPR = 0, CCPR_2 = 0;                //Modulo CCP
/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);
uint8_t read_EEPROM(uint8_t address);
void write_EEPROM(uint8_t address, uint8_t data);

/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    
    //INTERRUPCIONES DEL ADC - CONTROL MANUAL
    if(PIR1bits.ADIF){
        if(ADCON0bits.CHS == 0){                                                   //Servo 1
            SSPBUF = (uint8_t) (map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX2));   //Enviado por SPI esclavo
            var1=ADRESH;// Guardados "temporales" antes de llegar a EEPROM
        PIR1bits.ADIF = 0; 
        }
        else if(ADCON0bits.CHS == 1){   //POT 2.
            CCPR = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);                  //Servo 2
            CCPR1L = (uint8_t)(CCPR>>2);                                           //Usando CCP1 maestro
            CCP1CONbits.DC1B = CCPR & 0b11;
            var2=ADRESH;
        PIR1bits.ADIF = 0; 
        }
        else if(ADCON0bits.CHS == 2){                                              //Servo 3
            CCPR_2 = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX3);               //Usando CCP2 maestro
            CCPR2L = (uint8_t)(CCPR_2>>2);
            CCP2CONbits.DC2B0 = CCPR_2 & 0b01;    
            CCP2CONbits.DC2B1 = (CCPR_2 & 0b10)>>1;
            var3=ADRESH;
        PIR1bits.ADIF = 0;}
        else if(ADCON0bits.CHS == 3){                                              //Servo 4
            SSPBUF = (uint8_t) (map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX2)+128);//Enviado por SPI maestro
            var4=ADRESH;
        PIR1bits.ADIF = 0;}
    }
    
    //Interrupciones del PORTB - ESTADOS
    if(INTCONbits.RBIF){
        if (!STATE){                            //Cambio de Estado
            PORTD = (uint8_t)(PORTD << 1);
            if (PORTD == 8){PORTD = 1;}
        }
        else if (!ACTION){                      //Boton de acci?n
                                                //Dependiendo del estado
                                                //Cambia la acci?n
            if (PORTDbits.RD0==1){              //Estado 1: Escribe en la EEPROM  
                write_EEPROM(1, var1);          
                __delay_ms(50);
                write_EEPROM(2, var2);
                __delay_ms(50);
                write_EEPROM(3, var3);
                __delay_ms(50);
                write_EEPROM(4, var4);
                __delay_ms(50);
                 INTCONbits.RBIF = 0;
            }
            else if (PORTDbits.RD1==1){
                LEC1 = read_EEPROM(1);          //REPRODUCCI?N Servo 1
                SSPBUF = (uint8_t) (map(LEC1, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX2));
                __delay_ms(50);
                LEC2 = read_EEPROM(2);          //REPRODUCCI?N Servo 2
                CCPR = map(LEC2, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
                CCPR1L = (uint8_t)(CCPR>>2);
                CCP1CONbits.DC1B = CCPR & 0b11;
                __delay_ms(50);
                LEC3 = read_EEPROM(3);          //REPRODUCCI?N Servo 3
                CCPR_2 = map(LEC3, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX3); 
                CCPR2L = (uint8_t)(CCPR_2>>2);
                CCP2CONbits.DC2B0 = CCPR_2 & 0b01;    
                CCP2CONbits.DC2B1 = (CCPR_2 & 0b10)>>1;
                __delay_ms(50);
                LEC4 = read_EEPROM(4);          //REPRODUCCI?N Servo 4
                SSPBUF = (uint8_t) (map(LEC4, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX2)+128);
                __delay_ms(50);
                 INTCONbits.RBIF = 0;
            }
        }
        
        else if (!ACTION2){                 //Pulsador De guardado de segunda posici?n
            if (PORTDbits.RD0==1){
                write_EEPROM(5, var1);
                __delay_ms(50);
                write_EEPROM(6, var2);
                __delay_ms(50);
                write_EEPROM(7, var3);
                __delay_ms(50);
                write_EEPROM(8, var4);
                __delay_ms(50);
                 INTCONbits.RBIF = 0;
            }
            else if (PORTDbits.RD1==1){
                LEC1 = read_EEPROM(5);          //REPRODUCCI?N 1
                SSPBUF = (uint8_t) (map(LEC1, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX2));
                __delay_ms(50);
                LEC2 = read_EEPROM(6);          //REPRODUCCI?N 2
                CCPR = map(LEC2, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
                CCPR1L = (uint8_t)(CCPR>>2);
                CCP1CONbits.DC1B = CCPR & 0b11;
                __delay_ms(50);
                LEC3 = read_EEPROM(7);
                CCPR_2 = map(LEC3, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX3); 
                CCPR2L = (uint8_t)(CCPR_2>>2);
                CCP2CONbits.DC2B0 = CCPR_2 & 0b01;    
                CCP2CONbits.DC2B1 = (CCPR_2 & 0b10)>>1;
                __delay_ms(50);
                LEC4 = read_EEPROM(8);
                SSPBUF = (uint8_t) (map(LEC4, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX2)+128);
                __delay_ms(50);
                 INTCONbits.RBIF = 0;
            }
        }
        
         INTCONbits.RBIF = 0;
    }
    
        //COMUNICACI?N EUSART
        if(PIR1bits.RCIF){                      //Comuniaci?n con la interfaz en PC        
            
            if (PORTDbits.RD2){
            RECIV = 0b00111111 & RCREG;         //Verificaci?n del valor a mapear
            CHECK = 0b11000000 & RCREG;         //Verifica que servo es
            
           //MOVIMIENTO DE SERVOMOTORES POR CONTROL DE PC
           if (CHECK==192){                     //SERVO 1
               CCPR = map(RECIV, IN_MIN, IN_MAX2, OUT_MIN, OUT_MAX);
               CCPR1L = (uint8_t)(CCPR>>2);
               CCP1CONbits.DC1B = CCPR & 0b11;
           }
           else if (CHECK==64){                 //SERVO 2
               SSPBUF = (uint8_t) (map(RECIV, IN_MIN, IN_MAX2, OUT_MIN, OUT_MAX2)+0);
           }
           else if (CHECK==128){                //SERVO 3
               SSPBUF = (uint8_t) (map(RECIV, IN_MIN, IN_MAX2, OUT_MIN, OUT_MAX2)+128);
           }
           else if (CHECK==0){                  //SERVO 4
               CCPR_2 = map(RECIV, IN_MIN, IN_MAX2, OUT_MIN, OUT_MAX3); 
               CCPR2L = (uint8_t)(CCPR_2>>2);
               CCP2CONbits.DC2B0 = CCPR_2 & 0b01;    
               CCP2CONbits.DC2B1 = (CCPR_2 & 0b10)>>1;
           }
            }}
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){
       if (PORTDbits.RD0==1){               //SI ESTAMOS EN CONTROL MANUAL
        if(ADCON0bits.GO == 0){             // No hay proceso de conversion
            if (ADCON0bits.CHS == 0){ADCON0bits.CHS = 0b0001;}
            else if (ADCON0bits.CHS == 1){ADCON0bits.CHS = 0b0010;}
            else if (ADCON0bits.CHS == 2){ADCON0bits.CHS = 0b0011;}
            else if (ADCON0bits.CHS == 3){ADCON0bits.CHS = 0b0000;}
            __delay_us(40);
            ADCON0bits.GO = 1;              // Iniciamos proceso de conversi?n
        }
       }}
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){       
    
    // Configuraci?n del oscilador interno
    OSCCONbits.IRCF = 0b100;    // 1MHz
    OSCCONbits.SCS = 1;         // Reloj interno
    
    // Configuraci?n de puertos
    ANSEL   =    0b1111;
    ANSELH  =    0;
    
    TRISA   =  0b1111;
    PORTA   = 0x00;
    
    TRISC   = 0x90;
    PORTC   = 0x00;
    
    TRISB   = 0b111;
    PORTB   = 0x00;
    
    TRISD   = 0x00;
    PORTD   = 0x01;
    
    //Configuraci?n del PULL-UP PORTB
    OPTION_REGbits.nRBPU = 0; 
    WPUB = 0b111;
    IOCB = 0b111;
    PORTB=PORTB;
    
    // Configuraci?n ADC
    ADCON0bits.ADCS = 0b00;     // Fosc/2
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 0b0000;    // Seleccionamos el AN0
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(100);             // Sample time
    
    // Configuraci?n PWM
    TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
    TRISCbits.TRISC1 = 1;
    PR2 = 31;                   // periodo de 2ms -> 0.002 = (PR2+1)*4*1/(Fosc)*Pes
    
    // Configuraci?n CCP
    CCP1CON = 0;                // Apagamos CCP1
    CCP1CONbits.P1M = 0;        // Modo single output
    CCP1CONbits.CCP1M = 0b1100; // PWM
    CCP2CON = 0;                // Apagamos CCP2
    CCP2CONbits.CCP2M = 0b1100;
    
    CCPR1L = 32>>2;
    CCPR2L = 32>>2;                 //Resolucion 125
    CCP1CONbits.DC1B = 32 & 0b11;    // 0.5ms ancho de pulso / 25% ciclo de trabajo
    CCP2CONbits.DC2B0 = 32 & 0b01;         // 
    CCP2CONbits.DC2B1 = (32 & 0b10)>>1; 
    
    //TMR2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
    T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Encendemos TMR2
    while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente
    
    TRISCbits.TRISC2 = 0;       // Habilitamos salida de PWM
    TRISCbits.TRISC1 = 0;
    
    // Configuraci?n de SPI  
    // SSPCON<5:0>
    SSPCONbits.SSPM  =  0b0000; //-> FOSC/4 -> 250kbits/s
    SSPCONbits.CKP   =       0;
    SSPCONbits.SSPEN =       1;
    
    // Configuraci?n del MAESTRO     
    // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1;
    SSPSTATbits.SMP = 1;
    SSPBUF = LECTURA_POT;
    
    // Configuraciones de comunicacion serial
    //SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG=25 <- Valores de tabla 12-5
    TXSTAbits.SYNC = 0;         // Comunicaci?n ascincrona (full-duplex)
    TXSTAbits.BRGH = 1;         // Baud rate de alta velocidad 
    BAUDCTLbits.BRG16 = 1;      // 16-bits para generar el baud rate
    
    SPBRG = 25;
    SPBRGH = 0;                 // Baud rate ~9600, error -> 0.16%
    
    RCSTAbits.SPEN = 1;         // Habilitamos comunicaci?n
    TXSTAbits.TX9 = 0;          // Utilizamos solo 8 bits
    TXSTAbits.TXEN = 1;         // Habilitamos transmisor
    RCSTAbits.CREN = 1;         // Habilitamos receptor

    // Configuraci?n de interrucpiones
    INTCONbits.RBIF = 0;
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;
    INTCONbits.RBIE = 1;        //Int PORTB
    INTCONbits.PEIE = 1;        //Globales y Perifericas
    PIE1bits.RCIE = 1;
    INTCONbits.GIE = 1;
}

/*interpolaci?n
*  y = y0 + [(y1 - y0)/(x1-x0)]*(x-x0)
*  -------------------------------------------------------------------
*  | x0 -> valor m?nimo de ADC | y0 -> valor m?nimo de ancho de pulso|
*  | x  -> valor actual de ADC | y  -> resultado de la interpolaci?n | 
*  | x1 -> valor m?ximo de ADC | y1 -> valor m?ximo de ancho de puslo|
*  ------------------------------------------------------------------- 
*/
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
        unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
} 

/*------------------------------------------------------------------------------
 *MEMORIA EEPROM - FUNCI?N
 -----------------------------------------------------------------------------*/
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