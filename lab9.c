#include <lab9.h>
#include "lcd.h"
#include "graphics.h"
#include "color.h"
#include "ports.h"
#include "accel.h"
#include <stdio.h>
#include <time.h>
#include <stdint.h>
#include <stdbool.h>

// regaddr[]={0x0F,0x10,0x11};
// Packet[]={0x01,0x09,0x1E};
char strx[20];
char stry[20];
int lower = 0, upper = 4;
int array[4];
int ByteX, ByteY;
unsigned char state=0;
#define play 1
#define menu 0
#define stop 2
int posX = 80;
int posY = 80;
float accX;
float accY;
unsigned int i=0;
int avgArrX[5]={0,0,0,0,0};
int avgArrY[5]={0,0,0,0,0};


//******************************************************************************
// Example Commands ************************************************************
//******************************************************************************

#define SLAVE_ADDR  0x26

/* CMD_TYPE_X_SLAVE are example commands the master sends to the slave.
 * The slave will send example SlaveTypeX buffers in response.
 *
 * CMD_TYPE_X_MASTER are example commands the master sends to the slave.
 * The slave will initialize itself to receive MasterTypeX example buffers.
 * */

#define CMD_TYPE_0_SLAVE      0x02 //xlsb
#define CMD_TYPE_1_SLAVE      0x03 //xmsb
#define CMD_TYPE_2_SLAVE      0x04 //ylsb
#define CMD_TYPE_3_SLAVE      0x05 //ymsb

#define CMD_TYPE_0_MASTER      0x0F
#define CMD_TYPE_1_MASTER      0x10
#define CMD_TYPE_2_MASTER      0x11

#define TYPE_0_LENGTH   1
#define TYPE_1_LENGTH   2
#define TYPE_2_LENGTH   3

#define MAX_BUFFER_SIZE     1

/* MasterTypeX are example buffers initialized in the master, they will be
 * sent by the master to the slave.
 * SlaveTypeX are example buffers initialized in the slave, they will be
 * sent by the slave to the master.
 * */

uint8_t MasterType01 [TYPE_0_LENGTH] = { 0x01};
uint8_t MasterType02 [TYPE_0_LENGTH] = { 0x09};
uint8_t MasterType03 [TYPE_0_LENGTH] = { 0x1E};

uint8_t SlaveType0 [TYPE_0_LENGTH] = {0};

//******************************************************************************
// General I2C State Machine ***************************************************
//******************************************************************************

typedef enum I2C_ModeEnum{
    IDLE_MODE,
    NACK_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    SWITCH_TO_RX_MODE,
    SWITHC_TO_TX_MODE,
    TIMEOUT_MODE
} I2C_Mode;


/* Used to track the state of the software state machine*/
I2C_Mode MasterMode = IDLE_MODE;

/* The Register Address/Command to use*/
uint8_t TransmitRegAddr = 0;

/* ReceiveBuffer: Buffer used to receive data in the ISR
 * RXByteCtr: Number of bytes left to receive
 * ReceiveIndex: The index of the next byte to be received in ReceiveBuffer
 * TransmitBuffer: Buffer used to transmit data in the ISR
 * TXByteCtr: Number of bytes left to transfer
 * TransmitIndex: The index of the next byte to be transmitted in TransmitBuffer
 * */
uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t RXByteCtr = 0;
uint8_t ReceiveIndex = 0;
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t TXByteCtr = 0;
uint8_t TransmitIndex = 0;

int accel_update(int *array, int position);

/* I2C Write and Read Functions */

/* For slave device with dev_addr, writes the data specified in *reg_data
 *
 * dev_addr: The slave device address.
 *           Example: SLAVE_ADDR
 * reg_addr: The register or command to send to the slave.
 *           Example: CMD_TYPE_0_MASTER
 * *reg_data: The buffer to write
 *           Example: MasterType0
 * count: The length of *reg_data
 *           Example: TYPE_0_LENGTH
 *  */
I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count);

/* For slave device with dev_addr, read the data specified in slaves reg_addr.
 * The received data is available in ReceiveBuffer
 *
 * dev_addr: The slave device address.
 *           Example: SLAVE_ADDR
 * reg_addr: The register or command to send to the slave.
 *           Example: CMD_TYPE_0_SLAVE
 * count: The length of data to read
 *           Example: TYPE_0_LENGTH
 *  */
I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t count);
void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count);


I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t count)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;
    RXByteCtr = count;
    TXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize slave address and interrupts */
    UCB1I2CSA = dev_addr;
    UCB1IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
    UCB1IE &= ~UCRXIE;                       // Disable RX interrupt
    UCB1IE |= UCTXIE;                        // Enable TX interrupt

    UCB1CTLW0 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(LPM0_bits + GIE);              // Enter LPM0 w/ interrupts

    return MasterMode;

}


void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    uint8_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex] = source[copyIndex];
    }
}

I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;

    //Copy register data to TransmitBuffer
    CopyArray(reg_data, TransmitBuffer, count);

    TXByteCtr = count;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize slave address and interrupts */
    UCB1I2CSA = dev_addr;
    UCB1IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
    UCB1IE &= ~UCRXIE;                       // Disable RX interrupt
    UCB1IE |= UCTXIE;                        // Enable TX interrupt

    UCB1CTLW0 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(LPM0_bits + GIE);              // Enter LPM0 w/ interrupts

    return MasterMode;
}

int accel_setup() {
    I2C_Master_WriteReg(SLAVE_ADDR, CMD_TYPE_0_MASTER, MasterType01, TYPE_0_LENGTH);
    I2C_Master_WriteReg(SLAVE_ADDR, CMD_TYPE_1_MASTER, MasterType02, TYPE_0_LENGTH);
    I2C_Master_WriteReg(SLAVE_ADDR, CMD_TYPE_2_MASTER, MasterType03, TYPE_0_LENGTH);
    // set power mode and bandwidth
    // set data rate, enable X and Y axis
    // set range and resolution
    return 0; // or number for error
}

void introScreen(void) {

    setColor(COLOR_16_BLUE);
    drawString(10, 10, FONT_LG, "MAZE GAME");
    setColor(COLOR_16_WHITE);
    drawString(0, 30+10, FONT_SM, "Control the ball with the");
    drawString(0, 40+10, FONT_SM, "accelerometer. Pass");
    drawString(0, 50+10, FONT_SM, "through the maze to WIN.");
    setColor(COLOR_16_RED);
    drawString(0, 90+10, FONT_SM, "Press S1 to start.");
    drawString(0, 100+10, FONT_SM, "Press S2 to quit to menu.");
    setColor(COLOR_16_WHITE);

}

void swap(int *p,int *q) {
   int t;

   t=*p;
   *p=*q;
   *q=t;
}

void sort(int a[],int n) {
   int i,j;

   for(i = 0;i < n-1;i++) {
      for(j = 0;j < n-i-1;j++) {
         if(a[j] > a[j+1])
            swap(&a[j],&a[j+1]);
      }
   }
}

void writeData(uint8_t data) {
    P2OUT &= ~LCD_CS_PIN;
    P4OUT |= LCD_DC_PIN;
    UCB0TXBUF = data;
    while(UCB0STATW & UCBUSY);
    P2OUT |= LCD_CS_PIN;
}

void writeCommand(uint8_t command) {
    P2OUT &= ~LCD_CS_PIN;
    P4OUT &= ~LCD_DC_PIN;
    UCB0TXBUF = command;
    while(UCB0STATW & UCBUSY);
    P2OUT |= LCD_CS_PIN;
}

void initMSP430(void) {
    /****************************Switch for Frequency Control************************************/
    P1DIR   &= ~BIT1;
    P1OUT   |= BIT1;          // give pullup/down resistor a '1' so it pulls up
    P1REN   |= BIT1;          // enable pullup/down
    P1DIR   &= ~BIT2;
    P1OUT   |= BIT2;          // give pullup/down resistor a '1' so it pulls up
    P1REN   |= BIT2;          // enable pullup/down

    /*********************** Test Data Generator 1 kHz *********************/
    P9DIR   |= BIT1 | BIT2 | BIT3 | BIT4;   // Test Data Output
    TA2CTL   = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;
    TA2CCR1  = 1000;                                    // 1MHz * 1/1000 Hz
    TA2CCTL1 = CCIE;                                    // enable interrupts
    testcnt  = 0;                                       // start test counter at 0

    /**************************** PWM Backlight ****************************/

    P1DIR   |= BIT3;
    P1SEL0  |= BIT3;
    P1SEL1  &= ~BIT3;
    TA1CCR0  = 511;
    TA1CCTL2 = OUTMOD_7;
    TA1CCR2  = 255;
    TA1CTL   = TASSEL__ACLK | MC__UP | TACLR;

    /******************************** SPI **********************************/

    P2DIR  |=   LCD_CS_PIN;                     // DC and CS
    P4DIR  |=   LCD_DC_PIN;
    P1SEL0 |=   LCD_MOSI_PIN | LCD_UCBCLK_PIN;      // MOSI and UCBOCLK
    P1SEL1 &= ~(LCD_MOSI_PIN | LCD_UCBCLK_PIN);

    UCB0CTLW0 |= UCSWRST;       // Reset UCB0

    /*
     * UCBxCTLW0     - eUSCI_Bx Control Register 0
     * UCSSEL__SMCLK - SMCLK in master mode
     * UCCKPL        - Clock polarity select
     * UCMSB         - MSB first select
     * UCMST         - Master mode select
     * UCMODE_0      - eUSCI mode 3-pin SPI select
     * UCSYNC        -  Synchronous mode enable
     */
    //UCB0CTLW0 |= UCSSEL__SMCLK | UCCKPL | UCMSB | UCMST | UCMODE_0 | UCSYNC;
    UCB0CTLW0 |= UCSSEL__SMCLK | UCCKPH | UCMSB | UCMST | UCMODE_0 | UCSYNC;
    //    UCB0CTLW0 = UCCKPH + UCMSB + UCMST + UCSYNC + UCSSEL_2; // 3-pin, 8-bit SPI master


    UCB0BR0   |= 0x01;         // Clock = SMCLK/1
    UCB0BR1    = 0;
    UCB0CTL1  &= ~UCSWRST;     // Clear UCSWRST to release the eUSCI for operation


}

  void main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // kill the watchdog
    initMSP430();
    accel_init();
    //__delay_cycles(10);
    initLCD();
    clearScreen(1);
    introScreen();
    accel_setup();
    srand(time(0));
    while (TRUE) {
        switch(state){
        case menu:
            break;
        case play:
            setColor(COLOR_16_BLACK);
            drawCircle(posX,posY,5);
            I2C_Master_ReadReg(SLAVE_ADDR, CMD_TYPE_0_SLAVE, TYPE_0_LENGTH);   //get x lsb
            CopyArray(ReceiveBuffer, SlaveType0, TYPE_0_LENGTH);
            array[0] = ReceiveBuffer[0];

            I2C_Master_ReadReg(SLAVE_ADDR, CMD_TYPE_1_SLAVE, TYPE_0_LENGTH); //get x msb
            CopyArray(ReceiveBuffer, SlaveType0, TYPE_0_LENGTH);
            array[1] = ReceiveBuffer[0];

            I2C_Master_ReadReg(SLAVE_ADDR, CMD_TYPE_2_SLAVE, TYPE_0_LENGTH);//get y lsb
            CopyArray(ReceiveBuffer, SlaveType0, TYPE_0_LENGTH);
            array[2] = ReceiveBuffer[0];

            I2C_Master_ReadReg(SLAVE_ADDR, CMD_TYPE_3_SLAVE, TYPE_0_LENGTH);//get y msb
            CopyArray(ReceiveBuffer, SlaveType0, TYPE_0_LENGTH);
            array[3] = ReceiveBuffer[0];

            ByteX = accel_update(array,0);  //join msb lsb x
            ByteY = accel_update(array,1);

            avgArrX[i%5]=ByteX;   //put the value into the median sorter array to smooth data
            avgArrY[i%5]=-ByteY;
            i++;
            setColor(COLOR_16_BLUE);
            int j;
            accX = 0;
            accY = 0;
            for (j = 0; j < 5; j++){
                accX += avgArrX[j];
                accY += avgArrY[j];
            }
            accX = (accX * 0.001196)/5;  //+-4G for avgArrX * (2*4000*9.8)/(65535) to convert to linear acceleration using the sensitivity
            accY = (accY * 0.001196)/5;
            if((posX + accX <= 5) || (posX + accX >= 154)){ //X boundaries on the LCD
                accX*=-1.5;
                setColor(COLOR_16_ORANGE);//change color random
            }
            if ((posY + accY <= 5) || (posY + accY >= 120)){ //Y boundaries on the LCD
                accY*=-1.5;
                setColor(COLOR_16_ORANGE);
            }

            posX+=accX;
            posY+=accY;
            drawCircle(posX,posY,5);
            break;
        case stop:
            clearScreen(1);
            posX = 80;
            posY = 64;
            introScreen();
            state=menu;
            break;
        _nop();
        }
    }
}

#pragma vector = USCI_B1_VECTOR
__interrupt void USCI_B1_I2C_ISR(void){
    switch(__even_in_range(UCB1IV, USCI_I2C_UCBIT9IFG)){
        uint8_t rx_val = 0;
        case USCI_I2C_UCRXIFG0:      //receive interrupt
            rx_val = UCB1RXBUF;
            if (RXByteCtr)
            {
              ReceiveBuffer[ReceiveIndex++] = rx_val;
              RXByteCtr--;
            }
            if (RXByteCtr == 1)
            {
              UCB1CTLW0 |= UCTXSTP;
            }
            else if (RXByteCtr == 0)
            {
              UCB1IE &= ~UCRXIE;
              MasterMode = IDLE_MODE;
              __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
            }
            break;
        case USCI_I2C_UCTXIFG0:      //transmit interrupt
            switch (MasterMode)
            {
              case TX_REG_ADDRESS_MODE:
                  UCB1TXBUF = TransmitRegAddr;
                  if (RXByteCtr)
                      MasterMode = SWITCH_TO_RX_MODE;   // Need to start receiving now
                  else
                      MasterMode = TX_DATA_MODE;        // Continue to transmit with the data in Transmit Buffer
                  break;

              case SWITCH_TO_RX_MODE:
                  UCB1IE |= UCRXIE;              // Enable RX interrupt
                  UCB1IE &= ~UCTXIE;             // Disable TX interrupt
                  UCB1CTLW0 &= ~UCTR;            // Switch to receiver
                  MasterMode = RX_DATA_MODE;    // State is to receive data
                  UCB1CTLW0 |= UCTXSTT;          // Send repeated start
                  if (RXByteCtr == 1)
                  {
                      //Must send stop since this is the N-1 byte
                      while((UCB1CTLW0 & UCTXSTT));
                      UCB1CTLW0 |= UCTXSTP;      // Send stop condition
                  }
                  break;

              case TX_DATA_MODE:
                  if (TXByteCtr)
                  {
                      UCB1TXBUF = TransmitBuffer[TransmitIndex++];
                      TXByteCtr--;
                  }
                  else
                  {
                      //Done with transmission
                      UCB1CTLW0 |= UCTXSTP;     // Send stop condition
                      MasterMode = IDLE_MODE;
                      UCB1IE &= ~UCTXIE;                       // disable TX interrupt
                      __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
                  }
                  break;

              default:
                  __no_operation();
                  break;
            }
            break;
        default:
            break;

    }

}



// Interrupt Service Routine for Timer A2
// Only used for test data generation
#pragma vector=TIMER2_A1_VECTOR
__interrupt void TIMER2_A1_ISR(void)
{
  switch(__even_in_range(TA2IV,2))
  {
    case  0: break;                             // No interrupt
    case  2:                                    // CCR1
        P9OUT   &= ((testcnt << 1) | 0xE1 );    // set 0 bits
        P9OUT   |= ((testcnt << 1) & 0x1E );    // set 1 bits
        testcnt ++;                             // increment counter
        testcnt &= 0x0F;                        // limit to 4 bits
        TA2CCR1 += 1000;                        // back here in 1000 timer cycles
        break;
    default: break;
  }
}


  // Interrupt Service Routine for Port 1
#pragma vector=PORT1_VECTOR       // associate funct. w/ interrupt vector
__interrupt void Port_1(void)     // name of ISR
{
    switch(__even_in_range(P1IV,P1IV_P1IFG7))
        {
        case P1IV_NONE: // Vector  0: no interrupt
            break;
        case P1IV_P1IFG0:   // Vector:
            break;
        case P1IV_P1IFG1:   // Vector  4: P1.1 start
            if(state == menu){
                clearScreen(1);
                state = play;
            }
            break;
        case P1IV_P1IFG2:   // Vector  6: P1.2 stop
            if(state == play){
                state = stop;
            }
            break;
        case P1IV_P1IFG3:   // Vector  8: P1.3
        }

          __low_power_mode_off_on_exit();

}




