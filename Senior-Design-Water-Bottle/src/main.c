/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f0xx.h"
//#include "string.h"

const char test[] = "This bottle test";
int sample;
char msg;

//void set_char_msg(int, char);
void nano_wait(unsigned int);

void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}
void init_spi1() {
    // PA5  SPI1_SCK
    // PA6  SPI1_MISO
    // PA7  SPI1_MOSI
    // PA15 SPI1_NSS

    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA -> MODER &= ~(GPIO_MODER_MODER5 |
                        GPIO_MODER_MODER6 |
                        GPIO_MODER_MODER7 |
                        GPIO_MODER_MODER15);

    // Alternate Function 10
    GPIOA -> MODER |= (GPIO_MODER_MODER5_1 |
                       GPIO_MODER_MODER6_1 |
                       GPIO_MODER_MODER7_1 |
                       GPIO_MODER_MODER15_1);

    RCC -> APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1 -> CR1 &= ~(SPI_CR1_SPE);
    SPI1 -> CR1 |= SPI_CR1_BR; // 111 fpclk/256
    SPI1 -> CR1 |= SPI_CR1_MSTR; // 1 means cfg master

    // DS = 1001 10 bits
    SPI1 -> CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_3;
    SPI1 -> CR2 &= ~(SPI_CR2_DS_1 | SPI_CR2_DS_2);
    SPI1 -> CR2 |= SPI_CR2_SSOE | SPI_CR2_NSSP; // have to write this register all at once to avoid possible issue with data size

    SPI1 -> CR1 |= SPI_CR1_SPE;
}
void spi_cmd(unsigned int data) {
    while(!(SPI1->SR & SPI_SR_TXE)) {}
    SPI1->DR = data;
}
void spi_data(unsigned int data) {
    spi_cmd(data | 0x200);
}
void spi1_init_oled() {
    nano_wait(1000000);
    spi_cmd(0x38);
    spi_cmd(0x08);
    spi_cmd(0x01);
    nano_wait(2000000);
    spi_cmd(0x06);
    spi_cmd(0x02);
    spi_cmd(0x0c);
}
void spi1_display1(const char *string) {
    spi_cmd(0x02);
    while(*string != '\0') {
        spi_data(*string);
        string++;
    }
}
void spi1_display2(const char *string) {
    spi_cmd(0xc0);
    while(*string != '\0') {
        spi_data(*string);
        string++;
    }
}
/*uint16_t display[34] = {
        0x002, // Command to set the cursor at the first position line 1
        0x200+'E', 0x200+'C', 0x200+'E', 0x200+'3', 0x200+'6', + 0x200+'2', 0x200+' ', 0x200+'i',
        0x200+'s', 0x200+' ', 0x200+'t', 0x200+'h', + 0x200+'e', 0x200+' ', 0x200+' ', 0x200+' ',
        0x0c0, // Command to set the cursor at the first position line 2
        0x200+'c', 0x200+'l', 0x200+'a', 0x200+'s', 0x200+'s', + 0x200+' ', 0x200+'f', 0x200+'o',
        0x200+'r', 0x200+' ', 0x200+'y', 0x200+'o', + 0x200+'u', 0x200+'!', 0x200+' ', 0x200+' ',
};
void spi1_setup_dma(void) {
    DMA1_Channel3 -> CCR &= ~DMA_CCR_EN; // Turn off the enable bit for the channel

    RCC -> AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel3 -> CPAR = (uint32_t) &(SPI1 -> DR);
    DMA1_Channel3 -> CMAR = (uint32_t) display;
    DMA1_Channel3 -> CNDTR = 34; // msg is 8 elements
    DMA1_Channel3 -> CCR |= DMA_CCR_DIR; // Read from memory
    DMA1_Channel3 -> CCR |= DMA_CCR_MINC; // Memory increment mode enabled
    DMA1_Channel3 -> CCR &= ~(DMA_CCR_PSIZE | DMA_CCR_MSIZE); // Clearing before writing
    DMA1_Channel3 -> CCR |= DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0; // 01 means 16-bits
    DMA1_Channel3 -> CCR |= DMA_CCR_CIRC;
}*/
void init_tim6(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 48000-1;
    TIM6->ARR = 5000-1;
    TIM6->DIER |= TIM_DIER_UIE;
/*    TIM6->CR2 &= ~TIM_CR2_MMS;
    TIM6->CR2 |= TIM_CR2_MMS_1;*/
    TIM6->CR1 |= TIM_CR1_CEN;
    NVIC->ISER[0] = 1<<17;
}
void TIM6_DAC_IRQHandler(void)
{
    // TODO: Remember to acknowledge the interrupt right here.
    TIM6->SR &= ~TIM_SR_UIF;
    if(sample == 0) {
        spi1_display1("Temperature:    ");
        spi1_display2("20 degrees C ");
        sample = 1;
    }
    else if(sample == 1) {
        spi1_display1("Liquid Level: ");
        spi1_display2("50 mL           ");
        sample = 2;
    }
    else{
       spi1_display1("Battery Level: ");
       spi1_display2("50%             ");
        sample = 0;
    }
/*    init_spi1();
    spi1_init_oled();*/
    //spi1_display1(msg);
    //spi1_display2(test);






}
void init_adc(void) {
    init_spi1();
    spi1_init_oled();
    char line[21];
    float volt;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~0xc0;
    GPIOA->MODER |= 0xc0;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->CR2 |= RCC_CR2_HSI14ON;
    while(!(RCC->CR2 & RCC_CR2_HSI14ON));
    ADC1->CR |= ADC_CR_ADEN;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
    while((ADC1->CR & ADC_CR_ADSTART));
    while (1) {
        ADC1->CHSELR = 0;
        ADC1->CHSELR |= 1<<3;
        while(!(ADC1->ISR & ADC_ISR_ADRDY));
        ADC1->CR |= ADC_CR_ADSTART;
        while(!(ADC1->ISR & ADC_ISR_EOC));
        volt = (float)(ADC1->DR) * 3 / 4095.0;
        sprintf(line, "%2.2f", volt);
        spi1_display1(line);






    }

}
void init_dac(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~0x300;
    GPIOA->MODER |= 0x300;

    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    DAC->CR &= ~DAC_CR_EN1;
    DAC->CR &= ~DAC_CR_BOFF1;
    DAC->CR |= DAC_CR_TEN1;
    DAC->CR |= DAC_CR_TSEL1;
    DAC->CR |= DAC_CR_EN1;

    int x = 2000;
    while((DAC->SWTRIGR & DAC_SWTRIGR_SWTRIG1) == DAC_SWTRIGR_SWTRIG1);
    DAC->DHR12R1 = x;
    DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
}

void init_usart5(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;
    GPIOC->MODER &= ~0x3000000;
    GPIOC->MODER |= 0x2000000;
    GPIOC->AFR[1] |= 0x20000;
    GPIOD->MODER &= ~0x30;
    GPIOD->MODER |= 0x20;
    GPIOD->AFR[0] |= 0x200;

    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;
    USART5->CR1 |= ~USART_CR1_UE;
    USART5->CR1 |= ~(1<<28);
    USART5->CR1 &= ~USART_CR1_M;
    USART5->CR2 &= ~(USART_CR2_STOP_0);
    USART5->CR2 &= ~USART_CR2_STOP_1;
    USART5->CR1 &= ~USART_CR1_PCE;
    USART5->CR1 &= ~USART_CR1_OVER8;
    USART5->BRR = 0x000000a1;
    USART5->CR1 |= USART_CR1_TE | USART_CR1_RE;
    USART5->CR1 |= USART_CR1_UE;
    while(!(USART5->ISR & USART_ISR_TEACK) || !(USART5->ISR & USART_ISR_REACK)) {

    }
}
#define REGULAR_TEST
#if defined(REGULAR_TEST)
int main(void)
{
    //sample = 0;
    //init_tim6();
    //init_spi1();
    //spi1_init_oled();
    //spi1_display1("Good Day! ");
    init_adc();
    //init_dac();
/*    init_spi1();
    spi1_init_oled();
    spi1_display1("Hello there,");
    spi1_display2(test);*/


}
#endif

//#define TEST_UART
#if defined(TEST_UART)
#include <stdio.h>
int __io_putchar(int c) {
    if(c == '\n') {
        while(!(USART5->ISR & USART_ISR_TXE)) { }
        USART5->TDR = '\r';

    }
    while(!(USART5->ISR & USART_ISR_TXE)) { }
    USART5->TDR = c;
    return c;
}

int __io_getchar(void) {
     while (!(USART5->ISR & USART_ISR_RXNE)) { }
     char c = USART5->RDR;
     if(c =='\r') {
         c = '\n';
     }
     __io_putchar(c);
     return c;
}

int main() {
    init_usart5();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    printf("Enter your name: ");
    char name[80];
    fgets(name, 80, stdin);
    printf("Your name is %s", name);
    printf("Type any characters.\n");
    for(;;) {
        char c = getchar();
        putchar(c);
    }
}
#endif