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

// Top row
void spi1_display1(const char *string) {
    spi_cmd(0x02);
    while(*string != '\0') {
        spi_data(*string);
        string++;
    }
}

// Bottom row
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
void init_tim2(void) {
    RCC->APB1ENR |= 0x0000001;
    TIM2->PSC = 4800-1;
    TIM2->ARR = 1000-1;
    TIM2->DIER |= 1<<0;
    TIM2->CR1 |= 1<<0;
    NVIC->ISER[0] = 1<<15;
}
void init_tim3(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 4800 - 1;
    TIM3->ARR = 1000 - 1;
    TIM3->DIER |= 1<<0;
    TIM3->CR1 |= 1<<0;
    NVIC->ISER[0] = 1<<16;
}
float volt1;
char line1[21];
float volt2;
char line2[21];

void TIM2_IRQHandler(void) {
    TIM2->SR &= ~1<<0;
    ADC1->CHSELR = 0;
    ADC1->CHSELR |= 1 << 3;
    ADC1->CR |= ADC_CR_ADSTART;
    while(!(ADC1->ISR & ADC_ISR_EOC));
    volt1 = (float)(ADC1->DR) * 3 / 4095.0;
    sprintf(line1, "%2.2f", volt1);
    spi1_display1(line1);
}
void TIM3_IRQHandler(void) {
    TIM3->SR &= ~1<<0;
    ADC1->CHSELR = 0;
    ADC1->CHSELR |= 1 << 2;
    ADC1->CR |= ADC_CR_ADSTART;
    while(!(ADC1->ISR & ADC_ISR_EOC));
    volt2 = (float)(ADC1->DR) * 3 / 4095.0;
    sprintf(line2, "%2.2f", volt2);
    spi1_display2(line2);

}
void init_adc(void) {
/*    init_spi1();
    spi1_init_oled();
    char line1[21];
    char line2[21];
    float volt1;
    float volt2;*/
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~0xf0;
    GPIOA->MODER |= 0xf0;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->CR2 |= RCC_CR2_HSI14ON;
    while(!(RCC->CR2 & RCC_CR2_HSI14ON));
    ADC1->CR |= ADC_CR_ADEN;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
    while((ADC1->CR & ADC_CR_ADSTART));
/*    ADC1->CHSELR = 0;
    ADC1->CHSELR |= 1 << 3;*/
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
/*    while (1) {
        ADC1->CHSELR = 0;
        ADC1->CHSELR |= 1<<3;
        while(!(ADC1->ISR & ADC_ISR_ADRDY));
        ADC1->CR |= ADC_CR_ADSTART;
        while(!(ADC1->ISR & ADC_ISR_EOC));
        volt1 = (float)(ADC1->DR) * 3 / 4095.0;
        sprintf(line1, "%2.2f", volt1);
        spi1_display1(line1);
        ADC1->CHSELR = 0;
        ADC1->CHSELR |= 1 << 2;
        while(!(ADC1->ISR & ADC_ISR_ADRDY));
        ADC1->CR |= ADC_CR_ADSTART;
        while(!(ADC1->ISR & ADC_ISR_EOC));
        volt2 = (float)(ADC1->DR) * 3 / 4095.0;
        sprintf(line2, "%2.2f", volt2);
        spi1_display2(line2);*/
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

// time in us
void time_w0(float time, GPIO_TypeDef* GPIO, int pin_num)
{
    GPIO -> BRR  |= 0x1 << pin_num;
    nano_wait(time * 1000);
    GPIO -> BSRR |= 0x1 << pin_num;
}

void write_byte(int code, GPIO_TypeDef* GPIO, int pin_num)
{
    for (int i = 0; i < 8; i++)
    {
        //nano_wait(1 * 1000); // 1us recovery
        if (!(code & (0x1 << i)))
        {
            time_w0(117, GPIO, pin_num);
            nano_wait(3 * 1000);
        }
        else
        {
            time_w0(1, GPIO, pin_num);
            nano_wait(119 * 1000);
        }
    }
}

int read_bit_tmp(GPIO_TypeDef* GPIO, int pin_num)
{
    time_w0(1, GPIO, pin_num);
    nano_wait(13 * 1000); // Master needs to sample within 15 us of when the read was initiated
    int bit_read = (GPIO -> IDR & (0x1 << pin_num)) >> pin_num;
    nano_wait(45 * 1000); // Read slot minimum of 60 us total
    return bit_read;
}

int read_byte_tmp(GPIO_TypeDef* GPIO, int pin_num)
{
    int final_byte = 0x00;
    for (int i = 0; i < 8; i++)
    {
        final_byte |= read_bit_tmp(GPIO, pin_num) << i;
    }
    return final_byte;
}

// return 0 for success and return 1 for failure
int temp_talk_start(GPIO_TypeDef* GPIO, int pin_num)
{
    // Initialization
    time_w0(480, GPIO, pin_num); // Minimum of hold low 480 us
    nano_wait(60 * 1000); // Maximum of 60 us wait before sends back pull low confirmation
    if (GPIO -> IDR & (0x1 << pin_num))
    {
        return 1;
    }
    nano_wait(240 * 1000); // Holds low for maximum of 240 us

    // Rom commands
    write_byte(0xCC, GPIO, pin_num); // 0xCC skip rom command

    return 0;
}

// Need to make all temp transactions atomic
int temp_init(GPIO_TypeDef* GPIO, int pin_num)
{
    int success = 0;

    // Set up GPIO
    RCC -> AHBENR   |= RCC_AHBENR_GPIOCEN;
    GPIOC -> MODER  &= ~0x30000;
    GPIOC -> MODER  |=  0x10000; // Output mode 01 for pin PC8
    GPIOC -> PUPDR  &=  0x0;
    GPIOC -> OTYPER |=  0x1 << 8;   // Set pin PC8 open drain

    success |= temp_talk_start(GPIO, pin_num);

    // Function commands
    write_byte(0x4E, GPIO, pin_num); // write to scratch pad
    write_byte(0xB0, GPIO, pin_num); // write TH (high temp alarm) 0xB0
    write_byte(0x64, GPIO, pin_num); // write TL (low temp alarm) 0x64
    write_byte(0x1F, GPIO, pin_num); // cfg to 9 bit resolution (least accurate but fastest)
    // 0 [00] 11111 => 00 cfg bits the rest are reserved

    return success;
}

float get_temp(GPIO_TypeDef* GPIO, int pin_num)
{
    float final_temp = 0;

    temp_talk_start(GPIO, pin_num);
    write_byte(0x44, GPIO, pin_num);
    for(int i = 0; i < 2000 & !read_bit_tmp(GPIO, pin_num); i++); // max conversion time about 100 ms

    temp_talk_start(GPIO, pin_num);
    write_byte(0xBE, GPIO, pin_num);
    int tmp_bits = read_byte_tmp(GPIO, pin_num);
    tmp_bits |= read_byte_tmp(GPIO, pin_num) << 8;

    char debug_out[21];
    sprintf(debug_out, "0x%08X", tmp_bits);
    spi1_display1(debug_out);

    final_temp = ((float)tmp_bits) * 0.5;
    char debug_out2[21];
    sprintf(debug_out2, "%2.2f", tmp_bits);
    spi1_display2(debug_out2);

    return final_temp;
}

#define REGULAR_TEST
#if defined(REGULAR_TEST)
int main(void)
{
    //sample = 0;
    //init_tim6();
    init_spi1();
    spi1_init_oled();
    //spi1_display1("Good Day! ");
//    init_tim2();
//    init_tim3();
//    init_adc();
    //init_dac();
/*    init_spi1();
    spi1_init_oled();
    spi1_display1("Hello there,");
    spi1_display2(test);*/
    temp_init(GPIOC, 8);
    get_temp(GPIOC, 8);
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
