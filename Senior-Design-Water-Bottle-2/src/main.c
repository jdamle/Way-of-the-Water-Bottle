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
#include "string.h"
#define _USE_MATH_DEFINES
#include "math.h"
UART_HandleTypeDef huart3;



int sample = 0;
char msg;

//void set_char_msg(int, char);
void nano_wait(unsigned int);
int temp_init(GPIO_TypeDef*, int[]);
float get_temp(GPIO_TypeDef*, int);
float get_turbidity(void);
float get_liquid_vol(float);
float get_battery_percentage(void);
void display(int);

// Quick hacky fix of nanowait
// WARNING can only do micro waits
void nano_wait(unsigned int n) {
    //n = (unsigned int)(n / 6.29167 / 1.458);

    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");

    //micro_wait(n / 1000);
}

// Waits a microsecond using timer15
// WARNING won't work if timer 15 is not initialized
void micro_wait(unsigned int n)
{
    // Writes the value that triggers the capture compare
    // Timer will count in CNT until it matches CCR1
    TIM15 -> ARR = n - 1;
    TIM15 -> CR1 |= TIM_CR1_CEN; // Starts the counting
    while(!(TIM15 -> SR & TIM_SR_UIF)); // Checking for when capture/compare 1 interrupt flag goes high
    TIM15 -> SR &= ~(TIM_SR_UIF);
}

// Set up the
void init_tim15()
{
    RCC -> APB2ENR |= RCC_APB2ENR_TIM15EN;

    // Sets clk frequency to CK_INT(48Mhz) / 48 => 1Mhz so 1 us period
    TIM15 -> PSC = 48 - 1;
    TIM15 -> ARR = ~0x0;

    // Sets one-pulse mode to turn off the counter after the update event
    TIM15 -> CR1 |= TIM_CR1_OPM;

    TIM15 -> DIER |= TIM_DIER_UIE;
}

void init_spi1() {
    // PA5  SPI1_SCK
    // PA6  SPI1_MISO
    // PA7  SPI1_MOSI
    // PA4 SPI1_NSS
    // PA1 DC
    // PA8 SRCS
    // PA9 RST
    // PA10 BUSY
    // PA11 ENA

    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA -> MODER &= ~(GPIO_MODER_MODER4 |
                        GPIO_MODER_MODER5 |
                        GPIO_MODER_MODER6 |


                        GPIO_MODER_MODER7);

    // Alternate Function 10
    GPIOA -> MODER |= (GPIO_MODER_MODER4_1 |
                       GPIO_MODER_MODER5_1 |
                       GPIO_MODER_MODER6_1 |
                       GPIO_MODER_MODER7_1);

    // PA GPIO Output
/*
    GPIOA->MODER |= 0x550004;

    GPIOA->BSRR |= 1<<8 | 1<<9 | 1<<10 | 1<<11;

    //PA PUPDR
    GPIOA->PUPDR &= ~0xff0003;
    GPIOA->PUPDR |= 0x550004;
*/



    RCC -> APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1 -> CR1 &= ~(SPI_CR1_SPE);
    SPI1 -> CR1 |= SPI_CR1_BR; // 111 fpclk/256
    SPI1 -> CR1 |= SPI_CR1_MSTR; // 1 means cfg master

    // DS = 1001 10 bits
    SPI1 -> CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_3;
    SPI1 -> CR2 &= ~(SPI_CR2_DS_1 | SPI_CR2_DS_2);
    SPI1 -> CR2 |= SPI_CR2_SSOE| SPI_CR2_NSSP; // have to write this register all at once to avoid possible issue with data size

    SPI1 -> CR1 |= SPI_CR1_SPE;
}
//SPI for Gyroscope
void init_spi2() {
    //P12 - NSS
    //PB13 - SCK
    //PB14 - MISO
    //PB15 - MOSI
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODER12 | GPIO_MODER_MODER13 | GPIO_MODER_MODER14 | GPIO_MODER_MODER15 | GPIO_MODER_MODER1);
    GPIOB->MODER |= (GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1);
    GPIOB->AFR[1] &= ~(0xffff0000);
    GPIOB->MODER |= (0x4);
    //GPIOB->PUPDR |= 0x4;
    GPIOB->BSRR |= 1<<1;

    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    SPI2->CR1 &= ~(SPI_CR1_SPE);
    SPI2->CR1 |= SPI_CR1_BR;
    SPI2->CR1 |= SPI_CR1_MSTR;

    // DS = 1001 10 bits
    SPI2->CR2 |= SPI_CR2_DS_0 |SPI_CR2_DS_3;
    SPI2->CR2 &= ~(SPI_CR2_DS_1 | SPI_CR2_DS_2);
    SPI2->CR2 |= SPI_CR2_SSOE | SPI_CR2_NSSP;

    SPI2->CR1 |= SPI_CR1_SPE;

}
void spi2_cmd(unsigned int data) {
    nano_wait(10000);
    GPIOB->BRR |= 1<<1;
    while(!(SPI2->SR & SPI_SR_TXE)) {}
    SPI2->DR = data;
    nano_wait(60000);
    GPIOB->BSRR |= 1<<1;
}
int get_gyro_val(void) {
    int gyro_1;
    int gyro_2;
    spi_cmd(0xa2);
    while (!((SPI1->SR) &(1<<0))) {}
    gyro_1 = SPI1->DR;
    spi_cmd(0xa3);
    while (!((SPI1->SR) & (1<<0))) {}
    gyro_2 = SPI1->DR;
    return gyro_1;
}
void spi_cmd(unsigned int data) {
    //wait for the TXE bit to be set, then send data to the SPI DR register

    while(!(SPI1->SR & SPI_SR_TXE)) {}
    SPI1->DR = data;
}
void spi_data(unsigned int data) {
    spi2_cmd(data | 0x200);
}
void spi2_init_oled() {
    nano_wait(1000000);
    spi2_cmd(0x38);
    spi2_cmd(0x08);
    spi2_cmd(0x01);
    nano_wait(2000000);
    spi2_cmd(0x06);
    spi2_cmd(0x02);
    spi2_cmd(0x0c);
}
void spi1_init_eink(void) {
    //nano_wait(1000000);
    GPIOA->BSRR |= 1<<17;
    SPI1->CR2 &= ~SPI_CR2_NSSP;
    spi_cmd(0x04);
    spi_cmd(0x00);
    spi_cmd(0xFF);
    spi_cmd(0x02);
    spi_cmd(0x00);
    //nano_wait(2000000);
    spi_cmd(0x01);
    spi_cmd(0x1F);
    spi_cmd(0x50);
    spi_cmd(0x01);
    spi_cmd(0x97);
    spi_cmd(0xFE);
    SPI1->CR2 |= SPI_CR2_NSSP;
    //GPIOA->BSRR |= 1<<1;



}
//function writes string to top part of OLED display
void spi2_display1(const char *string) {
    spi2_cmd(0x02);
    //GPIOA->BSRR |= 1<<1;
    while(*string != '\0') {
        spi_data(*string);
        string++;
    }

}
//function writes string to bottom part of OLED display
void spi2_display2(const char *string) {
    spi2_cmd(0xc0);
    while(*string != '\0') {
        spi_data(*string);
        string++;
    }
}
//following subroutines are work in progress for E-ink display
/*void enable_ports(void) {
    //PA5: SCK
    //PA7: MOSI
    //PA14: DC
    //PA12: CS
    //PA15: RST
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER7 | GPIO_MODER_MODER14 | GPIO_MODER_MODER12 | GPIO_MODER_MODER15);
    GPIOA->AFR[0] &= ~(0xf0f00000);
    GPIOA->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER12_0 | GPIO_MODER_MODER15_0);

    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR5 | GPIO_PUPDR_PUPDR7 | GPIO_PUPDR_PUPDR14 | GPIO_PUPDR_PUPDR12 | GPIO_PUPDR_PUPDR15);
    GPIOA->PUPDR |= 0x4400;
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_5 | GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_14 | GPIO_OTYPER_OT_12 | GPIO_OTYPER_OT_15);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR5 | GPIO_OSPEEDR_OSPEEDR7 | GPIO_OSPEEDR_OSPEEDR14);








    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~0x3000000;
    GPIOB->PUPDR &= ~0x3000000;
    GPIOB->PUPDR |= 0x1000000;

}*/
void init_spi1_2(void) {

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 &= ~(SPI_CR1_SPE);
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
    SPI1->CR1 |= SPI_CR1_MSTR;
    SPI1->CR1 |= SPI_CR1_SSM;
    SPI1->CR1 |= SPI_CR1_SSI;
    SPI1->CR1 |= SPI_CR1_SPE;

}
inline void e_ink_write(uint8_t data) {
    while(!(SPI1->SR & SPI_SR_TXE)) {};
    *(uint8_t*) & SPI1->DR = data;
}


//end e-ink subroutines
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
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; //turn on clock for timer 6
    //set up prescaler and ARR values so display switches every 5 seconds
    TIM6->PSC = 4800-1;
    TIM6->ARR = 5000-1;
    //set up UIE and CEN bits and interrupt flag
    TIM6->DIER |= TIM_DIER_UIE;
/*    TIM6->CR2 &= ~TIM_CR2_MMS;
    TIM6->CR2 |= TIM_CR2_MMS_1;*/
    TIM6->CR1 |= TIM_CR1_CEN;
    NVIC->ISER[0] = 1<<17;
}
float volt1 = 0.0;
float volt2 = 0.0;
float volt3 = 0.0;
void TIM6_DAC_IRQHandler(void)
{
    // TODO: Remember to acknowledge the interrupt right here.
    TIM6->SR &= ~TIM_SR_UIF; //acknowledge interrupt;
    //if sample value is 0, display temperature
    if(sample == 0) {
        char temp_str[100];
        float temp;
        int temp_pins[4] = {3, 9, 10, 8};
        temp_init(GPIOA, temp_pins);
        temp = get_temp(GPIOA, 3);
        sprintf(temp_str, "%2.2f", temp);
        spi2_display1("Temperature:     ");
        strcat(temp_str, " degrees C");
        spi2_display2(temp_str);
        sample = 1;
    }
    //if sample value is 1, display liquid level
    else if(sample == 1) {
        char vol_str[100];
        float volume;
        volume = get_liquid_vol(0.03);
        sprintf(vol_str, "%2.2f", volt1);
        spi2_display1("Liquid Level:    ");
        strcat(vol_str, " V        ");
        spi2_display2(vol_str);
        sample = 2;
    }
    //if sample value is 2, display turbidity
    else if(sample == 2) {
        char turbid_str[100];
        float turbidity;
        turbidity = get_turbidity();
        sprintf(turbid_str, "%2.2f", volt2);
        spi2_display1("Turbidity:         ");
        strcat(turbid_str, " V     ");
        spi2_display2(turbid_str);
        sample = 3;
        //sample = 2;
    }
    //if sample value is 3, display battery level
    else if(sample == 3){
        char batt_str[100];
        float batt_percentage;
        batt_percentage = get_battery_percentage();
        sprintf(batt_str, "%2.2f", volt3);
        spi2_display1("Battery Level:      ");
        strcat(batt_str, "V    ");
        spi2_display2(batt_str);

        sample = 0;
    }
/*    init_spi1();
    spi2_init_oled();*/
    //spi2_display1(msg);
    //spi2_display2(test);






}
void init_tim2(void) {
    RCC->APB1ENR |= 0x0000001; // turn on clock for timer 2
    //set prescaler and ARR value so timer triggers every 10 seconds
    TIM2->PSC = 48-1;
    TIM2->ARR = 5000-1;
    //finish setting up timer by setting DIER and CR1 bits as well as interrupt flag
    TIM2->DIER |= 1<<0;
    TIM2->CR1 |= 1<<0;
    NVIC->ISER[0] = 1<<15;
}
void init_tim3(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //turn on clock for timer 3
    //set prescaler and ARR value so timer triggers every 10 seconds
    TIM3->PSC = 48 - 1;
    TIM3->ARR = 5000 - 1;
    //finish setting up timer by setting DIER and CR1 bits as well as interrupt flag
    TIM3->DIER |= 1<<0;
    TIM3->CR1 |= 1<<0;
    NVIC->ISER[0] = 1<<16;
}
void init_tim14(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN; //turn on clock for timer 14
    //set prescaler and ARR value
    TIM14->PSC = 48-1;
    TIM14->ARR = 5000-1;
    //finish setting up timer by setting DIER and CR1 bits as well as interrupt flag
    TIM14->DIER |= 1<<0;
    TIM14->CR1 |= 1<<0;
    NVIC->ISER[0] = 1<<19;
}
 //analog value coming from PA3
char line1[21]; //string value of volt1
 //analog value coming from PA2
char line2[21]; //string value of volt2
//analog value coming from battery
char line3[21]; //string value of volt3
//ISR for Tim2
#define HISTSIZE 128
int hist1[HISTSIZE] = { 0 };
int sum1 = 0;
int pos1 = 0;
void TIM2_IRQHandler(void) {
    TIM2->SR &= ~1<<0; //acknowledge interrupt
    //deselect all channels and select channel 3
    ADC1->CHSELR = 0;
    ADC1->CHSELR |= 1 << 2;
    ADC1->CR |= ADC_CR_ADSTART; //start the ADC
    while(!(ADC1->ISR & ADC_ISR_EOC)); //wait for end of conversion
    int reading2 = ADC1->DR;
    sum1 -= hist1[pos1];
    sum1 += hist1[pos1] = reading2;
    pos1 = (pos1 + 1) & (HISTSIZE - 1);
    float val2 = (sum1 >> 7);
    volt1 = val2 * 3.3 / 4095.0; //get value from DR register and convert to analog value
    //display this value on OLED line 1
    //sprintf(line1, "%2.2f", volt1);
    //spi2_display1(line1);
}

//ISR for Tim3
#define HISTSIZE2 128
int hist2[HISTSIZE2] = { 0 };
int sum2 = 0;
int pos2 = 0;
void TIM3_IRQHandler(void) {
    TIM3->SR &= ~1<<0; //acknowledge interrupt
    //deselect all ADC channels and select channel 2
    ADC1->CHSELR = 0;
    ADC1->CHSELR |= 1 << 1;
    ADC1->CR |= ADC_CR_ADSTART; //turn on ADC
    while(!(ADC1->ISR & ADC_ISR_EOC)); //wait for end of conversion
    //int hist2[HISTSIZE] = { 0 };
    //int sum2 = 0;
    //int pos2 = 0;
    int reading = ADC1->DR;
    sum2 -= hist2[pos2];
    sum2 += hist2[pos2] = reading;
    pos2 = (pos2 + 1) & (HISTSIZE2-1);
    float val = (sum2 >> 7);
    volt2 = val * 3.3 / 4095.0; //convert value from DR register to analog value
    //dispaly value on OLED line 2
    //sprintf(line2, "%2.2f", volt2);
    //spi2_display2(line2);

}
//ISR for Tim14
#define HISTSIZE3 128
int hist3[HISTSIZE3] = { 0 };
int sum3 = 0;
int pos3 = 0;
void TIM14_IRQHandler(void) {
    TIM14->SR &= ~1<<0;
    ADC1->CHSELR = 0;
    ADC1->CHSELR |= 1<<0;
    ADC1->CR |= ADC_CR_ADSTART;
    while(!(ADC1->ISR & ADC_ISR_EOC));
    int reading = ADC1->DR;
    sum3 -=hist3[pos3];
    sum3 += hist3[pos3] = reading;
    pos3 = (pos3 + 1) & (HISTSIZE3 - 1);
    float val = (sum3 >> 7);
    volt3 = val * 3.3 / 4095.0;
}
#define Rref 2.4
#define Supply 2.2
float get_liquid_vol(float radius) {
    // resistance values in kohms
    float rsense = (2.4 * volt1)/(2.0-volt1);
    float level_inch = (rsense - 2.2)/(-0.175);
    if(level_inch < 0.0) {
        level_inch = 0.0;
    }
    float level_meter = (level_inch) * 0.0254;
    float vol = 3.14159265 * (radius * radius) * level_meter;
    //return vol * 1000000;
    return level_inch;

}
float get_turbidity(void){
    float turbid = (volt2*6.76 - 4.2)/(-0.0015);
    if (turbid < 0.0) {
        turbid = 0.0;
    }
    return turbid;
}
#define Battery_Up 2.1
#define Battery_Down 1.5
float get_battery_percentage(void) {
    float batt_per = 0.0;
    batt_per = (volt3-Battery_Down)/(Battery_Up-Battery_Down) * 100;
    if(batt_per < 0.0) {
        batt_per = 0.0;
    }
    return batt_per;

}
void init_adc(void) {
/*    init_spi1();
    spi2_init_oled();
    char line1[21];
    char line2[21];
    float volt1;
    float volt2;*/


    //set up PA2 & PA3 for analog input
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~0x3f;
    GPIOA->MODER |= 0x3f;


    //set up clock for ADC1
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->CR2 |= RCC_CR2_HSI14ON; // turns on high-speed 14MHz clock
    while(!(RCC->CR2 & RCC_CR2_HSI14ON)); //waits for clock to be ready
    ADC1 -> SMPR |= 0x7; //Bits 111 sets Sampling time 239.5 ADC clock cycles (ADC clk => 14MHz)

    ADC1->CR |= ADC_CR_ADEN; // enables ADC
    while(!(ADC1->ISR & ADC_ISR_ADRDY)); //waits for ADC to be ready
    while((ADC1->CR & ADC_CR_ADSTART)); // wait for the ADSTART bit to be 0
/*    ADC1->CHSELR = 0;
    ADC1->CHSELR |= 1 << 3;*/
    while(!(ADC1->ISR & ADC_ISR_ADRDY)); // wait for ADC to be ready
/*    while (1) {
        ADC1->CHSELR = 0;
        ADC1->CHSELR |= 1<<3;
        while(!(ADC1->ISR & ADC_ISR_ADRDY));
        ADC1->CR |= ADC_CR_ADSTART;
        while(!(ADC1->ISR & ADC_ISR_EOC));
        volt1 = (float)(ADC1->DR) * 3 / 4095.0;
        sprintf(line1, "%2.2f", volt1);
        spi2_display1(line1);
        ADC1->CHSELR = 0;
        ADC1->CHSELR |= 1 << 2;
        while(!(ADC1->ISR & ADC_ISR_ADRDY));
        ADC1->CR |= ADC_CR_ADSTART;
        while(!(ADC1->ISR & ADC_ISR_EOC));
        volt2 = (float)(ADC1->DR) * 3 / 4095.0;
        sprintf(line2, "%2.2f", volt2);
        spi2_display2(line2);*/
}
//PC0
//PC1
//PC9
//PC10
void enable_gpio_ports(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~0x3c033f;
    GPIOC->MODER |= 0x140000;
    GPIOC->PUPDR &= ~(0x33f);
    GPIOC->PUPDR |= 0x22a;

}
void init_exti(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    /*SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI0 | SYSCFG_EXTICR1_EXTI1;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PC | SYSCFG_EXTICR1_EXTI1_PC;
    EXTI->RTSR |= EXTI_RTSR_TR0 | EXTI_RTSR_TR1;
    EXTI->IMR |= EXTI_IMR_MR0 | EXTI_IMR_MR1;*/
    //SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI2);
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PC;
    EXTI->RTSR |= EXTI_RTSR_TR4;
    EXTI->IMR &= ~0xffffffff;
    EXTI->IMR |= EXTI_IMR_MR4;
    NVIC->ISER[0] = 1 << 7;
}
/*void EXTI0_1_IRQHandler(void) {
    if(GPIOC->IDR & 1 << 0) {
        EXTI->PR |= EXTI_PR_PR0;
        GPIOC->ODR ^= (1<<9);
    }
    if (GPIOC->IDR & 1 << 1) {
        EXTI->PR |= EXTI_PR_PR1;
        GPIOC->ODR ^= (1<<10);
    }
}*/
void EXTI4_15_IRQHandler(void) {

    EXTI->PR |= EXTI_PR_PR4;

   if (sample == 4) {
        sample = 0;
    }
    sample++;
    //display(sample);

/*    if(GPIOC->IDR & 1 << 0) {
        EXTI->PR |= EXTI_PR_PR0;
        GPIOC->ODR ^= (1<<9);
    }
    if (GPIOC->IDR & 1 << 1) {
        EXTI->PR |= EXTI_PR_PR1;
        GPIOC->ODR ^= (1<<10);
    }*/
}


void display(int sample) {
    if(sample == 1) {
                char temp_str[100];
                float temp;
                int temp_pins[4] = {8, 0, 1, 2};
                temp_init(GPIOC, temp_pins);
                temp = get_temp(GPIOC, 8);
                sprintf(temp_str, "%2.2f", temp);
                spi2_display1("Temperature:     ");
                strcat(temp_str, " degrees C");
                spi2_display2(temp_str);
                //sample = 1;
            }
            //if sample value is 1, display liquid level
            else if(sample == 2) {
                char vol_str[100];
                float volume;
                volume = get_liquid_vol(0.03);
                sprintf(vol_str, "%2.2f", volume);
                spi2_display1("Liquid Level: ");
                strcat(vol_str, " mL        ");
                spi2_display2(vol_str);
                //sample = 2;
            }
            //if sample value is 2, display turbidity
            else if(sample == 3) {

                char turbid_str[100];
                float turbidity;
                turbidity = get_turbidity();
                sprintf(turbid_str, "%2.2f", turbidity);
                spi2_display1("Turbidity:         ");
                strcat(turbid_str, " NTU     ");
                spi2_display2(turbid_str);
                //sample = 3;
                //sample = 2;

            }
            //if sample value is 3, display battery level
            else if(sample == 4){
               spi2_display1("Battery Level: ");
               spi2_display2("50%             ");
               //sample = 0;
            }



}
void init_dac(void) {
    //set up PA4 for analog output
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~0x300;
    GPIOA->MODER |= 0x300;

    RCC->APB1ENR |= RCC_APB1ENR_DACEN; //enable clock to DAC
    DAC->CR &= ~DAC_CR_EN1; //disable DAC channel 1
    DAC->CR &= ~DAC_CR_BOFF1; //keep buffer off
    DAC->CR |= DAC_CR_TEN1; //enable trigger
    DAC->CR |= DAC_CR_TSEL1; //software trigger
    DAC->CR |= DAC_CR_EN1; //enable DAC channel 1

    int x = 2000; // set up value to go into DAC holding register (this value should yield about 1.425 V based on reference voltage of 3.0V and trial and error)
    while((DAC->SWTRIGR & DAC_SWTRIGR_SWTRIG1) == DAC_SWTRIGR_SWTRIG1); //wait for DAC to clear the SWTRIG1 bit
    DAC->DHR12R1 = x; //put this value into the holding register
    DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1; //trigger the conversion
}
//setup code for UART (for Bluetooth)
uint8_t RX_BUFFER[2];
uint8_t TX_BUFFER1[10];
uint8_t TX_BUFFER2[10];
void init_usart3(void) {
    //set up GPIO for alternate functions
    //PB11 - Rx
    //PB10 - Tx
    //PB13 - CTS
    //PB14 - RTS
/*    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~0x3cf00000;
    GPIOB->MODER |= 0x28a00000;
    GPIOB->AFR[1] |= 0x4404400;*/


    //set up code for UART3
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 9600;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
      Error_Handler();
    }

/*
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN; //set up clock for UART
    huart3.Instance = USART3;
    USART3->CR1 |= ~USART_CR1_UE; //turn of UE bit
    USART3->CR1 |= ~(1<<28);
    USART3->CR1 &= ~USART_CR1_M; //8 data bits/
    USART3->CR2 &= ~(USART_CR2_STOP_0);
    USART3->CR2 &= ~USART_CR2_STOP_1; //1 stop bit
    USART3->CR1 &= ~USART_CR1_PCE; //no parity
    USART3->CR1 &= ~USART_CR1_OVER8; //oversampling by 16
    USART3->BRR = 0x2580; //set baud rate
    USART3->CR1 |= USART_CR1_TE | USART_CR1_RE; //turn on TE and RE bits
    USART3->CR1 |= USART_CR1_UE; //turn on UE bit
    while(!(USART3->ISR & USART_ISR_TEACK) || !(USART3->ISR & USART_ISR_REACK)) {} //wait for transmit and receive buffers are empty
    NVIC->ISER[0] = 1 << 29;
*/

}
int error = 0;
void Error_Handler(void) {
    error = 1;
}
int on = 0;
uint8_t TX_BUFFER5[2] = {'A','T' };
//goes to this function once transmission is complete
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    //HAL_UART_Transmit_IT(&huart3, TX_BUFFER5, sizeof(TX_BUFFER5));
    //HAL_UART_Receive_IT(&huart3, RX_BUFFER, sizeof(RX_BUFFER));
    on = 1;
}
//goes to this function once reception is complete
uint8_t TX_BUFFER3[6] = {'L', 'E', 'D', ' ', 'O', 'N'};
uint8_t TX_BUFFER4[7] = {'L', 'E', 'D', ' ', 'O', 'F', 'F'};
//uint8_t TX_BUFFER5[10] = {'A','T'};
int ron = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    ron = 1;
    if(RX_BUFFER[0] == 'O') {
        GPIOC->BSRR |= 0x400;
        HAL_UART_Transmit_IT(&huart3, TX_BUFFER3, sizeof(TX_BUFFER3));
    }
    else {
        GPIOC->BSRR |= 0x4000000;
        HAL_UART_Transmit_IT(&huart3, TX_BUFFER4, sizeof(TX_BUFFER4));
    }
    HAL_UART_Receive_IT(&huart3, RX_BUFFER, sizeof(RX_BUFFER));
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

// Temperature Code
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

// pin_nums[0] = temp sense data pin
// pin_nums[1] = temp heat output
// pin_nums[2] = temp cool output
// pin_nums[3] = temp ctrl enable
// Heat => heat_or_cool: 0
// Cool => heat_or_cool: 1
void set_heat_cool(GPIO_TypeDef* GPIO, int pin_nums[4], int heat_or_cool)
{
    GPIO -> BRR  |= (0x1 << pin_nums[1]) |
                    (0x1 << pin_nums[2]) |
                    (0x1 << pin_nums[3]); // Make sure to turn off temperature before switching heating or cooling
    GPIO -> BSRR |= (0x1 << pin_nums[1 + heat_or_cool]);
}

// Need to make all temp transactions atomic
// pin_nums[0] = temp sense data pin
// pin_nums[1] = temp heat output
// pin_nums[2] = temp cool output
// pin_nums[3] = temp ctrl enable
int temp_init(GPIO_TypeDef* GPIO, int pin_nums[4])
{
    int success = 0;

    // Set up GPIO
    int let = 0;
    if (GPIO == GPIOB)
    {
        let++;
    }
    else if (GPIO == GPIOC)
    {
        let+=2;
    }
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN << let;
    for (int i = 0; i < 4; i++)
    {
        GPIO -> MODER  &= ~(0x3 << pin_nums[i]*2);
        GPIO -> MODER  |=  (0x1 << pin_nums[i]*2); // Output mode 01 (push_pull) for all pins
        GPIO -> PUPDR  &= ~(0x3 << pin_nums[i]*2); // No push pull resistors for pins
        if(i > 0) {
            GPIO ->PUPDR |= (0x2 << pin_nums[i]*2);
        }
    }
    GPIO -> OTYPER |=  0x1 << pin_nums[0];   // Set temp sense pin open drain
    GPIO -> BRR |= (0x1 << pin_nums[3]);
    GPIO -> BRR |= (0x1 << pin_nums[1]);
    GPIO -> BRR |= (0x1 << pin_nums[2]);


    success |= temp_talk_start(GPIO, pin_nums[0]);

    // Function commands
    write_byte(0x4E, GPIO, pin_nums[0]); // write to scratch pad
    write_byte(0xB0, GPIO, pin_nums[0]); // write TH (high temp alarm) 0xB0
    write_byte(0x64, GPIO, pin_nums[0]); // write TL (low temp alarm) 0x64
    write_byte(0x1F, GPIO, pin_nums[0]); // cfg to 9 bit resolution (least accurate but fastest)
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

 /*   char debug_out[21];
    sprintf(debug_out, "0x%08X", tmp_bits);
    spi2_display1(debug_out);*/

    final_temp = ((float)(tmp_bits >> 3)) * 0.5;
/*    char debug_out2[21];
    sprintf(debug_out2, "%2.2f", final_temp);
    spi2_display2(debug_out2);*/

    return final_temp;
}
void heat_cool(GPIO_TypeDef* GPIO, int pin_nums[4])
{
    float new_tmp = (float)RX_BUFFER[0];
    int temp_set = 0;
    int temp_pins[4] = {8, 0, 1, 2};
    temp_init(GPIOC, temp_pins);
    float temp_val = get_temp(GPIOC, temp_pins[0]);
    //if we need to heat
    if(new_tmp > (temp_val-3)) {
        set_heat_cool(GPIO, pin_nums, 0); //turn on heat pin
        //keep in loop until close to desired temperature
        while(temp_set == 0) {
            temp_init(GPIOC, temp_pins);
            temp_val = get_temp(GPIOC, temp_pins[0]); //keep polling temperature sensor
            if(new_tmp > (temp_set -3)) {
                continue;
            }
            else {
                temp_set = 1; //set to 1 if close to desired temperature
            }
        }

    }
    //if we need to cool
    else if (new_tmp < (temp_val+3)) {
        set_heat_cool(GPIO, pin_nums, 1); //turn on cool pin
        //keep in loop until close to desired temperature
        while(temp_set == 0) {
            temp_init(GPIOC, temp_pins);
            temp_val = get_temp(GPIOC, temp_pins[0]); //keep polling temperature sensor
            if(new_tmp < (temp_val + 3)) {
                continue;
            }
            else {
                temp_set = 1; //set to 1 if close to desired temperature
            }
        }
    }
    //turn off all temperature pins
    GPIO -> BRR  |= (0x1 << pin_nums[1]) |
                    (0x1 << pin_nums[2]) |
                    (0x1 << pin_nums[3]);




}
//#define DISPLAY_TEST
#if defined(DISPLAY_TEST)
int main(void)
{
    HAL_Init();
    //sample = 0;
    init_exti();
    init_tim6();
    init_tim3();
    init_tim2();
    //init_tim14();
    init_adc();
    //init_tim6();
    init_spi2();
    spi2_init_oled();
    //enable_gpio_ports();
    //int temp_pins[4] = {8, 0, 1, 2};
    //temp_init(GPIOC, temp_pins);
    //init_exti();
    spi2_display1("Good Day! ");
    //init_tim2();
    //init_tim3();
    //init_adc();
    //init_dac();
}
#endif
//#define TEST_DAC
#if defined(TEST_DAC)
int main(void) {
    init_dac();
}
#endif
//#define TEST_ADC
#if defined(TEST_ADC)
int main(void) {
    init_spi2();
    spi2_init_oled();
    init_tim2();
    init_tim3();
    init_adc();
}
#endif
#define TEST_UART
#if defined(TEST_UART)
#include <stdio.h>
/*
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
}*/
/*char wake_string[100] = "wake wake wake up!!!!";
char receieved_string[10];
int length = strlen(wake_string);*/
int main() {
/*    char wake_string[100] = "wake wake wake up!!!!";
    char received_string[10];
    int length = strlen(wake_string);*/

    //init_tim15();

    HAL_Init();
    //enable_gpio_ports();
    //init_exti();
    sample = 0;
    init_adc();

    // ADC Timers
    init_tim3();
    init_tim2();
    init_tim14();

    init_spi2();
    spi2_init_oled();

    int temp_pins[4] = {3, 9, 10, 8};
    temp_init(GPIOA, temp_pins);
    GPIOA -> BRR |= (0x1 << 10);

    spi2_display1("Good Day! ");

    init_tim6();

/*    init_spi2();
    spi2_init_oled();
    //enable_gpio_ports();
    int temp_pins[4] = {3, 9, 10, 8};
    temp_init(GPIOA, temp_pins);
    GPIOA -> BRR |= (0x1 << 11); // Make sure wireless charging is enabled
    GPIOA -> BRR |= (0x1 << 12);
    char temp_str[100];
    float temp;
    temp = get_temp(GPIOA, 3);
    sprintf(temp_str, "%2.2f", temp);
    spi2_display1("Temperature:     ");
    strcat(temp_str, " degrees C");
    spi2_display2(temp_str);
    //init_exti();
    spi2_display1("Me alive");*/

    //int temp_pins[4] = {3, 9, 10, 8};
        //temp_init(GPIOA, temp_pins);

    /*RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
        GPIOA -> MODER  &= ~(0x3 << 2*2);
        GPIOA -> MODER  |=  (0x1 << 2*2); // Output mode 01 (push_pull) for all pins
        GPIOA -> PUPDR  &= ~(0x3 << 2*2); // No push pull resistors for pins

    GPIOA -> BRR |= 0x1 << 2;
    nano_wait(200 * 1000);
    GPIOA -> BSRR |= 0x1 << 2;
    nano_wait(100 * 1000);
    GPIOA -> BRR |= 0x1 << 2;*/

    //init_usart3();
    //enable_gpio_ports();
/*    for(int i = 0; i < 10; i++) {
        TX_BUFFER1[i] = i;
    }
    for(int i = 0; i < 10; i++) {
        TX_BUFFER2[i] = i;
    }*/
    //HAL_UART_Receive_IT(&huart3, RX_BUFFER, sizeof(RX_BUFFER));
    //HAL_UART_Receive_IT(&huart3, RX_BUFFER, sizeof(RX_BUFFER));
    //HAL_UART_Transmit_IT(&huart3, TX_BUFFER5, sizeof(TX_BUFFER5));
    //HAL_UART_Receive_IT(&huart3, RX_BUFFER, sizeof(RX_BUFFER));

/*
    while(1) {
        HAL_Delay(250);
        ron = 2;
        if(RX_BUFFER[0] == 'O') {
            GPIOC->BSRR |= 1<<10;
        }
    }
*/

 /*   while(1) {
        HAL_Delay(250);
    }*/

    //HAL_UART_Receive_IT(&huart3, RX_BUFFER, sizeof(RX_BUFFER));
    //init_spi1();
    //spi2_init_oled();
    //spi2_display1("Hi there");

/*    for(int i = 0; i < length; i++) {
        while(!(USART3->ISR & USART_ISR_TXE)) {}
        USART3->TDR = wake_string[i];
        spi2_display1(USART3->TDR);
        //nano_wait(10000);
        received_string[i] = USART3->RDR;

    }
    nano_wait(10000);*/
    /*for(int j = 0; j < 10; j++) {
        while(!(USART3->ISR & USART_ISR_RXE)) {}

    }


/*    setbuf(stdin,0);
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
    }*/
}
#endif
//#define TEST_TOGGLE
#if defined(TEST_TOGGLE)
int main(void) {
    enable_gpio_ports();
    init_exti();

}
#endif
//#define TEST_TEMP
#if defined(TEST_TEMP)
int main(void) {
    char temp[16];
    float temp_val;
    init_spi1();
    spi2_init_oled();

    int temp_pins[4] = {8, 0, 1, 2};
    temp_init(GPIOC, temp_pins);
    temp_val = get_temp(GPIOC, temp_pins[0]);
    sprintf(temp, "%2.1f", temp_val);
    spi2_display1(temp);
}
#endif
//#define TEST_EINK
#if defined(TEST_EINK)
int main(void) {
    HAL_Init();
    init_spi1();
    spi1_init_eink();
    spi2_display1("HI");

}
#endif
//#define PCB_TEST
#if defined(PCB_TEST)
int main(void) {
    HAL_Init();
    init_spi2();
    spi2_init_oled();
    int temp_pins[4] = {3, 9, 10, 8};
    temp_init(GPIOA, temp_pins);
    GPIOA -> BRR |= (0x1 << 10);
    spi2_display1("HI");

}
#endif
//#define BLUETOOTH
#if defined(BLUETOOTH)
int main(void) {
    HAL_Init();
    enable_gpio_ports();
    init_usart3();
    HAL_UART_Receive_IT(&huart3, RX_BUFFER, sizeof(RX_BUFFER));
    HAL_UART_Transmit_IT(&huart3, TX_BUFFER5, sizeof(TX_BUFFER5));

}
#endif
