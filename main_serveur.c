#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

// Definitions
#define BUFFER_SIZE 100

// Global Variables
char buffer_rec[BUFFER_SIZE];
char buffer[BUFFER_SIZE];
char buffer_send[BUFFER_SIZE];
int bool0 = 0;
int bool1 = 0;
int on_off = 0;
int i, j;
int k = 0;
float temp;
uint16_t ADC_value[3] = {0, 0, 0};

// Function Prototypes
void hse_clk(void);
void Config_EXTI0(void);
void Config_TIMER2(void);
void Config_ADC1(void);
void Config_DMA2(void);
void Config_USART2(void);
void Config_USART3(void);
void Config_ESP8266_SERVER_STA(void);
void Delay(__IO uint32_t nCount);
void clear_buffer(void);
void clear_buffer_send(void);
void process(void);
void USART2_SendChar(char a);
void Send_String_USART2(char* txt);
void USART3_SendChar(char a);
void Send_String_USART3(char* txt);

// Configurations
void hse_clk(void) {
    RCC->CR |= RCC_CR_HSEON; // Enable HSE
    while (!(RCC->CR & RCC_CR_HSERDY)); // Wait until HSE is ready
    RCC->CFGR = RCC_CFGR_SW_0; // Select HSE as system clock
    while (!(RCC->CFGR & RCC_CFGR_SWS_0)); // Wait for HSE to be used as system clock
}

void Config_EXTI0(void) {
    RCC->APB2ENR |= 1 << 14; // Enable clock for system configuration
    GPIOA->MODER |= 00 << 0; // Enable PA0 as digital input
    SYSCFG->EXTICR[0] = 0x0; // Select PA0 as External Interrupt Source
    EXTI->RTSR |= 0x1; // Rising edge trigger
    EXTI->IMR |= 0x1; // Enable interrupt
    NVIC_EnableIRQ(EXTI0_IRQn); // Enable interrupt in NVIC
}

void Config_TIMER2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 clock
    TIM2->PSC = 7999; // Set prescaler to get Ti=1ms
    TIM2->ARR = 999; // Set auto-reload to get T_cycle=1s
    TIM2->DIER |= TIM_DIER_UIE; // Enable interrupt on update event
    TIM2->DIER |= TIM_DIER_UDE; // Enable Update DMA request
    TIM2->CR2 |= 0x0020; // Update trigger output
}

void Config_ADC1(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Enable ADC1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable GPIOC clock
    GPIOA->MODER |= GPIO_MODER_MODER4; // PA4 (CH4): analog mode
    GPIOC->MODER |= GPIO_MODER_MODER4; // PC4 (CH14): analog mode
    GPIOC->MODER |= GPIO_MODER_MODER5; // PC5 (CH15): analog mode

    ADC1->CR1 |= ADC_CR1_SCAN; // Scan mode (multiple channels)
    ADC1->CR1 |= ADC_CR1_DISCEN; // Discontinuous mode
    ADC1->CR1 |= ADC_CR1_DISCNUM_1; // 3 conversions after start
    ADC1->CR1 |= ADC_CR1_EOCIE; // EOC generates interruption
    ADC1->SQR1 |= ADC_SQR1_L_1; // 3 conversions
    ADC1->SQR3 |= (0x4 << 0);
    ADC1->SQR3 |= (0xE << 5);
    ADC1->SQR3 |= (0xF << 10); // Rank of conversion: ch4, ch14, ch15
    ADC1->SMPR2 = 0x0; // Sample time 3 cycles for all three channels
    ADC1->CR2 |= 1 << 9; // DDS=1
    ADC1->CR2 |= ADC_CR2_ADON; // ADC on
    ADC1->CR2 |= ADC_CR2_EOCS; // EOC flag enable
    ADC1->CR2 |= ADC_CR2_EXTEN_0; // Trigger Detection Rising Edge
    ADC1->CR2 |= ADC_CR2_DMA; // DMA mode
    ADC1->CR2 |= ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2; // Timer 2 TRGO event
}

void Config_DMA2(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; // Enable DMA2 clock
    DMA2_Stream0->CR &= ~DMA_SxCR_EN; // Disable Stream 0
    DMA2_Stream0->CR &= ~(DMA_SxCR_CHSEL); // Channel 0 select
    DMA2_Stream0->CR &= ~(DMA_SxCR_PL); // Low priority
    DMA2_Stream0->CR |= DMA_SxCR_CIRC; // Circular mode enabled
    DMA2_Stream0->CR |= DMA_SxCR_MSIZE_0; // Memory data size 16 bits
    DMA2_Stream0->CR |= DMA_SxCR_PSIZE_0; // Peripheral data size 16 bits
    DMA2_Stream0->CR &= ~(DMA_SxCR_DIR); // Peripheral to Memory
    DMA2_Stream0->CR |= DMA_SxCR_MINC; // Memory increment mode enable
    DMA2_Stream0->CR |= DMA_SxCR_TCIE; // Interrupt enable for transfer complete
    DMA2_Stream0->NDTR = 3; // Number of data to transfer in stream
    DMA2_Stream0->M0AR = (uint32_t)&ADC_value[0]; // Address destination
    DMA2_Stream0->PAR = (uint32_t)&ADC1->DR; // Address source
    DMA2_Stream0->FCR &= ~DMA_SxFCR_DMDIS; // Enable direct mode and disable FIFO mode
    DMA2_Stream0->CR |= DMA_SxCR_EN; // Enable Stream 0
    NVIC_EnableIRQ(DMA2_Stream0_IRQn); // Enable interrupt in NVIC
}

void Config_USART2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Enable USART2 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIOA clock
    GPIOA->MODER |= GPIO_MODER_MODER2_1; // PA2 as alternate function
    GPIOA->MODER |= GPIO_MODER_MODER3_1; // PA3 as alternate function
    GPIOA->AFR[0] |= (0x7 << 8) | (0x7 << 12); // USART2_TX -> PA2, USART2_RX -> PA3
    USART2->BRR = 8000000 / 9600; // 9600 Baud
    USART2->CR1 |= USART_CR1_UE; // Enable USART2
    USART2->CR1 |= USART_CR1_TE; // Enable transmitter
    USART2->CR1 |= USART_CR1_RE; // Enable receiver
    USART2->CR1 |= USART_CR1_RXNEIE; // RXNE interrupt enable
    USART2->CR1 |= USART_CR1_IDLEIE; // Idle interrupt enable
    NVIC_EnableIRQ(USART2_IRQn); // Enable interrupt in NVIC
}

void Config_USART3(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // Enable USART3 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable GPIOC clock
    GPIOC->MODER |= GPIO_MODER_MODER10_1; // PC10 as alternate function
    GPIOC->MODER |= GPIO_MODER_MODER11_1; // PC11 as alternate function
    GPIOC->AFR[1] |= (0x7 << 8) | (0x7 << 12); // USART3_TX -> PC10, USART3_RX -> PC11
    USART3->BRR = 8000000 / 9600; // 9600 Baud
    USART3->CR1 |= USART_CR1_UE; // Enable USART3
    USART3->CR1 |= USART_CR1_TE; // Enable transmitter
    USART3->CR1 |= USART_CR1_RE; // Enable receiver
    USART3->CR1 |= USART_CR1_RXNEIE; // RXNE interrupt enable
    USART3->CR1 |= USART_CR1_IDLEIE; // Idle interrupt enable
    NVIC_EnableIRQ(USART3_IRQn); // Enable interrupt in NVIC
}

void Config_ESP8266_SERVER_STA(void) {
    Delay(10000000);
    Send_String_USART2("AT+RST\r\n");
    Delay(1000000);
    Send_String_USART2("AT\r\n");
    Delay(1000000);
    Send_String_USART2("AT+CWMODE=1\r\n");
    Delay(1000000);
    Send_String_USART2("AT+CWJAP=\"S9Plus\",\"38065112cx\"\r\n");
    Delay(1000000);
    Send_String_USART2("AT+CIPMUX=1\r\n");
    Delay(1000000);
    Send_String_USART2("AT+CIPSERVER=1,80\r\n");
    Delay(1000000);
}

// Main Logic
int main(void) {
    hse_clk();
    Config_EXTI0();
    Config_TIMER2();
    Config_ADC1();
    Config_DMA2();
    Config_USART2();
    Config_USART3();
    Config_ESP8266_SERVER_STA();
    TIM2->CR1 |= TIM_CR1_CEN; // Enable Timer 2

    ADC1->CR2 |= ADC_CR2_SWSTART; // Start ADC conversion

    while (1) {
        if (bool0) {
            bool0 = 0;
            process();
        }
    }
}

// Interrupt Service Routines (ISRs)
void DMA2_Stream0_IRQHandler(void) {
    if (DMA2->LISR & DMA_LISR_TCIF0) {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF0; // Clear transfer complete interrupt flag
        ADC1->SR &= ~ADC_SR_EOC; // Clear ADC EOC flag
        bool0 = 1; // Indicate new data is available
    }
}

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR |= EXTI_PR_PR0; // Clear pending bit
        if (!on_off) {
            ADC1->CR2 |= ADC_CR2_SWSTART; // Start ADC conversion
            on_off = 1;
        }
    }
}

void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_IDLE) {
        buffer_rec[j++] = USART2->DR; // Read data register to clear IDLE flag
        bool1 = 1;
        USART2->SR &= ~USART_SR_IDLE; // Clear idle line flag
    }
    if (USART2->SR & USART_SR_RXNE) {
        buffer_rec[j++] = USART2->DR; // Read data register
        if (j == BUFFER_SIZE) j = 0; // Wrap around buffer index
    }
}

// Buffer Management
void clear_buffer(void) {
    for (i = 0; i < BUFFER_SIZE; i++) {
        buffer[i] = 0;
    }
    i = 0;
}

void clear_buffer_send(void) {
    for (k = 0; k < BUFFER_SIZE; k++) {
        buffer_send[k] = 0;
    }
    k = 0;
}

// Data Processing
void process(void) {
    if (strstr(buffer_rec, "give data")) {
        sprintf(buffer, "temp=%.2f,CH4=%d,CH14=%d,CH15=%d\n", temp, ADC_value[0], ADC_value[1], ADC_value[2]);
        sprintf(buffer_send, "AT+CIPSEND=0,%d\r\n", strlen(buffer));
        Send_String_USART2(buffer_send);
        Send_String_USART2(buffer);
        clear_buffer();
        clear_buffer_send();
    }
}

// USART Communication
void USART2_SendChar(char a) {
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = a;
}

void Send_String_USART2(char* txt) {
    while (*txt) {
        USART2_SendChar(*txt++);
    }
}

void USART3_SendChar(char a) {
    while (!(USART3->SR & USART_SR_TXE));
    USART3->DR = a;
}

void Send_String_USART3(char* txt) {
    while (*txt) {
        USART3_SendChar(*txt++);
    }
}

// Delay Function
void Delay(__IO uint32_t nCount) {
    while (nCount--) {}
}
