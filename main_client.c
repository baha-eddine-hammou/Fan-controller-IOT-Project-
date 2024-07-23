#include <stm32f4xx.h>
#include <stdio.h>
#include <string.h>
#include "ADC_DMA_TIM.h"
#include "DHT11.h"

#define BUFFER_SIZE 500
#define MOTOR_BUFFER_SIZE 5

char buffer[BUFFER_SIZE];
char motor_buffer[MOTOR_BUFFER_SIZE];
char receivedChar;
int buffer_index = 0, set_motor = 0, buffer_complete = 0;
int adc_value[3];
int speed = 500;  // Example speed value, adjust as needed

void reset_buffer(char* buffer, int size) {
    memset(buffer, 0, size);
    buffer_index = 0;
}

// System clock initialization
void HSEInit(void) {
    RCC->CFGR = 0x1;  // Select HSE as system clock
    RCC->CR |= RCC_CR_HSEON;  // HSE on
    while (!(RCC->CR & RCC_CR_HSERDY));  // Wait until HSE ready
}

// USART2 initialization
void USART2_Init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->AFR[0] |= (0x7 << (4 * 2)) | (0x7 << (4 * 3));
    GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;

    USART2->CR1 &= ~USART_CR1_UE;
    USART2->BRR = 8000000 / 115200;
    USART2->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
    USART2->CR1 |= USART_CR1_RXNEIE | USART_CR1_IDLEIE;

    NVIC_EnableIRQ(USART2_IRQn);
}

// USART2 interrupt handler
void USART2_IRQHandler() {
    if (USART2->SR & USART_SR_RXNE) {
        receivedChar = USART2->DR;

        if (receivedChar == '\n' || receivedChar == '\r') {
            receivedChar = '\0';
        }

        buffer[buffer_index++] = receivedChar;

        if (receivedChar == '\0') {
            buffer_complete = 1;
        }
    }

    if (USART2->SR & USART_SR_IDLE) {
        (void)USART2->DR;
        (void)USART2->SR;
        buffer_complete = 1;
        buffer_index = 0;
    }
}

void USART_SendChar(USART_TypeDef* USART, char a) {
    while (!(USART->SR & USART_SR_TXE));
    USART->DR = a;
    while (!(USART->SR & USART_SR_TC));
}

void USART_SendText(USART_TypeDef* USART, const char* txt) {
    while (*txt) {
        USART_SendChar(USART, *txt++);
    }
}

int compare(const char* buffer, const char* expected_response) {
    return strstr(buffer, expected_response) != NULL;
}

void send_at_command(const char* command, const char* expected_response, int max_attempts, int* global_attempts) {
    int attempts = 0;
    while (*global_attempts == 0) {
        while (attempts < max_attempts) {
            USART_SendText(USART2, command);
            while (!buffer_complete);
            buffer_complete = 0;

            if (!compare(buffer, expected_response)) {
                attempts++;
                reset_buffer(buffer, BUFFER_SIZE);
            } else {
                break;
            }
        }

        if (attempts == max_attempts) {
            (*global_attempts)++;
        } else {
            break;
        }
    }
}

void ESPInit() {
    int attempts = 0;
    while (1) {
        send_at_command("AT\r\n", "OK", 3, &attempts);
        send_at_command("AT+RST\r\n", "ready", 3, &attempts);
        send_at_command("AT+CWMODE=1\r\n", "OK", 3, &attempts);
        send_at_command("AT+CWJAP=\"S9Plus\",\"3805112cx\"\r\n", "OK", 3, &attempts);
        send_at_command("AT+CIPMUX=1\r\n", "OK", 3, &attempts);
        send_at_command("AT+CIPSERVER=1,80\r\n", "OK", 3, &attempts);

        if (attempts > 0) {
            send_at_command("AT+RST", "READY", 1, &attempts);
            attempts = 0;
        } else {
            break;
        }
    }
}

void TIM3_Init() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->CR1 = 0;
    TIM3->CR2 = 0;
    TIM3->PSC = 19;
    TIM3->ARR = 999;

    TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
    TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
    TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
    TIM3->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOAEN;
    GPIOB->AFR[0] |= (0x2 << (4 * 0)) | (0x2 << (4 * 1));
    GPIOA->AFR[0] |= (0x2 << (4 * 6)) | (0x2 << (4 * 7));
    GPIOB->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1;
    GPIOA->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;

    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM3->CR1 |= TIM_CR1_CEN;
}

void setMotorDirection(int in1, int in2, int in3, int in4) {
    TIM3->CCR1 = speed * in1;
    TIM3->CCR2 = speed * in2;
    TIM3->CCR3 = speed * in3;
    TIM3->CCR4 = speed * in4;
}

char* extract_data_from_buffer(const char* buffer, char* data_out, int data_out_size) {
    const char* ipd_marker = "+IPD,";
    const char* colon_ptr = NULL;
    const char* data_start = NULL;
    int data_len = 0;
    int i;

    const char* ipd_ptr = strstr(buffer, ipd_marker);
    if (ipd_ptr != NULL) {
        ipd_ptr += strlen(ipd_marker);
        colon_ptr = strchr(ipd_ptr, ':');
        if (colon_ptr != NULL) {
            data_len = atoi(ipd_ptr);
            data_start = colon_ptr + 1;

            for (i = 0; i < data_len && i < data_out_size - 1; i++) {
                data_out[i] = data_start[i];
            }
            data_out[i] = '\0';

            return data_out;
        }
    }

    data_out[0] = '\0';
    return data_out;
}

char extract_client_id(const char* str) {
    const char* ipd_ptr = strstr(str, "+IPD,");
    if (ipd_ptr == NULL) return '5';
    const char* comma_ptr = strchr(ipd_ptr, ',');
    return *(comma_ptr + 1);
}

void close_this_connection() {
    char dynamic_client_id = extract_client_id(buffer);
    char dynamic_cipclose[20];
    sprintf(dynamic_cipclose, "AT+CIPCLOSE=%c\r\n", dynamic_client_id);
    send_at_command(dynamic_cipclose, "OK", 1, 0);
}

void force_close() {
    send_at_command("AT+CIPCLOSE=5\r\n", "OK", 1, 0);
}

void dynamic_send(const char* data) {
    char dynamic_length[50];
    char dynamic_client_id = extract_client_id(buffer);
    sprintf(dynamic_length, "AT+CIPSEND=%c,%d\r\n", dynamic_client_id, strlen(data));
    send_at_command(dynamic_length, "OK\r\n>", 1, 0);
    USART_SendText(USART2, data);
}

void Load_Webpage(const char* data) {
    char dynamic_length[50];
    char dynamic_client_id = extract_client_id(buffer);
    sprintf(dynamic_length, "AT+CIPSEND=%c,%d\r\n", dynamic_client_id, strlen(data));
    send_at_command(dynamic_length, "OK\r\n>", 1, 0);
    USART_SendText(USART2, data);
}

int main() {
    HSEInit();
    USART2_Init();
    I2C_Config();
    ADC_DMA_TIM_Init();
    TIM3_Init();
    ESPInit();

    while (1) {
        if (set_motor) {
            if (strcmp(motor_buffer, "Up") == 0) {
                setMotorDirection(1, 0, 1, 0);
            } else if (strcmp(motor_buffer, "Down") == 0) {
                setMotorDirection(0, 1, 0, 1);
            } else if (strcmp(motor_buffer, "Right") == 0) {
                setMotorDirection(1, 0, 0, 0);
            } else if (strcmp(motor_buffer, "Left") == 0) {
                setMotorDirection(0, 0, 1, 0);
            } else {
                setMotorDirection(0, 0, 0, 0);
            }
            set_motor = 0;
            reset_buffer(motor_buffer, MOTOR_BUFFER_SIZE);
        }

        if (buffer_complete) {
            // Process the complete buffer here
            buffer_complete = 0;
            reset_buffer(buffer, BUFFER_SIZE);
        }
    }
}
