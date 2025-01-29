/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "lwrb.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * \brief           Structure of UART associated buffers and indexes
 */
typedef struct {
    lwrb_t tx_rb; /**< TX ring buffer for DMA to read out */
    uint8_t tx_rb_data[384]; /**< TX ring buffer data */
    volatile size_t tx_dma_current_len; /**< Current TX DMA transfer length */
    lwrb_t rx_process_rb; /**< RX data processing ring buffer */
    uint8_t rx_process_rb_data[384]; /**< RX data processing ring buffer data */
    uint8_t rx_dma_buff[64]; /**< RX circular buffer for DMA to write to */
    size_t old_pos; /**< Previous DMA write to index */
} uart_buff_t;

/**
 * \brief           Structure of TX DMA controller, channel, and associated flag clearing functions for a UART
 */
typedef struct {
    DMA_TypeDef* controller; /**< DMA controller (DMA1/DMA2) */
    uint32_t channel; /**< DMA Channel */
    void (*clear_flag_TC)(DMA_TypeDef*); /**< Channel specific function pointer to clear TC flag */
    void (*clear_flag_HT)(DMA_TypeDef*); /**< Channel specific function pointer to clear HT flag */
    void (*clear_flag_GI)(DMA_TypeDef*); /**< Channel specific function pointer to clear GI flag */
    void (*clear_flag_TE)(DMA_TypeDef*); /**< Channel specific function pointer to clear TE flag */
} dma_tx_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/**
 * \brief           Should tx be looped back
 */
#define LOOPBACK 0

/**
 * \brief           Calculate length of statically allocated array
 */
#define ARRAY_LEN(x) (sizeof(x) / sizeof((x)[0]))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uart_buff_t lpuart1_buff, usart1_buff;

dma_tx_t lpuart1_dma, usart1_dma;


/**
 * \brief           Ring buffer instance for TX data
 */
//lwrb_t lpuart1_tx_rb, usart1_tx_rb;

/**
 * \brief           Ring buffer data array for TX DMA
 */
//uint8_t lpuart1_tx_rb_data[384], usart1_tx_rb_data[384];

/**
 * \brief           Length of currently active TX DMA transfer
 */
//volatile size_t lpuart1_tx_dma_current_len, usart1_tx_dma_current_len;

/**
 * \brief           USART RX buffer for DMA to transfer every received byte
 * \note            Contains raw data that are about to be processed by different events
 */
//uint8_t lpuart1_rx_dma_buff[64], usart1_rx_dma_buff[64];

/**
 * \brief           DMA RX buffer previous position counter
 */
//size_t lpuart1_old_pos, usart1_old_pos;

/**
 * \brief           Ring buffer instance for processing buffer
 */
//lwrb_t lpuart1_rx_process_rb, usart1_rx_process_rb;

/**
 * \brief           Ring buffer data array for processing
 */
//uint8_t lpuart1_rx_process_rb_data[384], usart1_rx_process_rb_data[384];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

void lpuart1_init(void);
void uart_rx_check(uart_buff_t* uart_buff);
void uart_process_data(uart_buff_t* uart_buff, const void* data, size_t len);
void uart_send_string(uart_buff_t* uart_buff, dma_tx_t* dma_tx, const char* str);
void uart_send_data(uart_buff_t* uart_buff, dma_tx_t* dma_tx, const void* data, size_t len);
uint8_t uart_start_tx_dma_transfer(uart_buff_t* uart_buff, dma_tx_t* dma_tx);
uint8_t find_crlf_noloop(uart_buff_t* uart_buff, size_t peekahead, uint8_t* old_char);
void process_char_loop(uart_buff_t* uart_buff, size_t peekahead, uint8_t* old_char);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */
    size_t write_len, peekahead;
    uint8_t old_char = 0;

    peekahead = 0;

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    /* System interrupt init*/
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* SysTick_IRQn interrupt configuration */
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));

    /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
     */
    LL_PWR_DisableUCPDDeadBattery();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    /* USER CODE BEGIN 2 */

    /* Initialize lpuart1 tx ringbuff */
    lwrb_init(&lpuart1_buff.tx_rb, lpuart1_buff.tx_rb_data, sizeof(lpuart1_buff.tx_rb_data));

    /* Initialize lpuart1 rx processing ringbuffer */
    lwrb_init(&lpuart1_buff.rx_process_rb, lpuart1_buff.rx_process_rb_data, sizeof(lpuart1_buff.rx_process_rb_data));

    /* Initialize usart1 tx ringbuff */
    lwrb_init(&usart1_buff.tx_rb, usart1_buff.tx_rb_data, sizeof(usart1_buff.tx_rb_data));

    /* Initialize usart1 rx processing ringbuffer */
    lwrb_init(&usart1_buff.rx_process_rb, usart1_buff.rx_process_rb_data, sizeof(usart1_buff.rx_process_rb_data));


    /* Initialize all configured peripherals */
    lpuart1_init();

    /* Notify user to start sending data */
    uart_send_string(&lpuart1_buff, &lpuart1_dma, "USART DMA example: DMA HT & TC + USART IDLE LINE IRQ + RTOS processing\r\n");
    uart_send_string(&lpuart1_buff, &lpuart1_dma, "Start sending data to STM32\r\n");

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        if (!LOOPBACK) {
            if (peekahead < lwrb_get_full(&lpuart1_buff.rx_process_rb)) {
                if(find_crlf_noloop(&lpuart1_buff, peekahead, &old_char)) {
                    write_len = lwrb_get_linear_block_read_length(&lpuart1_buff.rx_process_rb);
                    uart_send_data(&lpuart1_buff/*&usart1_buff */, &lpuart1_dma/* &usart1_dma */, lwrb_get_linear_block_read_address(&lpuart1_buff.rx_process_rb), write_len);
                    lwrb_skip(&lpuart1_buff.rx_process_rb, write_len);
                    if((write_len = lwrb_get_linear_block_read_length(&lpuart1_buff.rx_process_rb)) > 0) {
                        uart_send_data(&lpuart1_buff/*&usart1_buff */, &lpuart1_dma/* &usart1_dma */, lwrb_get_linear_block_read_address(&lpuart1_buff.rx_process_rb), write_len);
                        lwrb_skip(&lpuart1_buff.rx_process_rb, write_len);
                    }
                    peekahead = 0;
                } else {
                    ++peekahead;
                }
            }
        } else {
            if (peekahead < lwrb_get_full(&lpuart1_buff.rx_process_rb)) {
            }
        }
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4) {
    }
    LL_PWR_EnableRange1BoostMode();
    LL_RCC_HSI_Enable();
    /* Wait till HSI is ready */
    while (LL_RCC_HSI_IsReady() != 1) {
    }

    LL_RCC_HSI_SetCalibTrimming(64);
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_4, 85, LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_EnableDomain_SYS();
    LL_RCC_PLL_Enable();
    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1) {
    }

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
    }

    /* Insure 1us transition state at intermediate medium speed clock*/
    for (__IO uint32_t i = (170 >> 1); i != 0; i--);

    /* Set AHB prescaler*/
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    LL_Init1msTick(170000000);

    LL_SetSystemCoreClock(170000000);
}

/* USER CODE BEGIN 4 */

/**
 * \brief           Check for new data received with DMA
 * \param[in]       uart_buff: UART peripheral buffers to use
 *
 * User must select context to call this function from:
 * - Only interrupts (DMA HT, DMA TC, UART IDLE) with same preemption priority level
 * - Only thread context (outside interrupts)
 *
 * If called from both context-es, exclusive access protection must be implemented
 * This mode is not advised as it usually means architecture design problems
 *
 * When IDLE interrupt is not present, application must rely only on thread context,
 * by manually calling function as quickly as possible, to make sure
 * data are read from raw buffer and processed.
 *
 * Not doing reads fast enough may cause DMA to overflow unread received bytes,
 * hence application will lost useful data.
 *
 * Solutions to this are:
 * - Improve architecture design to achieve faster reads
 * - Increase raw buffer size and allow DMA to write more data before this function is called
 */
void uart_rx_check(uart_buff_t* uart_buff) {
    size_t pos;

    /* Calculate current position in buffer and check for new data available */
    pos = ARRAY_LEN(uart_buff->rx_dma_buff) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1);
    if (pos != uart_buff->old_pos) {                       /* Check change in received data */
        if (pos > uart_buff->old_pos) {                    /* Current position is over previous one */
            /*
             * Processing is done in "linear" mode.
             *
             * Application processing is fast with single data block,
             * length is simply calculated by subtracting pointers
             *
             * [   0   ]
             * [   1   ] <- old_pos |------------------------------------|
             * [   2   ]            |                                    |
             * [   3   ]            | Single block (len = pos - old_pos) |
             * [   4   ]            |                                    |
             * [   5   ]            |------------------------------------|
             * [   6   ] <- pos
             * [   7   ]
             * [ N - 1 ]
             */
            uart_process_data(uart_buff, &uart_buff->rx_dma_buff[uart_buff->old_pos], pos - uart_buff->old_pos);
        } else {
            /*
             * Processing is done in "overflow" mode..
             *
             * Application must process data twice,
             * since there are 2 linear memory blocks to handle
             *
             * [   0   ]            |---------------------------------|
             * [   1   ]            | Second block (len = pos)        |
             * [   2   ]            |---------------------------------|
             * [   3   ] <- pos
             * [   4   ] <- old_pos |---------------------------------|
             * [   5   ]            |                                 |
             * [   6   ]            | First block (len = N - old_pos) |
             * [   7   ]            |                                 |
             * [ N - 1 ]            |---------------------------------|
             */
            uart_process_data(uart_buff, &uart_buff->rx_dma_buff[uart_buff->old_pos], ARRAY_LEN(uart_buff->rx_dma_buff) - uart_buff->old_pos);
            if (pos > 0) {
                uart_process_data(uart_buff, &uart_buff->rx_dma_buff[0], pos);
            }
        }
        uart_buff->old_pos = pos;                          /* Save current position as old for next transfers */
    }
}

/**
 * \brief           Check if DMA is active and if not try to send data
 * \return          `1` if transfer just started, `0` if on-going or no data to transmit
 * \param[in]       uart_buff: UART peripheral buffers to use
 * \param[in]       dma_tx: DMA controller, channel, and interrupt flag clearing functions
 */
uint8_t uart_start_tx_dma_transfer(uart_buff_t* uart_buff, dma_tx_t* dma_tx) {
    uint32_t primask;
    uint8_t started = 0;

    /*
     * First check if transfer is currently in-active,
     * by examining the value of tx_dma_current_len variable.
     *
     * This variable is set before DMA transfer is started and cleared in DMA TX complete interrupt.
     *
     * It is not necessary to disable the interrupts before checking the variable:
     *
     * When tx_dma_current_len == 0
     *    - This function is called by either application or TX DMA interrupt
     *    - When called from interrupt, it was just reset before the call,
     *         indicating transfer just completed and ready for more
     *    - When called from an application, transfer was previously already in-active
     *         and immediate call from interrupt cannot happen at this moment
     *
     * When tx_dma_current_len != 0
     *    - This function is called only by an application.
     *    - It will never be called from interrupt with tx_dma_current_len != 0 condition
     *
     * Disabling interrupts before checking for next transfer is advised
     * only if multiple operating system threads can access to this function w/o
     * exclusive access protection (mutex) configured,
     * or if application calls this function from multiple interrupts.
     *
     * This example assumes worst use case scenario,
     * hence interrupts are disabled prior every check
     */
    primask = __get_PRIMASK();
    __disable_irq();
    if (uart_buff->tx_dma_current_len == 0
            && (uart_buff->tx_dma_current_len = lwrb_get_linear_block_read_length(&uart_buff->tx_rb)) > 0) {
        /* Disable channel if enabled */
        LL_DMA_DisableChannel(dma_tx->controller, dma_tx->channel);

        /* Clear all flags */
        dma_tx->clear_flag_TC(dma_tx->controller);
        dma_tx->clear_flag_HT(dma_tx->controller);
        dma_tx->clear_flag_GI(dma_tx->controller);
        dma_tx->clear_flag_TE(dma_tx->controller);

        /* Prepare DMA data and length */
        LL_DMA_SetDataLength(dma_tx->controller, dma_tx->channel, uart_buff->tx_dma_current_len);
        LL_DMA_SetMemoryAddress(dma_tx->controller, dma_tx->channel, (uint32_t)lwrb_get_linear_block_read_address(&uart_buff->tx_rb));

        /* Start transfer */
        LL_DMA_EnableChannel(dma_tx->controller, dma_tx->channel);
        started = 1;
    }
    __set_PRIMASK(primask);
    return started;
}

/**
 * \brief           Process received data over UART
 * \note            Either process them directly or copy to other bigger buffer
 * \param[in]       uart_buff: UART peripheral buffers to use
 * \param[in]       data: Data to process
 * \param[in]       len: Length in units of bytes
 */
void uart_process_data(uart_buff_t* uart_buff, const void* data, size_t len) {
    lwrb_write(&uart_buff->rx_process_rb, data, len);        /* Write data to RX processing buffer for character analysis */
}

/**
 * \brief           Send string to UART
 * \note            Will queue to send if DMA transfer in progress
 * \param[in]       uart_buff: UART peripheral buffers to use
 * \param[in]       str: String to send
 */
void uart_send_string(uart_buff_t* uart_buff, dma_tx_t* dma_tx, const char* str) {
    lwrb_write(&uart_buff->tx_rb, str, strlen(str)); /* Write data to TX buffer */
    uart_start_tx_dma_transfer(uart_buff, dma_tx);              /* Then try to start transfer */
}

/**
 * \brief           Send data to UART
 * \note            Will queue to send if DMA transfer in progress
 * \param[in]       uart_buff: UART peripheral buffers to use
 * \param[in]       data: Data to process
 * \param[in]       len: Length in units of bytes
 */
void uart_send_data(uart_buff_t* uart_buff, dma_tx_t* dma_tx, const void* data, size_t len) {
    lwrb_write(&uart_buff->tx_rb, data, len);
    uart_start_tx_dma_transfer(uart_buff, dma_tx);
}

/**
 * \brief           LPUART1 Initialization Function
 */
void lpuart1_init(void) {
    LL_LPUART_InitTypeDef LPUART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_LPUART1);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    /*
     * LPUART1 GPIO Configuration
     *
     * PA2   ------> LPUART1_TX
     * PA3   ------> LPUART1_RX
     */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* LPUART1 RX DMA init */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_LPUART1_RX);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, LL_LPUART_DMA_GetRegAddr(LPUART1, LL_LPUART_DMA_REG_DATA_RECEIVE));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)lpuart1_buff.rx_dma_buff);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ARRAY_LEN(lpuart1_buff.rx_dma_buff));

    /* LPUART1 TX DMA init */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_LPUART1_TX);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, LL_LPUART_DMA_GetRegAddr(LPUART1, LL_LPUART_DMA_REG_DATA_TRANSMIT));

    /* Enable HT & TC interrupts for RX */
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

    /* Enable HT & TC interrupts for TX */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);

    /* DMA interrupt init for RX & TX */
    NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    NVIC_SetPriority(DMA1_Channel2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    /* Initialize LPUART1 */
    LPUART_InitStruct.PrescalerValue = LL_LPUART_PRESCALER_DIV1;
    LPUART_InitStruct.BaudRate = 115200;
    LPUART_InitStruct.DataWidth = LL_LPUART_DATAWIDTH_8B;
    LPUART_InitStruct.StopBits = LL_LPUART_STOPBITS_1;
    LPUART_InitStruct.Parity = LL_LPUART_PARITY_NONE;
    LPUART_InitStruct.TransferDirection = LL_LPUART_DIRECTION_TX_RX;
    LPUART_InitStruct.HardwareFlowControl = LL_LPUART_HWCONTROL_NONE;
    LL_LPUART_Init(LPUART1, &LPUART_InitStruct);
    LL_LPUART_SetTXFIFOThreshold(LPUART1, LL_LPUART_FIFOTHRESHOLD_1_8);
    LL_LPUART_SetRXFIFOThreshold(LPUART1, LL_LPUART_FIFOTHRESHOLD_1_8);
    LL_LPUART_DisableFIFO(LPUART1);
    LL_LPUART_EnableDMAReq_RX(LPUART1);
    LL_LPUART_EnableDMAReq_TX(LPUART1);
    LL_LPUART_EnableIT_IDLE(LPUART1);

    /* LPUART1 interrupt */
    NVIC_SetPriority(LPUART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(LPUART1_IRQn);

    /* Associated DMA channel struct for LPUART1 */
    lpuart1_dma.controller = DMA1;
    lpuart1_dma.channel = LL_DMA_CHANNEL_2;
    lpuart1_dma.clear_flag_TC = &LL_DMA_ClearFlag_TC2;
    lpuart1_dma.clear_flag_GI = &LL_DMA_ClearFlag_GI2;
    lpuart1_dma.clear_flag_HT = &LL_DMA_ClearFlag_HT2;
    lpuart1_dma.clear_flag_TE = &LL_DMA_ClearFlag_TE2;

    /* Enable LPUART and DMA */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_LPUART_Enable(LPUART1);
    while (!LL_LPUART_IsActiveFlag_TEACK(LPUART1) || !LL_LPUART_IsActiveFlag_REACK(LPUART1)) {}
}


/* Interrupt handlers here */

/**
 * \brief           DMA1 channel1 interrupt handler for LPUART1 RX
 */
void DMA1_Channel1_IRQHandler(void) {
    /* Check half-transfer complete interrupt */
    if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_CHANNEL_1) && LL_DMA_IsActiveFlag_HT1(DMA1)) {
        LL_DMA_ClearFlag_HT1(DMA1);             /* Clear half-transfer complete flag */
        uart_rx_check(&usart1_buff);                       /* Check data */
    }

    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_1) && LL_DMA_IsActiveFlag_TC1(DMA1)) {
        LL_DMA_ClearFlag_TC1(DMA1);             /* Clear transfer complete flag */
        uart_rx_check(&usart1_buff);                       /* Check data */
    }

    /* Implement other events when needed */
}

/**
 * \brief           DMA1 channel2 interrupt handler for LPUART1 TX
 */
void DMA1_Channel2_IRQHandler(void) {
    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_2) && LL_DMA_IsActiveFlag_TC2(DMA1)) {
        LL_DMA_ClearFlag_TC2(DMA1);             /* Clear transfer complete flag */
        lwrb_skip(&lpuart1_buff.tx_rb, lpuart1_buff.tx_dma_current_len);/* Skip buffer, it has been successfully sent out */
        lpuart1_buff.tx_dma_current_len = 0;           /* Reset data length */
        uart_start_tx_dma_transfer(&lpuart1_buff, &lpuart1_dma);          /* Start new transfer */
    }

    /* Implement other events when needed */
}


/**
 * \brief           LPUART1 global interrupt handler
 */
void LPUART1_IRQHandler(void) {
    /* Check for IDLE line interrupt */
    if (LL_LPUART_IsEnabledIT_IDLE(LPUART1) && LL_LPUART_IsActiveFlag_IDLE(LPUART1)) {
        LL_LPUART_ClearFlag_IDLE(LPUART1);      /* Clear IDLE line flag */
        uart_rx_check(&lpuart1_buff);                       /* Check data */
    }

    /* Implement other events when needed */
}


uint8_t find_crlf_noloop(uart_buff_t* uart_buff, size_t peekahead, uint8_t* old_char) {
    uint8_t new_char;

    lwrb_peek(&uart_buff->rx_process_rb, peekahead, &new_char, 1);
    if((*old_char == '\r') & (new_char == '\n')) {
        *old_char = new_char;
        return(1);
    }

    *old_char = new_char;
    return(0);
}

void process_char_loop(uart_buff_t* uart_buff, size_t peekahead, uint8_t* old_char) {

}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
