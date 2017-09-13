#include "stm32f4xx.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "at86rf233.h"
#include "bmm.h"
#include "mac_control.h"

void Delay(unsigned int Val);
void ChangePllFreq(uint32_t Pll_Value, bool from_hse);
void InterruptTuning();

void calculate_distance();

void SPI_Tuning(SPI_InitTypeDef *SPI_Init);
void SPI_read_reg(uint8_t reg, uint8_t *data);
void SPI_write_reg(uint8_t reg, uint16_t data);
uint8_t* SPI_read_frame_buf();
void SPI_write_frame_buf(uint8_t *data);
void SPI_read_pmu_values();

void USART_send_hex_data(uint8_t *data);
void USART_send_string_data(uint8_t *str);
void USART_send_number(uint16_t number, bool nl);
void Initial_USART_info();

void TRX_change_state(uint8_t new_state);
void TRX_init_batmon_reg();
void TRX_pll_tuning();
void TRX_pll_change();
void TRX_read_addr_fields();
void TRX_write_default_addr_fields();
void TRX_choose_role();
void TRX_print_reg_info();
void TRX_send_frame();
uint8_t* build_frame(uint8_t *len, uint8_t *data);

uint8_t SPI_sram_read_frame_buf(uint8_t addr);
void SPI_sram_write_frame_buf(uint8_t addr, uint8_t data);

// Write dummy value to read from spi
#define SPI_DUMMY_VALUE 				(0x00)

// Write access command of the transceiver
#define WRITE_ACCESS_COMMAND            (0xC0)

// Read access command to the tranceiver
#define READ_ACCESS_COMMAND             (0x80)

// Read access command to the frame buffer of the tranceiver
#define FRAME_BUFFER_READ_ACCESS_COMMAND             (0x20)

// Write access command to the frame buffer of the tranceiver
#define FRAME_BUFFER_WRITE_ACCESS_COMMAND             (0x60)
 
// Read access command to the frame buffer of the tranceiver
#define SRAM_FRAME_BUFFER_READ_ACCESS_COMMAND             (0x00)

// Write access command to the frame buffer of the tranceiver
#define SRAM_FRAME_BUFFER_WRITE_ACCESS_COMMAND             (0x40)


#define PMU_VALUES_DEFAULT_NUMBER (16)

#define TOM_VALUES_DEFAULT_NUMBER (8)

#define delay_tim_freq (1000000)

#define SLP_TR_PULSE GPIOA->BSRRH &= ~GPIO_Pin_3;\
                     GPIOA->BSRRL |= GPIO_Pin_3; __no_operation();\
                     GPIOA->BSRRL &= ~GPIO_Pin_3; GPIOA->BSRRH |= GPIO_Pin_3;
                    
#define RESULT_NUMBER 2
// --------------- GLOBALS ------------------------- //

// TOM active
bool TOM_IS_ACTIVE = false;

// PMU active
bool PMU_IS_ACTIVE = false;

// Both TOM and PMU active
bool PMU_TOM_IS_ACTIVE = false;

// Number of exchanges
short EXNG_NUMBER = 4;

// Number of PMU measurements
volatile short PM_NUMBER = PMU_VALUES_DEFAULT_NUMBER;

__packed enum PEER_ROLE
{
    NO_ROLE = 0x00,
    INITIATOR = 0x01,
    REFLECTOR = 0x02
} peer_role;


__packed enum PEER_ACTION
{
    NO_ACTION = 0x00,
    SEND = 0x01,
    RECEIVE = 0x02,
    READ_BUFFER = 0x03
} peer_action;

__packed enum PEER_STATE
{
    IDLE = 0x00,
    INIT_OF_MEASUREMENT = 0x01,
    WAIT_FOR_FRAME = 0x02,
    WAIT_FOR_ACK = 0x03,
    WAIT_FOR_REQ_FRAME = 0x04,
    TOM_MSRM = 0x05,
    PMU_MSRM = 0x06,
    PMU_TOM_MSRM = 0x07,
    RES_CALCULUS = 0x08,
} peer_state;

__packed typedef enum 
{ 
    PMU_IDLE = 0x00,
    PMU_INIT = 0x01,
    PMU_RECEIVER = 0x02,
    PMU_SENDER = 0x03,
    PMU_READ_BUFFER = 0x04,
    PMU_FREQ_CHANGE = 0x05,
    PMU_FINISH = 0xff
} PMU_STATE;

__packed typedef enum 
{ 
    TOM_IDLE = 0x00,
    TOM_INIT = 0x01,
    TOM_RECEIVER = 0x02,
    TOM_SENDER = 0x03,
    TOM_READ_BUFFER = 0x04,
    TOM_FREQ_CHANGE = 0x05,
    TOM_FINISH = 0xff
} TOM_STATE;

__packed struct
{   
    PMU_STATE pmu_state;
    uint8_t pmu_step;
} pmu_service_struct;

__packed typedef struct
{
  uint8_t crc_trac_status; // status of transaction
  uint8_t lqi;
  uint8_t ed;
  uint8_t tom1;
  uint8_t tom2;
  uint8_t tom3;
  uint8_t fec;
} tom_package_info;

uint8_t tom0 = 0;
uint8_t tom1 = 0;
uint8_t tom2 = 0;
uint8_t fec1 = 0;
uint8_t fec2 = 0;

// arrays for initiator and reflector data of pmu measurements
uint8_t pmu_res[4][RESULT_NUMBER];
uint8_t pmu_res_peer[4][RESULT_NUMBER];

// arrays for initiator and reflector data of tom measurements
tom_package_info tom_res[4][RESULT_NUMBER];
tom_package_info tom_res_peer[4][RESULT_NUMBER];

uint8_t res_exch_num = 0;
uint8_t PMU_VAL_PTR = 0;

__packed struct
{   
    TOM_STATE tom_state;
    uint8_t tom_step;
} tom_service_struct;

uint8_t pmu_measurements[PMU_VALUES_DEFAULT_NUMBER];
uint8_t tom_measurements[TOM_VALUES_DEFAULT_NUMBER];

bool print_result = false;

int main(void)
{
    GPIO_InitTypeDef GPIO_InitDef;
    SPI_InitTypeDef SPI_Init;
    USART_InitTypeDef USART_InitStruct;
    
    RCC->CR |= ((uint32_t)RCC_CR_HSION);
    while(!(RCC->CR & RCC_CR_HSIRDY));
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
    ChangePllFreq(2, false); // 32MHZ
    
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    RCC_PCLK1Config(RCC_HCLK_Div1);
    RCC_PCLK2Config(RCC_HCLK_Div1); // 32
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    // SPI initialization
    SPI_DeInit(SPI1);
    SPI_Tuning(&SPI_Init);
    
    // UART initialization
    USART_DeInit(USART1);
    USART_StructInit(&USART_InitStruct);
    USART_InitStruct.USART_BaudRate = 38400;
    USART_Init(USART1, &USART_InitStruct);
    // Turn tranceiver and receiver + RXIE
    USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    
    // Set as NSS of SPI1
    GPIO_InitDef.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
    GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitDef.GPIO_Speed = GPIO_Speed_50MHz;
    // Initialize pin PA4
    GPIO_Init(GPIOA, &GPIO_InitDef);
    GPIOA->BSRRL |= GPIO_Pin_4;
    
    // Initialize pins PA5, PA6, PA7 as AF of SPI1
    GPIO_InitDef.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
    GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitDef.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitDef);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
    
    // Initialize pins PA9, PA10 as AF of USART1
    GPIO_InitDef.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
    GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitDef.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitDef);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
    
    // Initialize pin PA0 as input 
    GPIO_InitDef.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitDef);
    
    // Initialize pins PA2 as out /RST 
    GPIO_InitDef.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
    GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitDef.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitDef);
    // Set RST hi - default state of this pin
    GPIOA->BSRRL |= GPIO_Pin_2;
    
    // PA3 as SLP_TR out
    GPIO_InitDef.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
    GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitDef.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitDef);
    // Set SLP_TR low - default state of this pin
    GPIOA->BSRRH |= GPIO_Pin_3;
    
    // PB0 as DIG2 in, PB1 as IRQ in
    GPIO_InitDef.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitDef);
    
    // LEDS
    GPIO_InitDef.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
    GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitDef.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitDef);
    
    // GPIOB 15 as in
    GPIO_InitDef.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitDef.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitDef);
    
    // TIM1 initialization    
    TIM_TimeBaseInitTypeDef *Timer;
    
    Timer->TIM_ClockDivision = TIM_CKD_DIV1;
    Timer->TIM_CounterMode = TIM_CounterMode_Down;
    //0x3d09 15625 0x0c35 15625 / 3125 = 5 HZ
    Timer->TIM_Period = (uint16_t) 0x0000;
    //SystemCoreClock / delay_tim_freq - 1;//0x001f; // 32M / 2048 = 15625
    Timer->TIM_Prescaler = (uint16_t) 0x1f;
    //Timer->TIM_RepetitionCounter = 0x0;
    TIM_ARRPreloadConfig(TIM2, DISABLE);
    TIM_TimeBaseInit(TIM2, Timer);
    TIM_InternalClockConfig(TIM2);
    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
    TIM2->CNT = (uint16_t) 0xffff;
    
    if (TIM2->SR & TIM_IT_Update)
    {
        NVIC_ClearPendingIRQ(TIM2_IRQn);
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
        
    Timer->TIM_ClockDivision = TIM_CKD_DIV1;
    Timer->TIM_CounterMode = TIM_CounterMode_Up;
    //0x3d09 15625 0x0c35 15625 / 3125 = 5 HZ
    Timer->TIM_Period = (uint16_t) 0x1000;
    //SystemCoreClock / delay_tim_freq - 1;//0x001f; // 32M / 2048 = 15625
    Timer->TIM_Prescaler = (uint16_t) 0x3d08;
    //Timer->TIM_RepetitionCounter = 0x0;
    TIM_ARRPreloadConfig(TIM3, DISABLE);
    TIM_TimeBaseInit(TIM3, Timer);
    TIM_InternalClockConfig(TIM3);
    TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
    TIM3->CNT = (uint16_t) 0x0000;
    
    if (TIM3->SR & TIM_IT_Update) 
    {
      NVIC_ClearPendingIRQ(TIM3_IRQn);
      TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
    
    // reset to TRX
    GPIOA->BSRRL &= ~GPIO_Pin_2;
    GPIOA->BSRRH |= GPIO_Pin_2;
    Delay(10);
    GPIOA->BSRRH &= ~GPIO_Pin_2;
    GPIOA->BSRRL |= GPIO_Pin_2;
    
    // Inerrut customization
    InterruptTuning();
    
    // return in the 1 bit 
    // TRX_STATUS + IRQ_MASK_MODE = 1 + TX_AUTO_CRC_ON + IRQ_2_EXT_EN = 1
    SPI_write_reg(RG_TRX_CTRL_1, 0x66);
    // XAH_CTRL_1: ARET_TX_TS_EN = 1
    SPI_write_reg(RG_XAH_CTRL_1, 0x80);
    // TOM_MODE = 1 + PMU_ON = 0 + turn_theCLKM_16 pin 
    SPI_write_reg(RG_TRX_CTRL_0, 0x85);
    
    RCC->CR |= ((uint32_t)RCC_CR_HSEBYP);
    while(!(RCC->CR & RCC_CR_HSEBYP));
    RCC->CR |= ((uint32_t)RCC_CR_HSEON);
    while(!(RCC->CR & RCC_CR_HSEON));
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);//RCC_SYSCLKSource_HSE
    ChangePllFreq(2, true);
    
    // data to save states of tranceiver
    Initial_USART_info();
    
    // switch TRX from P_ON state to TRX_OFF state
    TRX_change_state(TRX_OFF);
    
    TRX_pll_tuning();
    TRX_init_batmon_reg();
    TRX_write_default_addr_fields();
    
    // tuning irq mask register, turn on all interrupts, except p_on, pll's, rx_start
    SPI_write_reg(RG_IRQ_MASK, 0x78);
        
    // config role of the rtb-item
    TRX_choose_role();

    for(int i = 0; i < PMU_VALUES_DEFAULT_NUMBER; i++) pmu_measurements[i] = 0;
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < RESULT_NUMBER; j++)
        {
          pmu_res[i][j] = 0;
          pmu_res_peer[i][j] = 0;
        }
    }
    SPI_write_reg(RG_TRX_CTRL_0, 0xA5);
    
    while (1) 
    {             
        if (peer_action == SEND)
        {
            if (peer_state != IDLE)
            {
                TRX_send_frame();
            }
        }
        else if (peer_action == READ_BUFFER)
        {
            
            uint8_t *frm = NULL;
            uint8_t phr = 0;

            frm = SPI_read_frame_buf();

            if (frm != NULL)
            {   
                phr = *frm;
                if ((frm[12] == 'T') && (frm[13] == 'O') && (frm[14] == 'M'))
                {
                    tom_service_struct.tom_state = TOM_SENDER;
                    peer_action = SEND;
                } 
                else if ((frm[12] == 'P') && (frm[13] == 'M') && (frm[14] == 'U'))
                {
                    tom_service_struct.tom_state = PMU_SENDER;
                    peer_action = SEND;
                }                    
            }
        } 
    }
}

void Delay(unsigned int Val) 
{
    for(; Val != 0; Val--) 
    {
        __no_operation();
    }
}

// don't care
void ChangePllFreq(uint32_t Pll_Value, bool from_hse)
{
    // Выбираем HSI как источник системной частоты
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
    
    // Ожидаем, пока HSI выберется как источник системной частоты
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x00) 
    {
        
    }

    // Выключаем PLL
    RCC_PLLCmd(DISABLE);

    // Ожидаем, пока PLL снимет бит готовности
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == SET) 
    {
        
    }

    /*
    * Конфигурируем множитель PLL
    * vco = 8 000 000 / 4 * 96 = 192M;
    * sysclk = vco / 6 = 32M;
    * usb = vco / 4 = 48M;
    */
    if (from_hse)
    {
        RCC_PLLConfig(RCC_PLLSource_HSE, 4, 48, 4 + Pll_Value, 4);
    }
    else
    {
        RCC_PLLConfig(RCC_PLLSource_HSI, 4, 48, 4 + Pll_Value, 4);
    }

    // Включаем PLL
    RCC_PLLCmd(ENABLE);

    // Ожидаем, пока PLL выставит бит готовности
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) 
    {
        
    }

    // Выбираем PLL как источник системной частоты
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    // Ожидаем, пока PLL выберется как источник системной частоты
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
    {
        
    }
    SystemCoreClock = 32000000;
    return;
}

void SPI_Tuning(SPI_InitTypeDef *SPI_Initial)
{
    // Initialize the SPI_Direction member
    SPI_Initial->SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    // initialize the SPI_Mode member
    SPI_Initial->SPI_Mode = SPI_Mode_Master;
    // initialize the SPI_DataSize member
    SPI_Initial->SPI_DataSize = SPI_DataSize_8b;
    // Initialize the SPI_CPOL member
    SPI_Initial->SPI_CPOL = SPI_CPOL_Low;
    // Initialize the SPI_CPHA member
    SPI_Initial->SPI_CPHA = SPI_CPHA_1Edge;
    // Initialize the SPI_NSS member
    SPI_Initial->SPI_NSS = SPI_NSS_Soft;
    // Initialize the SPI_BaudRatePrescaler member
    SPI_Initial->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    // Initialize the SPI_FirstBit member
    SPI_Initial->SPI_FirstBit = SPI_FirstBit_MSB;
    // Initialize the SPI_CRCPolynomial member
    SPI_Initial->SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, SPI_Initial);
    
    SPI1->CR2 &= 0;
    SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);
    
    // interrupt_tuning
    SPI1->CR2 |= SPI_CR2_SSOE;
    SPI1->CR2 |= SPI_CR2_ERRIE;
    SPI_Cmd(SPI1, ENABLE);
}


void InterruptTuning()
{
    // PB15 external interrupt
    SYSCFG -> EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PB;
    EXTI -> IMR |= EXTI_IMR_MR15;
    // EXTI -> EMR |= EXTI_EMR_MR15;
    // EXTI -> RTSR |= EXTI_RTSR_TR15;
    EXTI -> FTSR |= EXTI_FTSR_TR15;
    NVIC_SetPriority(EXTI15_10_IRQn, 3);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    
    // PB0 PB1
    SYSCFG -> EXTICR[0] |=  SYSCFG_EXTICR1_EXTI1_PB | SYSCFG_EXTICR1_EXTI0_PB;
    EXTI -> IMR |= EXTI_IMR_MR1 | EXTI_IMR_MR0;
    EXTI -> RTSR |= EXTI_RTSR_TR1;// | EXTI_RTSR_TR0;
    EXTI -> FTSR |= EXTI_FTSR_TR0;// | EXTI_FTSR_TR1;
    NVIC_SetPriority(EXTI0_IRQn, 2);
    NVIC_SetPriority(EXTI1_IRQn, 4);
    NVIC_DisableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
       
    // NVIC_SPI_IRQ
    NVIC_SetPriority(SPI1_IRQn, 5);
    NVIC_EnableIRQ(SPI1_IRQn);
    // NVIC_USART_IRQ
    NVIC_SetPriority(USART1_IRQn, 6);
    NVIC_EnableIRQ(USART1_IRQn);
    
    // NVIC_TIM2_IRQn
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);
    
    // NVIC_TIM3_IRQn
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    NVIC_SetPriority(TIM3_IRQn, 3);
    NVIC_EnableIRQ(TIM3_IRQn);
    
    __enable_irq();
}

void TIM2_IRQHandler() 
{
    NVIC_DisableIRQ(TIM2_IRQn);
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    
    PM_NUMBER--;
    uint16_t sr = TIM2->SR;
        
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM3_IRQHandler() 
{
    NVIC_DisableIRQ(TIM3_IRQn);
    NVIC_ClearPendingIRQ(TIM3_IRQn);
    TIM_Cmd(TIM3, DISABLE);
    TIM3->CNT = 0x0000;
    
    if (peer_state == WAIT_FOR_ACK)
    {
        USART_send_string_data("TIME_ERROR: NO ACK RECEIVED\n\r");
    }
    else if (peer_state == WAIT_FOR_REQ_FRAME)
    {
        USART_send_string_data("TIME_ERROR: NO REQ_FRAME RECEIVED\n\r");
    }
    else if (peer_state == PMU_MSRM)
    {
        USART_send_string_data("TIME_ERROR: NO PMU_START RECEIVED\n\r");
    }
    else if (peer_state == TOM_MSRM)
    {
        USART_send_string_data("TIME_ERROR: NO TOM_START RECEIVED\n\r");
    }
    else if (peer_state == RES_CALCULUS)
    {
        USART_send_string_data("TIME_ERROR: NO PMU_RES_REQ RECEIVED\n\r");
    }
    
    if (peer_role == INITIATOR)
    {
        TRX_change_state(PLL_ON);// BUSY_TX_ARET
        peer_state = IDLE;
    }
    else if (peer_role == REFLECTOR)
    {
        TRX_change_state(RX_ON);// BUSY_TX_ARET
        peer_state = WAIT_FOR_FRAME;
    }
    
    tom_service_struct.tom_state = TOM_IDLE;
    pmu_service_struct.pmu_state = PMU_IDLE;
    
    
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    NVIC_EnableIRQ(TIM3_IRQn);
}

void EXTI15_10_IRQHandler()
{
  
    NVIC_DisableIRQ(EXTI15_10_IRQn);
    for(int i = 0; i < 10; i++)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14);
        Delay(400000);
        GPIO_ResetBits(GPIOB, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14);
        Delay(400000);
    }
    
    TRX_change_state(PLL_ON);// BUSY_TX_ARET
    
    EXTI->PR |= EXTI_PR_PR15;
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void EXTI0_IRQHandler()
{
    NVIC_DisableIRQ(EXTI0_IRQn);
    
    TIM_Cmd(TIM3, DISABLE);
    TIM3->CNT = 0;
    NVIC_ClearPendingIRQ(EXTI0_IRQn);
    EXTI->PR |= EXTI_PR_PR0;
            
    if (peer_state == TOM_MSRM)
    {
        if (tom_service_struct.tom_state == TOM_RECEIVER)
        {
            tom_service_struct.tom_step += 1;
            if (tom_service_struct.tom_step <= 2)
            {
                tom_service_struct.tom_state = TOM_SENDER;
                peer_action = SEND;
            }
        }
        else
        {
            tom_service_struct.tom_state = TOM_RECEIVER;
            TRX_change_state(RX_ON);
            tom_service_struct.tom_step += 1;
        }
        
        
        if (tom_service_struct.tom_step >= 3)
        {
            tom_service_struct.tom_step = 0;
          
            if (peer_role == INITIATOR)
            {
                TRX_change_state(PLL_ON);
                peer_state = RES_CALCULUS;
            }
            else if (peer_role == REFLECTOR)
            {
                TRX_change_state(RX_ON);
                peer_state = RES_CALCULUS;
            }
            
             // read TOM values from bufer
            USART_send_string_data("IRQ TRX_TOM_RESULTS:\n\r");
            uint8_t tom = SPI_sram_read_frame_buf(0x7d);
            USART_send_hex_data(&tom);
            tom = SPI_sram_read_frame_buf(0x7e);
            USART_send_hex_data(&tom);
            tom = SPI_sram_read_frame_buf(0x7f);
            USART_send_hex_data(&tom);

            tom_service_struct.tom_state = TOM_IDLE;
            
            NVIC_EnableIRQ(EXTI1_IRQn);
        }
        else
        {
            NVIC_EnableIRQ(EXTI0_IRQn);
        }        
    }
    else if (peer_state == PMU_MSRM)
    {
        
        /*
        * Antenna diversity is enabled
        * manual control, set 0 antenna as default
        */
        SPI_write_reg(RG_ANT_DIV, 0x05);
        // 0 antenna, if 1 antenna - 0x07
        uint8_t antenna_select = 0x05;
        // AGC_ON
        SPI_write_reg(RG_TST_AGC, 0x2E);
        
        if (peer_role == INITIATOR)
        {
            uint8_t cc_band;
            SPI_read_reg(RG_CC_CTRL_1, &cc_band);
            cc_band |= 0x08;
            SPI_write_reg(RG_CC_CTRL_1, (uint16_t) cc_band);
            uint8_t cc_number = 0x20;
            
            fec2 = SPI_sram_read_frame_buf(0x7c);
            
            // mutual cycle for all antennas
            for(uint8_t k = 0; k < 4; k++)
            {
                
                // cycle for all freq intervals
                while(1)
                {
                    pmu_res[k][pmu_service_struct.pmu_step] = 0;
                    
                    // write defaultvalue to tim2 CNT, start timer
                    TIM2->CNT = (uint16_t) 0xffff;
                    uint16_t start = TIM2->CNT;
                    TIM2->CR1 |= TIM_CR1_CEN;
                    
                    // select new cc_band
                    SPI_write_reg(RG_CC_CTRL_0, (uint16_t) cc_number);
                    
                    // wait for 40 us syncro event
                    while ((uint16_t)(start - TIM2->CNT) < 40);
                    TIM2->CR1 &= ~TIM_CR1_CEN;
                    
                    //start timer
                    start = TIM2->CNT;
                    TIM2->CR1 |= TIM_CR1_CEN;
                    
                    // for PLL synchronization
                    TRX_change_state(CMD_TRX_OFF);
                    
                    // for reception of the frame
                    TRX_change_state(CMD_RX_ON);
                    
                    // Turn the PMU_ON
                    SPI_write_reg(RG_TRX_CTRL_0, 0xA5);
                    
                    // dop
                    SPI_read_pmu_values();
                    //wait for 40 us syncro event
                    while ((uint16_t)(start - TIM2->CNT) < 696); // dop 216
                    TIM2->CR1 &= ~TIM_CR1_CEN;
                    
                    // start timer
                    start = TIM2->CNT;
                    TIM2->CR1 |= TIM_CR1_CEN;
                    
                    // force to state of transmission
                    TRX_change_state(CMD_TRX_OFF);
                    TRX_change_state(CMD_PLL_ON);
                    
                    // slp_tr hi - begin of transmission
                    SLP_TR_PULSE;
                                        
                    /**
                    * wait for 40 us syncro event 
                    * + 16us for PA stable operation + 160us for transmission shr
                    */
                    while ((uint16_t)(start - TIM2->CNT) < 696); // dop 216
                    TIM2->CR1 &= ~TIM_CR1_CEN;
                    
                    // start
                    start = TIM2->CNT;
                    TIM2->CR1 |= TIM_CR1_CEN;
                    
                    // wait for 40 us syncro event
                    while ((uint16_t)(start - TIM2->CNT) < 40);
                    TIM2->CR1 &= ~TIM_CR1_CEN;
                    
                    // force to state of transmission
                    TRX_change_state(CMD_TRX_OFF);
                    
                    // Turn pmu off
                    SPI_write_reg(RG_TRX_CTRL_0, 0x85);
                    
                    uint16_t buf_value = 0;
                    for (short i = 0; i < PMU_VALUES_DEFAULT_NUMBER; i++)
                    {
                        buf_value += pmu_measurements[i];
                        pmu_measurements[i] = 0;
                    }
                    pmu_res[k][pmu_service_struct.pmu_step] = (uint8_t)(buf_value >> (PMU_VALUES_DEFAULT_NUMBER / 2));
                    
                    cc_number++;
                    pmu_service_struct.pmu_step++;
                    if (pmu_service_struct.pmu_step < RESULT_NUMBER) break;
                }
                
                pmu_service_struct.pmu_step = 0;
                cc_number = 0x20;
            }
            
            TRX_change_state(PLL_ON);
            peer_state = RES_CALCULUS;
            pmu_service_struct.pmu_state = PMU_FINISH;
            pmu_service_struct.pmu_step = 0;
            PMU_IS_ACTIVE = false;
            peer_role = INITIATOR;
            
            SPI_read_reg(RG_CC_CTRL_1, &cc_band);
            cc_band &= ~0x08;
            SPI_write_reg(RG_CC_CTRL_1, (uint16_t) cc_band);
            SPI_write_reg(RG_PHY_CC_CCA, (uint16_t) IEEE_CHANNEL_11);
            
            /*
            * Antenna diversity is disabled
            * set 0 antenna as default
            */
            SPI_write_reg(RG_ANT_DIV, (uint16_t) 0x01);
            
            // Control the sensitivity of the receiver to normal
            USART_send_string_data("FEC:");
            USART_send_number(fec1, true);
            USART_send_number(fec2, true);
            
            for (uint8_t j = 0; j < 4; j++)
            {
                for (int i = 0; i < RESULT_NUMBER; i++)
                {   
                    USART_send_number(j, false);
                    USART_send_string_data(": ");
                    USART_send_number(pmu_res[j][i], true);
                }
            }
        }
        else if (peer_role == REFLECTOR)
        {
            uint8_t cc_band;
            SPI_read_reg(RG_CC_CTRL_1, &cc_band);
            cc_band |= 0x08;
            SPI_write_reg(RG_CC_CTRL_1, (uint16_t) cc_band);
            uint8_t cc_number = 0x20;
            
            // mutual cycle for all antennas
            for(uint8_t k = 0; k < 4; k++)
            {
                
                // cycle for all freq intervals
                while(1)
                {
                    pmu_res[k][pmu_service_struct.pmu_step] = 0;
                    // write defaultvalue to tim2 CNT, start timer
                    TIM2->CNT = (uint16_t) 0xffff;
                    uint16_t start = TIM2->CNT;
                    TIM2->CR1 |= TIM_CR1_CEN;
                    
                    // select new cc_band
                    SPI_write_reg(RG_CC_CTRL_0, (uint16_t) cc_number);
                    
                    // wait for 40 us syncro event
                    while ((uint16_t)(start - TIM2->CNT) < 40);
                    TIM2->CR1 &= ~TIM_CR1_CEN;
                    
                    // start timer
                    start = TIM2->CNT;
                    TIM2->CR1 |= TIM_CR1_CEN;
                    
                    // force to state of transmission
                    TRX_change_state(CMD_TRX_OFF);
                    TRX_change_state(CMD_PLL_ON);
                    
                    // slp_tr hi - begin of transmission
                    SLP_TR_PULSE;
                                        
                    /**
                    * wait for 40 us syncro event 
                    * + 16us for PA stable operation + 160us for transmission shr
                    */
                    while ((uint16_t)(start - TIM2->CNT) < 696); // dop 216
                    TIM2->CR1 &= ~TIM_CR1_CEN;
                    
                    // start timer
                    start = TIM2->CNT;
                    TIM2->CR1 |= TIM_CR1_CEN;
                    
                    // for PLL synchronization
                    // TRX_change_state(CMD_FORCE_PLL_ON);
                    TRX_change_state(CMD_TRX_OFF);
                    
                    // for reception of the frame
                    TRX_change_state(CMD_RX_ON);
                    
                    // Turn the PMU_ON
                    SPI_write_reg(RG_TRX_CTRL_0, 0xA5);
                    
                    // dop
                    SPI_read_pmu_values();
                    
                    // wait for 40 us syncro event
                    while ((uint16_t)(start - TIM2->CNT) < 696); // dop 216
                    TIM2->CR1 &= ~TIM_CR1_CEN;
                    
                    // start timer
                    start = TIM2->CNT;
                    TIM2->CR1 |= TIM_CR1_CEN;
                    
                    while ((uint16_t)(start - TIM2->CNT) < 40);
                    TIM2->CR1 &= ~TIM_CR1_CEN;
                    
                    // force to state of transmission
                    TRX_change_state(CMD_FORCE_TRX_OFF);
                    
                    // Turn pmu off
                    SPI_write_reg(RG_TRX_CTRL_0, 0x85);
                    
                    uint16_t buf_value = 0;
                    for (short i = 0; i < PMU_VALUES_DEFAULT_NUMBER; i++)
                    {
                        buf_value += pmu_measurements[i];
                        pmu_measurements[i] = 0;
                    }
                    pmu_res[k][pmu_service_struct.pmu_step] = (uint8_t)(buf_value >> (PMU_VALUES_DEFAULT_NUMBER / 2));
                    
                    cc_number += 4;
                    pmu_service_struct.pmu_step++;
                    if (pmu_service_struct.pmu_step < RESULT_NUMBER) break;
                }
                
                pmu_service_struct.pmu_step = 0;
                cc_number = 0x20;
                
            }
            
            TRX_change_state(RX_ON);
            peer_state = RES_CALCULUS;
            pmu_service_struct.pmu_state = PMU_FINISH;
            pmu_service_struct.pmu_step = 0;
            PMU_IS_ACTIVE = false;
            peer_role = REFLECTOR;
            
            SPI_read_reg(RG_CC_CTRL_1, &cc_band);
            cc_band &= ~0x08;
            SPI_write_reg(RG_CC_CTRL_1, (uint16_t) cc_band);
            SPI_write_reg(RG_PHY_CC_CCA, (uint16_t) IEEE_CHANNEL_11);
            
            /*
            * Antenna diversity is disabled
            * set 0 antenna as default
            */
            SPI_write_reg(RG_ANT_DIV, 0x01);
            
            // Control the sensitivity of the receiver to normal
            USART_send_string_data("FEC:");
            USART_send_number(fec1, true);
            USART_send_number(fec2, true);
            
            for (uint8_t j = 0; j < 4; j++)
            {
                for (int i = 0; i < RESULT_NUMBER; i++)
                {   
                    USART_send_number(j, false);
                    USART_send_string_data(": ");
                    USART_send_number(pmu_res[j][i], true);
                }
            }
        }
    }        
    else if (peer_state == PMU_TOM_MSRM)
    {
      
    }
}

void EXTI1_IRQHandler()
{
    NVIC_DisableIRQ(EXTI1_IRQn);
    
    NVIC_ClearPendingIRQ(EXTI1_IRQn);
    EXTI->PR |= EXTI_PR_PR1;
    
    // avoid of spiruous events
    NVIC_ClearPendingIRQ(EXTI0_IRQn);
    EXTI->PR |= EXTI_PR_PR0;
    
    uint8_t interrupt;
    SPI_read_reg(RG_IRQ_STATUS, &interrupt);
    
    uint8_t state;
    SPI_read_reg(RG_TRX_STATUS, &state);
    
    if ((interrupt & TRX_IRQ_3_TRX_END) && (peer_state != IDLE))
    {
        if (peer_state == INIT_OF_MEASUREMENT)
        {
            if (tom_service_struct.tom_state == TOM_SENDER)
            {
                TRX_change_state(RX_ON);
                tom_service_struct.tom_state = TOM_RECEIVER;
                peer_state = WAIT_FOR_ACK;
                
                TIM3->CNT = 0;
                TIM_Cmd(TIM3, ENABLE);
            }
            else if (pmu_service_struct.pmu_state == PMU_SENDER)
            {
                TRX_change_state(RX_ON);
                pmu_service_struct.pmu_state = PMU_RECEIVER;
                peer_state = WAIT_FOR_ACK;
                
                TIM3->CNT = 0;
                TIM_Cmd(TIM3, ENABLE);
            }
            
        }
        else if (peer_state == WAIT_FOR_ACK)
        {
            if (tom_service_struct.tom_state == TOM_RECEIVER)
            {
                TIM_Cmd(TIM3, DISABLE);
                TIM3->CNT = 0;
                
                tom_service_struct.tom_state = TOM_READ_BUFFER;
                peer_action = READ_BUFFER;
            }
            else if (tom_service_struct.tom_state == TOM_SENDER)
            {
                TRX_change_state(RX_ON);
                tom_service_struct.tom_state = TOM_RECEIVER;
                peer_state = TOM_MSRM;
                
                NVIC_EnableIRQ(EXTI0_IRQn);
                
                TIM3->CNT = 0;
                TIM_Cmd(TIM3, ENABLE);
            }
            else if (pmu_service_struct.pmu_state == PMU_RECEIVER)
            {
                TIM_Cmd(TIM3, DISABLE);
                TIM3->CNT = 0;
                
                pmu_service_struct.pmu_state = PMU_READ_BUFFER;
                peer_action = READ_BUFFER;
            }
            else if (pmu_service_struct.pmu_state == PMU_SENDER)
            {
                TRX_change_state(RX_ON);
                pmu_service_struct.pmu_state = PMU_RECEIVER;
                peer_state = PMU_MSRM;
                
                NVIC_EnableIRQ(EXTI0_IRQn);
                
                TIM3->CNT = 0;
                TIM_Cmd(TIM3, ENABLE);
            }
            
        }
        else if (peer_state == WAIT_FOR_FRAME)
        {
            if (tom_service_struct.tom_state == TOM_SENDER)
            {
                TRX_change_state(RX_ON);
                tom_service_struct.tom_state = TOM_RECEIVER;
                peer_state = WAIT_FOR_REQ_FRAME;
                
                TIM3->CNT = 0;
                TIM_Cmd(TIM3, ENABLE);
            }
            else if (pmu_service_struct.pmu_state == PMU_SENDER)
            {
                TRX_change_state(RX_ON);
                pmu_service_struct.pmu_state = PMU_RECEIVER;
                peer_state = WAIT_FOR_REQ_FRAME;
                
                TIM3->CNT = 0;
                TIM_Cmd(TIM3, ENABLE);
            }
            else
            {
                peer_action = READ_BUFFER;
            }
            
        }
        else if (peer_state == WAIT_FOR_REQ_FRAME)
        {
            if (tom_service_struct.tom_state == TOM_RECEIVER)
            {
                TIM_Cmd(TIM3, DISABLE);
                TIM3->CNT = 0;
              
                tom_service_struct.tom_state == TOM_READ_BUFFER;
                peer_action = READ_BUFFER;
            }
            else if (tom_service_struct.tom_state == TOM_SENDER)
            {
                
            }
            else if (pmu_service_struct.pmu_state == PMU_RECEIVER)
            {
                TIM_Cmd(TIM3, DISABLE);
                TIM3->CNT = 0;
              
                pmu_service_struct.pmu_state = PMU_READ_BUFFER;
                peer_action = READ_BUFFER;
            }
            else if (pmu_service_struct.pmu_state == PMU_SENDER)
            {
                
            }
        }
        else if (peer_state == RES_CALCULUS)
        {
            if (peer_role == INITIATOR)
            {
                TRX_change_state(PLL_ON);//BUSY_TX_ARET
                peer_state = IDLE;
            }
            else if (peer_role == REFLECTOR)
            {
                TRX_change_state(RX_ON);//BUSY_TX_ARET
                peer_state = WAIT_FOR_FRAME;
            }
        }
    }
    
    
    if (peer_state != TOM_MSRM)
    {
        NVIC_EnableIRQ(EXTI1_IRQn);   
    }
}

void SPI1_IRQHandler()
{
    NVIC_DisableIRQ(SPI1_IRQn);
      
    uint16_t SR = SPI1->SR;
    
    GPIO_SetBits(GPIOB, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14);
    Delay(1600000);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14);
    Delay(1600000);
    
    USART_send_string_data("SPI_ERROR: ");
    if (SR & SPI_I2S_IT_OVR) 
    {
        USART_send_string_data("OVERRUN\n\r");
    }
    
    NVIC_EnableIRQ(SPI1_IRQn);
}

void SPI_read_reg(uint8_t reg, uint8_t* data)
{
    uint8_t buf;
    uint8_t address = READ_ACCESS_COMMAND | reg;
    
    GPIOA->BSRRH |= GPIO_Pin_4;
    while(!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = address;
    while(!(SPI1->SR & SPI_SR_RXNE));
    buf = SPI1->DR;
     
    while(!(SPI1->SR & SPI_SR_TXE));
    // Dummy byte to generate clock
    SPI1->DR = SPI_DUMMY_VALUE;//0x00;
    while(!(SPI1->SR & SPI_SR_RXNE));
    *data = SPI1->DR;
    
    GPIOA->BSRRH &= ~GPIO_Pin_4;
    GPIOA->BSRRL |= GPIO_Pin_4;
}

void SPI_write_reg(uint8_t reg, uint16_t data)
{
    uint8_t buf;
    uint8_t address = WRITE_ACCESS_COMMAND | reg;
    
    GPIOA->BSRRH |= GPIO_Pin_4;
    while(!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = address;
    while(!(SPI1->SR & SPI_SR_RXNE));
    buf = SPI1->DR;
     
    while(!(SPI1->SR & SPI_SR_TXE));
    // Dummy byte to generate clock
    SPI1->DR = data;
    while(!(SPI1->SR & SPI_SR_RXNE));
    buf = SPI1->DR;
    
    GPIOA->BSRRH &= ~GPIO_Pin_4;
    GPIOA->BSRRL |= GPIO_Pin_4;
}

void USART_send_hex_data(uint8_t *data)
{
    while(!(USART1->SR & USART_SR_TXE));
    USART1->DR = '0';
    while(!(USART1->SR & USART_SR_TXE));
    while(!(USART1->SR & USART_SR_TC));
    USART1->DR = 'x';
    while(!(USART1->SR & USART_SR_TXE));
    while(!(USART1->SR & USART_SR_TC));
    
    if ((*data >> 4) > 9)
    {
        USART1->DR = (*data >> 4) - 10 + 'A';
    }
    else
    {
        USART1->DR = (*data >> 4) + '0';
    }
    while(!(USART1->SR & USART_SR_TXE));
    while(!(USART1->SR & USART_SR_TC));
    if ((*data & 0x0F) > 9)
    {
        USART1->DR = (*data & 0x0F) - 10 + 'A';
    }
    else
    {
        USART1->DR = (*data & 0x0F) + '0';
    }
    while(!(USART1->SR & USART_SR_TXE));
    while(!(USART1->SR & USART_SR_TC));
    USART1->DR = 10;
    while(!(USART1->SR & USART_SR_TXE));
    while(!(USART1->SR & USART_SR_TC));
    USART1->DR = 13;
    while(!(USART1->SR & USART_SR_TXE));
    while(!(USART1->SR & USART_SR_TC));
}

void USART_send_string_data(uint8_t *str)
{
    for(int i = 0; i < strlen(str); i++)
    {
        while(!(USART1->SR & USART_SR_TXE));
        USART1->DR = str[i];
        while(!(USART1->SR & USART_SR_TC));
    }
}

void USART1_IRQHandler()
{
    char symbol;
    NVIC_DisableIRQ(USART1_IRQn);
    if (USART1->SR & USART_SR_RXNE)
    {
        USART1->SR &= ~USART_SR_RXNE;
        symbol = USART1->DR;
    }

    USART_send_string_data("Command: ");
    USART_send_string_data(&symbol);
    USART_send_string_data("\n\r");
    
    if (symbol == 'P')
    {
        TRX_pll_change();
    }
    
    uint8_t *ptr = NULL;
    uint8_t len = 0;
    
    if (symbol == 'F')
    {
        ptr = build_frame(&len, "TOM");
    
        if (ptr != NULL)
        {   
            for(int k = 0; k < len; k++)
            {
                USART_send_hex_data(ptr);
                ptr++;
            }
        }
    }
    
    if (symbol == 'W') 
    {
        SPI_write_frame_buf("DATA");
    }
    
    if (symbol == 'S') 
    {
        if (peer_role == INITIATOR)
        {
            SPI_write_frame_buf("DATA");
            TRX_change_state(CMD_TX_START);
        }
        else
        {
            USART_send_string_data("WARNING: you aren't an INITIATOR\n\r");
        }
    }
    
    if (symbol == 'h')
    {
        TRX_print_reg_info();
    }
    
    uint8_t *frm = NULL;
    uint8_t phr = 0;
    
    if (symbol == 'R') 
    {
        TRX_change_state(RX_ON);
    }
    
    // TOM part of programm
    if (symbol == 't')
    {
        if (peer_role == INITIATOR)
        {
            peer_action = SEND;
            peer_state = INIT_OF_MEASUREMENT;
            tom_service_struct.tom_state = TOM_SENDER;
        }
        else
        {
            USART_send_string_data("WARNING: you aren't an INITIATOR\n\r");
        }
    }
    
    // TOM part of programm
    if (symbol == 'p')
    {
        if (peer_role == INITIATOR)
        {
            SPI_write_frame_buf("PMU");
            TRX_change_state(PLL_ON);
            TRX_change_state(CMD_TX_START);
        }
        else
        {
            USART_send_string_data("WARNING: you aren't an INITIATOR\n\r");
        }
    }
    
    NVIC_EnableIRQ(USART1_IRQn);
}

void Initial_USART_info()
{
    USART_send_string_data("HELLO\n\rThis is measurement diploma project\n\r");
    USART_send_string_data("\n\rSend 'h' for help menu\n\r");
    USART_send_string_data("Send 'p' for phase measurement\n\r");
    USART_send_string_data("Send 't' for TOM measurement\n\r");
    USART_send_string_data("Send 'x' for both phase & TOM measurement\n\r");
}

void TRX_pll_tuning()
{
    uint8_t data;
    SPI_read_reg(RG_PHY_CC_CCA, &data);
    USART_send_string_data("RG_PHY_CC_CCA: ");
    USART_send_hex_data(&data);
    SPI_read_reg(RG_CC_CTRL_1, &data);
    USART_send_string_data("RG_CC_CTRL_1: ");
    USART_send_hex_data(&data);
    SPI_read_reg(RG_CC_CTRL_0, &data);
    USART_send_string_data("RG_CC_CTRL_0: ");
    USART_send_hex_data(&data); 
}

void TRX_pll_change()
{
    uint8_t cc_band;
    SPI_read_reg(RG_CC_CTRL_1, &cc_band);
    cc_band |= 0x08;
    SPI_write_reg(RG_CC_CTRL_1, (uint16_t) cc_band);
    SPI_read_reg(RG_CC_CTRL_1, &cc_band);
    uint8_t cc_number = 0x20;
    SPI_read_reg(RG_CC_CTRL_0, &cc_number);
    if ((cc_number < 0xff) && !(cc_number == 0)) 
    {
        cc_number += 1;
    }
    else
    {
        cc_number &= ~0xff;
        cc_number |= 0x20;
    }
    // select new cc_band
    SPI_write_reg(RG_CC_CTRL_0, (uint16_t) cc_number);
}

void TRX_read_addr_fields()
{
    uint8_t data;
    SPI_read_reg(RG_SHORT_ADDR_0, &data);
    USART_send_string_data("\n\rML RG_SHORT_ADDR_0: ");
    USART_send_hex_data(&data);
    
    SPI_read_reg(RG_SHORT_ADDR_1, &data);
    USART_send_string_data("ML RG_SHORT_ADDR_1: ");
    USART_send_hex_data(&data);
    
    SPI_read_reg(RG_PAN_ID_0, &data);
    USART_send_string_data("ML RG_PAN_ID_0: ");
    USART_send_hex_data(&data);
    
    SPI_read_reg(RG_PAN_ID_1, &data);
    USART_send_string_data("ML RG_PAN_ID_1: ");
    USART_send_hex_data(&data);
}

void TRX_write_default_addr_fields()
{
    uint8_t data = (uint8_t) SHORT_ADDR;
    SPI_write_reg(RG_SHORT_ADDR_0, (uint16_t) data);
    data = (uint8_t) (SHORT_ADDR >> 8);
    SPI_write_reg(RG_SHORT_ADDR_1, (uint16_t) data);
    
    data = (uint8_t) PAN_ID;
    SPI_write_reg(RG_PAN_ID_0, (uint16_t) data);
    data = (uint8_t) (PAN_ID >> 8);
    SPI_write_reg(RG_PAN_ID_1, (uint16_t) data);
}

uint8_t* build_frame(uint8_t *len, uint8_t* data)
{
    uint8_t data_length = 0;
    
    if ((strlen(data) > 0 ) && (strlen(data) <= 114)) 
    {
        data_length = strlen(data);
    }
    
    // 13 - length of mac_header, 1 - length of phr
    uint8_t *ptr = (uint8_t *) calloc(13 + data_length + 1, sizeof(uint8_t));
    
    uint8_t *buf = ptr;
    
    // Length of the frame. Max value - aMaxPHYPacketSize = 127
    *buf++ = *len = 13 + data_length;
    
     /**
     * FCF - filling 1 byte
     * 001 - Frame Type - data frame
     * 0 - security bit - no security is applied
     * 0 - Frame Pending - no pendind, we are not going to transmit more 
     * 0 - Acknowledgment Request - we are no waiting for an acknowledgment
     * 0 - PAN ID Compression - we don't compress addresses,
     * even though destination and source addresses are present
     */
     *buf++ = 0x20;//0b0010000x;
     
     /**
     * FCF - filling 2 byte
     * 10 - Destination Addressing Mode - Address field contains a 16-bit short address
     * 01 - Frame Version - Frames are compatible with IEEE 802.15.4-2006
     * 10 - Source Addressing Mode - Address field contains a 16-bit short address
     */
     *buf++ = 0x26;//0bxx100110;
     
     // Sequence number - 0x00 as default
     *buf++ = 0x00;
     // Destination address
     *buf++ = (uint8_t) (PAN_ID >> 8);
     *buf++ = (uint8_t) PAN_ID;
     *buf++ = (uint8_t) (SHORT_ADDR >> 8);
     *buf++ = (uint8_t) SHORT_ADDR;
     // Source address
     *buf++ = (uint8_t) (PAN_ID >> 8);
     *buf++ = (uint8_t) PAN_ID;
     *buf++ = (uint8_t) (SHORT_ADDR >> 8);
     *buf++ = (uint8_t) SHORT_ADDR;
     
     // 11 bytes before data payload
     
     // Data payload
     for (int i = 0; i < data_length; i++)
     {
        *buf++ = data[i];
     }
     
     // MAC Footer (MFR) Fields - further two secure bytes follow.
     return ptr;
}

void SPI_write_frame_buf(uint8_t *data)
{
    uint8_t *ptr = NULL;
    uint8_t len = 0;
    
    ptr = build_frame(&len, data);
    
    uint8_t buf;
    
    uint8_t command = FRAME_BUFFER_WRITE_ACCESS_COMMAND;
    
    GPIOA->BSRRH |= GPIO_Pin_4;
    while(!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = command;
    while(!(SPI1->SR & SPI_SR_RXNE));
    buf = SPI1->DR;
    
    // PHR - 1 byte + MPDU - 2 byte - FCS autogenerated
    for(int k = 0; k < len + 1; k++)
    {
        while(!(SPI1->SR & SPI_SR_TXE));
        SPI1->DR = *ptr++; // write data and incr pointer
        while(!(SPI1->SR & SPI_SR_RXNE));
        buf = SPI1->DR; // read return value
    }
    GPIOA->BSRRH &= ~GPIO_Pin_4;
    GPIOA->BSRRL |= GPIO_Pin_4;   
}

uint8_t* SPI_read_frame_buf()
{
    uint8_t buf;
    uint8_t command = FRAME_BUFFER_READ_ACCESS_COMMAND;
        
    GPIOA->BSRRH |= GPIO_Pin_4;
    while(!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = command;

    while(!(SPI1->SR & SPI_SR_RXNE));
    buf = SPI1->DR; //read phy_status - get length
    
    while(!(SPI1->SR & SPI_SR_TXE));
    // Dummy byte to generate clock
    SPI1->DR = SPI_DUMMY_VALUE;
    while(!(SPI1->SR & SPI_SR_RXNE));
    buf = SPI1->DR; //read PHR - get length
    
    uint8_t *data = NULL;
    uint8_t *head = NULL;
    
    if (buf != 0)
    {
        data = (uint8_t *) calloc(buf + 5, sizeof(uint8_t));
        head = data;
        *data++ = buf;
    
        for (int i = 0; i < buf + 4; i++)
        {
            while(!(SPI1->SR & SPI_SR_TXE));
            // Dummy byte to generate clock
            SPI1->DR = SPI_DUMMY_VALUE;
            while(!(SPI1->SR & SPI_SR_RXNE));
            *data++ = SPI1->DR;
        }
    }

    
    GPIOA->BSRRH &= ~GPIO_Pin_4;
    GPIOA->BSRRL |= GPIO_Pin_4;
    return head;
}

uint8_t SPI_sram_read_frame_buf(uint8_t addr)
{
    uint8_t buf;
    uint8_t command = SRAM_FRAME_BUFFER_READ_ACCESS_COMMAND;
        
    GPIOA->BSRRH |= GPIO_Pin_4;
    while(!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = command;

    while(!(SPI1->SR & SPI_SR_RXNE));
    buf = SPI1->DR; //read phy_status
    
    while(!(SPI1->SR & SPI_SR_TXE));
    // Byte of address to read
    SPI1->DR = addr;
    while(!(SPI1->SR & SPI_SR_RXNE));
    buf = SPI1->DR; // read return value
    
    while(!(SPI1->SR & SPI_SR_TXE));
    // Dummy byte to generate clock
    SPI1->DR = SPI_DUMMY_VALUE;
    while(!(SPI1->SR & SPI_SR_RXNE));
    buf = SPI1->DR; // read register data
    
    GPIOA->BSRRH &= ~GPIO_Pin_4;
    GPIOA->BSRRL |= GPIO_Pin_4;
    //SPI_Cmd(SPI1, DISABLE);

    return buf;
}

void SPI_sram_write_frame_buf(uint8_t addr, uint8_t data)
{
    uint8_t buf;
    uint8_t command = SRAM_FRAME_BUFFER_WRITE_ACCESS_COMMAND;
        
    GPIOA->BSRRH |= GPIO_Pin_4;
    while(!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = command;

    while(!(SPI1->SR & SPI_SR_RXNE));
    buf = SPI1->DR; //read phy_status
    
    while(!(SPI1->SR & SPI_SR_TXE));
    // Byte to access needed register
    SPI1->DR = addr;
    while(!(SPI1->SR & SPI_SR_RXNE));
    buf = SPI1->DR; // read return value
    
    while(!(SPI1->SR & SPI_SR_TXE));
    // Byte of data to write
    SPI1->DR = data;
    while(!(SPI1->SR & SPI_SR_RXNE));
    buf = SPI1->DR; // read return value
    
    GPIOA->BSRRH &= ~GPIO_Pin_4;
    GPIOA->BSRRL |= GPIO_Pin_4;
    
}

void TRX_init_batmon_reg()
{
    // low voltage range is set, low voltage threshold is set to 0x0 - 1.7v
    uint8_t data = 0x00;
    
    SPI_read_reg(RG_BATMON, &data);
    USART_send_string_data("\n\rRG_BATMON: ");
    USART_send_hex_data(&data);
    
    data &= 0Xf0;//0Xe0
    data |= 0X10;
    SPI_write_reg(RG_BATMON, (uint16_t) data);
    
    SPI_read_reg(RG_BATMON, &data);
    USART_send_string_data("RG_BATMON: ");
    USART_send_hex_data(&data);
}

void TRX_change_state(uint8_t new_state)
{
    uint8_t data = 0;
    SPI_read_reg(RG_TRX_STATUS, &data);
    
    while(data != new_state)
    {
        if (data != STATE_TRANSITION_IN_PROGRESS)
        {
            SPI_write_reg(RG_TRX_STATE, new_state);
        }
        SPI_read_reg(RG_TRX_STATUS, &data);
            
        if ((new_state == CMD_FORCE_PLL_ON) && (data == PLL_ON))
        {
            break;
        }
        
        if ((new_state == CMD_FORCE_TRX_OFF) && (data == TRX_OFF))
        {
            break;
        }
        
        if ((new_state == CMD_TX_START) && ((data == BUSY_TX) || (data == BUSY_TX_ARET)))
        {
            break;
        }
    }
}

void TRX_choose_role()
{
    if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15))
    {
       USART_send_string_data("RTB_ROLE_SELECTION: ");
       
       uint16_t led = GPIO_Pin_12;
       
       while(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15))
       {
            GPIO_SetBits(GPIOB, led);
            Delay(12000000);
            GPIO_ResetBits(GPIOB, led);
            if (led < GPIO_Pin_13) 
            {
                led = led << 1;
            }
            else led = GPIO_Pin_12;
       }
       
       if (led == GPIO_Pin_12)
       {
            USART_send_string_data("Initiator\n\r");
            peer_role = INITIATOR;
            TRX_change_state(PLL_ON);
       }
       else if (led == GPIO_Pin_13)
       {
            USART_send_string_data("Reflector\n\r");
            peer_role = REFLECTOR;
            TRX_change_state(RX_ON);
            peer_state = WAIT_FOR_FRAME;
       }
       
        uint8_t interrupt;
        SPI_read_reg(RG_IRQ_STATUS, &interrupt);
    }
}

void TRX_print_reg_info()
{
    uint8_t data;
    
    USART_send_string_data("h: Register information\n\r");
    
    SPI_read_reg(RG_TRX_CTRL_1, &data);
    USART_send_string_data("RG_TRX_CTRL_1: ");
    USART_send_hex_data(&data);
    SPI_read_reg(RG_IRQ_MASK, &data);
    USART_send_string_data("RG_IRQ_MASK: ");
    USART_send_hex_data(&data);
    
    TRX_read_addr_fields();
    USART_send_string_data("RG_TRX_STATUS: ");
    SPI_read_reg(RG_TRX_STATUS, &data);
    USART_send_hex_data(&data);
    USART_send_string_data("RG_IRQ_STATUS: ");
    SPI_read_reg(RG_IRQ_STATUS, &data);
    USART_send_hex_data(&data);
}

void USART_send_number(uint16_t number, bool nl)
{
    uint16_t thres = 10000;
    bool allow = false;
  
    while(thres >= 10)
    {
        if (number >= thres)
        {   
            allow = true;
        }
        if (allow)
        {
            USART1->DR = number / thres + '0';
            while(!(USART1->SR & USART_SR_TXE));
            while(!(USART1->SR & USART_SR_TC));
            number = number % thres;
        }
        thres = thres / 10;
    }
    USART1->DR = number + '0';
    while(!(USART1->SR & USART_SR_TXE));
    while(!(USART1->SR & USART_SR_TC));
    if (nl)
    {
        USART1->DR = 10;
        while(!(USART1->SR & USART_SR_TXE));
        while(!(USART1->SR & USART_SR_TC));
        USART1->DR = 13;
        while(!(USART1->SR & USART_SR_TXE));
        while(!(USART1->SR & USART_SR_TC));
    }
}

void calculate_distance()
{
    float prom_res[4] = {0,0,0,0};
    float prom_res_peer[4] = {0,0,0,0};
    float res = 0;
    
    for (uint8_t j = 0; j < 4; j++)
    {
        for (int i = 0; i < RESULT_NUMBER - 1; i++)
        {   
            
                prom_res[j] += pmu_res_peer[j][i + 1] - pmu_res_peer[j][i];
                prom_res[j] += pmu_res[j][i + 1] - pmu_res[j][i];           
            prom_res[j] = prom_res[j] / 2;
        }
        prom_res[j] = prom_res[j] / (RESULT_NUMBER - 1);
    }
    
    // 0.586;// 255 * 150 * 100;
    // because sm / 4 * 6.28 / 6.28 / 500000 * 300 000 000; 0.373
    res = (prom_res[0] + prom_res[1] + prom_res[2] + prom_res[3])\
      * 0.294 * 100;
    USART_send_string_data("RESULT, in sm: ");
    USART_send_number((uint8_t)res, true);
}


void SPI_read_pmu_values()
{
    uint8_t buf;
    uint8_t address = READ_ACCESS_COMMAND | RG_PHY_PMU_VALUE;
    
    for (short i = 0; i < PMU_VALUES_DEFAULT_NUMBER; i++)
    {
        GPIOA->BSRRH |= GPIO_Pin_4;
        while(!(SPI1->SR & SPI_SR_TXE));
        SPI1->DR = address;
        while(!(SPI1->SR & SPI_SR_RXNE));
        buf = SPI1->DR;
         
        while(!(SPI1->SR & SPI_SR_TXE));
        // Dummy byte to generate clock
        SPI1->DR = SPI_DUMMY_VALUE;//0x00;
        while(!(SPI1->SR & SPI_SR_RXNE));
        pmu_measurements[i] = SPI1->DR;
        
        GPIOA->BSRRH &= ~GPIO_Pin_4;
        GPIOA->BSRRL |= GPIO_Pin_4;
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
        __no_operation();
    }
}

void TRX_send_frame()
{
    if ((peer_state != WAIT_FOR_REQ_FRAME) && \
            !((peer_state == TOM_MSRM) || (peer_state == PMU_MSRM)))
    {
        if (tom_service_struct.tom_state == TOM_SENDER)
        {
            SPI_write_frame_buf("TOM");
        }
        else if (pmu_service_struct.pmu_state == PMU_SENDER)
        {
            SPI_write_frame_buf("PMU");
        }
    
        TRX_change_state(PLL_ON);
    }
    else
    {
        TRX_change_state(CMD_TX_ARET_ON);
        if (tom_service_struct.tom_state == TOM_SENDER)
        {
            SPI_write_frame_buf("TOM");
            peer_state = TOM_MSRM;
        }
        else if (pmu_service_struct.pmu_state == PMU_SENDER)
        {
            SPI_write_frame_buf("PMU");
            peer_state = PMU_MSRM;
        }
        NVIC_EnableIRQ(EXTI0_IRQn);
        NVIC_DisableIRQ(EXTI1_IRQn);
    }
    SLP_TR_PULSE;
    peer_action = NO_ACTION;
}