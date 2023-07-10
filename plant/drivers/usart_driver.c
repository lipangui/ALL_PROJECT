#include "usart_driver.h"
#include "system_driver.h"
#include "common.h"
#include "utils_queue.h"
#include "stm32f1xx.h"
#define USART123_RCC() do{ __HAL_RCC_USART1_CLK_ENABLE();\
											  __HAL_RCC_USART2_CLK_ENABLE();\
											  __HAL_RCC_USART3_CLK_ENABLE();\
											  __HAL_RCC_DMA1_CLK_ENABLE();}while(0)
#define GPIO_ADCD_RCC() do{__HAL_RCC_GPIOA_CLK_ENABLE();\
												__HAL_RCC_GPIOB_CLK_ENABLE();\
												__HAL_RCC_GPIOC_CLK_ENABLE();\
												__HAL_RCC_GPIOD_CLK_ENABLE();}while(0)
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
        while((USART1->SR&0X40)==0);
        USART1->DR = (uint8_t)ch;
        return ch;
}
typedef struct
{
	GPIO_TypeDef *gpio;
	USART_TypeDef *usart;
	IRQn_Type usart_irq;
	IRQn_Type dma_irq;
	queue_t *tx_usart;
	queue_t *rx_usart;
	DMA_Channel_TypeDef *tx_dma;
	DMA_Channel_TypeDef *rx_dma;
	uint32_t baudrate;
	bool dma_enable;
	uint8_t tx_status;
}usart_t;
usart_t serial_usart[7];
UART_HandleTypeDef usart[7];
DMA_HandleTypeDef tx_dmax[7];
DMA_HandleTypeDef rx_dmax[7];
static uint16_t content_number=0;
static uint16_t rx_content_number=0;
static bool recv=false;
static uint16_t serial_dma_count[7];
void (*MY_UART_IDLECallback)(void);

static void serial_usart_init(serialx_t usartx)
{
	 switch(usartx)
	 {
        case serial1:	serial_usart[usartx].dma_irq=DMA1_Channel4_IRQn;
        					serial_usart[usartx].rx_dma=DMA1_Channel5;
        					serial_usart[usartx].tx_dma=DMA1_Channel4;
        					serial_usart[usartx].usart=USART1;
        					serial_usart[usartx].usart_irq=USART1_IRQn;
        					break;
        case serial2:	serial_usart[usartx].dma_irq=DMA1_Channel7_IRQn;
        					serial_usart[usartx].rx_dma=DMA1_Channel6;
	 	 	 	 	 	    serial_usart[usartx].tx_dma=DMA1_Channel7;
	 	 	 	 	 	    serial_usart[usartx].usart=USART2;
	 	 	 	 	 	    serial_usart[usartx].usart_irq=USART2_IRQn;
	 	 	 	 	 	    break;
        case serial3: serial_usart[usartx].dma_irq=DMA1_Channel2_IRQn;
        					serial_usart[usartx].rx_dma=DMA1_Channel3;
        					serial_usart[usartx].tx_dma=DMA1_Channel2;
        					serial_usart[usartx].usart=USART3;
        					serial_usart[usartx].usart_irq=USART3_IRQn;
	 	 	 	 	 	    break;
/*	 	 case serial4: __HAL_RCC_USART1_CLK_ENABLE();
	 	 	 	 	 	 	__HAL_RCC_GPIOA_CLK_ENABLE();
	 	 	 	 	 	 	break;
	 	 case serial5:__HAL_RCC_USART1_CLK_ENABLE();
	 	 	 	 	 	 	__HAL_RCC_GPIOA_CLK_ENABLE();
                            break;
	 	 case serial6: __HAL_RCC_USART1_CLK_ENABLE();
	 	 	 	 	 	 	__HAL_RCC_GPIOA_CLK_ENABLE();
                            break;*/
	 	default:break;
	 }
	// __HAL_RCC_DMA2_CLK_ENABLE();
}
static void serial_gpio_tx_select(serialx_t usartx)
{
	  GPIO_InitTypeDef GPIO_InitStruct;
	switch(usartx)
	{
		case serial1:    GPIO_InitStruct.Pin = GPIO_PIN_9;serial_usart[usartx].gpio=GPIOA;break;
		case serial2:    GPIO_InitStruct.Pin = GPIO_PIN_2;serial_usart[usartx].gpio=GPIOA;break;
		case serial3:    GPIO_InitStruct.Pin = GPIO_PIN_10;serial_usart[usartx].gpio=GPIOB;break;
/*		case serial4:    GPIO_InitStruct.Pin = GPIO_PIN_9;break;
		case serial5:    GPIO_InitStruct.Pin = GPIO_PIN_9;break;
		case serial6:    GPIO_InitStruct.Pin = GPIO_PIN_9;break;*/
	 	default:break;
	}
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(serial_usart[usartx].gpio, &GPIO_InitStruct);
}
static void serial_gpio_rx_select(serialx_t usartx)
{
	  GPIO_InitTypeDef GPIO_InitStruct;
	switch(usartx)
	{
		case serial1:    GPIO_InitStruct.Pin = GPIO_PIN_10;serial_usart[usartx].gpio=GPIOA;break;
		case serial2:    GPIO_InitStruct.Pin = GPIO_PIN_3;serial_usart[usartx].gpio=GPIOA;break;
		case serial3:    GPIO_InitStruct.Pin = GPIO_PIN_11;serial_usart[usartx].gpio=GPIOB;break;
/*		case serial4:    GPIO_InitStruct.Pin = GPIO_PIN_9;break;
		case serial5:    GPIO_InitStruct.Pin = GPIO_PIN_9;break;
		case serial6:    GPIO_InitStruct.Pin = GPIO_PIN_9;break;*/
	 	default:break;
	}
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(serial_usart[usartx].gpio, &GPIO_InitStruct);
}
static void serial_set_dma(bool usart_flag,serialx_t port)
{
	if(usart_flag==true)
	{
		serial_usart[port].dma_enable=usart_flag;
		if(serial_usart[port].rx_dma)
		{
			rx_dmax[port].Instance = serial_usart[port].rx_dma;
			rx_dmax[port].Init.PeriphInc = DMA_PINC_DISABLE;
			rx_dmax[port].Init.MemInc = DMA_MINC_ENABLE;
			rx_dmax[port].Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
			rx_dmax[port].Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
			rx_dmax[port].Init.Priority = DMA_PRIORITY_MEDIUM;
			rx_dmax[port].Init.Direction = DMA_PERIPH_TO_MEMORY;
			rx_dmax[port].Init.Mode = DMA_CIRCULAR;
			HAL_DMA_Init(&rx_dmax[port]) ;
	        __HAL_LINKDMA(&usart[port],hdmarx,rx_dmax[port]);

		}
		if(serial_usart[port].tx_dma)
		{
			tx_dmax[port].Instance = serial_usart[port].tx_dma;
			tx_dmax[port].Init.PeriphInc = DMA_PINC_DISABLE;
			tx_dmax[port].Init.MemInc = DMA_MINC_ENABLE;
			tx_dmax[port].Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
			tx_dmax[port].Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
			tx_dmax[port].Init.Priority = DMA_PRIORITY_MEDIUM;
			tx_dmax[port].Init.Direction = DMA_MEMORY_TO_PERIPH;
			tx_dmax[port].Init.Mode = DMA_NORMAL;
			//tx_dmax.Init.Mode = DMA_CIRCULAR;
			HAL_DMA_Init(&tx_dmax[port]) ;
			HAL_NVIC_SetPriority(serial_usart[port].dma_irq, port%4, port/4);
			HAL_NVIC_EnableIRQ(serial_usart[port].dma_irq);
			__HAL_LINKDMA(&usart[port],hdmatx,tx_dmax[port]);
		}
	}
	else{
		HAL_NVIC_SetPriority(serial_usart[port].usart_irq, port/4, port%4);
		HAL_NVIC_EnableIRQ(serial_usart[port].usart_irq);
	}
}
static void serial_usart_cofing(serialx_t port,uint32_t baudrate)
{
	usart[port].Instance = serial_usart[port].usart;
	usart[port].Init.BaudRate = baudrate;
	usart[port].Init.WordLength = UART_WORDLENGTH_8B;
	usart[port].Init.StopBits = UART_STOPBITS_1;
	usart[port].Init.Parity = UART_PARITY_NONE;
	usart[port].Init.Mode = UART_MODE_TX_RX;
	usart[port].Init.HwFlowCtl = UART_HWCONTROL_NONE;
	usart[port].Init.OverSampling = UART_OVERSAMPLING_16;
}
void serial_driver_open(uint32_t baudrate,bool usart_flag,serialx_t port,queue_t *rx_buffer,queue_t *tx_buffer,void (*serial_handler)(void) )
{
	USART123_RCC();
	GPIO_ADCD_RCC();
	serial_usart_init(port);
	serial_gpio_tx_select(port);
	serial_gpio_rx_select(port);

	serial_usart[port].rx_usart=rx_buffer;
	serial_usart[port].tx_usart=tx_buffer;

	serial_usart_cofing(port,baudrate);
	serial_set_dma(usart_flag,port);

	HAL_UART_Init(&usart[port]);
	if(usart_flag==true)
	{
		HAL_UART_Receive_DMA(&usart[port],serial_usart[port].rx_usart->buffer,serial_usart[port].rx_usart->size);
		serial_dma_count[port]=__HAL_DMA_GET_COUNTER(usart[port].hdmarx);
	}
	else
	{
		MY_UART_IDLECallback=serial_handler;
		__HAL_UART_ENABLE_IT(&usart[port], UART_IT_RXNE);
		__HAL_UART_ENABLE_IT(&usart[port], UART_IT_IDLE);
		__HAL_UART_ENABLE(&usart[port]);
	}
}
bool driver_is_busy(serialx_t port)
{
   if(port>=serial_max)	return true;
   else return false;
}
void serial_driver_set_baudrate(serialx_t port,uint32_t baudratex)
{
	USART123_RCC();
	serial_usart_cofing(port,baudratex);
	HAL_UART_Init(&usart[port]);
	if(serial_usart[port].dma_enable==true)
	{
		HAL_UART_Receive_DMA(&usart[port],serial_usart[port].rx_usart->buffer,serial_usart[port].rx_usart->size);
	}
	else
	{
		__HAL_UART_ENABLE_IT(&usart[port], UART_IT_RXNE);
		__HAL_UART_ENABLE_IT(&usart[port], UART_IT_IDLE);
		__HAL_UART_ENABLE(&usart[port]);
	}
}
static void serial_driver_dma_send(serialx_t port,uint8_t *tx_content,uint16_t len)
{

	if(len!=0)
		{
			while(serial_usart[port].tx_status!= SERIAL_STATUS_IDLE);
			serial_usart[port].tx_usart->front = 0;
			serial_usart[port].tx_usart->rail = 0;
			while(len)
			{
				len--;
				serial_usart[port].tx_usart->buffer[serial_usart[port].tx_usart->front++]=tx_content[content_number++];
				if(serial_usart[port].tx_usart->front==serial_usart[port].tx_usart->size)  break;
			}
			serial_usart[port].tx_status=SERIAL_STATUS_DMA;
			usart[port].hdmatx->Instance->CMAR=(uint32_t)serial_usart[port].tx_usart->buffer;
			usart[port].hdmatx->Instance->CNDTR=serial_usart[port].tx_usart->front;
			HAL_DMA_Start_IT(usart[port].hdmatx, (uint32_t) serial_usart[port].tx_usart->buffer, (uint32_t)&usart[port].Instance->DR, serial_usart[port].tx_usart->front);
			__HAL_DMA_ENABLE(usart[port].hdmatx);
			usart[port].Instance->CR3 |= USART_CR3_DMAT;
			//HAL_UART_Transmit_DMA(&usart[port],serial_usart[port].tx_usart->buffer,serial_usart[port].tx_usart->front);
			if(len > 0) serial_driver_dma_send(port, tx_content, len);
		}
	if(len==0) content_number=0;
}
void serial_driver_send(serialx_t port,uint8_t *tx_content,uint16_t len)
{
	if( serial_usart[port].dma_enable==true)
	{
	  serial_driver_dma_send(port,tx_content,len);
	}
	if(serial_usart[port].dma_enable==false)
	{
		HAL_UART_Transmit(&usart[port],tx_content,len,10);
	}
}
static bool serial_driver_dma_read(serialx_t port,uint8_t *rx_content)
{
	 uint16_t lenght;
	 serial_usart[port].rx_usart->rail=serial_usart[port].rx_usart->size - __HAL_DMA_GET_COUNTER(usart[port].hdmarx);
	 lenght = ( serial_usart[port].rx_usart->rail + serial_usart[port].rx_usart->size - serial_usart[port].rx_usart->front )%serial_usart[port].rx_usart->size;
	 if(queue_empty_full(serial_usart[port].rx_usart)!=0)
	 {
	   while(lenght)
	   {
		   lenght--;
		   rx_content[rx_content_number++] = queue_pop(serial_usart[port].rx_usart);
	   }
	       rx_content[rx_content_number]='\0';
	       return true;
	  }
	 else if(queue_empty_full(serial_usart[port].rx_usart)==0) rx_content_number=0;
	 return false;
}

static bool serial_driver_nomal_read(serialx_t port,uint8_t *rx_content)
{
	uint16_t i=0;
	while(queue_empty_full(serial_usart[port].rx_usart)!=0&&recv==true)
	{
		rx_content[i++]=queue_pop(serial_usart[port].rx_usart);
	}
	recv=false;
	if(i!=0)
	{
		rx_content[i]='\0';
		return true;
	}
	else return false;
}
bool serial_driver_read(serialx_t port,uint8_t *rx_content)
{
	if(serial_usart[port].dma_enable==true)
	{
		return serial_driver_dma_read(port,rx_content);
	}
	else
	{
		return serial_driver_nomal_read(port,rx_content);
	}
}
void serial_driver_update(serialx_t port)
{
	uint16_t dma_count=0;
	uint16_t len=0;
	dma_count=__HAL_DMA_GET_COUNTER(usart[port].hdmarx);
    if (dma_count > serial_dma_count[port])
    {
        len =serial_usart[port].rx_usart->size + serial_dma_count[port] - dma_count;
    }
    else if (dma_count < serial_dma_count[port])
    {
        len = serial_dma_count[port] - dma_count;
    }
    serial_dma_count[port]=dma_count;
    serial_usart[port].rx_usart->rail=(serial_usart[port].rx_usart->rail+len)%serial_usart[port].rx_usart->size;
    if(len>0)
    {
    if(serial_usart[port].rx_usart->rail==serial_usart[port].rx_usart->front) serial_usart[port].rx_usart->overflow=true;
    }
}
uint8_t serial_driver_receive(serialx_t port)
{
	if(serial_usart[port].dma_enable==true)
	serial_driver_update(port);
	queue_empty_full(serial_usart[port].rx_usart);
	if(serial_usart[port].rx_usart->empty==true)
	return 0;
	return queue_pop(serial_usart[port].rx_usart);
}
void MY_HAL_UART_IRQHandler(UART_HandleTypeDef *huart,serialx_t port)
{
	if( __HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)!=RESET)
	{
		queue_push(serial_usart[port].rx_usart,huart->Instance->DR);
		__HAL_UART_CLEAR_FLAG(huart,UART_FLAG_RXNE);
	}
	if( __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)!=RESET )
	{
		queue_empty_full(serial_usart[port].rx_usart);
		recv=true;
		MY_UART_IDLECallback();
		 __HAL_UART_CLEAR_IDLEFLAG(huart);
	}
}
void MY_HAL_USART_DMA_IRQHandler(UART_HandleTypeDef *huart,serialx_t port)
{
	if ((((huart->hdmatx->DmaBaseAddress->ISR) & (DMA_FLAG_TC1 << huart->hdmatx->ChannelIndex)) != RESET) && (((huart->hdmatx->Instance->CCR) & DMA_IT_TC) != RESET))
	{
		__HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmatx));
		__HAL_DMA_DISABLE(huart->hdmatx);
		serial_usart[port].tx_status=SERIAL_STATUS_IDLE;
	}
/*    if(__HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmatx) == RESET)
    {
            __HAL_DMA_CLEAR_FLAG(huart->hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmatx));
            __HAL_DMA_DISABLE(huart->hdmatx);

            serial_usart[port].tx_status = SERIAL_STATUS_IDLE;
    }*/
}
void USART1_IRQHandler(void)
{
	MY_HAL_UART_IRQHandler(&usart[1],1);
}
void USART2_IRQHandler(void)
{
	MY_HAL_UART_IRQHandler(&usart[2],2);
}
void USART3_IRQHandler(void)
{
	MY_HAL_UART_IRQHandler(&usart[3],3);
}
/*
void USART4_IRQHandler(void)
{
	  HAL_UART_IRQHandler(&usart[4]);
}
void USART5_IRQHandler(void)
{
	  HAL_UART_IRQHandler(&usart[5]);
}
void USART6_IRQHandler(void)
{
	  HAL_UART_IRQHandler(&usart[6]);
}*/

void DMA1_Channel4_IRQHandler(void)
{
	MY_HAL_USART_DMA_IRQHandler(&usart[1],1);
}
void DMA1_Channel2_IRQHandler(void)
{
	MY_HAL_USART_DMA_IRQHandler(&usart[3],3);
}
void DMA1_Channel7_IRQHandler(void)
{
	MY_HAL_USART_DMA_IRQHandler(&usart[2],2);
}
