#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Ini 1 do motor de passo */
#define IN1_PIO     PIOD
#define IN1_PIO_ID  ID_PIOD
#define IN1_PIO_PIN 30
#define IN1_PIO_PIN_MASK (1 << IN1_PIO_PIN)

/* Ini 2 do motor de passo */
#define IN2_PIO     PIOA
#define IN2_PIO_ID  ID_PIOA
#define IN2_PIO_PIN 6
#define IN2_PIO_PIN_MASK (1 << IN2_PIO_PIN)

/* Ini 3 do motor de passo */
#define IN3_PIO     PIOC
#define IN3_PIO_ID  ID_PIOC
#define IN3_PIO_PIN 19
#define IN3_PIO_PIN_MASK (1 << IN3_PIO_PIN)

/* Ini 4 do motor de passo */
#define IN4_PIO     PIOA
#define IN4_PIO_ID  ID_PIOA
#define IN4_PIO_PIN 2
#define IN4_PIO_PIN_MASK (1 << IN4_PIO_PIN)

/* Botao 1 do OLED */
#define BUT1_PIO PIOD
#define BUT1_PIO_ID ID_PIOD
#define BUT1_PIO_IDX 28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX)

/* Botao 2 do OLED */
#define BUT2_PIO PIOC
#define BUT2_PIO_ID ID_PIOC
#define BUT2_PIO_IDX 31
#define BUT2_PIO_IDX_MASK (1u << BUT2_PIO_IDX)

/* Botao 3 do OLED */
#define BUT3_PIO PIOA
#define BUT3_PIO_ID ID_PIOA
#define BUT3_PIO_IDX 19
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX)

/* Constants */
#define DEGREES_PER_STEP 0.17578125
#define ECHO_PRESCALE 1000
#define ECHO_MAX_WAIT_TIME 5


/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_MODO_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MODO_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_MOTOR_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MOTOR_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

xQueueHandle xQueueModo;
xQueueHandle xQueueSteps;

SemaphoreHandle_t xSemaphoreRTT;

/** prototypes */
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);
void motor_init(void);
void RTT_Handler(void);

static void BUT_init(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
static void configure_console(void);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but1_callback(void) {
	uint32_t modo = 180;
    xQueueSendFromISR(xQueueModo, &modo, 10);
}

void but2_callback(void) {
	uint32_t modo = 90;
	xQueueSendFromISR(xQueueModo, &modo, 10);
}

void but3_callback(void) {
	uint32_t modo = 45;
	xQueueSendFromISR(xQueueModo, &modo, 10);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_modo(void *pvParameters) {
	gfx_mono_ssd1306_init();
  	gfx_mono_draw_string("Modo: ", 0, 0, &sysfont);
	gfx_mono_draw_string("Esperando", 0, 20, &sysfont);
	
	uint32_t modo;
	char modo_str[10];

	uint32_t steps;

	for (;;)  {
		if (xQueueReceive(xQueueModo, &modo, 1000)) {
			// Atualiza o OLED
            sprintf(modo_str, "%u graus", modo);
			gfx_mono_draw_string(modo_str, 0, 20, &sysfont);

			// Envia para a fila de passos
			steps = (uint32_t) (modo / DEGREES_PER_STEP);
			xQueueSend(xQueueSteps, &steps, 10);
		}
	}
}

static void task_motor(void *pvParameters) {
	motor_init();

	uint32_t steps;

	for (;;)  {
		if (xQueueReceive(xQueueSteps, &steps, 1000)) {
			// Gira o motor
			for (uint32_t i = 0; i < steps; i += 4) {
				pio_set(IN1_PIO, IN1_PIO_PIN_MASK);
				pio_clear(IN2_PIO, IN2_PIO_PIN_MASK);
				pio_clear(IN3_PIO, IN3_PIO_PIN_MASK);
				pio_clear(IN4_PIO, IN4_PIO_PIN_MASK);

				RTT_init(ECHO_PRESCALE, ECHO_MAX_WAIT_TIME, RTT_MR_ALMIEN);
				for (;;) {
					if (xSemaphoreTake(xSemaphoreRTT, 200) == pdTRUE) {
						break;
					}	
				}
			
				pio_clear(IN1_PIO, IN1_PIO_PIN_MASK);
				pio_set(IN2_PIO, IN2_PIO_PIN_MASK);
				pio_clear(IN3_PIO, IN3_PIO_PIN_MASK);
				pio_clear(IN4_PIO, IN4_PIO_PIN_MASK);
				
				RTT_init(ECHO_PRESCALE, ECHO_MAX_WAIT_TIME, RTT_MR_ALMIEN);
				for (;;) {
					if (xSemaphoreTake(xSemaphoreRTT, 200) == pdTRUE) {
						break;
					}	
				}

				pio_clear(IN1_PIO, IN1_PIO_PIN_MASK);
				pio_clear(IN2_PIO, IN2_PIO_PIN_MASK);
				pio_set(IN3_PIO, IN3_PIO_PIN_MASK);
				pio_clear(IN4_PIO, IN4_PIO_PIN_MASK);
				
				RTT_init(ECHO_PRESCALE, ECHO_MAX_WAIT_TIME, RTT_MR_ALMIEN);
				for (;;) {
					if (xSemaphoreTake(xSemaphoreRTT, 200) == pdTRUE) {
						break;
					}	
				}

				pio_clear(IN1_PIO, IN1_PIO_PIN_MASK);
				pio_clear(IN2_PIO, IN2_PIO_PIN_MASK);
				pio_clear(IN3_PIO, IN3_PIO_PIN_MASK);
				pio_set(IN4_PIO, IN4_PIO_PIN_MASK);

				RTT_init(ECHO_PRESCALE, ECHO_MAX_WAIT_TIME, RTT_MR_ALMIEN);
				for (;;) {
					if (xSemaphoreTake(xSemaphoreRTT, 200) == pdTRUE) {
						break;
					}	
				}
			}
		}
		
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void BUT_init(void) {
	/* liga o clock do button */
    pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);

    /* conf botao como entrada */
    pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);

    pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
	pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, 60);
	pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 60);

    pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but1_callback);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but2_callback);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but3_callback);

    pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
    pio_get_interrupt_status(BUT1_PIO);
	
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT2_PIO);

	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT3_PIO);

    /* configura prioridae */
    NVIC_EnableIRQ(BUT1_PIO_ID);
    NVIC_SetPriority(BUT1_PIO_ID, 4);

	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 4);

	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 4);
}


void motor_init(void) {
    pmc_enable_periph_clk(IN1_PIO_ID);
	pmc_enable_periph_clk(IN2_PIO_ID);
	pmc_enable_periph_clk(IN3_PIO_ID);
	pmc_enable_periph_clk(IN4_PIO_ID);

	pio_configure(IN1_PIO, PIO_OUTPUT_0, IN1_PIO_PIN_MASK, PIO_DEFAULT);
	pio_configure(IN2_PIO, PIO_OUTPUT_0, IN2_PIO_PIN_MASK, PIO_DEFAULT);
	pio_configure(IN3_PIO, PIO_OUTPUT_0, IN3_PIO_PIN_MASK, PIO_DEFAULT);
	pio_configure(IN4_PIO, PIO_OUTPUT_0, IN4_PIO_PIN_MASK, PIO_DEFAULT);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

    uint16_t pllPreScale = (int)(((float)32768) / freqPrescale);

    rtt_sel_source(RTT, false);
    rtt_init(RTT, pllPreScale);

    if (rttIRQSource & RTT_MR_ALMIEN) {
        uint32_t ul_previous_time;
        ul_previous_time = rtt_read_timer_value(RTT);
        while (ul_previous_time == rtt_read_timer_value(RTT));
        rtt_write_alarm_time(RTT, IrqNPulses + ul_previous_time);
    }

    /* config NVIC */
    NVIC_DisableIRQ(RTT_IRQn);
    NVIC_ClearPendingIRQ(RTT_IRQn);
    NVIC_SetPriority(RTT_IRQn, 4);
    NVIC_EnableIRQ(RTT_IRQn);

    /* Enable RTT interrupt */
    if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
        rtt_enable_interrupt(RTT, rttIRQSource);
    else
        rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
}

void RTT_Handler(void) {
  uint32_t ul_status;
  ul_status = rtt_get_status(RTT);

  /* IRQ due to Alarm */
  if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
    	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphoreRTT, &xHigherPriorityTaskWoken);
    }  
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the buttons */
	BUT_init();

	/* Initialize the console uart */
	configure_console();

	/* Create task to control modo */
	if (xTaskCreate(task_modo, "modo", TASK_MODO_STACK_SIZE, NULL, TASK_MODO_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create modo task\r\n");
	}

	/* Create task to control motor */
	if (xTaskCreate(task_motor, "motor", TASK_MOTOR_STACK_SIZE, NULL, TASK_MOTOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create motor task\r\n");
	}

	xQueueModo = xQueueCreate(32, sizeof(uint32_t));
	if(xQueueModo == NULL) {
		printf("Failed to create queue modo\r\n");
	}

	xQueueSteps = xQueueCreate(32, sizeof(uint32_t));
	if(xQueueSteps == NULL) {
		printf("Failed to create queue steps\r\n");
	}

	xSemaphoreRTT = xSemaphoreCreateBinary();
	if (xSemaphoreRTT == NULL) {
		printf("Failed to create RTT semaphore\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS n�o deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
