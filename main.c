#include "stm32f10x.h"
#include "pin_config.h"
#include "dht11.h"
#include "ir_nec.h"
#include "esp8266.h"
#include "http_handler.h"
#include "ssd1306.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ============ 全局变量 ============ */

static volatile uint32_t sys_tick_ms = 0;

volatile uint8_t lock_locked = 1;
volatile uint8_t system_armed = 0;
volatile uint8_t g_light_on = 0;
volatile uint8_t g_ac_on = 0;
volatile uint8_t g_lock_on = 0;
volatile uint8_t g_curtain_percent = 0;
volatile uint8_t g_system_armed = 0;
volatile uint8_t g_alert_active = 0;
volatile int16_t g_temperature = 0;
volatile int16_t g_humidity = 0;

volatile uint8_t g_led_living = 0;
volatile uint8_t g_led_bedroom = 0;
volatile uint8_t g_led_kitchen = 0;

static uint32_t last_sensor_read = 0;
static uint32_t last_pir_read = 0;
static uint32_t last_alert_check = 0;
static uint32_t lock_timer = 0;
static uint8_t lock_auto_close = 0;

static uint16_t stepper_current_step = 0;
static uint16_t stepper_target_step = 0;
#define STEPPER_MAX_STEPS 2048

static const uint8_t stepper_seq[8][4] = {
    {1,0,0,0}, {1,1,0,0}, {0,1,0,0}, {0,1,1,0},
    {0,0,1,0}, {0,0,1,1}, {0,0,0,1}, {1,0,0,1}
};
static uint8_t stepper_seq_idx = 0;

/* ============ 延时函数 ============ */

void Delay_ms(uint32_t ms)
{
    uint32_t start = sys_tick_ms;
    while ((sys_tick_ms - start) < ms);
}

uint32_t GetTick(void)
{
    return sys_tick_ms;
}

void SysTick_Handler(void)
{
    sys_tick_ms++;
}

/* ============ 时钟配置 ============ */

static void RCC_Configuration(void)
{
    ErrorStatus HSEStartUpStatus;

    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus == SUCCESS) {
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        FLASH_SetLatency(FLASH_Latency_2);

        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        RCC_PCLK2Config(RCC_HCLK_Div1);
        RCC_PCLK1Config(RCC_HCLK_Div2);
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
        RCC_PLLCmd(ENABLE);
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        while (RCC_GetSYSCLKSource() != 0x08);
    }

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

/* ============ GPIO 初始化 ============ */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;

    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;

    gpio.GPIO_Pin = RELAY_LIGHT_GPIO_PIN;
    GPIO_Init(RELAY_LIGHT_GPIO_PORT, &gpio);
    GPIO_ResetBits(RELAY_LIGHT_GPIO_PORT, RELAY_LIGHT_GPIO_PIN);

    gpio.GPIO_Pin = RELAY_AC_GPIO_PIN;
    GPIO_Init(RELAY_AC_GPIO_PORT, &gpio);
    GPIO_ResetBits(RELAY_AC_GPIO_PORT, RELAY_AC_GPIO_PIN);

    gpio.GPIO_Pin = RELAY_LOCK_GPIO_PIN;
    GPIO_Init(RELAY_LOCK_GPIO_PORT, &gpio);
    GPIO_ResetBits(RELAY_LOCK_GPIO_PORT, RELAY_LOCK_GPIO_PIN);

    gpio.GPIO_Pin = LED_LIVING_GPIO_PIN;
    GPIO_Init(LED_LIVING_GPIO_PORT, &gpio);
    GPIO_ResetBits(LED_LIVING_GPIO_PORT, LED_LIVING_GPIO_PIN);

    gpio.GPIO_Pin = LED_BEDROOM_GPIO_PIN;
    GPIO_Init(LED_BEDROOM_GPIO_PORT, &gpio);
    GPIO_ResetBits(LED_BEDROOM_GPIO_PORT, LED_BEDROOM_GPIO_PIN);

    gpio.GPIO_Pin = LED_KITCHEN_GPIO_PIN;
    GPIO_Init(LED_KITCHEN_GPIO_PORT, &gpio);
    GPIO_ResetBits(LED_KITCHEN_GPIO_PORT, LED_KITCHEN_GPIO_PIN);

    gpio.GPIO_Pin = LED_DEBUG_GPIO_PIN;
    GPIO_Init(LED_DEBUG_GPIO_PORT, &gpio);
    GPIO_SetBits(LED_DEBUG_GPIO_PORT, LED_DEBUG_GPIO_PIN);

    gpio.GPIO_Pin = STEPPER_PIN1 | STEPPER_PIN2 | STEPPER_PIN3 | STEPPER_PIN4;
    GPIO_Init(STEPPER_PORT, &gpio);
    GPIO_ResetBits(STEPPER_PORT, STEPPER_PIN1 | STEPPER_PIN2 | STEPPER_PIN3 | STEPPER_PIN4);

    gpio.GPIO_Pin = DHT11_GPIO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(DHT11_GPIO_PORT, &gpio);

    gpio.GPIO_Pin = PIR_GPIO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(PIR_GPIO_PORT, &gpio);

    gpio.GPIO_Pin = BUZZER_GPIO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(BUZZER_GPIO_PORT, &gpio);

    gpio.GPIO_Pin = IR_GPIO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(IR_GPIO_PORT, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);
}

/* ============ USART1 初始化 ============ */

static void MX_USART1_Init(void)
{
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(ESP8266_USART, &usart);
    USART_Cmd(ESP8266_USART, ENABLE);

    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

/* ============ TIM2 初始化 ============ */

static void MX_TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;

    tim.TIM_Prescaler = 71;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 65535;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &tim);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init(TIM2, &oc);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

/* ============ 蜂鸣器 ============ */

void buzzer_beep(uint16_t ms)
{
    TIM_Cmd(BUZZER_TIM, DISABLE);
    BUZZER_TIM->PSC = 71;
    BUZZER_TIM->ARR = 999;
    TIM_SetCompare4(BUZZER_TIM, 500);
    TIM_Cmd(BUZZER_TIM, ENABLE);
    Delay_ms(ms);
    TIM_Cmd(BUZZER_TIM, DISABLE);
    BUZZER_TIM->PSC = 71;
    BUZZER_TIM->ARR = 65535;
    TIM_SetCompare4(BUZZER_TIM, 0);
    TIM_Cmd(BUZZER_TIM, ENABLE);
}

/* ============ 步进电机 ============ */

static void stepper_write_pins(const uint8_t *s)
{
    GPIO_WriteBit(STEPPER_PORT, STEPPER_PIN1, s[0] ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(STEPPER_PORT, STEPPER_PIN2, s[1] ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(STEPPER_PORT, STEPPER_PIN3, s[2] ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(STEPPER_PORT, STEPPER_PIN4, s[3] ? Bit_SET : Bit_RESET);
}

void stepper_set_position(uint8_t percent)
{
    if (percent > 100) percent = 100;
    stepper_target_step = (uint16_t)((uint32_t)percent * STEPPER_MAX_STEPS / 100);
}

static void stepper_run_step(uint8_t forward)
{
    if (forward) {
        stepper_current_step++;
        stepper_seq_idx = (stepper_seq_idx + 1) & 7;
    } else {
        if (stepper_current_step > 0) stepper_current_step--;
        stepper_seq_idx = (stepper_seq_idx - 1) & 7;
    }
    stepper_write_pins(stepper_seq[stepper_seq_idx]);
}

static void stepper_update(void)
{
    if (stepper_current_step == stepper_target_step) return;
    uint8_t forward = stepper_current_step < stepper_target_step;
    stepper_run_step(forward);
    Delay_ms(2);
}

/* ============ 传感器读取 ============ */

static void read_sensor(void)
{
    DHT11_Data dht = {0};
    if (DHT11_Read(&dht) && dht.valid) {
        g_temperature = dht.temperature;
        g_humidity = dht.humidity;
    }
}

/* ============ 安防检测 ============ */

static void check_alert(void)
{
    if (!g_system_armed) return;

    if (GPIO_ReadInputDataBit(PIR_GPIO_PORT, PIR_GPIO_PIN) == Bit_SET) {
        buzzer_beep(1000);
        g_alert_active = 1;
    }
}

/* ============ 红外遥控检测 ============ */

static void check_ir_remote(void)
{
    IR_Code code;
    if (IR_GetCode(&code) && code.valid) {
        if (IR_CheckAuthorized(code.address, code.command)) {
            GPIO_SetBits(RELAY_LOCK_GPIO_PORT, RELAY_LOCK_GPIO_PIN);
            g_lock_on = 1;
            lock_timer = GetTick();
            lock_auto_close = 1;
        }
    }
}

/* ============ 门锁自动关闭 ============ */

static void check_lock_auto(void)
{
    if (lock_auto_close && (GetTick() - lock_timer > 5000)) {
        GPIO_ResetBits(RELAY_LOCK_GPIO_PORT, RELAY_LOCK_GPIO_PIN);
        g_lock_on = 0;
        lock_auto_close = 0;
    }
}

/* ============ 主函数 ============ */

int main(void)
{
    RCC_Configuration();
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);

    MX_GPIO_Init();
    MX_USART1_Init();
    MX_TIM2_Init();

    ESP8266_Init();
    IR_Init();
    IR_AddAuthorizedCode(0x00FF, 0xA25D);

    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_Cmd(TIM2, ENABLE);

    Delay_ms(1000);

    buzzer_beep(200);
    Delay_ms(500);

    ESP8266_StartAP();
    buzzer_beep(200);

    /* 启动时LED亮2秒（确认GPIO正常） */
    GPIO_ResetBits(LED_LIVING_GPIO_PORT, LED_LIVING_GPIO_PIN);
    GPIO_ResetBits(LED_BEDROOM_GPIO_PORT, LED_BEDROOM_GPIO_PIN);
    GPIO_ResetBits(LED_KITCHEN_GPIO_PORT, LED_KITCHEN_GPIO_PIN);
    GPIO_ResetBits(LED_DEBUG_GPIO_PORT, LED_DEBUG_GPIO_PIN);
    Delay_ms(2000);
    GPIO_SetBits(LED_LIVING_GPIO_PORT, LED_LIVING_GPIO_PIN);
    GPIO_SetBits(LED_BEDROOM_GPIO_PORT, LED_BEDROOM_GPIO_PIN);
    GPIO_SetBits(LED_KITCHEN_GPIO_PORT, LED_KITCHEN_GPIO_PIN);
    GPIO_SetBits(LED_DEBUG_GPIO_PORT, LED_DEBUG_GPIO_PIN);
    Delay_ms(500);

    /* OLED UI */
    OLED_Init();
    OLED_ShowString(28, 0, "SmartHome", 16);
    OLED_ShowString(0, 16, "Temp:", 16);
    OLED_ShowString(0, 32, "Humi:", 16);
    OLED_Update();

    while (1) {
        uint32_t now = GetTick();
        uint8_t cmd_processed = 0;  /* 命令处理标志 */

        /* 心跳指示：调试LED每500ms闪烁一次 */
        static uint32_t last_heartbeat = 0;
        static uint8_t heartbeat_state = 0;
        if (now - last_heartbeat > 500) {
            last_heartbeat = now;
            heartbeat_state = !heartbeat_state;
            if (heartbeat_state) {
                GPIO_ResetBits(LED_DEBUG_GPIO_PORT, LED_DEBUG_GPIO_PIN);
            } else {
                GPIO_SetBits(LED_DEBUG_GPIO_PORT, LED_DEBUG_GPIO_PIN);
            }
        }

        /* 处理来自APP的TCP命令（直接在缓冲区搜索） */
        {
            extern uint8_t esp_rx_buf[];
            extern volatile uint16_t esp_rx_idx;
            extern volatile uint8_t esp_rx_complete;

            if (esp_rx_complete) {
                /* 在缓冲区中搜索LED命令 */
                if (strstr((char *)esp_rx_buf, "living_on") != NULL) {
                    GPIO_ResetBits(LED_LIVING_GPIO_PORT, LED_LIVING_GPIO_PIN);
                    g_led_living = 1;
                    cmd_processed = 1;
                } else if (strstr((char *)esp_rx_buf, "living_off") != NULL) {
                    GPIO_SetBits(LED_LIVING_GPIO_PORT, LED_LIVING_GPIO_PIN);
                    g_led_living = 0;
                    cmd_processed = 1;
                } else if (strstr((char *)esp_rx_buf, "bedroom_on") != NULL) {
                    GPIO_ResetBits(LED_BEDROOM_GPIO_PORT, LED_BEDROOM_GPIO_PIN);
                    g_led_bedroom = 1;
                    cmd_processed = 1;
                } else if (strstr((char *)esp_rx_buf, "bedroom_off") != NULL) {
                    GPIO_SetBits(LED_BEDROOM_GPIO_PORT, LED_BEDROOM_GPIO_PIN);
                    g_led_bedroom = 0;
                    cmd_processed = 1;
                } else if (strstr((char *)esp_rx_buf, "kitchen_on") != NULL) {
                    GPIO_ResetBits(LED_KITCHEN_GPIO_PORT, LED_KITCHEN_GPIO_PIN);
                    g_led_kitchen = 1;
                    cmd_processed = 1;
                } else if (strstr((char *)esp_rx_buf, "kitchen_off") != NULL) {
                    GPIO_SetBits(LED_KITCHEN_GPIO_PORT, LED_KITCHEN_GPIO_PIN);
                    g_led_kitchen = 0;
                    cmd_processed = 1;
                } else if (strstr((char *)esp_rx_buf, "all_on") != NULL) {
                    GPIO_ResetBits(LED_LIVING_GPIO_PORT, LED_LIVING_GPIO_PIN);
                    GPIO_ResetBits(LED_BEDROOM_GPIO_PORT, LED_BEDROOM_GPIO_PIN);
                    GPIO_ResetBits(LED_KITCHEN_GPIO_PORT, LED_KITCHEN_GPIO_PIN);
                    g_led_living = 1;
                    g_led_bedroom = 1;
                    g_led_kitchen = 1;
                    cmd_processed = 1;
                } else if (strstr((char *)esp_rx_buf, "all_off") != NULL) {
                    GPIO_SetBits(LED_LIVING_GPIO_PORT, LED_LIVING_GPIO_PIN);
                    GPIO_SetBits(LED_BEDROOM_GPIO_PORT, LED_BEDROOM_GPIO_PIN);
                    GPIO_SetBits(LED_KITCHEN_GPIO_PORT, LED_KITCHEN_GPIO_PIN);
                    g_led_living = 0;
                    g_led_bedroom = 0;
                    g_led_kitchen = 0;
                    cmd_processed = 1;
                }

                /* 清除缓冲区 */
                esp_rx_complete = 0;
                esp_rx_idx = 0;
                memset(esp_rx_buf, 0, ESP_RX_BUF_SIZE);

                /* 命令处理后立即发送状态确认 */
                if (cmd_processed) {
                    uint8_t pir = GPIO_ReadInputDataBit(PIR_GPIO_PORT, PIR_GPIO_PIN);
                    char ack[48];
                    snprintf(ack, sizeof(ack), "%d,%d,%s,%d,%d,%d\r\n",
                        g_temperature, g_humidity, pir ? "y" : "n",
                        g_led_living, g_led_bedroom, g_led_kitchen);
                    ESP8266_TCPSend(0, ack);
                }
            }
        }

        if (now - last_sensor_read > 2000) {
            last_sensor_read = now;
            read_sensor();

            char line[17];
            OLED_ShowString(56, 16, "       ", 16);
            snprintf(line, sizeof(line), "%d C", g_temperature);
            OLED_ShowString(56, 16, (const uint8_t *)line, 16);
            OLED_ShowString(56, 32, "       ", 16);
            snprintf(line, sizeof(line), "%d %%", g_humidity);
            OLED_ShowString(56, 32, (const uint8_t *)line, 16);
            OLED_Update();

            /* TCP 推送传感器数据 (通道0) - 仅在没有待处理命令时发送 */
            extern uint8_t esp_rx_buf[];
            extern volatile uint16_t esp_rx_idx;
            if (esp_rx_idx == 0) {
                uint8_t pir = GPIO_ReadInputDataBit(PIR_GPIO_PORT, PIR_GPIO_PIN);
                char tcp[48];
                snprintf(tcp, sizeof(tcp), "%d,%d,%s,%d,%d,%d\r\n",
                    g_temperature, g_humidity, pir ? "y" : "n",
                    g_led_living, g_led_bedroom, g_led_kitchen);
                ESP8266_TCPSend(0, tcp);
            }
        }

        if (now - last_pir_read > 1000) {
            last_pir_read = now;
            char line[17];
            uint8_t pir = GPIO_ReadInputDataBit(PIR_GPIO_PORT, PIR_GPIO_PIN);
            OLED_ShowString(0, 48, "                ", 16);
            snprintf(line, sizeof(line), "PIR:%s", pir ? "y" : "n");
            OLED_ShowString(0, 48, (const uint8_t *)line, 16);
            OLED_Update();
        }

        if (now - last_alert_check > 500) {
            last_alert_check = now;
            check_alert();
        }

        stepper_update();
        check_ir_remote();
        check_lock_auto();
    }
}
