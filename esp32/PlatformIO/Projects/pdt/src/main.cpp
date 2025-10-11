#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_log.h>
#include <errno.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <cstring>
#include <cstdlib>
#include <cinttypes>
#include <inttypes.h>

namespace
{
    // Queue for strings
    QueueHandle_t m_string_queue = nullptr;
    const constexpr int MAX_COUNT{ 1000 }; // optional safety cap (not enforced here)
    const constexpr char *TAG{ "app" };

    // UART settings - use USB-serial console
    const uart_port_t UART_NUM = UART_NUM_0; // console port
    const int UART_BAUD = 115200;
    const int TXD_PIN = UART_PIN_NO_CHANGE;
    const int RXD_PIN = UART_PIN_NO_CHANGE;
    const int BUF_SIZE = 256; // max token / buffer size

    void producer(void *p);
    void consumer(void *p);

    // Helper: init UART
    void init_uart()
    {
        // zero-init so all fields are deterministic (important!)
        uart_config_t uart_config{};
        uart_config.baud_rate = UART_BAUD;
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.parity    = UART_PARITY_DISABLE;
        uart_config.stop_bits = UART_STOP_BITS_1;
        uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE; // we don't use HW flow control here

        // When flow_ctrl is disabled this can be 0, but initialize it explicitly to be safe:
        uart_config.rx_flow_ctrl_thresh = 0;

        uart_config.source_clk = UART_SCLK_APB;

        ESP_ERROR_CHECK( uart_param_config(UART_NUM, &uart_config) );
        ESP_ERROR_CHECK( uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );

        const int DRIVER_RX_BUF = 1024;
        const int DRIVER_TX_BUF = 0;
        ESP_ERROR_CHECK( uart_driver_install(UART_NUM, DRIVER_RX_BUF, DRIVER_TX_BUF, 0, nullptr, 0) );
    }

    //
    // --- MOTOR / PWM configuration (DRV8871) ---
    //
    // Pins to connect to IN1 / IN2 of DRV8871
    // Escoge pines que no colisionen con tus periféricos; 18/19 son ejemplo común.
    const gpio_num_t IN1_PIN = GPIO_NUM_18;
    const gpio_num_t IN2_PIN = GPIO_NUM_19;

    // ledc (PWM) settings
    const int PWM_FREQ = 20000;          // 20 kHz (según datasheet: 0-200kHz, 20kHz buen compromiso)
    const int PWM_RES_BITS = 8;          // 8-bit resolution -> duty 0..255
    const int PWM_MAX_DUTY = (1 << PWM_RES_BITS) - 1; // 255

    // ledc timer and channels
    const ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;
    const ledc_mode_t LEDC_MODE = LEDC_HIGH_SPEED_MODE;
    const int LEDC_TIMER_BIT = PWM_RES_BITS;

    // canales: uno por pin
    const ledc_channel_t CHANNEL_IN1 = LEDC_CHANNEL_0;
    const ledc_channel_t CHANNEL_IN2 = LEDC_CHANNEL_1;

    // forward declarations
    void init_motor_pwm();
    void move_DC_motor(int rpm); // rpm param used as arbitrary signed speed (see doc below)

} // end anonymous namespace

extern "C" void app_main()
{
    ESP_LOGI(TAG, "application started");

    // init motor PWM BEFORE creating tasks (so move_DC_motor is ready)
    init_motor_pwm();

    // create queue (5 elements, each of size BUF_SIZE bytes for a string)
    m_string_queue = xQueueCreate(5, BUF_SIZE);
    if (m_string_queue == nullptr) {
        ESP_LOGE(TAG, "failed to create queue");
        return;
    }

    // init UART (producer will read from it)
    init_uart();

    // create tasks
    xTaskCreate(producer, "producer", 4096, nullptr, 5, nullptr);
    xTaskCreate(consumer, "consumer-0", 4096, (void *)0, 5, nullptr);


    // print running tasks (optional)
    char buffer[256]{0};
    vTaskList(buffer);
    ESP_LOGI(TAG, "\n%s", buffer);
} // end of app_main

// Empty placeholder functions for steppers

void move_stepper1(int steps) {
    // TODO: implement Stepper 1 control
    ESP_LOGI("motor", "move_stepper1 called with steps=%d", steps);
}

void move_stepper2(int steps) {
    // TODO: implement Stepper 2 control
    ESP_LOGI("motor", "move_stepper2 called with steps=%d", steps);
}

void move_stepper3(int steps) {
    // TODO: implement Stepper 3 control
    ESP_LOGI("motor", "move_stepper3 called with steps=%d", steps);
}

// --- MOTOR / PWM implementation ---
namespace
{
    void init_motor_pwm()
    {
        static bool initialized = false;
        if (initialized) return;

        // configure timer
        ledc_timer_config_t timer_cfg{};
        timer_cfg.speed_mode = LEDC_MODE;
        timer_cfg.timer_num = LEDC_TIMER;
        timer_cfg.duty_resolution = (ledc_timer_bit_t)LEDC_TIMER_BIT;
        timer_cfg.freq_hz = PWM_FREQ;
        timer_cfg.clk_cfg = LEDC_AUTO_CLK;
        ESP_ERROR_CHECK( ledc_timer_config(&timer_cfg) );

        // configure channel for IN1
        ledc_channel_config_t ch1_cfg{};
        ch1_cfg.speed_mode = LEDC_MODE;
        ch1_cfg.channel = CHANNEL_IN1;
        ch1_cfg.timer_sel = LEDC_TIMER;
        ch1_cfg.intr_type = LEDC_INTR_DISABLE;
        ch1_cfg.gpio_num = (int)IN1_PIN;
        ch1_cfg.duty = 0;
        ch1_cfg.hpoint = 0;
        ESP_ERROR_CHECK( ledc_channel_config(&ch1_cfg) );

        // configure channel for IN2
        ledc_channel_config_t ch2_cfg{};
        ch2_cfg.speed_mode = LEDC_MODE;
        ch2_cfg.channel = CHANNEL_IN2;
        ch2_cfg.timer_sel = LEDC_TIMER;
        ch2_cfg.intr_type = LEDC_INTR_DISABLE;
        ch2_cfg.gpio_num = (int)IN2_PIN;
        ch2_cfg.duty = 0;
        ch2_cfg.hpoint = 0;
        ESP_ERROR_CHECK( ledc_channel_config(&ch2_cfg) );

        // ensure pins are outputs (ledc config already sets them, but keep explicit)
        gpio_set_direction(IN1_PIN, GPIO_MODE_OUTPUT);
        gpio_set_direction(IN2_PIN, GPIO_MODE_OUTPUT);

        // start with motor stopped
        ESP_ERROR_CHECK( ledc_set_duty(LEDC_MODE, CHANNEL_IN1, 0) );
        ESP_ERROR_CHECK( ledc_update_duty(LEDC_MODE, CHANNEL_IN1) );
        ESP_ERROR_CHECK( ledc_set_duty(LEDC_MODE, CHANNEL_IN2, 0) );
        ESP_ERROR_CHECK( ledc_update_duty(LEDC_MODE, CHANNEL_IN2) );

        initialized = true;
        ESP_LOGI(TAG, "motor PWM initialized: freq=%d Hz, res=%d bits, pins IN1=%d IN2=%d",
                 PWM_FREQ, PWM_RES_BITS, (int)IN1_PIN, (int)IN2_PIN);
    }

    // move_DC_motor: interpreta 'rpm' como velocidad arbitraria firmada
    // - if |rpm| <= 100 -> se interpreta como porcentaje 0..100 %
    // - else -> se interpreta como valor directo y se saturará a 255
    //
    // Sign:
    //   rpm > 0  => forward  (IN1 = 0, PWM en IN2)
    //   rpm < 0  => reverse  (IN2 = 0, PWM en IN1)
    //   rpm == 0 => coast (ambos 0)
    void move_DC_motor(int rpm) {
        // normalize and compute duty
        int abs_val = rpm >= 0 ? rpm : -rpm;
        int duty = 0;

        if (abs_val <= 100) {
            // interpret as percent
            duty = (abs_val * PWM_MAX_DUTY) / 100;
        } else {
            // interpret as direct value, clamp to [0..PWM_MAX_DUTY]
            duty = abs_val;
            if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
        }

        if (rpm == 0) {
            // coast: both outputs low (High-Z behavior from driver when inputs low)
            ESP_LOGI("motor", "move_DC_motor: coast (rpm=0) -> duty=0");
            ESP_ERROR_CHECK( ledc_set_duty(LEDC_MODE, CHANNEL_IN1, 0) );
            ESP_ERROR_CHECK( ledc_update_duty(LEDC_MODE, CHANNEL_IN1) );
            ESP_ERROR_CHECK( ledc_set_duty(LEDC_MODE, CHANNEL_IN2, 0) );
            ESP_ERROR_CHECK( ledc_update_duty(LEDC_MODE, CHANNEL_IN2) );
            return;
        }

        if (rpm > 0) {
            // forward: IN1 = 0, PWM on IN2
            ESP_LOGI("motor", "move_DC_motor: forward rpm=%d -> duty=%d", rpm, duty);
            // ensure IN1 = 0 (no PWM)
            ESP_ERROR_CHECK( ledc_set_duty(LEDC_MODE, CHANNEL_IN1, 0) );
            ESP_ERROR_CHECK( ledc_update_duty(LEDC_MODE, CHANNEL_IN1) );

            // set PWM on IN2
            ESP_ERROR_CHECK( ledc_set_duty(LEDC_MODE, CHANNEL_IN2, duty) );
            ESP_ERROR_CHECK( ledc_update_duty(LEDC_MODE, CHANNEL_IN2) );
        } else {
            // reverse: IN2 = 0, PWM on IN1
            ESP_LOGI("motor", "move_DC_motor: reverse rpm=%d -> duty=%d", rpm, duty);
            // ensure IN2 = 0 (no PWM)
            ESP_ERROR_CHECK( ledc_set_duty(LEDC_MODE, CHANNEL_IN2, 0) );
            ESP_ERROR_CHECK( ledc_update_duty(LEDC_MODE, CHANNEL_IN2) );

            // set PWM on IN1
            ESP_ERROR_CHECK( ledc_set_duty(LEDC_MODE, CHANNEL_IN1, duty) );
            ESP_ERROR_CHECK( ledc_update_duty(LEDC_MODE, CHANNEL_IN1) );
        }
    }

} // end anonymous namespace

namespace
{
    void producer(void *p)
    {
        // Buffer for incoming bytes
        uint8_t data_buf[BUF_SIZE];
        size_t read_pos = 0;

        // We'll read bytes continuously and accumulate until newline, then extract the token.
        while (true)
        {
            // read available bytes (non-blocking with small timeout)
            int len = uart_read_bytes(UART_NUM, data_buf + read_pos, BUF_SIZE - 1 - read_pos, pdMS_TO_TICKS(100));
            if (len > 0) {
                read_pos += len;
                // search for newline(s)
                for (size_t i = 0; i < read_pos; ++i) {
                    if (data_buf[i] == '\n' || data_buf[i] == '\r') {
                        // found end of line -> extract token from start to i (skip empty lines)
                        if (i == 0) {
                            // shift buffer left, remove this newline
                            memmove(data_buf, data_buf + 1, read_pos - 1);
                            --read_pos;
                            --i;
                            continue;
                        }

                        // create null-terminated string token
                        size_t token_len = i;
                        char token[BUF_SIZE];
                        // ensure not to overflow (token_len < BUF_SIZE by design)
                        if (token_len >= static_cast<size_t>(BUF_SIZE)) {
                            token_len = BUF_SIZE - 1;
                        }
                        memcpy(token, data_buf, token_len);
                        token[token_len] = '\0';

                        // shift remaining bytes to front (skip the newline and possible \r\n)
                        size_t shift_start = i + 1;
                        // handle \r\n: if next char exists and is the complementary newline, skip it too
                        if (shift_start < read_pos && (data_buf[shift_start] == '\n' || data_buf[shift_start] == '\r')) {
                            ++shift_start;
                        }
                        size_t new_read_pos = read_pos - shift_start;
                        memmove(data_buf, data_buf + shift_start, new_read_pos);
                        read_pos = new_read_pos;

                        // send token string to queue (blocking if queue full)
                        if (xQueueSendToBack(m_string_queue, token, pdMS_TO_TICKS(1000)) == pdPASS) {
                            ESP_LOGI(TAG, "p: received '%s' and enqueued", token);
                        } else {
                            ESP_LOGW(TAG, "p: queue full, dropped '%s'", token);
                        }

                        // restart outer for loop from beginning (since buffer changed)
                        i = (size_t)-1; // will become 0 after ++ in for loop
                    }
                }
            }

            // small delay to avoid busy loop when nothing arrives
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // never reached, but good practice
        vTaskDelete(nullptr);
    } // end of producer

void consumer(void *param) {
    int cpu_id = (int)(intptr_t)param;
    char recv_buf[BUF_SIZE];

    while (true) {
        if (xQueueReceive(m_string_queue, recv_buf, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "c%d: '%s'", cpu_id, recv_buf);

            // Variables to hold the parsed values
            int DC_rpm = 0;
            int Stepper1_steps = 0;
            int Stepper2_steps = 0;
            int Stepper3_steps = 0;

            // Parse the string
            int parsed = sscanf(recv_buf, "%d,%d,%d,%d",
                                &DC_rpm, &Stepper1_steps, &Stepper2_steps, &Stepper3_steps);

            if (parsed == 4) {
                ESP_LOGI(TAG, "Parsed: DC=%d, S1=%d, S2=%d, S3=%d",
                         DC_rpm, Stepper1_steps, Stepper2_steps, Stepper3_steps);

                // Example: call functions using parsed values
                move_DC_motor(DC_rpm);
                move_stepper1(Stepper1_steps);
                move_stepper2(Stepper2_steps);
                move_stepper3(Stepper3_steps);
            } else {
                ESP_LOGW(TAG, "Failed to parse message: '%s'", recv_buf);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2)); // let other tasks run
    }
}

} // end of anonymous namespace
