
// V1 PRUEBA REALIZADA EN CASA NUEVA ANDALUCIA, FUNCIONA CORRECTAMENTE,
// HACE FALTA HACER PRUEBAS CERCA DE ANTENA LORA
// ENVIO DE DATOS DE LA ULTIMA MEDICIONA ANTEN DE INIT_JOIN()
// ANTES DE HACER EL INIT_JOIN CON TTN HACE 6 MEDICIONES, ES DECIR EL CONTADOR ESTA A 5
// ESTE SE VEN EN EL TERMINAL:
/*
I (341) main_task: Started on CPU0
I (351) main_task: Calling app_main()
I (351) _i2c_init: Initializing I2C Bus.......
I (351) _i2c_init: I2C Bus Initialized, ESP_OK
I (661) OLED: OLED configured successfully
I (15761) dht20_read_task: Entering measerument loop
I (15761) LoRa: Tareas inicializadas
I (15761) main_task: Returned from app_main()
I (15861) dht20_read_task: is calibrated....
I (16041) dht20_read_task: Temperature: 22.8C.   Avg: 2.3C
I (16041) dht20_read_task: Humidity:    73.6%.   Avg: 7.4%

I (16161) SGP30: CO2eq: 400 ppm, TVOC: 0 ppb
I (16171) DEPURACION: CONTADOR = 1
I (18271) dht20_read_task: is calibrated....
I (18451) dht20_read_task: Temperature: 22.8C.   Avg: 4.6C
I (18451) dht20_read_task: Humidity:    73.6%.   Avg: 14.7%

I (18571) SGP30: CO2eq: 400 ppm, TVOC: 0 ppb
I (18581) DEPURACION: CONTADOR = 2
I (20681) dht20_read_task: is calibrated....
I (20861) dht20_read_task: Temperature: 22.8C.   Avg: 6.8C
I (20861) dht20_read_task: Humidity:    73.6%.   Avg: 22.1%

I (20981) SGP30: CO2eq: 408 ppm, TVOC: 5 ppb
I (20991) DEPURACION: CONTADOR = 3
I (23091) dht20_read_task: is calibrated....
I (23271) dht20_read_task: Temperature: 22.8C.   Avg: 9.1C
I (23271) dht20_read_task: Humidity:    73.6%.   Avg: 29.4%

I (23391) SGP30: CO2eq: 409 ppm, TVOC: 10 ppb
I (23401) DEPURACION: CONTADOR = 4
I (25501) dht20_read_task: is calibrated....
I (25681) dht20_read_task: Temperature: 22.8C.   Avg: 11.4C
I (25681) dht20_read_task: Humidity:    73.6%.   Avg: 36.8%

I (25801) SGP30: CO2eq: 407 ppm, TVOC: 17 ppb
I (25811) DEPURACION: CONTADOR = 5
I (27911) dht20_read_task: is calibrated....
I (28091) dht20_read_task: Temperature: 22.8C.   Avg: 13.7C
I (28091) dht20_read_task: Humidity:    73.5%.   Avg: 44.1%

I (28211) SGP30: CO2eq: 403 ppm, TVOC: 17 ppb
I (28231) ttn_prov: DevEUI, AppEUI/JoinEUI and AppKey saved in NVS storage
Joining...
I (28241) gpio: GPIO[5]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 
I (28241) gpio: GPIO[16]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 
I (28251) gpio: GPIO[2]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 0| Pulldown: 1| Intr:1 
I (28261) gpio: GPIO[15]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 0| Pulldown: 1| Intr:1 
I (28271) ttn_hal: IO initialized
I (28271) ttn_hal: SPI initialized
I (28281) ttn_hal: Timer initialized
I (28321) ttn: event EV_JOINING
I (35501) ttn: event EV_TXSTART
I (40531) ttn: event EV_RXSTART
I (41531) ttn: event EV_RXSTART
I (41801) ttn: event EV_JOIN_TXCOMPLETE
Join failed.
I (41801) DEPURACION: CONTADOR = 1
I (43901) dht20_read_task: is calibrated....
I (44081) dht20_read_task: Temperature: 22.8C.   Avg: 16.0C
I (44081) dht20_read_task: Humidity:    73.4%.   Avg: 51.5%

I (44201) SGP30: CO2eq: 411 ppm, TVOC: 19 ppb
I (44211) DEPURACION: CONTADOR = 2
I (46311) dht20_read_task: is calibrated....
I (46491) dht20_read_task: Temperature: 22.8C.   Avg: 18.2C
I (46491) dht20_read_task: Humidity:    73.2%.   Avg: 58.8%

I (46611) SGP30: CO2eq: 400 ppm, TVOC: 17 ppb
I (46621) DEPURACION: CONTADOR = 3
I (48721) dht20_read_task: is calibrated....
I (48901) dht20_read_task: Temperature: 22.8C.   Avg: 20.5C
I (48901) dht20_read_task: Humidity:    73.2%.   Avg: 66.1%

I (49021) SGP30: CO2eq: 435 ppm, TVOC: 20 ppb
I (49031) DEPURACION: CONTADOR = 4
I (51131) dht20_read_task: is calibrated....
I (51311) dht20_read_task: Temperature: 22.8C.   Avg: 22.8C
I (51311) dht20_read_task: Humidity:    73.2%.   Avg: 73.4%

I (51431) SGP30: CO2eq: 401 ppm, TVOC: 17 ppb
I (51441) DEPURACION: CONTADOR = 5
I (53541) dht20_read_task: is calibrated....
I (53721) dht20_read_task: Temperature: 22.8C.   Avg: 22.8C
I (53721) dht20_read_task: Humidity:    73.1%.   Avg: 73.4%

I (53841) SGP30: CO2eq: 403 ppm, TVOC: 16 ppb
Joining...
I (53851) ttn: event EV_JOINING
I (59511) ttn: event EV_TXSTART
I (64551) ttn: event EV_RXSTART
I (65541) ttn: event EV_RXSTART
I (65811) ttn: event EV_JOIN_TXCOMPLETE
Join failed.
I (65821) DEPURACION: CONTADOR = 1
I (67921) dht20_read_task: is calibrated....
I (68101) dht20_read_task: Temperature: 22.9C.   Avg: 22.8C
I (68101) dht20_read_task: Humidity:    73.0%.   Avg: 73.3%

I (68221) SGP30: CO2eq: 408 ppm, TVOC: 25 ppb
I (68231) DEPURACION: CONTADOR = 2
I (70331) dht20_read_task: is calibrated....
I (70511) dht20_read_task: Temperature: 22.9C.   Avg: 22.8C
I (70511) dht20_read_task: Humidity:    73.0%.   Avg: 73.3%

I (70631) SGP30: CO2eq: 417 ppm, TVOC: 27 ppb
I (70641) DEPURACION: CONTADOR = 3
I (72741) dht20_read_task: is calibrated....
I (72921) dht20_read_task: Temperature: 22.9C.   Avg: 22.8C
I (72921) dht20_read_task: Humidity:    73.0%.   Avg: 73.2%

I (73041) SGP30: CO2eq: 411 ppm, TVOC: 23 ppb
I (73051) DEPURACION: CONTADOR = 4
I (75151) dht20_read_task: is calibrated....
I (75331) dht20_read_task: Temperature: 22.9C.   Avg: 22.8C
I (75331) dht20_read_task: Humidity:    72.9%.   Avg: 73.2%

I (75451) SGP30: CO2eq: 403 ppm, TVOC: 27 ppb
I (75461) DEPURACION: CONTADOR = 5
I (77561) dht20_read_task: is calibrated....
I (77741) dht20_read_task: Temperature: 22.9C.   Avg: 22.8C
I (77741) dht20_read_task: Humidity:    72.9%.   Avg: 73.1%

I (77861) SGP30: CO2eq: 410 ppm, TVOC: 30 ppb
Joining...
I (77871) ttn: event EV_JOINING
I (84111) ttn: event EV_TXSTART
I (89151) ttn: event EV_RXSTART
I (90141) ttn: event EV_RXSTART
I (90421) ttn: event EV_JOIN_TXCOMPLETE
Join failed.
I (90421) DEPURACION: CONTADOR = 1
I (92521) dht20_read_task: is calibrated....
I (92701) dht20_read_task: Temperature: 22.9C.   Avg: 22.9C
I (92701) dht20_read_task: Humidity:    72.8%.   Avg: 73.0%

I (92821) SGP30: CO2eq: 410 ppm, TVOC: 40 ppb
I (92831) DEPURACION: CONTADOR = 2
I (94931) dht20_read_task: is calibrated....
I (95111) dht20_read_task: Temperature: 22.9C.   Avg: 22.9C
I (95111) dht20_read_task: Humidity:    72.8%.   Avg: 73.0%

I (95231) SGP30: CO2eq: 427 ppm, TVOC: 43 ppb
*/

// Se ha construido correctamente el dia 14/11/2024 a las 16:58


#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "dht20.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h" // generated by "make menuconfig"
#include "ssd1306.h"
#include "font8x8_basic.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "ttn.h"
#include "esp_timer.h" // Para medir el tiempo

// AppEUI, DevEUI y AppKey
const char *appEui = "0000000000000000";
const char *devEui = "70B3D57ED006B4C5";
const char *appKey = "528ED36701C123C3954535625253BFF2";

// Pines y configuración de recursos
#define TTN_SPI_HOST      SPI2_HOST
#define TTN_SPI_DMA_CHAN  SPI_DMA_DISABLED
#define TTN_PIN_SPI_SCLK  18
#define TTN_PIN_SPI_MOSI  23
#define TTN_PIN_SPI_MISO  19
#define TTN_PIN_NSS       5
#define TTN_PIN_RXTX      TTN_NOT_CONNECTED
#define TTN_PIN_RST       16
#define TTN_PIN_DIO0      2
#define TTN_PIN_DIO1      15
#define TX_INTERVAL 30

// SPG30
#define SGP30_ADDR 0x58  // Dirección I2C del SGP30
#define OLED_I2C_ADDRESS 0x3C // Direccion I2C del ssd1306
#define ACK_CHECK_EN 0x1 // Habilitar ACK

// Definición de la pantalla OLED, ajusta estas constantes según tu configuración
#define SSD1306_MAX_PAGES 4 // Número de páginas en la pantalla (ajustar si es necesario)
#define SSD1306_WIDTH 128   // Ancho de la pantalla en píxeles
#define SSD1306_HEIGHT 32   // Alto de la pantalla en píxeles
#define TAG "SSD1306"

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

static uint8_t mydata[64];
static uint8_t contUnMinutoEnvioTTN = 0;
static const char *TAGSPG30 = "SGP30";
static const char *tag = "dht20_read_task";
uint8_t cur_page = 0;
TaskHandle_t read_data_h;

esp_err_t sgp30_write_command(uint16_t command);
esp_err_t sgp30_read_data(uint16_t *co2_eq, uint16_t *tvoc);
void dht20_read_task(void *param);

// Variables de estado global
bool is_initialized = false;

void setup_ttn(); // Prototipo de la función

// Tarea de envío de mensajes
void sendMessages(void* pvParameter)
{
    while (1) {
        printf("Sending message...\n");
        ttn_response_code_t res = ttn_transmit_message(mydata, sizeof(mydata) - 1, 1, false);
        printf(res == TTN_SUCCESSFUL_TRANSMISSION ? "Message sent.\n" : "Transmission failed.\n");

        vTaskDelay(TX_INTERVAL * pdMS_TO_TICKS(1000));
    }
}

// Manejo de recepción de mensajes
void messageReceived(const uint8_t* message, size_t length, ttn_port_t port)
{
    printf("Message of %d bytes received on port %d:", length, port);
    for (int i = 0; i < length; i++)
        printf(" %02x", message[i]);
    printf("\n");
}

// Inicialización de la OLED
void i2c_oled_init()
{
    SSD1306_t dev; // Crea una instancia de SSD1306_t
    dev._i2c_num = I2C_NUM_0;
    dev._address = 0x3C;
    dev._width = SSD1306_WIDTH;
    dev._height = SSD1306_HEIGHT;
    dev._pages = 4; // Para OLED de 128x32

    // Inicializa el comando I2C para la OLED
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev._address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_OFF, true); // Apagar la pantalla
    i2c_master_write_byte(cmd, OLED_CMD_SET_MUX_RATIO, true);
    i2c_master_write_byte(cmd, 0x1F, true); // 32 píxeles
    i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_OFFSET, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_START_LINE, true);

    i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP_1, true); // Mapeo de segmentos
    i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true);   // Modo de escaneo
    i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_CLK_DIV, true);
    i2c_master_write_byte(cmd, 0x80, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_COM_PIN_MAP, true);
    i2c_master_write_byte(cmd, 0x02, true); // Mapeo de pines COM para 32 píxeles
    i2c_master_write_byte(cmd, OLED_CMD_SET_CONTRAST, true);
    i2c_master_write_byte(cmd, 0xFF, true);
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_RAM, true); // Modo RAM
    i2c_master_write_byte(cmd, OLED_CMD_SET_VCOMH_DESELCT, true);
    i2c_master_write_byte(cmd, 0x40, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_MEMORY_ADDR_MODE, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_PAGE_ADDR_MODE, true); // Modo de dirección de página
    i2c_master_write_byte(cmd, 0x00, true);                        // Dirección de columna baja
    i2c_master_write_byte(cmd, 0x10, true);                        // Dirección de columna alta
    i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
    i2c_master_write_byte(cmd, 0x14, true); // Habilitar la bomba de carga
    i2c_master_write_byte(cmd, OLED_CMD_DEACTIVE_SCROLL, true);
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_NORMAL, true); // Modo normal
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);     // Encender la pantalla

    i2c_master_stop(cmd);

    esp_err_t res = i2c_master_cmd_begin(dev._i2c_num, cmd, 100);
    if (res == ESP_OK)
    {
        ESP_LOGI("OLED", "OLED configured successfully");
    }
    else
    {
        ESP_LOGE("OLED", "OLED configuration failed. code: 0x%.2X", res);
    }
    i2c_cmd_link_delete(cmd);
}

// Limpiar pantalla OLED
void task_ssd1306_display_clear(void *ignore)
{
    i2c_cmd_handle_t cmd;
    uint8_t zero[128] = {0}; // Buffer de 128 ceros para limpiar cada línea

    for (uint8_t i = 0; i < 8; i++) // Limpiar las 8 páginas (líneas) de la pantalla
    {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
        i2c_master_write_byte(cmd, 0xB0 | i, true); // Seleccionar página (línea)

        i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
        i2c_master_write(cmd, zero, 128, true); // Escribir 128 ceros en la línea
        i2c_master_stop(cmd);
        // Aumentar el tiempo de espera para garantizar la comunicación
        i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
    }

    // Borrar la tarea una vez completada
    vTaskDelete(NULL);
}

// Mostrar texto en OLED
void task_ssd1306_display_text(void *arg_text)
{
    char *text = (char *)arg_text;
    uint8_t text_len = strlen(text);

    i2c_cmd_handle_t cmd;

    // CON ESTO DEFINO LAS PAGINAS QUE QUIERO QUE SE ESCRIBAN
    if (cur_page > 3)
    {
        cur_page = 0;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
    i2c_master_write_byte(cmd, 0x00, true); // reset column
    i2c_master_write_byte(cmd, 0x10, true);
    i2c_master_write_byte(cmd, 0xB0 | cur_page, true); // reset page

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);

    for (uint8_t i = 0; i < text_len; i++)
    {
        if (text[i] == '\n')
        {
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

            i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
            i2c_master_write_byte(cmd, 0x00, true); // reset column
            i2c_master_write_byte(cmd, 0x10, true);
            i2c_master_write_byte(cmd, 0xB0 | ++cur_page, true); // increment page

            i2c_master_stop(cmd);
            i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10));
            i2c_cmd_link_delete(cmd);
        }
        else
        {
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

            i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
            i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)text[i]], 8, true);

            i2c_master_stop(cmd);
            i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10));
            i2c_cmd_link_delete(cmd);
        }
    }

    cur_page++;
    vTaskDelete(NULL);
}

// Tarea de lectura de datos DHT20 y SGP30
void dht20_read_task(void *param)
{
    dht20_data_t measurements;
    char temp_str[20], humid_str[20], co2_str[20], tvoc_str[20];
    uint16_t co2_eq, tvoc;

    ESP_LOGI(tag, "Entering measerument loop");

    while (1)
    {
        if (dht20_is_calibrated())
        {
            ESP_LOGI(tag, "is calibrated....");
        }
        else
        {
            ESP_LOGI(tag, "is NOT calibrated....");
        }
        
        ESP_ERROR_CHECK(sgp30_write_command(0x2008)); // Comando Measure_air_quality
        ESP_ERROR_CHECK(sgp30_read_data(&co2_eq, &tvoc));

        (void)dht20_read_data(&measurements);
        ESP_LOGI(tag, "Temperature:\t%.1fC.\t Avg: %.1fC", measurements.temperature, measurements.temp_avg);
        ESP_LOGI(tag, "Humidity:   \t%.1f%%.\t Avg: %.1f%%\n", measurements.humidity, measurements.humid_avg);

        // Convertir los valores de temperatura y humedad a cadenas
        snprintf(temp_str, sizeof(temp_str), "Temp: %.1fC", measurements.temperature);
        snprintf(humid_str, sizeof(humid_str), "Hum: %.1f%%", measurements.humidity);

        // Limpiar la pantalla antes de escribir nuevo contenido
        xTaskCreate(task_ssd1306_display_clear, "ssd1306_display_clear", 2048, NULL, 6, NULL);
        vTaskDelay(pdMS_TO_TICKS(100)); // Esperar a que se limpie la pantalla

        // Mostrar la temperatura en la pantalla OLED
        xTaskCreate(task_ssd1306_display_text, "ssd1306_display_text", 2048, (void *)temp_str, 6, NULL);
        vTaskDelay(pdMS_TO_TICKS(10)); // Esperar un poco antes de escribir la siguiente línea

        // Mostrar la humedad en la pantalla OLED
        xTaskCreate(task_ssd1306_display_text, "ssd1306_display_text", 2048, (void *)humid_str, 6, NULL);
        vTaskDelay(pdMS_TO_TICKS(10)); // Esperar un poco antes de seguir

        ESP_LOGI(TAGSPG30, "CO2eq: %d ppm, TVOC: %d ppb", co2_eq, tvoc);

        snprintf(co2_str, sizeof(co2_str), "CO2eq: %d ppm", co2_eq);
        snprintf(tvoc_str, sizeof(tvoc_str), "TVOC: %d ppb", tvoc);

        // Mostrar CO2eq en la tercera página
        xTaskCreate(task_ssd1306_display_text, "display_co2", 2048, (void *)co2_str, 5, NULL);
        vTaskDelay(pdMS_TO_TICKS(10)); // Esperar un poco antes de escribir la siguiente línea

        // Mostrar TVOC en la cuarta página
        xTaskCreate(task_ssd1306_display_text, "display_tvoc", 2048, (void *)tvoc_str, 5, NULL);

        // Preparar el payload para enviar datos por loRaWAN
        snprintf((char *)mydata, sizeof(mydata), "T:%.1f H:%.1f CO2:%d TVOC:%d", 
                 measurements.temperature, measurements.humidity, co2_eq, tvoc);


        // Envio de datos a TTN la primera vez y despues cada 60 segundos
        if(/*contUnMinutoEnvioTTN == 0 ||*/ contUnMinutoEnvioTTN == 5)
        {
            setup_ttn();

            printf("Joining...\n");
            if (ttn_join())
            {
                printf("Joined.\n");
                xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void* )0, 3, NULL);
            }
            else
            {
                printf("Join failed.\n");
            }

            contUnMinutoEnvioTTN = 0;
        }

        contUnMinutoEnvioTTN++;
        ESP_LOGI("DEPURACION", "CONTADOR = %d", contUnMinutoEnvioTTN);

        vTaskDelay(pdMS_TO_TICKS(2000)); // Esperar 2 segundos antes de la próxima lectura
    }
}

// Función para escribir comandos al sensor
esp_err_t sgp30_write_command(uint16_t command)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SGP30_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (command >> 8) & 0xFF, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, command & 0xFF, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Función para leer los datos del sensor
esp_err_t sgp30_read_data(uint16_t *co2_eq, uint16_t *tvoc)
{
    uint8_t data[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SGP30_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read(cmd, data, sizeof(data) - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + 5, I2C_MASTER_NACK); // CRC (no usado aquí)
    i2c_master_stop(cmd);
    vTaskDelay(pdMS_TO_TICKS(100)); // MUY IMPORTANTE ESTE RETARDO DE 100 mS
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK)
    {
        *co2_eq = (data[0] << 8) | data[1];
        *tvoc = (data[3] << 8) | data[4];
    }
    return ret;
}

void initialize_peripherals() {
    dht20_begin(); // Inicializar DHT20
    i2c_oled_init(); // Inicializar pantalla OLED
    ESP_ERROR_CHECK(sgp30_write_command(0x2003)); // Comando Init_air_quality
}

void display_initial_message() {
    // Mensaje inicial
    const char *init_message = "    POR FAVOR\n\n     ESPERE";

    //task_ssd1306_display_clear(NULL);  // Llama directamente si solo necesitas limpiar una vez
    //vTaskDelay(pdMS_TO_TICKS(100)); // Espera a que se limpie la pantalla
    //task_ssd1306_display_text((void *)init_message); // Muestra mensaje sin necesidad de una tarea extra

    // Limpiar la pantalla antes de escribir nuevo contenido
    xTaskCreate(task_ssd1306_display_clear, "ssd1306_display_clear", 2048, NULL, 6, NULL);
    vTaskDelay(pdMS_TO_TICKS(100)); // Esperar a que se limpie la pantalla

    // Crea la tarea para mostrar el mensaje en la pantalla OLED
    xTaskCreate(task_ssd1306_display_text, "ssd1306_display_text", 2048, (void *)init_message, 6, NULL);
}

void setup_ttn()
{
    esp_err_t err;

    if (!is_initialized) 
    {
        // Initialize the GPIO ISR handler service
        err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        ESP_ERROR_CHECK(err);
        
        // Initialize the NVS (non-volatile storage) for saving and restoring the keys
        err = nvs_flash_init();
        ESP_ERROR_CHECK(err);

        // Initialize SPI bus
        spi_bus_config_t spi_bus_config = {
            .miso_io_num = TTN_PIN_SPI_MISO,
            .mosi_io_num = TTN_PIN_SPI_MOSI,
            .sclk_io_num = TTN_PIN_SPI_SCLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1
        }; 
        err = spi_bus_initialize(TTN_SPI_HOST, &spi_bus_config, TTN_SPI_DMA_CHAN);
        ESP_ERROR_CHECK(err);

        // Initialize TTN
        ttn_init();

        // Configure the SX127x pins
        ttn_configure_pins(TTN_SPI_HOST, TTN_PIN_NSS, TTN_PIN_RXTX, TTN_PIN_RST, TTN_PIN_DIO0, TTN_PIN_DIO1);

        // The below line can be commented after the first run as the data is saved in NVS
        ttn_provision(devEui, appEui, appKey);

        is_initialized = true;

    }

    // Register callback for received messages
    ttn_on_message(messageReceived);

    // ttn_set_adr_enabled(false);
    // ttn_set_data_rate(TTN_DR_US915_SF7);
    // ttn_set_max_tx_pow(14);
}

void app_main(void)
{
    // Inicializar periféricos
    initialize_peripherals();

    // Mostrar mensaje inicial en la pantalla
    display_initial_message();

    // Espera inicial antes de iniciar tareas de sensores
    vTaskDelay(pdMS_TO_TICKS(15000)); 

    // Se vuelve a poner la pagina de la pantalla a 0
    cur_page = 0;
    contUnMinutoEnvioTTN = 0;

    // Crear tareas para lectura de sensores y transmisión LoRa
    xTaskCreate(dht20_read_task, "read_values", 256 * 11, NULL, 3, &read_data_h); 
    //xTaskCreate(&lmic_task, "lmic_task", 2048, NULL, 5, NULL); // Tarea LMIC

    ESP_LOGI("LoRa", "Tareas inicializadas");
}