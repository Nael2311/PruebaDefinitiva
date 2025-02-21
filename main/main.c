
// NAEL GEORGES CHEHADE 
// TFG UMA 2024/2025
// Sistema de Monitorización de la Calidad del Aire

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "dht20.h" // Libreria sensor temperatura y humedad
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "ssd1306.h" // Libreria pantalla OLED
#include "font8x8_basic.h" // Libreria para pantalla OLED
#include "esp_event.h"
#include "nvs_flash.h"
#include "ttn.h" // Libreria para conexion con TTN 
#include "esp_timer.h"    // Para medir el tiempo
#include "esp_task_wdt.h" // Watch dog
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_sleep.h"

// AppEUI, DevEUI y AppKey
const char *appEui = "0000000000000000";
const char *devEui = "70B3D57ED006B4C5";
const char *appKey = "528ED36701C123C3954535625253BFF2";

// Tamaño máximo permitido por LoRaWAN para el payload
#define MAX_PAYLOAD_SIZE 51

// Pines y configuración de recursos
#define TTN_SPI_HOST SPI2_HOST
#define TTN_SPI_DMA_CHAN SPI_DMA_DISABLED
#define TTN_PIN_SPI_SCLK 18
#define TTN_PIN_SPI_MOSI 23
#define TTN_PIN_SPI_MISO 19
#define TTN_PIN_NSS 5
#define TTN_PIN_RXTX TTN_NOT_CONNECTED
#define TTN_PIN_RST 16
#define TTN_PIN_DIO0 2
#define TTN_PIN_DIO1 15
#define Numero_mediciones_siguiente_conexion 60 // cada hora aprox para la funcion conexion_ttn_bajo_consumo
#define Numero_mensajes_enviados_probar_conexion 10 // despues del envio de X mensajes se prueba la conexion reiniciando el sistema
#define Segundos_mediciones_niveles_tipicos 60
#define Segundos_mediciones_niveles_moderados 20
#define Segundos_mediciones_niveles_perjudiciales 10
// #define Segundos_intento_conexion_TTN 3600 // para la funcion conexion_ttn

// Para monitorear tareas, activar en el menuconfig FREERTOS_USE_TRACE_FACILITY
#define MAX_TASKS 20 // Ajusta según el número de tareas esperadas

// SPG30
#define SGP30_ADDR 0x58       // Dirección I2C del SGP30
#define OLED_I2C_ADDRESS 0x3C // Direccion I2C del ssd1306
#define ACK_CHECK_EN 0x1      // Habilitar ACK

// Definición de la pantalla OLED, ajusta estas constantes según tu configuración
#define SSD1306_MAX_PAGES 4 // Número de páginas en la pantalla (ajustar si es necesario)
#define SSD1306_WIDTH 128   // Ancho de la pantalla en píxeles
#define SSD1306_HEIGHT 32   // Alto de la pantalla en píxeles
#define TAG "SSD1306"

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

static uint8_t mydata[64];
int contNumMediciones = -1;
static const char *TAGSPG30 = "SGP30";
static const char *tag = "dht20_read_task";
uint8_t cur_page = 0;
TaskHandle_t read_data_h;
TaskHandle_t conexion_ttn_h;
TaskHandle_t conexion_ttn_bajo_consumo_h;

// Prototipo de la función
esp_err_t sgp30_write_command(uint16_t command);
esp_err_t sgp30_read_data(uint16_t *co2_eq, uint16_t *tvoc);
void dht20_read_task(void *param);
void setup_ttn();
void Conexion_TTN_bajo_consumo(void *parametro);
void sleep_manager(int segundos);

// Variables de estado global
bool is_initialized = false;
int len;
bool conexion_ttn = true;
char ttn_estado_conexion[17] = "TTN desconectado";
bool estado_tarea_medicion = true;
bool estado_tarea_conexionTTN = false;
int ultimo_valor = -1;

// static int start_ticks = 0;
// static int end_ticks = 0;
// static int active_time_ticks = 0;
// float active_time_seconds = 0;

// Manejo de recepción de mensajes
void messageReceived(const uint8_t *message, size_t length, ttn_port_t port)
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
    i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100)); // IMPORTANTE QUE SE 100 MINIMO
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
            i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100)); // IMPORTANTE QUE SE 100 MINIMO
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
            i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100)); // IMPORTANTE QUE SE 100 MINIMO
            i2c_cmd_link_delete(cmd);
        }
    }

    cur_page++;
}

// Tarea de lectura de datos DHT20 y SGP30 y escrtura en oled
void dht20_read_task(void *param)
{
    dht20_data_t measurements;
    uint16_t co2_eq, tvoc;
    char temp_hum_str[17], co2_str[17], tvoc_str[17];

    // Este bucle es para inicializar el SGP30
    if (contNumMediciones == -1)
    {
        for (int x = 0; x < 10; x++)
        {
            // Lectura de valores de CO2 y TVOC
            ESP_ERROR_CHECK(sgp30_write_command(0x2008)); // Comando Measure_air_quality
            ESP_ERROR_CHECK(sgp30_read_data(&co2_eq, &tvoc));
            vTaskDelay(pdMS_TO_TICKS(1000)); // Espera antes de realizar otra medicion para no saturar el bus i2c
        }

        ESP_LOGI(tag, "SGP30 Calibrado");
    }

    // start_ticks = xTaskGetTickCount();

    ESP_LOGI(tag, "Entrando en bucle de medición");

    // Limpiar la pantalla antes de escribir nuevo contenido
    task_ssd1306_display_clear(NULL);

    // Se vuelve a poner la pagina de la pantalla a 0
    cur_page = 0;

    while (1)
    {
        estado_tarea_medicion = true;

        if (dht20_is_calibrated())
        {
            ESP_LOGI(tag, "DHT20 Calibrado");
        }
        else
        {
            ESP_LOGI(tag, "DHT20 No calibrado");
        }

        // Lectrura de valores de humedad y temperatura
        (void)dht20_read_data(&measurements);
        vTaskDelay(pdMS_TO_TICKS(100)); // Espera antes de realizar otra medicion para no saturar el bus i2c

        // Lectura de valores de CO2 y TVOC
        ESP_ERROR_CHECK(sgp30_write_command(0x2008)); // Comando Measure_air_quality
        ESP_ERROR_CHECK(sgp30_read_data(&co2_eq, &tvoc));
        vTaskDelay(pdMS_TO_TICKS(100)); // Espera antes de realizar otra medicion para no saturar el bus i2c

        // Convertir los valores de temperatura y humedad en una sola línea
        snprintf(temp_hum_str, sizeof(temp_hum_str), "T:%.1fC H:%.1f%%", measurements.temperature, measurements.humidity);

        // Convertir los valores de co2 y TVOC a cadenas
        snprintf(co2_str, sizeof(co2_str), "CO2eq: %d ppm", co2_eq);
        snprintf(tvoc_str, sizeof(tvoc_str), "TVOC: %d ppb", tvoc);

        // Mostrar los datos en el terminal
        ESP_LOGI(tag, "Temperature:\t%.1fC.\t Avg: %.1fC", measurements.temperature, measurements.temp_avg);
        ESP_LOGI(tag, "Humidity:   \t%.1f%%.\t Avg: %.1f%%", measurements.humidity, measurements.humid_avg);
        ESP_LOGI(TAGSPG30, "CO2eq: %d ppm, TVOC: %d ppb\n", co2_eq, tvoc);

        // Mostrar la temperatura y la humedad en la pantalla OLED
        task_ssd1306_display_text((void *)temp_hum_str);
        vTaskDelay(pdMS_TO_TICKS(100)); // Esperar un poco antes de escribir la siguiente línea

        // Mostrar CO2eq en la pantalla OLED
        task_ssd1306_display_text((void *)co2_str);
        vTaskDelay(pdMS_TO_TICKS(100)); // Esperar un poco antes de escribir la siguiente línea

        // Mostrar TVOC en la pantalla OLED
        task_ssd1306_display_text((void *)tvoc_str);
        vTaskDelay(pdMS_TO_TICKS(100)); // Esperar un poco antes de escribir la siguiente línea

        // Mostrar ttn desconectado en pantalla OLED
        task_ssd1306_display_text((void *)ttn_estado_conexion);
        vTaskDelay(pdMS_TO_TICKS(10)); // Esperar un poco antes de escribir la siguiente línea

        // Preparar el payload para enviar datos por loRaWAN
        len = snprintf((char *)mydata, sizeof(mydata), "T:%.1f H:%.1f CO2:%d TVOC:%d",
                       measurements.temperature, measurements.humidity, co2_eq, tvoc);

        contNumMediciones++;
        ESP_LOGI("NUMERO DE MEDICIONES", "%d\n", contNumMediciones);

        if (contNumMediciones == 0 || contNumMediciones % Numero_mediciones_siguiente_conexion == 0)
        {
            xTaskCreate(Conexion_TTN_bajo_consumo, "ttn_task_bajo_consumo", 10240, NULL, 4, &conexion_ttn_bajo_consumo_h); // Tarea conexion ttn
        }

        vTaskDelay(pdMS_TO_TICKS(100));

        // end_ticks = xTaskGetTickCount();

        // active_time_ticks = end_ticks - start_ticks;
        // active_time_seconds = (float)active_time_ticks / configTICK_RATE_HZ;
        // ESP_LOGI("Tiempo activo", "%f\n", active_time_seconds);

        if (co2_eq > 2000 || tvoc > 1000)
        {
            estado_tarea_medicion = false;
            if (estado_tarea_conexionTTN == false)
            {
                sleep_manager(Segundos_mediciones_niveles_perjudiciales);
            }
            else
            {
                vTaskDelay(Segundos_mediciones_niveles_perjudiciales * pdMS_TO_TICKS(1000)); // Esperar 10 segundos antes de la próxima lectura
            }
        }
        else if (co2_eq > 1000 || tvoc > 500)
        {
            estado_tarea_medicion = false;
            if (estado_tarea_conexionTTN == false)
            {
                sleep_manager(Segundos_mediciones_niveles_moderados);
            }
            else
            {
                vTaskDelay(Segundos_mediciones_niveles_moderados * pdMS_TO_TICKS(1000)); // Esperar 20 segundos antes de la próxima lectura
            }
        }
        else
        {
            estado_tarea_medicion = false;
            if (estado_tarea_conexionTTN == false)
            {
                sleep_manager(Segundos_mediciones_niveles_tipicos);
            }
            else
            {
                vTaskDelay(Segundos_mediciones_niveles_tipicos * pdMS_TO_TICKS(1000)); // Esperar 60 segundos antes de la próxima lectura
            }
        }
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

// Inicializar sensores y pantalla
void initialize_peripherals()
{
    dht20_begin();                                // Inicializar DHT20
    i2c_oled_init();                              // Inicializar pantalla OLED
    ESP_ERROR_CHECK(sgp30_write_command(0x2003)); // Comando Init_air_quality
}

// Escribir mensaje inicial
void display_initial_message()
{
    // Mensaje inicial
    const char *init_message = "    POR FAVOR\n\n     ESPERE";

    // Limpiar la pantalla antes de escribir nuevo contenido
    task_ssd1306_display_clear(NULL);

    // Mostrar el mensaje en la pantalla OLED
    task_ssd1306_display_text((void *)init_message);
}

// Inicializar gpio, spi, ttn, configurar sx1276
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
            .quadhd_io_num = -1};
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
}

// Funcion para la conexion con ttn
/*
void Conexion_TTN(void *parametro)
{

    int fail_count = 0; // Contador de intentos fallidos
    int cont_sent = 0;  // Conador de mensaje enviados

    while (1)
    {
        strcpy(ttn_estado_conexion, "TTN desconectado");

        setup_ttn();

        ESP_LOGI("Info TTN", "Intentando conexion a TTN");
        if (ttn_join())
        {
            ESP_LOGI("Info TTN", "Conexion establecida");
            strcpy(ttn_estado_conexion, "TTN conectado");

            while (1)
            {
                ESP_LOGI("Info TTN", "Enviando mensaje");

                ttn_response_code_t res = ttn_transmit_message(mydata, len, 1, false);

                if (res == TTN_SUCCESSFUL_TRANSMISSION)
                {
                    ESP_LOGI("Info TTN", "Mensaje enviado.");
                    fail_count = 0; // Reinicia el contador si la transmisión tiene éxito
                    cont_sent++;
                    if (cont_sent >= 5) // Despues de enviar 5 mensajes reniciar el sistema para probar la conexion
                    {
                        ESP_LOGI("Info TTN", "Pronto se reiniciara el sistema para volver a probar la conexion.");
                        // Con este bucle aseguro que la tarea de medicion no este midiendo
                        while (estado_tarea_medicion)
                        {
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }
                        esp_restart();
                    }
                }
                else
                {
                    ESP_LOGI("Info TTN", "Transimision de mensaje fallida");
                    fail_count++; // Incrementa el contador de fallos

                    if (fail_count >= 2) // Si hay 2 intentos fallidos consecutivos
                    {
                        ESP_LOGI("Info TTN", "Dos intentos de envio de mensaje fallidos");
                        fail_count = 0;
                        break; // Sal del bucle while
                    }
                }

                ESP_LOGI("Info TTN", "En %d segundos se intentara enviar otro mensaje", Segundos_mediciones_niveles_tipicos);
                vTaskDelay(Segundos_mediciones_niveles_tipicos * pdMS_TO_TICKS(1000)); // Despues de cada medicion tipica se vuelve a enviar mensaje
            }
        }
        else
        {
            ESP_LOGI("Info TTN", "Intento de conexion fallido");
        }

        ESP_LOGI("Info TTN", "En %d minutos se intentara conectarse a la red TTN", Segundos_intento_conexion_TTN / 60);
        vTaskDelay(Segundos_intento_conexion_TTN * pdMS_TO_TICKS(1000));
    }
}
*/

// Funcion para la conexion con ttn
void Conexion_TTN_bajo_consumo(void *parametro)
{
    estado_tarea_conexionTTN = true;

    int fail_count = 0; // Contador de intentos fallidos
    int cont_sent = 0;  // Conador de mensaje enviados

    setup_ttn();

    ESP_LOGI("Info TTN", "Intentando conexion a TTN");
    if (ttn_join())
    {
        ESP_LOGI("Info TTN", "Conexion establecida");
        strcpy(ttn_estado_conexion, "TTN conectado   ");

        cur_page = 3; // Para cambiar solo la linea del estado del ttn
        
        // Mostrar ttn conectado en pantalla OLED
        task_ssd1306_display_text((void *)ttn_estado_conexion);
        vTaskDelay(pdMS_TO_TICKS(10)); // Esperar un poco antes de escribir la siguiente línea

        while (1)
        {
            ESP_LOGI("Info TTN", "Enviando mensaje");

            ttn_response_code_t res = ttn_transmit_message(mydata, len, 1, false);

            if (res == TTN_SUCCESSFUL_TRANSMISSION)
            {
                ESP_LOGI("Info TTN", "Mensaje enviado");

                // Despues de enviar X mensajes reniciar el sistema para probar la conexion
                if (cont_sent >= Numero_mensajes_enviados_probar_conexion)
                {
                    ESP_LOGI("Info TTN", "En unos segundos se reiniciara el sistema para volver a probar la conexion\n");

                    // Con este bucle aseguro que la tarea de medicion no este midiendo
                    while (estado_tarea_medicion)
                    {
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }

                    vTaskDelay(pdMS_TO_TICKS(1000)); // Una pequeña espera de un segundo antes de resetear el sistema
                    esp_restart();
                }

                fail_count = 0; // Reinicia el contador si la transmisión tiene éxito
                cont_sent++;
            }
            else
            {
                ESP_LOGI("Info TTN", "Transimision de mensaje fallida");
                fail_count++; // Incrementa el contador de fallos

                if (fail_count >= 2) // Si hay 2 intentos fallidos consecutivos
                {
                    ESP_LOGI("Info TTN", "Dos intentos de envio de mensaje fallidos");

                    ESP_LOGI("Info TTN", "En unos segundos se reiniciara el sistema para volver a probar la conexion\n");

                    // Con este bucle aseguro que la tarea de medicion no este midiendo
                    while (estado_tarea_medicion)
                    {
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }

                    vTaskDelay(pdMS_TO_TICKS(1000)); // Una pequeña espera de un segundo antes de resetear el sistema
                    esp_restart();
                }
            }

            ultimo_valor = contNumMediciones;
            ESP_LOGI("Info TTN", "Despues de la siguiente medicion se intentara enviar otro mensaje\n");

            // Hasta que no haya como minimo otra medicion no se enviara otro mensaje
            while (ultimo_valor == contNumMediciones)
            {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
    }
    else
    {
        ESP_LOGI("Info TTN", "Intento de conexion fallido");
    }

    strcpy(ttn_estado_conexion, "TTN desconectado");
    ESP_LOGI("Info TTN", "En %d mediciones se intentara conectarse a la red TTN\n", Numero_mediciones_siguiente_conexion);
    estado_tarea_conexionTTN = false;
    vTaskDelete(conexion_ttn_bajo_consumo_h);
}

// Funcion para monitorear la tareas activadas, activar en el menuconfig FREERTOS_USE_TRACE_FACILITY
void listar_tareas()
{
    TaskStatus_t task_status_array[MAX_TASKS];
    uint32_t total_run_time;
    UBaseType_t num_tasks;

    // Obtiene el estado de todas las tareas
    num_tasks = uxTaskGetSystemState(task_status_array, MAX_TASKS, &total_run_time);

    ESP_LOGI("TASK_LIST", "Tareas activas: %d", num_tasks);
    for (int i = 0; i < num_tasks; i++)
    {
        ESP_LOGI("TASK_LIST",
                 "Tarea: %s, Estado: %d, Prioridad: %d, Stack libre: %lu",
                 task_status_array[i].pcTaskName,
                 task_status_array[i].eCurrentState,
                 task_status_array[i].uxCurrentPriority,
                 (unsigned long)task_status_array[i].usStackHighWaterMark);
    }
}

void sleep_manager(int segundos)
{
    // Ambas tareas están inactivas. El sistema puede entrar en modo de suspensión.
    ESP_LOGI("Info Bajo Consumo", "Ambas tareas están inactivas. El sistema entrara en modo de suspension.\n");

    // Configurar el temporizador RTC para despertar después de WAKEUP_TIME_SEC
    esp_sleep_enable_timer_wakeup(segundos * 1000000); // Convertir a microsegundos

    // Configurar el modo de suspensión ligera
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_light_sleep_start(); // Entra en suspensión ligera, no reinicia el sistema
    // esp_deep_sleep_start(); // Si se utiliza el deep, al salirde este se hace como un reset del sistema entero

    // Mensaje al despertar del modo de suspensión
    ESP_LOGI("Info Bajo Consumo", "El sistema ha salido del modo de suspension.\n");
}

void app_main(void)
{
    // Deshabilitar Wi-Fi
    esp_wifi_stop();
    esp_wifi_deinit();

    // Inicializar periféricos
    initialize_peripherals();

    // Mostrar mensaje inicial en la pantalla
    display_initial_message();

    // Espera inicial antes de iniciar tareas de sensores
    vTaskDelay(pdMS_TO_TICKS(15000));

    // Verifica si hay alguna tarea en ejecucion, y elimínala si es necesario:
    if (read_data_h != NULL)
    {
        vTaskDelete(read_data_h);
        read_data_h = NULL;
    }
    if (conexion_ttn_h != NULL)
    {
        vTaskDelete(conexion_ttn_h);
        conexion_ttn_h = NULL;
    }

    // Crear tareas para lectura de sensores y transmisión LoRa
    xTaskCreate(dht20_read_task, "read_values", 4096, NULL, 3, &read_data_h);

    // xTaskCreate(Conexion_TTN, "ttn_task", 10240, NULL, 4, &conexion_ttn_h); // Tarea conexion ttn

    // while (1)
    // {
    //     vTaskDelay(pdMS_TO_TICKS(120000)); // Actualiza cada 5 segundos
    //     listar_tareas();                 // Llama a la función para listar tareas
    // }
}
