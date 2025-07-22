/*------------------------ Estação Metereológica ------------------------*/

#include "pico/stdlib.h"
#include "pico/bootrom.h"

#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"

#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "html.h"

#include "aht20.h"
#include "bmp280.h"
#include "ws2812b.h"

#include <stdio.h>
#include <math.h>

/*------------------------------- Macros --------------------------------*/

// Parâmetros para I2C
#define I2C_PORT_AHT20 i2c0         
#define I2C_SDA_AHT20 0                 
#define I2C_SCL_AHT20 1                  
#define I2C_PORT_BMP280 i2c1  
#define I2C_SDA_BMP280 2
#define I2C_SCL_BMP280 3

#define WRAP 1000
#define DIV_CLK 250

#define WIFI_SSID "Galaxy A23 77BF"
#define WIFI_PASS "qfdc4141"

// Limites padrão de cada grandeza
#define TEMPERATURA_STD_MIN 0
#define TEMPERATURA_STD_MAX 100
#define TEMPERATURA_STD_OFFSET 0
#define UMIDADE_STD_MIN 0
#define UMIDADE_STD_MAX 100
#define UMIDADE_STD_OFFSET 0
#define PRESSAO_STD_MIN 0
#define PRESSAO_STD_MAX 1500
#define PRESSAO_STD_OFFSET 0

#define SENSOR_READ_INTERVAL 1000000

#define SEA_LEVEL_PRESSURE 101325.0 // Pressão ao nível do mar em Pa

#define botaoB 6

/*-----------------------------------------------------------------------*/

/*------------------------------- Structs -------------------------------*/

// Armazenamento de informações de uma grandeza
typedef struct 
{
    double leitura;
    double leituraAnterior;
    float limiteMin;
    float limiteMax;
    bool passouLim;
    Rgb corLed;
    Rgb currCorMatriz;
    uint8_t currAlturaMatriz;
} GrandezaData;

struct http_state
{
    char response[4096];
    size_t len;
    size_t sent;
};

/*-----------------------------------------------------------------------*/

/*-------------------------- Variáveis Globais --------------------------*/

// Array e indexação para cada grandeza lida
enum
{
    TEMPERATURA,
    UMIDADE,
    PRESSAO,
    NUM_GRANDEZA
};
static GrandezaData grandeza[NUM_GRANDEZA];

float offset[NUM_GRANDEZA] = {TEMPERATURA_STD_OFFSET, UMIDADE_STD_OFFSET, PRESSAO_STD_OFFSET};

static struct bmp280_calib_param params;

static bool alarme_on = false;
Rgb cor_warn = {1, 0, 0};

static const uint8_t buzzer_pin = 21;
static uint slice;
static struct repeating_timer timer;
static volatile bool buzzer_on = false;

static double altitude;

/*-----------------------------------------------------------------------*/

/*----------------------------- Protótipos ------------------------------*/

static void param_grandezas_init();
static double calculate_altitude(double pressure);
static void ler_sensores();
static bool capturou_valor_diff();
static bool ultrapassou_lim();
static bool is_same_color(Rgb color1, Rgb color2);
static uint8_t mapear_grandeza_matriz(uint8_t index);
static void matriz_draw_graph();
bool buzzer_callback(struct repeating_timer *t);
void btn_handler(uint gpio, uint32_t events);

static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err);
static void start_http_server(void);

/*-----------------------------------------------------------------------*/

/*----------------------------- Função Main -----------------------------*/

int main()
{
    // Para ser utilizado o modo BOOTSEL com botão B
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &btn_handler);
   // Fim do trecho para modo BOOTSEL com botão B

    stdio_init_all();

    param_grandezas_init();

    matriz_init();

    // Inicializa o Buzzer
    gpio_set_function(buzzer_pin, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(buzzer_pin);

    pwm_set_wrap(slice, WRAP);
    pwm_set_clkdiv(slice, DIV_CLK); 
    pwm_set_gpio_level(buzzer_pin, 0);
    pwm_set_enabled(slice, true);

    // Inicializa o I2C do AHT20 e do BMP280
    i2c_init(I2C_PORT_AHT20, 400 * 1000);
    gpio_set_function(I2C_SDA_AHT20, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_AHT20, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_AHT20);
    gpio_pull_up(I2C_SCL_AHT20);

    i2c_init(I2C_PORT_BMP280, 400 * 1000);
    gpio_set_function(I2C_SDA_BMP280, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_BMP280, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_BMP280);
    gpio_pull_up(I2C_SCL_BMP280);

    // Inicializa o AHT20
    aht20_reset(I2C_PORT_AHT20);
    aht20_init(I2C_PORT_AHT20);

    // Inicializa o BMP280
    bmp280_init(I2C_PORT_BMP280);
    bmp280_get_calib_params(I2C_PORT_BMP280, &params);

     // Aguarda alguns segundos para estabilização do Wi-Fi
    sleep_ms(2000);

    // Inicializa o Wi-Fi e o servidor web
    if (cyw43_arch_init())
    {   
        printf("Wi-Fi falhou! 1\n");
        return 1;
    }
    printf("CYW43 init\n");

    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000))
    {   
        printf("Wi-Fi falhou! 2\n");
        return 1;
    }
    printf("CYW43 sta mode\n");

    uint8_t *ip = (uint8_t *)&(cyw43_state.netif[0].ip_addr.addr);
    printf("%d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
    sleep_ms(2000); // Intervalo para visualização do IP

    start_http_server();

    absolute_time_t last_read_time = 0;
    absolute_time_t now;

    while (true)
    {   
        // Polling para manter conexão
        cyw43_arch_poll();

        now = get_absolute_time();
        if (absolute_time_diff_us(last_read_time, now) > SENSOR_READ_INTERVAL || last_read_time == 0)
        {   
            last_read_time = now;
            ler_sensores();

            if (ultrapassou_lim())
            {   
                if (!alarme_on)
                {
                    alarme_on = true;
                    buzzer_on = true;
                    add_repeating_timer_ms(200, buzzer_callback, NULL, &timer);
                }
            }
            else
            {
                alarme_on = false;
            }

            if (capturou_valor_diff())
            {
                for (int i = 0; i < NUM_GRANDEZA; i++)
                {
                    grandeza[i].leituraAnterior = grandeza[i].leitura;
                }
                matriz_draw_graph();
            }
        }
        
        sleep_ms(500);
    }

    cyw43_arch_deinit();
    return 0;
}

/*-----------------------------------------------------------------------*/

/*------------------------------- Funções -------------------------------*/

// Inicializa as variáveis da struct de cada grandeza
static void param_grandezas_init()
{
    for (int i = 0; i < NUM_GRANDEZA; i++)
    {
        grandeza[i].leitura = 0;
        grandeza[i].leituraAnterior = 0;
        grandeza[i].passouLim = false;

        switch (i)
        {
            case TEMPERATURA:
                grandeza[i].limiteMin = TEMPERATURA_STD_MIN;
                grandeza[i].limiteMax = TEMPERATURA_STD_MAX;
                grandeza[i].corLed = matriz_get_cor(AMARELO);
                break;
            case UMIDADE:
                grandeza[i].limiteMin = UMIDADE_STD_MIN;
                grandeza[i].limiteMax = UMIDADE_STD_MAX;
                grandeza[i].corLed = matriz_get_cor(AZUL);
                break;
            case PRESSAO:
                grandeza[i].limiteMin = PRESSAO_STD_MIN;
                grandeza[i].limiteMax = PRESSAO_STD_MAX;
                grandeza[i].corLed = matriz_get_cor(BRANCO);
                break;
        }
    }
}

// Calcula a altitude a partir da pressão atmosférica
static double calculate_altitude(double pressure)
{
    return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903));
}

static void ler_sensores()
{   
    // Estrutura para armazenar os dados do sensor
    AHT20_Data data;
    int32_t raw_temp_bmp;
    int32_t raw_pressure;

    // Leitura do BMP280
    bmp280_read_raw(I2C_PORT_BMP280, &raw_temp_bmp, &raw_pressure);
    double temperatura_bmp = bmp280_convert_temp(raw_temp_bmp, &params) / 100.0f; // Converte para ponto flutuante
    uint32_t pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &params);

    // Cálculo da altitude e converte pressão lida
    altitude = calculate_altitude(pressure);
    grandeza[PRESSAO].leitura = (pressure / 1000.0f) + offset[PRESSAO];

    // Leitura do AHT20
    if (aht20_read(I2C_PORT_AHT20, &data))
    {   
        // Obtém a média da temperatura obtida nos dois sensores
        double temperatura_aht = data.temperature;
        grandeza[TEMPERATURA].leitura = ((temperatura_aht + temperatura_bmp) / 2.0f) + offset[TEMPERATURA];

        // Armazena umidade
        grandeza[UMIDADE].leitura = data.humidity + offset[UMIDADE];

    }
    else
    {   
        // Considera a temperatura como a do BMP280
        grandeza[TEMPERATURA].leitura = temperatura_bmp + offset[TEMPERATURA];
        printf("Erro na leitura do AHT10!\n\n");
    }

    printf("Pressao = %.3f kPa\n", grandeza[PRESSAO].leitura);
    printf("Temperatura: = %.2f C\n", grandeza[TEMPERATURA].leitura);
    printf("Umidade: %.2f %%\n", grandeza[UMIDADE].leitura);
    printf("Altitude estimada: %.2f m\n\n\n", altitude);
}

static bool ultrapassou_lim()
{   
    bool passou = false;
    for (int i = 0; i < NUM_GRANDEZA; i++)
    {
        if (grandeza[i].leitura > grandeza[i].limiteMax || grandeza[i].leitura < grandeza[i].limiteMin)
        {   
            grandeza[i].passouLim = true;
            passou = true;
        }
        else
        {
            grandeza[i].passouLim = false;
        }
    }
    return passou;
}

// Verifica se alguma leitura dos sensores é diferente da armazenada atualmente
static bool capturou_valor_diff()
{
    for (int i = 0; i < NUM_GRANDEZA; i++)
    {
        if (grandeza[i].leitura != grandeza[i].leituraAnterior)
        {
            return true;
        }
    }
    return false;
}

// Verifica se duas cores são iguais
static bool is_same_color(Rgb color1, Rgb color2)
{
    return color1.r == color2.r && color1.g == color2.g && color1.b == color2.b;
}

static uint8_t mapear_grandeza_matriz(uint8_t index)
{   
    GrandezaData select = grandeza[index]; 

    // Limita o valor lido  
    float valor_lido = select.leitura;
    if (valor_lido > select.limiteMax)
    {
        valor_lido = select.limiteMax;
    }
    else if (valor_lido < select.limiteMin)
    {
        valor_lido = select.limiteMin;
    }

    // Mapeia para a altura de LEDs
    float raw = (valor_lido - select.limiteMin) * (MATRIZ_ROWS - 1) / (select.limiteMax - select.limiteMin);
    uint8_t mapeado = round(raw) + 1;
    return mapeado;
}

static void matriz_draw_graph()
{   
    Rgb cor;
    uint8_t altura;
    bool mudou = false;
    uint8_t coluna = 0;

    for (int i = 0; i < NUM_GRANDEZA; i++)
    {   
        Rgb cor = grandeza[i].passouLim ? cor_warn : grandeza[i].corLed;
        uint8_t altura = mapear_grandeza_matriz(i);

        if (altura != grandeza[i].currAlturaMatriz || !is_same_color(cor, grandeza[i].currCorMatriz))
        {
            matriz_draw_coluna(4, coluna, 5, matriz_get_cor(APAGADO));
            matriz_draw_coluna(4, coluna, altura, cor);
            grandeza[i].currAlturaMatriz = altura;
            grandeza[i].currCorMatriz = cor;
            mudou = true;
        }
        coluna += 2;
    }

    if (mudou)
    {
        matriz_send_data();
    }
}

// Callback dos botões
void btn_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

// Callback do buzzer
bool buzzer_callback(struct repeating_timer *t)
{   
    if (!alarme_on)
    {   
        pwm_set_gpio_level(buzzer_pin, 0);
        buzzer_on = false;
        return false;
    }

    if (buzzer_on)
    {
        pwm_set_gpio_level(buzzer_pin, WRAP / 2);
    }
    else
    {
        pwm_set_gpio_level(buzzer_pin, 0);
    }
    buzzer_on = !buzzer_on;

    return true;
}

static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    struct http_state *hs = (struct http_state *)arg;
    hs->sent += len;
    if (hs->sent >= hs->len)
    {
        tcp_close(tpcb);
        free(hs);
    }
    return ERR_OK;
}

static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (!p) {
        tcp_close(tpcb);
        return ERR_OK;
    }

    char *req = (char *)p->payload;
    struct http_state *hs = malloc(sizeof(struct http_state));
    if (!hs) {
        pbuf_free(p);
        tcp_close(tpcb);
        return ERR_MEM;
    }
    hs->sent = 0;

    if (strstr(req, "GET /estado")) {
        char json[300];
        int len = snprintf(json, sizeof(json),
            "{\"temp\":%.1f,\"hum\":%.1f,\"pres\":%.1f,\"alt\":%.1f,"
            "\"tmin\":%.1f,\"tmax\":%.1f,\"hmin\":%.1f,\"hmax\":%.1f,"
            "\"pmin\":%.1f,\"pmax\":%.1f}",
            grandeza[TEMPERATURA].leitura, grandeza[UMIDADE].leitura, grandeza[PRESSAO].leitura, altitude,
            grandeza[TEMPERATURA].limiteMin, grandeza[TEMPERATURA].limiteMax,
            grandeza[UMIDADE].limiteMin, grandeza[UMIDADE].limiteMax,
            grandeza[PRESSAO].limiteMin, grandeza[PRESSAO].limiteMax);

        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: %d\r\nConnection: close\r\n\r\n%s",
            len, json);
    }

    else if (strstr(req, "GET /config?")) {
        // Função auxiliar para extrair parâmetros da URL
        float extract_value(char *base, const char *key) {
            char *pos = strstr(base, key);
            return pos ? atof(pos + strlen(key)) : NAN;
        }

        struct {
            const char *min_key, *max_key, *ofs_key;
        } keys[NUM_GRANDEZA] = {
            {"tempmin=", "tempmax=", "tempofs="},
            {"hummin=", "hummax=", "humofs="},
            {"presmin=", "presmax=", "presofs="}
        };

        for (int i = 0; i < NUM_GRANDEZA; i++) {
            float vmin = extract_value(req, keys[i].min_key);
            float vmax = extract_value(req, keys[i].max_key);
            float vofs = extract_value(req, keys[i].ofs_key);

            if (!isnan(vmin)) grandeza[i].limiteMin = vmin;
            if (!isnan(vmax)) grandeza[i].limiteMax = vmax;
            if (!isnan(vofs)) offset[i] = vofs;

            // Correção de coerência
            if (grandeza[i].limiteMax <= grandeza[i].limiteMin)
                grandeza[i].limiteMax = grandeza[i].limiteMin + 1;
        }

        const char *ok = "Configurações atualizadas";
        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: %d\r\nConnection: close\r\n\r\n%s",
            (int)strlen(ok), ok);
    }

    else if (strstr(req, "GET /config")) {
        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: %d\r\nConnection: close\r\n\r\n%s",
            (int)strlen(HTML_CONFIG), HTML_CONFIG);
    }

    else { // Página inicial
        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: %d\r\nConnection: close\r\n\r\n%s",
            (int)strlen(HTML_HOME), HTML_HOME);
    }

    tcp_arg(tpcb, hs);
    tcp_sent(tpcb, http_sent);
    tcp_write(tpcb, hs->response, hs->len, TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);
    pbuf_free(p);
    return ERR_OK;
}

static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    tcp_recv(newpcb, http_recv);
    return ERR_OK;
}

static void start_http_server(void)
{
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb)
    {
        printf("Erro ao criar PCB TCP\n");
        return;
    }
    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK)
    {
        printf("Erro ao ligar o servidor na porta 80\n");
        return;
    }
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, connection_callback);
    printf("Servidor HTTP rodando na porta 80...\n");
}

/*-----------------------------------------------------------------------*/