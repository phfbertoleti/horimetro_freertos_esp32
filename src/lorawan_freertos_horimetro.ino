/* Projeto: horímetro com LoRaWAN (modo: ABP), usando FreeRTOS como sistema operacional embarcado.
 *          O número de horas máximo contabilizado é de 1193046 horas.
 *          O projeto já está configurado para operar na rede LoRaWAN da ATC / Everynet no Brasil.
 *          Este programa NÃO faz uso do modo sleep (light sleep ou deep sleep) do ESP32. Dessa forma,
 *          não se trata de um projeto low power, utilizando a bateria somente como um "backup" de energia.
 *          
 * Autor: Pedro Bertoleti e agradecimentos a:
 *        - Professor Marcelus Guirardello por toda a ajuda na codificação da 
 *        comunicação LoRaWAN.
 *        - Renan Tesch, por toda a ajuda na melhoria da rotina de leitura de 
 *        tensão de bateria.
 *        - José Morais, por toda a ajuda com Event Groups e NVS
 * 
 * Placa de desenvolvimento utilizada: Heltec LoRa wifi v1 (https://www.curtocircuito.com.br/placa-wifi-lora-32-esp32-lora-display-oled.html?gclid=EAIaIQobChMIqOLj9ZG-6gIVBL7ACh1xvQorEAkYASABEgKLOfD_BwE)
 * 
 * GPIO utilizado para strt/stop do horímetro: 36
 * 
 * IMPORTANTE:
 * 1) Este projeto considera a tensão da bateria lida no GPIO37 (ADC1_1), onde a tensão é lida num divisor de tensão 
 *    (resistor de 470k / 0,25W e resistor de 100k / 0,25W). 
 * 
 * VBAT -----------
 *                |                               R1: resistor de 470k / 0,25W
 *               ---                              R2: resistor de 100k / 0,25W
 *                R1                              RL: impedância do ADC (calculado: 13M)
 *               --- 
 *                |       ADC1_1 (Vmax: 0.73V)
 *                |---------
 *                |        |
 *               ---      --- 
 *                R2       RL 
 *               ---      --- 
 *                |        |  
 * GND ---------------------
 * 
 *    NÃO SE ESQUEÇA DE USAR O DIVISOR DE TENSÃO AQUI!! O ADC do ESP32 suporta, no máximo, 1,1V (0dB), 
 *    enquanto a tensão de bateria pode chegar a 4,2V.
 * 
 * 2) Esse projeto faz uso da biblioteca "MCCI LoRaWAN LMIC Library". 
 *    Este projeto foi testado com a versão 2.3.2 da mesma.
 *    
 * 3) Antes de compilar, é preciso deixar o arquivo lmic_project_config.h   
 *    (dentro na pasta da biblioteca: project_config/lmic_project_config.h) com o 
 *     conteúdo conforme abaixo:
 *
 *    // project-specific definitions
 *    //#define CFG_eu868 1
 *    //#define CFG_us915 1
 *    #define CFG_au921 1
 *    //#define CFG_as923 1
 *    // #define LMIC_COUNTRY_CODE LMIC_COUNTRY_CODE_JP      
 *    //#define CFG_in866 1
 *    #define CFG_sx1276_radio 1
 *    //#define LMIC_USE_INTERRUPTS
 */

#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP085.h>
#include "nvs_flash.h"
#include "OLED_defs.h"
#include "BATERIA_defs.h"
#include "RADIO_LORA_defs.h"
#include "LORAWAN_defs.h"
#include "TEMPORIZACOES_defs.h"
#include "PRIOS_defs.h"
#include "STACK_SIZES_defs.h"
#include "GPIO_defs.h"
#include "TIMERS_IDS_defs.h"

/* Definições gerais */
#define BAUDRATE_SERIAL_DEBUG               115200
#define TEMPO_ENTRE_LEITURAS_BMP180         500 //ms
#define TEMPO_PARA_RESETAR_HORIMETRO        6000 //ms

/* Constantes do rádio LoRa: GPIOs utilizados para comunicação
   com rádio SX1276 */
const lmic_pinmap lmic_pins = {
  .nss = RADIO_NSS_PORT,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = RADIO_RESET_PORT,
  .dio = {RADIO_DIO_0_PORT, RADIO_DIO_1_PORT, LMIC_UNUSED_PIN},  //dio2 não é utilizado.
};

/* Estruturas */

/* Estrutura que contem cada linha que pode ser escrita no display OLED da placa */
typedef struct
{
    char linha2[OLED_LINE_MAX_SIZE];
    char linha3[OLED_LINE_MAX_SIZE];
    char linha4[OLED_LINE_MAX_SIZE];
}TTela_display;

/* Estrutura de tempos do horimetro */
typedef struct
{
    unsigned long horimetro_parcial;
    unsigned long horimetro_total;
}TTempos_horimetro;


/* Estrutura com os dados já no formado que são enviados via LoRaWAN */
typedef struct __attribute__((__packed__))
{
   unsigned long  horimetro_total;   //4 bytes
   unsigned long  horimetro_parcial; //4 bytes
   char           tensao_mult_10;    //1 byte
   char           carga_bateria;     //1 byte
}TDados_lorawan;

#define TAMANHO_DADOS_LORAWAN   sizeof(TDados_lorawan)

/* Objetos globais */
Adafruit_SSD1306 display(OLED_SCREEN_WIDTH, 
                         OLED_SCREEN_HEIGHT, 
                         &Wire, 
                         OLED_RESET, 
                         100000UL, 
                         100000UL);

/* Semáforo */
SemaphoreHandle_t xI2C_semaphore;
SemaphoreHandle_t xSerial_semaphore;

/* Filas */
QueueHandle_t xQueue_bateria;
QueueHandle_t xQueue_display;
QueueHandle_t xQueue_tempo_horimetro;
QueueHandle_t xQueue_envio_lorawan;

/* Soft Timers */
TimerHandle_t timer_debounce_gpio;
TimerHandle_t timer_1s;

/* Event groups */
#define EV_GPIO (1<<0)
EventGroupHandle_t evt_gpio;

#define EV_1S (1<<0)
EventGroupHandle_t evt_1s;

/* Variaveis e objetos globais */
static osjob_t sendjob; //objeto para job de envio de dados via ABP
esp_adc_cal_characteristics_t adc_cal; //Estrutura que contem as informacoes para calibracao

/* Relação de tensão x carga da bateria */
#define PONTOS_MAPEADOS_BATERIA   11
char cargas_mapeadas[PONTOS_MAPEADOS_BATERIA] = { 0,
                                                  3, 
                                                  13,
                                                  22,
                                                  39,
                                                  53,
                                                  62,
                                                  74,
                                                  84,
                                                  94,
                                                  100 };
                             
float tensao_x_carga[PONTOS_MAPEADOS_BATERIA] = {3.2,   //0%
                                                 3.3,   //3%
                                                 3.4,   //13%
                                                 3.5,   //22%
                                                 3.6,   //39%
                                                 3.7,   //53%
                                                 3.8,   //62%
                                                 3.9,   //74%
                                                 4.0,   //84%
                                                 4.1,   //94%
                                                 4.2 }; //100%



/* Tarefas */
void task_oled( void *pvParameters );                         
void task_formata_medicoes_display( void *pvParameters );
void task_envio_lorawan( void *pvParameters );
void task_horimetro( void *pvParameters );

/* Protótipos */
void escreve_alerta_bateria_baixa(void);
char calculo_carga_bateria(float tensao_bateria);
void configura_adc_bateria(void);
float le_tensao_bateria(void);
unsigned long diferenca_tempo(unsigned long tstamp);
void do_send(osjob_t* j, unsigned long total, unsigned long parcial);
void grava_horimetro_total(unsigned long ht);
unsigned long le_horimetro_total(void);

/*
 * ISRs
 */

/* ISR chamada no overflow do timer do debounce de GPIO */
void ISR_timer_debounce(TimerHandle_t xTimer)
{
    xEventGroupSetBits(evt_gpio, EV_GPIO);
}

/* ISR chamada no overflow do timer de 1 segundo */
void ISR_timer_1_segundo(TimerHandle_t xTimer)
{
    xEventGroupSetBits(evt_1s, EV_1S);
}

/* ISR responsável por tratar GPIO para
   começar o funcionamento do horímetro*/ 
void IRAM_ATTR ISR_start_horimetro() 
{
    xTimerStart(timer_debounce_gpio, pdMS_TO_TICKS(10));                                   
}

/* ISR responsável por tratar GPIO para
   terminar o funcionamento do horímetro*/ 
void IRAM_ATTR ISR_stop_horimetro() 
{
    xTimerStart(timer_debounce_gpio, pdMS_TO_TICKS(10));                                   
}


/*
 *  Implementações
 */
 
/* Função: calcula diferença de tempo (ms) entre tempo atual e referência passada
 * Parâmetros: referÊncia de tempo passada
 * Retorno:  diferença de tempo
 */
unsigned long diferenca_tempo(unsigned long tstamp)
{
    return (millis() - tstamp);
}


/* Função: configura ADC para leitura da tensão de bateria
 * Parâmetros: nenhum
 * Retorno:  nenhum
 */
void configura_adc_bateria(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_1,ADC_ATTEN_DB_0);
    
    esp_adc_cal_value_t adc_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 1100, &adc_cal);
    
    if (adc_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        if (xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE)
        {
            Serial.println("ADC CALV ref eFuse encontrado: ");
            Serial.print(adc_cal.vref);
            Serial.print("mV");
            xSemaphoreGive(xSerial_semaphore);
        }
    }
    else if (adc_type == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        if (xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE)
        {
            Serial.println("ADC CAL Two Point eFuse encontrado");
            xSemaphoreGive(xSerial_semaphore);
        }
    }
    else
    {
        if (xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE)
        {  
            Serial.println("ADC CAL Nada encontrado, utilizando Vref padrao: ");
            Serial.print(adc_cal.vref);
            Serial.print("mV");
            xSemaphoreGive(xSerial_semaphore);
        }
    }
}

/* Função: le tensão da bateria
 * Parâmetros: nenhum
 * Retorno:  tensão da bateria (V)
 */
float le_tensao_bateria(void)
{
    unsigned long leitura_adc_bateria = 0;
    unsigned long soma_leitura_adc_bateria = 0;
    float tensao_bateria = 0.0;
    float tensao_adc = 0.0;
    int i;

    for(i=0; i<NUMERO_LEITURAS_BATERIA; i++)
    {
        leitura_adc_bateria = adc1_get_raw(ADC1_CHANNEL_1);
        soma_leitura_adc_bateria = soma_leitura_adc_bateria + leitura_adc_bateria;
    }

    leitura_adc_bateria = soma_leitura_adc_bateria / NUMERO_LEITURAS_BATERIA;
    tensao_adc = esp_adc_cal_raw_to_voltage(leitura_adc_bateria, &adc_cal);  //unidade: mV
    tensao_adc = tensao_adc / 1000.0; //unidade: V

    /*         bateria                 adc
                  4.1V     -------     0.73V 
           tensao_bateria  -------     tensao_adc  

       tensao_bateria = tensao_adc*(4.1/0.73)
    */

    tensao_bateria = tensao_adc*(4.1/0.73);
    
    return tensao_bateria;
}


/* Função: calcula a porcentagem de carga restante da bateria
 * Parâmetros: tensão da bateria 
 * Retorno: porcentagem de carga restante da bateria (0..100%)
 */
char calculo_carga_bateria(float tensao_bateria)
{
    char carga_bateria = 0;
    char carga_bateria_float = 0.0;
    int i;
    int idx_menor_distancia;
    bool carga_calculada = false;
    float distancias[(PONTOS_MAPEADOS_BATERIA-1)] = {0.0};
    float menor_distancia;
    float x0, y0, x1, y1, m;

    /* Verifica se a tensão da bateria já está nos niveis mapeados */
    for (i=0; i<PONTOS_MAPEADOS_BATERIA; i++)
    {
        if (i < (PONTOS_MAPEADOS_BATERIA - 1))
            distancias[i] = abs(tensao_bateria - tensao_x_carga[i]);
        
        if (tensao_x_carga[i] == tensao_bateria)
        {
            carga_bateria_float = (float)cargas_mapeadas[i];
            carga_calculada = true;
            break;
        }
    }

    if ( carga_calculada == false)
    {
        /* Se a tensão da bateria não está nos níveis mapeados,
           calcula a carga com base na interpolação linear entre os dois níveis
           mapeados mais próximos */
        menor_distancia = distancias[0];
        idx_menor_distancia = 0;
        
        for (i=1; i<(PONTOS_MAPEADOS_BATERIA - 1); i++)
        {
            if ( distancias[i] < menor_distancia )
            {
                menor_distancia = distancias[i];
                idx_menor_distancia = i;                
            }
        }

        //tensão: eixo x
        //carga: eixo y
        //tensao mais prox da mapeada: x0
        //carga mais prox da mapeada: y0
        //tensão mapeada imediatamente acima: x1
        //carga mapeada imediatamente acima: y1
        //Coeficiente angular da reta que passa pelos dois níveis mapeados mais próximos: m = (y1-y0) / (x1 - x0)
        //equação de reta: y = m*(x-x0) + y0 -> y = m*tensoa_bateria -m*x0 + y0

        x0 = tensao_x_carga[idx_menor_distancia];
        y0 = cargas_mapeadas[idx_menor_distancia];
        x1 = tensao_x_carga[idx_menor_distancia + 1];
        y1 = cargas_mapeadas[idx_menor_distancia + 1];        
        m = ( (y1-y0) / (x1 - x0) );       
        carga_bateria_float = ((m*tensao_bateria) - (m*x0) + y0);        
    }

    /* Caso a bateria esteja totalmente carregada, ainterpolação seja feita nos dois últimos níveis mapeados. 
     * Nesse caso, se o ADC apresentar algum erro de leitura para cima, a carga calculada poderá ser ligeiramente
     * maior que 100%. Nesse caso, trava-se a carga em 100%.  
     */
    if (carga_bateria_float > 100.0)
        carga_bateria_float = 100.0;

    carga_bateria = (char)carga_bateria_float;
    return carga_bateria;
}

/* Callbacks para uso cpm OTAA apenas (por este projeto usar ABP, isso, eles 
 *  estão vazios) */
void os_getArtEui (u1_t* buf) 
{ 
    /* Não utilizado neste projeto */  
}

void os_getDevEui (u1_t* buf) 
{ 
    /* Não utilizado neste projeto */  
}

void os_getDevKey (u1_t* buf) 
{ 
    /* Não utilizado neste projeto */  
}

/* Callback de evento: todo evento do LoRaAN irá chamar essa
   callback, de forma que seja possível saber o status da 
   comunicação com o gateway LoRaWAN. */
void onEvent (ev_t ev) 
{
    if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE )
    {
        Serial.print(os_getTime());
        Serial.print(": ");
        Serial.println(ev);
        xSemaphoreGive(xSerial_semaphore);
    }
    
    switch(ev) 
    {
        case EV_SCAN_TIMEOUT:
            break;
        case EV_BEACON_FOUND:
            break;
        case EV_BEACON_MISSED:
            break;
        case EV_BEACON_TRACKED:
            break;
        case EV_JOINING:
            break;
        case EV_JOINED:
            break;
        case EV_JOIN_FAILED:
            break;
        case EV_REJOIN_FAILED:
            break;
        case EV_TXCOMPLETE:
            if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) != pdTRUE )
            {
                break;
            }

            /* COntrole do semáforo serial obtido. Printa na serial as informações do evento. */
            Serial.println (millis());
            Serial.println(F("EV_TXCOMPLETE (incluindo espera pelas janelas de recepção)"));

            /* Verifica se ack foi recebido do gateway */
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Ack recebido"));

            /* Verifica se foram recebidos dados do gateway */  
            if (LMIC.dataLen > 0) 
            {
                Serial.println(F("Recebidos "));
                Serial.println(LMIC.dataLen);
                Serial.println(F(" bytes (payload) do gateway"));
              
                /* Como houve recepção de dados do gateway, os coloca
                   em um array para uso futuro. */
                uint8_t dados_recebidos = LMIC.frame[LMIC.dataBeg + 0];
                Serial.print(F("Dados recebidos: "));
                Serial.write(dados_recebidos);
            }

            /* Devolve o controle do semáforo da serial */
            xSemaphoreGive(xSerial_semaphore);
            
            break;

        case EV_LOST_TSYNC:
            break;
        case EV_RESET:
            break;
        case EV_RXCOMPLETE:
            break;
        case EV_LINK_DEAD:
            break;
        case EV_LINK_ALIVE:
            break;
        case EV_TXSTART:
            if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE )
            {
                Serial.println(F("EV_TXSTART"));
                Serial.println (millis());
                Serial.println(LMIC.freq);
                xSemaphoreGive(xSerial_semaphore);
            }
            break;
        default:
            break;
    }
}

/* Função para envio de dados ao gateway LoRaWAN */
void do_send(osjob_t* j, unsigned long total, unsigned long parcial)
{
    static uint8_t mydata[TAMANHO_DADOS_LORAWAN];    
    float tensao_bateria_mult_10;
    float tensao_bateria;
    char carga_bateria;
    TDados_lorawan dados_lorawan;
    BaseType_t resultado_peek_fila;

    /* le temperatura e pressao */
    esp_task_wdt_reset();

    do{
        resultado_peek_fila = xQueuePeek(xQueue_bateria, (void *)&tensao_bateria, TEMPO_PARA_LER_FILAS);        
    }while(resultado_peek_fila != pdTRUE);
    
    esp_task_wdt_reset();
    
    tensao_bateria_mult_10 = tensao_bateria*10.0;
    carga_bateria = calculo_carga_bateria(tensao_bateria);
   
    /* Formata dados a serem enviados na estrutura */    
    dados_lorawan.tensao_mult_10 = (char)(tensao_bateria_mult_10);
    dados_lorawan.carga_bateria = carga_bateria;    
    dados_lorawan.horimetro_total = total;
    dados_lorawan.horimetro_parcial = parcial;
    memcpy(mydata, (uint8_t *)&dados_lorawan, TAMANHO_DADOS_LORAWAN);
    
    /* Verifica se já há um envio sendo feito.
       Em caso positivo, o envio atual é suspenso. */
    if (LMIC.opmode & OP_TXRXPEND) 
    {
        if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE )
        {
            Serial.println(F("OP_TXRXPEND: ha um envio ja pendente, portanto o envio atual nao sera feito"));
            xSemaphoreGive(xSerial_semaphore);
        }
    } 
    else 
    {
        /* Aqui, o envio pode ser feito. */
        /* O pacote LoRaWAN é montado e o coloca na fila de envio. */
        LMIC_setTxData2(4, mydata, sizeof(mydata), 0);

        if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE )
        {
            Serial.println(F("Pacote LoRaWAN na fila de envio.")); 
            xSemaphoreGive(xSerial_semaphore);     
        }
    }
    esp_task_wdt_reset();
}

/* Função: grava na flash o horímetro total
 * Parâmetros: horimetro total (em segundos)
 * Retorno: nenhum
 */
void grava_horimetro_total(unsigned long ht)
{
    nvs_handle handler_particao_nvs;
    unsigned long ht_lido;
    esp_err_t err;
    int32_t bff;
    
    err = nvs_flash_init_partition("nvs");
    
    if (err != ESP_OK)
    {
        if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE )
        {
            Serial.println(F("Falha ao iniciar partição NVS.")); 
            xSemaphoreGive(xSerial_semaphore);     
        }
        
        return;
    }

    err = nvs_open_from_partition("nvs", "gr_lt", NVS_READWRITE, &handler_particao_nvs);
    if (err != ESP_OK)
    {
        if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE )
        {
            Serial.println(F("Falha ao abrir NVS como escrita/leitura")); 
            xSemaphoreGive(xSerial_semaphore);     
        }
        
        return;
    }

    /* Atualiza valor do horimetro total */
    err = nvs_set_i32(handler_particao_nvs, "ht", ht);

    if (err != ESP_OK)
    {
        if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE )
        {
            Serial.println(F("Erro ao gravar horimetro")); 
            xSemaphoreGive(xSerial_semaphore);     
        }
        
        nvs_close(handler_particao_nvs);
        return;
    }

    if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE )
    {
        Serial.println("Horimetro gravado com sucesso"); 
        xSemaphoreGive(xSerial_semaphore);     
    }
    
    nvs_commit(handler_particao_nvs);    
    nvs_close(handler_particao_nvs);
}

/* Função: le da flash o horímetro total
 * Parâmetros: nenhum
 * Retorno: horimetro total (em segundos)
 */
unsigned long le_horimetro_total(void)
{
    nvs_handle handler_particao_nvs;
    unsigned long ht_lido;
    esp_err_t err;
    int32_t bff;
    
    err = nvs_flash_init_partition("nvs");
    
    if (err != ESP_OK)
    {
        if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE )
        {
            Serial.println(F("Falha ao iniciar partição NVS.")); 
            xSemaphoreGive(xSerial_semaphore);     
        }
        
        return 0;
    }

    err = nvs_open_from_partition("nvs", "gr_lt", NVS_READWRITE, &handler_particao_nvs);
    if (err != ESP_OK)
    {
        if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE )
        {
            Serial.println(F("Falha ao abrir NVS como escrita/leitura")); 
            xSemaphoreGive(xSerial_semaphore);     
        }
        
        return 0;
    }

    /* Verifica se a key existe. 
       Se existir, le seu valor. 
       Se não existir, força o valor 0. 
    */
    err = nvs_get_i32(handler_particao_nvs, "ht", &bff);
    
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        /* Key não encontrada. Esta deve ser criada e inicializada em zero. */
        if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE )
        {
            Serial.println(F("Key não existe. Criando-a e inicializando-a em zero...")); 
            xSemaphoreGive(xSerial_semaphore);     
        }
        
        err = nvs_set_i32(handler_particao_nvs, "ht", 0);

        if (err != ESP_OK)
        {
            if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE )
            {
                Serial.println(F("Falha criar key")); 
                xSemaphoreGive(xSerial_semaphore);     
            }

            nvs_close(handler_particao_nvs);
            return 0; 
        }
        else
        {
            if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE )
            {
                Serial.println(F("Key criada e inicializada com sucesso")); 
                xSemaphoreGive(xSerial_semaphore);     
            }

            nvs_commit(handler_particao_nvs);
            nvs_close(handler_particao_nvs);
        }
    }
    else
    {
        /* Key encontrada. Faz a leitura. */
        err = nvs_get_i32(handler_particao_nvs, "ht", (int32_t *)&ht_lido);
       
        if (err != ESP_OK)
        {
            if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE )
            {
                Serial.println(F("Erro ao ler Key")); 
                xSemaphoreGive(xSerial_semaphore);     
            } 

            nvs_close(handler_particao_nvs);
            return 0;
        }
        else
        {
            if ( xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE )
            {
                Serial.println("Key lida com sucesso"); 
                xSemaphoreGive(xSerial_semaphore);     
            }

            nvs_close(handler_particao_nvs);
            return ht_lido;
        }
    }
}

void setup() 
{
    /* Inicializa serial de debug */
    Serial.begin(BAUDRATE_SERIAL_DEBUG);

    /* Inicializa I²C (para comunicação com OLED e BMP180)*/
    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    
    /* Inicializa display */
    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) 
    { 
        Serial.println("[ERRO] não foi possivel inicializar display. O NodeMCU será reiniciado em 1s...");
        delay(1000);
        ESP.restart();
    }
    else
    {
        Serial.println("Display inicializado.");
        
        display.clearDisplay();
        display.setTextColor(WHITE);
        display.setTextSize(1);
        display.setCursor(0,OLED_LINHA_1);
        display.println("Inicializando...");
        display.display();
    }

    /* Criação dos semáforos para serial e I²C (usados para comunicação com OLED e BMP180) */
    xSerial_semaphore = xSemaphoreCreateMutex();
    xI2C_semaphore = xSemaphoreCreateMutex();

    if ( (xI2C_semaphore == NULL) || (xSerial_semaphore == NULL) )
    {
        Serial.println("Falha ao criar semáforos.");
        delay(1000);
        ESP.restart(); 
    }

    /* Criação das filas */
    xQueue_display = xQueueCreate( 1, sizeof( TTela_display ) );
    xQueue_bateria = xQueueCreate( 1, sizeof( float ) );
    xQueue_tempo_horimetro = xQueueCreate( 1, sizeof( TTempos_horimetro ) );
    xQueue_envio_lorawan = xQueueCreate( 1, sizeof( TTempos_horimetro ) );

    if ( (xQueue_display == NULL) || (xQueue_bateria == NULL) ||
         (xQueue_tempo_horimetro == NULL) || (xQueue_envio_lorawan == NULL) )
    {
        Serial.println("Falha ao criar filas.");
        delay(1000);
        ESP.restart();  
    } 

    /* Inicia o Task WDT com 60 segundos */
    esp_task_wdt_init(60, true); 

    /* Agenda execução das tarefas */
    xTaskCreatePinnedToCore(task_oled,  
                           "oled", 
                           STACK_SIZE_OLED, 
                           NULL,  
                           PRIO_OLED, 
                           NULL, 
                           1);     

    xTaskCreate(task_horimetro ,
                "horimetro",
                STACK_SIZE_HORIMETRO,   
                NULL,
                PRIO_HORIMETRO,  
                NULL );

    xTaskCreate(task_envio_lorawan ,
                "lorawan",
                STACK_SIZE_LORAWAN,   
                NULL,
                PRIO_LORAWAN,  
                NULL );
                           
    xTaskCreate(task_formata_medicoes_display ,
                "formata_tela",
                STACK_SIZE_FORMATA_LINHAS,   
                NULL,
                PRIO_FORMATA_LINHAS,  
                NULL );                                                     

    xTaskCreate(task_bateria ,
                "bateria",
                STACK_SIZE_BATERIA,   
                NULL,
                PRIO_BATERIA,  
                NULL );                                                                 
                
}

void loop() 
{
    /* Nada é feito aqui. As tarefas cuidam de tudo. */     
}

/* Tarefa responsável por atualizar display OLED */
void task_oled( void *pvParameters )
{
    TTela_display tela_display;

    /* Habilita o monitoramento do Task WDT nesta tarefa */
    esp_task_wdt_add(NULL); 
            
    while(1)
    {
        if ( xSemaphoreTake(xI2C_semaphore, TEMPO_PARA_OBTER_SEMAFORO ) != pdTRUE )
        {
            /* Alimenta WDT e tenta novamente */
            esp_task_wdt_reset();
            continue;
        }
        
        if (xQueueReceive(xQueue_display, (void *)&tela_display, TEMPO_PARA_LER_FILAS) == pdTRUE) 
        {
            display.clearDisplay();
            display.setCursor(0,OLED_LINHA_1);
            display.println("       Leituras");
            display.setCursor(0,OLED_LINHA_2);
            display.print(tela_display.linha2);
            display.setCursor(0,OLED_LINHA_3);
            display.print(tela_display.linha3);
            display.setCursor(0,OLED_LINHA_4);
            display.print(tela_display.linha4);
            display.display();
        }

        xSemaphoreGive(xI2C_semaphore);
        esp_task_wdt_reset();
        vTaskDelay( TEMPO_REFRESH_DISPLAY / portTICK_PERIOD_MS ); 
    }
}

/* Tarefa responsavel por receber todas as medições e colocá-las (formatadas para o display)
   em uma estrutura */
void task_formata_medicoes_display( void *pvParameters )
{
    TTela_display tela_display;
    BaseType_t resultado_envio_fila_display;
    float tensao_bateria;
    unsigned long contador_tempo_segundos_total;
    unsigned long contador_tempo_segundos_parcial;
    unsigned long horas_total, horas_parcial;
    int minutos_total, minutos_parcial;
    int segundos_total, segundos_parcial;
    TTempos_horimetro tempos_horimetro;
    
    /* Tempo para comunciação I²C com display acontecer sem problemas */
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); 

    /* Habilita o monitoramento do Task WDT nesta tarefa */
    esp_task_wdt_add(NULL); 

    while(1)
    {
        esp_task_wdt_reset();
        
        xQueuePeek(xQueue_bateria, (void *)&tensao_bateria, TEMPO_PARA_LER_FILAS);
        xQueuePeek(xQueue_tempo_horimetro, (void *)&tempos_horimetro, TEMPO_PARA_LER_FILAS);

        /* Horímetro total */
        contador_tempo_segundos_total = tempos_horimetro.horimetro_total;
        horas_total = contador_tempo_segundos_total / 3600;
        contador_tempo_segundos_total = contador_tempo_segundos_total % 3600;
        minutos_total =  contador_tempo_segundos_total / 60;
        contador_tempo_segundos_total = contador_tempo_segundos_total % 60;
        segundos_total = contador_tempo_segundos_total;

        /* Horímetro parcial */
        contador_tempo_segundos_parcial = tempos_horimetro.horimetro_parcial;
        horas_parcial = contador_tempo_segundos_parcial / 3600;
        contador_tempo_segundos_parcial = contador_tempo_segundos_parcial % 3600;
        minutos_parcial =  contador_tempo_segundos_parcial / 60;
        contador_tempo_segundos_parcial = contador_tempo_segundos_parcial % 60;
        segundos_parcial = contador_tempo_segundos_parcial;        

        sprintf(tela_display.linha2, "HT: %ld:%02d:%02d", horas_total, minutos_total, segundos_total);
        sprintf(tela_display.linha3, "HP: %ld:%02d:%02d", horas_parcial, minutos_parcial, segundos_parcial);
        sprintf(tela_display.linha4, "Bat: %.2fV", tensao_bateria);
        

        esp_task_wdt_reset();
        
        do 
        {
            resultado_envio_fila_display = xQueueSend(xQueue_display, (void *)&tela_display, TEMPO_PARA_INSERIR_FILAS);
        }while (resultado_envio_fila_display != pdTRUE);

        esp_task_wdt_reset();
        
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }
}

/* Tarefa responsavel por enviar via LoRaWAN as medições */
void task_envio_lorawan( void *pvParameters )
{
    int b;
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    TTempos_horimetro tempos_horimetro;
    
    /* Inicializa comunicação SPI com rádio LoRa */
    SPI.begin(RADIO_SCLK_PORT, RADIO_MISO_PORT, RADIO_MOSI_PORT);

    /* Inicializa stack LoRaWAN */
    os_init();
    LMIC_reset();

    /* Inicializa chaves usadas na comunicação ABP */
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);

    /* Faz inicializações de rádio pertinentes a região do 
       gateway LoRaWAN (ATC / Everynet Brasil) */
    for (b=0; b<8; ++b) 
        LMIC_disableSubBand(b);

    LMIC_enableChannel(0); // 915.2 MHz
    LMIC_enableChannel(1); // 915.4 MHz
    LMIC_enableChannel(2); // 915.6 MHz
    LMIC_enableChannel(3); // 915.8 MHz
    LMIC_enableChannel(4); // 916.0 MHz
    LMIC_enableChannel(5); // 916.2 MHz
    LMIC_enableChannel(6); // 916.4 MHz
    LMIC_enableChannel(7); // 916.6 MHz

    LMIC_setAdrMode(0);
    LMIC_setLinkCheckMode(0);

    /* Data rate para janela de recepção RX2 */
    LMIC.dn2Dr = DR_SF12CR;

    /* Configura data rate de transmissão e ganho do rádio
       LoRa (dBm) na transmissão */
    LMIC_setDrTxpow(DR_SF12, GANHO_LORA_DBM);

    /* Habilita o monitoramento do Task WDT nesta tarefa */
    esp_task_wdt_add(NULL); 

    while(1)
    {
        esp_task_wdt_reset();
        
        /* Envia um pacote LoRaWAN de acordo com periodicidade definida em TX_INTERVAL */
        if( xQueueReceive(xQueue_envio_lorawan, (void *)&tempos_horimetro, ( TickType_t )1) == pdTRUE )
        {        
            do_send(&sendjob, tempos_horimetro.horimetro_total, tempos_horimetro.horimetro_parcial);
        }

        os_runloop_once();

        vTaskDelay( ( TickType_t )1 );
    }
}

/* Tarefa responsavel por ler a tensão da bateria */
void task_bateria( void *pvParameters )
{
    float tensao_bateria;
        
    /* Configura ADC da bateria */
    configura_adc_bateria(); 

    /* Habilita o monitoramento do Task WDT nesta tarefa */
    esp_task_wdt_add(NULL);

    while(1)
    {
        esp_task_wdt_reset();
        
        /* Le o ADC considerando sua calibração */
        tensao_bateria = le_tensao_bateria();
        xQueueOverwrite(xQueue_bateria, (void *)&tensao_bateria);
        vTaskDelay( TEMPO_ENTRE_LEITURAS_BATERIA / portTICK_PERIOD_MS );
    }
}

/* Task responsável por:
   - Start / stop da contabilização de tempo
   - Contabilizar tempo
*/
void task_horimetro( void *pvParameters )
{
    EventBits_t evbits_gpio;
    EventBits_t evbits_timer_1s;
    TTempos_horimetro tempos_horimetro;
    bool deve_resetar_horimetro = false;
    unsigned long t_ref = 0;

    /* Habilita o monitoramento do Task WDT nesta tarefa */
    esp_task_wdt_add(NULL);

    /* Verifica se horimetro deve ser resetado.
       A condição para reset é o GPIO do horimetro ficar em nivel alto pelo tempo definido em TEMPO_PARA_RESETAR_HORIMETRO
    */
    pinMode(GPIO_HORIMETRO, INPUT);

    t_ref = millis();
    
    while( digitalRead(GPIO_HORIMETRO) == HIGH)
    {
        if (diferenca_tempo(t_ref) >= TEMPO_PARA_RESETAR_HORIMETRO)
        {
           deve_resetar_horimetro = true;

           if (xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE)
           {  
                Serial.println("O horimetro sera resetado.");
                xSemaphoreGive(xSerial_semaphore);
           }
           
           break;  
        }            
    }

    if (deve_resetar_horimetro == true)
    {
        /* Reseta horimetro */
        tempos_horimetro.horimetro_total = 0;
        grava_horimetro_total(0);
    }
    else
    {
        /* Faz a leitura do horimetro total */
        tempos_horimetro.horimetro_total = le_horimetro_total();
    }
    /* Cria timer (auto-reload) para contabilizar tempo */
    timer_1s = xTimerCreate("timer_1_segundo", 
                            pdMS_TO_TICKS(1000), 
                            pdTRUE, 
                            ID_TIMER_1S, 
                            ISR_timer_1_segundo);

    /* Cria timer (one-shot) para debounce do GPIO */
    timer_debounce_gpio = xTimerCreate("timer_debounce", 
                                       pdMS_TO_TICKS(GPIO_TEMPO_DEBOUNCE_MS), 
                                       pdFALSE, 
                                       ID_TIMER_GPIO, 
                                       ISR_timer_debounce);

    /* Cria o event groups para GPIO do horímetro e do timer de 
       um segundo */
    evt_gpio = xEventGroupCreate();
    evt_1s = xEventGroupCreate();
 
    esp_task_wdt_reset();

    while(1)
    {
        tempos_horimetro.horimetro_parcial = 0;
        xQueueOverwrite(xQueue_tempo_horimetro, (void *)&tempos_horimetro);
        
        /* Configura GPIO do horímetro como entrada e cria interrupção 
           (modo definido por GPIO_INT_START */
        attachInterrupt(GPIO_HORIMETRO, ISR_start_horimetro, GPIO_INT_START);

        if (xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE)
        {  
            Serial.println("** Aguardando GPIO do horimetro indicar start");
            xSemaphoreGive(xSerial_semaphore);
        }
    
        esp_task_wdt_reset();
         
        /* Aguarda evento sinalizando que GPIO foi para nível alto */
        do {
            evbits_gpio = xEventGroupWaitBits(evt_gpio,            //handler do event group
                                              EV_GPIO,             //Máscara com GPIO a ser considerado
                                              pdTRUE,              //Limpa automaticamente o bit ao retornar
                                              pdFALSE,             //Retorna quando qualquer um dos bits for setado 
                                              pdMS_TO_TICKS(10));  //Aguarda por até 10ms para o bit ser setado    
    
            esp_task_wdt_reset();
        }while( (evbits_gpio & EV_GPIO) == 0);

        if (xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE)
        {  
            Serial.println("** GPIO do horimetro indicou start");
            xSemaphoreGive(xSerial_semaphore);
        }
    
        esp_task_wdt_reset();
        
        /* GPIO sinalizou via evento que horímetro deve começar a contabilizar tempo.
           As seguintes ações devem ser feitas: 
           1- O timer de debounce de GPIO deve parar
           2- o modo da interrupção do GPIO deve mudar para GPIO_INT_STOP.
           3- deve ser iniciado um timer de 1s (com auto-reload) para
              contabilização de tempo.
           4- a contagem de tempo deve ser enviada, visa fila, para tarefa
              de formatação das linhas do OLED.
           5- se o GPIO sinalizar (através de evento) que é para parar o
              horímetro, a contagem total é enviada por LoRaWAN a nuvem.     
           6- parar timer de 1 segundo
        */
        xTimerStop(timer_debounce_gpio, pdMS_TO_TICKS(10));   
        detachInterrupt(GPIO_HORIMETRO);
        attachInterrupt(GPIO_HORIMETRO, ISR_stop_horimetro, GPIO_INT_STOP);
                                
        xTimerStart(timer_1s, pdMS_TO_TICKS(10));
                               
        esp_task_wdt_reset();

        if (xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE)
        {  
            Serial.println("** Aguardando GPIO do horimetro indicar stop");
            xSemaphoreGive(xSerial_semaphore);
        }

        do {
            evbits_gpio = xEventGroupWaitBits(evt_gpio,           
                                              EV_GPIO,            
                                              pdTRUE,             
                                              pdFALSE,            
                                              pdMS_TO_TICKS(1));

            evbits_timer_1s = xEventGroupWaitBits(evt_1s,           
                                                  EV_1S,            
                                                  pdTRUE,             
                                                  pdFALSE,            
                                                  pdMS_TO_TICKS(1));
            
            if ( evbits_timer_1s & EV_1S)
            {
                tempos_horimetro.horimetro_parcial++; 
                tempos_horimetro.horimetro_total++;
                xQueueOverwrite(xQueue_tempo_horimetro, (void *)&tempos_horimetro);          

                if (xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE)
                {  
                    Serial.print("- Tempo total: ");
                    Serial.print(tempos_horimetro.horimetro_total);
                    Serial.println("s");
                    xSemaphoreGive(xSerial_semaphore);
                }
            }

            esp_task_wdt_reset();
        }while( (evbits_gpio & EV_GPIO) == 0);
        
        detachInterrupt(GPIO_HORIMETRO);
        xTimerStop(timer_1s, pdMS_TO_TICKS(10));
        xQueueOverwrite(xQueue_tempo_horimetro, (void *)&tempos_horimetro);
        xQueueSend(xQueue_envio_lorawan, (void *)&tempos_horimetro, TEMPO_PARA_INSERIR_FILAS);
        grava_horimetro_total(tempos_horimetro.horimetro_total);

        if (xSemaphoreTake(xSerial_semaphore, TEMPO_PARA_OBTER_SEMAFORO) == pdTRUE)
        {  
            Serial.println("** GPIO do horimetro indicou stop");
            xSemaphoreGive(xSerial_semaphore);
        }
        
        esp_task_wdt_reset();
    }
}
