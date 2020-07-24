# Projeto - horímetro com FreeRTOS, LoRaWAN e ESP32

Este repositório contém um projeto de um horímetro (total e parcial) na forma de um end-device LoRaWAN (ABP), preparado para se conectar a rede pública da ATC (American Tower Corporation). 
No quesito hardware, este projeto utiliza a placa de desenvolvimento Heltec WiFi LoRa v1 (alimentado a bateria ou via cabo USB) e um push-button simulando a entrada de start / stop do horímetro (conectado ao GPIO 36). Esta placa de desenvolvimento conta com o SX1276 como rádio LoRa e o ESP32 como SoC.
Todo o software embarcado é feito utilizando o FreeRTOS como sistema operacional embarcado. Esse projeto deve ser aberto e compilado na Arduino IDE. O projeto possui as seguintes finalidades:

* Contabilização de horímetro total (salvo em flash)
* Contabilização de horímetro parcial
* Leitura periódica da tensão de bateria (V) e carga da bateria (0 .. 100%)
* Exibição das medições no display OLED 128x64 contido na placa de desenvolvimento.

O envio dos horímetros parcial e total é feito ao fim da contabilização de tempo parcial.

## Limites

Os horímetros parcial e total possuem os seguintes limites de tempo:

* Horímetro parcial: 1.193.046 horas
* Horímetro total: 1.193.046 horas

## Unidade de medição

Visando maior gama de utilização, tanto o horímetro parcial quanto o total contabiliza o tempo em segundos. Logo, o horímetro possui resolução de 1 segundo.

## Funcionamento - contabilização de tempo e envio por LoRaWAN

O funcionamento do projeto é conforme descrito a seguir:

1) O dispositivo inicia exibindo no display o horímetro total (lido da memória flash), horímetro parcial e tensão de bateria
2) Quando o GPIO 36 vai para nível alto (rising), a contagem começa, incrementando tanto o horímetro total quanto o parcial
3) Quando o GPIO 36 vai para nível baixo (falling), a contagem de tempo para. Nessa hora, é gravado em flash o horímetro total e é enviado, via LoRaWAN, os horímetros parcial e total para a nuvem.

## Funcionamento - reset do horímetro total

Para resetar / zerar o horímetro total, o procedimento é o seguinte:

1) Com o projeto desligado, faça o GPIO 36 ir a nível alto.
2) Ligue o projeto e aguarde 6 segundos.
3) O horímetro total foi resetado (e é exibido como 0:00:00 no display OLED).

## IMPORTANTE
1) Este projeto considera a tensão da bateria lida no GPIO37 (ADC1_1), onde a tensão é lida num divisor de tensão 
  (resistor de 470k / 0,25W e resistor de 100k / 0,25W). 
 
NÃO SE ESQUEÇA DE USAR O DIVISOR DE TENSÃO AQUI!! O ADC do ESP32 suporta, no máximo, 1,1V (0dB), 
enquanto a tensão de bateria pode chegar a 4,2V.
 
2) Esse projeto faz uso da biblioteca "MCCI LoRaWAN LMIC Library". Este projeto foi testado com a versão 2.3.2 da mesma.
3) Antes de compilar, é preciso deixar o arquivo lmic_project_config.h (dentro na pasta da biblioteca: project_config/lmic_project_config.h) com o conteúdo conforme abaixo:
```
// project-specific definitions
//#define CFG_eu868 1
//#define CFG_us915 1
#define CFG_au921 1
//#define CFG_as923 1
// #define LMIC_COUNTRY_CODE LMIC_COUNTRY_CODE_JP      
//#define CFG_in866 1
#define CFG_sx1276_radio 1
//#define LMIC_USE_INTERRUPTS
```
4) Você precisará da network session key, application session key (definidos por você ou pela operadora da ATC) e do Device Address (fornecido pela operadora em caráter experimental ou adquirido por meios oficiais). Substitua estas informações no arquivo LORAWAN_defs.h.
Para obtenção das chaves e tudo mais em termos de conectividade 
LoRaWAN, entre em contato com uma das empresas credenciadas pela ATC:
 
https://iotopenlabs.io/home/catalogo-de-solucoes/conectividade-lorawan/

5) O circuito esquemático encontra-se na pasta "ckt_esquematico", tanto como figura como no formato Fritzing.

Este projeto é de autoria de Pedro Bertoleti. 

Agradecimentos as seguintes pessoas:
* Professor Marcelus Guirardello (ETEC - Bento Quirino - Campinas-SP), por toda a ajuda na codificação da comunicação LoRaWAN.
* José Morais, por toda a ajuda com revisão de código e troca de ideias sobre rotinas de watch dog, event groups e NVS.
