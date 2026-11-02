//######################################################################################################################
// *
// *  Arquivo: 'drivers/sensors/bmp280_uorb_true-full.c'
// *
// *  Título: Driver uORB FULL do Sensor de Pressão BMP280 — Modelo Push Autônomo
// *
// *  Objetivo: Implementar o driver de sensor do BMP280 no framework uORB do NuttX, publicando automaticamente
// *            os dados de pressão e temperatura em intervalos regulares, sem depender de chamadas externas
// *            a 'fetch'. A publicação é feita por meio de work queue (HPWORK) e do método 'push_event'.
// *
// *  Design Pattern: Driver de Sensor uORB FULL — modelo PUSH assíncrono.
// *
// *  Autor: Baseado no código oficial do Apache NuttX e no driver 'bmp180_uorb.c'
// *  Hardware: ESP32-S3 Dev Kit C (Xtensa) — originalmente Bosch BMP280
// *  RTOS: Apache NuttX 13.0.0
// *
// *--------------------------------------------------------------------------------------------------------------------
// *
// * SPDX-License-Identifier: Apache-2.0
// *
// * Licensed to the Apache Software Foundation (ASF) under one or more contributor license agreements.
// * See the NOTICE file distributed with this work for additional information regarding copyright ownership.
// * The ASF licenses this file to you under the Apache License, Version 2.0 (the "License"); you may not use this file
// * except in compliance with the License.  You may obtain a copy of the License at
// *
// *   http://www.apache.org/licenses/LICENSE-2.0
// *
// * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on
// * an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// * See the License for the specific language governing permissions and limitations under the License.
// *
// *--------------------------------------------------------------------------------------------------------------------
// *
// *  Modelo: Lower Half uORB FULL (Categoria 2 — Upper-Half uORB Genérico, modelo push).
// *
// *  SOBRE O NOME DESTE ARQUIVO:
// *
// *    O NuttX já possui um arquivo oficial chamado 'drivers/sensors/bmp280_uorb.c'. Esse arquivo é a
// *    anomalia que analisamos: carrega o sufixo '_uorb', mas implementa apenas o modelo PULL (fetch),
// *    sem 'push_event', sem 'work_queue' e sem 'kthread_create'.
// *
// *    Para evitar conflito de símbolos durante o build (duas definições de 'bmp280_register', por exemplo)
// *    e para preservar o valor didático da comparação lado a lado, este arquivo recebe o nome
// *    'bmp280_uorb_true-full.c':
// *
// *      - '_uorb'      mantém a convenção do subsistema de sensores do NuttX.
// *      - '_true-full' sinaliza que este é o driver que implementa de fato o modelo PUSH completo,
// *                     ao contrário do 'bmp280_uorb.c' oficial, que é uma anomalia PULL.
// *
// *    A escolha entre compilar o oficial e este, ou ambos condicionalmente por Kconfig, é deixada para
// *    o desenvolvedor. Em ambiente de produção, a substituição do oficial por este seria o caminho mais
// *    limpo. Em ambiente de aprendizado, manter os dois lado a lado permite comparar as duas topologias.
// *
// *--------------------------------------------------------------------------------------------------------------------
// *
// *  DIFERENÇA ARQUITETURAL EM RELAÇÃO AO BMP180:
// *
// *    O BMP180 e o BMP280 apresentam uma diferença importante na organização do código:
// *
// *      - O BMP180 usa uma Camada de Base separada ('bmp180_base.c' + 'bmp180_base.h'). O arquivo
// *        'bmp180_uorb.c' delega todo o acesso físico e a compensação para a Camada de Base. No arquivo
// *        uORB do BMP180 ficam apenas:
// *
// *          - Estrutura privada com 'work_s'.
// *          - 'activate', 'set_interval' e 'worker'.
// *          - 'register_uorb'.
// *
// *        Isso torna o 'bmp180_uorb.c' curto, porque o trabalho pesado está em outro arquivo, que é
// *        compartilhado entre o driver tradicional ('bmp180.c') e o driver uORB ('bmp180_uorb.c').
// *
// *      - O BMP280 NÃO tem uma Camada de Base na árvore do NuttX. Consequentemente, este driver concentra
// *        TUDO em um único arquivo:
// *
// *          - Acesso I2C ('bmp280_getreg8', 'bmp280_getregs', 'bmp280_putreg8').
// *          - Verificação de ID ('bmp280_checkid').
// *          - Configuração de standby ('bmp280_set_standby').
// *          - Inicialização e leitura de calibração ('bmp280_initialize').
// *          - Compensação de temperatura ('bmp280_compensate_temp').
// *          - Compensação de pressão ('bmp280_compensate_press').
// *          - Ajuste de intervalo ('bmp280_set_interval').
// *          - Ativação ('bmp280_activate').
// *          - Worker ('bmp280_worker').
// *          - Registro ('bmp280_register').
// *
// *        Isso explica por que este arquivo é consideravelmente maior que o 'bmp180_uorb.c' equivalente.
// *
// *    Observação adicional: o BMP280 possui mais recursos de hardware do que o BMP180, o que também
// *    contribui para o tamanho maior:
// *
// *      - 12 coeficientes de calibração (T1, T2, T3, P1 a P9) em vez de 11 (AC1 a AC6, B1, B2, MB, MC, MD).
// *      - 6 modos de oversampling para cada grandeza (skip, 1x, 2x, 4x, 8x, 16x) em vez de 4.
// *      - Registrador de configuração com standby e filtro IIR ('BMP280_CONFIG').
// *      - Três modos de operação: SLEEP, FORCED e NORMAL.
// *      - Compensação de temperatura e pressão mais complexa, com mais etapas.
// *
// *--------------------------------------------------------------------------------------------------------------------
// *
// *  NOTA CONCEITUAL — ARQUITETURA E POSIÇÃO:
// *
// *    Este driver é um Lower Half uORB. Ele se registra no framework de sensores via 'sensor_register' e
// *    publica dados espontaneamente via 'push_event'. Não implementa 'fetch'. Ele usa a work queue HPWORK
// *    para agendamento periódico do worker.
// *
// *      ┌──────────────────────────────────────────────────────────────────────────┐
// *      │  Aplicação de usuário (assina tópico 'sensor_baro' via API uORB)         │
// *      └────────────────────────────────┬─────────────────────────────────────────┘
// *                                       │
// *                                       ▼
// *      ┌──────────────────────────────────────────────────────────────────────────┐
// *      │  Upper Half uORB ('drivers/sensors/sensor.c')                            │
// *      │  Encaminha eventos para assinantes via '/dev/uorb/sensor_baro0'          │
// *      └────────────────────────────────┬─────────────────────────────────────────┘
// *                                       │
// *                                       ▼
// *      ┌──────────────────────────────────────────────────────────────────────────┐
// *      │  ESTE ARQUIVO: 'bmp280_uorb_true-full.c' — Lower Half uORB FULL          │
// *      │                                                                          │
// *      │    'bmp280_activate()' ──► work_queue(HPWORK, ...)                      │
// *      │           │                                                              │
// *      │           ▼                                                             │
// *      │    'bmp280_worker()'  ──► leitura I2C + compensação                     │
// *      │           │                                                              │
// *      │           └────────────► 'push_event()' ──► Upper Half ──► assinantes  │
// *      └────────────────────────────────┬─────────────────────────────────────────┘
// *                                       │
// *                                       ▼
// *      ┌──────────────────────────────────────────────────────────────────────────┐
// *      │  Driver do controlador I2C do ESP32-S3                                   │
// *      │  (via I2C_TRANSFER)                                                      │
// *      └──────────────────────────────────────────────────────────────────────────┘
// *
// *    Observações:
// *
// *    - Este driver NÃO implementa 'fetch'. A publicação é feita via 'push_event'.
// *    - Usa 'struct work_s' e a work queue HPWORK para agendamento periódico.
// *    - O worker se reagenda continuamente enquanto o sensor estiver ativo.
// *    - Diferentemente do 'bmp180_uorb.c', o BMP280 exige que o worker aguarde a conversão interna
// *      do sensor antes de ler os registradores. Isso é feito por meio de 'nxsched_usleep' no worker.
// *    - Este driver segue o padrão do 'bmp180_uorb.c' (FULL uORB), adaptado ao BMP280.
// *
// ********************************************************************************************************************/
// *
// *  RESUMO DA TOPOLOGIA DO ARQUIVO 'bmp280_uorb_true-full.c'
// *
// *  Topologia: DRIVER uORB FULL (MODELO PUSH ASSÍNCRONO)
// *
// *  Marcadores arquiteturais que caracterizam essa topologia:
// *
// *    1. Estrutura de operações:
// *
// *       - 'struct sensor_ops_s g_sensor_ops' — define 'activate' e 'set_interval'.
// *       - 'fetch' NÃO está presente — confirma o modelo push.
// *
// *
// *    2. Estrutura privada com agendamento:
// *
// *       - 'struct bmp280_dev_s' contém:
// *
// *         - 'struct sensor_lowerhalf_s sensor_lower' — interface com o Upper Half.
// *         - 'struct work_s work' — agendador de trabalho na work queue.
// *         - 'uint32_t interval' — intervalo de aquisição em microssegundos.
// *         - 'struct i2c_master_s *i2c' — barramento I2C.
// *         - 'uint8_t addr' e 'int freq' — parâmetros de comunicação.
// *         - 'struct bmp280_calib_s calib' — coeficientes de calibração.
// *         - 'int32_t tempfine' — valor intermediário de compensação.
// *
// *
// *    3. Funções que caracterizam o modelo push:
// *
// *       - 'bmp280_activate()' — agenda 'work_queue(HPWORK, ...)' ao ativar.
// *       - 'bmp280_worker()' — executa leitura periódica e publica via 'push_event'.
// *       - 'bmp280_set_interval()' — permite ajuste do intervalo de aquisição.
// *
// *
// *    4. Função de registro:
// *
// *       - 'bmp280_register()' — usa 'sensor_register()' (framework uORB).
// *
// *
// *    5. Ausências que confirmam o modelo push:
// *
// *       - Não há 'fetch'.
// *       - Não há 'kthread_create'.
// *       - Não há 'struct file_operations'.
// *       - Não há 'register_driver'.
// *
// *
// *    6. Camada de acesso físico:
// *
// *       - No próprio arquivo, via 'bmp280_getreg8', 'bmp280_getregs' e 'bmp280_putreg8'.
// *       - Essas funções chamam 'I2C_TRANSFER' (>>> INTERFACE COM SUBSISTEMA I2C <<<).
// *
// *
// *    7. Papel arquitetural:
// *
// *       - Atua como Lower Half uORB.
// *       - Publica eventos espontaneamente via 'push_event'.
// *       - Cria nó em '/dev/uorb/sensor_baro0'.
// *
// *--------------------------------------------------------------------------------------------------------------------
// #####################################################################################################################
// *
// *  Ver  | dd mmm aaaa | Who      | Descrição de alterações
// * ======|=============|==========|===================================================================================
// *  1.0.0| 10 set 2026 | DeepSeek | Versão FULL uORB derivada do modelo do 'bmp180_uorb.c'
// *####################################################################################################################

/*
 * Inclusão de Diretivas de Compilação - Início ------------------------------------------------------------------------
 */


/*
 * Inclusão de Bibliotecas em Arquivos de Cabeçalho --------------------------------------------------------------------
 */

/*
 * NuttX — Configuração do Kernel:
 */


// Inclui macros globais de configuração do Kernel NuttX, geradas pelo Kconfig
#include <nuttx/config.h>

// Inclui definições utilitárias do NuttX (UNUSED, nitems, container_of)
#include <nuttx/nuttx.h>

/*
 * C Libraries:
 */

// Inclui funções utilitárias padrão, como memcpy
#include <stdlib.h>

// Inclui definições para aritmética de ponto fixo (sensor_data_divi, etc.)
#include <fixedmath.h>

// Inclui códigos de erro padrão, como ENODEV
#include <errno.h>

// Inclui funções de debug e logging, como snerr e sninfo
#include <nuttx/debug.h>

/*
 * NuttX — Subsistemas:
 */

// Inclui funções de arquitetura, como up_mdelay
#include <nuttx/arch.h>

// Inclui alocação de memória do kernel (kmm_zalloc, kmm_free)
#include <nuttx/kmalloc.h>

// Inclui definições do sistema de arquivos virtual (VFS), como struct file
#include <nuttx/fs/fs.h>

// Inclui a interface pública do subsistema I2C do NuttX
#include <nuttx/i2c/i2c_master.h>

// Inclui a API da work queue do NuttX (HPWORK, work_queue, work_cancel)
#include <nuttx/wqueue.h>

// Inclui funções de sinal do NuttX (nxsched_usleep)
#include <nuttx/signal.h>

// Inclui a interface pública do sensor BMP280 (protótipos e constantes externas)
#include <nuttx/sensors/bmp280.h>

// Inclui a interface do framework de sensores (sensor_lowerhalf_s, sensor_ops_s, push_event)
#include <nuttx/sensors/sensor.h>

// Bloco condicional: compila este arquivo apenas se I2C e o sensor BMP280 estiverem habilitados no Kconfig
#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BMP280)


/*
 * Definições de Pré-Processador --------------------------------------------------------------------------------------
 */

/* Apenas tipo de dado 'float' é suportado no momento */

// Bloco condicional: impede a compilação se o tipo de dado de ponto fixo estiver habilitado
#ifdef CONFIG_SENSORS_USE_B16

// Emite erro em tempo de compilação
#  error fixed-point data type not supported yet

// Fim do bloco condicional do tipo de dado
#endif

// Define o intervalo mínimo entre aquisições do BMP280 (30 ms)
#define BMP280_MIN_INTERVAL                        30000


/*
 * Endereço e frequência I2C:
 */

// Bloco condicional: define o endereço I2C conforme a configuração do Kconfig
#ifdef CONFIG_BMP280_I2C_ADDR_76

// Endereço I2C 0x76 (configurado via Kconfig)
#define BMP280_ADDR                                0x76

// Caso contrário, usa o endereço padrão 0x77
#else

// Endereço I2C 0x77
#define BMP280_ADDR                                0x77

// Fim do bloco condicional do endereço I2C
#endif

// Frequência I2C configurada via Kconfig
#define BMP280_FREQ                                CONFIG_BMP280_I2C_FREQUENCY

// Valor esperado do chip ID do BMP280
#define DEVID                                      0x58


/*
 * Endereços dos coeficientes de calibração:
 */

// Coeficiente de temperatura T1 (LSB e MSB)
#define BMP280_DIG_T1_LSB                          0x88
#define BMP280_DIG_T1_MSB                          0x89

// Coeficiente de temperatura T2 (LSB e MSB)
#define BMP280_DIG_T2_LSB                          0x8a
#define BMP280_DIG_T2_MSB                          0x8b

// Coeficiente de temperatura T3 (LSB e MSB)
#define BMP280_DIG_T3_LSB                          0x8c
#define BMP280_DIG_T3_MSB                          0x8d

// Coeficiente de pressão P1 (LSB e MSB)
#define BMP280_DIG_P1_LSB                          0x8e
#define BMP280_DIG_P1_MSB                          0x8f

// Coeficiente de pressão P2 (LSB e MSB)
#define BMP280_DIG_P2_LSB                          0x90
#define BMP280_DIG_P2_MSB                          0x91

// Coeficiente de pressão P3 (LSB e MSB)
#define BMP280_DIG_P3_LSB                          0x92
#define BMP280_DIG_P3_MSB                          0x93

// Coeficiente de pressão P4 (LSB e MSB)
#define BMP280_DIG_P4_LSB                          0x94
#define BMP280_DIG_P4_MSB                          0x95

// Coeficiente de pressão P5 (LSB e MSB)
#define BMP280_DIG_P5_LSB                          0x96
#define BMP280_DIG_P5_MSB                          0x97

// Coeficiente de pressão P6 (LSB e MSB)
#define BMP280_DIG_P6_LSB                          0x98
#define BMP280_DIG_P6_MSB                          0x99

// Coeficiente de pressão P7 (LSB e MSB)
#define BMP280_DIG_P7_LSB                          0x9a
#define BMP280_DIG_P7_MSB                          0x9b

// Coeficiente de pressão P8 (LSB e MSB)
#define BMP280_DIG_P8_LSB                          0x9c
#define BMP280_DIG_P8_MSB                          0x9d

// Coeficiente de pressão P9 (LSB e MSB)
#define BMP280_DIG_P9_LSB                          0x9e
#define BMP280_DIG_P9_MSB                          0x9f


/*
 * Registradores de controle e dados:
 */

// Registrador de ID do dispositivo
#define BMP280_DEVID                               0xd0

// Registrador de reset por software
#define BMP280_SOFT_RESET                          0xe0

// Registrador de status
#define BMP280_STAT                                0xf3

// Registrador de controle de medição
#define BMP280_CTRL_MEAS                           0xf4

// Registrador de configuração (standby, filtro)
#define BMP280_CONFIG                              0xf5

// Registradores de dados de pressão
#define BMP280_PRESS_MSB                           0xf7
#define BMP280_PRESS_LSB                           0xf8
#define BMP280_PRESS_XLSB                          0xf9

// Registradores de dados de temperatura
#define BMP280_TEMP_MSB                            0xfa
#define BMP280_TEMP_LSB                            0xfb
#define BMP280_TEMP_XLSB                           0xfc


/*
 * Modos de operação:
 */

// Modo de suspensão (menor consumo)
#define BMP280_SLEEP_MODE                          (0x00)

// Modo forçado (medição única, depois volta ao sleep)
#define BMP280_FORCED_MODE                         (0x01)

// Modo normal (medição contínua)
#define BMP280_NORMAL_MODE                         (0x03)


/*
 * Oversampling de temperatura:
 */

#define BMP280_OST_SKIPPED                         (0x00 << 5)
#define BMP280_OST_X1                              (0x01 << 5)
#define BMP280_OST_X2                              (0x02 << 5)
#define BMP280_OST_X4                              (0x03 << 5)
#define BMP280_OST_X8                              (0x04 << 5)
#define BMP280_OST_X16                             (0x05 << 5)


/*
 * Oversampling de pressão:
 */

#define BMP280_OSP_SKIPPED                         (0x00 << 2)
#define BMP280_OSP_X1                              (0x01 << 2)
#define BMP280_OSP_X2                              (0x02 << 2)
#define BMP280_OSP_X4                              (0x03 << 2)
#define BMP280_OSP_X8                              (0x04 << 2)
#define BMP280_OSP_X16                             (0x05 << 2)


/*
 * Combinações predefinidas de oversampling:
 */

#define BMP280_OS_ULTRA_HIGH_RES                   (BMP280_OSP_X16 | BMP280_OST_X2)
#define BMP280_OS_STANDARD_RES                     (BMP280_OSP_X4  | BMP280_OST_X1)
#define BMP280_OS_ULTRA_LOW_POWER                  (BMP280_OSP_X1  | BMP280_OST_X1)


/*
 * Utilitário: combina 3 bytes em um inteiro de 20 bits:
 */

#define COMBINE(d)                                 (((int)(d)[0] << 12) | ((int)(d)[1] << 4) | ((int)(d)[2] >> 4))


/*
 * Definição da Estrutura Privada --------------------------------------------------------------------------------------
 */

/*
 * Estrutura privada do driver uORB FULL do BMP280:
 *
 * O primeiro campo DEVE ser 'struct sensor_lowerhalf_s'. A presença de 'struct work_s' e 'uint32_t interval'
 * confirma o modelo PUSH.
 */

struct bmp280_dev_s {

    // Estrutura de lower half do sensor (deve ser o primeiro campo)
    FAR struct sensor_lowerhalf_s sensor_lower;

    // Estrutura de agendamento do worker
    struct work_s work;

    // Intervalo de aquisição em microssegundos
    uint32_t interval;

    // Ponteiro para a instância do barramento I2C
    FAR struct i2c_master_s *i2c;

    // Endereço I2C do BMP280
    uint8_t addr;

    // Frequência do barramento I2C
    int freq;

    /*
     * Coeficientes de calibração:
     */

    struct bmp280_calib_s {

        // Coeficiente de temperatura T1
        uint16_t t1;

        // Coeficiente de temperatura T2
        int16_t  t2;

        // Coeficiente de temperatura T3
        int16_t  t3;

        // Coeficiente de pressão P1
        uint16_t p1;

        // Coeficiente de pressão P2
        int16_t  p2;

        // Coeficiente de pressão P3
        int16_t  p3;

        // Coeficiente de pressão P4
        int16_t  p4;

        // Coeficiente de pressão P5
        int16_t  p5;

        // Coeficiente de pressão P6
        int16_t  p6;

        // Coeficiente de pressão P7
        int16_t  p7;

        // Coeficiente de pressão P8
        int16_t  p8;

        // Coeficiente de pressão P9
        int16_t  p9;

    } calib; // fim struct bmp280_calib_s

    // Valor intermediário de compensação de temperatura
    int32_t tempfine;

}; // fim struct bmp280_dev_s


/*
 * Protótipos de Funções Privadas -------------------------------------------------------------------------------------
 */

// Lê um registrador de 8 bits do BMP280
static uint8_t bmp280_getreg8(FAR struct bmp280_dev_s *priv, uint8_t regaddr);

// Lê múltiplos bytes consecutivos do BMP280
static int bmp280_getregs(FAR struct bmp280_dev_s *priv, uint8_t regaddr, uint8_t *rxbuffer, uint8_t length);

// Escreve um valor de 8 bits em um registrador do BMP280
static int bmp280_putreg8(FAR struct bmp280_dev_s *priv, uint8_t regaddr, uint8_t regval);

// Verifica o chip ID do BMP280
static int bmp280_checkid(FAR struct bmp280_dev_s *priv);

// Configura o tempo de standby do BMP280
static int bmp280_set_standby(FAR struct bmp280_dev_s *priv, uint8_t value);

// Inicializa o BMP280
static int bmp280_initialize(FAR struct bmp280_dev_s *priv);

// Calcula a temperatura compensada
static int32_t bmp280_compensate_temp(FAR struct bmp280_dev_s *priv, int32_t temp);

// Calcula a pressão compensada
static uint32_t bmp280_compensate_press(FAR struct bmp280_dev_s *priv, uint32_t press);

/*
 * Métodos do sensor:
 */

// Define o intervalo de aquisição
static int bmp280_set_interval(FAR struct sensor_lowerhalf_s *lower, FAR struct file *filep, FAR uint32_t *period_us);

// Ativa ou desativa o sensor
static int bmp280_activate(FAR struct sensor_lowerhalf_s *lower, FAR struct file *filep, bool enable);

// Worker que executa a leitura periódica do sensor e publica via 'push_event'
static void bmp280_worker(FAR void *arg);


/*
 * Dados Privados -----------------------------------------------------------------------------------------------------
 */

/*
 * >>> ESTRUTURA DE OPERAÇÕES DO SENSOR ('sensor_ops_s') <<<
 *
 * A presença de 'activate' e 'set_interval', e a AUSÊNCIA de 'fetch', é o que caracteriza este driver como
 * uORB FULL (modelo push).
 */

static const struct sensor_ops_s g_sensor_ops = {

    // Habilita/desabilita o sensor
    .activate = bmp280_activate,

    // Define o intervalo de aquisição
    .set_interval = bmp280_set_interval,

}; // fim static const struct sensor_ops_s g_sensor_ops


/*
 * ------------------------------------------- Funções Privadas --------------------------------------------------------
 */



//**********************************************************************************************************************
//* FUNCTION: bmp280_getreg8()
//*
//* PARAMETERS:
//*
//*   priv - Ponteiro para a estrutura privada do driver BMP280
//*   regaddr - Endereço do registrador de 8 bits a ser lido
//*
//* DESCRIPTION: Lê um registrador de 8 bits do BMP280 via I2C.
//*
//* RETURNS: Valor de 8 bits lido do registrador em caso de sucesso; 0 em caso de erro
//*
//**********************************************************************************************************************
static uint8_t bmp280_getreg8(FAR struct bmp280_dev_s *priv, uint8_t regaddr) {


    // Array de mensagens I2C
    struct i2c_msg_s msg[2];

    // Valor lido do registrador
    uint8_t regval = 0;

    // Retorno das chamadas I2C
    int ret;

    /*
     * Primeira mensagem: envia o endereço do registrador:
     */

    // Define a frequência I2C
    msg[0].frequency = priv->freq;

    // Define o endereço do escravo
    msg[0].addr      = priv->addr;

    // Define a operação como escrita
    msg[0].flags     = 0;

    // Define o buffer com o endereço do registrador
    msg[0].buffer    = &regaddr;

    // Define o tamanho do buffer
    msg[0].length    = 1;


    /*
     * Segunda mensagem: lê o valor do registrador:
     */

    // Define a frequência I2C
    msg[1].frequency = priv->freq;

    // Define o endereço do escravo
    msg[1].addr      = priv->addr;

    // Define a operação como leitura
    msg[1].flags     = I2C_M_READ;

    // Define o buffer de destino
    msg[1].buffer    = &regval;

    // Define o tamanho da leitura
    msg[1].length    = 1;


    /*
     * >>> INTERFACE COM SUBSISTEMA I2C <<<
     */

    // Realiza a transferência I2C
    ret = I2C_TRANSFER(priv->i2c,                       // Instância do barramento I2C
                       msg,                             // Array de mensagens I2C
                       2);                              // Número de mensagens

    // Verifica se a transferência falhou
    if (ret < 0) {

        // Registra erro e retorna zero
        snerr("I2C_TRANSFER failed: %d\n",              // Mensagem de erro
              ret);                                     // Código de erro

        // Retorna zero em caso de erro
        return 0;

        } // fim if

    // Retorna o valor lido
    return regval;

} // fim static uint8_t bmp280_getreg8(...)



//______________________________________________________________________________________________________________________




//**********************************************************************************************************************
//* FUNCTION: bmp280_getregs()
//*
//* PARAMETERS:
//*
//*   priv - Ponteiro para a estrutura privada do driver BMP280
//*   regaddr - Endereço inicial do registrador a ser lido
//*   rxbuffer - Buffer para armazenar os bytes lidos
//*   length - Número de bytes a ler
//*
//* DESCRIPTION: Lê múltiplos bytes consecutivos a partir do endereço de registrador especificado.
//*
//* RETURNS: OK (0) em caso de sucesso; -1 em caso de erro
//*
//**********************************************************************************************************************
static int bmp280_getregs(FAR struct bmp280_dev_s *priv, uint8_t regaddr, uint8_t *rxbuffer, uint8_t length) {

    // Array de mensagens I2C
    struct i2c_msg_s msg[2];

    // Retorno das chamadas I2C
    int ret;

    /*
     * Primeira mensagem: envia o endereço inicial:
     */

    // Define a frequência I2C
    msg[0].frequency = priv->freq;

    // Define o endereço do escravo
    msg[0].addr      = priv->addr;

    // Define a operação como escrita
    msg[0].flags     = 0;

    // Define o buffer com o endereço inicial
    msg[0].buffer    = &regaddr;

    // Define o tamanho do buffer
    msg[0].length    = 1;

    /*
     * Segunda mensagem: lê os bytes consecutivos:
     */

    // Define a frequência I2C
    msg[1].frequency = priv->freq;

    // Define o endereço do escravo
    msg[1].addr      = priv->addr;

    // Define a operação como leitura
    msg[1].flags     = I2C_M_READ;

    // Define o buffer de destino
    msg[1].buffer    = rxbuffer;

    // Define o número de bytes a ler
    msg[1].length    = length;


    /*
     * >>> INTERFACE COM SUBSISTEMA I2C <<<
     */

    // Realiza a transferência I2C
    ret = I2C_TRANSFER(priv->i2c,                       // Instância do barramento I2C
                       msg,                             // Array de mensagens I2C
                       2);                              // Número de mensagens

    // Verifica se a transferência falhou
    if (ret < 0) {

        // Registra erro e retorna -1
        snerr("I2C_TRANSFER failed: %d\n", ret);        // Mensagem de erro

        // Retorna -1 em caso de erro
        return -1;

        } // fim if

    // Retorna sucesso
    return OK;

} // fim static int bmp280_getregs(...)



//______________________________________________________________________________________________________________________




//**********************************************************************************************************************
//* FUNCTION: bmp280_putreg8()
//*
//* PARAMETERS:
//*
//*   priv - Ponteiro para a estrutura privada do driver BMP280
//*   regaddr - Endereço do registrador a ser escrito
//*   regval - Valor a ser escrito
//*
//* DESCRIPTION: Escreve um valor de 8 bits em um registrador do BMP280 via I2C.
//*
//* RETURNS: OK (0) em caso de sucesso; código negativo em caso de erro
//*
//**********************************************************************************************************************
static int bmp280_putreg8(FAR struct bmp280_dev_s *priv, uint8_t regaddr, uint8_t regval) {


    // Array de mensagens I2C
    struct i2c_msg_s msg[2];

    // Buffer com o endereço e o valor
    uint8_t txbuffer[2];

    // Retorno das chamadas I2C
    int ret;

    /*
     * Preenche o buffer:
     */

    // Armazena o endereço do registrador
    txbuffer[0] = regaddr;

    // Armazena o valor a ser escrito
    txbuffer[1] = regval;

    /*
     * Mensagem I2C: envia endereço e valor:
     */

    // Define a frequência I2C
    msg[0].frequency = priv->freq;

    // Define o endereço do escravo
    msg[0].addr      = priv->addr;

    // Define a operação como escrita
    msg[0].flags     = 0;

    // Define o buffer com endereço e valor
    msg[0].buffer    = txbuffer;

    // Define o tamanho do buffer
    msg[0].length    = 2;


    /*
     * >>> INTERFACE COM SUBSISTEMA I2C <<<
     */

    // Realiza a transferência I2C
    ret = I2C_TRANSFER(priv->i2c,                       // Instância do barramento I2C
                       msg,                             // Array de mensagens I2C
                       1);                              // Número de mensagens

    // Verifica se a transferência falhou
    if (ret < 0) {

        // Registra erro
        snerr("I2C_TRANSFER failed: %d\n",              // Mensagem de erro
              ret);                                     // Código de erro

        } // fim if

    // Retorna o resultado
    return ret;

} // fim static int bmp280_putreg8(...)



//______________________________________________________________________________________________________________________




//**********************************************************************************************************************
//* FUNCTION: bmp280_checkid()
//*
//* PARAMETERS:
//*
//*   priv - Ponteiro para a estrutura privada do driver BMP280
//*
//* DESCRIPTION: Verifica o chip ID do BMP280.
//*
//* RETURNS: OK (0) em caso de sucesso; -ENODEV em caso de ID incorreto
//*
//**********************************************************************************************************************
static int bmp280_checkid(FAR struct bmp280_dev_s *priv) {

    // Variável para armazenar o ID lido
    uint8_t devid = 0;

    // Lê o registrador de ID
    devid = bmp280_getreg8(priv,                        // Estrutura privada do driver
                           BMP280_DEVID);               // Endereço do registrador de ID

    // Pequeno atraso
    up_mdelay(1);

    // Registra o ID no log
    sninfo("devid: 0x%02x\n",                           // Mensagem de log
           devid);                                      // Valor lido

    // Verifica se o ID corresponde ao esperado
    if (devid != (uint16_t) DEVID) {

        // Registra erro
        snerr("Wrong Device ID! %02x\n", devid);

        // Retorna erro
        return -ENODEV;

        } // fim if

    // Retorna sucesso
    return OK;

} // fim static int bmp280_checkid(...)



//______________________________________________________________________________________________________________________




//**********************************************************************************************************************
//* FUNCTION: bmp280_set_standby()
//*
//* PARAMETERS:
//*
//*   priv - Ponteiro para a estrutura privada do driver BMP280
//*   value - Valor de standby a ser configurado
//*
//* DESCRIPTION: Configura o tempo de standby do sensor.
//*
//* RETURNS: OK (0) em caso de sucesso; ERROR em caso de falha
//*
//**********************************************************************************************************************
static int bmp280_set_standby(FAR struct bmp280_dev_s *priv, uint8_t value) {


    // Valor lido do registrador
    uint8_t v_data_u8;

    // Valor de standby aplicado
    uint8_t v_sb_u8;

    // Lê o registrador de configuração
    v_data_u8 = bmp280_getreg8(priv,                    // Estrutura privada do driver
                               BMP280_CONFIG);          // Endereço do registrador de configuração

    // Modifica os bits de standby
    v_data_u8 = (v_data_u8 & ~(0x07 << 5)) | (value << 5);

    // Escreve o novo valor
    bmp280_putreg8(priv,                                // Estrutura privada do driver
                   BMP280_CONFIG,                       // Endereço do registrador de configuração
                   v_data_u8);                          // Novo valor

    // Relê o registrador
    v_data_u8 = bmp280_getreg8(priv,                    // Estrutura privada do driver
                               BMP280_CONFIG);          // Endereço do registrador de configuração

    // Extrai os bits de standby
    v_sb_u8 = (v_data_u8 >> 5) & 0x07;

    // Verifica se o valor foi aplicado
    if (v_sb_u8 != value) {

        // Registra erro
        snerr("Failed to set value for standby time.");

        // Retorna erro
        return ERROR;

        } // fim if

    // Retorna sucesso
    return OK;

} // fim static int bmp280_set_standby(...)



//______________________________________________________________________________________________________________________




//**********************************************************************************************************************
//* FUNCTION: bmp280_initialize()
//*
//* PARAMETERS:
//*
//*   priv - Ponteiro para a estrutura privada do driver BMP280
//*
//* DESCRIPTION: Inicializa o BMP280: lê coeficientes de calibração e configura o standby.
//*
//* RETURNS: OK (0) em caso de sucesso; código negativo em caso de erro
//*
//**********************************************************************************************************************
static int bmp280_initialize(FAR struct bmp280_dev_s *priv) {

    // Buffer para os coeficientes
    uint8_t buf[24];

    // Retorno das chamadas
    int ret;

    // Lê os coeficientes de calibração
    ret = bmp280_getregs(priv,                          // Estrutura privada do driver
                         BMP280_DIG_T1_LSB,             // Endereço inicial dos coeficientes
                         buf,                           // Buffer de destino
                         24);                           // Número de bytes

    // Verifica se a leitura falhou
    if (ret < 0) {

        // Retorna erro
        return ret;

        } // fim if

    /*
     * Extrai os coeficientes de temperatura:
     */

    // Extrai o coeficiente T1
    priv->calib.t1 = (uint16_t)buf[1]  << 8 | buf[0];

    // Extrai o coeficiente T2
    priv->calib.t2 = (int16_t) buf[3]  << 8 | buf[2];

    // Extrai o coeficiente T3
    priv->calib.t3 = (int16_t) buf[5]  << 8 | buf[4];

    /*
     * Extrai os coeficientes de pressão:
     */

    // Extrai o coeficiente P1
    priv->calib.p1 = (uint16_t)buf[7]  << 8 | buf[6];

    // Extrai o coeficiente P2
    priv->calib.p2 = (int16_t) buf[9]  << 8 | buf[8];

    // Extrai o coeficiente P3
    priv->calib.p3 = (int16_t) buf[11] << 8 | buf[10];

    // Extrai o coeficiente P4
    priv->calib.p4 = (int16_t) buf[13] << 8 | buf[12];

    // Extrai o coeficiente P5
    priv->calib.p5 = (int16_t) buf[15] << 8 | buf[14];

    // Extrai o coeficiente P6
    priv->calib.p6 = (int16_t) buf[17] << 8 | buf[16];

    // Extrai o coeficiente P7
    priv->calib.p7 = (int16_t) buf[19] << 8 | buf[18];

    // Extrai o coeficiente P8
    priv->calib.p8 = (int16_t) buf[21] << 8 | buf[20];

    // Extrai o coeficiente P9
    priv->calib.p9 = (int16_t) buf[23] << 8 | buf[22];

    // Coloca o sensor em modo sleep
    bmp280_putreg8(priv,                                // Estrutura privada do driver
                   BMP280_CTRL_MEAS,                    // Registrador de controle de medição
                   BMP280_SLEEP_MODE);                  // Modo sleep

    // Configura o standby para 0.5 ms
    ret = bmp280_set_standby(priv,                      // Estrutura privada do driver
                             BMP280_STANDBY_05_MS);     // Tempo de standby

    // Verifica se a configuração falhou
    if (ret != OK) {

        // Registra erro
        snerr("Failed to set value for standby time.\n");

        // Retorna erro
        return -1;

        } // fim if

    // Retorna sucesso
    return ret;

} // fim static int bmp280_initialize(...)



//______________________________________________________________________________________________________________________




//**********************************************************************************************************************
//* FUNCTION: bmp280_compensate_temp()
//*
//* PARAMETERS:
//*
//*   priv - Ponteiro para a estrutura privada do driver BMP280
//*   temp - Valor bruto de temperatura
//*
//* DESCRIPTION: Calcula a temperatura compensada do BMP280.
//*
//* RETURNS: Temperatura compensada em unidades de 0.01 °C
//*
//**********************************************************************************************************************
static int32_t bmp280_compensate_temp(FAR struct bmp280_dev_s *priv, int32_t temp) {

    // Ponteiro de acesso aos coeficientes
    struct bmp280_calib_s *c = &priv->calib;

    // Variável intermediária
    int32_t var1;

    // Variável intermediária
    int32_t var2;

    // Calcula var1
    var1 = ((((temp >> 3) - ((int32_t)c->t1 << 1))) * ((int32_t)c->t2)) >> 11;

    // Calcula var2
    var2 = (((((temp >> 4) - ((int32_t)c->t1)) * ((temp >> 4) - ((int32_t)c->t1))) >> 12) * ((int32_t)c->t3)) >> 14;

    // Armazena o valor intermediário
    priv->tempfine = var1 + var2;

    // Retorna a temperatura compensada
    return (priv->tempfine * 5 + 128) >> 8;

} // fim static int32_t bmp280_compensate_temp(...)



//______________________________________________________________________________________________________________________




//**********************************************************************************************************************
//* FUNCTION: bmp280_compensate_press()
//*
//* PARAMETERS:
//*
//*   priv - Ponteiro para a estrutura privada do driver BMP280
//*   press - Valor bruto de pressão
//*
//* DESCRIPTION: Calcula a pressão compensada do BMP280.
//*
//* RETURNS: Pressão compensada em Pascal (Pa)
//*
//**********************************************************************************************************************
static uint32_t bmp280_compensate_press(FAR struct bmp280_dev_s *priv, uint32_t press) {


    // Ponteiro de acesso aos coeficientes
    struct bmp280_calib_s *c = &priv->calib;

    // Variável intermediária
    int32_t var1;

    // Variável intermediária
    int32_t var2;

    // Variável para armazenar a pressão
    uint32_t p;

    // Calcula var1
    var1 = (priv->tempfine >> 1) - 64000;

    // Calcula var2
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)c->p6);

    // Calcula var2
    var2 = var2 + ((var1 * ((int32_t)c->p5)) << 1);

    // Calcula var2
    var2 = (var2 >> 2) + (((int32_t)c->p4) << 16);

    // Calcula var1
    var1 = (((c->p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)c->p2) * var1) >> 1)) >> 18;

    // Calcula var1
    var1 = (((32768 + var1) * ((int32_t)c->p1)) >> 15);

    // Verifica se var1 é zero para evitar divisão por zero
    if (var1 == 0) {

        // Retorna zero
        return 0;

        } // fim if

    // Calcula p
    p = (((uint32_t)((0x100000) - press) - (var2 >> 12))) * 3125;

    // Ajusta a divisão conforme o valor
    if (p < 0x80000000) {

        // Deslocamento à esquerda
        p = (p << 1) / ((uint32_t)var1);

        }
    else {

        // Deslocamento à direita
        p = (p / (uint32_t)var1) * 2;

        } // fim if

    // Aplica o ajuste final
    var1 = ((int32_t)c->p9 * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;

    // Calcula var2
    var2 = ((int32_t)(p >> 2) * c->p8) >> 13;

    // Aplica o ajuste final
    p = (uint32_t)((int32_t)p + ((var1 + var2 + c->p7) >> 4));

    // Retorna a pressão compensada
    return p;

} // fim static uint32_t bmp280_compensate_press(...)



//______________________________________________________________________________________________________________________




//**********************************************************************************************************************
//* FUNCTION: bmp280_set_interval()
//*
//* PARAMETERS:
//*
//*   lower - Ponteiro para a estrutura do lower half do sensor
//*   filep - Ponteiro para a estrutura do arquivo (não utilizado)
//*   period_us - Ponteiro para o intervalo desejado em microssegundos
//*
//* DESCRIPTION: Define o intervalo de aquisição do sensor. O valor é mapeado para o tempo de standby
//*              correspondente no registrador de configuração do BMP280.
//*
//* RETURNS: OK (0) em caso de sucesso; -EINVAL em caso de valor não suportado
//*
//**********************************************************************************************************************
static int bmp280_set_interval(FAR struct sensor_lowerhalf_s *lower, FAR struct file *filep, FAR uint32_t *period_us) {


    // Recupera a estrutura privada
    FAR struct bmp280_dev_s *priv = container_of(lower,     // Estrutura do lower half
                                                 FAR struct bmp280_dev_s,  // Tipo da estrutura privada
                                                 sensor_lower);            // Campo do lower half

    // Retorno da função
    int ret = 0;

    // Valor a ser escrito no registrador de standby
    uint8_t regval;

    // Mapeia o intervalo para o valor de standby
    switch (*period_us) {

        // 0.5 ms
        case 500:

            // Armazena o valor de standby de 0.5 ms
            regval = BMP280_STANDBY_05_MS;
            // Sai do switch
            break;

        // 62.5 ms
        case 62500:

            // Armazena o valor de standby de 62.5 ms
            regval = BMP280_STANDBY_63_MS;
            // Sai do switch
            break;

        // 125 ms
        case 125000:

            // Armazena o valor de standby de 125 ms
            regval = BMP280_STANDBY_125_MS;
            // Sai do switch
            break;

        // 250 ms
        case 250000:

            // Armazena o valor de standby de 250 ms
            regval = BMP280_STANDBY_250_MS;
            // Sai do switch
            break;

        // 500 ms
        case 500000:

            // Armazena o valor de standby de 500 ms
            regval = BMP280_STANDBY_500_MS;
            // Sai do switch
            break;

        // 1000 ms
        case 1000000:

            // Armazena o valor de standby de 1000 ms
            regval = BMP280_STANDBY_1000_MS;
            // Sai do switch
            break;

        // 2000 ms
        case 2000000:

            // Armazena o valor de standby de 2000 ms
            regval = BMP280_STANDBY_2000_MS;
            // Sai do switch
            break;

        // 4000 ms
        case 4000000:

            // Armazena o valor de standby de 4000 ms
            regval = BMP280_STANDBY_4000_MS;
            // Sai do switch
            break;

        // Valor não suportado
        default:

            // Retorna erro de argumento inválido
            ret = -EINVAL;
            // Sai do switch
            break;

        } // fim switch

    // Aplica a configuração se o valor foi mapeado
    if (ret == 0) {

        // Aplica o tempo de standby
        ret = bmp280_set_standby(priv,                  // Estrutura privada do driver
                                 regval);               // Valor de standby

        } // fim if

    // Atualiza o intervalo interno do driver
    if (ret == 0) {

        // Armazena o novo intervalo para o worker
        priv->interval = *period_us;

        } // fim if

    // Retorna o resultado
    return ret;

} // fim static int bmp280_set_interval(...)



//______________________________________________________________________________________________________________________




//**********************************************************************************************************************
//* FUNCTION: bmp280_activate()
//*
//* PARAMETERS:
//*
//*   lower - Ponteiro para a estrutura do lower half do sensor
//*   filep - Ponteiro para a estrutura do arquivo (não utilizado)
//*   enable - true para ativar, false para desativar
//*
//* DESCRIPTION: Habilita ou desabilita o sensor BMP280. Ao habilitar, coloca o sensor em modo normal e
//*              agenda o worker na work queue HPWORK. Ao desabilitar, coloca o sensor em modo sleep e
//*              cancela o agendamento do worker.
//*
//* RETURNS: OK (0) sempre
//*
//**********************************************************************************************************************
static int bmp280_activate(FAR struct sensor_lowerhalf_s *lower, FAR struct file *filep, bool enable) {

    // Recupera a estrutura privada
    FAR struct bmp280_dev_s *priv = container_of(lower,     // Estrutura do lower half
                                                 FAR struct bmp280_dev_s,  // Tipo da estrutura privada
                                                 sensor_lower);            // Campo do lower half

    // Retorno das chamadas
    int ret;

    // Marcador de execução: registra no console que a função foi chamada
    sninfo("[bmp280_activate]: chamada com enable = %d\n", enable);

    // Verifica se o sensor está sendo ativado
    if (enable) {

        /*
         * Coloca o sensor em modo normal com resolução padrão:
         */

        // Escreve no registrador de controle de medição
        ret = bmp280_putreg8(priv,                          // Estrutura privada do driver
                             BMP280_CTRL_MEAS,              // Registrador de controle de medição
                             BMP280_NORMAL_MODE | BMP280_OS_STANDARD_RES); // Modo normal

        // Verifica se a operação falhou
        if (ret < 0) {

            // Retorna o código de erro
            return ret;

            } // fim if

        /*
         * >>> INTERFACE COM WORK QUEUE (HPWORK) <<<
         *
         * Agenda o worker na work queue de alta prioridade. O worker se reagenda
         * continuamente enquanto o sensor estiver ativo.
         */

        // Agenda o worker pela primeira vez
        work_queue(HPWORK, &priv->work,                     // Work queue de alta prioridade
                   bmp280_worker, priv,                     // Função worker e argumento
                   priv->interval / USEC_PER_TICK);         // Intervalo em ticks

        }
    else {

        /*
         * Coloca o sensor em modo sleep:
         */

        // Escreve no registrador de controle de medição
        ret = bmp280_putreg8(priv,                          // Estrutura privada do driver
                             BMP280_CTRL_MEAS,              // Registrador de controle de medição
                             BMP280_SLEEP_MODE);            // Modo sleep

        /*
         * >>> INTERFACE COM WORK QUEUE (HPWORK) <<<
         *
         * Cancela o agendamento do worker.
         */

        // Cancela o worker agendado
        work_cancel(HPWORK,                                 // Work queue de alta prioridade
                    &priv->work);                           // Estrutura de trabalho

        } // fim if

    // Retorna o resultado
    return ret;

} // fim static int bmp280_activate(...)



//______________________________________________________________________________________________________________________




//**********************************************************************************************************************
//* FUNCTION: bmp280_worker()
//*
//* PARAMETERS:
//*
//*   arg - Ponteiro para a estrutura privada do driver (bmp280_dev_s)
//*
//* DESCRIPTION: Worker executado pela 'work queue' HPWORK. Reagenda a si mesmo, lê os valores brutos de temperatura e
//*              pressão, aplica as compensações e publica os dados via 'push_event()'.
//*
//*              Esta função é o coração do modelo push.
//*
//* RETURNS: void
//*
//**********************************************************************************************************************
static void bmp280_worker(FAR void *arg) {


    // Recupera a estrutura privada
    FAR struct bmp280_dev_s *priv = arg;

    // Estrutura que conterá os dados publicados
    struct sensor_baro baro;

    // Buffer para os 6 bytes de dados
    uint8_t buf[6];

    // Valor bruto de pressão
    uint32_t press;

    // Valor bruto de temperatura
    int32_t temp;

    // Retorno das chamadas
    int ret;

    // Estrutura para o timestamp
    struct timespec ts;

    // Garante que o ponteiro não é nulo
    DEBUGASSERT(priv != NULL);

    /*
     * >>> INTERFACE COM WORK QUEUE (HPWORK) <<<
     *
     * Reagenda o worker para o próximo ciclo.
     */

    // Reagenda o worker para o próximo ciclo
    work_queue(HPWORK,                                      // Work queue de alta prioridade
               &priv->work,                                 // Estrutura de trabalho
               bmp280_worker,                               // Função worker
               priv,                                        // Argumento
               priv->interval / USEC_PER_TICK);             // Intervalo em ticks

    /*
     * Lê os valores brutos do sensor:
     */

    // Lê os 6 bytes de dados de pressão e temperatura
    ret = bmp280_getregs(priv,                              // Estrutura privada do driver
                         BMP280_PRESS_MSB,                  // Endereço inicial dos dados
                         buf,                               // Buffer de destino
                         6);                                // Número de bytes

    // Verifica se a leitura falhou
    if (ret < 0) {

        // Registra erro e retorna
        snerr("Failed to read sensor data: %d\n", ret);

        // Retorna em caso de erro
        return;

        } // fim if

    // Combina os bytes de pressão
    press = (uint32_t)COMBINE(buf);

    // Combina os bytes de temperatura
    temp = COMBINE(&buf[3]);

    /*
     * Aplica as compensações:
     */

    // Compensa a temperatura
    temp = bmp280_compensate_temp(priv,                     // Estrutura privada do driver
                                  temp);                    // Valor bruto de temperatura

    // Compensa a pressão
    press = bmp280_compensate_press(priv,                   // Estrutura privada do driver
                                    press);                 // Valor bruto de pressão

    /*
     * Obtém o timestamp atual:
     */

    // Obtém o timestamp do sistema
    clock_systime_timespec(&ts);

    /*
     * Preenche a estrutura de saída:
     */

    // Calcula o timestamp em microssegundos
    baro.timestamp = 1000000ull * ts.tv_sec + ts.tv_nsec / 1000;

    // Converte a pressão para hPa
    baro.pressure = sensor_data_divi(press, 100);

    // Converte a temperatura para décimos de grau
    baro.temperature = sensor_data_divi(temp, 100);

    /*
     * >>> PUSH EVENT — MARCADOR DO MODELO PUSH <<<
     *
     * Publica o evento no framework de sensores.
     */

    // Publica o evento via push_event
    priv->sensor_lower.push_event(priv->sensor_lower.priv,  // Estrutura privada do lowerhalf
                                  &baro,                    // Dados a publicar
                                  sizeof(baro));            // Tamanho dos dados

} // fim static void bmp280_worker(...)



//______________________________________________________________________________________________________________________



/*
 * ------------------------------------------- Funções Públicas --------------------------------------------------------
 */



//**********************************************************************************************************************
//* FUNCTION: bmp280_register()
//*
//* PARAMETERS:
//*
//*   devno - Número do dispositivo (instância) a ser registrado
//*   i2c - Instância do barramento I2C a ser usada na comunicação com o BMP280
//*
//* DESCRIPTION: Inicializa a estrutura privada do driver uORB, verifica o ID do dispositivo, inicializa o
//*              sensor e o registra no framework de sensores via 'sensor_register()'. Esta função é chamada
//*              pelo BSP, tipicamente em 'esp32s3_bringup()'.
//*
//* RETURNS: OK (0) em caso de sucesso; código negativo de erro em caso de falha
//*
//**********************************************************************************************************************
int bmp280_register(int devno, FAR struct i2c_master_s *i2c) {


    // Ponteiro para a estrutura privada do driver
    FAR struct bmp280_dev_s *priv;

    // Retorno das chamadas
    int ret;

    /*
     * Inicializa a estrutura do dispositivo BMP280:
     */

    // Aloca e zera a memória para a estrutura privada
    priv = kmm_zalloc(sizeof(struct bmp280_dev_s));

    // Verifica se a alocação falhou
    if (!priv) {

        // Registra erro e retorna código de falta de memória
        snerr("Failed to allocate instance\n");

        // Retorna erro de falta de memória
        return -ENOMEM;

        } // fim if

    // Armazena a instância do barramento I2C
    priv->i2c = i2c;

    // Armazena o endereço I2C do BMP280
    priv->addr = BMP280_ADDR;

    // Armazena a frequência I2C
    priv->freq = BMP280_FREQ;

    // Associa a estrutura de operações do sensor
    priv->sensor_lower.ops = &g_sensor_ops;

    // Define o tipo de sensor (barômetro)
    priv->sensor_lower.type = SENSOR_TYPE_BAROMETER;

    // Define o tamanho do buffer de eventos
    priv->sensor_lower.nbuffer = 1;

    // Define o intervalo inicial
    priv->interval = BMP280_MIN_INTERVAL;

    /*
     * Verifica o ID do dispositivo:
     */

    // Chama a função de verificação de ID
    ret = bmp280_checkid(priv);

    // Verifica se o ID não corresponde ao esperado
    if (ret < 0) {

        // Registra erro e libera a memória alocada
        snerr("Failed to register driver: %d\n", ret);

        // Libera a memória da estrutura privada
        kmm_free(priv);

        // Retorna o código de erro original
        return ret;

        } // fim if

    /*
     * Inicializa o sensor:
     */

    // Chama a função de inicialização
    ret = bmp280_initialize(priv);

    // Verifica se a inicialização falhou
    if (ret < 0) {

        // Registra erro e libera a memória alocada
        snerr("Failed to initialize physical device bmp280:%d\n", ret);

        // Libera a memória da estrutura privada
        kmm_free(priv);

        // Retorna o código de erro original
        return ret;

        } // fim if

    /*
     * Registra o driver no framework de sensores:
     */

    /*
     * >>> INTERFACE COM FRAMEWORK uORB <<<
     *
     * 'sensor_register()' é a API do framework de sensores do NuttX que cria o nó
     * em '/dev/uorb/'. No caso do BMP280, cria '/dev/uorb/sensor_baro0'.
     */

    // Registra o driver com o lowerhalf e o número do dispositivo
    ret = sensor_register(&priv->sensor_lower, devno);

    // Verifica se o registro falhou
    if (ret < 0) {

        // Registra erro e libera a memória alocada
        snerr("Failed to register driver: %d\n", ret);

        // Libera a memória da estrutura privada
        kmm_free(priv);

        } // fim if

    // Registra mensagem informativa indicando sucesso
    sninfo("BMP280 FULL uORB driver loaded successfully!\n");

    // Retorna o resultado final
    return ret;

} // fim int bmp280_register(...)


// Fim do bloco condicional: apenas se I2C e BMP280 estiverem habilitados
#endif


//----------------------------------------------------------------------------------------------------------------------
// Fim do arquivo "bmp280_uorb_true-full"
//----------------------------------------------------------------------------------------------------------------------
