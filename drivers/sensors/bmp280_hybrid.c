//######################################################################################################################
// *
// *  Arquivo: 'drivers/sensors/bmp280_hybrid.c'
// *
// *  Título: Driver uORB Híbrido do Sensor de Pressão BMP280 — Suporte a Modelos Push e Pull
// *
// *  Objetivo: Implementar o driver de sensor do BMP280 com suporte a DOIS modelos de operação, selecionáveis
// *            em tempo de compilação pela flag 'CONFIG_SENSORS_BMP280_POLL':
// *
// *              - Com 'CONFIG_SENSORS_BMP280_POLL' habilitada: modelo PUSH (FULL uORB). O driver usa uma
// *                thread dedicada e publica dados espontaneamente via 'push_event'.
// *
// *              - Com 'CONFIG_SENSORS_BMP280_POLL' desabilitada: modelo PULL. O driver implementa apenas 'fetch' e
// *                aguarda leitura sob demanda.
// *
// *  Design Pattern: Driver uORB PARCIAL (híbrido condicional) — padrão oficial do NuttX para drivers que
// *                  precisam suportar ambos os modelos.
// *
// *  Autor: Baseado no código oficial do Apache NuttX
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
// *  Modelo: Lower Half uORB PARCIAL (híbrido condicional).
// *
// *  NOTA CONCEITUAL — ARQUITETURA E POSIÇÃO:
// *
// *    Este driver é um Lower Half uORB que suporta DOIS modelos distintos, selecionados em tempo de
// *    compilação. A escolha afeta o modo de operação do hardware e a forma como os dados fluem para o
// *    Upper Half.
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
// *      │  ESTE ARQUIVO: 'bmp280_hybrid.c' — Lower Half uORB PARCIAL               │
// *      │                                                                          │
// *      │    COM CONFIG_SENSORS_BMP280_POLL:                                       │
// *      │      'bmp280_activate()' ──► kthread_create('bmp280_thread')            │
// *      │           │                                                              │
// *      │           ▼                                                             │
// *      │      'bmp280_thread()'  ──► leitura I2C                                 │
// *      │           │                                                              │
// *      │           └────────────► 'push_event()' ──► Upper Half ──► assinantes  │
// *      │                                                                          │
// *      │    SEM CONFIG_SENSORS_BMP280_POLL:                                       │
// *      │      'bmp280_fetch()' ────► leitura I2C sob demanda pelo Upper Half     │
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
// *    - Este driver NÃO usa uma Camada de Base separada. O acesso físico e a compensação estão no
// *      próprio arquivo, seguindo o padrão do 'bmp280_uorb.c' original.
// *
// *    - A estrutura de operações do sensor ('g_sensor_ops') é montada condicionalmente em tempo de
// *      compilação, incluindo 'fetch' apenas quando o modo poll está desabilitado.
// *
// *    - O modo PUSH requer uma thread dedicada ('kthread_create') e o uso de um semáforo de espera
// *      ('sem_t run') para bloquear a thread quando o sensor estiver desabilitado.
// *
// *    - O modo PUSH usa o modo normal do hardware (medição contínua) e uma flag 'enabled' interna.
// *      O modo PULL usa o modo forçado do hardware para medição única sob demanda.
// *
// ********************************************************************************************************************/
// *
// *  RESUMO DA TOPOLOGIA DO ARQUIVO 'bmp280_hybrid.c'
// *
// *  Topologia: DRIVER uORB PARCIAL (HÍBRIDO CONDICIONAL)
// *
// *  Marcadores arquiteturais que caracterizam essa topologia:
// *
// *    1. Estrutura de operações condicional:
// *
// *       - 'struct sensor_ops_s g_sensor_ops' é montada com '#ifdef CONFIG_SENSORS_BMP280_POLL'.
// *       - Com POLL: 'activate', 'set_interval'. Sem 'fetch'.
// *       - Sem POLL: 'activate', 'set_interval', 'fetch'.
// *
// *
// *    2. Estrutura privada com campos condicionais:
// *
// *       - 'struct bmp280_dev_s' contém:
// *         - 'struct sensor_lowerhalf_s sensor_lower' — interface com o Upper Half.
// *         - 'struct i2c_master_s *i2c' — barramento I2C.
// *         - 'uint8_t addr' e 'int freq' — parâmetros de comunicação.
// *         - 'bool activated' — estado do sensor.
// *         - 'struct bmp280_calib_s calib' — coeficientes de calibração.
// *         - 'int32_t tempfine' — valor intermediário de compensação.
// *       - Campos condicionais (apenas com POLL):
// *         - 'sem_t run' — semáforo para bloqueio da thread.
// *         - 'uint32_t interval' — intervalo de aquisição.
// *         - 'bool enabled' — estado de habilitação.
// *
// *
// *    3. Funções condicionais:
// *
// *       - 'bmp280_thread()' — criada apenas com POLL. Executa leitura periódica e 'push_event'.
// *       - 'bmp280_fetch()' — criada apenas sem POLL. Executa leitura sob demanda.
// *
// *
// *    4. Função de registro:
// *
// *       - 'bmp280_register()' — usa 'sensor_register()' (framework uORB).
// *       - Cria a thread condicionalmente ao final do registro.
// *
// *
// *    5. Presenças que confirmam o modelo híbrido:
// *
// *       - 'kthread_create' (apenas com POLL).
// *       - 'push_event' (apenas com POLL).
// *       - 'fetch' (apenas sem POLL).
// *       - 'sem_wait' e 'sem_post' (apenas com POLL).
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
// *       - Suporta tanto o modelo PUSH (FULL) quanto o modelo PULL, conforme a configuração.
// *       - É o padrão da maioria dos drivers uORB oficiais do NuttX.
// *
// *--------------------------------------------------------------------------------------------------------------------
// #####################################################################################################################
// *
// *  Ver  | dd mmm aaaa | Who      | Descrição de alterações
// * ======|=============|==========|===================================================================================
// *  1.0.0| 10 set 2026 | DeepSeek | Versão inicial do driver híbrido (Fase 3)
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

// Inclui funções utilitárias padrão, como memcpy e strtoul
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

// Inclui a interface pública do sensor BMP280 (protótipos e constantes externas)
#include <nuttx/sensors/bmp280.h>

// Inclui a interface do framework de sensores (sensor_lowerhalf_s, sensor_ops_s)
#include <nuttx/sensors/sensor.h>

// Inclui a API de criação de threads do kernel (apenas no modo POLL)
#include <nuttx/kthread.h>

// Inclui a API de semáforos POSIX (apenas no modo POLL)
#include <nuttx/semaphore.h>

// Bloco condicional: compila este arquivo apenas se I2C e o sensor BMP280 estiverem habilitados no Kconfig
#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BMP280)


/*
 * Definições de Pré-Processador --------------------------------------------------------------------------------------
 */


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
 * Definição da Estrutura Privada -------------------------------------------------------------------------------------
 */

/*
 * Estrutura privada do driver híbrido do BMP280:
 *
 * Esta estrutura encapsula o estado do driver. O primeiro campo DEVE ser
 * 'struct sensor_lowerhalf_s', conforme exigido pelo framework de sensores.
 *
 * Os campos condicionais (sem_t run, uint32_t interval, bool enabled) existem
 * apenas quando o modo POLL está habilitado.
 */

struct bmp280_dev_s {

    // Estrutura de lower half do sensor (deve ser o primeiro campo)
    FAR struct sensor_lowerhalf_s sensor_lower;

    // Ponteiro para a instância do barramento I2C
    FAR struct i2c_master_s *i2c;

    // Endereço I2C do BMP280
    uint8_t addr;

    // Frequência do barramento I2C
    int freq;

    // Estado de ativação do sensor
    bool activated;

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

    /*
     * Campos condicionais — existem apenas com CONFIG_SENSORS_BMP280_POLL:
     */

    // Bloco condicional: campos adicionais do modo POLL
#ifdef CONFIG_SENSORS_BMP280_POLL

    // Semáforo para bloquear a thread quando o sensor estiver desabilitado
    sem_t run;

    // Intervalo de aquisição em microssegundos
    uint32_t interval;

    // Estado de habilitação (controla a thread)
    bool enabled;

    // Fim do bloco condicional dos campos do modo POLL
#endif

}; // fim struct bmp280_dev_s


//______________________________________________________________________________________________________________________



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

// Bloco condicional: protótipos específicos de cada modo
#ifndef CONFIG_SENSORS_BMP280_POLL

// Realiza a leitura sob demanda (modelo PULL)
static int bmp280_fetch(FAR struct sensor_lowerhalf_s *lower, FAR struct file *filep, FAR char *buffer, size_t buflen);

// Fim do bloco condicional sem POLL
#endif

// Bloco condicional: thread do modo POLL
#ifdef CONFIG_SENSORS_BMP280_POLL

// Thread que executa a leitura periódica e publica via push_event
static int bmp280_thread(int argc, FAR char **argv);

// Fim do bloco condicional com POLL
#endif


/*
 * Dados Privados -----------------------------------------------------------------------------------------------------
 */

/*
 * >>> ESTRUTURA DE OPERAÇÕES DO SENSOR (sensor_ops_s) <<<
 *
 * Montada condicionalmente em tempo de compilação:
 *
 *   - Com POLL: 'activate', 'set_interval'. Sem 'fetch'.
 *   - Sem POLL: 'activate', 'set_interval', 'fetch'.
 */

static const struct sensor_ops_s g_sensor_ops = {

    // Habilita/desabilita o sensor
    .activate = bmp280_activate,

    // Define o intervalo de aquisição
    .set_interval = bmp280_set_interval,

    // Bloco condicional: 'fetch' apenas quando POLL está desabilitado
#ifndef CONFIG_SENSORS_BMP280_POLL

    // Realiza a leitura sob demanda
    .fetch = bmp280_fetch,

    // Fim do bloco condicional sem POLL
#endif

}; // fim static const struct sensor_ops_s g_sensor_ops


//______________________________________________________________________________________________________________________



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

    // Array de mensagens I2C (endereço + leitura)
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

        // Registra erro indicando falha na transferência
        snerr("I2C_TRANSFER failed: %d\n", ret);        // Mensagem de erro

        // Retorna zero em caso de erro
        return 0;

        } // fim if

    // Retorna o valor lido do registrador
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
static int bmp280_getregs(FAR struct bmp280_dev_s *priv, uint8_t regaddr,
                          uint8_t *rxbuffer, uint8_t length) {

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

        // Registra erro indicando falha na transferência
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
static int bmp280_putreg8(FAR struct bmp280_dev_s *priv, uint8_t regaddr,
                          uint8_t regval) {

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

        // Registra erro indicando falha na transferência
        snerr("I2C_TRANSFER failed: %d\n", ret);        // Mensagem de erro

        } // fim if

    // Retorna o resultado da transferência
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

    // Pequeno atraso para estabilização
    up_mdelay(1);

    // Registra o ID lido no log
    sninfo("devid: 0x%02x\n", devid);

    // Verifica se o ID corresponde ao esperado
    if (devid != (uint16_t) DEVID) {

        // Registra erro indicando ID incorreto
        snerr("Wrong Device ID! %02x\n", devid);

        // Retorna erro de dispositivo não encontrado
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

    // Modifica apenas os bits de standby (bits 5-7)
    v_data_u8 = (v_data_u8 & ~(0x07 << 5)) | (value << 5);

    // Escreve o novo valor no registrador
    bmp280_putreg8(priv,                                // Estrutura privada do driver
                   BMP280_CONFIG,                       // Endereço do registrador de configuração
                   v_data_u8);                          // Novo valor

    // Relê o registrador de configuração
    v_data_u8 = bmp280_getreg8(priv,                    // Estrutura privada do driver
                               BMP280_CONFIG);          // Endereço do registrador de configuração

    // Extrai os bits de standby
    v_sb_u8 = (v_data_u8 >> 5) & 0x07;

    // Verifica se o valor aplicado corresponde ao esperado
    if (v_sb_u8 != value) {

        // Registra erro indicando falha na configuração
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

        // Retorna o código de erro
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

        // Registra erro indicando falha na configuração
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
    FAR struct bmp280_dev_s *priv = container_of(lower,     				// Estrutura do lower half
                                                 FAR struct bmp280_dev_s,   // Tipo da estrutura privada
                                                 sensor_lower);             // Campo do lower half

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

    // Bloco condicional: atualiza o intervalo interno apenas no modo POLL
#ifdef CONFIG_SENSORS_BMP280_POLL

    // Atualiza o intervalo interno do driver
    if (ret == 0) {

        // Armazena o novo intervalo para a thread
        priv->interval = *period_us;

        } // fim if

    // Fim do bloco condicional do modo POLL
#endif

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
//* DESCRIPTION: Habilita ou desabilita o sensor BMP280.
//*
//*              - No modo POLL: acorda ou bloqueia a thread conforme o estado.
//*              - No modo PULL: coloca o sensor em modo normal ou sleep.
//*
//* RETURNS: OK (0) sempre
//*
//**********************************************************************************************************************
static int bmp280_activate(FAR struct sensor_lowerhalf_s *lower, FAR struct file *filep, bool enable) {

    // Recupera a estrutura privada
    FAR struct bmp280_dev_s *priv = container_of(lower,     				// Estrutura do lower half
                                                 FAR struct bmp280_dev_s,   // Tipo da estrutura privada
                                                 sensor_lower);             // Campo do lower half

    // Retorno das chamadas
    int ret;

    // Bloco condicional: comportamento do modo POLL
#ifdef CONFIG_SENSORS_BMP280_POLL

    // Variável que indica se a thread deve ser acordada
    bool start_thread = false;

    // Se o sensor está sendo habilitado e ainda não estava
    if (enable && !priv->enabled) {

        // Marca que a thread deve ser acordada
        start_thread = true;

        } // fim if

    // Atualiza o estado de habilitação
    priv->enabled = enable;

    // Se a thread deve ser acordada
    if (start_thread) {

        // Acorda a thread
        nxsem_post(&priv->run);

        } // fim if

    // Fim do bloco condicional do modo POLL
#else

    // Bloco condicional: comportamento do modo PULL
    if (enable) {

        // Coloca o sensor em modo normal com resolução padrão
        ret = bmp280_putreg8(priv,                          					  // Estrutura privada do driver
                             BMP280_CTRL_MEAS,              					  // Registrador de controle de medição
                             BMP280_NORMAL_MODE | BMP280_OS_STANDARD_RES); 		  // Modo normal

        // Verifica se a operação falhou
        if (ret < 0) {

            // Retorna o código de erro
            return ret;

            } // fim if

        }
    else {

        // Coloca o sensor em modo sleep
        ret = bmp280_putreg8(priv,                        				   	   // Estrutura privada do driver
                             BMP280_CTRL_MEAS,            					   // Registrador de controle de medição
                             BMP280_SLEEP_MODE);         					   // Modo sleep

        } // fim if

    // Verifica se a operação foi bem-sucedida
    if (ret >= 0) {

        // Atualiza o estado de ativação
        priv->activated = enable;

        } // fim if

    // Fim do bloco condicional do modo PULL
#endif

    // Retorna sucesso
    return OK;

} // fim static int bmp280_activate(...)


//______________________________________________________________________________________________________________________


/* 
 * Bloco condicional: 
*/

// Função 'fetch' apenas, sem POLL
#ifndef CONFIG_SENSORS_BMP280_POLL

//**********************************************************************************************************************
//* FUNCTION: bmp280_fetch()
//*
//* PARAMETERS:
//*
//*   lower - Ponteiro para a estrutura do lower half do sensor
//*   filep - Ponteiro para a estrutura do arquivo (não utilizado)
//*   buffer - Buffer de destino para os dados lidos
//*   buflen - Tamanho do buffer em bytes
//*
//* DESCRIPTION: Realiza uma leitura sob demanda do BMP280. Se o sensor estiver em modo 'sleep', coloca-o em modo
//*              forçado para uma única medição. Lê os valores brutos, aplica as compensações e copia o resultado
//*              para o buffer fornecido.
//*
//* RETURNS: Número de bytes lidos (buflen) em caso de sucesso; código negativo em caso de erro
//*
//**********************************************************************************************************************
static int bmp280_fetch(FAR struct sensor_lowerhalf_s *lower, FAR struct file *filep, FAR char *buffer, size_t buflen) {

    // Recupera a estrutura privada
    FAR struct bmp280_dev_s *priv = container_of(lower,     				// Estrutura do lower half
                                                 FAR struct bmp280_dev_s,   // Tipo da estrutura privada
                                                 sensor_lower);             // Campo do lower half

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

    // Estrutura de saída
    struct sensor_baro baro_data;

    // Verifica o tamanho do buffer
    if (buflen != sizeof(baro_data)) {

        // Retorna erro de argumento inválido
        return -EINVAL;

        } // fim if

    // Verifica se o sensor está em modo sleep
    if (!priv->activated) {

        // Coloca o sensor em modo forçado para uma única medição
        ret = bmp280_putreg8(priv,                         						  // Estrutura privada do driver
                             BMP280_CTRL_MEAS,           						  // Registrador de controle de medição
                             BMP280_FORCED_MODE | 						          // Modo forçado
							 BMP280_OS_ULTRA_LOW_POWER); 						  // Modo Ultra Low-Power 
        // Verifica se a operação falhou
        if (ret < 0) {

            // Retorna o código de erro
            return ret;

            } // fim if

        // Aguarda 6 ms (modo Ultra Low-Power)
        up_mdelay(6);

        } // fim if

    // Lê os 6 bytes de dados
    ret = bmp280_getregs(priv,                        							  // Estrutura privada do driver
                         BMP280_PRESS_MSB,            							  // Endereço inicial dos dados
                         buf,                         							  // Buffer de destino
                         6);                          							  // Número de bytes

    // Verifica se a leitura falhou
    if (ret < 0) {

        // Retorna o código de erro
        return ret;

        } // fim if

    // Combina os bytes de pressão
    press = (uint32_t)COMBINE(buf);

    // Combina os bytes de temperatura
    temp = COMBINE(&buf[3]);

    // Aplica a compensação de temperatura
    temp = bmp280_compensate_temp(priv, temp);

    // Aplica a compensação de pressão
    press = bmp280_compensate_press(priv, press);

    // Obtém o timestamp do sistema
    clock_systime_timespec(&ts);

    // Calcula o timestamp em microssegundos
    baro_data.timestamp = 1000000ull * ts.tv_sec + ts.tv_nsec / 1000;

    // Converte a pressão para hPa
    baro_data.pressure = sensor_data_divi(press, 100);

    // Converte a temperatura para décimos de grau
    baro_data.temperature = sensor_data_divi(temp, 100);

    // Copia os dados para o buffer
    memcpy(buffer, &baro_data, sizeof(baro_data));

    // Retorna o número de bytes lidos
    return buflen;

} // fim static int bmp280_fetch(...)


// Fim do bloco condicional sem POLL
#endif



//______________________________________________________________________________________________________________________



/* 
 * Bloco condicional: 
*/

// 'Thread' do modo POLL
#ifdef CONFIG_SENSORS_BMP280_POLL

//**********************************************************************************************************************
//* FUNCTION: bmp280_thread()
//*
//* PARAMETERS:
//*
//*   argc - Número de argumentos da thread
//*   argv - Vetor de argumentos (argv[1] contém o ponteiro para a estrutura privada)
//*
//* DESCRIPTION: Thread dedicada do modo POLL. Aguarda no semáforo quando o sensor está desabilitado. Ao ser
//*              acordada, lê os valores brutos periodicamente, aplica as compensações e publica os dados via
//*              'push_event()'. Este é o coração do modelo PUSH no modo híbrido.
//*
//* RETURNS: OK (0) — a thread nunca retorna, permanece em loop
//*
//**********************************************************************************************************************
static int bmp280_thread(int argc, FAR char **argv) {

    // Recupera a estrutura privada a partir do argumento
    FAR struct bmp280_dev_s *priv = (FAR struct bmp280_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));

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

    // Loop infinito da thread
    while (true) {

        // Verifica se o sensor está desabilitado
        if (!priv->enabled) {

            // Bloqueia a thread aguardando o semáforo
            ret = nxsem_wait(&priv->run);

            // Verifica se a espera falhou
            if (ret < 0) {

                // Continua o loop
                continue;

                } // fim if

            } // fim if

        // Verifica se o sensor está habilitado
        if (priv->enabled) {

            // Lê os 6 bytes de dados
            ret = bmp280_getregs(priv,                          			// Estrutura privada do driver
                                 BMP280_PRESS_MSB,          			    // Endereço inicial dos dados
                                 buf,                        			    // Buffer de destino
                                 6);                          			    // Número de bytes

            // Verifica se a leitura falhou
            if (ret < 0) {

                // Registra erro indicando falha na leitura
                snerr("Failed to read sensor data: %d\n", ret);

                } // fim if

            // Se a leitura foi bem-sucedida
            if (ret == OK) {

                // Combina os bytes de pressão
                press = (uint32_t)COMBINE(buf);

                // Combina os bytes de temperatura
                temp = COMBINE(&buf[3]);

                // Aplica a compensação de temperatura
                temp = bmp280_compensate_temp(priv, temp);

                // Aplica a compensação de pressão
                press = bmp280_compensate_press(priv, press);

                // Obtém o timestamp do sistema
                clock_systime_timespec(&ts);

                // Calcula o timestamp em microssegundos
                baro.timestamp = 1000000ull * ts.tv_sec + ts.tv_nsec / 1000;

                // Converte a pressão para hPa
                baro.pressure = sensor_data_divi(press, 100);

                // Converte a temperatura para décimos de grau
                baro.temperature = sensor_data_divi(temp, 100);

                // Publica o evento via push_event
                priv->sensor_lower.push_event(priv->sensor_lower.priv,  // Estrutura privada do lowerhalf
                                              &baro,                    // Dados a publicar
                                              sizeof(baro));            // Tamanho dos dados

                } // fim if interno

            } // fim if externo

        // Aguarda o intervalo antes da próxima leitura
        nxsched_usleep(priv->interval);

        } // fim while

    // Retorna sucesso (nunca alcançado)
    return OK;

} // fim static int bmp280_thread(...)



// Fim do bloco condicional com POLL
#endif



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
//* DESCRIPTION: Inicializa a estrutura privada do driver, verifica o ID do dispositivo, inicializa o sensor
//*              e o registra no framework de sensores via 'sensor_register()'. No modo POLL, cria ainda a
//*              thread dedicada. Esta função é chamada pelo BSP, tipicamente em 'esp32s3_bringup()'.
//*
//* RETURNS: OK (0) em caso de sucesso; código negativo de erro em caso de falha
//*
//**********************************************************************************************************************
int bmp280_register(int devno, FAR struct i2c_master_s *i2c) {

    // Ponteiro para a estrutura privada do driver
    FAR struct bmp280_dev_s *priv;

    // Retorno das chamadas
    int ret;

    // Bloco condicional: variáveis para criação da thread no modo POLL
#ifdef CONFIG_SENSORS_BMP280_POLL

    // Vetor de argumentos para a thread
    FAR char *argv[2];

    // Buffer para o argumento (ponteiro da estrutura privada)
    char arg1[32];

    // Fim do bloco condicional de variáveis do modo POLL
#endif

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

    // Bloco condicional: inicialização dos campos do modo POLL
#ifdef CONFIG_SENSORS_BMP280_POLL

    // Define o estado inicial como desabilitado
    priv->enabled = false;

    // Define o intervalo inicial
    priv->interval = CONFIG_SENSORS_BMP280_POLL_INTERVAL;

    // Inicializa o semáforo de controle da thread
    nxsem_init(&priv->run, 0, 0);

    // Fim do bloco condicional de inicialização do modo POLL
#endif

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

        // Retorna o código de erro original
        return ret;

        } // fim if

    // Bloco condicional: criação da thread no modo POLL
#ifdef CONFIG_SENSORS_BMP280_POLL

    /*
     * >>> INTERFACE COM KTHREAD <<<
     *
     * Cria a thread dedicada que executará a leitura periódica e publicará os eventos
     * via 'push_event()'.
     */

    // Prepara o argumento com o ponteiro da estrutura privada
    snprintf(arg1, 16, "%p", priv);

    /*
     * Preenche o vetor de argumentos:
     */

    // Armazena o ponteiro da estrutura privada como primeiro argumento
    argv[0] = arg1;

    // Termina o vetor de argumentos com NULL
    argv[1] = NULL;

    // Cria a thread com prioridade padrão e tamanho de pilha configurado
    ret = kthread_create("bmp280_thread",                					   // Nome da thread
                         SCHED_PRIORITY_DEFAULT,           					   // Prioridade padrão do escalonador
                         CONFIG_SENSORS_BMP280_THREAD_STACKSIZE,			   // Tamanho da pilha
                         bmp280_thread,                     				   // Função de entrada da thread
                         argv);                             				   // Vetor de argumentos

    // Verifica se a criação da thread falhou
    if (ret < 0) {

        // Registra erro indicando falha na criação da thread
        snerr("Failed to create thread: %d\n", ret);

        // Desregistra o driver do framework de sensores
        sensor_unregister(&priv->sensor_lower, devno);

        // Libera a memória da estrutura privada
        kmm_free(priv);

        // Retorna o código de erro
        return ret;

        } // fim if

    // Fim do bloco condicional de criação da thread do modo POLL
#endif

    // Registra mensagem informativa indicando sucesso
    sninfo("BMP280 hybrid uORB driver loaded successfully!\n");

    // Retorna o resultado final
    return ret;

} // fim int bmp280_register(...)


// Fim do bloco condicional: apenas se I2C e BMP280 estiverem habilitados
#endif


//----------------------------------------------------------------------------------------------------------------------
// Fim do arquivo "bmp280_hybrid.c"
//----------------------------------------------------------------------------------------------------------------------

