#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/timer.h" 

/* ── MAPEAMENTO DE HARDWARE (Pinos Físicos -> GP) ──────────────────────────── */
#define PINO_SENSOR  26       /* ADC0 (Pino Físico 31) - Divisor resistivo      */
#define PINO_POT     27       /* ADC1 (Pino Físico 32) - Potenciômetro de alvo  */
#define ADC_CH_POT   1        /* Canal MUX do ADC para o GP27                   */
#define PINO_PULSO   15       /* GP15 (Pino Físico 20) - Pulso Anti-eletrólise  */
#define PINO_RELE    10       /* GP10 (Pino Físico 14) - Aciona a Lâmpada       */

/* ── DISPLAY LCD I2C (HD44780 16x2) ────────────────────────────────────────── */
#define LCD_SDA      20       /* I2C0 SDA (Pino Físico 26)                      */
#define LCD_SCL      21       /* I2C0 SCL (Pino Físico 27)                      */
#define LCD_I2C      i2c0     /* Barramento de hardware                         */
#define LCD_ADDR     0x27     /* Endereço padrão PCF8574                        */

/* ── PARÂMETROS METROLÓGICOS E TIMING ──────────────────────────────────────── */
#define AMOSTRAS           20       /* Filtro de rajada (Oversampling)          */
#define HISTERESE          5        /* Lâmpada liga apenas se Umidade >= Alvo + 5% */
#define INTERVALO_CICLO_US 500000   /* 500ms entre as medições do ADC           */
#define TEMPO_ACOMODACAO_US 1000    /* 1ms para a carga estabilizar na esponja  */

/* ── MÁQUINA DE ESTADOS (FSM) ──────────────────────────────────────────────── */
typedef enum {
    ESTADO_ESPERANDO_CICLO,
    ESTADO_ACOMODANDO_TENSAO
} EstadoSensor_t;

/* ═══════════════════════════════════════════════════════════════════════════
 * DRIVER BARE METAL: LCD HD44780
 * ═══════════════════════════════════════════════════════════════════════════ */
#define LCD_BL  (1 << 3)
#define LCD_EN  (1 << 2)
#define LCD_RS  (1 << 0)

static void lcd_write_i2c(uint8_t byte) {
    uint8_t buf = byte | LCD_BL;
    i2c_write_blocking(LCD_I2C, LCD_ADDR, &buf, 1, false);
}

static void lcd_pulse_enable(uint8_t data) {
    lcd_write_i2c(data | LCD_EN);
    sleep_us(1); 
    lcd_write_i2c(data & ~LCD_EN);
    sleep_us(50);
}

static void lcd_send(uint8_t value, uint8_t mode) {
    uint8_t high = (value & 0xF0)        | mode;
    uint8_t low  = ((value << 4) & 0xF0) | mode;
    lcd_pulse_enable(high);
    lcd_pulse_enable(low);
}

static void lcd_cmd(uint8_t cmd) { lcd_send(cmd, 0);      }
static void lcd_char(char c)     { lcd_send(c,   LCD_RS); }

static void lcd_init(void) {
    sleep_ms(50); 
    lcd_pulse_enable(0x30); sleep_ms(5);
    lcd_pulse_enable(0x30); sleep_us(150);
    lcd_pulse_enable(0x30);
    lcd_pulse_enable(0x20);  
    lcd_cmd(0x28);           
    lcd_cmd(0x0C);           
    lcd_cmd(0x06);           
    lcd_cmd(0x01);           
    sleep_ms(2);             
}

static void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t offsets[] = {0x00, 0x40};
    lcd_cmd(0x80 | (col + offsets[row]));
}

static void lcd_print(const char *str) {
    while (*str) lcd_char(*str++);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FUNÇÕES MATEMÁTICAS E DE CONVERSÃO
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Aplica a regressão polinomial obtida do diagnóstico empírico na bancada */
static uint8_t adc_para_pct(uint16_t adc) {
    float x = (float)adc;
    
    /* 1. Cálculo da equação de calibração (saída em escala fracionária unitária) */
    float y = (-1.742e-7f * x * x) + (1.805e-3f * x) - 3.412f;

    /* 2. Ajuste de escala (Transforma a fração unitária em porcentagem bruta) */
    float percentual = y * 100.0f;

    /* 3. Clamping: Protege o sistema contra ruídos e *overflow* mecânico */
    if (percentual > 100.0f) return 100;
    if (percentual < 0.0f)   return 0;
    
    return (uint8_t)percentual;
}

/* Lê o potenciômetro e converte para degraus absolutos de 5% */
static uint8_t ler_target_pct(void) {
    adc_select_input(ADC_CH_POT);
    uint16_t raw = adc_read();                      
    uint8_t  pct = (uint8_t)((raw * 100UL) / 4095);
    pct = ((pct + 2) / 5) * 5;                     
    if (pct > 100) pct = 100;
    return pct;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * LOOP DE CONTROLE (ENTRY POINT)
 * ═══════════════════════════════════════════════════════════════════════════ */
int main(void) {
    stdio_init_all();

    /* Subsistema Analógico */
    adc_init();
    adc_gpio_init(PINO_SENSOR);
    adc_gpio_init(PINO_POT);

    /* Subsistema de Potência (Transistores e Relés) */
    gpio_init(PINO_PULSO);
    gpio_set_dir(PINO_PULSO, GPIO_OUT);
    gpio_put(PINO_PULSO, 0); 

    gpio_init(PINO_RELE);
    gpio_set_dir(PINO_RELE, GPIO_OUT);
    gpio_put(PINO_RELE, 0);

    /* Inicialização I2C e Display */
    i2c_init(LCD_I2C, 100 * 1000);              
    gpio_set_function(LCD_SDA, GPIO_FUNC_I2C);
    gpio_set_function(LCD_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(LCD_SDA);
    gpio_pull_up(LCD_SCL);
    lcd_init();

    printf("--- Inicializando FSM de Automacao c/ Histerese (Jay Protocol) ---\n");

    /* Variáveis da FSM e Controle de Tempo */
    uint32_t tempo_ultimo_ciclo = time_us_32();
    uint32_t tempo_pulso_ligado = 0;
    EstadoSensor_t estado_atual = ESTADO_ESPERANDO_CICLO;
    
    uint16_t adc_sensor  = 0;
    uint8_t  umidade_pct = 0;
    uint8_t  target_pct  = 0;
    
    /* Memória volátil do estado da lâmpada para operação do Schmitt Trigger */
    bool lampada_ligada = false;

    while (true) {
        uint32_t tempo_atual = time_us_32();

        switch (estado_atual) {
            
            case ESTADO_ESPERANDO_CICLO:
                if (tempo_atual - tempo_ultimo_ciclo >= INTERVALO_CICLO_US) {
                    gpio_put(PINO_PULSO, 1);
                    tempo_pulso_ligado = tempo_atual;
                    estado_atual = ESTADO_ACOMODANDO_TENSAO;
                }
                break;

            case ESTADO_ACOMODANDO_TENSAO:
                if (tempo_atual - tempo_pulso_ligado >= TEMPO_ACOMODACAO_US) {
                    
                    /* 1. AQUISIÇÃO (Oversampling) */
                    uint32_t soma_adc = 0;
                    adc_select_input(0); 
                    for (int i = 0; i < AMOSTRAS; i++) {
                        soma_adc += adc_read();
                    }
                    gpio_put(PINO_PULSO, 0); /* Corte da eletrólise */
                    
                    /* 2. PROCESSAMENTO DIGITAL */
                    adc_sensor = soma_adc / AMOSTRAS;
                    umidade_pct = adc_para_pct(adc_sensor);
                    target_pct  = ler_target_pct();

                    /* 3. ATUAÇÃO: SCHMITT TRIGGER (Lógica de Exaustão/Secagem) */
                    /* Lâmpada LIGA apenas se passar do teto (Alvo + 5%) */
                    if (umidade_pct >= (target_pct + HISTERESE)) {
                        lampada_ligada = true;
                    } 
                    /* Lâmpada DESLIGA apenas ao secar até atingir o alvo exato */
                    else if (umidade_pct <= target_pct) {
                        lampada_ligada = false;
                    }
                    /* Entre o alvo e o teto, a lâmpada preserva o estado anterior (Zona Morta) */
                    
                    gpio_put(PINO_RELE, lampada_ligada ? 1 : 0);

                    /* 4. INTERFACE HOMEM-MÁQUINA (LCD 16x2) */
                    char linha0[17], linha1[17];
                    // Formatação exata para caber em 16 colunas. %3d garante alinhamento visual
                    snprintf(linha0, sizeof(linha0), "UH: %3d%%  [%s]", umidade_pct, lampada_ligada ? "ON" : "  ");
                    snprintf(linha1, sizeof(linha1), "Alvo: %3d%%      ", target_pct);
                    
                    lcd_set_cursor(0, 0); lcd_print(linha0);
                    lcd_set_cursor(0, 1); lcd_print(linha1);

                    /* 5. TELEMETRIA UART */
                    printf("ADC: %4u | Umid: %3u%% | Alvo: %3u%% | Teto: %3u%% | Lamp: %s\n",
                           adc_sensor, umidade_pct, target_pct, (target_pct + HISTERESE), lampada_ligada ? "LIGADA" : "DESLIGADA");

                    tempo_ultimo_ciclo = tempo_atual;
                    estado_atual = ESTADO_ESPERANDO_CICLO;
                }
                break;
        }
    }

    return 0;
}