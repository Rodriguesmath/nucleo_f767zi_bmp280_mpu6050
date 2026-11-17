# NUCLEO-F767ZI Dual-Sensor Monitor: BMP280 (I2C) + MPU-6050 (I2C)

Projeto unificado para a placa **NUCLEO-F767ZI** (**STM32F767ZIT6**, Arm Cortex-M7 @ 216 MHz com FPU de precisão dupla), integrando o barômetro **Bosch BMP280** (em modo **I2C** conforme datasheet Bosch) e o sensor IMU **MPU-6050** (6-DOF + Filtro Complementar) no mesmo barramento **I2C1**, com telemetria serial via **ST-LINK Virtual COM Port (USART3)**.

---

## 1. Conexões de Hardware na NUCLEO-F767ZI

Ambos os sensores compartilham o barramento **I2C1** nos pinos padrão Arduino da NUCLEO-144 (evitando `PB7`, que é compartilhado com o LED azul `LD2`):

| Sinal / Pino | Pino NUCLEO-F767ZI | Conector Arduino | BMP280 (I2C) | MPU-6050 (I2C) | Observação |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **VCC / VDD** | `3.3V` | `CN8 pino 7` | `VCC` | `VCC` | Alimentação 3.3V |
| **GND** | `GND` | `CN8 pino 9` | `GND` | `GND` | Referência comum |
| **I2C1_SCL** | `PB8` | `CN7 pino 2 (D15)` | `SCL / SCK` | `SCL` | Clock 400 kHz (pull-up 4.7k recomendado) |
| **I2C1_SDA** | `PB9` | `CN7 pino 4 (D14)` | `SDA / SDI` | `SDA` | Dados Open-Drain |
| **Seleção I2C BMP** | `3.3V` | — | `CSB` | — | **Obrigatório HIGH (VDDIO) para modo I2C** |
| **Endereço BMP280** | `GND` (ou `3.3V`) | — | `SDO` | — | `GND` = `0x76` (primário) / `3.3V` = `0x77` |
| **Endereço MPU-6050** | `GND` | — | — | `AD0` | `GND` = `0x68` |

### 1.1 Telemetria Serial e LEDs On-Board

- **ST-LINK VCP (`USART3`)**: `PD8` (`TX`) / `PD9` (`RX`), `115200 bps`, `8N1` direto pelo cabo USB do ST-LINK.
- **LD1 (Verde, `PB0`)**: Pisca a cada ciclo de telemetria (500 ms) quando ambos os sensores estão ativos.
- **LD3 (Vermelho, `PB14`)**: Acende em caso de falha ou desconexão de qualquer sensor (com tentativa automática de reconexão a cada 500 ms).

---

## 2. Conformidade com a Especificação I2C da Bosch (BMP280)

Implementação em [`Core/Src/bmp280.c`](Core/Src/bmp280.c):

- **Seleção de Modo e Endereçamento (Seções 5.1 e 5.2)**: Endereços 7-bit `0x76` (`SDO=GND`) e `0x77` (`SDO=VDDIO`) com sondagem automática no boot; bit `spi3w_en` (`0xF5[0]`) mantido em `0`.
- **Sequência de Configuração (Seções 4.3.4 e 4.3.5)**: Soft reset (`0xE0 = 0xB6`), espera mínima `>= 2 ms`, polling de `im_update` (`0xF3[0]`) antes da leitura dos 24 bytes de calibração (`0x88..0x9F`), e entrada em `SLEEP` antes de atualizar `CONFIG` (`0xF5`) e `CTRL_MEAS` (`0xF4`).
- **Protocolo de Escrita/Leitura I2C (Seções 3.9, 5.2.1 e 5.2.2)**: Escritas sem auto-incremento (pares `[reg, val]`) e leitura em burst único de 6 bytes (`0xF7..0xFC`) via Repeated Start (`HAL_I2C_Mem_Read`) para garantir coerência entre MSB/LSB/XLSB.
- **Compensação & Altitude (Seção 8)**: Fórmulas exatas Bosch (`int32`, `int64`, `float`, atualizando `t_fine` pela temperatura antes da pressão) e fórmula hipsométrica compensada por temperatura.

---

## 3. Fusão Sensorial MPU-6050 (100 Hz)

Implementação em [`Core/Src/Mpu6050.c`](Core/Src/Mpu6050.c) e [`Core/Src/ComplementaryFilter.c`](Core/Src/ComplementaryFilter.c):

- Amostragem a **100 Hz** (`10 ms`) em burst de 14 registradores (`0x3B..0x48`: acelerômetro ±2g, temperatura interna, giroscópio ±250 °/s).
- Filtro complementar ($\alpha = 0.98$) usando intervalo real $\Delta t$ medido via `HAL_GetTick()`, mantendo precisão da integração mesmo durante transmissões UART.

---

## 4. Compilação, Testes e Gravação

```bash
# 1. Executar suíte de testes unitários no host (BMP280 + edge cases + MPU6050 + barramento compartilhado)
make test

# 2. Compilar firmware STM32F767ZIT6 (.elf, .hex, .bin)
make all

# 3. Gravar (flashar) firmware na NUCLEO-F767ZI via ST-LINK SWD (0x08000000 + verify + reset)
make flash

# 4. Executar testes do bridge serial e abrir visualizador 3D Three.js
make viewer
```

> `make flash` invoca o `STM32_Programmer_CLI` (`CUBEPROG` no [`Makefile`](Makefile), detectado no `PATH` ou na instalação padrão do STM32CubeIDE):
>
> ```bash
> STM32_Programmer_CLI -c port=SWD -w build/nucleo_f767zi_bmp280_mpu6050.bin 0x08000000 -v -rst
> ```
>
> Caso o `STM32_Programmer_CLI` esteja em outro diretório, sobrescreva com `make flash CUBEPROG=/caminho/STM32_Programmer_CLI`.

---

## 5. Visualizador 3D em Three.js & Flight HUD (`viewer/`)

Aplicação WebGL2 de alta taxa de quadros (`60-144 FPS`) em [`viewer/`](viewer/) conectada ao bridge serial binário (`Float32Array` de 56 bytes via WebSocket em [`viewer/server.py`](viewer/server.py)):

- **Aeronave 3D Articulada ([`viewer/src/aircraft.ts`](viewer/src/aircraft.ts))**: Fuselagem, asas e superfícies móveis (ailerons, profundor, leme) acionadas pelas taxas do giroscópio (`gyroX/Y/Z`), hélice animada pelo VSI, luzes de navegação e vetor 3D de aceleração/carga G (`accelX/Y/Z`).
- **Aproveitamento Total dos Sensores ([`viewer/src/telemetry.ts`](viewer/src/telemetry.ts), [`viewer/src/hud.ts`](viewer/src/hud.ts))**:
  - **Atitude**: Quaternion SLERP a partir de `Roll`, `Pitch`, `Yaw` (50 Hz no stream UART não-bloqueante por interrupção `USART3_IRQHandler`).
  - **Barometria BMP280 (10 Hz)**: Altitude hipsométrica com ajuste de `QNH (hPa)`, altitude relativa (`Zero Alt`), `VSI (dH/dt)` e densidade do ar $\rho = \frac{P}{287.058 \cdot T}$ (`kg/m³`).
  - **Primary Flight Display (PFD) & Gráficos Ring-Buffer**: Horizonte artificial, bússola, altímetro/VSI, indicador de derrapagem (`accelY`), carga G (`|a|`), $\Delta T$ (`IMU - OAT`) e gráficos em `Float32Array` sem alocação por quadro.
  - **Modos**: Bridge Python (`Firefox` / qualquer navegador), `WebSerial` direto (`Chromium/Chrome`) e `Simulador` offline integrado.

### Exemplo de Saída Serial (`115200 8N1` — 50 Hz IMU / 10 Hz BMP280, ASCII 7-bit CuteCom)

> Saída UART utiliza exclusivamente ASCII 7-bit imprimível (`degC`, `\r\n`, sem sequências UTF-8 multi-byte ou códigos ANSI), evitando caracteres corrompidos (`\0xc2\0xb0` ou `Â°`) no CuteCom. Configure o CuteCom em `115200`, `8N1`, `CR, LF` e modo ASCII (sem `Hex output`).

```text
=======================================================================
  NUCLEO-F767ZI Dual-Sensor Monitor: BMP280 (I2C) + MPU-6050 (I2C)
  SYSCLK: 216MHz | I2C1: PB8(SCL)/PB9(SDA) @ 400kHz | VCP: PD8/PD9
=======================================================================
[INFO] Probing BMP280 over I2C (0x76 / 0x77)...
[OK] BMP280 detected over I2C at 0x76! Chip ID: 0x58
[INFO] Calibration: dig_T1=27504, dig_T2=26435, dig_T3=-1000
[INFO] Calibration: dig_P1=36477, dig_P2=-10685, dig_P3=3024
[INFO] Probing MPU-6050 over I2C at 0x68...
[OK] MPU-6050 detected over I2C at 0x68! WHO_AM_I: 0x68
[NUCLEO-F767ZI BMP280 I2C] Temp:  24.85 degC | Press:  1012.30 hPa ( 101230 Pa) | Hypsometric Alt:    7.84 m
[NUCLEO-F767ZI MPU6050 I2C] Acc:  +0.01, -0.02, +1.00 g | Gyro:   +0.12,  -0.05,  +0.03 dps | Temp: 26.40 degC | Pose R:  -1.1 P:  +0.6 Y:  +0.0 deg
```
