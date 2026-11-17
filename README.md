# NUCLEO-F767ZI — BMP280 + MPU-6050

Firmware para a **NUCLEO-F767ZI** (STM32F767ZIT6 @ 216 MHz) lendo um barômetro **BMP280** e uma IMU **MPU-6050** no barramento `I2C1`, com filtro complementar no MCU, saída serial via ST-LINK VCP (`USART3`) e visualizador 3D em Three.js.

## Ligações na placa

Os dois sensores compartilham o `I2C1` nos pinos do conector Arduino (sem usar `PB7`, que divide pino com o `LD2`):

| Sinal | Pino NUCLEO | Conector | BMP280 | MPU-6050 | Observação |
| :--- | :--- | :--- | :--- | :--- | :--- |
| VCC | `3.3V` | `CN8 pino 7` | `VCC` | `VCC` | Alimentação 3.3V |
| GND | `GND` | `CN8 pino 9` | `GND` | `GND` | Terra comum |
| I2C1_SCL | `PB8` | `CN7 pino 2 (D15)` | `SCL / SCK` | `SCL` | 400 kHz (pull-up 4.7k recomendado) |
| I2C1_SDA | `PB9` | `CN7 pino 4 (D14)` | `SDA / SDI` | `SDA` | Open-drain |
| Modo I2C | `3.3V` | — | `CSB` | — | Precisa ficar em nível alto (`3.3V`) |
| Endereço BMP280 | `GND` ou `3.3V` | — | `SDO` | — | `GND` = `0x76` / `3.3V` = `0x77` |
| Endereço MPU-6050 | `GND` | — | — | `AD0` | `GND` = `0x68` |

Serial e LEDs on-board:
- **UART (`USART3` / ST-LINK VCP):** `PD8` (`TX`) e `PD9` (`RX`), `115200 8N1` pelo cabo USB da placa.
- **LD1 (verde, `PB0`):** pisca a cada 500 ms quando ambos os sensores estão respondendo.
- **LD3 (vermelho, `PB14`):** acende se algum sensor falhar ou desconectar (o firmware tenta reconectar a cada 100 ms).

## Notas de implementação

- **BMP280 ([`Core/Src/bmp280.c`](Core/Src/bmp280.c)):** sonda automaticamente `0x76` e `0x77` no boot, faz soft reset (`0xE0 = 0xB6`), espera `im_update` liberar antes de ler os 24 bytes de calibração (`0x88..0x9F`) e lê pressão + temperatura em burst de 6 bytes (`0xF7..0xFC`). A altitude usa a equação hipsométrica compensada por temperatura com `1013.25 hPa` como referência padrão.
- **MPU-6050 ([`Core/Src/Mpu6050.c`](Core/Src/Mpu6050.c), [`Core/Src/ComplementaryFilter.c`](Core/Src/ComplementaryFilter.c)):** leitura em burst de 14 bytes (`0x3B..0x48`, ±2g e ±250 °/s) a 100 Hz, com filtro complementar (`alpha = 0.98`) usando o `dt` real via `HAL_GetTick()`.
- **Serial ([`Core/Src/main.c`](Core/Src/main.c)):** ring buffer de 2 KB com TX não-bloqueante por interrupção (`USART3_IRQHandler`), enviando MPU/atitude a 50 Hz e BMP280 a 10 Hz.

## Build, testes e gravação

```bash
# Testes unitários no host (BMP280, edge cases, MPU-6050 e filtro)
make test

# Compilar firmware (.elf, .hex, .bin)
make all

# Gravar na placa via ST-LINK SWD (0x08000000, com verify e reset)
make flash

# Compilar frontend (se necessário) e abrir o visualizador 3D
make viewer
```

O target `flash` do [`Makefile`](Makefile) chama o `STM32_Programmer_CLI` do `PATH` ou da instalação padrão do STM32CubeIDE:

```bash
STM32_Programmer_CLI -c port=SWD -w build/nucleo_f767zi_bmp280_mpu6050.bin 0x08000000 -v -rst
```

Se o binário estiver em outro lugar, passe o caminho direto: `make flash CUBEPROG=/caminho/STM32_Programmer_CLI`.

## Visualizador 3D (`viewer/`)

Interface em Three.js ([`viewer/`](viewer/)) alimentada por [`viewer/server.py`](viewer/server.py) via WebSocket binário (`Float32Array` de 56 bytes), ou diretamente pelo navegador via WebSerial (Chrome/Chromium), além de um modo simulador offline.

- **Aeronave 3D ([`viewer/src/aircraft.ts`](viewer/src/aircraft.ts)):** move atitude por quaternion (roll, pitch, yaw), deflete superfícies pelos dados do giroscópio, gira hélice pelo VSI e exibe vetor de aceleração.
- **Barometria e HUD ([`viewer/src/telemetry.ts`](viewer/src/telemetry.ts), [`viewer/src/hud.ts`](viewer/src/hud.ts)):** recalcula altitude hipsométrica com ajuste de `QNH`, tem botão `Zero Alt` para altitude relativa, estima `VSI`, densidade do ar, carga G e plota PFD + gráficos em tempo real.

## Exemplo de saída serial

A saída usa apenas ASCII 7-bit (`degC` e `\r\n`, sem UTF-8 ou códigos ANSI), então funciona direto no CuteCom em `115200`, `8N1`, `CR, LF`, modo ASCII:

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
