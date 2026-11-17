export type TransportMode = 'standby' | 'bridge' | 'webserial' | 'simulator';

export interface SerialPortInfo {
  device: string;
  description: string;
}

export interface BridgeStatus {
  type: 'status';
  connected: boolean;
  port: string | null;
  baud: number;
  ports: SerialPortInfo[];
  error: string | null;
}

export interface TelemetryData {
  rollDeg: number;
  pitchDeg: number;
  yawDeg: number;
  accelXG: number;
  accelYG: number;
  accelZG: number;
  gyroXDps: number;
  gyroYDps: number;
  gyroZDps: number;
  mpuTempC: number;
  bmpTempC: number;
  bmpPressHpa: number;
  bmpAltM: number;
  packetRateHz: number;
  gLoad: number;
  airDensityKgM3: number;
  hypsometricAltM: number;
  relAltM: number;
  vsiMs: number;
  deltaTempC: number;
}

const BMP_RE =
  /BMP280[^\]]*\]\s*Temp:\s*([+-]?\d+(?:\.\d+)?)\s*(?:degC|°C)\s*\|\s*Press:\s*([+-]?\d+(?:\.\d+)?)\s*hPa.*?\|\s*Hypsometric Alt:\s*([+-]?\d+(?:\.\d+)?)\s*m/;
const MPU_RE =
  /MPU6050[^\]]*\]\s*Acc:\s*([+-]?\d+(?:\.\d+)?)\s*,\s*([+-]?\d+(?:\.\d+)?)\s*,\s*([+-]?\d+(?:\.\d+)?)\s*g\s*\|\s*Gyro:\s*([+-]?\d+(?:\.\d+)?)\s*,\s*([+-]?\d+(?:\.\d+)?)\s*,\s*([+-]?\d+(?:\.\d+)?)\s*dps\s*\|\s*Temp:\s*([+-]?\d+(?:\.\d+)?)\s*(?:degC|°C)\s*\|\s*Pose\s*R:\s*([+-]?\d+(?:\.\d+)?)\s*P:\s*([+-]?\d+(?:\.\d+)?)\s*Y:\s*([+-]?\d+(?:\.\d+)?)\s*deg/;

export class TelemetryClient {
  public mode: TransportMode = 'standby';
  public qnhHpa = 1013.25;
  public data: TelemetryData = {
    rollDeg: 0,
    pitchDeg: 0,
    yawDeg: 0,
    accelXG: 0,
    accelYG: 0,
    accelZG: 1,
    gyroXDps: 0,
    gyroYDps: 0,
    gyroZDps: 0,
    mpuTempC: 25,
    bmpTempC: 25,
    bmpPressHpa: 1013.25,
    bmpAltM: 0,
    packetRateHz: 0,
    gLoad: 1,
    airDensityKgM3: 1.225,
    hypsometricAltM: 0,
    relAltM: 0,
    vsiMs: 0,
    deltaTempC: 0,
  };

  private rawRoll = 0;
  private rawPitch = 0;
  private rawYaw = 0;
  private rollOffset = 0;
  private pitchOffset = 0;
  private yawOffset = 0;
  private altBaseM = 0;
  private altInitialized = false;
  private lastAltM = 0;
  private lastAltTimeSec = 0;

  private ws: WebSocket | null = null;
  private wsReconnectTimer: number | null = null;
  private simTime = 0;
  private simLastYaw = 0;
  private simLastPitch = 0;
  private simLastRoll = 0;

  private webSerialReader: ReadableStreamDefaultReader<Uint8Array> | null = null;
  private webSerialPort: { close: () => Promise<void> } | null = null;
  private webSerialRateCount = 0;
  private webSerialRateStart = performance.now();

  public onStatus?: (status: BridgeStatus) => void;
  public onLog?: (line: string) => void;
  public onModeChange?: (mode: TransportMode) => void;

  public startWebSocket(url = 'ws://127.0.0.1:8765'): void {
    this.stopWebSerial();
    if (this.ws && (this.ws.readyState === WebSocket.OPEN || this.ws.readyState === WebSocket.CONNECTING)) {
      return;
    }

    const ws = new WebSocket(url);
    ws.binaryType = 'arraybuffer';
    this.ws = ws;

    ws.onmessage = (ev: MessageEvent) => {
      if (ev.data instanceof ArrayBuffer) {
        if (this.mode !== 'bridge') {
          this.setMode('bridge');
        }
        this.applyBinaryFrame(new Float32Array(ev.data));
      } else if (typeof ev.data === 'string') {
        try {
          const msg = JSON.parse(ev.data) as BridgeStatus | { type: 'log'; line: string };
          if (msg.type === 'status') {
            if (msg.connected && this.mode !== 'simulator') {
              this.setMode('bridge');
            } else if (!msg.connected && this.mode === 'bridge') {
              this.setMode('standby');
            }
            this.onStatus?.(msg);
          } else if (msg.type === 'log') {
            this.onLog?.(msg.line);
          }
        } catch {
          // Ignore malformed control payload
        }
      }
    };

    ws.onclose = () => {
      this.ws = null;
      if (this.mode === 'bridge') {
        this.setMode('standby');
      }
      if (this.wsReconnectTimer === null) {
        this.wsReconnectTimer = window.setTimeout(() => {
          this.wsReconnectTimer = null;
          if (this.mode !== 'webserial') {
            this.startWebSocket(url);
          }
        }, 2000);
      }
    };
  }

  public sendBridgeCommand(cmd: Record<string, unknown>): void {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(cmd));
    }
  }

  public async connectWebSerial(baudRate = 115200): Promise<void> {
    const navSerial = (navigator as unknown as {
      serial?: {
        requestPort: () => Promise<{
          open: (opts: { baudRate: number }) => Promise<void>;
          close: () => Promise<void>;
          readable: ReadableStream<Uint8Array> | null;
        }>;
      };
    }).serial;

    if (!navSerial) {
      this.onLog?.('[ERR] Web Serial API indisponivel neste navegador (use Bridge Python ou Chromium).');
      return;
    }

    try {
      await this.stopWebSerial();
      const port = await navSerial.requestPort();
      await port.open({ baudRate });
      this.webSerialPort = port;
      this.setMode('webserial');
      this.onLog?.(`[OK] Web Serial conectado @ ${baudRate} bps`);

      const decoder = new TextDecoder('ascii');
      let buffer = '';
      const reader = port.readable?.getReader();
      if (!reader) return;
      this.webSerialReader = reader;

      while (this.mode === 'webserial') {
        const { value, done } = await reader.read();
        if (done) break;
        if (!value) continue;
        buffer += decoder.decode(value, { stream: true });
        let nlIdx = buffer.indexOf('\n');
        while (nlIdx !== -1) {
          const line = buffer.slice(0, nlIdx).trim();
          buffer = buffer.slice(nlIdx + 1);
          this.parseAsciiLine(line);
          nlIdx = buffer.indexOf('\n');
        }
      }
    } catch (err) {
      this.onLog?.(`[WARN] Web Serial: ${String(err)}`);
      if (this.mode === 'webserial') {
        this.setMode('standby');
      }
    }
  }

  public async stopWebSerial(): Promise<void> {
    try {
      await this.webSerialReader?.cancel();
    } catch {
      // Ignore cancel error
    }
    this.webSerialReader = null;
    try {
      await this.webSerialPort?.close();
    } catch {
      // Ignore close error
    }
    this.webSerialPort = null;
  }

  public toggleSimulator(): void {
    if (this.mode === 'simulator') {
      this.setMode('standby');
    } else {
      this.altInitialized = false;
      this.setMode('simulator');
    }
  }

  public tarePose(): void {
    this.rollOffset = this.rawRoll;
    this.pitchOffset = this.rawPitch;
    this.yawOffset = this.rawYaw;
    this.updateDerivedMetrics();
  }

  public zeroAltitude(): void {
    this.altBaseM = this.data.hypsometricAltM;
    this.altInitialized = true;
    this.data.relAltM = 0;
  }

  public setQnh(qnh: number): void {
    if (Number.isFinite(qnh) && qnh >= 850 && qnh <= 1100) {
      this.qnhHpa = qnh;
      this.updateDerivedMetrics();
    }
  }

  public stepSimulator(dtSec: number): void {
    if (this.mode !== 'simulator') return;
    this.simTime += dtSec;

    const targetRoll = 28 * Math.sin(this.simTime * 0.75) + 8 * Math.sin(this.simTime * 2.1);
    const targetPitch = 12 * Math.sin(this.simTime * 0.55) + 3 * Math.cos(this.simTime * 1.7);
    const targetYaw = (this.simTime * 18) % 360;

    const safeDt = Math.max(dtSec, 1e-3);
    const gx = (targetRoll - this.simLastRoll) / safeDt;
    const gy = (targetPitch - this.simLastPitch) / safeDt;
    let dyaw = targetYaw - this.simLastYaw;
    if (dyaw < -180) dyaw += 360;
    const gz = dyaw / safeDt;

    this.simLastRoll = targetRoll;
    this.simLastPitch = targetPitch;
    this.simLastYaw = targetYaw;

    const altSim = 12 + 8 * Math.sin(this.simTime * 0.35) + 1.2 * Math.sin(this.simTime * 1.4);
    const pressSim = 1013.25 * Math.pow(1 - altSim / 44330, 1 / 0.1902949);
    const tempSim = 24.5 - 0.0065 * altSim + 0.15 * Math.sin(this.simTime * 0.3);

    const rollRad = (targetRoll * Math.PI) / 180;
    const pitchRad = (targetPitch * Math.PI) / 180;

    this.rawRoll = targetRoll;
    this.rawPitch = targetPitch;
    this.rawYaw = targetYaw;

    this.data.accelXG = -Math.sin(pitchRad);
    this.data.accelYG = Math.sin(rollRad) * Math.cos(pitchRad);
    this.data.accelZG = Math.cos(rollRad) * Math.cos(pitchRad) + 0.08 * Math.sin(this.simTime * 3.1);
    this.data.gyroXDps = gx;
    this.data.gyroYDps = gy;
    this.data.gyroZDps = gz;
    this.data.mpuTempC = 29.8 + 0.2 * Math.sin(this.simTime * 0.2);
    this.data.bmpTempC = tempSim;
    this.data.bmpPressHpa = pressSim;
    this.data.bmpAltM = altSim;
    this.data.packetRateHz = 1 / safeDt;

    this.updateDerivedMetrics();
  }

  private setMode(next: TransportMode): void {
    this.mode = next;
    this.onModeChange?.(next);
  }

  private applyBinaryFrame(arr: Float32Array): void {
    if (arr.length < 14) return;
    this.rawRoll = arr[0];
    this.rawPitch = arr[1];
    this.rawYaw = arr[2];
    this.data.accelXG = arr[3];
    this.data.accelYG = arr[4];
    this.data.accelZG = arr[5];
    this.data.gyroXDps = arr[6];
    this.data.gyroYDps = arr[7];
    this.data.gyroZDps = arr[8];
    this.data.mpuTempC = arr[9];
    this.data.bmpTempC = arr[10];
    this.data.bmpPressHpa = arr[11];
    this.data.bmpAltM = arr[12];
    this.data.packetRateHz = arr[13];
    this.updateDerivedMetrics();
  }

  private parseAsciiLine(line: string): void {
    if (!line) return;
    const mpu = MPU_RE.exec(line);
    if (mpu) {
      this.data.accelXG = parseFloat(mpu[1]);
      this.data.accelYG = parseFloat(mpu[2]);
      this.data.accelZG = parseFloat(mpu[3]);
      this.data.gyroXDps = parseFloat(mpu[4]);
      this.data.gyroYDps = parseFloat(mpu[5]);
      this.data.gyroZDps = parseFloat(mpu[6]);
      this.data.mpuTempC = parseFloat(mpu[7]);
      this.rawRoll = parseFloat(mpu[8]);
      this.rawPitch = parseFloat(mpu[9]);
      this.rawYaw = parseFloat(mpu[10]);
      this.bumpWebSerialRate();
      this.updateDerivedMetrics();
      return;
    }

    const bmp = BMP_RE.exec(line);
    if (bmp) {
      this.data.bmpTempC = parseFloat(bmp[1]);
      this.data.bmpPressHpa = parseFloat(bmp[2]);
      this.data.bmpAltM = parseFloat(bmp[3]);
      this.bumpWebSerialRate();
      this.updateDerivedMetrics();
      return;
    }

    if (line.startsWith('[') || line.startsWith('Check')) {
      this.onLog?.(line);
    }
  }

  private bumpWebSerialRate(): void {
    this.webSerialRateCount++;
    const now = performance.now();
    const dt = (now - this.webSerialRateStart) / 1000;
    if (dt >= 0.5) {
      this.data.packetRateHz = this.webSerialRateCount / dt;
      this.webSerialRateCount = 0;
      this.webSerialRateStart = now;
    }
  }

  private updateDerivedMetrics(): void {
    this.data.rollDeg = this.rawRoll - this.rollOffset;
    this.data.pitchDeg = this.rawPitch - this.pitchOffset;
    this.data.yawDeg = this.rawYaw - this.yawOffset;

    const ax = this.data.accelXG;
    const ay = this.data.accelYG;
    const az = this.data.accelZG;
    this.data.gLoad = Math.sqrt(ax * ax + ay * ay + az * az);

    const tempK = Math.max(this.data.bmpTempC + 273.15, 1.0);
    const pressPa = Math.max(this.data.bmpPressHpa * 100, 1.0);
    this.data.airDensityKgM3 = pressPa / (287.058 * tempK);

    // Live QNH-compensated hypsometric altitude matching BMP280 firmware formula
    const hypsoAlt = (tempK / 0.0065) * (1 - Math.pow(this.data.bmpPressHpa / this.qnhHpa, 0.1902949));
    this.data.hypsometricAltM = Number.isFinite(hypsoAlt) ? hypsoAlt : this.data.bmpAltM;

    if (!this.altInitialized) {
      this.altBaseM = this.data.hypsometricAltM;
      this.lastAltM = this.data.hypsometricAltM;
      this.lastAltTimeSec = performance.now() / 1000;
      this.altInitialized = true;
    }

    this.data.relAltM = this.data.hypsometricAltM - this.altBaseM;
    this.data.deltaTempC = this.data.mpuTempC - this.data.bmpTempC;

    const nowSec = performance.now() / 1000;
    const dt = nowSec - this.lastAltTimeSec;
    if (dt >= 0.08) {
      const rawVsi = (this.data.hypsometricAltM - this.lastAltM) / dt;
      this.data.vsiMs = this.data.vsiMs * 0.72 + rawVsi * 0.28;
      this.lastAltM = this.data.hypsometricAltM;
      this.lastAltTimeSec = nowSec;
    }
  }
}
