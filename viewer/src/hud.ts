import type { TelemetryData } from './telemetry';

class RingSeries {
  public readonly buf: Float32Array;
  private head = 0;
  private count = 0;

  constructor(public readonly capacity: number) {
    this.buf = new Float32Array(capacity);
  }

  public push(v: number): void {
    this.buf[this.head] = v;
    this.head = (this.head + 1) % this.capacity;
    if (this.count < this.capacity) {
      this.count++;
    }
  }

  public get(idxFromOldest: number): number {
    const start = (this.head - this.count + this.capacity) % this.capacity;
    return this.buf[(start + idxFromOldest) % this.capacity];
  }

  public length(): number {
    return this.count;
  }
}

export class FlightHud {
  private readonly pfdCtx: CanvasRenderingContext2D;
  private readonly altCtx: CanvasRenderingContext2D;
  private readonly gyroCtx: CanvasRenderingContext2D;
  private readonly accCtx: CanvasRenderingContext2D;

  private readonly altSeries = new RingSeries(240);
  private readonly gxSeries = new RingSeries(240);
  private readonly gySeries = new RingSeries(240);
  private readonly gzSeries = new RingSeries(240);
  private readonly axSeries = new RingSeries(240);
  private readonly aySeries = new RingSeries(240);
  private readonly azSeries = new RingSeries(240);
  private readonly gLoadSeries = new RingSeries(240);

  public showPfd = true;

  constructor(
    private readonly pfdCanvas: HTMLCanvasElement,
    altCanvas: HTMLCanvasElement,
    gyroCanvas: HTMLCanvasElement,
    accCanvas: HTMLCanvasElement
  ) {
    const pfd = pfdCanvas.getContext('2d');
    const alt = altCanvas.getContext('2d');
    const gyro = gyroCanvas.getContext('2d');
    const acc = accCanvas.getContext('2d');
    if (!pfd || !alt || !gyro || !acc) {
      throw new Error('Failed to acquire 2D canvas contexts');
    }
    this.pfdCtx = pfd;
    this.altCtx = alt;
    this.gyroCtx = gyro;
    this.accCtx = acc;
  }

  public resizePfd(width: number, height: number): void {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    this.pfdCanvas.width = Math.floor(width * dpr);
    this.pfdCanvas.height = Math.floor(height * dpr);
    this.pfdCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  public pushSample(d: TelemetryData): void {
    this.altSeries.push(d.relAltM);
    this.gxSeries.push(d.gyroXDps);
    this.gySeries.push(d.gyroYDps);
    this.gzSeries.push(d.gyroZDps);
    this.axSeries.push(d.accelXG);
    this.aySeries.push(d.accelYG);
    this.azSeries.push(d.accelZG);
    this.gLoadSeries.push(d.gLoad);
  }

  public render(d: TelemetryData, viewW: number, viewH: number): void {
    this.drawPfd(d, viewW, viewH);
    this.drawSingleChart(this.altCtx, this.altSeries, '#38bdf8', 'm');
    this.drawTripleChart(this.gyroCtx, this.gxSeries, this.gySeries, this.gzSeries, 120, ['#f43f5e', '#22c55e', '#38bdf8']);
    this.drawTripleChart(this.accCtx, this.axSeries, this.aySeries, this.gLoadSeries, 2.5, ['#f43f5e', '#22c55e', '#f59e0b']);
  }

  private drawPfd(d: TelemetryData, w: number, h: number): void {
    const ctx = this.pfdCtx;
    ctx.clearRect(0, 0, w, h);
    if (!this.showPfd) return;

    const cx = w * 0.5;
    const cy = h * 0.52;
    const radius = Math.min(w, h) * 0.2;

    ctx.save();
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.clip();

    // Artificial Horizon Pitch Ladder
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate((-d.rollDeg * Math.PI) / 180);
    const pitchPx = d.pitchDeg * (radius / 35);
    ctx.translate(0, pitchPx);

    ctx.strokeStyle = 'rgba(56, 189, 248, 0.72)';
    ctx.fillStyle = 'rgba(56, 189, 248, 0.85)';
    ctx.lineWidth = 1.5;
    ctx.font = '10px monospace';
    ctx.textAlign = 'right';

    // Horizon line
    ctx.beginPath();
    ctx.moveTo(-radius * 0.9, 0);
    ctx.lineTo(radius * 0.9, 0);
    ctx.stroke();

    for (let deg = -30; deg <= 30; deg += 10) {
      if (deg === 0) continue;
      const y = -deg * (radius / 35);
      const half = deg % 20 === 0 ? radius * 0.32 : radius * 0.18;
      ctx.beginPath();
      if (deg < 0) ctx.setLineDash([4, 4]);
      else ctx.setLineDash([]);
      ctx.moveTo(-half, y);
      ctx.lineTo(half, y);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillText(`${deg}`, -half - 4, y + 3);
    }
    ctx.restore();
    ctx.restore();

    // Fixed Center Aircraft Boresight
    ctx.save();
    ctx.strokeStyle = '#f59e0b';
    ctx.lineWidth = 2.2;
    ctx.beginPath();
    ctx.moveTo(cx - 26, cy);
    ctx.lineTo(cx - 8, cy);
    ctx.lineTo(cx, cy + 7);
    ctx.lineTo(cx + 8, cy);
    ctx.lineTo(cx + 26, cy);
    ctx.stroke();

    // Outer roll ring arc
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.35)';
    ctx.lineWidth = 1.4;
    ctx.beginPath();
    ctx.arc(cx, cy, radius, -Math.PI * 0.83, -Math.PI * 0.17);
    ctx.stroke();

    // Heading Compass Tape (top center)
    const hdg = ((d.yawDeg % 360) + 360) % 360;
    ctx.fillStyle = 'rgba(15, 23, 42, 0.72)';
    ctx.fillRect(cx - 60, cy - radius - 34, 120, 22);
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.5)';
    ctx.strokeRect(cx - 60, cy - radius - 34, 120, 22);
    ctx.fillStyle = '#e2e8f0';
    ctx.font = 'bold 12px monospace';
    ctx.textAlign = 'center';
    ctx.fillText(`HDG ${hdg.toFixed(1).padStart(5, '0')} deg`, cx, cy - radius - 19);

    // Slip/skid lateral G indicator (bottom of PFD)
    const slipOffset = Math.max(-28, Math.min(28, d.accelYG * 40));
    ctx.strokeStyle = 'rgba(148, 163, 184, 0.5)';
    ctx.strokeRect(cx - 34, cy + radius + 10, 68, 12);
    ctx.fillStyle = '#22c55e';
    ctx.beginPath();
    ctx.arc(cx + slipOffset, cy + radius + 16, 4.5, 0, Math.PI * 2);
    ctx.fill();

    // Right Altimeter & VSI box
    const altX = cx + radius + 14;
    ctx.fillStyle = 'rgba(15, 23, 42, 0.75)';
    ctx.fillRect(altX, cy - 22, 92, 44);
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.45)';
    ctx.strokeRect(altX, cy - 22, 92, 44);
    ctx.fillStyle = '#38bdf8';
    ctx.textAlign = 'left';
    ctx.font = 'bold 11px monospace';
    ctx.fillText(`ALT ${d.relAltM.toFixed(1)}m`, altX + 6, cy - 4);
    ctx.fillStyle = d.vsiMs >= 0 ? '#22c55e' : '#f59e0b';
    ctx.fillText(`VSI ${d.vsiMs >= 0 ? '+' : ''}${d.vsiMs.toFixed(1)}`, altX + 6, cy + 13);

    // Left G-load box
    const gX = cx - radius - 96;
    ctx.fillStyle = 'rgba(15, 23, 42, 0.75)';
    ctx.fillRect(gX, cy - 22, 82, 44);
    ctx.strokeStyle = 'rgba(245, 158, 11, 0.45)';
    ctx.strokeRect(gX, cy - 22, 82, 44);
    ctx.fillStyle = '#f59e0b';
    ctx.font = 'bold 11px monospace';
    ctx.fillText(`G ${d.gLoad.toFixed(2)}g`, gX + 6, cy - 4);
    ctx.fillStyle = '#e2e8f0';
    ctx.fillText(`R ${d.rollDeg.toFixed(0)}`, gX + 6, cy + 13);

    ctx.restore();
  }

  private drawSingleChart(ctx: CanvasRenderingContext2D, s: RingSeries, color: string, unit: string): void {
    const w = ctx.canvas.width;
    const h = ctx.canvas.height;
    ctx.clearRect(0, 0, w, h);

    ctx.fillStyle = 'rgba(2, 6, 23, 0.65)';
    ctx.fillRect(0, 0, w, h);

    const n = s.length();
    if (n < 2) return;

    let min = Infinity;
    let max = -Infinity;
    for (let i = 0; i < n; i++) {
      const v = s.get(i);
      if (v < min) min = v;
      if (v > max) max = v;
    }
    const pad = Math.max(0.5, (max - min) * 0.2);
    min -= pad;
    max += pad;
    const span = max - min;

    ctx.strokeStyle = color;
    ctx.lineWidth = 1.6;
    ctx.beginPath();
    for (let i = 0; i < n; i++) {
      const x = (i / (s.capacity - 1)) * w;
      const y = h - ((s.get(i) - min) / span) * (h - 14) - 7;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();

    ctx.fillStyle = '#94a3b8';
    ctx.font = '10px monospace';
    ctx.fillText(`${max.toFixed(1)}${unit}`, 4, 11);
    ctx.fillText(`${min.toFixed(1)}${unit}`, 4, h - 4);
  }

  private drawTripleChart(
    ctx: CanvasRenderingContext2D,
    s1: RingSeries,
    s2: RingSeries,
    s3: RingSeries,
    absLimit: number,
    colors: [string, string, string]
  ): void {
    const w = ctx.canvas.width;
    const h = ctx.canvas.height;
    ctx.clearRect(0, 0, w, h);

    ctx.fillStyle = 'rgba(2, 6, 23, 0.65)';
    ctx.fillRect(0, 0, w, h);

    // Zero center line
    ctx.strokeStyle = 'rgba(148, 163, 184, 0.2)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, h * 0.5);
    ctx.lineTo(w, h * 0.5);
    ctx.stroke();

    const series = [s1, s2, s3];
    for (let k = 0; k < 3; k++) {
      const s = series[k];
      const n = s.length();
      if (n < 2) continue;

      ctx.strokeStyle = colors[k];
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      for (let i = 0; i < n; i++) {
        const x = (i / (s.capacity - 1)) * w;
        const norm = Math.max(-1, Math.min(1, s.get(i) / absLimit));
        const y = h * 0.5 - norm * (h * 0.44);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }
  }
}
