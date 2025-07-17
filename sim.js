// Parametry fizyczne drona
const DRONE_MASS = 1.2; // kg
const DRONE_WIDTH = 80; // px
const DRONE_HEIGHT = 20; // px
const DRONE_MOMENT = 0.08; // kg*m^2 (przybliżenie)
const GRAVITY = 9.81; // m/s^2
const PIXELS_PER_METER = 80;
const GROUND_Y = 450; // px (pozycja podłoża na canvasie)

// --- LOGI ---
const LOG_ENABLED = true;

// --- ZADANE WARTOŚCI (tymczasowo, przed inicjalizacją PID) ---
let targetAltitude = 200; // px
let targetVy = 0; // px/s

// --- DOMYŚLNE NASTAWY PID (przed użyciem) ---
let pidParams = {
    rate: { kp: 4.0, ki: 0.0, kd: 0.0 },
    att:  { kp: 4.0, ki: 0.0, kd: 0.0 },
    alt:  { kp: 0.1, ki: 0.0, kd: 0.0 },
    vrate:{ kp: 0.1, ki: 0.0, kd: 0.0 }
};

let maxVy = 50; // px/s (domyślnie)
let maxRollRate = 400 * Math.PI / 180; // rad/s (domyślnie 400°/s)
// --- TYMCZASOWE PID (zostaną zainicjalizowane po zdefiniowaniu klasy PID) ---
let altPid = null;
let vratePid = null;

// --- PANEL LOGÓW ---
const logPanel = document.getElementById('log-panel');
const showLogCheckbox = document.getElementById('show-log-checkbox');
const autoscrollCheckbox = document.getElementById('autoscroll-checkbox');
const logTextarea = document.getElementById('log-textarea');
const copyLogBtn = document.getElementById('copy-log-btn');
const clearLogBtn = document.getElementById('clear-log-btn');
const saveLogBtn = document.getElementById('save-log-btn');

showLogCheckbox.addEventListener('change', () => {
    logPanel.style.display = showLogCheckbox.checked ? '' : 'none';
});
copyLogBtn.addEventListener('click', () => {
    logTextarea.select();
    document.execCommand('copy');
});
clearLogBtn.addEventListener('click', () => {
    logBuffer = [];
    logTextarea.value = '';
});
saveLogBtn.addEventListener('click', () => {
    const now = new Date();
    const pad = n => n.toString().padStart(2, '0');
    const fname = `log_${pad(now.getFullYear()%100)}${pad(now.getMonth()+1)}${pad(now.getDate())}_${pad(now.getHours())}${pad(now.getMinutes())}.txt`;
    const blob = new Blob([logTextarea.value], {type: 'text/plain'});
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = fname;
    document.body.appendChild(a);
    a.click();
    setTimeout(() => { document.body.removeChild(a); }, 100);
});

// --- KONTROLA SYMULACJI ---
const pauseBtn = document.getElementById('pause-btn');
const resetBtn = document.getElementById('reset-btn');
const thrustVectorsCheckbox = document.getElementById('thrust-vectors-checkbox');
let isPaused = false;
let showThrustVectors = false;
// --- USTAWIENIE DOMYŚLNE WEKTORÓW CIĄGU ---
showThrustVectors = thrustVectorsCheckbox.checked;

pauseBtn.addEventListener('click', () => {
    isPaused = !isPaused;
    pauseBtn.textContent = isPaused ? 'Resume' : 'Pause';
    if (!isPaused) requestAnimationFrame(loop);
});
resetBtn.addEventListener('click', () => {
    drone.reset();
    joyX = 0;
    joyY = 1;
    drawJoystick();
    // Reset PID
    altPid.reset();
    vratePid.reset();
    targetAltitude = drone.y;
    targetVy = 0;
});
thrustVectorsCheckbox.addEventListener('change', e => {
    showThrustVectors = thrustVectorsCheckbox.checked;
});

// --- LOGOWANIE ---
let logBuffer = [];
function logMessage(msg) {
    if (!LOG_ENABLED) return;
    const now = new Date();
    const time = now.toLocaleTimeString('pl-PL', { hour12: false }) + '.' + now.getMilliseconds().toString().padStart(3, '0');
    logBuffer.push(`[${time}] ${msg}`);
    if (logBuffer.length > 5000) logBuffer = logBuffer.slice(-5000); // max 5000 linii
    logTextarea.value = logBuffer.join('\n');
    if (autoscrollCheckbox.checked) {
        logTextarea.scrollTop = logTextarea.scrollHeight;
    }
}

// --- LOGOWANIE STANU DRONA ---
let lastLogTime = 0;
function logDroneState(ts) {
    if (!LOG_ENABLED) return;
    if (ts - lastLogTime < 33) return; // log co ~33 ms (30 Hz)
    lastLogTime = ts;
    const altitude = canvas.height - drone.y;
    let state = `x=${drone.x.toFixed(1)} y=${drone.y.toFixed(1)} alt=${altitude.toFixed(1)} vx=${drone.vx.toFixed(2)} vy=${drone.vy.toFixed(2)} roll=${(drone.angle*180/Math.PI).toFixed(1)}° rollRate=${drone.omega.toFixed(2)} left=${drone.leftThrust.toFixed(2)} right=${drone.rightThrust.toFixed(2)} joyX=${joyX.toFixed(2)} joyY=${joyY.toFixed(2)}`;
    if (altitudePidCheckbox && altitudePidCheckbox.checked && altPid && vratePid) {
        state += ` targetAlt=${targetAltitude.toFixed(1)} targetVy=${targetVy.toFixed(1)}`;
    }
    if (ratePidCheckbox && ratePidCheckbox.checked && ratePid) {
        state += ` targetRollRate=${targetRollRate.toFixed(2)}`;
    }
    if (attitudePidCheckbox && attitudePidCheckbox.checked && attPid) {
        state += ` targetRoll=${(targetRoll*180/Math.PI).toFixed(1)}°`;
    }
    logMessage(state);
}

class Drone {
    constructor() {
        this.reset();
    }

    reset() {
        // Stan translacyjny
        this.x = 400; // px (środek canvasu)
        this.y = 200; // px
        this.vx = 0;
        this.vy = 0;
        this.ax = 0;
        this.ay = 0;
        // Stan rotacyjny
        this.angle = 0; // rad
        this.omega = 0; // prędkość kątowa
        this.alpha = 0; // przyspieszenie kątowe
        // Siły i momenty
        this.leftThrust = 0; // 0-1
        this.rightThrust = 0; // 0-1
    }

    setThrust(left, right) {
        this.leftThrust = Math.max(0, Math.min(1, left));
        this.rightThrust = Math.max(0, Math.min(1, right));
    }

    update(dt) {
        // --- SIŁY ---
        const maxThrust = DRONE_MASS * GRAVITY * 2;
        const leftF = this.leftThrust * maxThrust;
        const rightF = this.rightThrust * maxThrust;
        const arm = (DRONE_WIDTH / 2) / PIXELS_PER_METER;
        // Siły od silników w układzie świata
        const leftFx = Math.sin(this.angle) * leftF;
        const leftFy = -Math.cos(this.angle) * leftF;
        const rightFx = Math.sin(this.angle) * rightF;
        const rightFy = -Math.cos(this.angle) * rightF;
        let totalFx = leftFx + rightFx;
        let totalFy = leftFy + rightFy + DRONE_MASS * GRAVITY;
        // --- POPRAWKA: moment od thrustów (fizycznie poprawny znak) ---
        let totalMoment = arm * (leftF - rightF);
        // --- KOLIZJE Z PODŁOŻEM ---
        const hw = DRONE_WIDTH / 2;
        const hh = DRONE_HEIGHT / 2;
        const corners = [
            {x: -hw, y: -hh},
            {x: hw, y: -hh},
            {x: hw, y: hh},
            {x: -hw, y: hh}
        ].map(c => {
            const rx = Math.cos(this.angle) * c.x - Math.sin(this.angle) * c.y;
            const ry = Math.sin(this.angle) * c.x + Math.cos(this.angle) * c.y;
            return {x: this.x + rx, y: this.y + ry, rx, ry, local: c};
        });
        let groundContacts = [];
        for (const corner of corners) {
            if (corner.y > GROUND_Y) {
                groundContacts.push(corner);
            }
        }
        if (groundContacts.length === 1) {
            // --- REALISTYCZNY PIVOT: jeden róg na ziemi ---
            const pivot = groundContacts[0];
            // Pozycja środka masy względem pivotu (metry)
            const dx = (this.x - pivot.x) / PIXELS_PER_METER;
            const dy = (this.y - pivot.y) / PIXELS_PER_METER;
            // Moment od grawitacji względem pivotu
            const gravityMoment = dx * DRONE_MASS * GRAVITY;
            totalMoment += gravityMoment;
            // Ogranicz translację: pivot nie może wejść pod ziemię
            const dy_pen = pivot.y - GROUND_Y;
            if (dy_pen > 0) {
                this.y -= dy_pen;
                // Wyzeruj prędkość pionową pivotu (przybliżenie)
                this.vy = 0;
            }
        } else if (groundContacts.length >= 2) {
            // --- Dwa lub więcej rogów na ziemi: leżenie płasko ---
            // Przesuń drona w górę, aż najniższy róg będzie na powierzchni
            let maxDy = 0;
            for (const c of groundContacts) {
                const dy = c.y - GROUND_Y;
                if (dy > maxDy) maxDy = dy;
            }
            if (maxDy > 0) this.y -= maxDy;
            // Mocno tłum prędkości
            this.vx *= 0.3;
            this.vy = 0;
            this.omega *= 0.3;
        }
        // --- OPÓR POWIETRZA (tłumienie) ---
        const LINEAR_DAMPING = 0.3;
        const ANGULAR_DAMPING = 2.0;
        // --- RÓWNANIA RUCHU ---
        this.ax = totalFx / DRONE_MASS;
        this.ay = totalFy / DRONE_MASS;
        this.alpha = totalMoment / DRONE_MOMENT;
        // Aktualizacja prędkości i pozycji
        this.vx += this.ax * dt;
        this.vy += this.ay * dt;
        this.omega += this.alpha * dt;
        // Tłumienie
        this.vx -= this.vx * LINEAR_DAMPING * dt;
        this.vy -= this.vy * LINEAR_DAMPING * dt;
        this.omega -= this.omega * ANGULAR_DAMPING * dt;
        this.x += this.vx * dt * PIXELS_PER_METER;
        this.y += this.vy * dt * PIXELS_PER_METER;
        this.angle += this.omega * dt;
        // Ogranicz kąt do -PI..PI
        if (this.angle > Math.PI) this.angle -= 2 * Math.PI;
        if (this.angle < -Math.PI) this.angle += 2 * Math.PI;
    }

    draw(ctx, showThrustVectors = false) {
        const motor_height = -6; // wysokość wirników ponad korpusem (px)
        ctx.save();
        ctx.translate(this.x, this.y);
        ctx.rotate(this.angle);
        // Korpus
        ctx.fillStyle = '#4af';
        ctx.fillRect(-DRONE_WIDTH/2, -DRONE_HEIGHT/2, DRONE_WIDTH, DRONE_HEIGHT);
        // Wirniki (regulowana wysokość)
        ctx.fillStyle = '#fff';
        ctx.beginPath();
        ctx.arc(-DRONE_WIDTH/2, -DRONE_HEIGHT/2 - motor_height, 10, 0, 2*Math.PI); // lewy
        ctx.arc(DRONE_WIDTH/2, -DRONE_HEIGHT/2 - motor_height, 10, 0, 2*Math.PI); // prawy
        ctx.fill();
        // Wektory ciągu
        if (showThrustVectors) {
            const maxThrust = DRONE_MASS * GRAVITY * 2;
            ctx.save();
            ctx.globalAlpha = 1.0;
            // Lewy
            ctx.strokeStyle = '#f00';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(-DRONE_WIDTH/2, 0);
            ctx.lineTo(-DRONE_WIDTH/2, 0 - this.leftThrust * maxThrust * 2.0);
            ctx.stroke();
            // Prawy
            ctx.strokeStyle = '#f00';
            ctx.beginPath();
            ctx.moveTo(DRONE_WIDTH/2, 0);
            ctx.lineTo(DRONE_WIDTH/2, 0 - this.rightThrust * maxThrust * 2.0);
            ctx.stroke();
            ctx.restore();
        }
        // Płomienie (ciąg)
        ctx.globalAlpha = 0.7;
        ctx.fillStyle = '#ff0';
        if (this.leftThrust > 0.01) {
            ctx.beginPath();
            ctx.moveTo(-DRONE_WIDTH/2, 0);
            ctx.lineTo(-DRONE_WIDTH/2 - 8, 0 - 8 * this.leftThrust);
            ctx.lineTo(-DRONE_WIDTH/2 - 8, 0 + 8 * this.leftThrust);
            ctx.closePath();
            ctx.fill();
        }
        if (this.rightThrust > 0.01) {
            ctx.beginPath();
            ctx.moveTo(DRONE_WIDTH/2, 0);
            ctx.lineTo(DRONE_WIDTH/2 + 8, 0 - 8 * this.rightThrust);
            ctx.lineTo(DRONE_WIDTH/2 + 8, 0 + 8 * this.rightThrust);
            ctx.closePath();
            ctx.fill();
        }
        ctx.globalAlpha = 1.0;
        ctx.restore();
    }
}

// --- Inicjalizacja ---
const canvas = document.getElementById('sim-canvas');
const ctx = canvas.getContext('2d');
const drone = new Drone();
targetAltitude = drone.y;

// --- Joystick ---
const joystickContainer = document.getElementById('joystick-container');
const joySize = 160;
const joyRadius = joySize / 2;
let joyX = 0, joyY = 1; // joyY=1: throttle minimalny (dron nie odlatuje)
let dragging = false;

function drawJoystick() {
    const c = joystickContainer;
    let joyCanvas = c.querySelector('canvas');
    if (!joyCanvas) {
        joyCanvas = document.createElement('canvas');
        joyCanvas.width = joySize;
        joyCanvas.height = joySize;
        c.appendChild(joyCanvas);
    }
    const jctx = joyCanvas.getContext('2d');
    jctx.clearRect(0, 0, joySize, joySize);
    // Tło
    jctx.beginPath();
    jctx.arc(joyRadius, joyRadius, joyRadius-2, 0, 2*Math.PI);
    jctx.fillStyle = dragging ? '#333' : '#222';
    jctx.fill();
    // Krzyż
    jctx.strokeStyle = '#555';
    jctx.lineWidth = 2;
    jctx.beginPath();
    jctx.moveTo(joyRadius, 20); jctx.lineTo(joyRadius, joySize-20);
    jctx.moveTo(20, joyRadius); jctx.lineTo(joySize-20, joyRadius);
    jctx.stroke();
    // Gałka
    const knobX = joyRadius + joyX * (joyRadius-28);
    const knobY = joyRadius + joyY * (joyRadius-28);
    jctx.beginPath();
    jctx.arc(knobX, knobY, 24, 0, 2*Math.PI);
    jctx.fillStyle = dragging ? '#4af' : '#888';
    jctx.shadowColor = '#000';
    jctx.shadowBlur = 8;
    jctx.fill();
    jctx.shadowBlur = 0;
    // Opis
    jctx.font = '14px sans-serif';
    jctx.fillStyle = '#fff';
}

drawJoystick();

let lastLoggedJoyX = null;
let lastLoggedJoyY = null;

function setJoystickFromEvent(e) {
    const rect = joystickContainer.getBoundingClientRect();
    const x = (e.touches ? e.touches[0].clientX : e.clientX) - rect.left;
    const y = (e.touches ? e.touches[0].clientY : e.clientY) - rect.top;
    let dx = (x - joyRadius) / (joyRadius-28);
    let dy = (y - joyRadius) / (joyRadius-28);
    dx = Math.max(-1, Math.min(1, dx));
    dy = Math.max(-1, Math.min(1, dy));
    // Loguj tylko jeśli pozycja się zmieniła (z dokładnością do 0.01)
    if (LOG_ENABLED && (lastLoggedJoyX === null || Math.abs(dx - lastLoggedJoyX) > 0.01 || Math.abs(dy - lastLoggedJoyY) > 0.01)) {
        logMessage(`Joystick changed: joyX=${dx.toFixed(2)} joyY=${dy.toFixed(2)}`);
        lastLoggedJoyX = dx;
        lastLoggedJoyY = dy;
    }
    joyX = dx;
    joyY = dy;
    drawJoystick();
}

joystickContainer.addEventListener('mousedown', e => {
    dragging = true;
    setJoystickFromEvent(e);
});
window.addEventListener('mousemove', e => {
    if (dragging) setJoystickFromEvent(e);
});
window.addEventListener('mouseup', e => {
    if (dragging) {
        dragging = false;
        joyX = 0;
        joyY = 1; // throttle minimalny
        drawJoystick();
    }
});
// Touch
joystickContainer.addEventListener('touchstart', e => {
    dragging = true;
    setJoystickFromEvent(e);
    e.preventDefault();
});
joystickContainer.addEventListener('touchmove', e => {
    if (dragging) setJoystickFromEvent(e);
    e.preventDefault();
});
joystickContainer.addEventListener('touchend', e => {
    dragging = false;
    joyX = 0;
    joyY = 1; // throttle minimalny
    drawJoystick();
    e.preventDefault();
});

// --- KLASA PID ---
class PID {
    constructor(kp, ki, kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.integral = 0;
        this.lastError = 0;
        this.lastTime = null;
    }
    
    update(setpoint, feedback) {
        const now = performance.now();
        if (this.lastTime === null) {
            this.lastTime = now;
            return 0;
        }
        
        const dt = (now - this.lastTime) / 1000;
        if (dt <= 0 || !isFinite(dt)) return 0;
        this.lastTime = now;
        
        const error = setpoint - feedback;
        this.integral += error * dt;
        const derivative = (error - this.lastError) / dt;
        this.lastError = error;
        
        return this.kp * error + this.ki * this.integral + this.kd * derivative;
    }
    
    reset() {
        this.integral = 0;
        this.lastError = 0;
        this.lastTime = null;
    }
}

// --- INICJALIZACJA PID ---
altPid = new PID(pidParams.alt.kp, pidParams.alt.ki, pidParams.alt.kd);
vratePid = new PID(pidParams.vrate.kp, pidParams.vrate.ki, pidParams.vrate.kd);
let ratePid = new PID(pidParams.rate.kp, pidParams.rate.ki, pidParams.rate.kd);
let attPid = new PID(pidParams.att.kp, pidParams.att.ki, pidParams.att.kd);

let targetRollRate = 0; // rad/s
let targetRoll = 0; // rad

function updateRollControl(joyX) {
    // --- Kaskadowe sterowanie roll ---
    if (!ratePidCheckbox.checked) {
        // JoyX steruje thrustami jak dotychczas
        return null;
    }
    // Ograniczenia
    // const maxRollRate = 50 * Math.PI / 180; // 50°/s
    const maxRoll = 45 * Math.PI / 180; // 45°
    if (attitudePidCheckbox.checked) {
        // JoyX steruje kątem roll
        targetRoll = joyX * maxRoll;
        // Attitude PID: targetRoll -> targetRollRate
        targetRollRate = attPid.update(targetRoll, drone.angle);
        targetRollRate = Math.max(-maxRollRate, Math.min(maxRollRate, targetRollRate));
    } else {
        // JoyX steruje prędkością kątową roll
        targetRollRate = joyX * maxRollRate;
    }
    // Rate PID: targetRollRate -> wyjście (różnica thrustów)
    const rollRateCmd = ratePid.update(targetRollRate, drone.omega);
    // Ogranicz wyjście do [-1, 1]
    return Math.max(-1, Math.min(1, rollRateCmd));
}

// --- AKTUALIZACJA PARAMETRÓW PID ---
function updatePidParams() {
    altPid = new PID(pidParams.alt.kp, pidParams.alt.ki, pidParams.alt.kd);
    vratePid = new PID(pidParams.vrate.kp, pidParams.vrate.ki, pidParams.vrate.kd);
    ratePid = new PID(pidParams.rate.kp, pidParams.rate.ki, pidParams.rate.kd);
    attPid = new PID(pidParams.att.kp, pidParams.att.ki, pidParams.att.kd);
}

// --- W PĘTLI STEROWANIA ---
let lastUpdateTime = performance.now();

function updateAltitudeControl(joyY) {
    if (!altitudePidCheckbox.checked) {
        return (1 - joyY) / 2;
    } else {
        // joyY: -1 (góra) do 1 (dół), neutralny = 1
        // Setpoint: joyY=-1 -> top (canvas.height), joyY=1 -> bottom (0)
        const setpoint = ((-joyY + 1) / 2) * canvas.height;
        // Feedback: wysokość drona w tej samej skali (0=dół, canvas.height=góra)
        const altitude = canvas.height - drone.y;
        targetAltitude = setpoint;
        targetVy = altPid.update(targetAltitude, altitude);
        targetVy = Math.max(-maxVy, Math.min(maxVy, targetVy));
        const throttle = vratePid.update(targetVy, -drone.vy); // -drone.vy bo y rośnie w dół
        return Math.max(0, Math.min(1, throttle));
    }
}

function joystickToThrustsFull(x, y) {
    // Altitude PID
    let throttle;
    if (altitudePidCheckbox.checked) {
        throttle = updateAltitudeControl(y);
    } else {
        throttle = (1 - y) / 2;
    }
    // Roll PID
    let diff;
    const rollRateCmd = updateRollControl(x);
    if (rollRateCmd === null) {
        // Brak PID, joyX steruje thrustami jak dotychczas
        diff = x * 0.7;
    } else {
        // PID, wyjście steruje różnicą thrustów (skaluj do 0.7)
        diff = rollRateCmd * 0.7;
    }
    let left = throttle + diff/2;
    let right = throttle - diff/2;
    left = Math.max(0, Math.min(1, left));
    right = Math.max(0, Math.min(1, right));
    return [left, right];
}

function drawGround() {
    ctx.fillStyle = '#393';
    ctx.fillRect(0, GROUND_Y, canvas.width, canvas.height - GROUND_Y);
}

function draw() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawGround();
    drone.draw(ctx, showThrustVectors);
}

// --- W PĘTLI ---
let lastTime = null;
function loop(ts) {
    if (isPaused) return;
    if (!lastTime) lastTime = ts;
    const dt = Math.min((ts - lastTime) / 1000, 0.04); // max 25 FPS step
    lastTime = ts;
    // --- joystick steruje thrustami ---
    const [left, right] = joystickToThrustsFull(joyX, joyY);
    drone.setThrust(left, right);
    drone.update(dt);
    draw();
    logDroneState(ts);
    requestAnimationFrame(loop);
}

// Uruchom pętlę od razu jeśli nie jest zapauzowane
if (!isPaused) requestAnimationFrame(loop);

// --- OBSŁUGA PANELU PID I CHECKBOXÓW ---
const ratePidCheckbox = document.getElementById('rate-pid-checkbox');
const attitudePidCheckbox = document.getElementById('attitude-pid-checkbox');
const attitudePidLabel = document.getElementById('attitude-pid-label');
const altitudePidCheckbox = document.getElementById('altitude-pid-checkbox');
const pidSettingsBtn = document.getElementById('pid-settings-btn');
const pidSettingsPanel = document.getElementById('pid-settings-panel');
const closePidSettingsBtn = document.getElementById('close-pid-settings-btn');

// Przypisz domyślne wartości do pól
function setPidInputs() {
    document.getElementById('rate-kp').value = pidParams.rate.kp;
    document.getElementById('rate-ki').value = pidParams.rate.ki;
    document.getElementById('rate-kd').value = pidParams.rate.kd;
    document.getElementById('att-kp').value = pidParams.att.kp;
    document.getElementById('att-ki').value = pidParams.att.ki;
    document.getElementById('att-kd').value = pidParams.att.kd;
    document.getElementById('alt-kp').value = pidParams.alt.kp;
    document.getElementById('alt-ki').value = pidParams.alt.ki;
    document.getElementById('alt-kd').value = pidParams.alt.kd;
    document.getElementById('vrate-kp').value = pidParams.vrate.kp;
    document.getElementById('vrate-ki').value = pidParams.vrate.ki;
    document.getElementById('vrate-kd').value = pidParams.vrate.kd;
    document.getElementById('max-vy').value = maxVy;
    document.getElementById('max-rollrate').value = Math.round(maxRollRate * 180 / Math.PI);
}
setPidInputs();

// Aktualizuj nastawy po zmianie
['rate','att','alt','vrate'].forEach(type => {
    ['kp','ki','kd'].forEach(param => {
        document.getElementById(`${type}-${param}`).addEventListener('change', e => {
            pidParams[type][param] = parseFloat(e.target.value);
            updatePidParams(); // Aktualizuj PID po zmianie nastaw
        });
    });
});

document.getElementById('max-vy').addEventListener('change', e => {
    maxVy = parseFloat(e.target.value);
});
document.getElementById('max-rollrate').addEventListener('change', e => {
    maxRollRate = parseFloat(e.target.value) * Math.PI / 180;
});

// Otwieranie/zamykanie panelu
pidSettingsBtn.addEventListener('click', () => {
    pidSettingsPanel.style.display = pidSettingsPanel.style.display === 'none' ? 'block' : 'none';
});
closePidSettingsBtn.addEventListener('click', () => {
    pidSettingsPanel.style.display = 'none';
});

// Attitude PID wyszarzony gdy rate PID wyłączony
ratePidCheckbox.addEventListener('change', () => {
    if (ratePidCheckbox.checked) {
        attitudePidCheckbox.disabled = false;
        attitudePidLabel.style.opacity = 1.0;
    } else {
        attitudePidCheckbox.disabled = true;
        attitudePidCheckbox.checked = false;
        attitudePidLabel.style.opacity = 0.5;
    }
});
// Inicjalizacja stanu
if (!ratePidCheckbox.checked) {
    attitudePidCheckbox.disabled = true;
    attitudePidLabel.style.opacity = 0.5;
} 

// --- Po inicjalizacji DOM ---
ratePidCheckbox.checked = false;
attitudePidCheckbox.checked = false;
altitudePidCheckbox.checked = false;
attitudePidCheckbox.disabled = true;
attitudePidLabel.style.opacity = 0.5; 