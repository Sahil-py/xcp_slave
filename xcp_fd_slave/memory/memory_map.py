"""
Simulated ECU memory — derived entirely from xcp_fd_slave/a2l/sample.a2l.

Key constraints from MOD_COMMON:
  BYTE_ORDER MSB_FIRST   → big-endian (Motorola) for all multi-byte values
  ALIGNMENT_BYTE    1
  ALIGNMENT_WORD    2
  ALIGNMENT_LONG    4
  ALIGNMENT_FLOAT32_IEEE 4

Measurement variables are ULONG / SLONG (32-bit integer, big-endian).
Calibration characteristics use FLOAT32_SCALAR → FLOAT32_IEEE (big-endian).

XCP COMM_MODE_BASIC bit 7 must be set to 1 (Motorola) in the CONNECT response.
"""
from __future__ import annotations
import math
import struct
import threading

# ---------------------------------------------------------------------------
# Measurement addresses  (ULONG / SLONG, 4 bytes each, big-endian)
# ---------------------------------------------------------------------------

# 0x1000 block — engine performance
ADDR_ENGINE_SPEED       = 0x00001000   # ULONG  rpm            0–8000
ADDR_ENGINE_LOAD        = 0x00001004   # ULONG  %              0–100
ADDR_INJECTION_TIME     = 0x00001008   # ULONG  µs             0–25000
ADDR_IGNITION_ANGLE     = 0x0000100C   # SLONG  0.1 deg BTDC   -100–550
ADDR_THROTTLE_POS       = 0x00001010   # ULONG  %              0–100
ADDR_ENGINE_TORQUE      = 0x00001014   # ULONG  0.1 Nm         0–4500
ADDR_MASS_AIR_FLOW      = 0x00001018   # ULONG  0.01 g/s       0–60000
ADDR_MANIFOLD_PRESSURE  = 0x0000101C   # ULONG  0.1 kPa        200–3000

# 0x2000 block — temperatures
ADDR_COOLANT_TEMP       = 0x00002000   # SLONG  0.1 °C         -400–1300
ADDR_OIL_TEMP           = 0x00002004   # SLONG  0.1 °C         -400–1600
ADDR_INTAKE_AIR_TEMP    = 0x00002008   # SLONG  0.1 °C         -400–800
ADDR_EXHAUST_GAS_TEMP   = 0x0000200C   # ULONG  0.1 °C         0–9000
ADDR_TRANSMISSION_TEMP  = 0x00002010   # SLONG  0.1 °C         -400–1400

# 0x3000 block — dynamics
ADDR_VEHICLE_SPEED      = 0x00003000   # ULONG  0.1 km/h       0–2800
ADDR_WHEEL_SPEED_FL     = 0x00003004   # ULONG  0.1 km/h       0–2800
ADDR_WHEEL_SPEED_FR     = 0x00003008   # ULONG  0.1 km/h       0–2800
ADDR_WHEEL_SPEED_RL     = 0x0000300C   # ULONG  0.1 km/h       0–2800
ADDR_WHEEL_SPEED_RR     = 0x00003010   # ULONG  0.1 km/h       0–2800
ADDR_LONG_ACCELERATION  = 0x00003014   # SLONG  0.01 m/s²      -2000–2000
ADDR_LAT_ACCELERATION   = 0x00003018   # SLONG  0.01 m/s²      -1500–1500
ADDR_YAW_RATE           = 0x0000301C   # SLONG  0.1 deg/s      -1500–1500

# 0x4000 block — fuel / exhaust
ADDR_FUEL_RAIL_PRESSURE = 0x00004000   # ULONG  0.1 bar        0–2200
ADDR_LAMBDA_SENSOR_1    = 0x00004004   # ULONG  0.001 lambda   500–2000
ADDR_LAMBDA_SENSOR_2    = 0x00004008   # ULONG  0.001 lambda   500–1500
ADDR_FUEL_TANK_LEVEL    = 0x0000400C   # ULONG  %              0–100
ADDR_FUEL_CONSUMPTION   = 0x00004010   # ULONG  0.1 l/100km    0–500

# 0x5000 block — electrical
ADDR_BATTERY_VOLTAGE    = 0x00005000   # ULONG  0.01 V         800–1600
ADDR_BATTERY_CURRENT    = 0x00005004   # SLONG  0.1 A          -2500–2500
ADDR_ALTERNATOR_VOLTAGE = 0x00005008   # ULONG  0.01 V         1100–1550
ADDR_ECU_SUPPLY_VOLTAGE = 0x0000500C   # ULONG  0.001 V        4500–5500

# ---------------------------------------------------------------------------
# Calibration addresses  (FLOAT32_IEEE, 4 bytes each, big-endian)
# ---------------------------------------------------------------------------

ADDR_MAX_ENGINE_SPEED           = 0x00008000   # float32  rpm       5000–9000
ADDR_REV_LIMITER_CUT_IN         = 0x00008004   # float32  rpm       4500–8500
ADDR_IDLE_TARGET_SPEED          = 0x00008008   # float32  rpm       600–950
ADDR_IDLE_TARGET_SPEED_COLD     = 0x0000800C   # float32  rpm       900–1500
ADDR_TORQUE_LIMIT               = 0x00008010   # float32  Nm        0–500
ADDR_IGNITION_ADVANCE_BASE      = 0x00008014   # float32  deg BTDC  0–45
ADDR_INJECTOR_FLOW_RATE         = 0x00008018   # float32  cc/min    200–800
ADDR_INJECTOR_DEAD_TIME         = 0x0000801C   # float32  ms        0–2
ADDR_TARGET_LAMBDA_IDLE         = 0x00008020   # float32  lambda    0.95–1.05
ADDR_TARGET_LAMBDA_WOT          = 0x00008024   # float32  lambda    0.80–0.98
ADDR_FUEL_PUMP_DUTY             = 0x00008028   # float32  %         50–100
ADDR_CRANKING_FUEL_ENRICHMENT   = 0x0000802C   # float32  %         100–250
ADDR_FAN_ON_TEMP                = 0x00008030   # float32  °C        85–105
ADDR_FAN_OFF_TEMP               = 0x00008034   # float32  °C        75–98
ADDR_OIL_TEMP_WARNING           = 0x00008038   # float32  °C        120–150
ADDR_COOLANT_TEMP_WARNING       = 0x0000803C   # float32  °C        105–130
ADDR_COLD_START_TEMP_THRESHOLD  = 0x00008040   # float32  °C        10–40
ADDR_BATTERY_LOW_VOLTAGE        = 0x00008044   # float32  V         10–12
ADDR_ALTERNATOR_TARGET_VOLTAGE  = 0x00008048   # float32  V         13–15
ADDR_MAX_FUEL_RAIL_PRESSURE     = 0x0000804C   # float32  bar       100–220

# ---------------------------------------------------------------------------
# Internal slave area (not in A2L)
# ---------------------------------------------------------------------------

ADDR_ID_STRING_AREA = 0x0000FF00   # 256-byte scratch buffer for GET_ID

# 64 KiB covers the highest A2L address (0x804F) and the ID area (0xFF00)
_MEM_SIZE = 0x10000


class MemoryMap:
    """
    64 KiB flat ECU memory, big-endian (MSB_FIRST per A2L MOD_COMMON).

    Measurements stored as big-endian int32 / uint32.
    Characteristics stored as big-endian IEEE 754 float32.
    """

    def __init__(self) -> None:
        self._data = bytearray(_MEM_SIZE)
        self._lock = threading.Lock()
        self._init_defaults()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def read(self, address: int, size: int) -> bytes:
        if size == 0:
            return b""
        self._check_range(address, size)
        with self._lock:
            return bytes(self._data[address: address + size])

    def write(self, address: int, data: bytes) -> None:
        if not data:
            return
        self._check_range(address, len(data))
        with self._lock:
            self._data[address: address + len(data)] = data

    def store_id_string(self, text: str) -> int:
        """Write ASCII text to the ID scratch area; return its start address."""
        encoded = text.encode("ascii")[:256]
        with self._lock:
            self._data[ADDR_ID_STRING_AREA: ADDR_ID_STRING_AREA + len(encoded)] = encoded
        return ADDR_ID_STRING_AREA

    # ------------------------------------------------------------------
    # Simulation — all values scaled per A2L PHYS_UNIT / resolution
    # ------------------------------------------------------------------

    def simulate_tick(self, t: float) -> None:
        """
        Update all simulated variables as a function of elapsed time t (seconds).

        Units are the raw ECU integer values (matching A2L scaling):
          ULONG/SLONG measurements: raw integer in PHYS_UNIT with resolution 1
          FLOAT32 characteristics:  not updated (write-only from master)
        """
        with self._lock:
            # --- engine speed: idle + light load cycle ---
            rpm = int(800 + 600 * (1 + math.sin(t * 0.30)))          # 800–2000 rpm
            rpm = max(0, min(8000, rpm))
            self._wu32(ADDR_ENGINE_SPEED, rpm)

            # --- engine load follows rpm ---
            load = max(10, min(100, int((rpm - 600) / 14)))           # ~14–100 %
            self._wu32(ADDR_ENGINE_LOAD, load)

            # --- throttle position (% opening, 2–80) ---
            throttle = max(2, min(100, int(load * 0.75)))
            self._wu32(ADDR_THROTTLE_POS, throttle)

            # --- injection pulse width: base + load term ---
            inj = max(500, min(25000, int(1800 + load * 70)))         # µs
            self._wu32(ADDR_INJECTION_TIME, inj)

            # --- ignition advance: base + load advance ---
            ign = max(-100, min(550, int(80 + load * 2)))             # 0.1 deg BTDC
            self._ws32(ADDR_IGNITION_ANGLE, ign)

            # --- engine torque (0.1 Nm) ---
            torque = max(0, min(4500, int(load * 35)))
            self._wu32(ADDR_ENGINE_TORQUE, torque)

            # --- mass air flow (0.01 g/s): rpm + load driven ---
            maf = max(0, min(60000, int(150 + rpm * 0.04 + load * 8)))
            self._wu32(ADDR_MASS_AIR_FLOW, maf)

            # --- manifold pressure (0.1 kPa): vacuum at low load ---
            map_p = max(200, min(3000, int(300 + load * 7)))
            self._wu32(ADDR_MANIFOLD_PRESSURE, map_p)

            # --- coolant temperature warming asymptotically to 90 °C ---
            coolant = int(200 + 700 * (1.0 - math.exp(-t * 0.015)))  # 0.1 °C
            coolant = max(-400, min(1300, coolant))
            self._ws32(ADDR_COOLANT_TEMP, coolant)

            # --- oil temperature (slower warm-up to ~100 °C) ---
            oil_t = int(200 + 800 * (1.0 - math.exp(-t * 0.008)))
            oil_t = max(-400, min(1600, oil_t))
            self._ws32(ADDR_OIL_TEMP, oil_t)

            # --- intake air temperature: ambient + slight heat soak ---
            iat = max(-400, min(800, int(200 + 80 * (1.0 - math.exp(-t * 0.02)))))
            self._ws32(ADDR_INTAKE_AIR_TEMP, iat)

            # --- exhaust gas temperature (0.1 °C): fast response to load ---
            egt = max(0, min(9000, int(2500 + load * 12 + rpm * 0.2)))
            self._wu32(ADDR_EXHAUST_GAS_TEMP, egt)

            # --- transmission temperature (very slow warm-up to ~70 °C) ---
            trans_t = max(-400, min(1400, int(200 + 500 * (1.0 - math.exp(-t * 0.005)))))
            self._ws32(ADDR_TRANSMISSION_TEMP, trans_t)

            # --- vehicle speed: slow drive cycle (0.1 km/h) ---
            vspd = max(0, min(2800, int(500 + 450 * math.sin(t * 0.10))))
            self._wu32(ADDR_VEHICLE_SPEED, vspd)

            # --- individual wheel speeds with small variance ---
            self._wu32(ADDR_WHEEL_SPEED_FL, max(0, min(2800, vspd + int(15 * math.sin(t * 0.7)))))
            self._wu32(ADDR_WHEEL_SPEED_FR, max(0, min(2800, vspd - int(10 * math.sin(t * 0.9)))))
            self._wu32(ADDR_WHEEL_SPEED_RL, max(0, min(2800, vspd + int(20 * math.sin(t * 0.5)))))
            self._wu32(ADDR_WHEEL_SPEED_RR, max(0, min(2800, vspd - int(12 * math.sin(t * 0.6)))))

            # --- accelerations (0.01 m/s²) ---
            long_a = max(-2000, min(2000, int(200 * math.cos(t * 0.10))))
            self._ws32(ADDR_LONG_ACCELERATION, long_a)
            lat_a = max(-1500, min(1500, int(120 * math.sin(t * 0.15))))
            self._ws32(ADDR_LAT_ACCELERATION, lat_a)

            # --- yaw rate (0.1 deg/s) ---
            yaw = max(-1500, min(1500, int(100 * math.sin(t * 0.12))))
            self._ws32(ADDR_YAW_RATE, yaw)

            # --- fuel rail pressure (0.1 bar): rises with load ---
            frp = max(0, min(2200, int(500 + load * 12)))
            self._wu32(ADDR_FUEL_RAIL_PRESSURE, frp)

            # --- lambda sensors: closed-loop oscillation around stoich ---
            lambda1 = max(500, min(2000, int(1000 + 20 * math.sin(t * 2.5))))
            self._wu32(ADDR_LAMBDA_SENSOR_1, lambda1)
            lambda2 = max(500, min(1500, int(1000 + 15 * math.sin(t * 2.5 + 0.5))))
            self._wu32(ADDR_LAMBDA_SENSOR_2, lambda2)

            # --- fuel tank: slowly draining from 75 % ---
            tank = max(0, min(100, int(75 - t * 0.005)))
            self._wu32(ADDR_FUEL_TANK_LEVEL, tank)

            # --- fuel consumption (0.1 l/100km) ---
            fc = max(0, min(500, int(50 + load * 2)))
            self._wu32(ADDR_FUEL_CONSUMPTION, fc)

            # --- battery and alternator (with alternator running) ---
            batt_v = max(800, min(1600, int(1380 + 20 * math.sin(t * 0.5))))
            self._wu32(ADDR_BATTERY_VOLTAGE, batt_v)
            batt_i = max(-2500, min(2500, int(-100 + load * 3)))
            self._ws32(ADDR_BATTERY_CURRENT, batt_i)
            alt_v = max(1100, min(1550, int(1400 + 10 * math.sin(t * 0.3))))
            self._wu32(ADDR_ALTERNATOR_VOLTAGE, alt_v)

            # --- ECU 5V supply with small ripple (0.001 V) ---
            ecu_v = max(4500, min(5500, int(5000 + 30 * math.sin(t * 5.0))))
            self._wu32(ADDR_ECU_SUPPLY_VOLTAGE, ecu_v)

    # ------------------------------------------------------------------
    # Private
    # ------------------------------------------------------------------

    def _init_defaults(self) -> None:
        """Initial warm-idle values matching A2L ranges (big-endian)."""
        # Measurements
        self._wu32(ADDR_ENGINE_SPEED,       800)
        self._wu32(ADDR_ENGINE_LOAD,         10)
        self._wu32(ADDR_INJECTION_TIME,    2800)
        self._ws32(ADDR_IGNITION_ANGLE,      90)   # 9.0 deg BTDC
        self._wu32(ADDR_THROTTLE_POS,         5)
        self._wu32(ADDR_ENGINE_TORQUE,      400)   # 40.0 Nm
        self._wu32(ADDR_MASS_AIR_FLOW,      200)   # 2.00 g/s
        self._wu32(ADDR_MANIFOLD_PRESSURE,  370)   # 37.0 kPa (idle vacuum)
        self._ws32(ADDR_COOLANT_TEMP,       200)   # 20.0 °C (cold)
        self._ws32(ADDR_OIL_TEMP,           200)   # 20.0 °C
        self._ws32(ADDR_INTAKE_AIR_TEMP,    220)   # 22.0 °C
        self._wu32(ADDR_EXHAUST_GAS_TEMP,  2500)   # 250.0 °C
        self._ws32(ADDR_TRANSMISSION_TEMP,  200)   # 20.0 °C
        self._wu32(ADDR_VEHICLE_SPEED,        0)
        self._wu32(ADDR_WHEEL_SPEED_FL,       0)
        self._wu32(ADDR_WHEEL_SPEED_FR,       0)
        self._wu32(ADDR_WHEEL_SPEED_RL,       0)
        self._wu32(ADDR_WHEEL_SPEED_RR,       0)
        self._ws32(ADDR_LONG_ACCELERATION,    0)
        self._ws32(ADDR_LAT_ACCELERATION,     0)
        self._ws32(ADDR_YAW_RATE,             0)
        self._wu32(ADDR_FUEL_RAIL_PRESSURE,  500)  # 50.0 bar
        self._wu32(ADDR_LAMBDA_SENSOR_1,    1000)  # 1.000 lambda
        self._wu32(ADDR_LAMBDA_SENSOR_2,    1000)
        self._wu32(ADDR_FUEL_TANK_LEVEL,      75)  # 75 %
        self._wu32(ADDR_FUEL_CONSUMPTION,     80)  # 8.0 l/100km
        self._wu32(ADDR_BATTERY_VOLTAGE,    1380)  # 13.80 V
        self._ws32(ADDR_BATTERY_CURRENT,     -50)  # -5.0 A (quiescent draw)
        self._wu32(ADDR_ALTERNATOR_VOLTAGE, 1400)  # 14.00 V
        self._wu32(ADDR_ECU_SUPPLY_VOLTAGE, 5000)  # 5.000 V

        # Calibration characteristics (FLOAT32, big-endian)
        self._wf32(ADDR_MAX_ENGINE_SPEED,          7500.0)
        self._wf32(ADDR_REV_LIMITER_CUT_IN,        7000.0)
        self._wf32(ADDR_IDLE_TARGET_SPEED,           800.0)
        self._wf32(ADDR_IDLE_TARGET_SPEED_COLD,     1200.0)
        self._wf32(ADDR_TORQUE_LIMIT,               350.0)
        self._wf32(ADDR_IGNITION_ADVANCE_BASE,       12.0)
        self._wf32(ADDR_INJECTOR_FLOW_RATE,         350.0)
        self._wf32(ADDR_INJECTOR_DEAD_TIME,           0.4)
        self._wf32(ADDR_TARGET_LAMBDA_IDLE,           1.00)
        self._wf32(ADDR_TARGET_LAMBDA_WOT,            0.88)
        self._wf32(ADDR_FUEL_PUMP_DUTY,              80.0)
        self._wf32(ADDR_CRANKING_FUEL_ENRICHMENT,   150.0)
        self._wf32(ADDR_FAN_ON_TEMP,                 95.0)
        self._wf32(ADDR_FAN_OFF_TEMP,                88.0)
        self._wf32(ADDR_OIL_TEMP_WARNING,           135.0)
        self._wf32(ADDR_COOLANT_TEMP_WARNING,        115.0)
        self._wf32(ADDR_COLD_START_TEMP_THRESHOLD,   20.0)
        self._wf32(ADDR_BATTERY_LOW_VOLTAGE,         11.5)
        self._wf32(ADDR_ALTERNATOR_TARGET_VOLTAGE,   14.2)
        self._wf32(ADDR_MAX_FUEL_RAIL_PRESSURE,     200.0)

    @staticmethod
    def _check_range(address: int, size: int) -> None:
        if address < 0 or (address + size) > _MEM_SIZE:
            raise ValueError(
                f"Memory access out of range: "
                f"addr=0x{address:08X} size={size} "
                f"(memory is 0x{_MEM_SIZE:08X} bytes)"
            )

    def _wu32(self, addr: int, value: int) -> None:
        """Write big-endian unsigned 32-bit integer (A2L ULONG)."""
        self._data[addr: addr + 4] = struct.pack(">I", value & 0xFFFFFFFF)

    def _ws32(self, addr: int, value: int) -> None:
        """Write big-endian signed 32-bit integer (A2L SLONG)."""
        self._data[addr: addr + 4] = struct.pack(">i", value)

    def _wf32(self, addr: int, value: float) -> None:
        """Write big-endian IEEE 754 float32 (A2L FLOAT32_SCALAR)."""
        self._data[addr: addr + 4] = struct.pack(">f", value)
