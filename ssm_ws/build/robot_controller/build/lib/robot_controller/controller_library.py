import os
import logging
from typing import Tuple

import RPi.GPIO as GPIO
import Adafruit_PCA9685


# -------------------------
# Config
# -------------------------
PWM_FREQ_HZ: int = 60
PWM_MIN: int = 0
PWM_MAX: int = 4095
PWM_DEADBAND: int = 1200

WHEEL_BASE_M: float = 0.143
WHEEL_RADIUS_M: float = 0.035
V_MAX_MPS: float = 0.5

# Logger (meglio del print)
logger = logging.getLogger("controller_library")
logger.setLevel(logging.INFO)  # oppure DEBUG per test


# -------------------------
# Hardware setup
# -------------------------
_self_id = os.environ.get("SELF_ID")

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(PWM_FREQ_HZ)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pin H-bridge
IN1, IN2, IN3, IN4 = 23, 24, 27, 22
ENA, ENB = 0, 1  # PCA9685 channels

for pin in [IN1, IN2, IN3, IN4]:
    GPIO.setup(pin, GPIO.OUT)


def custom_speed(speed_left: float, speed_right: float) -> None:
    """Invia PWM ai motori (direzione via GPIO, duty via PCA9685)."""
    # Direzione left
    if speed_left < 0:
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN1, GPIO.HIGH)
        speed_left = -speed_left
    else:
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN1, GPIO.LOW)

    # Direzione right
    if speed_right < 0:
        GPIO.output(IN4, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        speed_right = -speed_right
    else:
        GPIO.output(IN4, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)

    # Clamp difensivo
    speed_left = max(PWM_MIN, min(PWM_MAX, int(speed_left)))
    speed_right = max(PWM_MIN, min(PWM_MAX, int(speed_right)))

    # Scrivi solo se serve (evita transazioni I²C inutili)
    pwm.set_pwm(ENA, 0, speed_left)
    pwm.set_pwm(ENB, 0, speed_right)


def stopcar() -> None:
    """Arresta immediatamente entrambi i motori."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm.set_pwm(ENA, 0, 0)
    pwm.set_pwm(ENB, 0, 0)


def get_pwm(V_R: float, V_L: float) -> Tuple[float, float]:
    """Converte velocità tangenziali desiderate in PWM (magnitudine)."""
    V_R = abs(V_R)
    V_L = abs(V_L)

    # Saturazione
    V_sat_R = min(V_R, 0.1)
    V_sat_L = min(V_L, 0.1)

    # Scala per mantenere proporzioni
    alpha_R = (V_sat_R / V_R) if V_R > 0 else 1.0
    alpha_L = (V_sat_L / V_L) if V_L > 0 else 1.0
    alpha = min(alpha_L, alpha_R)

    V_scaled_R = V_R * alpha
    V_scaled_L = V_L * alpha

    # Normalizzazione [0,1]
    v_R = V_scaled_R / V_MAX_MPS if V_MAX_MPS > 0 else 0.0
    v_L = V_scaled_L / V_MAX_MPS if V_MAX_MPS > 0 else 0.0

    # Con deadband
    pwm_R = PWM_DEADBAND + (PWM_MAX - PWM_DEADBAND) * v_R if v_R > 0 else 0.0
    pwm_L = PWM_DEADBAND + (PWM_MAX - PWM_DEADBAND) * v_L if v_L > 0 else 0.0

    pwm_R = max(PWM_MIN, min(PWM_MAX, pwm_R))
    pwm_L = max(PWM_MIN, min(PWM_MAX, pwm_L))

    return pwm_L, pwm_R


def calculate_speed(v_lin: float, v_ang: float) -> Tuple[float, float]:
    """Converte (v, ω) in PWM signed per le ruote."""
    V_R = v_lin + (WHEEL_BASE_M / 2.0) * v_ang
    V_L = v_lin - (WHEEL_BASE_M / 2.0) * v_ang

    sign_R = -1.0 if V_R < 0 else 1.0
    sign_L = -1.0 if V_L < 0 else 1.0

    pwm_L, pwm_R = get_pwm(V_R, V_L)

    pwm_L *= sign_L
    pwm_R *= sign_R

    return pwm_L, pwm_R


def move_robot(v_lin: float, v_ang: float) -> None:
    """Applica comando (v, ω) ai motori."""
    v_left, v_right = calculate_speed(v_lin, v_ang)
    custom_speed(v_left, v_right)
