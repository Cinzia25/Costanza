import os
import time
from typing import Tuple

import RPi.GPIO as GPIO
import Adafruit_PCA9685


# -------------------------
# Hardware / calibration constants
# -------------------------
PWM_FREQ_HZ: int = 60          # PCA9685 PWM frequency
PWM_MIN: int = 0               # Absolute minimum duty for PCA9685
PWM_MAX: int = 4095            # Absolute maximum duty for PCA9685

# Empirical deadband compensation: below this, motors may not move.
# Keep this as-is if already tuned on your platform.
PWM_DEADBAND: int = 1200

# Robot kinematics (tweak to your platform)
WHEEL_BASE_M: float = 0.143    # Distance between wheel centers [m]
WHEEL_RADIUS_M: float = 0.035  # Wheel radius [m] (not used in current mapping)

# Max tangential wheel speed used for scaling to PWM
V_MAX_MPS: float = 0.5         # Maximum wheel tangential speed [m/s]


# -------------------------
# Hardware setup
# -------------------------
# Optional: read a per-robot ID from environment (currently unused).
# Kept as underscore-prefixed to signal "unused but intentional".
_self_id = os.environ.get("SELF_ID")

# Initialize PWM driver and GPIO
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(PWM_FREQ_HZ)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# H-bridge logic pins (INx) and PCA9685 channels (ENx) for left/right motor
IN1 = 23
IN2 = 24
IN3 = 27
IN4 = 22

ENA = 0  # Left motor PWM channel on PCA9685
ENB = 1  # Right motor PWM channel on PCA9685

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)


def custom_speed(speed_left: float, speed_right: float) -> None:
    """
    Drive the H-bridge for differential motors with signed PWM magnitudes.

    Positive values command forward rotation; negative values command reverse.
    Direction is set via the INx pins and magnitude via the PCA9685 duty.

    Parameters
    ----------
    speed_left : float
        Signed PWM value for the left motor. Magnitude is interpreted as duty
        (0..4095); sign sets direction via IN1/IN2.
    speed_right : float
        Signed PWM value for the right motor. Magnitude is interpreted as duty
        (0..4095); sign sets direction via IN3/IN4.

    Notes
    -----
    This function does not clamp values; upstream functions should ensure the
    PWM range is valid for the PCA9685 (0..4095).
    """
    # Select directions and convert to magnitudes
    if speed_left < 0:
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN1, GPIO.HIGH)
        speed_left = -speed_left
    else:
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN1, GPIO.LOW)

    if speed_right < 0:
        GPIO.output(IN4, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        speed_right = -speed_right
    else:
        GPIO.output(IN4, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)

    # Apply PWM to each motor (expects 0..4095)
    pwm.set_pwm(ENA, 0, int(speed_left))
    pwm.set_pwm(ENB, 0, int(speed_right))


def stopcar() -> None:
    """
    Immediately stop both motors (coast/brake depending on H-bridge wiring).

    Sets all H-bridge inputs LOW and zeroes PWM duty on both channels.
    Use this for safety stops or during shutdown.
    """
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm.set_pwm(ENA, 0, 0)
    pwm.set_pwm(ENB, 0, 0)


def get_pwm(V_R: float, V_L: float) -> Tuple[float, float]:
    """
    Map desired wheel tangential speeds to PWM duty (left, right).

    The mapping:
      1) Saturates requested speeds to ±V_MAX_MPS.
      2) Scales to [0, 1] relative to V_MAX_MPS.
      3) Converts to PCA9685 counts with a deadband offset so motors start
         moving more reliably.

    Parameters
    ----------
    V_R : float
        Desired right wheel tangential speed [m/s] (signed).
    V_L : float
        Desired left  wheel tangential speed [m/s] (signed).

    Returns
    -------
    (pwm_L, pwm_R) : tuple[float, float]
        Unsigned PWM duties for left and right wheels in [0, 4095].
        Sign handling is performed in `calculate_speed()` / `custom_speed()`.

    Notes
    -----
    - This function ignores input sign and returns magnitudes only.
    - The deadband (PWM_DEADBAND) and V_MAX_MPS are empirical and may need
      tuning for your platform and supply voltage.
    """
    # Work with magnitudes
    V_R = abs(V_R)
    V_L = abs(V_L)

    # Saturate to max allowed wheel speed
    V_sat_R = min(V_R, 0.1)
    V_sat_L = min(V_L, 0.1)

    # Compute scaling to preserve commanded ratio when one wheel saturates
    alpha_R = (V_sat_R / V_R) if V_R > 0 else 1.0
    alpha_L = (V_sat_L / V_L) if V_L > 0 else 1.0
    alpha = min(alpha_L, alpha_R)

    # Apply common scaling
    #V_scaled_R = V_sat_R * alpha
    #V_scaled_L = V_sat_L * alpha
    V_scaled_R = V_R * alpha
    V_scaled_L = V_L * alpha

    # Normalize to [0, 1]
    v_R = V_scaled_R / V_MAX_MPS if V_MAX_MPS > 0 else 0.0
    v_L = V_scaled_L / V_MAX_MPS if V_MAX_MPS > 0 else 0.0
    print('v_r: ',v_R,  'v_l: ', v_L)

    # Map to PCA9685 duty counts with deadband offset
    pwm_R = PWM_DEADBAND + (PWM_MAX - PWM_DEADBAND) * v_R if v_R > 0 else 0.0
    pwm_L = PWM_DEADBAND + (PWM_MAX - PWM_DEADBAND) * v_L if v_L > 0 else 0.0

    # Ensure we stay within absolute limits (defensive)
    pwm_R = max(PWM_MIN, min(PWM_MAX, pwm_R))
    pwm_L = max(PWM_MIN, min(PWM_MAX, pwm_L))

    return pwm_L, pwm_R


def calculate_speed(v_lin: float, v_ang: float) -> Tuple[float, float]:
    """
    Convert differential-drive (v, ω) commands to signed PWM for each wheel.

    Kinematics:
        V_R = v + (b/2) * ω
        V_L = v - (b/2) * ω
    where:
        v : linear velocity of chassis [m/s]
        ω : angular velocity about vertical axis [rad/s]
        b : track width (WHEEL_BASE_M) [m]

    Parameters
    ----------
    v_lin : float
        Linear velocity command v [m/s].
    v_ang : float
        Angular velocity command ω [rad/s].

    Returns
    -------
    (pwm_left, pwm_right) : tuple[float, float]
        Signed PWM commands for left and right wheels in [-4095, 4095].
        The sign conveys direction; magnitude is compatible with PCA9685.

    Notes
    -----
    - Wheel radius is not used here because we map tangential wheel speeds
      directly to PWM through `get_pwm()`.
    - Direction is applied later in `custom_speed()`.
    """
    # Wheel tangential speeds from (v, ω)
    V_R = v_lin + (WHEEL_BASE_M / 2.0) * v_ang  # [m/s]
    V_L = v_lin - (WHEEL_BASE_M / 2.0) * v_ang  # [m/s]

    # Track direction for each wheel
    sign_R = -1.0 if V_R < 0 else 1.0
    sign_L = -1.0 if V_L < 0 else 1.0

    # Get unsigned PWM magnitudes from desired speeds
    pwm_L, pwm_R = get_pwm(V_R, V_L)

    # Re-apply signs so downstream can set INx pins correctly
    pwm_L *= sign_L
    pwm_R *= sign_R

    return pwm_L, pwm_R


def move_robot(v_lin: float, v_ang: float) -> None:
    """
    High-level entry point: apply a (v, ω) command to the motors.

    Parameters
    ----------
    v_lin : float
        Linear velocity v [m/s].
    v_ang : float
        Angular velocity ω [rad/s].

    Behavior
    --------
    - Computes wheel-specific signed PWM via `calculate_speed()`.
    - Sets direction pins and PWM duty via `custom_speed()`.

    Safety
    ------
    - This function does no range checking on v_lin/v_ang.
      Upstream nodes (e.g., ROS) should bound commands to safe values
      for your platform.
    """
    v_left, v_right = calculate_speed(v_lin, v_ang)
    custom_speed(v_left, v_right)
