import numpy as np
import matplotlib.pyplot as plt


# ============================================================
# File Name  : dq_transform_demo.py
# Author     : Williamson Lin
# Description:
#   This script demonstrates the basic control-oriented flow of:
#       abc -> alpha-beta -> dq
#
#   The purpose of this file is not only mathematical verification,
#   but also to serve as an algorithm prototype before future
#   firmware/DSP implementation.
#
#   In digital power / motor control applications, coordinate
#   transformation is often used to convert sinusoidal AC variables
#   into near-DC quantities in the synchronous rotating frame,
#   making PI control easier to design.
#
# Note:
#   This demo uses balanced three-phase signals for transformation
#   study. Although Totem-Pole PFC is usually single-phase, the
#   underlying concept of coordinate transform is still valuable
#   for understanding PLL, current loop design, and control modeling.
# ============================================================


# ------------------------------------------------------------
# Function: gen_three_phase_signal
# Purpose :
#   Generate balanced 3-phase sinusoidal signals
#
# Input   :
#   amp       - signal amplitude
#   freq_hz   - electrical frequency
#   t         - time array
#
# Output  :
#   ia, ib, ic, theta
#
# Remarks :
#   theta = w*t is the electrical angle used later in Park transform
# ------------------------------------------------------------
def gen_three_phase_signal(amp, freq_hz, t):
    omega = 2.0 * np.pi * freq_hz
    theta = omega * t

    ia = amp * np.cos(theta)
    ib = amp * np.cos(theta - 2.0 * np.pi / 3.0)
    ic = amp * np.cos(theta + 2.0 * np.pi / 3.0)

    return ia, ib, ic, theta


# ------------------------------------------------------------
# Function: clarke_transform
# Purpose :
#   Convert 3-phase stationary quantities (abc)
#   into 2-axis stationary frame (alpha-beta)
#
# Input   :
#   ia, ib, ic - 3-phase quantities
#
# Output  :
#   alpha, beta
#
# Remarks :
#   This is a commonly used amplitude-invariant form.
# ------------------------------------------------------------
def clarke_transform(ia, ib, ic):
    alpha = (2.0 / 3.0) * (ia - 0.5 * ib - 0.5 * ic)
    beta = (2.0 / 3.0) * ((np.sqrt(3.0) / 2.0) * (ib - ic))
    return alpha, beta


# ------------------------------------------------------------
# Function: park_transform
# Purpose :
#   Convert alpha-beta stationary frame into dq rotating frame
#
# Input   :
#   alpha, beta - stationary frame quantities
#   theta       - rotating electrical angle
#
# Output  :
#   d, q
#
# Remarks :
#   If theta is synchronized with the input AC vector,
#   then d/q can become nearly DC values.
#   This is the key reason why dq transform is widely used
#   in digital control systems.
# ------------------------------------------------------------
def park_transform(alpha, beta, theta):
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)

    d = alpha * cos_t + beta * sin_t
    q = -alpha * sin_t + beta * cos_t

    return d, q


# ------------------------------------------------------------
# Function: inv_park_transform
# Purpose :
#   Convert dq rotating frame back to alpha-beta frame
# ------------------------------------------------------------
def inv_park_transform(d, q, theta):
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)

    alpha = d * cos_t - q * sin_t
    beta = d * sin_t + q * cos_t

    return alpha, beta


# ------------------------------------------------------------
# Function: inv_clarke_transform
# Purpose :
#   Convert alpha-beta frame back to abc frame
# ------------------------------------------------------------
def inv_clarke_transform(alpha, beta):
    ia = alpha
    ib = -0.5 * alpha + (np.sqrt(3.0) / 2.0) * beta
    ic = -0.5 * alpha - (np.sqrt(3.0) / 2.0) * beta
    return ia, ib, ic


# ------------------------------------------------------------
# Function: main
# Purpose :
#   1. Generate balanced 3-phase input
#   2. Perform abc -> alpha-beta -> dq
#   3. Reconstruct back to abc for verification
#   4. Plot waveforms for visualization
#
# Firmware Thinking:
#   This flow can be viewed as an algorithm-level prototype
#   before moving to embedded C / DSP implementation.
# ------------------------------------------------------------
def main():
    # ========================================================
    # [1] Simulation Parameters
    # ========================================================
    AMP = 10.0               # peak amplitude
    FREQ_HZ = 50.0           # electrical frequency
    SIM_TIME = 0.04          # 40 ms
    SAMPLE_NUM = 2000

    t = np.linspace(0.0, SIM_TIME, SAMPLE_NUM)

    # ========================================================
    # [2] Generate 3-phase signals
    # ========================================================
    ia, ib, ic, theta = gen_three_phase_signal(AMP, FREQ_HZ, t)

    # ========================================================
    # [3] Clarke Transform: abc -> alpha-beta
    # ========================================================
    alpha, beta = clarke_transform(ia, ib, ic)

    # ========================================================
    # [4] Park Transform: alpha-beta -> dq
    #     Here theta is synchronized with the source angle.
    #     Therefore:
    #         d-axis should be close to a constant
    #         q-axis should be close to zero
    # ========================================================
    d, q = park_transform(alpha, beta, theta)

    # ========================================================
    # [5] Inverse Transform Verification
    #     dq -> alpha-beta -> abc
    # ========================================================
    alpha_rec, beta_rec = inv_park_transform(d, q, theta)
    ia_rec, ib_rec, ic_rec = inv_clarke_transform(alpha_rec, beta_rec)

    # ========================================================
    # [6] Numeric Check
    # ========================================================
    print("==================================================")
    print("DQ Transform Demo")
    print("==================================================")
    print(f"d mean     = {np.mean(d):.6f}")
    print(f"q mean     = {np.mean(q):.6f}")
    print(f"d std dev  = {np.std(d):.6f}")
    print(f"q std dev  = {np.std(q):.6f}")
    print()

    print("Expected behavior under synchronous angle:")
    print("1. d-axis becomes nearly DC")
    print("2. q-axis becomes nearly zero")
    print()

    err_ia = np.max(np.abs(ia - ia_rec))
    err_ib = np.max(np.abs(ib - ib_rec))
    err_ic = np.max(np.abs(ic - ic_rec))

    print("Reconstruction error:")
    print(f"max |ia - ia_rec| = {err_ia:.10f}")
    print(f"max |ib - ib_rec| = {err_ib:.10f}")
    print(f"max |ic - ic_rec| = {err_ic:.10f}")

    # ========================================================
    # [7] Plot for GitHub demonstration / report usage
    # ========================================================
    plt.figure(figsize=(12, 8))

    plt.subplot(3, 1, 1)
    plt.plot(t, ia, label='ia')
    plt.plot(t, ib, label='ib')
    plt.plot(t, ic, label='ic')
    plt.title('Balanced 3-Phase Signals (abc)')
    plt.ylabel('Amplitude')
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(t, alpha, label='alpha')
    plt.plot(t, beta, label='beta')
    plt.title('Clarke Transform Output (alpha-beta)')
    plt.ylabel('Amplitude')
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(t, d, label='d')
    plt.plot(t, q, label='q')
    plt.title('Park Transform Output (dq)')
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()