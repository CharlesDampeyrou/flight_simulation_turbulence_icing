import numpy as np


def Hu(s, parameters):
    """
    Filter for obtaining longitudinal turbulence from band-limited white noise of unit variance.
    The formula is taken from the American military standard MIL-HDBK-1797A.
    Parameters :
    - s : frequency
    - parameters: dictionary containing filter parameters:
        - sigma_u: turbulence standard deviation, m.s-1
        - Lu: turbulence length scale, m
        - V: aircraft speed, m.s-1
    """
    sigma_u = parameters["sigma_u"]
    Lu = parameters["Lu"]
    V = parameters["V"]
    return (
        sigma_u
        * np.sqrt(2 * Lu / np.pi / V)
        * (1 + 0.25 * Lu * s / V)
        / (1 + 1.357 * Lu * s / V + 0.1987 * (Lu * s / V) ** 2)
    )


def Hp(s, parameters):
    """
    Filter for obtaining 'rotary disturbance velocity' from band-limited white noise of unit variance.
    The formula is taken from the American military standard MIL-HDBK-1797A.
    Parameters :
    - s : frequency
    - parameters: dictionary containing filter parameters
        - sigma_w: standard deviation of vertical turbulence, m.s-1
        - Lw: vertical turbulence length scale, m
        - b: aircraft wingspan, m
        - V: aircraft speed, m.s-1
    """
    sigma_w = parameters["sigma_w"]
    Lw = parameters["Lw"]
    b = parameters["b"]
    V = parameters["V"]
    return (
        sigma_w
        * np.sqrt(0.8 / V)
        * (np.pi / 4 / b) ** (1 / 6)
        / ((2 * Lw) ** (1 / 3) * (1 + 4 * b * s / np.pi / V))
    )


def Hv(s, parameters):
    """
    Filter for obtaining lateral turbulence from band-limited white noise of unit variance.
    The formula is taken from the American military standard MIL-HDBK-1797A.
    Parameters :
    - s : frequency
    - parameters: dictionary containing filter parameters:
        - sigma_v: standard deviation of turbulence, m.s-1
        - Lv: turbulence length scale, m
        - V: aircraft speed, m.s-1
    """
    sigma_v = parameters["sigma_v"]
    Lv = parameters["Lv"]
    V = parameters["V"]
    return (
        sigma_v
        * np.sqrt(2 * Lv / np.pi / V)
        * (1 + 2.7478 * 2 * Lv * s / V + 0.3398 * (2 * Lv * s / V) ** 2)
        / (
            1
            + 2.9958 * 2 * Lv * s / V
            + 1.9754 * (2 * Lv * s / V) ** 2
            + 0.1539 * (2 * Lv * s / V) ** 3
        )
    )


def Hr(s, parameters):
    """
    Filter for obtaining 'rotary disturbance velocity' from band-limited white noise of unit variance.
    The formula is taken from the American military standard MIL-HDBK-1797A.
    Parameters :
    - s : frequency
    - parameters: dictionary containing filter parameters
        - sigma_v: standard deviation of horizontal turbulence, m.s-1
        - Lv: vertical turbulence length scale, m
        - b: aircraft wingspan, m
        - V: aircraft speed, m.s-1
    """
    b = parameters["b"]
    V = parameters["V"]
    Hv_value = Hv(s, parameters)
    return -s * Hv_value / V / (1 + 3 * b * s / np.pi / V)


def Hw(s, parameters):
    """
    Filter for obtaining vertical turbulence from band-limited white noise of unit variance.
    The formula is taken from the American military standard MIL-HDBK-1797A.
    Parameters :
    - s : frequency
    - parameters: dictionary containing filter parameters:
        - sigma_w: standard deviation of turbulence, m.s-1
        - Lw: turbulence length scale, m
        - V: aircraft speed, m.s-1
    """
    sigma_w = parameters["sigma_w"]
    Lw = parameters["Lw"]
    V = parameters["V"]
    return (
        sigma_w
        * np.sqrt(2 * Lw / np.pi / V)
        * (1 + 2.7478 * 2 * Lw * s / V + 0.3398 * (2 * Lw * s / V) ** 2)
        / (
            1
            + 2.9958 * 2 * Lw * s / V
            + 1.9754 * (2 * Lw * s / V) ** 2
            + 0.1539 * (2 * Lw * s / V) ** 3
        )
    )


def Hq(s, parameters):
    """
    Filter for obtaining 'rotary disturbance velocity' from band-limited white noise of unit variance.
    The formula is taken from the American military standard MIL-HDBK-1797A.
    Parameters :
    - s : frequency
    - parameters: dictionary containing filter parameters
        - sigma_w: standard deviation of vertical turbulence, m.s-1
        - Lw: vertical turbulence length scale, m
        - b: aircraft wingspan, m
    - V: aircraft speed, m.s-1
    """
    b = parameters["b"]
    V = parameters["V"]
    Hw_value = Hw(s, parameters)
    return s * Hw_value / V / (1 + 4 * b * s / np.pi / V)


def phi_u(omega, parameters):
    """
    Power spectrum of longitudinal turbulence. The spectrum
    is taken from the American military standard MIL-HDBK-1797A.
    Parameters :
    - omega: frequency, rad.s-1
    - parameters: dictionary containing filter parameters:
        - sigma_u: turbulence standard deviation, m.s-1
        - Lu: turbulence length scale, m
        - V: aircraft speed, m.s-1
    """
    sigma_u = parameters["sigma_u"]
    Lu = parameters["Lu"]
    V = parameters["V"]
    return sigma_u**2 * 2 * Lu / np.pi / (1 + (1.339 * Lu * omega / V) ** 2) ** (5 / 6)


def phi_v(omega, parameters):
    """
    Power spectrum of lateral turbulence. The spectrum
    is taken from the American military standard MIL-HDBK-1797A.
    Parameters :
    - omega: frequency, rad.s-1
    - parameters: dictionary containing filter parameters:
        - sigma_v: turbulence standard deviation, m.s-1
        - Lv: turbulence length scale, m
        - V: aircraft speed, m.s-1
    """
    sigma_v = parameters["sigma_v"]
    Lv = parameters["Lv"]
    V = parameters["V"]
    return (
        sigma_v**2
        * 2
        * Lv
        / np.pi
        * (1 + 8 / 3 * (2.678 * Lv * omega / V) ** 2)
        / (1 + (2.678 * Lv * omega / V) ** 2) ** (11 / 6)
    )


def phi_w(omega, parameters):
    """
    Power spectrum of vertical turbulence. The spectrum
    is taken from the American military standard MIL-HDBK-1797A.
    Parameters :
    - omega: frequency, rad.s-1
    - parameters: dictionary containing spectrum parameters:
        - sigma_w: turbulence standard deviation, m.s-1
        - Lw: turbulence length scale, m
        - V: aircraft speed, m.s-1
    """
    sigma_w = parameters["sigma_w"]
    Lw = parameters["Lw"]
    V = parameters["V"]
    return (
        sigma_w**2
        * 2
        * Lw
        / np.pi
        * (1 + 8 / 3 * (2.678 * Lw * omega / V) ** 2)
        / (1 + (2.678 * Lw * omega / V) ** 2) ** (11 / 6)
    )


def phi_p(omega, parameters):
    """
    Longitudinal rotary disturbance velocity power spectrum.
    MIL-HDBK-1797A does not provide a formula for rotary disturbance velocity spectra.
    They are deduced from MIL-F-8785C by replacing Lv by 2*Lv and Lw by 2*Lw since their definition differs between the two standards.
    - omega: frequency, rad.s-1
    - parameters: dictionary containing spectrum parameters:
        - sigma_w: standard deviation of vertical turbulence, m.s-1
        - Lw: vertical turbulence length scale, m
        - b: aircraft wingspan, m
        - V: aircraft speed, m.s-1
    """
    sigma_w = parameters["sigma_w"]
    Lw = parameters["Lw"]
    b = parameters["b"]
    V = parameters["V"]
    return (
        sigma_w**2
        / 2
        / Lw
        * 0.8
        * (2 * np.pi * Lw / 4 / b) ** (1 / 3)
        / (1 + (4 * b * omega / np.pi / V) ** 2)
    )


def phi_q(omega, parameters):
    """
    y-axis rotary disturbance velocity power spectrum.
    MIL-HDBK-1797A does not provide a formula for rotary disturbance velocity spectra.
    They are deduced from MIL-F-8785C by replacing Lv by 2*Lv and Lw by 2*Lw since their definition differs between the two standards.
    - omega: frequency, rad.s-1
    - parameters: dictionary containing spectrum parameters:
        - sigma_w: standard deviation of vertical turbulence, m.s-1
        - Lw: vertical turbulence length scale, m
        - b: aircraft wingspan, m
        - V: aircraft speed, m.s-1
    """
    b = parameters["b"]
    V = parameters["V"]
    return (
        (omega / V) ** 2
        / (1 + (4 * b * omega / np.pi / V) ** 2)
        * phi_w(omega, parameters)
    )


def phi_r(omega, parameters):
    """
    z-axis rotary disturbance velocity power spectrum.
    MIL-HDBK-1797A does not provide a formula for rotary disturbance velocity spectra.
    They are deduced from MIL-F-8785C by replacing Lv by 2*Lv and Lw by 2*Lw since their definition differs between the two standards.
    - omega: frequency, rad.s-1
    - parameters: dictionary containing spectrum parameters:
        - sigma_v: standard deviation of lateral turbulence, m.s-1
        - Lv: lateral turbulence length scale, m
        - b: aircraft wingspan, m
        - V: aircraft speed, m.s-1
    """
    b = parameters["b"]
    V = parameters["V"]
    return (
        ((omega / V) ** 2)
        / (1 + (3 * b * omega / np.pi / V) ** 2)
        * phi_v(omega, parameters)
    )


"""
The functions f_u, f_v, f_w, f_p, f_q and f_r are the module of the turbulence spectrum. They are used to generate the turbulence because phi_u, phi_v, phi_w, phi_p, phi_q and phi_r are the power spectra of the turbulence and therefore the square of the module of the Fourier transform of the turbulence.
"""


def f_u(omega, parameters):
    return np.sqrt(phi_u(omega, parameters))


def f_v(omega, parameters):
    return np.sqrt(phi_v(omega, parameters))


def f_w(omega, parameters):
    return np.sqrt(phi_w(omega, parameters))


def f_p(omega, parameters):
    return np.sqrt(phi_p(omega, parameters))


def f_q(omega, parameters):
    return np.sqrt(phi_q(omega, parameters))


def f_r(omega, parameters):
    return np.sqrt(phi_r(omega, parameters))
