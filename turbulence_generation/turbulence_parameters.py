import numpy as np


def turbulence_std(h_m, turbulence_level):
    """
    This function returns the standard deviation of the turbulence intensity
    in the 3 directions, as a function of the altitude.
    The formula is taken from the military standard MIL-HDBK-1797A (page 799-810).
    Parameters:
    - h_m : altitude in meters
    - turbulence_level : turbulence level (0, 1, 2 or 3 for no-turbulence, light,
    moderate or severe turbulence)
    Returns :
    - sigma_u : standard deviation of the longitudinal turbulence
    - sigma_v : standard deviation of the lateral turbulence
    - sigma_w : standard deviation of the vertical turbulence
    """
    m2feet = 3.28084
    h_ft = h_m * m2feet
    if h_ft <= 1000:
        if turbulence_level == 0:
            W20_kn = 5  # Wind speed at 20 ft (knots)
        elif turbulence_level == 1:
            W20_kn = 15
        elif turbulence_level == 2:
            W20_kn = 30
        elif turbulence_level == 3:
            W20_kn = 45
        sigma_w_ft_sec = 0.1 * W20_kn
        sigma_u_ft_sec = sigma_w_ft_sec / (0.177 + 0.000823 * h_ft) ** 0.4
        sigma_v_ft_sec = sigma_u_ft_sec
        sigma_u = sigma_u_ft_sec / m2feet
        sigma_v = sigma_v_ft_sec / m2feet
        sigma_w = sigma_w_ft_sec / m2feet
    elif h_ft >= 2000:  # stds from figure 262, page 799
        if turbulence_level == 0:
            altitudes_ft = [0, 80000]
            sigmas_ft_sec = [1, 1]
        elif turbulence_level == 1:
            altitudes_ft = [0, 8000, 16000, 80000]
            sigmas_ft_sec = [5, 5, 3, 3]
        elif turbulence_level == 2:
            altitudes_ft = [0, 10000, 44000, 80000]
            sigmas_ft_sec = [10, 10, 3, 3]
        elif turbulence_level == 3:
            altitudes_ft = [2000, 4000, 20000, 80000]
            sigmas_ft_sec = [15, 20, 20, 3]
        sigma_u_ft_sec = np.interp(h_ft, altitudes_ft, sigmas_ft_sec)
        sigma_v_ft_sec = sigma_u_ft_sec
        sigma_w_ft_sec = sigma_u_ft_sec
        sigma_u = sigma_u_ft_sec / m2feet
        sigma_v = sigma_v_ft_sec / m2feet
        sigma_w = sigma_w_ft_sec / m2feet
    else:
        sigma_u_1000_ft, sigma_v_1000_ft, sigma_w_1000_ft = turbulence_std(
            1000, turbulence_level
        )
        sigma_u_2000_ft, sigma_v_2000_ft, sigma_w_2000_ft = turbulence_std(
            2000, turbulence_level
        )
        sigma_u = np.interp(h_ft, [1000, 2000], [sigma_u_1000_ft, sigma_u_2000_ft])
        sigma_v = np.interp(h_ft, [1000, 2000], [sigma_v_1000_ft, sigma_v_2000_ft])
        sigma_w = np.interp(h_ft, [1000, 2000], [sigma_w_1000_ft, sigma_w_2000_ft])
    return sigma_u, sigma_v, sigma_w


def turbulence_scale_length(h_m):
    """
    This function returns the turbulence scale length in the 3 directions, as a
    function of the altitude.
    The formula is taken from the military standard MIL-HDBK-1797A (page 799-810).
    Parameters:
    - h_m : altitude in meters
    Returns :
    - Lu : turbulence scale length in the longitudinal direction, m
    - Lv : turbulence scale length in the lateral direction, m
    - Lw : turbulence scale length in the vertical direction, m
    """
    h_m = max(h_m, 1)  # To avoid division by zero
    m2feet = 3.28084
    h_ft = h_m * m2feet
    if h_ft <= 1000:
        Lu_ft = h_ft / (0.177 + 0.000823 * h_ft) ** 1.2
        Lv_ft = Lu_ft / 2
        Lw_ft = h_ft / 2
        Lu = Lu_ft / m2feet
        Lv = Lv_ft / m2feet
        Lw = Lw_ft / m2feet
    elif h_ft >= 2000:
        Lu = 2500 / m2feet
        Lv = 1250 / m2feet
        Lw = 1250 / m2feet
    else:
        Lu_1000_ft, Lv_1000_ft, Lw_1000_ft = turbulence_scale_length(1000)
        Lu_2000_ft, Lv_2000_ft, Lw_2000_ft = turbulence_scale_length(2000)
        Lu = np.interp(h_ft, [1000, 2000], [Lu_1000_ft, Lu_2000_ft])
        Lv = np.interp(h_ft, [1000, 2000], [Lv_1000_ft, Lv_2000_ft])
        Lw = np.interp(h_ft, [1000, 2000], [Lw_1000_ft, Lw_2000_ft])
    return Lu, Lv, Lw


def turbulence_bandwidth(h_m, v):
    Lu, Lv, Lw = turbulence_scale_length(h_m)
    return 100 * v / max(Lu, Lv, Lw)
