import random

import numpy as np
import pandas as pd

from turbulence_spectra import (
    Hu,
    Hv,
    Hw,
    Hp,
    Hr,
    Hq,
    f_u,
    f_v,
    f_w,
    f_p,
    f_q,
    f_r,
)
from turbulence_parameters import (
    turbulence_std,
    turbulence_scale_length,
    turbulence_bandwidth,
)


def create_turbulence_files(
    instruction_dir_path,
    turbulence_dir_path,
    plane_parameters,
    turbulence_mean_length=180,
    turbulence_part=0.05,
    sampling_frequency=100,
    turbulence_level=2,
):
    """
    Creates turbulence files.
    Files are created in the turbulence_dir_path folder.
    Files are csv files with 8 columns: time, turbulence, u, v, w, p, q, r.
    The turbulence column is binary and indicates whether turbulence is in progress. The
    u, v and w columns are the turbulence translational components. The p, q and r columns are the
    turbulence rotational components.
    Turbulence is generated using the generate_3d_turbulence function.
    The instants in the files are the instants during turbulence,
    sampled at sampling_frequency, as well as a sample before and after
    and after turbulence, with u=v=w=0 and p=q=r=0, to obtain the turbulence wind at any time
    by linear regression.

    Parameters :
    - instruction_dir_path: path to folder containing flight instruction files
    - turbulence_dir_path: path to folder where turbulence files will be saved
    - turbulence_mean_length: average length of turbulence in seconds
    - turbulence_part: proportion of time spent in turbulence
    - sampling_frequency: turbulence sampling frequency
    - turbulence_level: turbulence level during turbulence. Possible turbulence levels
    are 1 (light), 2 (moderate) and 3 (severe).
    """
    file_paths = instruction_dir_path.glob("*.csv")
    for file_path in file_paths:
        turb_file_path = turbulence_dir_path / (file_path.name[:-4] + "_turbwind.csv")
        if not turb_file_path.exists():
            print(f"Creating turbulence file for flight {file_path.name[:-4]}")
            safire_data = pd.read_csv(file_path)
            turbulence_data = create_turbulence_complete_flight(
                safire_data,
                plane_parameters,
                turbulence_mean_length=turbulence_mean_length,
                turbulence_part=turbulence_part,
                sampling_frequency=sampling_frequency,
                turbulence_level=turbulence_level,
            )
            turbulence_data["timestamp"] = turbulence_data.index
            turbulence_data = turbulence_data[
                ["timestamp", "turbulence", "u", "v", "w", "p", "q", "r"]
            ]  # Reorder columns
            turbulence_data.to_csv(turb_file_path, index=False)


def create_calm_conditions_files(
    instruction_dir_path,
    wind_conditions_dir_path,
    plane_parameters,
    parameter_actualisation_period=180,
    sampling_frequency=100,
    turbulence_level=0,
):
    """
    Creates calm condition files.
    Parameters :
    - instruction_dir_path: path to folder containing flight instruction files
    - calm_conditions_dir_path: path to folder where wind files will be saved
    - plane_parameters: dictionary of aircraft parameters to be taken into account for turbulence
    turbulence generation (“b”: aircraft wingspan)
    - parameter_actualization_period: period for updating turbulence parameters
    from aircraft speed and altitude
    - sampling_frequency: turbulence sampling frequency
    - turbulence_level: turbulence level during the complete flight. Possible turbulence levels
    are 0 (calm conditions), 1 (light), 2 (moderate) and 3 (severe).
    """
    file_paths = instruction_dir_path.glob("*.csv")
    for file_path in file_paths:
        conditions_file_path = wind_conditions_dir_path / (
            file_path.name[:-4] + "_calmwind.csv"
        )
        if not conditions_file_path.exists():
            safire_data = pd.read_csv(file_path)
            wind_data = create_calm_conditions_complete_flight(
                safire_data,
                plane_parameters,
                parameter_actualisation_period=parameter_actualisation_period,
                sampling_frequency=sampling_frequency,
                turbulence_level=turbulence_level,
            )
            wind_data["timestamp"] = wind_data.index
            wind_data["turbulence"] = 0
            wind_data = wind_data[
                ["timestamp", "turbulence", "u", "v", "w", "p", "q", "r"]
            ]
            wind_data.to_csv(conditions_file_path, index=False)


def create_turbulence_complete_flight(
    safire_data,
    plane_parameters,
    turbulence_mean_length=180,
    turbulence_part=0.02,
    sampling_frequency=100,
    transition_duration=3,
    turbulence_level=2,
):
    """
    Creates a dataframe containing turbulence.
    The dataframe contains the timestamp, turbulence, u, v, w, p, q and r columns.
    The turbulence column is binary and indicates whether turbulence is occurring.
    The u, v and w columns are the turbulence wind components.
    The p, q and r columns are the fictitious rotation speeds induced by turbulence.
    Turbulence is generated using the generate_3d_turbulence function.
    The function's attributes are as follows:
    - safire_data: dataframe containing flight instructions
    - plane_parameters: dictionary of aircraft parameters to be taken into account for turbulence
    turbulence generation (“b”: aircraft wingspan)
    - turbulence_mean_length: average length of turbulence in seconds
    - turbulence_part: proportion of time spent in turbulence
    - sampling_frequency: turbulence sampling frequency
    - transition_duration: duration of the transition between turbulence and non-turbulence, in
    seconds. The transition is the function f(x)=0.5*(1-cos(pi*x/transition_duration)).
    - turbulence_level: turbulence level during turbulence. Possible turbulence levels
    are 1 (light), 2 (moderate) and 3 (severe).

    Algo principle: create a turbulence dataframe and a non-turbulence dataframe,
    then merge them, taking transitions into account.
    """
    flight_duration = (
        int(safire_data["timestamp"].max()) + 1
    )  # Adding one second to avoid rounding problems at the end of the simulation
    turb_data = pd.DataFrame(
        index=np.around(
            np.linspace(
                0,
                flight_duration - 1 / sampling_frequency,
                flight_duration * sampling_frequency,
            ),
            6,
        ),  # Rounding to the microsecond to avoid index problems related to rounding
        columns=["turbulence", "u", "v", "w", "p", "q", "r"],
        data=0.0,
    )
    no_turb_data = turb_data.copy(deep=True)
    turbulence_start_times, turbulence_durations = generate_turbulence_times(
        flight_duration, turbulence_mean_length, turbulence_part
    )
    no_turb_dfs = list()
    turb_dfs = list()
    v_series = safire_data["platform_speed_wrt_air : from pitot (m/s)"]
    h_series = safire_data["altitude : from GPS (meter)"]
    # Creation of the first non-turbulence period (if necessary)
    if turbulence_start_times[0] != 0:
        first_no_turb = create_turbulence_df(
            v_series.iloc[0 : turbulence_start_times[0]].mean(),
            h_series.iloc[0 : turbulence_start_times[0]].mean(),
            plane_parameters,
            0,
            turbulence_start_times[0] + transition_duration,
            sampling_frequency,
            False,
            turbulence_level=turbulence_level,
        )
        no_turb_data.update(first_no_turb)
        no_turb_dfs.append(first_no_turb)
    # Creation of the turbulence and non-turbulence periods in the middle of the flight
    for i in range(len(turbulence_start_times) - 1):
        turb_df = create_turbulence_df(
            v_series.iloc[
                turbulence_start_times[i] : turbulence_start_times[i]
                + turbulence_durations[i]
            ].mean(),
            h_series.iloc[
                turbulence_start_times[i] : turbulence_start_times[i]
                + turbulence_durations[i]
            ].mean(),
            plane_parameters,
            turbulence_start_times[i],
            turbulence_durations[i],
            sampling_frequency,
            True,
            turbulence_level=turbulence_level,
        )
        turb_data.update(turb_df)
        turb_dfs.append(turb_df)
        v = v_series.iloc[
            turbulence_start_times[i]
            + turbulence_durations[i] : turbulence_start_times[i + 1]
        ].mean()
        h = h_series.iloc[
            turbulence_start_times[i]
            + turbulence_durations[i] : turbulence_start_times[i + 1]
        ].mean()
        after_turbulence_df = create_turbulence_df(
            v,
            h,
            plane_parameters,
            turbulence_start_times[i] + turbulence_durations[i] - transition_duration,
            turbulence_start_times[i + 1]
            - turbulence_start_times[i]
            + 2 * transition_duration,
            sampling_frequency,
            False,
            turbulence_level=turbulence_level,
        )
        no_turb_data.update(after_turbulence_df)
        no_turb_dfs.append(after_turbulence_df)
    # Creation of the last turbulence period and the last non-turbulence period (if necessary)
    turbulence_start_time = turbulence_start_times[-1]
    turbulence_duration = turbulence_durations[-1]
    v = v_series.iloc[
        turbulence_start_time : turbulence_start_time + turbulence_duration
    ].mean()
    h = h_series.iloc[
        turbulence_start_time : turbulence_start_time + turbulence_duration
    ].mean()
    turb_df = create_turbulence_df(
        v,
        h,
        plane_parameters,
        turbulence_start_time,
        turbulence_duration,
        sampling_frequency,
        True,
        turbulence_level=turbulence_level,
    )
    turb_data.update(turb_df)
    turb_dfs.append(turb_df)
    if flight_duration - turbulence_start_time - turbulence_duration != 0:
        v = v_series.iloc[
            turbulence_start_time + turbulence_duration : flight_duration
        ].mean()
        h = h_series.iloc[
            turbulence_start_time + turbulence_duration : flight_duration
        ].mean()
        after_turbulence_df = create_turbulence_df(
            v,
            h,
            plane_parameters,
            turbulence_start_time + turbulence_duration - transition_duration,
            flight_duration
            - turbulence_start_time
            - turbulence_duration
            + transition_duration,
            sampling_frequency,
            False,
            turbulence_level=turbulence_level,
        )
        no_turb_data.update(after_turbulence_df)
        no_turb_dfs.append(after_turbulence_df)
    # Fusion des turbulences et non-turbulences
    df = merge_turbulence_and_noturbulence(
        turb_data,
        no_turb_data,
        turbulence_start_times,
        turbulence_durations,
        transition_duration,
        sampling_frequency,
    )
    return df


def create_calm_conditions_complete_flight(
    safire_data,
    plane_parameters,
    parameter_actualisation_period=180,
    sampling_frequency=100,
    turbulence_level=0,
    transition_duration=3,
):
    """
    Creates a dataframe containing calm conditions.
    The dataframe contains the columns timestamp, u, v, w, p, q and r.
    The u, v and w columns are the turbulence wind components.
    The columns p, q and r are the rotation rates induced by turbulence.
    Calm conditions are generated using the generate_3d_turbulence function.
    The function attributes are as follows:
    - safire_data: dataframe containing flight instructions
    - plane_parameters: dictionary of aircraft parameters to be taken into account for turbulence
    turbulence generation (“b”: aircraft wingspan)
    - parameter_actualization_period: period for updating turbulence parameters
    from aircraft speed and altitude
    - sampling_frequency: turbulence sampling frequency
    - turbulence_level: turbulence level during the complete flight. Possible turbulence levels are 0 (calm conditions), 1 (light), 2 (moderate) and 3 (severe).

    """
    flight_duration = (
        int(safire_data["timestamp"].max()) + 1
    )  # Ajout d'une seconde pour ne pas avoir de problème d'arrondi en fin de simulation
    wind_data = pd.DataFrame(
        index=np.around(
            np.linspace(
                0,
                flight_duration - 1 / sampling_frequency,
                flight_duration * sampling_frequency,
            ),
            6,
        ),  # Arrondi à la microseconde pour éviter les problèmes d'index liés aux arrondis
        columns=["u", "v", "w", "p", "q", "r", "turbulence"],
        data=0.0,
    )
    v_series = safire_data["platform_speed_wrt_air : from pitot (m/s)"]
    h_series = safire_data["altitude : from GPS (meter)"]
    nb_intervals = int(
        (flight_duration - transition_duration) / parameter_actualisation_period
    )
    for i in range(nb_intervals):
        wind_data_i = create_turbulence_df(
            v_series.iloc[
                i
                * parameter_actualisation_period : (i + 1)
                * parameter_actualisation_period
            ].mean(),
            h_series.iloc[
                i
                * parameter_actualisation_period : (i + 1)
                * parameter_actualisation_period
            ].mean(),
            plane_parameters,
            i * parameter_actualisation_period,
            parameter_actualisation_period + transition_duration,
            sampling_frequency,
            False,
            turbulence_level=turbulence_level,
        )
        wind_data_i[
            i * parameter_actualisation_period : i * parameter_actualisation_period
            + transition_duration
        ] *= np.repeat(
            np.linspace(0, 1, transition_duration * sampling_frequency + 1),
            wind_data_i.shape[1],
        ).reshape(
            -1, wind_data_i.shape[1]
        )
        wind_data_i[
            (i + 1) * parameter_actualisation_period
            - transition_duration : (i + 1) * parameter_actualisation_period
        ] *= np.repeat(
            np.linspace(1, 0, transition_duration * sampling_frequency + 1),
            wind_data_i.shape[1],
        ).reshape(
            -1, wind_data_i.shape[1]
        )
        wind_data = wind_data.add(wind_data_i, fill_value=0)
    last_interval = flight_duration - nb_intervals * parameter_actualisation_period
    wind_data_i = create_turbulence_df(
        v_series.iloc[nb_intervals * parameter_actualisation_period :].mean(),
        h_series.iloc[nb_intervals * parameter_actualisation_period :].mean(),
        plane_parameters,
        nb_intervals * parameter_actualisation_period,
        last_interval,
        sampling_frequency,
        False,
        turbulence_level=turbulence_level,
    )
    wind_data_i[
        nb_intervals
        * parameter_actualisation_period : nb_intervals
        * parameter_actualisation_period
        + transition_duration
    ] *= np.repeat(
        np.linspace(0, 1, transition_duration * sampling_frequency + 1),
        wind_data_i.shape[1],
    ).reshape(
        -1, wind_data_i.shape[1]
    )
    wind_data = wind_data.add(wind_data_i, fill_value=0)
    return wind_data


def create_turbulence_df(
    v,
    h,
    plane_parameters,
    turbulence_start_time,
    turbulence_duration,
    sampling_frequency,
    is_turbulence,
    turbulence_level=2,
):
    turbulence_level = turbulence_level if is_turbulence else 0
    sigma_u, sigma_v, sigma_w = turbulence_std(h, turbulence_level)
    Lu, Lv, Lw = turbulence_scale_length(h)
    bandwidth = min(turbulence_bandwidth(h, v), sampling_frequency / 2)
    turbulence_parameters = {
        "V": v,
        "sigma_u": sigma_u,
        "sigma_v": sigma_v,
        "sigma_w": sigma_w,
        "Lu": Lu,
        "Lv": Lv,
        "Lw": Lw,
        "bandwidth": bandwidth,
    }
    turbulence_parameters.update(plane_parameters)
    turbulence_data = generate_3d_turbulence(
        turbulence_duration * sampling_frequency,
        sampling_frequency,
        turbulence_parameters,
    )
    turbulence_df = pd.DataFrame(
        index=np.around(
            np.linspace(
                turbulence_start_time,
                turbulence_start_time + turbulence_duration - 1 / sampling_frequency,
                turbulence_data.shape[0],
            ),
            6,
        ),  # Arrondi à la microseconde pour éviter les problèmes d'index liés aux arrondis
        columns=["u", "v", "w", "p", "q", "r"],
        data=turbulence_data,
    )
    turbulence_df["turbulence"] = 1 if is_turbulence else 0
    return turbulence_df


def generate_turbulence_times(flight_duration, turbulence_mean_length, turbulence_part):
    """
    Generates turbulence start times and durations.
    Turbulence is generated randomly, with an average duration of turbulence_mean_length
    and a proportion of cumulative time in turbulence close to turbulence_part on average.
    The last turbulence can be truncated to respect the proportion of cumulative time in turbulence.
    """
    turbulence_durations = list()
    turbulence_start_times = list()
    total_turbulence_time = 0
    # Choix des durées de turbulences
    nb_turb = random.randint(
        max(int(flight_duration * turbulence_part / turbulence_mean_length) - 1, 1),
        int(flight_duration * turbulence_part / turbulence_mean_length) + 1,
    )
    for i in range(nb_turb):
        new_turb_duration = int(
            turbulence_mean_length + 0.2 * turbulence_mean_length * np.random.randn()
        )
        if new_turb_duration <= 0:
            continue
        turbulence_durations.append(new_turb_duration)
        total_turbulence_time += new_turb_duration

    # Choix des instants de début de turbulences
    ## Le choix est fait de manière à ce que les turbulences ne se chevauchent pas. La manière
    ## de faire est la suivante : on part des instants sans turbulences et on choisi aléatoirement
    ## des instants où intercaler des turbulences.
    no_turb_duration = flight_duration - total_turbulence_time
    turb_insertion_indexes = np.array(
        random.sample(range(no_turb_duration), len(turbulence_durations))
    )
    turb_insertion_indexes = np.sort(turb_insertion_indexes)
    turb_cumsum = np.cumsum(turbulence_durations)
    turb_cumsum = np.insert(turb_cumsum, 0, 0)
    for i, turb_insertion_index in enumerate(turb_insertion_indexes):
        turbulence_start_times.append(turb_insertion_index + turb_cumsum[i])
    return turbulence_start_times, turbulence_durations


def generate_3d_turbulence(N, sampling_frequency, turbulence_parameters):
    """
    Generates turbulence of length N.
    The turbulence is generated by the generate_perfect_spectra_noise function.
    The turbulence parameters (sigma_u, sigma_v, sigma_w, Lu, Lv, Lw and V) and the bandwidth are defined in the turbulence_parameters dictionary.
    Aircraft speed is defined by the V parameter.
    """
    turbulence = np.zeros((N, 6))
    for i, func in enumerate([f_u, f_v, f_w, f_p, f_q, f_r]):
        turbulence[:, i] = generate_perfect_spectra_noise(
            N,
            sampling_frequency,
            turbulence_parameters["bandwidth"],
            func,
            turbulence_parameters,
        )
    return turbulence


def merge_turbulence_and_noturbulence(
    turb_data,
    no_turb_data,
    turbulence_start_times,
    turbulence_durations,
    transition_duration,
    sampling_frequency,
    transition_function="linear",
):
    """
    Merges turbulence and non-turbulence by applying a transition function to each transition from turbulence to non-turbulence and vice versa.
    Parameters :
        - turb_data: dataframe containing turbulence
        - no_turb_data: non-turbulence dataframe
        - turbulence_start_times: list of turbulence start times
        - turbulence_durations: list of turbulence durations
        - transition_duration: duration of the transition between turbulence and non-turbulence, in seconds.
        - sampling_frequency: turbulence sampling frequency
        - transition_function: transition function between turbulence and non-turbulence. The functions are “linear” (f(x)=x/transition_duration) and “cosine” (f(x)=0.5*(1-cos(pi*x/transition_duration)))
    """
    turb_data = turb_data.copy()
    no_turb_data = no_turb_data.copy()
    coefs = (
        turb_data["turbulence"].copy().values
    )  # Coefficient de choix entre turbulences et non-turbulences
    turb_starts = [
        int(t * sampling_frequency) for t in turbulence_start_times if t != 0
    ]
    turb_ends = [
        int((t + d) * sampling_frequency)
        for t, d in zip(turbulence_start_times, turbulence_durations)
        if t + d != turb_data.shape[0] / sampling_frequency
    ]
    if transition_function == "linear":
        transition = np.linspace(0, 1, int(transition_duration * sampling_frequency))
    elif transition_function == "cosine":
        transition = 0.5 * (
            1
            - np.cos(
                np.linspace(0, np.pi, int(transition_duration * sampling_frequency))
            )
        )
    for turb_start in turb_starts:
        coefs[turb_start : turb_start + len(transition)] = transition
    for turb_end in turb_ends:
        coefs[turb_end - len(transition) : turb_end] = transition[::-1]
    data = turb_data[["u", "v", "w", "p", "q", "r"]].values * coefs.reshape(
        -1, 1
    ) + no_turb_data[["u", "v", "w", "p", "q", "r"]].values * (1 - coefs.reshape(-1, 1))
    df = pd.DataFrame(
        index=np.around(
            np.linspace(
                0,
                turb_data.shape[0] / sampling_frequency - 1 / sampling_frequency,
                turb_data.shape[0],
            ),
            6,
        ),
        columns=["u", "v", "w", "p", "q", "r"],
        data=data,
    )
    df["turbulence"] = turb_data["turbulence"]
    return df


def generate_perfect_spectra_noise(
    N, sampling_frequency, bandwidth, filtering_function, filtering_func_params
):
    """
    Generates N noise samples whose spectrum perfectly conforms to the filtering_function filter.
    The spectrum is considered in the Fourier domain.
    Generation is performed as follows:
    - Generation of the spectrum amplitude from the filtering_function formula
    - Generate the spectrum phase randomly between 0 and 2pi
    - Calculation of the inverse Fourier transform
    WARNING: the frequency at N/2 is ignored, I don't know how to handle it.
    Parameters :
    - N: number of samples
    - sampling_frequency: sampling frequency
    - bandwidth
    - filtering_function: filtering function (Hu, Hv or Hw) in the Fourier domain
    - filtering_func_params: dictionary containing filtering_function parameters
    """
    abs_spectrum = np.zeros(N)
    frequencies = np.fft.fftfreq(N, 1 / sampling_frequency)
    abs_spectrum[1 : int(N / 2)] = filtering_function(
        2 * np.pi * frequencies[1 : int(N / 2)], filtering_func_params
    )
    abs_spectrum[int(N / 2) + 1 :] = abs_spectrum[int(N / 2) - 1 : 0 : -1]
    phase_spectrum = np.zeros(N)
    assert bandwidth <= sampling_frequency / 2
    abs_spectrum[
        int(bandwidth * N / sampling_frequency) : -int(
            bandwidth * N / sampling_frequency
        )
    ] = 0
    phase_spectrum[1 : int(N / 2)] = np.random.uniform(0, 2 * np.pi, int(N / 2) - 1)
    phase_spectrum[int(N / 2) + 1 :] = -phase_spectrum[int(N / 2) - 1 : 0 : -1]
    spectrum = abs_spectrum * np.exp(1j * phase_spectrum)
    signal = np.sqrt(N) * np.fft.ifft(spectrum).real
    return signal
