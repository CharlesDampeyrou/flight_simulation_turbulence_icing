import random
import os

import numpy as np
import pandas as pd


def create_icing_time_files(
    instruction_dir_path,
    output_dir_path,
    freq=1,
    icing_duration=180,
    nb_icing_events=3,
    max_icing=1,
):
    """
    Create the files indicating the times of icing for all instruction files in the instruction directory.
    The times are randomly selected from the range of the instruction file.
    Parameters :
    - instruction_dir_path : path of the directory containing the instruction files
    - output_dir_path : path of the directory where the output files will be saved
    - freq : frequency of the output files in Hz
    - icing_duration : duration of the icing events in seconds
    - nb_icing_events : number of icing events per instruction file
    """
    instruct_files = instruction_dir_path.glob("*.csv")
    for instruct_file in instruct_files:
        df = pd.read_csv(instruct_file)
        flight_duration = df["timestamp"].iloc[-1]
        icing_df = create_icing_df(
            flight_duration, freq, icing_duration, nb_icing_events, max_icing=max_icing
        )
        icing_df.to_csv(
            output_dir_path / f"{instruct_file.stem}_icing.csv", index=False
        )


def create_icing_df(
    flight_duration, freq, icing_duration, nb_icing_events, max_icing=1
):
    """
    Create a DataFrame with the times of icing events.
    Parameters :
    - flight_duration : duration of the flight in seconds
    - freq : frequency of the output files in Hz
    - icing_duration : duration of the icing events in seconds
    - nb_icing_events : number of icing events
    """
    icing_df = pd.DataFrame(
        {"timestamp": np.arange((flight_duration + 2) * freq) / freq}
    )
    icing_df["icing"] = 0

    starting_indexes = [
        random.randint(
            0, flight_duration * freq - nb_icing_events * icing_duration * freq
        )
        for _ in range(nb_icing_events)
    ]
    starting_indexes.sort()
    for i, index in enumerate(starting_indexes):
        start_index = index + i * icing_duration * freq
        end_index = start_index + icing_duration * freq
        icing_df.loc[start_index:end_index, "icing"] = max_icing
    return icing_df
