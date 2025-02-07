# Aircraft simulation with icing and turbulence

Provide simulated flight data, incorporating icing and turbulence. It is based on Robert F. Stengel 6DOF simulation engine, with an addition of control, icing simulation, turbulence and intermediate state of the landing gear and flaps. The original code from Robert F. Stengel and its description can be found [here](https://stengel.mycpanel.princeton.edu/FDcodeB.html)

The purpose of this code is to provide simulated flight data for anomaly detection, but other applications are possible.

An article describing how this simulator works and the data it generates is currently being written. A link to this article will be made available as soon as it is published.

## Usage

The generation of simulated data is made using the file "file_generation.m". A flight plan, a turbulence file and and icing condition file must be provided. The flight plan file has to be a CSV file containing the following columns : time, target route angle, target altitude, target airspeed. The turbulence file has to be a CSV file with the following columns : time, presence of turbulence, u, v, w, p, q, r. (u, v, w) and (p, q, r) are respectively the translational and rotational perturbations applied to the aircraft. The icing condition file has to be a CSV file with columns representing time and the presence of icing conditions (0 or 1).

New turbulence files can be generated using  turbulence_generation/turbulence_generation.ipynb. The turbulence are generated using the Von Karman model.

Example data is provided for a 5 minute simulation. Other flight plans can be generated from real flights (example Safire+, link [here](https://safireplus.aeris-data.fr/)) or generated from scratch.

The data can be explored using the jupyter notebook Simulation_exploration/sim_explo.ipynb.

## Dependencies

The main script is runing on Matlab. It uses the Parallel Computing Toolbox. A usage without this toolbox is possible with minor modifications in "file_generation.m"

The turbulence generation notebook is run using python 3.11. The libraries needed are Pandas, Numpy, Matplotlib and Plotly

## Contributions

The original version of the simulation engine was created by Robert F. Stengel. It was modified by Charles Dampeyrou in 2024 in order to incorporate control of the aircraft, turbulence, icing and intermediate positions of the landing gear and flaps.

## Contact information

Charles Dampeyrou : charles.dampeyrou@gmail.com