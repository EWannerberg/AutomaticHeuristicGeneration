# Automatic Heuristic Generation for Optimal Control
_Master Thesis by Erik Wannerberg_

This repository containts the source code used in my Master's thesis, __Automatic Heuristic Generation for Optimal Control__.
Feel free to browse, download, test, and modify at your heart's desire.

The code is licensed under the [MIT License](https://en.wikipedia.org/wiki/MIT_License), which allows precisely what is suggested above, 
with the only requirement that you take the license file with you if you ship it somewhere else.

The thesis itself can be found on the [GitHub wiki](https://github.com/EWannerberg/AutomaticHeuristicGeneration/wiki) (direct link to thesis [here](../../wiki/EWanThesisAug2017.pdf)).

The code is split up into four parts:

### Model predictive control
The `ModelPredictiveControl` folder contains a `python` framework for 
[nonlinear Model Predictive Control](https://en.wikipedia.org/wiki/Model_predictive_control#Nonlinear_MPC) (MPC) using the `python` optimization
framework [`pyomo`](http://www.pyomo.org/). It has the following dependencies:
* `python3`
* `pyomo` itself, provided through `pip`
* `numpy`
* `scipy`
* [`Ipopt`](https://projects.coin-or.org/Ipopt) (optimization solver). Source code and binaries are available at their homepage. 

The MPC implementation with a model of a car on a hilly road is run by invoking `python` with `run.py` followed by the path to a `JSON` 
configuration file, of which an example is available at `example/test.json`. The path to the `ipopt` solver binary is specified there.

### Diffusion maps with closed observables
The `DiffusionMaps&ClosedObservables` folder contains mainly [`MATLAB`](https://www.mathworks.com/products/matlab.html) code for
[nonlinear dimensionality reduction](https://en.wikipedia.org/wiki/Dimensionality_reduction) using [diffusion maps](https://en.wikipedia.org/wiki/Diffusion_map).
The code is heavily inspired from Felix Dietrich's approach of *[closed observables](https://arxiv.org/pdf/1506.04793.pdf)*, and attempts to produce an accurate model of 
the dynamics of the controlled MPC variables.

It of course requires `MATLAB`, and uses the [MATLAB Statistics and Control Toolbox](https://www.mathworks.com/products/statistics.html) for a convenient pairwise distance
function. It is possible to encode this function in a few lines, so feel free to do so if you do not have this toolbox available.

The few lines of `python` code in there for data pruning have the same requirements as above, except for `pyomo` and `ipopt`.

A diffusion maps example can be run by running the `dmapsTesting` script, which uses an output file generated from a scenario with a height profile generated by 3 sinusoidal curves 
different amplitudes and frequencies.

### Validation
The `Validation` folder contains `MATLAB` code for validating results of the previous frameworks, and relies on the function in `runScenario.m` to run a simulation for a
user-specified scenario with a user-provided controller, following the specifications of those provided in the folder. `run_dmaps_testing.m` contains an example script
to construct a controller and run it in the framework. You can try it without modification, but it probably won't work, as the controller relies on varied data
to produce sensible results.

### Height data retrieval
For acquiring real height data, the `HeightDataRetrieval` contains `python` functions for downloading such using the 
[Google Maps Elevation API](https://developers.google.com/maps/documentation/elevation/start) as well as the 
[Directions API](https://developers.google.com/maps/documentation/directions/start). To use this, you need a 
Google Maps API key, which is available for free, linked at the API documentation. The key should be entered in `get_key.py`, where it should be returned
from the `get_key` function, instead of raising an exception.

The code depends on the [`googlemaps`](https://github.com/googlemaps/google-maps-services-python) module in `python`, which again is available through `pip`.



Any questions or requests for the master thesis are happily accepted. Have fun using!
