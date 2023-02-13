Simple Locomotion Model
================

<img src="img/simplelocomotionmodel.svg" width="50%" /><br> Notebooks
and code for optimization of a pendulum-like model of locomotion.
Dynamical model performs walking tasks specified by optimal control.
Human experiments to test model are described in publications.

## [Optimization of energy and time predicts dynamic speeds for human walking (Carlisle and Kuo 2023)](src/shortwalks.ipynb)

![short walks](img/shortwalks.svg) This [notebook](src/shortwalks.ipynb)
demonstrates a dynamic optimization for taking short walking bouts of
varying length. The model starts and ends at standing rest, and
self-selects (optimizes) the steps to minimize an energy-time objective.
Energy is expressed as the total push-off work performed by the model,
and time is the total duration of the walk, scaled by a time valuation
constant $c_t$. Human experimental data and code are available in a
[separate repository](https://github.com/kuo-lab/short_walk_experiment).

## [Humans plan for the near future to walk economically on uneven terrain (Darici and Kuo 2022)](src/uneventerrain.ipynb)

![walking on uneven terrain](img/uneventerrainwalking.svg) This
[notebook](src/uneventerrain.ipynb) demonstrates a dynamic optimization
for walking on uneven terrain. The model plans ahead for upcoming steps,
and determines a trajectory of push-offs and forward walking speed that
minimizes the energy and time expended for the terrain. Energy is the
push-off work performed by the model, and the time duration is set to
equal the time to walk the same distance on flat ground. Human
experimental data and code are available in a [separate
repository](https://github.com/kuo-lab/uneventerrainexperiment).

### About the model

The “simplest walking model” (Kuo 2002) models the legs as simple
pendulums. There is a point mass for the body, and infinitesimal point
masses for the feet. For the present publications, walking speed varies
by relatively small amounts, for which modulation of the swing leg is
treated as having low cost. For increasing speeds and step frequencies,
the model’s swing leg cost increases sharply (Kuo 2001).

The optimization is performed withj the [Julia
language](https://julialang.org), a fully open-source language. It uses
open-source packages for optimization ([JuMP](https://jump.dev/) and for
model dynamics [DynLoco](https://github.com/kuo-lab/DynLoco)). The
emphasis here is on simplicity and code readability. The Julia code
provides a minimal demonstration of the mechanics and optimization
approach. The entire tool chain is open source.

Users are invited to view the notebooks and source code in this
repository. Motivated individuals may also wish to execute and modify
the code themselves, which necessitates installation of Julia.

### Installation

- Install Julia according to the [Getting Started
  guide](https://docs.julialang.org/en/v1/manual/getting-started/)
- The Jupyter notebooks also require installation of
  [Jupyter](https://jupyter.org/). Assuming a
  [Python](https://www.python.org/) installation, Jupyter may be
  installed with [`pip`](https://jupyter.org/install) or
  [`conda`](https://anaconda.org/main/jupyter).
- Run Julia from the `simplelocomotionmodel` directory
- Install packages using package manager, from the Julia prompt:
  - `Using Pkg`
  - `Pkg.activate(".")` activates an environment with relevant packages.
    The environment is described by the `.toml` files in the current
    directory (i.e. `simplelocomotionmodel`) and lists the relevant
    packages. This includes the dynamic walking model
    [DynLoco](https://github.com/kuo-lab/DynLoco).
  - `Pkg.instantiate()` downloads and installs packages (can take
    several minutes)
- The Jupyter notebooks have file extension `ipynb` and can be opened
  and executed with a variety of viewers. A recommended workflow uses
  the [VS Code](https://code.visualstudio.com/) editing environment,
  which can open, execute, and render Jupyter notebooks.

### Dependencies

The model equations are implemented in the
[DynLoco](https://github.com/kuo-lab/DynLoco) package with Julia. This
package is automatically installed with the present repository’s
environment (see `instantiate` above). All other dependencies are within
the Julia packages and managed with the environment, so that no
additional management is necessary. Jupyter notebooks are generated with
[Quarto](https://quarto.org), with source provided in `.qmd` files.
Quarto is not necessary to open and interact with Jupyter notebooks.

### References

<div id="refs" class="references csl-bib-body hanging-indent">

<div id="ref-carlisle2023OptimizationEnergyTime" class="csl-entry">

Carlisle, Rebecca Elizabeth, and Arthur D Kuo. 2023. “Optimization of
Energy and Time Predicts Dynamic Speeds for Human Walking.” Edited by
Gordon J Berman. *eLife* 12 (February): e81939.
<https://doi.org/10.7554/eLife.81939>.

</div>

<div id="ref-darici2022HumansPlanFuture" class="csl-entry">

Darici, Osman, and Arthur D. Kuo. 2022. “Humans Plan for the Near Future
to Walk Economically on Uneven Terrain.” *arXiv* 2207.11224 (July).
<https://doi.org/10.48550/arXiv.2207.11224>.

</div>

<div id="ref-kuo2001SimpleModelBipedala" class="csl-entry">

Kuo, Arthur D. 2001. “[A Simple Model of Bipedal Walking Predicts the
Preferred Speed-Step Length
Relationship](https://www.ncbi.nlm.nih.gov/pubmed/11476370).” *Journal
of Biomechanical Engineering* 123 (3): 264–69.

</div>

<div id="ref-kuo2002EnergeticsActivelyPowereda" class="csl-entry">

———. 2002. “[Energetics of Actively Powered Locomotion Using the
Simplest Walking Model](https://www.ncbi.nlm.nih.gov/pubmed/11871597).”
*Journal of Biomechanical Engineering* 124 (1): 113–20.

</div>

</div>

### Issues and pull requests

Please file an
[issue](https://github.com/kuo-lab/simplelocomotionmodel/issues) for
bugs or problems. Pull requests are welcomed.
