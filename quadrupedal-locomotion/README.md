# Quadrupedal locomotion 

This directory contains an experiment that models quadrupedal locomotion using
both no-slip and purely viscous contact models. The experiment requires
installing [Ravelin](http://github.com/PositronicsLab/Ravelin), [Moby](http://github.com/PositronicsLab/Moby), and [Pacer](http://github.com/PositronicsLab/Pacer). Versions of the software that were used to conduct the experiments 
are included. See
installation pages for both libraries to determine installation requirements,
including architecture and required 3rd party packages. 

# Running the experiments

Assuming that both _Moby_ and _Pacer_ are built, you will run the no-slip 
experiment using the following command:

    $ moby-driver -p=./libLinksPlugin.so MODELS/links-no-slip.xml

You will run the purely viscous experiment using the following command:

    $ moby-driver -p=./libLinksPlugin.so MODELS/links-viscous.xml

The experiments can be timed with the `time` command in Linux. 
Experiments use the default (fast) models in both cases. See below for
testing with experimental controls.

# No-slip modeled as Coulomb friction

Modeling no-slip using Coulomb friction is accomplished by editing the file
**Moby/src/ImpactConstraintHandler.cpp** and changing line 148 from `if (all_inf)`
to `if (false && all_inf)`. Rebuild _Moby_ and re-run experiment to 
establish timings.

# Viscous model without PPM speedup

Modeling viscous friction without the PPM (Principal Pivoting Method) speedup
is accomplished by editing the file **Moby/src/SustainedUnilateralConstraintHandler.cpp** and changing line 109 from `if (all_frictionless)` to `if (false && all_frictionless)`. Rebuild _Moby_ and re-run experiment to establish timings.

