#!/bin/bash

cd lattice_random
python3 lattice_metric_experiment.py
cd ..
echo 'Going to 2nd experiment'
cd manual_flocking_gamma_0.8_random
python3 simple_metric_experiment.py
cd ..
echo 'Going to 3rd experiment'
cd flocking_gamma_0.8_random_large
python3 simple_metric_experiment.py
cd..