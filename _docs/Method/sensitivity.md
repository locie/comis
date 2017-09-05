---
title: Sensitivity
category: Implementation
order: 2
---
<p align="justify"> Sensitivity analysis analyze a mathematical model by studying the impact of the variability of model input factors on the output variable. Determining the inputs responsible for this variability with sensitivity indices, sensitivity analysis allows to reduce the variance of output if it is synonymous with vagueness, or ease the model by setting the entries whose variability does not influence the output variable.
inputs. The main kind of sensitivity analysis are : </p>


- Morris method
- Sobol method
- Fast Method

### Morris method
<p align="justify"> The method starts by sampling a set of start values within the defined ranges of possible values for all input variables and calculating the subsequent model outcome. The second step changes the values for one variable (all other inputs remaining at their start values) and calculates the resulting change in model outcome compared to the first run. Next, the values for another variable are changed (the previous variable is kept at its changed value and all other ones kept at their start values) and the resulting change in model outcome compared to the second run is calculated. This goes on until all input variables are changed. This procedure is repeated r times (where r is usually taken between 5 and 15), each time with a different set of start values, which leads to a number of <strong>r(k + 1) runs</strong>, where k is the number of input variables. Such number is very efficient compared to more demanding methods for sensitivity analysis</p>

 <p align="justify">A sensitivity analysis method widely used to screen factors in models of large dimensionality is the design proposed by Morris. <strong>The Morris method deals efficiently with models containing hundreds of input factors without relying on strict assumptions about the model </strong>, such as for instance additivity or monotonicity of the model input-output relationship. The Morris method is simple to understand and implement, and its results are easily interpreted. Furthermore, it is economic in the sense that it requires a number of model evaluations that is linear in the number of model factors. The method can be regarded as global as the final measure is obtained by averaging a number of local measures (the elementary effects), computed at different points of the input space. For these qualities , <strong> this method is that retained by default during the realization of our process</strong></p>


### Sobol method

<p align="justify">The Sobol method is based on sampling. To determine the sensitivity index of parameter Xi, a k × r matrix is created (k is the number of parameters taken into account in the analysis and r is the sample size), the parameter values are generated using Monte Carlo methods, y0 is then the response of this first sampling. Then a sampling is created per parameter and the value of all the parameters is modified except for the parameter Xi, yi corresponds to the response of each sampling. The sensitivity index can then be calculated. The number of simulations for defining first order indices is equal to <strong>r(k + 1)</strong> like for Morris method. The calculation of the total indices adds N simulations if the first order indices have been calculated and the total number of simulations is <strong>r(k + 2)</strong></p>

<p align="justify"> Due to the ability to calculate the total order indices, the Sobol method can be used in the event of failure of a first commission attempt. Indeed, if the parameters are correlated, the index of order two we tell.</p>

### FAST method

<p align="justify">Fourier Amplitude Sensitivity Testing (FAST) is a variance-based global sensitivity analysis method. The sensitivity value is defined based on conditional variances which indicate the individual or joint effects of the uncertain inputs on the output.</p>

<p align="justify">FAST first represents conditional variances via coefficients from the multiple Fourier series expansion of the output function. Then the ergodic theorem is applied to transform the multi-dimensional integral to a one-dimensional integral in evaluation of the Fourier coefficients. A set of incommensurate frequencies is required to perform the transform and most frequencies are irrational. To facilitate computation a set of integer frequencies is selected instead of the irrational frequencies. The integer frequencies are not strictly incommensurate, resulting in an error between the multi-dimensional integral and the transformed one-dimensional integral. However, the integer frequencies can be selected to be incommensurate to any order so that the error can be controlled meeting any precision requirement in theory. Using integer frequencies in the integral transform, the resulted function in the one-dimensional integral is periodic and the integral only needs to evaluate in a single period. Next, since the continuous integral function can be recovered from a set of finite sampling points if the Nyquist–Shannon sampling theorem is satisfied, the one-dimensional integral is evaluated from the summation of function values at the generated sampling points.</p>

<p align="justify">FAST is more efficient to calculate sensitivities than other variance-based global sensitivity analysis methods via Monte Carlo integration. However the calculation by FAST is usually limited to sensitivities referring to “main effect” or “total effect”. For these reason, the method was never used in our processus.</p>

## Implementation in Python

<p align ="justify">
The three sensitivity analysis methods cited are already implemented in the SALib package. Here we present an example of coupling SALib and Buildingspy for the realization of an analysis with Dymola. You can re-use this code by adapting it to your model. This example is done using the Sobol method, but it is easily adaptable to Morris by consulting the <a href="http://salib.readthedocs.io/en/latest/">SALib</a> website.
</p>

```python
# -*- coding: utf-8 -*-
"""
@author: wthomare
"""
#### Import all package required for the script
import buildingspy.simulate.Simulator as si
from SALib.sample import saltelli
from SALib.analyze import sobol
import os
from buildingspy.io.outputfile import Reader
import numpy as np
import shutil
from multiprocessing import Pool
import copy

TSimulation = 5184000 # Fixed the time of simulation

#### Define the model path and translate the initial value
model = u"model.path"
s = si.Simulator(model,u"dymola",u"case", packagePath="//home//wthomare//Desktop//LibraryName")
s.setStopTime(TSimulation)
s.setSolver('dassl')
s.translate()
s1=copy.deepcopy(s)

### Define parameters of the models with their mean values
problem = {
    'num_vars': 10,
    'names': [
    'datSolCol.B0',
    "datSolCol.B1",
    "datSolCol.y_intercept",
    "datSolCol.C1",
    "datSolCol.C2",
    "datSolCol.G_nominal",
    "datSolCol.dT_nominal",
    "lat",
    "azi",
    "til"],
    'bounds': [
               [-0.5,-0.1],
               [0.005,0.03],
               [0.1,1],
               [1,10],
               [0.01,0.1],
               [500,1500],
               [5,25],
               [0.7,0.9],
               [0,3.14],
               [0,1.57]]}

### Realize the sampling
W = saltelli.sample(problem,1000, calc_second_order=True)

### Define the number of simulation that you want parallelize
po = Pool(processes = 4)

def SaveResult(path,texte): ### Function to write the output value
        fichier = open("outputsMorris.txt","a")
        fichier.write(texte + "\n")
        fichier.close()

# Function to set common parameters and to run the simulation
def simulateCase(s):
    ''' Set common parameters and run a simulation.
    :param s: A simulator object.
    '''
    s.setStopTime(TSimulation)
    s.setTimeOut(600)
    s.showProgressBar(False)
    s.printModelAndTime()
    s.simulate_translated()

def evaluate(i): ### Function to create and simulate a new individual
    ''' Main method that configures and runs all simulations
    '''
    s1.setOutputDirectory('//media//wthomare//whereYouWant//ToSave//case %d' % (i))

    s1.addParameters({'datSolCol.B0' :W[i,0]})
    s1.addParameters({"datSolCol.B1" :W[i,1]})
    s1.addParameters({"datSolCol.y_intercept" :W[i,2]})
    s1.addParameters({"datSolCol.C1" :W[i,3]})
    s1.addParameters({"datSolCol.C2" :W[i,4]})
    s1.addParameters({"datSolCol.G_nominal" :W[i,5]})
    s1.addParameters({"datSolCol.dT_nominal" :W[i,6]})
    s1.addParameters({"lat" :W[i,7]})
    s1.addParameters({"azi" :W[i,8]})
    s1.addParameters({"til" :W[i,9]})

    simulateCase(s1)

if __name__ == '__main__':
  ### parallelize all simulations
   po.map(evaluate,range(len(W)))

  ### Research the output value for each simulation and write the result in a .txt file
   for i,X in enumerate(W):
        resultFile = os.path.join("//media","wthomare","whereYouWantS","ToSave",'case %d' % (i),u"ValidationPan.mat")
        outTemp=Reader(resultFile,"dymola")
        SaveResult("//home//wthomare//Bureau//PyWorDir//SaveOutputFile//NameModel",str(outTemp.integral('Solaire.y')))
        shutil.rmtree('//media//wthomare//whereYouWant//ToSave//case %d' % (i))

### Proceed to the Sensitivity analyze and print the result
   Y = np.loadtxt("outputsMorris.txt", float)
   Si = sobol.analyze(problem, Y, print_to_console=False)
   shutil.rmtree('case')
   print Si['S1']
```
