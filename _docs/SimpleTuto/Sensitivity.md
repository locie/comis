---
title: Sensitivity analyses
category: First Tutorial
order: 2
---

## Parameters listing

<p align="justify"> To proceed to the Sensitivity analyse, we should define the initial list of parameters </p>

<table BORDER="1">
  <CAPTION> Wood stove parameters </CAPTION>
 <TR>
  <TH> Parameters name </TH>
  <TH> Nominal Value </TH>
  <TH> Standart deviation </TH>
 </TR>

 <TR>
  <TH> Nominal Heating Power </TH>
  <TD> 6000 W </TD>
  <TD> 5% </TD>
 </TR>

 <TR>
  <TH> Nominal Temperature </TH>
  <TD> 80°C </TD>
  <TD> 5°C </TD>
 </TR>

 <TR>
  <TH> Nominal efficiency </TH>
  <TD> 90% </TD>
  <TD> 5% </TD>
 </TR>

 <TR>
  <TH> Nominal pressure drop </TH>
  <TD> 6000 </TD>
  <TD> 10% </TD>
 </TR>

 <TR>
  <TH> Water volume </TH>
  <TD> 0.009 m3 </TD>
  <TD> 3% </TD>
 </TR>

 <TR>
  <TH> Mass of boiler </TH>
  <TD> 9kg </TD>
  <TD> 3% </TD>
 </TR>

 <TR>
  <TH> LHV of the wood </TH>
  <TD> 5 kWh/kg </TD>
  <TD> 5% </TD>
 </TR>

 <TR>
  <TH> Density of th fuel </TH>
  <TD> 1000 kg </TD>
  <TD> 10% </TD>
 </TR>

 <TR>
  <TH> CO2 emisson at combustion </TH>
  <TD> 0.007 kgCO2/kgFuel </TD>
  <TD> 15% </TD>
 </TR>
</table>


<table BORDER="1">
 <CAPTION> Boiler pump and mixing circuit parameters </CAPTION>
 <TR>
  <TH> Parameters name </TH>
  <TH> Nominal Value </TH>
  <TH> Standart deviation </TH>
 </TR>

 <TR>
  <TH> Thermal conductivity ( lambda/thickness) </TH>
  <TD> 0.04 / 4 cm </TD>
  <TD> 7% </TD>
 </TR>

 <TR>
  <TH> Pipe length </TH>
  <TD> 5 m </TD>
  <TD> 3% </TD>
 </TR>

 <TR>
  <TH> Nominal pressure drop </TH>
  <TD> 3000 Pa </TD>
  <TD> 10% </TD>
 </TR>

 <TR>
  <TH> KV of the 3-Way valve </TH>
  <TD> 50 (m3/h.Bar)^(1/2) </TD>
  <TD> 20% </TD>
 </TR>
</table>

<table BORDER="1">
 <CAPTION> Hot water Tank parameters </CAPTION>
 <TR>
  <TH> Parameters name </TH>
  <TH> Nominal Value </TH>
  <TH> Standart deviation </TH>
 </TR>

 <TR>
  <TH> Volume of the Tank </TH>
  <TD> 3 m3 </TD>
  <TD> 5% </TD>
 </TR>

 <TR>
  <TH> height of the Tank </TH>
  <TD> 2 m </TD>
  <TD> 3% </TD>
 </TR>

 <TR>
  <TH> Insulation thickness </TH>
  <TD> 25 cm </TD>
  <TD> 7% </TD>
 </TR>

 <TR>
  <TH> Insulation Heat conductivity </TH>
  <TD> 0.04 W/(m.K) </TD>
  <TD> 10% </TD>
 </TR>
</table>

<table BORDER="1">
 <CAPTION> Control and Monitoring control </CAPTION>
 <TR>
  <TH> Parameters name </TH>
  <TH> Significant </TH>
  <TH> Standart deviation </TH>
 </TR>

 <TR>
  <TH> Minimal temperature at the bottom of the tank  </TH>
  <TD> 50°C </TD>
  <TD> 2°C</TD>
 </TR>

 <TR>
  <TH> Threshold to switch boiler off </TH>
  <TD> 1°C </TD>
  <TD> 1°C </TD>
 </TR>

 <TR>
  <TH> Pump delay  </TH>
  <TD> 60s </TD>
  <TD> 5% </TD>
 </TR>

 <TR>
  <TH> Proportionnal gain of PID  </TH>
  <TD> 0.1 </TD>
  <TD> 10% </TD>
 </TR>

 <TR>
  <TH> Integral gain of PID  </TH>
  <TD> 120 </TD>
  <TD> 10% </TD>
 </TR>


 <TR>
  <TH> Minimal return temperature to the wood stove  </TH>
  <TD> 60°C </TD>
  <TD> 2°C </TD>
 </TR>

 <TR>
  <TH>Test to block boiler if room air temperature is sufficiently high</TH>
  <TD> 16°C </TD>
  <TD> 1°C </TD>
 </TR>
</table>

<p align = "justify"> Only sensitivity analyzes are performed in this part of the tutorial. The results of the various sensitivity analyzes (component and system) will therefore be presented. In practice, the process can start with a component and continue until it is calibrated and then start again with another component. We will try here to be methodical and therefore not to use this way of doing.</p>

## Sensitivity of the boiler

<p align = "justify"> The following script was applied to obtain the weight of the table "Wood stove parameters </p>

```python
# -*- coding: utf-8 -*-
"""
@author: wthomare
"""
import buildingspy.simulate.Simulator as si
from SALib.sample import saltelli
from SALib.analyze import sobol
import os
from buildingspy.io.outputfile import Reader
import numpy as np
import shutil
from multiprocessing import Pool
import copy

TSimulation = 604800

model = u"FBM.Tutorial.FirstTuto"
s = si.Simulator(model,u"dymola",u"case", packagePath="//media//wthomare//DONNEES//comis")
s.setStopTime(TSimulation)
s.setSolver('dassl')
s.translate()
s1=copy.deepcopy(s)


problem = {
    'num_vars': 9,
    'names': [
    'boi.Q_flow_nominal',
    "boiler.T_nominal",
    "boiler.a",
    "boiler.dp_nominal",
    "boiler.VWat",
    "boiler.mDry",
    "datFue.h",
    "datFue.d",
    "datFue.mCO2"],
    'bounds': [[5700,6300],
               [75,85],
               [85,95],
               [5000,7000],
               [0.0873,0.0927],
               [8.73,9.27],
               [17100,18900],
               [900,1100],
               [0.00608,0.00805]]}

W = saltelli.sample(problem,100, calc_second_order=True)
po = Pool(processes = 4)

def SaveResult(path,texte):
        fichier = open("outputsMorris.txt","a")
        fichier.write(texte + "\n")
        fichier.close()

# Function to set common parameters and to run the simulation
def simulateCase(s):

    s.setStopTime(TSimulation)
    # Kill the process if it does not finish in 10 minute
    s.setTimeOut(600)
    s.showProgressBar(False)
    s.printModelAndTime()
    s.simulate_translated()


def evaluate(i):
    s1.setOutputDirectory('//media//wthomare//DONNEES//FyPy//case %d' % (i))

    s1.addParameters({"boiler.Q_flow_nominal' :W[i,0]})
    s1.addParameters({"boiler.T_nominal" :W[i,1]})
    s1.addParameters({"boiler.a" :W[i,2]})
    s1.addParameters({"boiler.dp_nominal" :W[i,3]})
    s1.addParameters({"boiler.VWat" :W[i,4]})
    s1.addParameters({"boiler.mDry" :W[i,5]})
    s1.addParameters({"datFue.h" :W[i,6]})
    s1.addParameters({"datFue.d" :W[i,7]})
    s1.addParameters({"datFue.mCO2" :W[i,8]})

    simulateCase(s1)

# Main function
if __name__ == '__main__':
   po.map(evaluate,range(len(W)) )
   for i,X in enumerate(W):
        resultFile = os.path.join("//media","wthomare","DONNEES","FyPy",'case %d' % (i),u"ValidationPan.mat")
        outTemp=Reader(resultFile,"dymola")
        SaveResult("//home//wthomare//Bureau//PyWorDir//SaveOutputFile//Boiler",str(outTemp.integral('Solaire.y')))
        shutil.rmtree('//media//wthomare//DONNEES//FyPy//case %d' % (i))

   Y = np.loadtxt("outputsMorris.txt", float)
   Si = sobol.analyze(problem, Y, print_to_console=False)
   shutil.rmtree('case')
   print Si['S1']
 ```

## Results :

<p align="justify"> Tables resume wich parameters was significant on our output for each component then for the system </p>

<table BORDER="1">
 <CAPTION> Wood stove parameters </CAPTION>
 <TR>
  <TH> Parameters name </TH>
  <TH> Significant </TH>
 </TR>

  <TR>
   <TH> Nominal Heating Power </TH>
   <TD> Yes </TD>
  </TR>

  <TR>
   <TH> Nominal Temperature </TH>
   <TD> No </TD>

  </TR>

  <TR>
   <TH> Nominal efficiency </TH>
   <TD> Yes </TD>
  </TR>

  <TR>
   <TH> Nominal pressure drop </TH>
   <TD> No </TD>
  </TR>

  <TR>
   <TH> Water volume </TH>
   <TD> No </TD>
  </TR>

  <TR>
   <TH> Mass of boiler </TH>
   <TD> No </TD>
  </TR>

  <TR>
   <TH> LHV of the wood </TH>
   <TD> Yes </TD>
  </TR>

  <TR>
   <TH> Density of th fuel </TH>
   <TD> No </TD>
  </TR>

  <TR>
   <TH> CO2 emisson at combustion </TH>
   <TD> No </TD>
  </TR>

</table>

<table BORDER="1">
 <CAPTION> Boiler pump and mixing circuit parameters </CAPTION>
 <TR>
  <TH> Parameters name </TH>
  <TH> Significant </TH>
 </TR>

 <TR>
  <TH> Thermal conductivity ( lambda/thickness) </TH>
  <TD> Yes </TD>
 </TR>

 <TR>
  <TH> Pipe length </TH>
  <TD> No </TD>
 </TR>

 <TR>
  <TH> Nominal pressure drop </TH>
  <TD> No </TD>
 </TR>

 <TR>
  <TH> KV of the 3-Way valve </TH>
  <TD> Yes </TD>
 </TR>
</table>

<table BORDER="1">
 <CAPTION> Hot water Tank parameters </CAPTION>
 <TR>
  <TH> Parameters name </TH>
  <TH> Significant </TH>
 </TR>

 <TR>
  <TH> Volume of the Tank </TH>
  <TD> No </TD>
 </TR>

 <TR>
  <TH> height of the Tank </TH>
  <TD> No </TD>
 </TR>

 <TR>
  <TH> Thermal conductivity ( lambda/thickness) </TH>
  <TD> Yes </TD>
 </TR>
</table>

<table BORDER="1">
 <CAPTION> Control and Monitoring control </CAPTION>
 <TR>
  <TH> Parameters name </TH>
  <TH> Significant </TH>
 </TR>

 <TR>
  <TH> Minimal temperature at the bottom of the tank  </TH>
  <TD> Yes </TD>
 </TR>

 <TR>
  <TH> Threshold to switch boiler off </TH>
  <TD> No </TD>
 </TR>

 <TR>
  <TH> Pump delay  </TH>
  <TD> No </TD>
 </TR>

 <TR>
  <TH> Proportionnal gain of PID  </TH>
  <TD> No </TD>
 </TR>

 <TR>
  <TH> Integral gain of PID  </TH>
  <TD> No </TD>
 </TR>


 <TR>
  <TH> Minimal return temperature to the wood stove  </TH>
  <TD> Yes </TD>
 </TR>

 <TR>
  <TH>Test to block boiler if room air temperature is sufficiently high</TH>
  <TD> Yes</TD>
 </TR>
</table>
