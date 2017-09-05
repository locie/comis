---
title: Parameters Calibration
category: First Tutorial
order: 4
---
<p align="justify">
As described in the <i>"Method"</i> section, the calibration process is applied with a bottom-up approach. The process start with the calibration of each components then by the calibration of the whole system. The script needed for the system calibration is given below.</p>

```python
#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 12 09:35:40 2017

@author: wthomare
"""
import buildingspy.simulate.Simulator as si
import numpy  as np
from buildingspy.io.outputfile import Reader
import pandas as pd
import os
import shutil
import copy

parametres=[]
parametres = {'kPipe':[0.036,0.044],
              'kTank':[0.036,0.044],
              'lesThrTRoo.threshold':[273.15+16,273.15+20],
              'mixingCircuit_Tset.KvReturn':[40,60],
              'TSup_nominal':[42,47],
              'TRet_Min':[58,62]}

order =['kPipe','kTank','lesThrTRoo.threshold','mixingCircuit_Tset.KvReturn','TSup_nominal','TRet_Min']

centroide_initial = [0.5] * len(parametres.items())
initial_guess = np.array(centroide_initial)

line     = []
expected = []
saw      = []
cont     = True

while cont == True:
    try:
        mesures = pd.read_table(os.path.join("//media","wthomare","DONNEES","comis","FBM","Utilities","Tutorial","FirstTutorial.txt"), skiprows=line)

        CPT_tags = u"integrator"
        t_tags    = u"t(s)"

        cont = False
        t_mesure    = np.array(mesures[t_tags])
        CPT_mesure = np.column_stack(mesures[CPT_tags])

        t0 = 0
        TSimulation = 604800
        cut = np.flatnonzero(t_mesure == TSimulation)+1
        CPT_mesure = CPT_mesure[:,range(cut)]

        t_mesure  = t_mesure[:cut]

        N = len(t_mesure)
        C = 1
        resol_CPT  = [0.1] * C
        w_CPT = 1. / np.array(resol_CPT)**2

    except Exception as e:
        errortype = e.message.split('.')[0].strip()
        if errortype == u"Error tokenizing data":
            cerror      = e.message.split(':')[1].strip().replace(',','')
            nums        = [n for n in cerror.split(' ') if str.isdigit(n)]
            expected.append(int(nums[0]))
            saw.append(int(nums[2]))
            line.append(int(nums[1])-1)
        else:
            cerror      = u"Unknown"
            print u"Unknown Error - 222"

model = u"Comis.Helios.Validation.ValidationPan"
s = si.Simulator(model,u"dymola",u"case", packagePath="//home//wthomare//Bureau//NouveauDossier")
s.setStopTime(TSimulation)
s.setSolver('dassl')
s.translate()
s1=copy.deepcopy(s)

def simulateCase(s):
    s.setStartTime(t0)
    s.setStopTime(TSimulation)
    s.setTimeOut(600)
    s.showProgressBar(False)
    s.printModelAndTime()
    s.simulate_translated()

def evaluation(individual):

	p = [ parametres[order[_]][0] + individual[_]*(parametres[order[_]][1]-parametres[order[_]][0]) for _ in range(len(order))]

	s1.setOutputDirectory('//media//wthomare//DONNEES//FyPy//case %d %d %d %d %d' % (100*p[0],100*p[1],100*p[2],100*p[3],100*p[4]))
	s1.addParameters({"kPipe":p[0]})
	s1.addParameters({"kTank":p[1]})
	s1.addParameters({"lesThrTRoo.threshold":p[2]})
  s1.addParameters({"'mixingCircuit_Tset.KvReturn'":p[3]})
  s1.addParameters({u"TSup_nominal":p[i,4]})
  s1.addParameters({u"TRet_Min":p[i,5]})
 	simulateCase(s1)

	if not os.path.exists(os.path.join("//media","wthomare","DONNEES","FyPy",'case %d %d %d %d %d' % (100*p[0],100*p[1],100*p[2],100*p[3],100*p[4]),u"FirstTuto.mat")):

	 print u"no file"
	 return (np.inf, )

	else:

	 resultFile = os.path.join("//media","wthomare","DONNEES","FyPy",'case %d %d %d %d %d' % (100*p[0],100*p[1],100*p[2],100*p[3],100*p[4]),u"FirstTuto.mat")
	 r=Reader(resultFile, u"dymola")

	 (t2,CPT_Calculer)=r.values("integrator.y")

	 CPT_calcul = np.column_stack ((t2,CPT_Calculer))
	 CPT_Init    = np.array([[0,0]])
	 CPT_Init[[0,0]] = CPT_calcul[[0,0]]

	 index=np.zeros((len(CPT_calcul)),dtype='i')

	 for i in range(len(CPT_calcul)):
          if CPT_calcul[i][0] in t_mesure:
              index [i]=i

	 CPT_calcul = CPT_calcul[index]

	 CPT_calcul =CPT_calcul[~(CPT_calcul[:,0] == 0)]

	 CPT_calcul = CPT_calcul[0:len(CPT_calcul):2]
	 CPT_calcul =  np.concatenate((CPT_Init,CPT_calcul), axis=0)

	 CPT_calcul = np.delete(CPT_calcul,(0), axis = 1)
	 CPT_calculbis = np.transpose(CPT_calcul)

	 CPT_mesurebis = np.transpose(CPT_mesure)

	 if  (len(CPT_mesurebis) == len(CPT_calcul)) :

          mc_CPT  = 1./N * np.sum( (CPT_mesure  - CPT_calculbis)**2, axis=0 )
          R_CPT  = 1./C * np.sum( w_CPT * mc_CPT )

          alpha = 0
          normX = np.sum( (np.array(individual)-initial_guess)**2 ) / len(individual)
          shutil.rmtree('//media//wthomare//DONNEES//FyPy//case %d %d %d %d %d' % (100*p[0],100*p[1],100*p[2],100*p[3],100*p[4]))

          return (R_CPT + alpha*normX, )

	 else :
          shutil.rmtree('//media//wthomare//DONNEES//FyPy//case %d %d %d %d %d' % (100*p[0],100*p[1],100*p[2],100*p[3],100*p[4]))
          return (np.inf, )

from deap import creator, base, tools, cma

creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)

toolbox = base.Toolbox()
toolbox.register("evaluate", evaluation)

def main():

    NGENMAX = 500
    NPOP = 12

    strategy = cma.Strategy(centroide_initial, sigma=0.3, lambda_=NPOP)
    toolbox.register(u"generate", strategy.generate, creator.Individual)
    toolbox.register(u"update", strategy.update)

    import multiprocessing
    pool = multiprocessing.Pool(processes=4)
    toolbox.register("map", pool.map)

    hof   = tools.HallOfFame(1)
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register(u"avg", np.mean, axis=0)
    stats.register(u"std", np.std, axis=0)
    stats.register(u"min", np.min, axis=0)
    stats.register(u"max", np.max, axis=0)
    hof_complet = np.zeros( (NGENMAX,len(centroide_initial)) )
    ecartypes   = np.zeros( (NGENMAX,len(centroide_initial)) )

    toutlemonde = np.zeros( (NGENMAX, NPOP, len(centroide_initial)) )
    touslesfits = np.zeros( (NGENMAX, NPOP) )

    column_names = [u"gen", u"evals"]
    if stats is not None:
        column_names += stats.functions.keys()
    logbook = tools.Logbook()
    logbook.header = column_names

    STOP = False
    gen = 0
    while not STOP:

        population = toolbox.generate()
        foo = np.array(population)
        ecartypes[gen] = np.abs( np.std(foo,axis=0) / np.mean(foo,axis=0) )
        toutlemonde[gen] = foo

        fitnesses = toolbox.map(toolbox.evaluate, population)
        k = 0
        for ind, fit in zip(population, fitnesses):
            ind.fitness.values = fit
            touslesfits[gen, k] = fit[0]
            k += 1

        hof.update(population)
        hof_complet[gen] = hof.items[0]
        toolbox.update(population)
        record = stats.compile(population)
        logbook.record(gen=gen, evals=len(population), **record)
        print(logbook.stream)

        gen +=1
        conv_paras   = np.max(ecartypes[gen-1]) < 1e-3
        conv_fitness = (record[u"std"][-1] / record[u"avg"][-1]) < 1e-5
        if (conv_paras and conv_fitness) or gen >= NGENMAX:
            STOP = True

    return logbook, ecartypes, toutlemonde, touslesfits, hof, hof_complet

if __name__ == "__main__":

    logbook, ecartypes, toutlemonde, touslesfits, hof, hof_complet = main()

    Ecartypes = ecartypes
    toutlemonde_2 = np.column_stack( [np.ravel(toutlemonde[:,:,_]) for _ in range(len(centroide_initial))] )
    OptGlobal = np.zeros_like(centroide_initial)
    OptParGen = np.zeros_like(hof_complet)
    ToutLeMonde = np.zeros_like(toutlemonde_2)
    for i in range(len(centroide_initial)):
        OptGlobal[i]     = parametres[order[i]][0] + hof.items[0][i]   *(parametres[order[i]][1]-parametres[order[i]][0])
        OptParGen[:,i]   = parametres[order[i]][0] + hof_complet[:,i]   *(parametres[order[i]][1]-parametres[order[i]][0])
        ToutLeMonde[:,i] = parametres[order[i]][0] + toutlemonde_2[:,i] *(parametres[order[i]][1]-parametres[order[i]][0])

    sigma_f = np.array( logbook.select(u"std") )
    minimum = np.array( logbook.select(u"min") )
    ngen = len(sigma_f)

    np.savetxt(u"CMA_bestfit.txt", OptGlobal)

    total = np.column_stack((sigma_f, minimum, OptParGen[:ngen], Ecartypes[:ngen]))
    np.savetxt(u"CMA_stats.txt", total)

    TousLesFits = np.ravel(touslesfits)
    np.savetxt(u"CMA_population.txt", np.column_stack((ToutLeMonde, TousLesFits)))
    normX = np.sum( (np.array(hof.items[0])-initial_guess)**2 ) / len(hof.items[0])
    print(normX)
 ```

<p align="justify"> The calibrations give following results :</p>

<table BORDER="1">
 <CAPTION> Control and Monitoring control </CAPTION>
 <TR>
  <TH> Parameters name </TH>
  <TH> Calibrated value </TH>
 </TR>

 <TR>
  <TH> Minimal temperature at the bottom of the tank  </TH>
  <TD> 48,6°C </TD>
 </TR>

 <TR>
  <TH> Minimal return temperature to the wood stove  </TH>
  <TD> 61.3°C </TD>
 </TR>

 <TR>
  <TH>Test to block boiler if room air temperature is sufficiently high</TH>
  <TD> 17.1°C</TD>
 </TR>
</table>

<table BORDER="1">
  <CAPTION> Wood stove parameters </CAPTION>
 <TR>
  <TH> Parameters name </TH>
  <TH> Calibrated value </TH>
 </TR>

 <TR>
  <TH> Nominal Heating Power </TH>
  <TD> 5886 W </TD>
 </TR>

 <TR>
  <TH> Nominal efficiency </TH>
  <TD> 0.86% </TD>
 </TR>

 <TR>
  <TH> LHV of the wood </TH>
  <TD> 4.71 kWh/kg </TD>
 </TR>
</table>

<table BORDER="1">
 <CAPTION> Boiler pump and mixing circuit parameters </CAPTION>
 <TR>
  <TH> Parameters name </TH>
  <TH> Calibrated value </TH>
 </TR>

 <TR>
  <TH> Thermal conductivity ( lambda/thickness) </TH>
  <TD> 0.08 / 4 cm </TD>
 </TR>

 <TR>
  <TH> KV of the 3-Way valve </TH>
  <TD> <strong>82 (m3/h.Bar)^(1/2)</strong> </TD>
 </TR>
</table>

<table BORDER="1">
 <CAPTION> Hot water Tank parameters </CAPTION>
 <TR>
  <TH> Parameters name </TH>
  <TH> Calibrated value </TH>
 </TR>

 <TR>
  <TH> Thermal conductivity ( lambda/thickness) </TH>
  <TD> 0.04 / 0.10 cm </TD>
 </TR>
</table>

# Results :

<p align="justify"> All parameters have change slowly from there nominal values but three are more outstanding :</p>

- The LHV of the Pellet wood
- The KV of the 3 Way valve
- Thermal conductivity ( lambda/thickness) (for the pipes and the storage tank)

<p align="justify"> The KV of the 3 way valve that controled the minimal return temperature to the stoove <strong> as increase of 60%</strong>. This may indicate that the valve controller fails to open/close the valve properly or that the installed model is oversized (or a mix ...). With a heat source wich risk to explose if the return temperature is not hot enough this can lead to a major problem ... The decrease of 6% of the LHV shows a poor stockage condition of the fuel (with an increase of the humidity of the wood) or initial characteristics not corresponding to wood pellet with a DIN + certification</p>

<p align="justify"> Thermal conductivity are also strongly migrated to poorer values in terms of thermal insulation. However, thermal losses are difficult to analyze. Indeed, even if this indicates a poor performance of the systems, losses at these levels are often released within the building. It is, therefore, possible that the performance of the system is affected without the comfort in the building being so.</p>

<img src="WoodCurve.png" alt="Calibration curve" style="width: 500px;" align ="middle"/>

<p align="justify"> However, we can see that the calibration don't give us a perfect adequacy between the measured on the calibrated model. This can be due to a too low number of simulations (500*12 in the previous script), too few parameters taken into account or any other sources of errors exposed in the other sections of the site. This doubt can be lifted by a conclusive validation phase</p>
