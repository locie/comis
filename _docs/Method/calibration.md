---
title: Calibration
category: Implementation
order: 3
---

## Implementation in Python
<p align ="justify"> As for sensitivity analyses algorithm, CMA-ES algorithm is already implemented in python package <a href="http://deap.readthedocs.io/en/master/">DEAP</a> </p>

```python
#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
@author: wthomare
"""
import buildingspy.simulate.Simulator as si
import numpy  as np
from buildingspy.io.outputfile import Reader
import pandas as pd
import os
import shutil
import copy

# Defining parameters and their intervals
parametres=[]
parametres = {'datSolCol.B0':[-0.5,-0.1],
              'datSolCol.y_intercept':[0.1,1],
              'datSolCol.C1':[1,10],
              'datSolCol.G_nominal':[500,1500],
              'lat2':[0.7,0.9],
              'azi2':[0,3.14],
              'til2':[0,1.57]}

order =['datSolCol.B0','datSolCol.y_intercept','datSolCol.C1','datSolCol.G_nominal','lat2','azi2','til2']

centroide_initial = [0.5] * len(parametres.items())
initial_guess = np.array(centroide_initial)

# Set the flags in the event of an erroneous line in the source file
line     = [] # where is the Error
expected = [] # What was expected
saw      = [] # What was instead
cont     = True

while cont == True:
    try:
        # Specifies the measurement file and the column names to read there
        mesures = pd.read_table(u"MeasurePoint.txt", skiprows=line)
        CPT_tags = u"CPTPRIM" # Column name
        t_tags    = u"t(s)" # Column name
        cont = False

        # The reading of the source file is interrupted to the maximum of the simulation time
        t_mesure    = np.array(mesures[t_tags])
        CPT_mesure = np.column_stack(mesures[CPT_tags])
        TSimulation = 5184000 # Simulation time
        cut = np.flatnonzero(t_mesure == TSimulation)+1
        CPT_mesure = CPT_mesure[:,range(cut)]
        t_mesure  = t_mesure[:cut]
        N = len(t_mesure)
        C = 1

        # Sensor weighting
        resol_CPT  = [0.1] * C
        w_CPT = 1. / np.array(resol_CPT)**2

    except Exception as e: # Processing of reading errors
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

#### Define the model path and translate the initial value
model = u"model.path"
s = si.Simulator(model,u"dymola",u"case", packagePath="//home//wthomare//Desktop//LibraryName")
s.setStopTime(TSimulation)
s.setSolver('dassl')
s.translate()
s1=copy.deepcopy(s)

# Function to set common parameters and to run the simulation
def simulateCase(s):
    ''' Set common parameters and run a simulation.
    :param s: A simulator object.
    '''
    s.setStartTime(t0)
    s.setStopTime(TSimulation)
    s.setTimeOut(600)
    s.showProgressBar(False)
    s.printModelAndTime()
    s.simulate_translated()

def evaluation(individual): ### Function to create and simulate a new individual

### Recreating the parameter values of the individual
	p = [ parametres[order[_]][0] + individual[_]*(parametres[order[_]][1]-parametres[order[_]][0]) for _ in range(len(order))]

	# Simulation with assignment of the parameters of the individual
	s1.setOutputDirectory('//media//wthomare//whereYouWant//ToSave//case %d %d %d %d %d %d %d' % (100*p[0],100*p[1],100*p[2],100*p[3],100*p[4],100*p[5],100*p[6]))
	s1.addParameters({"datSolCol.B0":p[0]})
	s1.addParameters({"datSolCol.y_intercept":p[1]})
	s1.addParameters({"datSolCol.C1":p[2]})
	s1.addParameters({"datSolCol.G_nominal":p[3]})
	s1.addParameters({"lat2":p[4]})
	s1.addParameters({"azi2":p[5]})
	s1.addParameters({"til2":p[6]})

 	simulateCase(s1)

	if not os.path.exists(os.path.join("//media","wthomare","whereYouWant","ToSave",'case %d %d %d %d %d %d %d' % (100*p[0],100*p[1],100*p[2],100*p[3],100*p[4],100*p[5],100*p[6]),u"ValidationPan.mat")):
	 print u"no file"
	 return (np.inf, )

	else:

   # Saving raw output data
	 resultFile = os.path.join("//media","wthomare","whereYouWant","ToSave",'case %d %d %d %d %d %d %d' % (100*p[0],100*p[1],100*p[2],100*p[3],100*p[4],100*p[5],100*p[6]),u"ValidationPan.mat")
	 r=Reader(resultFile, u"dymola")

	 (t2,CPT_Calculer)=r.values("Solaire.y")

 #Saving raw output data
	 CPT_calcul = np.column_stack ((t2,CPT_Calculer))
	 CPT_Init    = np.array([[0,0]])
	 CPT_Init[[0,0]] = CPT_calcul[[0,0]]

	 index=np.zeros((len(CPT_calcul)),dtype='i')

 #The calculation points are filtered at the same sampling as the measurements
	 for i in range(len(CPT_calcul)):
          if CPT_calcul[i][0] in t_mesure:
              index [i]=i

	 CPT_calcul = CPT_calcul[index]
	 CPT_calcul =CPT_calcul[~(CPT_calcul[:,0] == 0)]

 #The calculation points are filtered at the correct sampling but appearing in duplicate

	 CPT_calcul = CPT_calcul[0:len(CPT_calcul):2]
	 CPT_calcul =  np.concatenate((CPT_Init,CPT_calcul), axis=0)

 #Data is presented in the format of the measurement points for the calculation of least squares
	 CPT_calcul = np.delete(CPT_calcul,(0), axis = 1)
	 CPT_calculbis = np.transpose(CPT_calcul)
	 CPT_mesurebis = np.transpose(CPT_mesure)

	# Least squares between measurement and calculation
	 if  (len(CPT_mesurebis) == len(CPT_calcul)) :
          mc_CPT  = 1./N * np.sum( (CPT_mesure  - CPT_calculbis)**2, axis=0 )

	 # Sum of residues with its weighting

          R_CPT  = 1./C * np.sum( w_CPT * mc_CPT )

    # Regularization
          alpha = 0
          normX = np.sum( (np.array(individual)-initial_guess)**2 ) / len(individual)
          shutil.rmtree('//media//wthomare//whereYouWant//ToSave//case %d %d %d %d %d %d %d' % (100*p[0],100*p[1],100*p[2],100*p[3],100*p[4],100*p[5],100*p[6]))
          return (R_CPT + alpha*normX, )

	 else : ### Delete the simulation folder if the simulation fail
          shutil.rmtree('//media//wthomare//whereYouWant//ToSave//case %d %d %d %d %d %d %d' % (100*p[0],100*p[1],100*p[2],100*p[3],100*p[4],100*p[5],100*p[6]))
          return (np.inf, )

#Start of the evolution strategy for model calibration

from deap import creator, base, tools, cma

creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)

toolbox = base.Toolbox()
toolbox.register("evaluate", evaluation)

def main():

    # Maximum number of generation
    NGENMAX = 500
    # Population size by generation
    NPOP = 12

    # Save the strategy
    strategy = cma.Strategy(centroide_initial, sigma=0.3, lambda_=NPOP)
    toolbox.register(u"generate", strategy.generate, creator.Individual)
    toolbox.register(u"update", strategy.update)

    import multiprocessing ### parallelize 4 simulations
    pool = multiprocessing.Pool(processes=4)
    toolbox.register("map", pool.map)

    # Continue on with the evolutionary algorithm
    # Statistiques
    hof   = tools.HallOfFame(1)
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register(u"avg", np.mean, axis=0)
    stats.register(u"std", np.std, axis=0)
    stats.register(u"min", np.min, axis=0)
    stats.register(u"max", np.max, axis=0)
    hof_complet = np.zeros( (NGENMAX,len(centroide_initial)) )
    ecartypes   = np.zeros( (NGENMAX,len(centroide_initial)) )

    # For the analysis of identifiability
    toutlemonde = np.zeros( (NGENMAX, NPOP, len(centroide_initial)) )
    touslesfits = np.zeros( (NGENMAX, NPOP) )

    # Initialization of the logbook
    column_names = [u"gen", u"evals"]
    if stats is not None:
        column_names += stats.functions.keys()
    logbook = tools.Logbook()
    logbook.header = column_names

	 # Algorithm
    STOP = False
    gen = 0
    while not STOP:
        # Generate a new population
        population = toolbox.generate()
        # Deviation Type / mean of each parameter of the population
        foo = np.array(population)
        ecartypes[gen] = np.abs( np.std(foo,axis=0) / np.mean(foo,axis=0) )
        toutlemonde[gen] = foo

        # Evaluate the individuals
        fitnesses = toolbox.map(toolbox.evaluate, population)
        k = 0
        for ind, fit in zip(population, fitnesses):
            ind.fitness.values = fit
            touslesfits[gen, k] = fit[0]
            k += 1

        # Update hall of fame
        hof.update(population)
        hof_complet[gen] = hof.items[0]

        # Update the strategy with the evaluated individuals
        toolbox.update(population)

        # Update statistics
        record = stats.compile(population)
        logbook.record(gen=gen, evals=len(population), **record)
        print(logbook.stream)
        gen +=1

        # Check if it shoud stop
        conv_paras   = np.max(ecartypes[gen-1]) < 1e-3
        conv_fitness = (record[u"std"][-1] / record[u"avg"][-1]) < 1e-5
        if (conv_paras and conv_fitness) or gen >= NGENMAX:
            STOP = True

    return logbook, ecartypes, toutlemonde, touslesfits, hof, hof_complet

if __name__ == "__main__":

    logbook, ecartypes, toutlemonde, touslesfits, hof, hof_complet = main()
    # Formatting the results
    Ecartypes = ecartypes
    toutlemonde_2 = np.column_stack( [np.ravel(toutlemonde[:,:,_]) for _ in range(len(centroide_initial))] )
    OptGlobal = np.zeros_like(centroide_initial)
    OptParGen = np.zeros_like(hof_complet)
    ToutLeMonde = np.zeros_like(toutlemonde_2)
    for i in range(len(centroide_initial)):
        OptGlobal[i]     = parametres[order[i]][0] + hof.items[0][i]   *(parametres[order[i]][1]-parametres[order[i]][0])
        OptParGen[:,i]   = parametres[order[i]][0] + hof_complet[:,i]   *(parametres[order[i]][1]-parametres[order[i]][0])
        ToutLeMonde[:,i] = parametres[order[i]][0] + toutlemonde_2[:,i] *(parametres[order[i]][1]-parametres[order[i]][0])

   # We keep interesting statistics
    sigma_f = np.array( logbook.select(u"std") )
    minimum = np.array( logbook.select(u"min") )
    ngen = len(sigma_f)

    # Saving the best individual and the order of his properties
    np.savetxt(u"CMA_bestfit.txt", OptGlobal)

    # Generational stats: fitnesses and best individual of each generation
    total = np.column_stack((sigma_f, minimum, OptParGen[:ngen], Ecartypes[:ngen]))
    np.savetxt(u"CMA_stats.txt", total)

    # All individuals and their fitness
    TousLesFits = np.ravel(touslesfits)
    np.savetxt(u"CMA_population.txt", np.column_stack((ToutLeMonde, TousLesFits)))
    # For the L curve
    normX = np.sum( (np.array(hof.items[0])-initial_guess)**2 ) / len(hof.items[0])
    print(normX)

### Reload the CMA_population.txt file and compute the 95% interval of each parameter
    mesures = np.loadtxt(os.path.join(basepath,'CMA_population.txt'))

    ### Best individual for Sum(r²) computation
    bestfit = (N*min(mesures[:,len(mesures[1,:])-1]))/100


    Var = []
    ResPartial = np.zeros((len(mesures[1,:])-1,len(mesures)))
    InterV95=np.zeros((2,len(mesures[1,:])-1))

#### Loop for  (Sum(r²))/(N*Var) - Sum(r²Bestfit)
    for i in range(len(mesures[1,:])-1):### For each parameters
        List =[]
        Var.append(np.var(mesures[:,i])) ### Variance of each parameters

        for j in range(len(mesures)): ### For each simulation
            ResPartial[i,j]=((mesures[j,len(mesures[1,:])-1]/100)/(Var[i])) - bestfit ### Compute (r²/(N*Var) - r²Bestfit)

            if ResPartial[i,j]<= (min(ResPartial[i,:])+3.841459): ### Check if the result is into the 95% interval
                List.append(mesures[j,i]) ### sauvegarde de la valeur du paramètre
        if List :### Save the bound of the interval
            InterV95[0,i]= min(List)
            InterV95[1,i]= max(List)
        else : #### Otherwise save the failed of the loop ...
            InterV95[0,i]= np.nan
            InterV95[1,i]= np.nan
    ###Zone where the results are written just for information
        fit = np.full(shape = (1,len(mesures[:,0])),fill_value=min(ResPartial[i,:])+3.841459)
        fig, ax = plt.subplots()
        ax.plot(mesures[:,i], fit[0], color='red')
        ax.scatter(mesures[:,i], ResPartial[i,:])
        ax.set_title('parametre %d ' %(i))
        fig.show()
        plt.savefig(os.path.join(basepath,'parametre_%s.png'%(order[i])))
### Save the bound intervals
    np.savetxt(u"Incertitude_associe.txt", InterV95)
```
