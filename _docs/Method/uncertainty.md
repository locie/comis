---
title: Error uncertainty
category: Implementation
order: 4
---
<p align="justify">
It is impossible to compare the indicators with the raw values (measurements / simulations). Indeed, even if the two curves are very close, it is unlikely that they correspond perfectly. In addition, if the curves don't fit it doesn't mean that the model need to be calibrated.
</p>
<p align="justify">
Indeed, the uncertainties of measurements and the margins of errors of each influent parameter may be the causes of the initial difference. Moreover, with a confidence interval taking into account those sources of error it can be more easy to compare the curves. Indeed it is more tolerant and pratice to compare an interval that a raw value</p>

<p align="justify">
That's why we have created a script able to give us those intervals
</p>

-  An interval of uncertainties for the indicators from the measurements
- An interval of uncertainties for the indicators from the simualtions

<p align="justify">
The first one should taking into account the measurement's errors and the second the accuracy of definition of each parameter. But the overall process should screen all the search space and required as little as possible of simulations. To have a large overview of different method of uncertainty propagation  you can have a look on (Bontemps 2015). One of the best compromise is sample the search space of parameters and and measurements with the <Strong> Latin Hypercube Sampler </strong> then to compute the differents indicators with each sample. The number of sample required for an errors propagation with the LHS method is around 1.5 times the number of parameters or the number of sensors required to compute the indicators according to the litterature.  
</p>

<p align="justify">
An example of errors propagation in an indicator from the measurement and from the simulations is given below. The LHS sampler is already implemented in the Python package <strong>pyDOE</strong>. The simulation are runed with <strong>Dymola</strong> and the interface between Python and Dymola is done with <strong>Buildingspy</strong>
</p>

```python
#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
@author: wthomare
"""
#### Import all package required for the script
from pyDOE import lhs
import numpy  as np
import csv
from scipy.stats.distributions import norm
import buildingspy.simulate.Simulator as si
from buildingspy.io.outputfile import Reader
import copy
import os
import shutil
import statsmodels.api as sm
import matplotlib.pyplot as pltacf
import matplotlib.pyplot as pltccf
import matplotlib.pyplot as plt
import seaborn as sns
sns.set(style="darkgrid")
import statsmodels.stats.api as sms
import pandas as pd
from multiprocessing import Pool

po = Pool(40)
basepath = os.path.abspath('//your/working/directory')
TSimulation = 3000000

line     = []
expected = []
saw      = []    
cont     = True

# Import the measurement file
mesures = pd.read_table(u"Your_Measurement_File.txt", skiprows=line)
t_tags    = u"t(s)"  # Column name
Cpt_tags = u"CPT_REC"


t_mesure    = np.array(mesures[t_tags])
Cpt_mesure = np.column_stack(mesures[Cpt_tags])

#Stop to import your measurement at the good time step
cut = np.flatnonzero(t_mesure == TSimulation)+1
Cpt_mesure = Cpt_mesure[:,range(cut)]                        
t_mesure  = t_mesure[:cut]

# Translate your model but without simulatation   
model = u"Your.model.in.your.library"
s = si.Simulator(model,u"dymola","case", packagePath="//Where/is/your/library")
s.setStopTime(TSimulation)
s.setSolver(u"dassl")
s.translate()        

means = [120,120,100,0.04,273.15+19,273.15+16,8,19] #Give the normal value of each parameters
stdvs =[100,100,50,0.25,3,3,2,2] #Give the standart deviation (error) association at each parameter

p = lhs(8,samples=12) # Sample each 8 parameters 12 times / return a 8x12 matrix optmize to cover any search space

for i in range(len(means)): #Applied your sampling to a nomal law centered on the mean values and with stdvs for standard deviation
    p[:, i] = norm(loc=means[i], scale=stdvs[i]).ppf(p[:, i])

#Some string for futher used
filename = os.path.join(basepath,"OnGoingResult.csv")
filenorm = os.path.join(basepath,"NormResult.csv")
filepoint = os.path.join(basepath,"Point.csv")
fieldnames = ["Simulation %d" % (j) for j in range(len(p))]
fieldNorm =["Normale"]
fullnames = ["Simulation %d" % (j) for j in range(len(p))]
fullnames.append("Normale")             


def simulateCase(s):
#Simulation function
    s.setStopTime(TSimulation)
    s.showProgressBar(True)
    s.printModelAndTime()
    s.simulate_translated()

def DeltaMesure(Delta,mesure,NSample,alpha,absolue) :
# Function used to propagate measurement errors. We can define an relative error with imput absolue = False

    if absolue == True : #Si l'erreur est donnée en valeur absolue

        Inter = np.zeros((1,len(mesure[0,:])))     
        for i in range(len(mesure[0,:])):
            Inter[0,i] = Delta

# LHS sampling ofr N parameters and NSample sample            
        p = lhs(len(mesure[0,:]), samples=NSample)

# same function as previous
        for i in range(len(mesure[0,:])):
            p[:, i] = norm(loc=mesure[0,i], scale=(Inter[0,i]+0.000000001)).ppf(p[:, i])        

# Confidence interval at "alpha"%             
        Int95 =[]
        for i in range(len(mesure[0,:])):
            Iteration =sms.DescrStatsW(p[:,i]).tconfint_mean(alpha=alpha)
            Int95.append(Iteration)
        Int95 = np.array(Int95)       

        return (Int95)

    else : #if the error is relative
        InterR = np.zeros((1,len(mesure[0,:])))

        for i in range(len(mesure[0,:])):
            InterR[0,i] = Delta*mesure[0,i]

        p = lhs(len(mesure[0,:]), samples=NSample)

        for i in range(len(mesure[0,:])):
            p[:, i] = norm(loc=mesure[0,i], scale=(InterR[0,i]+0.000000001)).ppf(p[:, i])        

        Int95 =[]
        for i in range(len(mesure[0,:])):
            Iteration =sms.DescrStatsW(p[:,i]).tconfint_mean(alpha=alpha)
            Int95.append(Iteration)
        Int95 = np.array(Int95)       

        return (Int95)        

def lectureMat(fichier,output,Simulation_name,Temps,Size_name,row,field):
# Function to open the file of a simulation and tried to filter time step with no equivalence with our measurement time step
    if not os.path.exists(os.path.join("/some","path",fichier,"ModelName.mat")):
        print ("no file")      

    else:

        resultFile = os.path.join("/some","path",fichier,"ModelName.mat")
        r=Reader(resultFile, u"dymola")
        (t,Var_Calculer)=r.values(output)

        Var_Calculer  = np.column_stack ((t,Var_Calculer))
        Var_Filtrer = filtre(Var_Calculer,t_mesure)

        Var_calcul = np.delete(Var_Filtrer,(0), axis = 1)
        Var_calcul = np.reshape(Var_calcul, len(Var_calcul))

        Temps_calcul = np.delete(Var_Filtrer,(1), axis = 1)
        Temps_calcul = np.reshape(Temps_calcul, len(Temps_calcul))

        f, ax = pltccf.subplots()
        CCF = np.correlate(Var_calcul,Var_calcul,'full')
        ax.plot(CCF)
        pltccf.savefig(os.path.join(basepath,'ccf_%s.png'%(Simulation_name)))
        pltccf.close('all')

        fig = pltacf.figure(figsize=(12,8))       
        ax1 = fig.add_subplot(211)
        fig = sm.graphics.tsa.plot_acf(Var_calcul, ax=ax1)       
        pltacf.savefig(os.path.join(basepath,'acf_%s.png' %(Simulation_name)))
        pltacf.close('all')

        with open(os.path.join(basepath,Simulation_name), 'a') as csvfile:
            spamwriter = csv.DictWriter(csvfile,fieldnames=row,delimiter = ",",quotechar='|',lineterminator='\n')
            for j in range(len(Var_calcul)):
                spamwriter.writerow({field: Var_calcul[j]})

        with open(os.path.join(basepath,Temps), 'a') as csvfile:
            spamwriter = csv.DictWriter(csvfile,fieldnames=row ,delimiter = ",",quotechar='|',lineterminator='\n')
            for j in range(len(Temps_calcul)):
                spamwriter.writerow({field: Temps_calcul[j]})

        with open(os.path.join(basepath,Size_name), 'a') as csvfile:
            spamwriter = csv.DictWriter(csvfile,fieldnames=row ,delimiter = ",",quotechar='|',lineterminator='\n')
            for j in range((1)):
                spamwriter.writerow({field: len(Var_calcul)})      

def filtre(nonFiltrer,base) : # Filter function, first argument is the liste to filter, the second the reference. Can be optimised
    Filtrer =[]
    NoDoubleFiltrer =[]
    for i in range(len(base)):
        Filtrer.append(list(filter(lambda x:x[0]==base[i],nonFiltrer)))

    NoDoubleFiltrer.append(Filtrer[0][0])
    for i in range(1,len(base)):
        NoDoubleFiltrer.append(Filtrer[i][0])
    NoDoubleFiltrer = np.asarray(NoDoubleFiltrer)
    return(NoDoubleFiltrer)

def evaluate(i):

	# Difinition of each simulation     
    s1 = copy.deepcopy(s)
    s1.setOutputDirectory(os.path.join(basepath,'case %d' % (i)))
    s1.addParameters({u"kvV3V2":p[i,0]}) #Change the value of some parameters
    s1.addParameters({u"kvV3V3":p[i,1]})
    s1.addParameters({u"kvSun":p[i,2]})
    s1.addParameters({u"kIns":p[i,3]})
    s1.addParameters({u"generalSetpoint.TDay":p[i,4]})
    s1.addParameters({u"generalSetpoint.TNight":p[i,5]})
    s1.addParameters({u"generalSetpoint.tDay":p[i,6]})
    s1.addParameters({u"generalSetpoint.tNight":p[i,7]})


    simulateCase(s1)    

    lectureMat(fichier='case %d' %(i), output=u"ProductionChauffage.y",Simulation_name="Simulation_CPT %d.csv" %(i),Temps="Temps_CPT %d.csv"%(i), Size_name="Size_CPT %d.csv"%(i),row=[fieldnames[i]],field="Simulation %d"%(i))

def Launcher():
# Function to create any temporary TXT file required
    for i in range(len(p)):
        with open(os.path.join(basepath,"Size_CPT %d.csv"%(i)),'w')as csvfile :
            spamwriter =csv.DictWriter(csvfile,fieldnames=[fullnames[i]],delimiter=",",quotechar='|',lineterminator='\n')
            spamwriter.writeheader()

    for i in range(len(p)):
        with open(os.path.join(basepath,"Simulation_CPT %d.csv" %(i)),'w') as csvfile :
            spamwriter = csv.DictWriter(csvfile, fieldnames=[fieldnames[i]],delimiter =",", quotechar ='|',lineterminator='\n')
            spamwriter.writeheader()

    for i in range(len(p)):
        with open(os.path.join(basepath,"Temps_CPT %d.csv"%(i)),'w') as csvfile :
            spamwriter = csv.DictWriter(csvfile,fieldnames=[fieldnames[i]],delimiter =",", quotechar='|',lineterminator= '\n')
            spamwriter.writeheader()        
    po.map(evaluate,range(len(p)))

def Analyze (Titre,Size,SizeN,Simulation,Temps,Normale,TempsN,mesure,Delta,alpha,absolue):
#Function for unccertainty propagation in the simation results           
    Matrice = np.zeros((len(p),len(t_mesure)))
    MatriceTemp = np.zeros((len(p),len(t_mesure)))

    for i in range(len(p)):
        f = open(os.path.join(basepath,Simulation %(i)))
        Data = pd.read_csv(f,sep = "\n")
        for j in range(len(t_mesure)):
                Matrice[i,j] = Data["Simulation %d" %(i)][j]
        f.close()

    for i in range(len(p)):
        f = open(os.path.join(basepath,Temps %(i)))
        Data = pd.read_csv(f,sep = "\n")
        for j in range(len(t_mesure)):
                MatriceTemp[i,j] = Data["Simulation %d" %(i)][j]
        f.close()     

    Int95 = []
    for i in range(len(t_mesure)):
        Iteration =sms.DescrStatsW(Matrice[:,i]).tconfint_mean(alpha=0.05)
        Int95.append(Iteration)

    Int95 = np.array(Int95)

    IntervalMesure = DeltaMesure(Delta=Delta,mesure=mesure,NSample=10,alpha=alpha,absolue=absolue)

    Borne = np.ones(len(t_mesure), dtype=bool)
    CompLen = min(len(t_mesure),len(t_mesure))

    for i in range(CompLen):
       if Int95[i,0] <= IntervalMesure[i,0] and Int95[i,1] >= IntervalMesure[i,1]:
           Borne[i] = True
       else:Borne[i] = False      

    Out = len(Borne) - np.count_nonzero(Borne)

    OutInt95 =  ((float(Out)) / float(CompLen))*100

    print("The number of interval out of the interval is about %f /100 for %s" %(OutInt95,Titre))

    timepoint =[]
    ROI=[]
    subject=[]
    BOLD=[]

    for i in range(len(t_mesure)):

        timepoint.append(t_mesure[i]/86400)
        timepoint.append(t_mesure[i]/86400)
        timepoint.append(t_mesure[i]/86400)
        for j in range(len(p)) :
            timepoint.append(t_mesure[i]/86400)

        ROI.append("Measurement")
        ROI.append("Measurement")
        ROI.append("Measurement")
        for j in range(len(p)) :
            ROI.append("Simulation")

        subject.append(1)
        subject.append(2)
        subject.append(3)
        for j in range(len(p)) :
            subject.append(j+1)

        BOLD.append(IntervalMesure[i,0])
        BOLD.append(mesure[0][i])
        BOLD.append(IntervalMesure[i,1])
        for j in range(len(p)) :
            BOLD.append(Matrice[j][i])



    d={'timepoint':timepoint,
       'ROI':ROI,
       'subject':subject,
       'BOLD signal':BOLD}

    gammas = pd.DataFrame(d)   
    fig, ax = plt.subplots()
    ax = sns.tsplot(data=gammas, time="timepoint", unit="subject",
               condition="ROI", value="BOLD signal")
    ax.set_ylabel("%s" %(Titre))
    ax.set_xlabel('Time (Day)')
    fig = ax.get_figure()
    fig.savefig('Interval %s'%(Titre))

def main():
    #Call differents function required
    Launcher()
    Analyze(Titre=u"Energy used (MWh)",Size=u"Size_CPT %d.csv",SizeN=u"Size_CPT_Normale.csv",Simulation=u"Simulation_CPT %d.csv", Temps=u"Temps_CPT %d.csv",Normale=u"Normale_CPT.csv",TempsN=u"Temps_CPT_Normale.csv",mesure=Cpt_mesure,Delta=0.048,alpha=0.05,absolue=False)



    for i in range(len(p)):
        shutil.rmtree(os.path.join(basepath,u'case %d' % (i)))

        os.remove(os.path.join(basepath,u"Size_Tout %d.csv"%(i)))

        os.remove(os.path.join(basepath,u"Simulation_Tout %d.csv"%(i)))

        os.remove(os.path.join(basepath,u"Temps_Tout %d.csv"%(i)))

if __name__ == "__main__":

    main()
```
## Sources :
1. Bontemps, S. (2015). Validation expérimentale de modèles : application aux bâtiments basse consommation. Http://www.theses.fr. Retrieved from http://www.theses.fr/2015BORD0337
