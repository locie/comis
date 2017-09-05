---
title: Uncertainty propagation
category: First Tutorial
order: 3
---
<p align="justify">
We know which parameters is influencial on our model. We also purpose that the responsible part is in the "Boiler Room" But in a real case, <strong>we have to check whether our systems are operating within the permissible margin of error based on the uncertainty of the parameters</strong>. For this, as explained in the section "Uncertainty propagation", it is checked that the measurement is in an uncertainty interval. You can find the code required for this operation below. The principle is to take back the influential parameters and to vary them thanks to an LHS sampling and finally to create a confidence interval based on the output variation.
 </p>

 ```python
 #!/usr/bin/env python2
 # -*- coding: utf-8 -*-
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
 from multiprocessing import Pool
 import statsmodels.stats.api as sms
 import pandas as pd

 basepath = os.path.abspath('//media//wthomare//DONNEES//FyPy')
 TSimulation = 604800

 line     = []
 expected = []
 saw      = []
 cont     = True

 while cont == True:
     try:
         mesures = pd.read_table(os.path.join("//media","wthomare","DONNEES","comis","FBM","Utilities","Tutorial","FirstTutorial.txt"), skiprows=line)
         t_tags    = "t(s)"
         Cpt_tags  = "integrator"

         cont = False

         t_mesure    = np.array(mesures[t_tags])
         Cpt_mesure  = np.array(mesures[Cpt_tags])
         cut = np.flatnonzero(t_mesure == TSimulation)+1
         Cpt_mesure  = Cpt_mesure[:cut]
         t_mesure  = t_mesure[:cut]
         N = len(t_mesure)
         C = 1

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

 model = u"FBM.Tutorial.FirstTuto"
 s = si.Simulator(model,u"dymola",'//media//wthomare//DONNEES//FyPy//case', packagePath="/media/wthomare/DONNEES/comis")
 s.setStopTime(TSimulation)
 s.setSolver(u"dassl")
 s.translate()

 means = [0.90,18000000,0.04,0.04,273.15+45,273.15+16,50]
 stdvs =[0.05,900000,0.004,0.004,2,1,20]


 p = lhs(len(means), samples=100)

 for i in xrange(len(means)):
     p[:, i] = norm(loc=means[i], scale=stdvs[i]).ppf(p[:, i])

 filename = os.path.join(basepath,"OnGoingResult.csv")
 filenorm = os.path.join(basepath,"NormResult.csv")
 filepoint = os.path.join(basepath,"Point.csv")

 fieldnames = ["Simulation %d" % (j) for j in range(len(p))]
 fieldNorm =["Normale"]

 fullnames = ["Simulation %d" % (j) for j in range(len(p))]
 fullnames.append("Normale")

 def simulateCase(s):

     s.setStopTime(TSimulation)
     s.showProgressBar(False)
     s.printModelAndTime()
     s.simulate_translated()

 def SimRef() :

     s.setOutputDirectory('/media/wthomare/DONNEES/FyPy/case')
     simulateCase(s)
     if not os.path.exists(os.path.join("//media","wthomare","DONNEES","FyPy",'case',u"FirstTuto.mat")):

         print (u"no file")

     else:

         resultFile = os.path.join("//media","wthomare","DONNEES","FyPy",'case',u"FirstTuto.mat")
         r=Reader(resultFile, u"dymola")
         (t1,CPT_Calculer)=r.values(u"integrator.y")

         shutil.rmtree('//media//wthomare//DONNEES//FyPy//case')

         CPT_calculer  = np.column_stack ((t1,CPT_Calculer))
         CPT_calculer  = CPT_calculer[0:len(CPT_calculer):2]

         CPT_calcul = np.delete(CPT_calculer,(0), axis = 1)
         CPT_calcul = np.asarray(CPT_calcul)
         CPT_calcul = np.reshape(CPT_calcul, len(CPT_calcul))

         Temps_calcul = np.delete(CPT_calculer,(1), axis = 1)
         Temps_calcul = np.asarray(Temps_calcul)
         Temps_calcul = np.reshape(Temps_calcul, len(Temps_calcul))

         f, ax = pltccf.subplots()
         CCF = np.correlate(CPT_calcul,CPT_calcul,'full')
         ax.plot(CCF)
         pltccf.savefig(os.path.join(basepath,'ccf_case.png'))
         pltccf.close('all')

         fig = pltacf.figure(figsize=(12,8))
         ax1 = fig.add_subplot(211)
         fig = sm.graphics.tsa.plot_acf(CPT_calcul, ax=ax1)
         pltacf.savefig(os.path.join(basepath,'acf_case.png'))
         pltacf.close('all')

         with open(os.path.join(basepath,"Normale.csv"), 'a') as csvfile:
             spamwriter = csv.DictWriter(csvfile,fieldnames=fieldNorm,delimiter = ",",quotechar='|',lineterminator='\n')
             for j in range(len(CPT_calcul)):
                 spamwriter.writerow({"Normale" : CPT_calcul[j]})

         with open(os.path.join(basepath,"Temps Normale.csv"), 'a') as csvfile:
             spamwriter = csv.DictWriter(csvfile,fieldnames=fieldNorm ,delimiter = ",",quotechar='|',lineterminator='\n')
             for j in range(len(Temps_calcul)):
                 spamwriter.writerow({"Normale" : Temps_calcul[j]})

         with open(os.path.join(basepath,"Size Normale.csv"), 'a') as csvfile:
             spamwriter = csv.DictWriter(csvfile,fieldnames=fieldNorm ,delimiter = ",",quotechar='|',lineterminator='\n')
             for j in range((1)):
                 spamwriter.writerow({"Normale" : len(CPT_calcul)})

 def evaluate(i):

     s1 = copy.deepcopy(s)
     s1.setOutputDirectory('/media/wthomare/DONNEES/FyPy/case %d' % (i))
     s1.addParameters({u"eff":p[i,0]})
     s1.addParameters({u"datFue.h":p[i,1]})
     s1.addParameters({u"kPipe":p[i,2]})
     s1.addParameters({u"kTank":p[i,3]})
     s1.addParameters({u"TSup_nominal":p[i,4]})
     s1.addParameters({u"lesThrTRoo.threshold":p[i,5]})
     s1.addParameters({u"mixingCircuit_Tset.KvReturn":p[i,6]})
     simulateCase(s1)

     simulateCase(s1)
     if not os.path.exists(os.path.join("//media","wthomare","DONNEES","FyPy",'case %d' % (i),u"FirstTuto.mat")):

         print (u"no file")

     else:

         resultFile = os.path.join("//media","wthomare","DONNEES","FyPy",'case %d' % (i),u"FirstTuto.mat")
         r=Reader(resultFile, u"dymola")
         (t1,CPT_Calculer)=r.values(u"integrator.y")

         shutil.rmtree('//media//wthomare//DONNEES//FyPy//case %d' % (i))

         CPT_calculer  = np.column_stack ((t1,CPT_Calculer))
         CPT_calculer  = CPT_calculer[0:len(CPT_calculer):2]

         CPT_calcul = np.delete(CPT_calculer,(0), axis = 1)
         CPT_calcul = np.asarray(CPT_calcul)
         CPT_calcul = np.reshape(CPT_calcul, len(CPT_calcul))

         Temps_calcul = np.delete(CPT_calculer,(1), axis = 1)
         Temps_calcul = np.asarray(Temps_calcul)
         Temps_calcul=np.reshape(Temps_calcul, len(Temps_calcul))

         Temps_calcul = np.asarray(Temps_calcul)
         Temps_calcul = np.reshape(Temps_calcul, len(Temps_calcul))

         f, ax = pltccf.subplots()
         CCF = np.correlate(CPT_calcul,CPT_calcul,'full')
         ax.plot(CCF)
         pltccf.savefig(os.path.join(basepath,'ccf_case_%d.png' % (i)))
         pltccf.close('all')

         fig = pltacf.figure(figsize=(12,8))
         ax1 = fig.add_subplot(211)
         fig = sm.graphics.tsa.plot_acf(CPT_calcul, ax=ax1)
         pltacf.savefig(os.path.join(basepath,'acf_case_%d.png' % (i)))
         pltacf.close('all')

         with open(os.path.join(basepath,"Simulation %d.csv" % (i)), 'a') as csvfile:
             spamwriter = csv.DictWriter(csvfile,fieldnames=[fieldnames[i]],delimiter = ",",quotechar='|',lineterminator='\n')
             for j in range(len(CPT_calcul)):
                 spamwriter.writerow({"Simulation %d" % (i) : CPT_calcul[j]})

         with open(os.path.join(basepath,"Temps %d.csv" % (i)), 'a') as csvfile:
             spamwriter = csv.DictWriter(csvfile,fieldnames=[fieldnames[i]],delimiter = ",",quotechar='|',lineterminator='\n')
             for j in range(len(Temps_calcul)):
                 spamwriter.writerow({"Simulation %d" % (i) : Temps_calcul[j]})

         with open(os.path.join(basepath,"Size %d.csv"%(i)),'a') as csvfile:
             spamwriter = csv.DictWriter(csvfile,fieldnames=[fullnames[i]],delimiter = ",",quotechar='|',lineterminator='\n')
             for j in range((1)):
                 spamwriter.writerow({"Simulation %d" % (i) : len(CPT_calcul)})

 def Launcher():

     for i in range(len(p)):
         with open(os.path.join(basepath,"Size %d.csv"%(i)), 'wb') as csvfile:
             spamwriter = csv.DictWriter(csvfile, fieldnames=[fullnames[i]],delimiter = ",",quotechar='|',lineterminator='\n')
             spamwriter.writeheader()

     with open(os.path.join(basepath,"Size Normale.csv"), 'wb') as csvfile:
             spamwriter = csv.DictWriter(csvfile, fieldnames=fieldNorm,delimiter = ",",quotechar='|',lineterminator='\n')
             spamwriter.writeheader()


     with open(os.path.join(basepath,"Normale.csv"), 'wb') as csvfile:
             spamwriter = csv.DictWriter(csvfile, fieldnames=fieldNorm,delimiter = ",",quotechar='|',lineterminator='\n')
             spamwriter.writeheader()

     with open(os.path.join(basepath,"Temps Normale.csv"), 'wb') as csvfile:
             spamwriter = csv.DictWriter(csvfile, fieldnames=fieldNorm,delimiter = ",",quotechar='|',lineterminator='\n')
             spamwriter.writeheader()

     for i in range(len(p)):
          with open(os.path.join(basepath,"Simulation %d.csv" %(i)), 'wb') as csvfile:
             spamwriter = csv.DictWriter(csvfile, fieldnames=[fieldnames[i]],delimiter = ",",quotechar='|',lineterminator='\n')
             spamwriter.writeheader()

     for i in range(len(p)):
          with open(os.path.join(basepath,"Temps %d.csv" %(i)), 'wb') as csvfile:
             spamwriter = csv.DictWriter(csvfile, fieldnames=[fieldnames[i]],delimiter = ",",quotechar='|',lineterminator='\n')
             spamwriter.writeheader()

     po = Pool(processes = 4)
     po.map(evaluate,range(len(p)))

 def Analyze ():

     GoodSim =[]
     GoodCol = []
     GoodSimT = []

     dataSize = np.zeros((len(fullnames),1))
     GoodSize = np.zeros((len(fullnames),1))
     FichierSize = ["Size %d.csv" % (j) for j in range(len(p))]
     FichierSize.append("Size Normale.csv")

     for k in range(len(FichierSize)):
         with open(os.path.join(basepath,FichierSize[k]),'rb') as f:
             DataLen = pd.read_csv(f,sep = "\n")
             dataSize[k,0] = DataLen[fullnames[k]][0]

     sizeMax = int(max(dataSize))

     for i in range(len(dataSize)):
             if float(dataSize[i]) > (float(sizeMax) - 0.10*float(sizeMax)):
                 GoodSize[i] =1
             else : GoodSize[i]=0

     for i in range(0,len(GoodSize)-1):
         if GoodSize[i] == 1 :
             GoodSim.append("Simulation %d.csv" %(i))
             GoodSimT.append("Temps %d.csv" %(i))
             GoodCol.append("Simulation %d" %(i))


     if GoodSize[len(GoodSize)-1] == 1 :
             GoodSim.append("Normale.csv")
             GoodSimT.append("Temps Normale.csv")
             GoodCol.append("Normale")


     ResLen = dataSize*GoodSize
     NonZerLen = []
     for i in range(len(ResLen)):
         if ResLen[i] != 0:
             NonZerLen.append(ResLen[i])

     print("Le nombre de simulation prise en compte est de %d sur %d"%(len(NonZerLen),len(fullnames)))
     size = int(min(NonZerLen))

     Matrice = np.zeros(((np.count_nonzero(GoodSize)),size))
     MatriceTemp = np.zeros(((np.count_nonzero(GoodSize)),size))

     for i in range(np.count_nonzero(GoodSize)):
         f = open(os.path.join(basepath,GoodSim[i]))
         Data = pd.read_csv(f,sep = "\n")
         for j in range(size):
                 Matrice[i,j] = Data[GoodCol[i]][j]
         f.close()

     for i in range(np.count_nonzero(GoodSize)):
         f = open(os.path.join(basepath,GoodSimT[i]))
         Data = pd.read_csv(f,sep = "\n")
         for j in range(size):
                 MatriceTemp[i,j] = Data[GoodCol[i]][j]
         f.close()

     Int95 = []
     for i in range(size):
         Iteration =sms.DescrStatsW(Matrice[:,i]).tconfint_mean(alpha=0.05)
         Int95.append(Iteration)

     Int95 = np.array(Int95)

     temp = np.arange(0,size)
     for i in range(size):
         temp[i]=temp[i]*600

     fig, ax = plt.subplots()

     p1 = ax.plot(MatriceTemp[1,:], Int95[:,0],ls = '--',color = 'g')
     p2 = ax.plot(MatriceTemp[1,:],Int95[:,1],ls = '--', color='r')
     p3 = ax.plot(MatriceTemp[np.count_nonzero(GoodSize)-1,:],Matrice[np.count_nonzero(GoodSize)-1,:], ls = '--',color='b')
     p4 = ax.plot(t_mesure,Cpt_mesure,color='black')
     ax.legend((p1[0], p2[0],p3[0],p4[0]), ('Low', 'High','Normale','Mesure'),loc='upper left')
     plt.savefig(os.path.join(basepath,'Interval_de_confiance.png'))
     plt.close('all')

     Borne = np.ones(size, dtype=bool)
     CompLen = min(len(t_mesure),size)

     for i in range(CompLen):
        if Int95[i,0] <= Cpt_mesure[i] and Int95[i,1] >= Cpt_mesure[i]:
            Borne[i] = True
        else:Borne[i] = False

     Out = len(Borne) - np.count_nonzero(Borne)

     OutInt95 =  ((float(Out)) / float(CompLen))*100

     print("Le nombre de points hors interval est de %f /100" %(OutInt95))
 def main():

     Launcher()
     SimRef()
     Analyze()
     shutil.rmtree('//media//wthomare//DONNEES//FyPy//case')

     for i in range(len(p)):
         os.remove(os.path.join(basepath,"Size %d.csv"%(i)))
         os.remove(os.path.join(basepath,"Simulation %d.csv"%(i)))
         os.remove(os.path.join(basepath,"Temps %d.csv"%(i)))

     os.remove(os.path.join(basepath,"Normale.csv"))
     os.remove(os.path.join(basepath,"Temps Normale.csv"))
     os.remove(os.path.join(basepath,"Size Normale.csv"))

 if __name__ == "__main__":

     main()
 ```
<p align ="justify"> One of the results of this script is the following graph. The black curve, the consumption of wood over a week, is the measure. The curve is not within the confidence interval (the green and red curve) <strong>for 86% of the time steps</strong>. We can therefore conclude that our system works in a deteriorated way and requires a calibration phase to be commissioned </p>

<img src="Propagation.png" alt="propagation" style="width: 500px;" align ="middle"/>
