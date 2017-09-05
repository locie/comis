---
title: Overall Process
category: Implementation
order: 1
---

<head>
  <meta charset="utf-8">
  <link rel="stylesheet" href="./node_modules/mermaid/dist/mermaid.css">

  <script src="https://use.fontawesome.com/7065416dc9.js"></script>

</head>

> Full process flow chart

<body>
  <script src="./node_modules/mermaid/dist/mermaid.min.js"></script>
  <script>mermaid.initialize({startOnLoad:true});</script>

  <div class="mermaid">
  graph TB
      A(Sensitivity Analyses)  -- Selection of influencial parameters --> B(Definition of mean values and standard deviations)
      B --> C(Propagation of uncertainty with a LHS)
      C --> D(Define the interval of confidence at 95% of the indicators)
      D --> E(Is the measured indicator in the interval of simulation?)
      E -- Yes --> F(Your system works in accordance with the uncertainty of the measurement and of simulation)
      E -- No --> H(You need to to calibrated your numerical model)
      H --> I(Once parameter calibrated, determine with maximum likelyhood test the new uncertainty of the parameters)
      I --> L(Is the new numerical indicator close enough to the measured indicator)
      L -- Yes --> M(Validation phase of the model)
      M -- Succefull validation --> N(Identify sources of discrepancy)
      M -- Unsuccesfull validation --> O(See section 2.2 for explanation)
  </div>
</body>

<p align="justify"> Don't be worry the full procees will be explained step by step and in the most comprehensible way. Moreover all Python code needed for the realization of the differents step is shared in devoted section</p>

> **Commissionning definition** : The purpose of the commissioning phase of new buildings is to ensure its energy performance and those of these components. It integrates the implementation in design activities, reception, development, and the implementation of an energy monitoring based on an instrumentation plan tailored.

# Presentation of the problem

<p align="justify">According to the <strong> IAE Annex 47 </strong> conclusions (Neumannn et al. 2010), a major barrier to market penetration of commissioning phase is the lack of methods and associated tools to ensure that HVAC systems operate efficiently in buildings. The practice of calibrating simulations of building systems to assess the actual performance have been addressed in many research projects (M. Liu et al. 2003)(Park et al. 2013). However, the detailed modeling required for system calibration makes it difficult to operate this process on an entire building model. To facilitate this process, <strong> a multi-level modeling and calibration approach is explained in this website.</strong> </p>

# 1.  Do you need to a commissionning phase ?

<p align="justify">
The first step is to know whether the case study is operating within the expected performance intervals. In order to find out, it is necessary to create within the time interval studied a range of performance covering all the possible situations.<strong> This confidence interval represents the misunderstanding of the exact values of the parameters of the case study </strong>. Even if manufacturers provide documentations with parameter values and sometimes associated uncertainties, it is difficult to know the impact of the uncertainties of a parameter on a complex case. It is even more difficult when the number of parameters increases to several doozens because of the interactions between parameters. It might be tempting to vary all the parameters of a case study over the whole of their interval. However, this is too cumbersome in computation time. To limit this excess two mathematical tools can be associated
</p>

**1.  Sensitivity analyses**

**2. Propagation of uncertainty**

### 1.1. Sensitivity analyses

<p align="justify">
In order to reduce the computation time and to allow the definition of the confidence interval, we are looking to know which parameters are useful to consider and which can be withdrawn of the on going of process. Sensitivity analyses are perfectly suited for this purpose. (Technical details on conducting a sensitivity analysis are presented in the associated part of the site.) <strong>Indeed a sensitivity analysis classifies the influence of the parameters of a model on an output of the given model.</strong> The result is a % of influence on the output of the parameter in relation to the others considered. The sum of these % is therefore 100% if the parameters considered are judiciously chosen. The sum differs from 100% if the number of simulation is too small or by the too limited initial choice of the studied parameters</p>

<p align = "justify">
After realization of the sensitivity analyses, it is therefore sufficient <strong>to extract among the parameters those having the greatest influence until reaching a minimum influence threshold </strong> (typically 80% or 90%). However, this process is hugely dependant of output studies. Moreover, <strong>the set of parameters suited for a given indicator is by defaut unsuited for an other one</strong>.
</p>

<p align = "justify">
One of the results of this research project is the minimum of influence that a parameter must have to be calibrated and its new uncertainty determined. As you can see on the figure below, under a weight of 5 / 10% our calibration process give an new uncertainty interval length of several times the calibrated value. Indeed the figure represent the normalized interval of uncertainty (interval length divided by the calibrated value of the parameter in function of the weight given by the Morris screening)</p>

<img src="Influence.png" alt="Normalized interval length in function of parmeter's weight" style="width: 500px;" align ="middle"/>

<p align = "justify">
With this result, we choose to select parameters to calibrated with a minimum weight of 5% and a set of parameters with a global influence of 80%
</p>

### 1.2. Propagation of uncertainty
<p align = "justify">
When the set of parameters is define is no question to realize the confidence interval by varying in interval's uncertainties of each parameters with a fix step of variation. Indeed, even if the number of parameters is reduced, it would be a waste to time. Instead of, propagation of uncertainty can be realized with a sampler to screen more effectively the search space of all parameters. Details about different propagation of uncertainty methods can be consulted in (Bontemps 2015) or in the devoted section of the website.The one retains for our method is the <strong>Latin Hypercube Sampler (LHS)</strong> for its ability to cover the entire search space in an equiprobable manner while being a good compromise with the number of simulation required</p>

### 1.3 Confidence interval
<p align = "justify">
When the sampling is obtained and simulation are ran, we have different curves of the output value. With those curves, <strong>we can determine a interval of confidence for each time step of the curves.</strong> (typically a 95% confidence interval). Those intervals indicates in which range the same output issued from the measured can be acceptable.
</p>

<p align = "justify">
Those intervals can be plotted to know if the measured is in the good range. For each time steps, select the upper value of the interval and plot that curve. Repeat the process with the lower value of the interval or see the example below.
</p>

<img src="IntervalEx.png" alt="95% interval" style="width: 500px;" align ="middle"/>

<p align = "justify">
Plot the interval does not allow to automate this selection criterion (realized or not the calibration). To do this, it is necessary to check whether for each time step the measurement is in the interval and store the information in a boolean vector (whose length is therefore equal to the number of time steps). <strong>It is then possible to determine a % of time steps outside the interval and triggering the realization of the rest of the process</strong>
</p>

<p align = "justify">
We proceed the first part of the process (see below) and we know if the case studied required a commissioning phase. Let's move on to the next phase
</p>

<body>
  <script src="./node_modules/mermaid/dist/mermaid.min.js"></script>
  <script>mermaid.initialize({startOnLoad:true});</script>

  <div class="mermaid">
  graph TB
      A(Sensitivity Analyses)  -- Selection of influencial parameters --> B(Definition of mean values and standard deviations)
      B --> C(Propagation of uncertainty by LHS)
      C --> D(Define the interval of confidence at 95% of the indicator)
      D --> E(Is the measured indicator in the interval ?)
      E -- Yes --> F(Your system works in its range of performance)
      E -- No --> H(You need to to calibrated your numerical model)
  </div>
</body>

# 2.  How to proceed a commissionning phase ?

### 2.1 Calibration and validation realization

> The normal value of a parameter is the one issued from the As build doc. The normal simulation is a simulation with parameter values fixed to their normal values

<p align = "justify">
Let us assume that our case study needs to be commissioned because of these poor performances. As you have seen on page AAA, the originality of our method is based on the hierarchical structure chosen to define a building. <strong> The other originality is to choose a calibration algorithm to automate commissioning.</strong>
</p>

<p align = "justify">
Indeed,<strong> a calibration algorithm makes it possible to vary the parameter values from their normal values to their values which minimize an objective function.</strong> This objective function is in our methods the least squares function between the measurement and the simulation. Consequently, when calibration is performed, we obtain parameter values which minimize the gap between the simulation and the measurement. The new model (with these new parameter values) must then be validated over a different time range in order to validate the lessons learned during the calibration. Otherwise, the parameters obtained could result in an optimal over the initial time range without explaining the actual behavior of the case study.
</p>

<p align = "justify">
When the calibreation is completed, the confidence interval can be determine with the <strong>maximum likelihood estimator </strong> (ML) (Meeker & Escobar, 1995). Indeed with the ML estimator, a confidence regions/intervals can be determined with least square result of the calibration process (Lehmann 1986). Technically the method comes down to plot the curve Sum(r²)/(Var*N) -Sum(r_bestfit²) function of the parameter value. With Sum(r²) the least sqare result of a simulation, Sum(r_bestfit²) the least sqare result of the best individual, N the number of simulation and Var, the variance of the parameter. When this function is obtained for each simulation, we select all parameter values with a result above the <strong>minimum of the curve + 3.841459</strong></p>

<p align = "justify"> The value 3.841459 as explained in (Meeker & Escobar, 1995) is the value of the quantile of the chi square function with one degree of freedom. The result of such process is searchable below</p>

<img src="parametre_5.png" alt="ML estimator for a parameter" style="width: 500px;" align ="middle"/>

<img src="parametre_6.png" alt="ML estimator for an other parameter" style="width: 500px;" align ="middle"/>

<p align = "justify">When both curve are plotted, all parameter value between the minimum of the ML estimator and the constant are into the confidence interval. We just have to find the minimum and the maximum of those parameters to determine the bound of the confidence interval.</p>

<p align = "justify">
When the calibration and the validation is proceed two situations may be possible. Calibration and validation took place correctly. Then we can move on to the diagnostic phase. Calibration or validation is inconclusive.
</p>

### 2.2 Unsuccesfull calibration or validation

> Correlation is the relationship between two parameters, the value of one of which can not be find out without the other. By example, for an insulation material, the conductivity and the thickness of the material are correlated if we are looking for the effectiveness of the material.

<p align = "justify">
Let us begin with the second case. If the calibration does not seem to have sufficiently reduced the differences between measurement and simulation or if the validation is not as conclusive as the calibration, this can have several causes :
</p>

- The calibrated parameters are correlated
- The number of parameters is underestimated by the sensitivity analysis
- The numerical model has major differences with the real installation
- Major issues are present in the plant and can not be mimicked by the model
- The installation has changed between the calibration and the validation time series

<p align =  "justify">
The first point can be checked with the <strong>Second-order indices</strong> if you proceed to a sensitivity analyses with the Sobol method instead of the Morris method. More the indice is high more the parameters concerned are correlated. In order to fixe the issue, select one of the parameters concerned and delete it from the list and repeat the calibration process. The second point is more tricky to determine. It is necessary to check if in the models, submodels hiding implicit parameters did not go unnoticed in first selection ...
</p>

<p align =  "justify">
The third point is not strictly speaking our method. It uses know-how in numerical modeling. However, it is strongly recommended to make "white box" models to keep the physical sense of the informations. Thanks to this choice of modelization, it is simpler to analyze the variations of parameters. The fourth and fifth point is also at the discretion of the users and the solution on a case-by-case basis.</p>

### 2.3 Succesfull calibration and validation
<p align =  "justify">
On the other hand, if the calibration and the validation are conclusive it is then possible to switch to the expertise phase. This part uses the knowledge of the operator on the domain concerned by the model. Indeed, the variation of the influential parameters makes it possible to diagnose the errors affecting the model. For example, increasing the conductivity of an insulating material of a hot water storage tank will imply a bad caulking of the balloon. While a reduction in its size induces a reflection on the effective stratification height of the balloon.</p>

<p align =  "justify">
Otherwise, at this stage of the process, it is possible to determine the problems affecting the model whatever the level in our hierarchical organization. Indeed the errors detected at an "n-1" level are taken into account in modeling the model at level "n". Moreover, it is necessary that only the parameters of level n vary.</p>

## Sources :

1. Neumannn, C, H Yoshida, D Choinière, and N Feretti Milesi. 2010. “Annex 47 : Commissioning Tools for Existing and Low Energy Buildings.”
2. Liu, M, D. E. Claridge, N Bensouda, K Heinemeier, L Suk, and G Wei. 2003. “Manual of Procedures for Calibrating Simulations of Building Systems.” Berkeley.
3. Park, Sumee, Victor Norrefeldt, Sebastian Stratbuecker, Gunnar Grün, and Yong-Sung Jang. 2013. “Methodological Approach for Calibration of Building Energy Performance Simulation Models Applied to a Common ‘measurement and Verification’ Process.” Bauphysik 35 (4): 235–41.
4. Bontemps, S. (2015). Validation expérimentale de modèles : application aux bâtiments basse consommation. Http://www.theses.fr. Retrieved from http://www.theses.fr/2015BORD0337
5. Meeker, W. Q., & Escobar, L. A. (1995). Teaching about Approximate Confidence Regions Based on Maximum Likelihood Estimation. The American Statistician, 49(1), 48–53. http://doi.org/10.1080/00031305.1995.10476112
6. Lehmann, E. L. (1986), Testing Statistical Hypotheses (2nd ed.), New York
