---
title: Hierarchical structure
category: Commissionning method
order: 1
---

> Heating, ventilation and air conditioning (HVAC) is the technology of indoor and vehicular environmental comfort.

# Why should used a hierarchical structure ?
<p align="justify">
If the objective of our method is to commission an entire building, this operation can't be carried out in a single operation. In order to split up our method without losing precision, we have defined a hierarchical structure. <strong> This structure is a bottom-up approach focalized on HVAC systems </strong>. The structure consists of four levels of indicators available below.
</p>

  - Component level
  - System level
  - Service level
  - Building level

<p align="justify"> These different level are nested with different relation borrowed from data based language (many-to-many, many-to-one and one-to-many) visisble at the top of the previous figure. These levels have different roles in the processus of commissionning. Each of the following section focus on one level</p>

<img src="struc.png" alt="hierarchical structure" style="width: 800px;" align ="middle"/>

### Component Level

<p align="justify">
<strong>This level consists mainly on a data based of indicators.</strong> These indicators are structured by component. Each typical component that can be found in a HVAC system (pump, valve, solar pannel, hot battery, ...) is associated with a list of indicators and measurement points. These measure points (Temperature, Volume, Power, ...) are required to compute the indicators of the component. We have selected as sensor model, distribution and uncertainty the preconizations defined in (Costic, 2015).
 </p>

<p align="justify">
Indicators are grouped into two categories the energy performance indicators and component-specific indicators. <strong>The energy performance indicators </strong>are those agglomerated at the higher levels and are therefore indispensable to the realization of the entire process. As their names indicate, they also serve to ensure the intrinsic energy performance of the component. <strong> The component-specific indicators </strong> are used to characterize typical sources of malfunction (If the temperature in a tank is higher at the bottom that at the top, the temperature at the outlet of a fan is too hot  compared to the inlet temperature, ...). They are not necessary but generally requires only few additional measuring point. They are therefore used to measure the intrinsic non-energy performance of components</p>

<p align="justify">At the component level, models are usually already modeled in Modelica libraries (if you use this language). The numerical model is then reduced to a simple component with the initial conditions resulting from the measurement</p>

<img src="component.png" alt="component model" style="width: 800px;" align ="middle"/>

### System Level

<p align="justify"> <strong>The system level is used to ensure the performance of the elements linking and coordinating the various components.</strong> This level therefore ensures the performance of distribution networks and monitoring&control. At this level, the parameters taken into account in the commissioning process are therefore those relating to these two categories (eg: water temperature, system start-up time, insulation performance of a hot water network, ...). The development of indicators specific to each system that can be encountered in a building being impossible, the so-called "System" indicators consist only of energy performance indicators. They are calculated by the aggregation of the "component" energy performance indicators. The nesting of the component indicators to obtain the system indicators is available in section <i>The Nesting of different levels</i> </p>

<p align="justify"> The system indicators make it possible to assert the performances of the distribution networks as well as the monitoring&control, <strong>but it also makes it possible to calculate the energy performance of the system as a whole.</strong> Thus, it is possible to calculate the impact of a fault at the component level on the whole of a system, whether it is negligible or harmful to all the system.</p>

<p align="justify">The relationship between component level and system is <strong>many-to-many</strong>. Indeed, a component can belong to several systems (a boiler can belong to a heating system and a domestic hot water system) and a system generally comprising several components (an AHU includes fans, filters, heat exchangers, etc.)</p>

<p align="justify">At the system level, models are normally an association of previously used components, distribution networks components (pies, valves, ...) and monitoring&control elements (PID, Sensors, ...). Limit conditions are generally limited to the exterior and interior conditions of the building</p>

<img src="system.png" alt="system model" style="width: 800px;" align ="middle"/>

### Service Level

<p align="justify">The service level is divided into 5 main categories: <strong>Heating, Cooling, Domestic hot water, Lighting and Ventilation.</strong> All systems defined at the lower hierarchical level must be included in one of these categories. Service level indicators are agglomerates of system level indicators and are therefore energy performance indicators</p>

<p align="justify">These indicators are used to <strong>calculate the contribution of each system to a service and thus discriminate the interest of one system over another</strong>. It also makes it possible to judge the maturity of a technology and the interest of an innovative solution compared to a usual solution (eg : used solar pannel instead of a boiler for heating or )</p>

<p align="justify">The relationship between system and service levels is <strong>many-to-one</strong>. Indeed, a system is generally dedicated to a service but a service can include several systems (the ventilation of a building can be provided by several AHUs). If a system may be included in several services, then the share serving one service and the share serving the other must be separated and subdivided into two systems.</p>

<p align="justify"> At this level, the models require little development because they only constitute of present systems and external and internal conditions as boundary conditions. </p>

<img src="service.png" alt="service model" style="width: 800px;" align ="middle"/>

### Building Level

<p align="justify">The building level is used to couple the HVAC systems with an envelope model in order <strong>to calculate the reglementary energy performance indicators</strong> (eg :
<a href="https://fr.wikipedia.org/wiki/Diagnostic_de_performance_%C3%A9nerg%C3%A9tique">Energy Performance Certificate</a>) It is very important to remember that this level does not require a commissioning process since only an envelope model is added to the global model. Indeed, it is not our objective to qualify the effectiveness of the envelope of a building but only of these systems. HLC type indicators can be set up to describe the performance of the envelope, but the instrumentation and calculations necessary for the implementation of such an indicator is not the subject of our method.</p>

<p align="justify">Models can be tedious to develop at the building level according to the software used for the building envelope and the complexity of the architecture. However, the boundary conditions are then limited only to external conditions</p>

### The Nesting of different levels

<p align="justify"> <strong>When a component belongs to different systems</strong>, it is then necessary to determine in what proportion this component dedicates itself to each system. This determination is made by a <strong>partition coefficient</strong> on the energy performance indicators at the component level.</p>

<img src="flux.png" alt="flux separation" style="width: 400px;" align ="middle"/>

<p align="justify">If we consider each grey round as a component in the previous figure, we observe that the second belongs to two systems (or that the primary flow divides into two secondary flows). It is then necessary to calculate by a ratio of proportion the part leaving in the system 1 and in the system 2. <strong>This coefficient will be applied to all consumptions of components prior to this division</strong> in order to apply to each system the respective energy consumptions (the red flames on the schema). The same technique can be applied if a system belongs to more than one service, but as specified in the <i> Service level </i> section, we recommend to divide this system into two instead.</p>

# Reference :

Costic. 2015. L’instrumentation Des Bâtiments. ADEME créadequat. Guide pratique.
