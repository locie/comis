---
title: Modelica
category: Requirements
order: 2
---

### What is Modelica ?
**Modelica** is an object-oriented, declarative, multi-domain modeling language for component-oriented modeling of complex systems, e.g., systems containing mechanical, electrical, electronic, hydraulic, thermal, control, electric power or process-oriented subcomponents. For more information about the Modelica language see Web-book of Michael M. Tiller : [Modelica by Example](http://book.xogeny.com/)

### What will we need ?

We need here of a modeling and simulation environment and of some librairies. As expected the simulation environment has to be compatible with the Modelica language.

1. [Dymola](https://www.3ds.com/fr/produits-et-services/catia/produits/dymola/)
2. [Buildings library](https://github.com/lbl-srg/modelica-buildings)
3. [FBM library](https://github.com/locie/comis)


> [Dymola](https://www.3ds.com/fr/produits-et-services/catia/produits/dymola/)

Among modeling and simulation environment already alvailable Dymola seems to be the more stable and powerfull. However, Dymola is a commercial environment and if you need of a free environment others are available ([JModelica](http://www.jmodelica.org/), [OpenModelica](https://openmodelica.org/)) but often not fully compatible with the following libraries.


>[Buildings library](https://github.com/lbl-srg/modelica-buildings)

The Modelica Buildings library is a free open-source library with dynamic simulation models for building energy and control systems. The library contains models for

* HVAC systems,
* Controls,
*   Heat transfer among rooms and the outside,
*   Multizone airflow, including natural ventilation and contaminant transport,
*   Single-zone computational fluid dynamics coupled to heat transfer and HVAC systems,
*   Data-driven load prediction for demand response applications, and
*   Electrical DC and AC systems with two- or three-phases that can be balanced and unbalanced.

The primary use of the library is for flexible and fast modeling of building energy and control systems to accelerate innovation leading to cost-effective very low energy systems for new and existing buildings. All development is open-source and we welcome contributions. For more details see the [Google Group](https://groups.google.com/forum/#!forum/modelica-buildings), the [Dedicated website](http://simulationresearch.lbl.gov/modelica/) or the [Github](https://github.com/lbl-srg/modelica-buildings)

> [FBM library](https://github.com/locie/comis)

The [FBM library](https://github.com/locie/comis) is an additional modeling layer based on the [Buildings library](https://github.com/lbl-srg/modelica-buildings).This library reduces the implementation time of an entire building model with the most common system collection already completed. In addition, the sensors and mathematical formulas necessary for calculating indicators for commissioning are included in the system models. For the following examples, we will use a coupling of the [FBM library](https://github.com/locie/comis) and [Buildings library](https://github.com/lbl-srg/modelica-buildings). More informations and details coming soon about [FBM library](https://github.com/locie/comis) but still today in early phase of development

### How to set your Dymola environment

In order to work effectively, it is advisable to check some details with Dymola

* When Dymola is installed check in your environment variables if Dymola and ModelicaPath appear. If you use OpenModelica or JModelica, similar operation is advisable.

* Start a first and simple simulation with the basic Modelica library in order to check your licence file, your compiler and your working directory

* Import libraries required for your purpose and repeat the previous operation

* With your Python environment execute the examples available on the [Buildingspy Website](http://simulationresearch.lbl.gov/modelica/buildingspy/examples.html#simulating-a-model-with-two-different-parameter-values)

* If you suceed previous steps, your are ready for the rest of the tutorial
