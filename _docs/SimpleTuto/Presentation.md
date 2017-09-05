---
title: Presentation
category: First Tutorial
order: 1
---
# Model presentation

> The model required for the tutorial is available in the FBM library.

<p align="justify"> <strong>The case study is constitued of a building of 432m² with U-shap (3 wing of 144m²).</strong> The basis of the U-shape is turned towards the south. The weather conditions are those of Chicago. The envelop is modeled with 3 "Four RC element" components from the Buildings library (interior and exterior walls, floor, roof and windows definition) and with a HVAC system dedicated to the heating of the building. Details on the RC components from the Buildings library are available in the relevant sections of the library and will not be further detailed here. </p>

<img src="/Presentation/Building.png" alt="Envelop model" style="width: 600px;" align ="middle"/>

<p align="justify"> The HVAC system is divided into to part :</p>

- The Heat production (The blue rectangle calls "Boiler room")
- The Heat emission (The green rectangle calls "Hydraunic heating systems")

<p align="justify"><strong> The emission part is subdivided into the East and West wing.</strong> Both constitue an independant emitting network. Between the "Boiler room" and the "Hydraunic heating systems", a pump ensures the return of the cold water to the boiler room. The "Boiler room" is made of an wood pellet stove, a pump, a mixing circuit and a storage tank. The West and East wing of the HVAC system are made of a mixing circuit, a supply pump and for each wing of the building a thermostatic valve and a radiator. The south wing is heated partly by the West and the East circuit and so a thermostatic valve and a radiator is modeled in each of these circuits (for a total of 4 valves and 4 radiators). </p>

<p align="justify">The monitoring of the wood stove is made by a boolean loop wich ensure that the pump is running 30s before and 30s after that the boiler turns on or off. The boiler turn-on when the outdoor temperature and the indoor temperature are cold enough and turn-off otherwise. The minimal return temperature of the water is controled by PID bloc. The supply temperature of the emitting circuits is controled by a water logic and the pumps of each circuits is turn on/off when the outdoor and indoor temperature is cold enough. The proportional opening of the thermosstatic valves is emulated by a PID bloc.

<p align="justify"> The red rectangle name <strong>"Indicators"</strong> is the part of the model where indicators are computed (Component and system indicators). The Indicators will be our output values for the tutorial.</p>

<img src="/Presentation/Modele.png" alt="Full model" style="width: 600px;" align ="middle"/>

# Problem presentation

<p align="justify"> For the tutorial,<strong> we will be focus on the fuel consumption of the "Boiler room"</strong>. To measured this consumption, we will compute the mass of fuel burned by the stove for one week. As you can see, the boiler consumms 139% more in reality that what is expected with a well commissioned system <strong>(110 kg of pellet wood instead of 46kg and for an overcost of 25€)</strong></p>

<img src="/Presentation/WoodCurve.png" alt="Full model" style="width: 600px;" align ="middle"/>

<p align="justify">Let us suppose that this error is only due to a problem of commissioning in the "boiler-room". We will apply our method only to the components and parameters of this part of the model. For more details on the model, as mention, it is available in the FBM library with nominal values.<strong> We can now start the commissioning process by a sensitivity analyse </strong></p>
