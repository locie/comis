within FBM.Examples.Old_Template;
model DoubleStoreyHouse_2
  extends Modelica.Icons.Example;
  final parameter Integer nZones=2 "Number of zones";
package MediumA = Buildings.Media.Air
    "Medium model for air";
    package MediumW = Buildings.Media.Water "Medium model";
    parameter Integer nConExtWin = 2 "Number of constructions with a window";
  parameter Integer nConBou = 1
    "Number of surface that are connected to constructions that are modeled inside the room";
  parameter Integer nSurBou = 1
    "Number of surface that are connected to the room air volume";
  Buildings.Controls.SetPoints.OccupancySchedule
                                       occSch(occupancy=3600*{6,19})
                                              "Occupancy schedule"
    annotation (Placement(transformation(extent={{60,-50},{80,-30}})));
  Modelica.Blocks.Sources.Constant TRooNig(k=273.15 + 18)
    "Room temperature set point at night"
    annotation (Placement(transformation(extent={{60,-72},{80,-52}})));
  Modelica.Blocks.Sources.Constant TRooSet(k=273.15 + 21)
    annotation (Placement(transformation(extent={{60,-20},{80,0}})));
  Modelica.Blocks.Logical.Switch swi1 "Switch to select set point"
    annotation (Placement(transformation(extent={{102,-56},{122,-36}})));
  Modelica.Blocks.Routing.Replicator replicator1(nout=nZones)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={136,-20})));
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
        "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos")
    annotation (Placement(transformation(extent={{-112,110},{-92,130}})));
  parameter
    Buildings.HeatTransfer.Data.OpaqueConstructions.Insulation100Concrete200
    matLayExt(material={Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=
        0.25),Buildings.HeatTransfer.Data.Solids.Concrete(x=0.25)})
              "Construction material for exterior walls"
    annotation (Placement(transformation(extent={{-106,-74},{-86,-54}})));
  parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic matLayRoo(final
      nLay=2, material={Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=
        0.25),Buildings.HeatTransfer.Data.Solids.Concrete(x=0.15)})
                      "Construction material for roof"
    annotation (Placement(transformation(extent={{-46,-74},{-26,-54}})));
  parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic matLayFlo(
        final nLay=3, material={Buildings.HeatTransfer.Data.Solids.Concrete(x=
        0.15),Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.20),
        Buildings.HeatTransfer.Data.Solids.Concrete(x=0.10)})
                      "Construction material for floor"
    annotation (Placement(transformation(extent={{-14,-74},{6,-54}})));
  Modelica.Blocks.Sources.Constant uSha(k=0.25)
    "Control signal for the shading device"
    annotation (Placement(transformation(extent={{-112,88},{-92,108}})));
  Modelica.Blocks.Routing.Replicator replicator(nout=max(1, nConExtWin))
    annotation (Placement(transformation(extent={{-84,88},{-64,108}})));
  Buildings.ThermalZones.Detailed.MixedAir roo(
    redeclare package Medium = MediumA,
    nConBou=1,
    nConExt=3,
    AFlo=7*7,
    hRoo=3,
    datConExt(
      layers={matLayRoo,matLayExt,matLayExt},
      A={7*7,7*3,7*3},
      til={Buildings.Types.Tilt.Ceiling,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
      azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.W,Buildings.Types.Azimuth.N}),
    nConExtWin=nConExtWin,
    nConPar=1,
    nSurBou=0,
    datConBou(
      layers={matLayFlo},
      each A=7*7,
      each til=Buildings.Types.Tilt.Floor),
    datConExtWin(
      layers={matLayExt,matLayExt},
      A={7*3,7*3},
      glaSys={glaSys,glaSys},
      hWin={1.7,1.7},
      wWin={3.5,3.5},
      ove(
        wR={0,0},
        wL={0,0},
        gap={0.1,0.1},
        dep={1,1}),
      fFra={0.1,0.1},
      til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
      azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.E}),
    T_start=273.15 + 17,
    nPorts=2,
    datConPar(
      layers={matLayPar},
      each A=15,
      each til=Buildings.Types.Tilt.Wall),
    lat=0.65484753534827) "Room model"
    annotation (Placement(transformation(extent={{-36,70},{4,110}})));
  Buildings.ThermalZones.Detailed.MixedAir roo1(
    redeclare package Medium = MediumA,
    nConExt=2,
    nConPar=1,
    nConBou=1,
    nSurBou=2,
    datConBou(
      layers={matLayFlo},
      each A=7*7,
      til={Buildings.Types.Tilt.Floor}),
    AFlo=7*7,
    hRoo=3,
    surBou(
      A={7*3,7*7},
      each absIR=0.9,
      each absSol=0.9,
      til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Ceiling}),
    nConExtWin=nConExtWin,
    datConExt(
      layers={matLayExt,matLayExt},
      A={7*3,7*3},
      til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
      azi={Buildings.Types.Azimuth.E,Buildings.Types.Azimuth.N}),
    datConExtWin(
      layers={matLayExt,matLayExt},
      A={7*3,7*3},
      glaSys={glaSys,glaSys},
      hWin={1.7,1.7},
      wWin={3.5,3.5},
      ove(
        wR={0,0},
        wL={0,0},
        gap={0.1,0.1},
        dep={1,1}),
      fFra={0.1,0.1},
      til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
      azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.W}),
    T_start=273.15 + 17,
    nPorts=2,
    datConPar(
      layers={matLayPar},
      each A=15,
      each til=Buildings.Types.Tilt.Wall),
    lat=0.65484753534827) "Room model"
    annotation (Placement(transformation(extent={{-36,20},{4,60}})));
  Buildings.HeatTransfer.Sources.FixedTemperature TSoi[nConBou](each T=283.15)
    "Boundary condition for construction"
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        origin={30,12})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[nZones] TRoo
    annotation (Placement(transformation(extent={{64,60},{74,70}})));
  Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
        transformation(extent={{-84,100},{-44,140}}),   iconTransformation(
          extent={{-298,194},{-278,214}})));
  Modelica.Blocks.Math.MatrixGain gai(K=2*[0.5; 0.3; 0.2])
    "Matrix gain to split up heat gain in radiant, convective and latent gain"
    annotation (Placement(transformation(extent={{-72,50},{-52,70}})));
  Modelica.Blocks.Sources.CombiTimeTable intGaiFra(
       extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic, table=[0,0.03;
        3600*8,0.03; 3600*9,0.7; 3600*12,0.7; 3600*12,0.7; 3600*13,0.7; 3600*13,
        0.8; 3600*17,0.8; 3600*19,0.1; 3600*24,0.03])
    "Fraction of internal heat gain"
    annotation (Placement(transformation(extent={{-100,50},{-80,70}})));
  FBM.HeatingDHWsystems.ModularProductionSystem.MultiProduction_Radiator_DHW heating(
    nZones=nZones,
    CvDataSupply=Buildings.Fluid.Types.CvTypes.Kv,
    nSource=2,
    InsuHeatCondu=0.032,
    lat(displayUnit="deg") = 0.65484753534827,
    source={FBM.HeatingDHWsystems.Types.HeatSource.CondensingBoiler,FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField},
    kInsTes=0.032,
    til=0.785,
    includePipesSol=true,
    redeclare package Medium = MediumW,
    mPrim_flow_nominal=0.745,
    nSer=5,
    dInsTes=0.3,
    TSupNom=273.15 + 55,
    volumeTank=2,
    nPar=3,
    KvSupply=60,
    nbrNodesTes=10,
    InsuPipeThickness=0.03,
    VTanTes=10,
    hTanTes=2,
    Tes_start(displayUnit="K") = 273.15 + 60,
    QBoiler(displayUnit="kW") = 20000,
    Pipelength=2,
    QNom=ones(nZones)*4000,
    nOcc=4,
    dpHexPrim=12000,
    dpHexSecon=12000,
    T_nominal=363.15)
    annotation (Placement(transformation(extent={{116,20},{156,40}})));
  Modelica.Blocks.Sources.Pulse mDHW60C(
    period=10000,
    startTime=5000,
    offset=0,
    amplitude=0.1,
    width=4)        annotation (Placement(transformation(extent={{-10,-10},
            {10,10}},
        rotation=90,
        origin={142,-58})));
  parameter Buildings.HeatTransfer.Data.GlazingSystems.DoubleClearAir13Clear
    glaSys(haveExteriorShade=true, shade=
        Buildings.HeatTransfer.Data.Shades.Gray())
    annotation (Placement(transformation(extent={{22,-70},{42,-50}})));
  AHUSystems.DoubleFlux_FreeCooling_MechanicalCooling CTA(
    VRoom={7*7*3,7*7*3},
    m_flow_room={0.2,0.2},
    nReheat=2)
    annotation (Placement(transformation(extent={{120,98},{100,112}})));
  parameter Components.BaseClasses.PartiWall matLayPar
    annotation (Placement(transformation(extent={{-76,-74},{-56,-54}})));
equation
  connect(occSch.occupied,swi1. u2) annotation (Line(points={{81,-46},{98,-46},
          {100,-46}},        color={255,0,255}));
  connect(TRooNig.y,swi1. u3) annotation (Line(points={{81,-62},{100,-62},{100,
          -54}},    color={0,0,127}));
  connect(TRooSet.y,swi1. u1) annotation (Line(points={{81,-10},{96,-10},{96,
          -38},{100,-38}},
                    color={0,0,127}));
  connect(swi1.y,replicator1. u)
    annotation (Line(points={{123,-46},{136,-46},{136,-32}},
                                                 color={0,0,127}));
  connect(replicator1.y, heating.TSet) annotation (Line(points={{136,-9},{
          136,-9},{136,19.8}},           color={0,0,127}));
  connect(uSha.y,replicator. u) annotation (Line(
      points={{-91,98},{-86,98}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TSoi.port, roo1.surf_conBou)
    annotation (Line(points={{20,12},{-10,12},{-10,24}}, color={191,0,0}));
  connect(roo.heaPorAir, TRoo[1].port) annotation (Line(points={{-17,90},{50,90},
          {50,66},{64,66},{64,65}},
                            color={191,0,0}));
  connect(roo1.heaPorAir, TRoo[2].port) annotation (Line(points={{-17,40},{-17,40},
          {50,40},{50,66},{64,66},{64,65}},
                            color={191,0,0}));
  connect(roo.surf_conBou[1], roo1.surf_surBou[2]) annotation (Line(points={{-10,74},
          {-14,74},{-14,26.5},{-19.8,26.5}},     color={191,0,0}));
  connect(heating.heatPortCon[1], roo.heaPorAir) annotation (Line(points={{124.2,
          32},{50,32},{50,90},{-17,90}},     color={191,0,0}));
  connect(heating.heatPortRad[1], roo.heaPorRad) annotation (Line(points={{124,28},
          {50,28},{50,86.2},{-17,86.2}},         color={191,0,0}));
  connect(weaDat.weaBus, weaBus) annotation (Line(
      points={{-92,120},{-64,120}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(weaBus, roo.weaBus) annotation (Line(
      points={{-64,120},{20,120},{20,107.9},{1.9,107.9}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(weaBus, roo1.weaBus) annotation (Line(
      points={{-64,120},{19,120},{19,57.9},{1.9,57.9}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(heating.weaBus, weaBus) annotation (Line(
      points={{118,39},{70,39},{70,58},{18,58},{18,110},{18,120},{-64,120}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(intGaiFra.y,gai. u) annotation (Line(
      points={{-79,60},{-74,60}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dash));
  connect(gai.y, roo.qGai_flow) annotation (Line(points={{-51,60},{-48,60},{-48,
          98},{-37.6,98}}, color={0,0,127}));
  connect(gai.y, roo1.qGai_flow) annotation (Line(points={{-51,60},{-46,60},{-46,
          48},{-37.6,48}},     color={0,0,127}));
  connect(heating.heatPortCon[2], roo1.heaPorAir) annotation (Line(points={{124.2,
          32},{50,32},{50,40},{-17,40}},      color={191,0,0}));
  connect(heating.heatPortRad[2], roo1.heaPorRad) annotation (Line(points={{124,28},
          {50,28},{50,36},{16,36},{16,36.2},{-17,36.2}},
                                                  color={191,0,0}));
  connect(heating.heatPortCon[1], roo.heaPorAir) annotation (Line(points={{124.2,
          32},{124.2,32},{50,32},{50,90},{-17,90}},
                                             color={191,0,0}));
  connect(heating.heatPortRad[1], roo.heaPorRad) annotation (Line(points={{124,28},
          {50,28},{50,86.2},{-17,86.2}},         color={191,0,0}));
  connect(TRoo.T, heating.TSensor) annotation (Line(points={{74,65},{96,65},{96,
          24},{123.6,24}},     color={0,0,127}));
  connect(replicator.y, roo.uSha) annotation (Line(points={{-63,98},{-42,98},
          {-42,108},{-37.6,108}},color={0,0,127}));
  connect(replicator.y, roo1.uSha) annotation (Line(points={{-63,98},{-48,
          98},{-48,58},{-37.6,58}},
                                color={0,0,127}));
  connect(mDHW60C.y, heating.mDHW60C) annotation (Line(points={{142,-47},{
          142,-47},{142,19.8}}, color={0,0,127}));
  connect(weaBus, CTA.weaBus) annotation (Line(
      points={{-64,120},{118,120},{118,110.2},{118.2,110.2}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(TRoo.T, CTA.TRoo) annotation (Line(points={{74,65},{96,65},{96,112.2},
          {116.2,112.2}}, color={0,0,127}));
  connect(CTA.port_a1[1], roo.ports[1]) annotation (Line(points={{100,102.5},{
          -44,102.5},{-44,78},{-31,78}},
                             color={0,127,255}));
  connect(CTA.port_b1[1], roo.ports[2]) annotation (Line(points={{100,105.3},{
          -44,105.3},{-44,82},{-31,82}},
                                    color={0,127,255}));
  connect(CTA.port_a1[2], roo1.ports[1]) annotation (Line(points={{100,103.5},{
          -44,103.5},{-44,28},{-31,28}},
                             color={0,127,255}));
  connect(CTA.port_b1[2], roo1.ports[2]) annotation (Line(points={{100,106.3},{
          -44,106.3},{-44,32},{-31,32}},
                                    color={0,127,255}));
  annotation ( preferredView="info",
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-80},
            {160,140}})),
    experiment(StopTime=200000, Interval=900),
    __Dymola_experimentSetupOutput,
    Icon(coordinateSystem(extent={{-120,-80},{160,140}})),
    Documentation(info="<html>
<p>
Example for the used of the <a href=\"modelica://FBM.HeatingDHWsystems.ModularProductionSystem.MultiProduction_Radiator\">Multi-source template</a> </p>
<p>The example is composed of the same building that other examples with same radiators implementation but with two heat source and a thermal heat storage tank (TES).
The Heating system is composed of <a href=\"modelica://FBM.Components.SolarPannelFieldWithoutTes\">Solar collector</a> connect to the TES for Inter-seasonal heat storage and a usual gas boiler for the winter supplement
</p>
</html>",
revisions="<html>
<ul>
<li>
December 2016, Wilfried Thomaré :<br/>
First Implementation
</li>
</ul>
</html>"),    Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end DoubleStoreyHouse_2;
