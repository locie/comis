within FBM.Heating.Examples;
model Heating_Embedded
  "Example and test for heating system with embedded emissionExample and test for heating system with embedded emission"
  import Buildings;
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

  parameter
    FBM.Components.BaseClasses.RadiantSlabChar[nZones]
                                       radSlaCha_ValidationEmpa
    annotation (Placement(transformation(extent={{-134,-138},{-114,-118}})));
  FBM.Heating.Heating_Embedded heating(
    each RadSlaCha=radSlaCha_ValidationEmpa,
    CvDataSupply=Buildings.Fluid.Types.CvTypes.Kv,
    TSupMin=273.15 + 22,
    redeclare package Medium = MediumW,
    TSupNom=273.15 + 45,
    InsuHeatCondu=0.04,
    nZones=nZones,
    dTSupRetNom=5,
    corFac_val=0,
    AEmb={6*5 for i in 1:nZones},
    KvSupply=60,
    Pipelength=15,
    QNom={3000 for i in 1:nZones},
    TRoomNom={567.3,567.3})
                          annotation (Placement(transformation(extent={{114,20},
            {152,38}})));
  Modelica.Thermal.HeatTransfer.Components.Convection[nZones] convectionTabs
    annotation (Placement(transformation(
        extent={{8,-8},{-8,8}},
        rotation=0,
        origin={76,36})));
  FBM.Components.NakedTabs[nZones] nakedTabs(radSlaCha=
       radSlaCha_ValidationEmpa, A_floor=6*5)
    annotation (Placement(transformation(extent={{104,26},{84,46}})));
  Modelica.Blocks.Sources.RealExpression[nZones] realExpression(each y=11*6*4)
    annotation (Placement(transformation(extent={{150,48},{130,68}})));
  Buildings.Controls.SetPoints.OccupancySchedule
                                       occSch(occupancy=3600*{8,19})
                                              "Occupancy schedule"
    annotation (Placement(transformation(extent={{60,-50},{80,-30}})));
  Modelica.Blocks.Sources.Constant TRooNig(k=273.15 + 16)
    "Room temperature set point at night"
    annotation (Placement(transformation(extent={{60,-72},{80,-52}})));
  Modelica.Blocks.Sources.Constant TRooSet(k=273.15 + 21)
    annotation (Placement(transformation(extent={{60,-20},{80,0}})));
  Modelica.Blocks.Logical.Switch swi1 "Switch to select set point"
    annotation (Placement(transformation(extent={{102,-56},{122,-36}})));
  Modelica.Blocks.Routing.Replicator replicator1(nout=nZones)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={128,-10})));
  Modelica.Blocks.Sources.Constant mFlowDH(k=0)
    "Room temperature set point at night"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={150,-10})));
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam="modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos")
    annotation (Placement(transformation(extent={{-220,218},{-200,238}})));
  parameter
    Buildings.HeatTransfer.Data.OpaqueConstructions.Insulation100Concrete200
    matLayExt(material={Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.1),
        Buildings.HeatTransfer.Data.Solids.Concrete(x=0.15)})
              "Construction material for exterior walls"
    annotation (Placement(transformation(extent={{-100,-138},{-80,-118}})));
  parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Brick120 matLayPar
    "Construction material for partition walls"
    annotation (Placement(transformation(extent={{-60,-138},{-40,-118}})));
  parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic matLayRoo(final
      nLay=2, material={Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.1),
        Buildings.HeatTransfer.Data.Solids.Concrete(x=0.2)})
                      "Construction material for roof"
    annotation (Placement(transformation(extent={{-20,-138},{0,-118}})));
  parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic matLayFlo(
        final nLay=3, material={Buildings.HeatTransfer.Data.Solids.Concrete(x=0.2),
        Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.10),
        Buildings.HeatTransfer.Data.Solids.Concrete(x=0.05)})
                      "Construction material for floor"
    annotation (Placement(transformation(extent={{20,-138},{40,-118}})));
  Modelica.Blocks.Sources.Constant uSha(k=0.25)
    "Control signal for the shading device"
    annotation (Placement(transformation(extent={{-136,-34},{-116,-14}})));
  Modelica.Blocks.Routing.Replicator replicator(nout=max(1, nConExtWin))
    annotation (Placement(transformation(extent={{-108,-34},{-88,-14}})));
  Buildings.Rooms.MixedAir roo(
    redeclare package Medium = MediumA,
    nConPar=1,
    datConPar(
      layers={matLayPar},
      each A=10,
      each til=Buildings.Types.Tilt.Wall),
    nConBou=1,
    nSurBou=1,
    linearizeRadiation=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    nConExt=3,
    nPorts=4,
    datConBou(
      layers={matLayFlo},
      each A=7*7,
      each til=Buildings.Types.Tilt.Floor),
    AFlo=7*7,
    hRoo=3,
    datConExt(
      layers={matLayRoo,matLayExt,matLayExt},
      A={7*7,7*3,7*3},
      til={Buildings.Types.Tilt.Ceiling,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
      azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.W,Buildings.Types.Azimuth.N}),
    surBou(
      each A=7*7,
      each absIR=0.9,
      each absSol=0.9,
      each til=Buildings.Types.Tilt.Floor),
    nConExtWin=nConExtWin,
    T_start=273.15 + 18,
    datConExtWin(
      layers={matLayExt,matLayExt},
      A={7*3,7*3},
      glaSys={glaSys,glaSys},
      hWin={2,2},
      wWin={5,5},
      ove(
        wR={0,0},
        wL={0,0},
        gap={0.1,0.1},
        dep={1,1}),
      fFra={0.1,0.1},
      til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
      azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.E}),
    lat=0.73268921998722) "Room model"
    annotation (Placement(transformation(extent={{-36,70},{4,110}})));

  Buildings.Rooms.MixedAir roo1(
    redeclare package Medium = MediumA,
    nConExt=2,
    nConPar=1,
    datConPar(
      layers={matLayPar},
      each A=10,
      each til=Buildings.Types.Tilt.Wall),
    nConBou=1,
    linearizeRadiation=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    nSurBou=2,
    nPorts=4,
    datConBou(
      layers={matLayFlo},
      each A=7*7,
      til={Buildings.Types.Tilt.Floor}),
    AFlo=7*7,
    hRoo=3,
    datConExt(
      layers={matLayExt,matLayExt},
      A={7*3,7*3},
      til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
      azi={Buildings.Types.Azimuth.W,Buildings.Types.Azimuth.N}),
    surBou(
      A={7*3,7*7},
      each absIR=0.9,
      each absSol=0.9,
      til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Ceiling}),
    nConExtWin=nConExtWin,
    T_start=273.15 + 18,
    datConExtWin(
      layers={matLayExt,matLayExt},
      A={7*3,7*3},
      glaSys={glaSys,glaSys},
      hWin={2,2},
      wWin={5,5},
      ove(
        wR={0,0},
        wL={0,0},
        gap={0.1,0.1},
        dep={1,1}),
      fFra={0.1,0.1},
      til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
      azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.E}),
    lat=0.73268921998722) "Room model"
    annotation (Placement(transformation(extent={{-36,20},{4,60}})));
  Buildings.HeatTransfer.Sources.FixedTemperature TSoi[nConBou](each T=280.15)
    "Boundary condition for construction"
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        origin={30,12})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[nZones] TRoo
    annotation (Placement(transformation(extent={{68,90},{78,100}})));
  Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
              leaSou(redeclare package Medium = MediumA,
    azi=Buildings.Types.Azimuth.S,
    VRoo=7*7*3,
    s=1)
    "Model for air infiltration through the envelope"
    annotation (Placement(transformation(extent={{-154,150},{-118,190}})));
  Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
              leaEas(
    azi=Buildings.Types.Azimuth.E,
    VRoo=7*7*3,
    s=1,
    redeclare package Medium = MediumA)
    "Model for air infiltration through the envelope"
    annotation (Placement(transformation(extent={{-154,128},{-118,168}})));
  Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
              leaNor(
    azi=Buildings.Types.Azimuth.N,
    VRoo=7*7*3,
    s=1,
    redeclare package Medium = MediumA)
    "Model for air infiltration through the envelope"
    annotation (Placement(transformation(extent={{-154,110},{-118,150}})));
  Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
              leaWes(
    azi=Buildings.Types.Azimuth.W,
    VRoo=7*7*3,
    s=1,
    redeclare package Medium = MediumA)
    "Model for air infiltration through the envelope"
    annotation (Placement(transformation(extent={{-154,90},{-118,130}})));
  Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
              leaSou1(
    azi=Buildings.Types.Azimuth.S,
    VRoo=7*7*3,
    s=1,
    redeclare package Medium = MediumA)
    "Model for air infiltration through the envelope"
    annotation (Placement(transformation(extent={{-156,66},{-120,106}})));
  Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
              leaEas1(
    azi=Buildings.Types.Azimuth.E,
    VRoo=7*7*3,
    s=1,
    redeclare package Medium = MediumA)
    "Model for air infiltration through the envelope"
    annotation (Placement(transformation(extent={{-156,46},{-120,86}})));
  Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
              leaNor1(
    azi=Buildings.Types.Azimuth.N,
    VRoo=7*7*3,
    s=1,
    redeclare package Medium = MediumA)
    "Model for air infiltration through the envelope"
    annotation (Placement(transformation(extent={{-156,26},{-120,66}})));
  Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
              leaWes1(
    azi=Buildings.Types.Azimuth.W,
    s=1,
    redeclare package Medium = MediumA,
    VRoo=7*7*3)
    "Model for air infiltration through the envelope"
    annotation (Placement(transformation(extent={{-156,8},{-120,48}})));
  Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
        transformation(extent={{-196,172},{-156,212}}), iconTransformation(
          extent={{-298,194},{-278,214}})));
  Modelica.Blocks.Math.MatrixGain gai(K=3.5*[0.5; 0.3; 0.2])
    "Matrix gain to split up heat gain in radiant, convective and latent gain"
    annotation (Placement(transformation(extent={{-72,50},{-52,70}})));
  Modelica.Blocks.Sources.CombiTimeTable intGaiFra(
       extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic, table=[0,0.03;
        3600*8,0.03; 3600*9,0.7; 3600*12,0.7; 3600*12,0.7; 3600*13,0.7; 3600*13,
        0.8; 3600*17,0.8; 3600*19,0.1; 3600*24,0.03])
    "Fraction of internal heat gain"
    annotation (Placement(transformation(extent={{-100,50},{-80,70}})));
  parameter Buildings.HeatTransfer.Data.GlazingSystems.DoubleClearArgon13Clear
    glaSys(
    haveInteriorShade=true,
    shade=Buildings.HeatTransfer.Data.Shades.Gray(),
    UFra=2.1)
    annotation (Placement(transformation(extent={{60,-138},{80,-118}})));
equation
  connect(realExpression.y, convectionTabs.Gc) annotation (Line(
      points={{129,58},{76,58},{76,44}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(occSch.occupied,swi1. u2) annotation (Line(points={{81,-46},{98,-46},
          {100,-46}},        color={255,0,255}));
  connect(TRooNig.y,swi1. u3) annotation (Line(points={{81,-62},{100,-62},{100,
          -54}},    color={0,0,127}));
  connect(TRooSet.y,swi1. u1) annotation (Line(points={{81,-10},{96,-10},{96,
          -38},{100,-38}},
                    color={0,0,127}));
  connect(swi1.y,replicator1. u)
    annotation (Line(points={{123,-46},{128,-46},{128,-22}},
                                                 color={0,0,127}));
  connect(replicator1.y, heating.TSet) annotation (Line(points={{128,1},{134,1},
          {134,14},{134,19.82},{133,19.82}},          color={0,0,127}));
  connect(heating.mDHW60C, mFlowDH.y) annotation (Line(points={{138.7,19.82},{
          140,19.82},{140,1},{150,1}},   color={0,0,127}));
  connect(uSha.y,replicator. u) annotation (Line(
      points={{-115,-24},{-110,-24}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TSoi.port, roo1.surf_conBou)
    annotation (Line(points={{20,12},{-10,12},{-10,24}}, color={191,0,0}));
  connect(roo.heaPorAir, TRoo[1].port) annotation (Line(points={{-17,90},{48,90},
          {48,94},{56,94},{56,95},{68,95}},
                            color={191,0,0}));
  connect(roo1.heaPorAir, TRoo[2].port) annotation (Line(points={{-17,40},{48,40},
          {48,94},{68,94},{68,95}},
                            color={191,0,0}));
  connect(roo.surf_conBou[1], roo1.surf_surBou[2]) annotation (Line(points={{-10,
          74},{-46,74},{-46,26.5},{-19.8,26.5}}, color={191,0,0}));
  connect(heating.heatPortCon[1], roo.heaPorAir) annotation (Line(points={{114,
          30.8},{48,30.8},{48,90},{-17,90}},
                                       color={191,0,0}));
  connect(heating.heatPortRad[1], roo.heaPorRad) annotation (Line(points={{114,27.2},
          {48,27.2},{48,86.2},{-17,86.2}}, color={191,0,0}));
  connect(leaSou.port_b, roo.ports[1]) annotation (Line(points={{-118,170},{-88,
          170},{-88,77},{-31,77}}, color={0,127,255}));
  connect(leaEas.port_b, roo.ports[2]) annotation (Line(points={{-118,148},{-88,
          148},{-88,79},{-31,79}}, color={0,127,255}));
  connect(leaNor.port_b, roo.ports[3]) annotation (Line(points={{-118,130},{-86,
          130},{-86,81},{-31,81}}, color={0,127,255}));
  connect(leaWes.port_b, roo.ports[4]) annotation (Line(points={{-118,110},{-86,
          110},{-86,83},{-31,83}}, color={0,127,255}));
  connect(leaSou1.port_b, roo1.ports[1]) annotation (Line(points={{-120,86},{
          -104,86},{-104,27},{-31,27}},
                                   color={0,127,255}));
  connect(leaEas1.port_b, roo1.ports[2]) annotation (Line(points={{-120,66},{
          -104,66},{-104,29},{-31,29}},
                                   color={0,127,255}));
  connect(leaNor1.port_b, roo1.ports[3]) annotation (Line(points={{-120,46},{
          -104,46},{-104,31},{-31,31}},
                                   color={0,127,255}));
  connect(leaWes1.port_b, roo1.ports[4]) annotation (Line(points={{-120,28},{
          -104,28},{-104,33},{-31,33}},
                                   color={0,127,255}));
  connect(weaDat.weaBus, weaBus) annotation (Line(
      points={{-200,228},{-176,228},{-176,192}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(weaBus, leaSou.weaBus) annotation (Line(
      points={{-176,192},{-176,192},{-176,170},{-154,170}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(weaBus, leaEas.weaBus) annotation (Line(
      points={{-176,192},{-176,148},{-154,148}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(weaBus, leaNor.weaBus) annotation (Line(
      points={{-176,192},{-176,130},{-154,130}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(weaBus, leaWes.weaBus) annotation (Line(
      points={{-176,192},{-176,192},{-176,110},{-154,110}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(weaBus, leaSou1.weaBus) annotation (Line(
      points={{-176,192},{-176,139},{-176,86},{-156,86}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(weaBus, leaEas1.weaBus) annotation (Line(
      points={{-176,192},{-176,129},{-176,66},{-156,66}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(weaBus, leaNor1.weaBus) annotation (Line(
      points={{-176,192},{-176,192},{-176,46},{-156,46}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(weaBus, leaWes1.weaBus) annotation (Line(
      points={{-176,192},{-176,192},{-176,28},{-156,28}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(weaBus, roo.weaBus) annotation (Line(
      points={{-176,192},{2,192},{2,107.9},{1.9,107.9}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(weaBus, roo1.weaBus) annotation (Line(
      points={{-176,192},{1,192},{1,57.9},{1.9,57.9}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(heating.weaBus, weaBus) annotation (Line(
      points={{114.57,32.42},{110,32.42},{110,192},{-176,192}},
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
  connect(roo.heaPorAir, convectionTabs[1].fluid) annotation (Line(points={{-17,
          90},{26,90},{26,36},{68,36}}, color={191,0,0}));
  connect(convectionTabs[2].fluid, roo1.heaPorAir) annotation (Line(points={{68,
          36},{26,36},{26,40},{-17,40}}, color={191,0,0}));
  connect(heating.heatPortCon[2], roo1.heaPorAir) annotation (Line(points={{114,
          30.8},{50,30.8},{50,40},{-17,40}}, color={191,0,0}));
  connect(heating.heatPortRad[2], roo1.heaPorRad) annotation (Line(points={{114,
          27.2},{48,27.2},{48,36.2},{-17,36.2}}, color={191,0,0}));
  connect(heating.heatPortCon[1], roo.heaPorAir) annotation (Line(points={{114,30.8},
          {50,30.8},{50,90},{-17,90}}, color={191,0,0}));
  connect(heating.heatPortRad[1], roo.heaPorRad) annotation (Line(points={{114,27.2},
          {48,27.2},{48,86.2},{-17,86.2}}, color={191,0,0}));
  connect(TRoo.T, heating.TSensor) annotation (Line(points={{78,95},{96,95},{96,
          23.6},{113.62,23.6}}, color={0,0,127}));
  connect(nakedTabs.portCore, heating.heatPortEmb) annotation (Line(points={{104,
          36},{110,36},{110,34.4},{114,34.4}}, color={191,0,0}));
  connect(nakedTabs.port_a, convectionTabs.solid) annotation (Line(points={{94,46},
          {90,46},{90,36},{84,36}}, color={191,0,0}));
  connect(nakedTabs.port_b, convectionTabs.solid) annotation (Line(points={{94,26.2},
          {90,26.2},{90,36},{84,36}}, color={191,0,0}));
  connect(replicator.y, roo.uSha) annotation (Line(points={{-87,-24},{-46,-24},{
          -46,108},{-37.6,108}}, color={0,0,127}));
  connect(replicator.y, roo1.uSha) annotation (Line(points={{-87,-24},{-62,-24},
          {-62,58},{-37.6,58}}, color={0,0,127}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-220,-200},{220,
            240}})),
    experiment(StopTime=200000, Interval=900),
    __Dymola_experimentSetupOutput,
    Icon(coordinateSystem(extent={{-220,-200},{220,240}})));
end Heating_Embedded;
