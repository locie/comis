within FBM.Examples;
model Boiler_Embedded
  "Example and test for heating system with embedded emissionExample and test for heating system with embedded emission"
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
                                       radSlaCha_ValidationEmpa(rho_b=1600)
    annotation (Placement(transformation(extent={{-84,26},{-64,46}})));
  FBM.HeatingDHWsystems.SimpleHeatingSystem.Boiler_FloorHeating
    heating(
    each RadSlaCha=radSlaCha_ValidationEmpa,
    CvDataSupply=Buildings.Fluid.Types.CvTypes.Kv,
    redeclare package Medium = MediumW,
    InsuHeatCondu=0.04,
    nZones=nZones,
    corFac_val=0,
    KvSupply=60,
    isAHU=true,
    dTSupRetNom=10,
    Pipelength=12,
    InsuPipeThickness=0.01,
    TSupNom=273.15 + 50,
    TSupMin=273.15 + 22,
    QNom={3200 for i in 1:nZones},
    T_SetPoint_night={289.15 for i in 1:nZones},
    AEmb={6*6 for i in 1:nZones},
    TRoomNom={567.3,567.3})
    annotation (Placement(transformation(extent={{108,52},{132,72}})));
  Modelica.Thermal.HeatTransfer.Components.Convection[nZones] convectionTabs
    annotation (Placement(transformation(
        extent={{8,-8},{-8,8}},
        rotation=0,
        origin={60,22})));
  FBM.Components.NakedTabs[nZones] nakedTabs(radSlaCha=
       radSlaCha_ValidationEmpa, A_floor=7*7)
    annotation (Placement(transformation(extent={{92,12},{72,32}})));
  Modelica.Blocks.Sources.RealExpression[nZones] realExpression(each y=11*6*6)
    annotation (Placement(transformation(extent={{82,34},{62,54}})));
  Modelica.Blocks.Sources.Constant mFlowDH(k=0)
    "Room temperature set point at night"
    annotation (Placement(transformation(extent={{-3,-3},{3,3}},
        rotation=90,
        origin={127,41})));
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam="modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos")
    annotation (Placement(transformation(extent={{-36,128},{-16,148}})));
  parameter
    Buildings.HeatTransfer.Data.OpaqueConstructions.Insulation100Concrete200
    matLayExt(material={Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=
        0.5),Buildings.HeatTransfer.Data.Solids.Concrete(x=0.15)})
              "Construction material for exterior walls"
    annotation (Placement(transformation(extent={{-58,24},{-38,44}})));
  parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Brick120 matLayPar
    "Construction material for partition walls"
    annotation (Placement(transformation(extent={{-32,24},{-12,44}})));
  parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic matLayRoo(final
      nLay=2, material={Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=
        0.1),Buildings.HeatTransfer.Data.Solids.Concrete(x=0.15)})
                      "Construction material for roof"
    annotation (Placement(transformation(extent={{-86,2},{-66,22}})));
  parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic matLayFlo(
        final nLay=3, material={Buildings.HeatTransfer.Data.Solids.Concrete(x=
        0.15),Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.10),
        Buildings.HeatTransfer.Data.Solids.Concrete(x=0.05)})
                      "Construction material for floor"
    annotation (Placement(transformation(extent={{-58,2},{-38,22}})));
  Buildings.ThermalZones.Detailed.MixedAir roo(
    redeclare package Medium = MediumA,
    nConBou=1,
    nSurBou=1,
    nConExt=3,
    nPorts=2,
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
    nConPar=0,
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
    lat=0.73094389073523) "Room model"
    annotation (Placement(transformation(extent={{-32,84},{8,124}})));
  Buildings.ThermalZones.Detailed.MixedAir roo1(
    redeclare package Medium = MediumA,
    nConExt=2,
    nSurBou=2,
    nPorts=2,
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
    nConBou=1,
    datConBou(
      layers={matLayFlo},
      each A=7*7,
      each til=Buildings.Types.Tilt.Floor,
      each stateAtSurface_a=false),
    nConPar=0,
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
    lat=0.73094389073523) "Room model"
    annotation (Placement(transformation(extent={{-34,44},{6,84}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[nZones] TRoo
    annotation (Placement(transformation(extent={{30,98},{40,108}})));
  Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
        transformation(extent={{-14,118},{26,158}}),    iconTransformation(
          extent={{-298,194},{-278,214}})));
  Modelica.Blocks.Math.MatrixGain gai(K=1*[0.5; 0.4; 0.1])
    "Matrix gain to split up heat gain in radiant, convective and latent gain"
    annotation (Placement(transformation(extent={{-68,80},{-48,100}})));
  Modelica.Blocks.Sources.CombiTimeTable intGaiFra(
       extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic, table=[0,
        0.03; 3600*8,0.03; 3600*9,0.6; 3600*12,0.6; 3600*12,0.6; 3600*13,0.6;
        3600*13,0.7; 3600*17,0.7; 3600*19,0.1; 3600*24,0.03])
    "Fraction of internal heat gain"
    annotation (Placement(transformation(extent={{-94,80},{-74,100}})));
  FBM.AHUSystems.CTA_VAVReheat_Hydro     cTA_VAVReheat(
    VRoom={7*7*3,7*7*3},
    nReheat=nZones,
    redeclare package Medium = MediumW,
    redeclare package MediumA = MediumA,
    m_flow_nominal=0.50,
    OccOn=7,
    THeaOn=294.15,
    THeaOff=289.15,
    TRooSetHeaOcc=293.15,
    m_flow_room={0.20,0.20})
    annotation (Placement(transformation(extent={{136,104},{98,132}})));
  Buildings.HeatTransfer.Sources.FixedTemperature TSoi[nConBou](each T=278.15)
    "Boundary condition for construction"
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        origin={10,32})));
  Modelica.Blocks.Sources.RealExpression shade(y=0.2)
    annotation (Placement(transformation(extent={{-88,112},{-68,132}})));
  Modelica.Blocks.Routing.Replicator replicator(nout=2)
    annotation (Placement(transformation(extent={{-60,116},{-50,128}})));
  parameter Buildings.HeatTransfer.Data.GlazingSystems.DoubleClearAir13Clear
    glaSys(shade=Buildings.HeatTransfer.Data.Shades.Gray(), haveExteriorShade=
        true) annotation (Placement(transformation(extent={{-30,0},{-10,20}})));
equation
  connect(realExpression.y, convectionTabs.Gc) annotation (Line(
      points={{61,44},{60,44},{60,30}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(heating.mDHW60C, mFlowDH.y) annotation (Line(points={{126,51.8},{127,
          51.8},{127,44.3}},             color={0,0,127}));
  connect(roo.heaPorAir, TRoo[1].port) annotation (Line(points={{-13,104},{26,
          104},{26,102},{30,102},{30,103}},
                            color={191,0,0}));
  connect(roo1.heaPorAir, TRoo[2].port) annotation (Line(points={{-15,64},{26,
          64},{26,94},{26,104},{30,104},{30,103}},
                            color={191,0,0}));
  connect(roo.surf_conBou[1], roo1.surf_surBou[2]) annotation (Line(points={{-6,88},
          {-6,88},{-6,84},{-18,84},{-18,50.5},{-17.8,50.5}},
                                                 color={191,0,0}));
  connect(heating.heatPortCon[1], roo.heaPorAir) annotation (Line(points={{108.2,
          64},{100,64},{100,104},{-13,104}},
                                       color={191,0,0}));
  connect(heating.heatPortRad[1], roo.heaPorRad) annotation (Line(points={{108,60},
          {100,60},{100,100.2},{-13,100.2}},
                                           color={191,0,0}));
  connect(weaDat.weaBus, weaBus) annotation (Line(
      points={{-16,138},{6,138}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(weaBus, roo.weaBus) annotation (Line(
      points={{6,138},{4,138},{4,121.9},{5.9,121.9}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(weaBus, roo1.weaBus) annotation (Line(
      points={{6,138},{4,138},{4,81.9},{3.9,81.9}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(heating.weaBus, weaBus) annotation (Line(
      points={{102,71},{68,71},{68,138},{6,138}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(intGaiFra.y,gai. u) annotation (Line(
      points={{-73,90},{-70,90}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dash));
  connect(gai.y, roo.qGai_flow) annotation (Line(points={{-47,90},{-48,90},{-48,
          112},{-33.6,112}},
                           color={0,0,127}));
  connect(gai.y, roo1.qGai_flow) annotation (Line(points={{-47,90},{-48,90},{
          -48,72},{-35.6,72}}, color={0,0,127}));
  connect(roo.heaPorAir, convectionTabs[1].fluid) annotation (Line(points={{-13,104},
          {50,104},{50,22},{52,22}},    color={191,0,0}));
  connect(convectionTabs[2].fluid, roo1.heaPorAir) annotation (Line(points={{52,22},
          {50,22},{50,64},{-15,64}},     color={191,0,0}));
  connect(heating.heatPortCon[2], roo1.heaPorAir) annotation (Line(points={{108.2,
          64},{108,64},{-15,64}},            color={191,0,0}));
  connect(heating.heatPortRad[2], roo1.heaPorRad) annotation (Line(points={{108,60},
          {108,60},{108,60.2},{-15,60.2}},       color={191,0,0}));
  connect(heating.heatPortCon[1], roo.heaPorAir) annotation (Line(points={{108.2,
          64},{100,64},{100,104},{-13,104}},
                                       color={191,0,0}));
  connect(heating.heatPortRad[1], roo.heaPorRad) annotation (Line(points={{108,60},
          {100,60},{100,100.2},{-13,100.2}},
                                           color={191,0,0}));
  connect(TRoo.T, heating.TSensor) annotation (Line(points={{40,103},{96,103},{
          96,56},{107.6,56}},   color={0,0,127}));
  connect(nakedTabs.portCore, heating.heatPortEmb) annotation (Line(points={{92,22},
          {102,22},{102,52},{114,52}},         color={191,0,0}));
  connect(nakedTabs.port_a, convectionTabs.solid) annotation (Line(points={{82,32},
          {74,32},{74,22},{68,22}}, color={191,0,0}));
  connect(nakedTabs.port_b, convectionTabs.solid) annotation (Line(points={{82,12.2},
          {74,12.2},{74,22},{68,22}}, color={191,0,0}));
  connect(weaDat.weaBus, cTA_VAVReheat.weaBus) annotation (Line(
      points={{-16,138},{138,138},{138,130},{133.641,130},{133.641,129.365}},
      color={255,204,51},
      thickness=0.5));
  connect(cTA_VAVReheat.port_a, heating.port_a) annotation (Line(points={{130.759,
          113.882},{130.759,80.353},{121.6,80.353},{121.6,52}},     color={0,127,
          255}));
  connect(cTA_VAVReheat.port_b, heating.port_b) annotation (Line(points={{117.655,
          113.882},{117.655,82},{118,82},{118.6,82},{118.6,52}},    color={0,127,
          255}));
  connect(cTA_VAVReheat.port_a2[1], roo.ports[1]) annotation (Line(points={{98,
          107.294},{-46,107.294},{-46,92},{-27,92}},
                                      color={0,127,255}));
  connect(cTA_VAVReheat.port_b1[1], roo.ports[2]) annotation (Line(points={{98,
          124.588},{-46,124.588},{-46,96},{-27,96}},
                                      color={0,127,255}));
  connect(cTA_VAVReheat.port_a2[2], roo1.ports[1]) annotation (Line(points={{98,
          107.294},{-46,107.294},{-46,52},{-29,52}},
                                      color={0,127,255}));
  connect(cTA_VAVReheat.port_b1[2], roo1.ports[2]) annotation (Line(points={{98,
          124.588},{-46,124.588},{-46,56},{-29,56}},
                                      color={0,127,255}));
  connect(TRoo.T, cTA_VAVReheat.TRoo) annotation (Line(points={{40,103},{50,103},
          {50,132.165},{107.172,132.165}},color={0,0,127}));
  connect(roo1.surf_conBou[1], TSoi[1].port) annotation (Line(points={{-8,48},{
          -8,48},{-8,32},{0,32}}, color={191,0,0}));
  connect(shade.y, replicator.u)
    annotation (Line(points={{-67,122},{-61,122}}, color={0,0,127}));
  connect(replicator.y, roo.uSha)
    annotation (Line(points={{-49.5,122},{-33.6,122}}, color={0,0,127}));
  connect(replicator.y, roo1.uSha) annotation (Line(points={{-49.5,122},{-44,
          122},{-44,82},{-35.6,82}}, color={0,0,127}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,0},{140,
            160}})),
    experiment(StopTime=200000, Interval=900),
    __Dymola_experimentSetupOutput,
    Icon(coordinateSystem(extent={{-100,0},{140,160}})));
end Boiler_Embedded;
