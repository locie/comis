within FBM.Examples;
model Boiler_Embedded
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
    annotation (Placement(transformation(extent={{-196,132},{-176,152}})));
  FBM.HeatingDHWsystems.SimpleHeatingSystem.Boiler_FloorHeating
    heating(
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
    isAHU=true,
    TRoomNom={567.3,567.3})
    annotation (Placement(transformation(extent={{120,22},{158,40}})));
  Modelica.Thermal.HeatTransfer.Components.Convection[nZones] convectionTabs
    annotation (Placement(transformation(
        extent={{8,-8},{-8,8}},
        rotation=0,
        origin={62,24})));
  FBM.Components.NakedTabs[nZones] nakedTabs(radSlaCha=
       radSlaCha_ValidationEmpa, A_floor=7*7)
    annotation (Placement(transformation(extent={{94,14},{74,34}})));
  Modelica.Blocks.Sources.RealExpression[nZones] realExpression(each y=11*7*7)
    annotation (Placement(transformation(extent={{150,48},{130,68}})));
  Modelica.Blocks.Sources.Constant mFlowDH(k=0)
    "Room temperature set point at night"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={150,0})));
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam="modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos")
    annotation (Placement(transformation(extent={{-220,218},{-200,238}})));
  parameter
    Buildings.HeatTransfer.Data.OpaqueConstructions.Insulation100Concrete200
    matLayExt(material={Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.1),
        Buildings.HeatTransfer.Data.Solids.Concrete(x=0.15)})
              "Construction material for exterior walls"
    annotation (Placement(transformation(extent={{-162,132},{-142,152}})));
  parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Brick120 matLayPar
    "Construction material for partition walls"
    annotation (Placement(transformation(extent={{-122,132},{-102,152}})));
  parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic matLayRoo(final
      nLay=2, material={Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.1),
        Buildings.HeatTransfer.Data.Solids.Concrete(x=0.2)})
                      "Construction material for roof"
    annotation (Placement(transformation(extent={{-194,102},{-174,122}})));
  parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic matLayFlo(
        final nLay=3, material={Buildings.HeatTransfer.Data.Solids.Concrete(x=0.2),
        Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.10),
        Buildings.HeatTransfer.Data.Solids.Concrete(x=0.05)})
                      "Construction material for floor"
    annotation (Placement(transformation(extent={{-154,102},{-134,122}})));
  Buildings.ThermalZones.Detailed.MixedAir roo(
    redeclare package Medium = MediumA,
    nConPar=1,
    datConPar(
      layers={matLayPar},
      each A=10,
      each til=Buildings.Types.Tilt.Wall),
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
  Buildings.ThermalZones.Detailed.MixedAir roo1(
    redeclare package Medium = MediumA,
    nConExt=2,
    nConPar=1,
    datConPar(
      layers={matLayPar},
      each A=10,
      each til=Buildings.Types.Tilt.Wall),
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
    nConBou=1,
    datConBou(
      layers={matLayFlo},
      each A=7*7,
      each til=Buildings.Types.Tilt.Floor,
      each stateAtSurface_a=false),
    lat=0.73268921998722) "Room model"
    annotation (Placement(transformation(extent={{-36,20},{4,60}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[nZones] TRoo
    annotation (Placement(transformation(extent={{30,98},{40,108}})));
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
  parameter Buildings.HeatTransfer.Data.GlazingSystems.DoubleClearAir13Clear
    glaSys(                        shade=
        Buildings.HeatTransfer.Data.Shades.Gray(), haveExteriorShade=false)
    annotation (Placement(transformation(extent={{-124,102},{-104,122}})));
  FBM.AHUSystems.CTA_VAVReheat_Hydro     cTA_VAVReheat(
    VRoom={7*7*3,7*7*3},
    nReheat=nZones,
    m_flow_nominal=0.75,
    m_flow_room={0.20,0.20},
    redeclare package Medium = MediumW,
    redeclare package MediumA = MediumA,
    THeaOn=294.15,
    THeaOff=291.15,
    TRooSetHeaOcc=294.15)
    annotation (Placement(transformation(extent={{130,158},{10,226}})));
  Buildings.HeatTransfer.Sources.FixedTemperature TSoi[nConBou](each T=280.15)
    "Boundary condition for construction"
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        origin={18,6})));
equation
  connect(realExpression.y, convectionTabs.Gc) annotation (Line(
      points={{129,58},{62,58},{62,32}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(heating.mDHW60C, mFlowDH.y) annotation (Line(points={{148.5,21.82},{
          150,21.82},{150,11}},          color={0,0,127}));
  connect(roo.heaPorAir, TRoo[1].port) annotation (Line(points={{-17,90},{26,90},
          {26,102},{30,102},{30,103}},
                            color={191,0,0}));
  connect(roo1.heaPorAir, TRoo[2].port) annotation (Line(points={{-17,40},{26,
          40},{26,94},{26,104},{30,104},{30,103}},
                            color={191,0,0}));
  connect(roo.surf_conBou[1], roo1.surf_surBou[2]) annotation (Line(points={{-10,74},
          {-10,74},{-10,26},{-20,26},{-20,26.5},{-19.8,26.5}},
                                                 color={191,0,0}));
  connect(heating.heatPortCon[1], roo.heaPorAir) annotation (Line(points={{120.317,
          32.8},{108,32.8},{108,90},{-17,90}},
                                       color={191,0,0}));
  connect(heating.heatPortRad[1], roo.heaPorRad) annotation (Line(points={{120,29.2},
          {108,29.2},{108,86.2},{-17,86.2}},
                                           color={191,0,0}));
  connect(weaDat.weaBus, weaBus) annotation (Line(
      points={{-200,228},{-176,228},{-176,192}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(weaBus, roo.weaBus) annotation (Line(
      points={{-176,192},{2,192},{2,107.9},{1.9,107.9}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(weaBus, roo1.weaBus) annotation (Line(
      points={{-176,192},{3,192},{3,57.9},{1.9,57.9}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(heating.weaBus, weaBus) annotation (Line(
      points={{110.5,39.1},{110,39.1},{110,192},{-176,192}},
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
  connect(gai.y, roo1.qGai_flow) annotation (Line(points={{-51,60},{-48,60},{-48,
          48},{-37.6,48}},     color={0,0,127}));
  connect(roo.heaPorAir, convectionTabs[1].fluid) annotation (Line(points={{-17,90},
          {50,90},{50,24},{54,24}},     color={191,0,0}));
  connect(convectionTabs[2].fluid, roo1.heaPorAir) annotation (Line(points={{54,24},
          {50,24},{50,40},{-17,40}},     color={191,0,0}));
  connect(heating.heatPortCon[2], roo1.heaPorAir) annotation (Line(points={{120.317,
          32.8},{108,32.8},{108,40},{-17,40}},
                                             color={191,0,0}));
  connect(heating.heatPortRad[2], roo1.heaPorRad) annotation (Line(points={{120,
          29.2},{108,29.2},{108,36.2},{-17,36.2}},
                                                 color={191,0,0}));
  connect(heating.heatPortCon[1], roo.heaPorAir) annotation (Line(points={{120.317,
          32.8},{108,32.8},{108,90},{-17,90}},
                                       color={191,0,0}));
  connect(heating.heatPortRad[1], roo.heaPorRad) annotation (Line(points={{120,29.2},
          {108,29.2},{108,86.2},{-17,86.2}},
                                           color={191,0,0}));
  connect(TRoo.T, heating.TSensor) annotation (Line(points={{40,103},{96,103},{
          96,25.6},{119.367,25.6}},
                                color={0,0,127}));
  connect(nakedTabs.portCore, heating.heatPortEmb) annotation (Line(points={{94,24},
          {102,24},{102,22},{129.5,22}},       color={191,0,0}));
  connect(nakedTabs.port_a, convectionTabs.solid) annotation (Line(points={{84,34},
          {76,34},{76,24},{70,24}}, color={191,0,0}));
  connect(nakedTabs.port_b, convectionTabs.solid) annotation (Line(points={{84,14.2},
          {76,14.2},{76,24},{70,24}}, color={191,0,0}));
  connect(weaDat.weaBus, cTA_VAVReheat.weaBus) annotation (Line(
      points={{-200,228},{122,228},{122,224},{122,220},{122,219.6},{122.552,
          219.6}},
      color={255,204,51},
      thickness=0.5));
  connect(cTA_VAVReheat.port_a, heating.port_a) annotation (Line(points={{113.448,
          182},{113.448,16.353},{141.533,16.353},{141.533,22}},     color={0,127,
          255}));
  connect(cTA_VAVReheat.port_b, heating.port_b) annotation (Line(points={{72.069,
          182},{72.069,16.353},{136.783,16.353},{136.783,22}},      color={0,127,
          255}));
  connect(cTA_VAVReheat.port_a2[1], roo.ports[1]) annotation (Line(points={{10,166},
          {-46,166},{-46,78},{-31,78}},
                                      color={0,127,255}));
  connect(cTA_VAVReheat.port_b1[1], roo.ports[2]) annotation (Line(points={{10,208},
          {-46,208},{-46,82},{-31,82}},
                                      color={0,127,255}));
  connect(cTA_VAVReheat.port_a2[2], roo1.ports[1]) annotation (Line(points={{10,166},
          {-46,166},{-46,28},{-31,28}},
                                      color={0,127,255}));
  connect(cTA_VAVReheat.port_b1[2], roo1.ports[2]) annotation (Line(points={{10,208},
          {-46,208},{-46,32},{-31,32}},
                                      color={0,127,255}));
  connect(TRoo.T, cTA_VAVReheat.TRoo) annotation (Line(points={{40,103},{50,103},
          {50,226.4},{38.9655,226.4}},    color={0,0,127}));
  connect(roo1.surf_conBou[1], TSoi[1].port) annotation (Line(points={{-10,24},{
          -10,24},{-10,6},{8,6}}, color={191,0,0}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-220,-20},{180,
            240}})),
    experiment(StopTime=200000, Interval=900),
    __Dymola_experimentSetupOutput,
    Icon(coordinateSystem(extent={{-220,-20},{180,240}})));
end Boiler_Embedded;
