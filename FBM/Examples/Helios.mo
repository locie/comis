within FBM.Examples;
model Helios "Test for a new helios modelization"
  extends Modelica.Icons.Example;

  package MediumA = Buildings.Media.Air
    "Medium model for air";
    package MediumW = Buildings.Media.Water "Medium model";

parameter Modelica.SIunits.MassFlowRate m0FlowSolpri= 3.9
    "Nominal mass flow rate for the primary loop of solar pannels";
  parameter Modelica.SIunits.MassFlowRate m0FlowSolsec= 3.55
    "Nominal mass flow rate for the second loop of solar pannels";
  parameter Modelica.SIunits.MassFlowRate m0FlowECS= 0.7
    "Nominal mass flow rate for ECS network";
  parameter Modelica.SIunits.MassFlowRate mRadFlow_Eas=0.39;
  parameter Modelica.SIunits.MassFlowRate mRadFlow_NorEas=0.32;
  parameter Modelica.SIunits.MassFlowRate mRadFlow_SouEas=0.28;
  parameter Modelica.SIunits.MassFlowRate mRadFlow_Wes=1.01;
  parameter Modelica.SIunits.MassFlowRate mRadFlow_NorWes=0.26;
  parameter Modelica.SIunits.MassFlowRate mRadFlow_SouWes=0.34;
 // Flow rate for central hot water network
  parameter Modelica.SIunits.MassFlowRate m0RadFlow_Eas=mRadFlow_Eas+ mRadFlow_NorEas + mRadFlow_SouEas + 0.026
    "Nominal mass flow rate for East zone";
  parameter Modelica.SIunits.MassFlowRate m0RadFlow_Wes=mRadFlow_Wes+ mRadFlow_NorWes + mRadFlow_SouWes +0.026
    "Nominal mass flow rate for West zone";
  parameter Modelica.SIunits.MassFlowRate mFlowMun=8.61
    "Nominal mass flow rate for Munters HVACs and batteries networks";
  parameter Modelica.SIunits.MassFlowRate m0RECFlow= 4.55*(2/3)
    "Nominal mass flow rate for the hot water network";
 // Air flow rate for battery network
  parameter Modelica.SIunits.MassFlowRate mAirBat_S_N3=0.90;
  parameter Modelica.SIunits.MassFlowRate mAirBat_E_N3=0.29;
  parameter Modelica.SIunits.MassFlowRate mAirBat_N_N3=0.87;
  parameter Modelica.SIunits.MassFlowRate mAirBat_W_N3=0.80;
  parameter Modelica.SIunits.MassFlowRate mAirBat_N3 = mAirBat_W_N3 + mAirBat_N_N3 + mAirBat_E_N3 + mAirBat_S_N3;
  parameter Modelica.SIunits.MassFlowRate mAirBat_E_N2=1.3;
  parameter Modelica.SIunits.MassFlowRate mAirBat_N_N2=1.83;
  parameter Modelica.SIunits.MassFlowRate mAirBat_W_N2=1.94;
  parameter Modelica.SIunits.MassFlowRate mAirBat_N2 = mAirBat_W_N2 + mAirBat_N_N2 + mAirBat_E_N2;
  parameter Modelica.SIunits.MassFlowRate mAirBat_E_N1=4.39;
  parameter Modelica.SIunits.MassFlowRate mAirBat_W_N1=1.94;
  parameter Modelica.SIunits.MassFlowRate mAirBat_N1 = mAirBat_W_N1 + mAirBat_E_N1;
  parameter Modelica.SIunits.MassFlowRate mAirBat_S_S2=2.3;
  parameter Modelica.SIunits.MassFlowRate mAirBat_E_S2=2.91;
  parameter Modelica.SIunits.MassFlowRate mAirBat_S2 = mAirBat_S_S2 + mAirBat_E_S2;
  parameter Modelica.SIunits.MassFlowRate mAirBat_S_S1=0.95;
  parameter Modelica.SIunits.MassFlowRate mAirBat_E_S1=1.38;
  parameter Modelica.SIunits.MassFlowRate mAirBat_W_S1=2.31;
  parameter Modelica.SIunits.MassFlowRate mAirBat_S1 = mAirBat_S_S1 + mAirBat_E_S1 + mAirBat_W_S1;
   // Water flow rate for battery network
  parameter Modelica.SIunits.MassFlowRate mWatBat_E_N2=0.091;
  parameter Modelica.SIunits.MassFlowRate mWatBat_N_N2=0.175;
  parameter Modelica.SIunits.MassFlowRate mWatBat_W_N2=0.050;
  parameter Modelica.SIunits.MassFlowRate mWatBat_N2 = mWatBat_W_N2 + mWatBat_N_N2 + mWatBat_E_N2;
  parameter Modelica.SIunits.MassFlowRate mWatBat_E_N1=0.201;
  parameter Modelica.SIunits.MassFlowRate mWatBat_W_N1=0.082;
  parameter Modelica.SIunits.MassFlowRate mWatBat_N1 = mWatBat_W_N1 + mWatBat_E_N1;
  parameter Modelica.SIunits.MassFlowRate mWatBat_S_S2=0.184;
  parameter Modelica.SIunits.MassFlowRate mWatBat_E_S2=0.324;
  parameter Modelica.SIunits.MassFlowRate mWatBat_S2 = mWatBat_S_S2 + mWatBat_E_S2;
  parameter Modelica.SIunits.MassFlowRate mWatBat_S_S1=0.071;
  parameter Modelica.SIunits.MassFlowRate mWatBat_E_S1=0.095;
  parameter Modelica.SIunits.MassFlowRate mWatBat_W_S1=0.219;
  parameter Modelica.SIunits.MassFlowRate mWatBat_S1 = mWatBat_S_S1 + mWatBat_E_S1 + mWatBat_W_S1;
  // Water flow rate for munters/menerga hot deck
  parameter Modelica.SIunits.MassFlowRate mHotDeck_N1=0.40;
  parameter Modelica.SIunits.MassFlowRate mHotDeck_N2=2.78;
  parameter Modelica.SIunits.MassFlowRate mHotDeck_N3=0.83;
  parameter Modelica.SIunits.MassFlowRate mHotDeck_S1=0.50;
  parameter Modelica.SIunits.MassFlowRate mHotDeck_S2=1.08;
  // Air flow rate for munters/menerga hvac systems
  parameter Modelica.SIunits.MassFlowRate mAirFlow_N1=3.91;
  parameter Modelica.SIunits.MassFlowRate mAirFlow_N2_Sou=5.43;
  parameter Modelica.SIunits.MassFlowRate mAirFlow_N2_Rep=2.23;
  parameter Modelica.SIunits.MassFlowRate mAirFlow_N3=2.83;
  parameter Modelica.SIunits.MassFlowRate mAirFlow_S1=3.78;
  parameter Modelica.SIunits.MassFlowRate mAirFlow_S2_Sou=3.13;
  parameter Modelica.SIunits.MassFlowRate mAirFlow_S2_Rep=2.45;
  // Regroupement de massFlow pour simplification de l'ecriture du reseau CTA+Bat
  parameter Modelica.SIunits.MassFlowRate Flow_N1= mHotDeck_N1 + mWatBat_N1;
  parameter Modelica.SIunits.MassFlowRate Flow_N2= mHotDeck_N2 + mWatBat_N2;
  parameter Modelica.SIunits.MassFlowRate Flow_S1= mHotDeck_S1 + mWatBat_S1;
  parameter Modelica.SIunits.MassFlowRate Flow_S2= mHotDeck_S2 + mWatBat_S2;
  // Data pour la pompe de recirculation principale
 // Parameter for the boiler loop
 parameter Modelica.SIunits.Power Q_flow_nominal = 450000
    "Nominal power of heating plant";
 parameter Modelica.SIunits.Temperature TSup_nominal=273.15 + 45
    "Nominal supply temperature for radiators";
 parameter Modelica.SIunits.Temperature TRet_nominal=273.15 + 35
    "Nominal return temperature for radiators";
 parameter Modelica.SIunits.Temperature dTBoi_nominal = 20
    "Nominal temperature difference for boiler loop";
 parameter Modelica.SIunits.Pressure dpPip_nominal = 10000
    "Pressure difference of pipe (without valve)";
 parameter Modelica.SIunits.Pressure dpVal_nominal = 1000
    "Pressure difference of valve";
 parameter Modelica.SIunits.Pressure dpRoo_nominal = 10000
    "Pressure difference of flow leg that serves a room";
 parameter Modelica.SIunits.Pressure dpThrWayVal_nominal = 6000
    "Pressure difference of three-way valve";
 parameter Modelica.SIunits.Pressure dp_nominalWest= 28000
    "Pressure difference of West loop";
 parameter Modelica.SIunits.Pressure dp_nominalEast= 21000
    "Pressure difference of East loop";
  parameter Modelica.SIunits.Pressure dp_nominalHVAC= 130000
    "Pressure difference of HVAC loop";
 parameter Modelica.SIunits.Temperature TBoiSup_nominal = 273.15+80
    "Boiler nominal supply water temperature";
 parameter Modelica.SIunits.Temperature TBoiRet_min = 273.15+60
    "Boiler minimum return water temperature";
 parameter Modelica.SIunits.MassFlowRate mBoi_flow_nominal=
    Q_flow_nominal/(4200*dTBoi_nominal) "Boiler nominal mass flow rate";
  HeatingDHWsystems.SubModel.SolarPannel solarPanel(redeclare package Medium =
        MediumW,
    m_flow_nominal=m0FlowSolsec,
    mPrim_flow_nominal=m0FlowSolpri,
    lat(displayUnit="deg") = 0.806999999999,
    T_a2_nominal=273.15 + 60,
    VTan=25,
    hTan=1.9,
    nPar=6,
    nSer=78,
    dpHexPrim=18400,
    dpHexSecon=16600)
    annotation (Placement(transformation(extent={{-90,-92},{-70,-72}})));
  HeatingDHWsystems.SubModel.BoilerRoom boilerRoom(k=3, n=2,
    TSupBoi=273.15 + 90,
    Mass_flow=mBoi_flow_nominal,
    kTank=0.04,
    kPipe=0.04,
    QFLOW(displayUnit="kW") = 200000,
    TSup_nominal=273.15 + 90)                                annotation (
      Placement(transformation(
        extent={{12,-13},{-12,13}},
        rotation=0,
        origin={-18,-103})));
  HeatingDHWsystems.SubModel.HydraunicRadiatorCircuit hydraunicRadiatorCircuit(
    KvReturn=40,
    KvThermostatic=20,
    mWater_flow=1,
    TRoo_nominal=273.15 + 23,
    TSupMin=273.15 + 26,
    m_flow_nominal=mRadFlow_Wes,
    TOut_nominal=273.15 - 10,
    n=2,
    TSupNom=328.15,
    Q_flow_nominal=20000,
    T_a_nominal=328.15,
    T_b_nominal=318.15,
    TAir_nominal=292.15)
    annotation (Placement(transformation(extent={{88,-42},{108,-22}})));
  HeatingDHWsystems.SubModel.HydraunicRadiatorCircuit hydraunicRadiatorCircuit1(
    KvReturn=40,
    KvThermostatic=20,
    mWater_flow=1,
    TRoo_nominal=273.15 + 23,
    TSupMin=273.15 + 26,
    TOut_nominal=273.15 - 10,
    m_flow_nominal=mRadFlow_Eas,
    n=2,
    TSupNom=328.15,
    Q_flow_nominal=20000,
    T_a_nominal=328.15,
    T_b_nominal=318.15,
    TAir_nominal=292.15)
    annotation (Placement(transformation(extent={{114,-42},{134,-22}})));
  BaseClasses.ControlerGenerale controlerGenerale(n=3)
    annotation (Placement(transformation(extent={{-76,-52},{-56,-32}})));
  Buildings.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear val(redeclare
      package Medium = MediumW,
    m_flow_nominal=m0RECFlow,
    CvData=Buildings.Fluid.Types.CvTypes.Kv,
    Kv=200)
    annotation (Placement(transformation(extent={{-26,-74},{-18,-66}})));
  Buildings.Fluid.FixedResistances.Junction jun(redeclare package Medium =
        MediumW,
    m_flow_nominal=m0RECFlow*{1,-1,-1},
    dp_nominal=1000*{1,-1,-1})
    annotation (Placement(transformation(extent={{-14,-74},{-6,-66}})));
  ElementaryBlocs.PumpSupply_m_flow pumpReturn(measureReturnT=true, redeclare
      package Medium = MediumW,
    KvReturn=1,
    m_flow_nominal=m0RECFlow)
    annotation (Placement(transformation(extent={{18,-66},{-2,-86}})));
  Modelica.Blocks.Sources.RealExpression T7(y=pumpReturn.Tsup)
    annotation (Placement(transformation(extent={{-96,-40},{-84,-26}})));
  Modelica.Blocks.Sources.RealExpression T8(y=pumpReturn.Tret)
    annotation (Placement(transformation(extent={{-118,-52},{-106,-38}})));
  BaseClasses.GeneralSetPoint generalSetPoint
    annotation (Placement(transformation(extent={{64,-26},{76,-14}})));
  ElementaryBlocs.CollectorUnit colWest(redeclare package Medium = MediumW,
      m_flow_nominal=m0RECFlow)
    annotation (Placement(transformation(extent={{90,-86},{110,-66}})));
  BaseClasses.Weather weather
    annotation (Placement(transformation(extent={{-116,-26},{-96,-6}})));
  ElementaryBlocs.CollectorUnit colCTA(redeclare package Medium = MediumW,
      m_flow_nominal=m0RECFlow)
    annotation (Placement(transformation(extent={{28,-86},{48,-66}})));
  BaseClasses.PrimPump primPump annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={8,-94})));
  ElementaryBlocs.PumpSupply_m_flow pumpCTA(redeclare package Medium = MediumW,
    m_flow_nominal=1,
    KvReturn=mFlowMun)                      annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={40,-54})));
  BaseClasses.CTA_Controler cTA_Controler
    annotation (Placement(transformation(extent={{4,-28},{16,-16}})));
  Modelica.Blocks.Sources.BooleanExpression OnOff(y=generalSetPoint.OnOff)
    annotation (Placement(transformation(extent={{-22,-32},{-10,-16}})));
  Modelica.Blocks.Sources.BooleanExpression OnOff1(y=generalSetPoint.OnOff)
    annotation (Placement(transformation(
        extent={{-6,-7},{6,7}},
        rotation=90,
        origin={11,-108})));
  ElementaryBlocs.PowerSensor powerSensor(redeclare package Medium = MediumW,
      m_flow_nominal=mFlowMun)            annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={40,-30})));
  BaseClasses.BaseClassesVentilation.S1_batterie s1_batterie
    annotation (Placement(transformation(extent={{-66,66},{-34,94}})));
  BaseClasses.BaseClassesVentilation.S2_Batterie s2_Batterie
    annotation (Placement(transformation(extent={{-66,30},{-34,60}})));
  BaseClasses.BaseClassesVentilation.N1_Batterie n1_Batterie
    annotation (Placement(transformation(extent={{116,28},{150,56}})));
  BaseClasses.BaseClassesVentilation.N3_Batterie n3_Batterie
    annotation (Placement(transformation(extent={{116,64},{150,90}})));
  BaseClasses.BaseClassesVentilation.N2_Batterie n2_Batterie
    annotation (Placement(transformation(extent={{116,98},{150,124}})));
  Buildings.Fluid.Sources.Outside out(nPorts=10, redeclare package Medium =
        MediumA)                                 annotation (Placement(
        transformation(
        extent={{-6,-6},{6,6}},
        rotation=90,
        origin={10,14})));
  ElementaryBlocs.CollectorUnit collectorUnit(redeclare package Medium =
        MediumW, m_flow_nominal=mFlowMun)     annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={40,18})));
  ElementaryBlocs.CollectorUnit collectorUnit1(redeclare package Medium =
        MediumW, m_flow_nominal=mFlowMun - (Flow_N1 + Flow_N2 + mHotDeck_N3))
    annotation (Placement(transformation(extent={{-24,26},{-4,6}})));
  ElementaryBlocs.CollectorUnit collectorUnit2(redeclare package Medium =
        MediumW, m_flow_nominal=Flow_N1 + Flow_N2 + mHotDeck_N3)
                                               annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={40,44})));
  ElementaryBlocs.CollectorUnit collectorUnit3(redeclare package Medium =
        MediumW, m_flow_nominal=Flow_N2 + mHotDeck_N3)
                                               annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={40,74})));
  BaseClasses.HeliosEnvelope heliosEnvelope
    annotation (Placement(transformation(extent={{-22,100},{88,184}})));
  Buildings.Utilities.Math.Average ave(nin=3)
    annotation (Placement(transformation(extent={{30,-110},{20,-100}})));
  Buildings.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature
    annotation (Placement(transformation(extent={{-50,-118},{-58,-110}})));
  Modelica.Blocks.Routing.Multiplex3 multiplex3_1
    annotation (Placement(transformation(extent={{80,-64},{74,-58}})));
  ElementaryBlocs.MixingCircuit_Tset mixingCircuit_Tset(
    KvReturn=40,
    redeclare package Medium = MediumW,
    m_flow_nominal=mFlowMun)
                      annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={40,-6})));
  Buildings.Fluid.FixedResistances.Junction jun1(
    redeclare package Medium = MediumA,
    m_flow_nominal=mAirFlow_N2_Rep*{1,-1,1},
    dp_nominal=20*{1,-1,1})
    annotation (Placement(transformation(extent={{118,132},{126,124}})));
  Buildings.Fluid.FixedResistances.Junction jun2(
    redeclare package Medium = MediumA,
    m_flow_nominal=mAirFlow_N3*{1,-1,1},
    dp_nominal=20*{1,-1,1})
    annotation (Placement(transformation(extent={{94,96},{102,88}})));
  Buildings.Fluid.FixedResistances.Junction jun3(
    redeclare package Medium = MediumA,
    m_flow_nominal=mAirFlow_N3*{1,-1,1},
    dp_nominal=20*{1,-1,1})
    annotation (Placement(transformation(extent={{104,96},{112,88}})));
  Buildings.Fluid.FixedResistances.Junction jun4(
    redeclare package Medium = MediumA,
    m_flow_nominal=mAirFlow_N1*{1,-1,1},
    dp_nominal=20*{1,-1,1})
    annotation (Placement(transformation(extent={{96,62},{104,54}})));
  Buildings.Fluid.FixedResistances.Junction jun5(
    redeclare package Medium = MediumA,
    m_flow_nominal=mAirFlow_S1*{1,-1,1},
    dp_nominal=20*{1,-1,1})
    annotation (Placement(transformation(extent={{-38,104},{-46,96}})));
  Buildings.Fluid.FixedResistances.Junction jun6(
    redeclare package Medium = MediumA,
    m_flow_nominal=mAirFlow_S1*{1,-1,1},
    dp_nominal=20*{1,-1,1})
    annotation (Placement(transformation(extent={{-26,104},{-34,96}})));
  Buildings.Fluid.FixedResistances.Junction jun7(
    redeclare package Medium = MediumA,
    m_flow_nominal={1,-1,1}*mAirFlow_S2_Rep,
    dp_nominal=20*{1,-1,1})
    annotation (Placement(transformation(extent={{-18,68},{-26,60}})));
  Modelica.Blocks.Math.BooleanToReal booleanToReal(realTrue=mFlowMun)
    annotation (Placement(transformation(extent={{8,-58},{16,-50}})));
  Buildings.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear val1(
                                                                     redeclare
      package Medium = MediumW,
    m_flow_nominal=m0RECFlow,
    CvData=Buildings.Fluid.Types.CvTypes.Kv,
    Kv=200)
    annotation (Placement(transformation(extent={{-34,-78},{-42,-86}})));
  Buildings.Fluid.FixedResistances.Junction jun8(
                                                redeclare package Medium =
        MediumW,
    m_flow_nominal=m0RECFlow*{1,-1,-1},
    dp_nominal=1000*{1,-1,-1})
    annotation (Placement(transformation(extent={{-42,-74},{-34,-66}})));
  Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage val3(
    redeclare package Medium = MediumW,
    m_flow_nominal=m0RECFlow,
    CvData=Buildings.Fluid.Types.CvTypes.Kv,
    Kv=200,
    dpFixed_nominal=2000)
    annotation (Placement(transformation(extent={{-64,-74},{-56,-66}})));
  Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage val4(
    redeclare package Medium = MediumW,
    m_flow_nominal=m0RECFlow,
    CvData=Buildings.Fluid.Types.CvTypes.Kv,
    Kv=200,
    dpFixed_nominal=2000)
    annotation (Placement(transformation(extent={{-46,-86},{-54,-78}})));
  Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage val2(
    redeclare package Medium = MediumW,
    m_flow_nominal=m0RECFlow,
    CvData=Buildings.Fluid.Types.CvTypes.Kv,
    Kv=200,
    dpFixed_nominal=2000) annotation (Placement(transformation(
        extent={{4,-4},{-4,4}},
        rotation=90,
        origin={-22,-80})));
  Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage val5(
    redeclare package Medium = MediumW,
    m_flow_nominal=m0RECFlow,
    CvData=Buildings.Fluid.Types.CvTypes.Kv,
    Kv=200,
    dpFixed_nominal=2000) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={-10,-80})));
equation
  connect(val.port_2, jun.port_1) annotation (Line(points={{-18,-70},{-18,-70},
          {-14,-70}}, color={0,127,255}));
  connect(jun.port_2, pumpReturn.port_a2)
    annotation (Line(points={{-6,-70},{-2,-70}}, color={0,127,255}));
  connect(controlerGenerale.V3V3, val.y) annotation (Line(
      points={{-55,-33.6},{-55,-33.6},{-22,-33.6},{-22,-65.2}},
      color={0,0,127},
      pattern=LinePattern.Dash));

  connect(generalSetPoint.y, hydraunicRadiatorCircuit.TSetPoint) annotation (
      Line(
      points={{76.6,-20.12},{78.35,-20.12},{78.35,-24},{87.4,-24}},
      color={0,0,127},
      pattern=LinePattern.Dash));
  connect(generalSetPoint.y, hydraunicRadiatorCircuit1.TSetPoint) annotation (
      Line(
      points={{76.6,-20.12},{112.3,-20.12},{112.3,-24},{113.4,-24}},
      color={0,0,127},
      pattern=LinePattern.Dash));
  connect(hydraunicRadiatorCircuit.port_a, colWest.port_b3)
    annotation (Line(points={{94,-42},{94,-66}}, color={0,127,255}));
  connect(hydraunicRadiatorCircuit.port_b, colWest.port_a3) annotation (Line(
        points={{102,-42},{104,-42},{104,-86.4},{106,-86.4}}, color={0,127,255}));
  connect(weather.weaBus, generalSetPoint.weaBus) annotation (Line(
      points={{-100,-11},{62,-11},{62,-15.56},{65.56,-15.56}},
      color={255,204,51},
      thickness=0.5));
  connect(weather.weaBus, hydraunicRadiatorCircuit.weaBus) annotation (Line(
      points={{-100,-11},{94,-11},{94,-26},{93,-26}},
      color={255,204,51},
      thickness=0.5));
  connect(weather.weaBus, hydraunicRadiatorCircuit1.weaBus) annotation (Line(
      points={{-100,-11},{120,-11},{120,-26},{119,-26}},
      color={255,204,51},
      thickness=0.5));
  connect(weather.weaBus, solarPanel.weaBus) annotation (Line(
      points={{-100,-11},{-100,-73.8},{-87.6,-73.8}},
      color={255,204,51},
      thickness=0.5));
  connect(colCTA.port_b1, colWest.port_a1)
    annotation (Line(points={{48,-70},{48,-70},{90,-70}}, color={0,127,255}));
  connect(colCTA.port_a2, colWest.port_b2)
    annotation (Line(points={{48,-82},{48,-82},{90,-82}}, color={0,127,255}));
  connect(pumpReturn.port_b2, colCTA.port_a1)
    annotation (Line(points={{18,-70},{28,-70}}, color={0,127,255}));
  connect(pumpReturn.port_a1, colCTA.port_b2)
    annotation (Line(points={{18,-82},{28,-82}}, color={0,127,255}));
  connect(primPump.y, pumpReturn.u)
    annotation (Line(points={{8,-89.6},{8,-86.8}},           color={0,0,127}));
  connect(pumpCTA.port_a1, colCTA.port_b3) annotation (Line(points={{34,-64},{34,
          -64},{34,-66},{32,-66}},    color={0,127,255}));
  connect(pumpCTA.port_b2, colCTA.port_a3) annotation (Line(points={{46,-64},{46,
          -64},{46,-86.4},{44,-86.4}},    color={0,127,255}));
  connect(generalSetPoint.OnOff, hydraunicRadiatorCircuit.OnOffPump)
    annotation (Line(
      points={{76.6,-23.12},{76.3,-23.12},{76.3,-33.4},{87.6,-33.4}},
      color={255,0,255},
      pattern=LinePattern.Dash));
  connect(generalSetPoint.OnOff, hydraunicRadiatorCircuit1.OnOffPump)
    annotation (Line(
      points={{76.6,-23.12},{109.3,-23.12},{109.3,-33.4},{113.6,-33.4}},
      color={255,0,255},
      pattern=LinePattern.Dash));
  connect(weather.weaBus, cTA_Controler.weaBus) annotation (Line(
      points={{-100,-11},{4,-11},{4,-17.56},{5.56,-17.56}},
      color={255,204,51},
      thickness=0.5));
  connect(OnOff.y, cTA_Controler.OnOff) annotation (Line(points={{-9.4,-24},{4,-24},
          {4,-24.52}},      color={255,0,255}));
  connect(primPump.OnOff, OnOff1.y) annotation (Line(points={{11.2,-98},{11,-98},
          {11,-101.4}},       color={255,0,255}));
  connect(powerSensor.port_a1, pumpCTA.port_b1)
    annotation (Line(points={{34,-40},{34,-44}},          color={0,127,255}));
  connect(powerSensor.port_b2, pumpCTA.port_a2)
    annotation (Line(points={{46,-40},{46,-44}}, color={0,127,255}));
  connect(weather.weaBus, s2_Batterie.weaBus) annotation (Line(
      points={{-100,-11},{-68,-11},{-68,56.76},{-61.8154,56.76}},
      color={255,204,51},
      thickness=0.5));
  connect(weather.weaBus, s1_batterie.weaBus) annotation (Line(
      points={{-100,-11},{-68,-11},{-68,89.8519},{-62.2839,89.8519}},
      color={255,204,51},
      thickness=0.5));
  connect(weather.weaBus, n3_Batterie.weaBus) annotation (Line(
      points={{-100,-11},{114,-11},{114,86.4774},{120.954,86.4774}},
      color={255,204,51},
      thickness=0.5));
  connect(weather.weaBus, n2_Batterie.weaBus) annotation (Line(
      points={{-100,-11},{114,-11},{114,120.477},{120.954,120.477}},
      color={255,204,51},
      thickness=0.5));
  connect(weather.weaBus, n1_Batterie.weaBus) annotation (Line(
      points={{-100,-11},{114,-11},{114,52.8769},{118.931,52.8769}},
      color={255,204,51},
      thickness=0.5));
  connect(weather.weaBus, out.weaBus) annotation (Line(
      points={{-100,-11},{10,-11},{10,8},{9.88,8}},
      color={255,204,51},
      thickness=0.5));
  connect(s2_Batterie.FreshAir, out.ports[1]) annotation (Line(points={{-66,
          44.04},{-72,44.04},{-72,20},{7.84,20}}, color={0,140,72}));
  connect(s2_Batterie.ExhaustAit, out.ports[2]) annotation (Line(points={{-66,
          45.48},{-72,45.48},{-72,20},{8.32,20}}, color={0,140,72}));
  connect(s1_batterie.FreshAir, out.ports[3]) annotation (Line(points={{
          -33.7935,78.4444},{-27.8968,78.4444},{-27.8968,20},{8.8,20}}, color={
          0,140,72}));
  connect(s1_batterie.ExhaustAit, out.ports[4]) annotation (Line(points={{
          -33.8968,83.1111},{-33.8968,84},{-28,84},{-28,20},{9.28,20}}, color={
          0,140,72}));
  connect(out.ports[5], n3_Batterie.FreshAir) annotation (Line(points={{9.76,20},
          {112,20},{112,74.9871},{116,74.9871}}, color={0,140,72}));
  connect(n3_Batterie.ExhaustAit, out.ports[6]) annotation (Line(points={{116,
          78.1742},{112,78.1742},{112,20},{10.24,20}}, color={0,140,72}));
  connect(out.ports[7], n2_Batterie.FreshAir) annotation (Line(points={{10.72,
          20},{112,20},{112,108.987},{116,108.987}}, color={0,140,72}));
  connect(n2_Batterie.ExhaustAit, out.ports[8]) annotation (Line(points={{116,
          112.174},{112,112.174},{112,20},{11.2,20}}, color={0,140,72}));
  connect(out.ports[9], n1_Batterie.FreshAir) annotation (Line(points={{11.68,
          20},{112,20},{152,20},{152,41.0308},{150,41.0308}},          color={0,
          140,72}));
  connect(out.ports[10], n1_Batterie.ExhaustAit) annotation (Line(points={{12.16,
          20},{112,20},{152,20},{152,45.1231},{150,45.1231}},
        color={0,140,72}));
  connect(collectorUnit1.port_b1, collectorUnit.port_b3)
    annotation (Line(points={{-4,10},{14,10},{14,12},{30,12}},
                                                       color={0,127,255}));
  connect(collectorUnit1.port_a1, s2_Batterie.HotWater) annotation (Line(points={{-24,10},
          {-56,10},{-56,30},{-54.9231,30}},          color={0,127,255}));
  connect(collectorUnit1.port_b3, s1_batterie.HotWater) annotation (Line(points={{-20,6},
          {-40,6},{-40,65.6889},{-39.2645,65.6889}},             color={0,127,
          255}));
  connect(s2_Batterie.ColdWater, collectorUnit1.port_b2) annotation (Line(
        points={{-58.8615,29.88},{-58.4308,29.88},{-58.4308,22},{-24,22}},
                                                                         color=
          {0,127,255}));
  connect(collectorUnit1.port_a3, s1_batterie.ColdWater) annotation (Line(
        points={{-8,26.4},{-26,26.4},{-26,26},{-42.671,26},{-42.671,65.6889}},
        color={0,127,255}));
  connect(collectorUnit1.port_a2, collectorUnit.port_a3)
    annotation (Line(points={{-4,22},{50.4,22},{50.4,24}},
                                               color={0,127,255}));
  connect(collectorUnit2.port_a1, collectorUnit.port_b1)
    annotation (Line(points={{34,34},{34,28}},         color={0,127,255}));
  connect(collectorUnit2.port_b2, collectorUnit.port_a2)
    annotation (Line(points={{46,34},{46,28}},         color={0,127,255}));
  connect(collectorUnit2.port_b3, n1_Batterie.HotWater) annotation (Line(points={{30,38},
          {144.255,38},{144.255,27.6769}},                           color={0,
          127,255}));
  connect(collectorUnit2.port_a3, n1_Batterie.ColdWater) annotation (Line(
        points={{50.4,50},{50,50},{50,27.6769},{140.738,27.6769}}, color={0,127,
          255}));
  connect(collectorUnit3.port_a1, collectorUnit2.port_b1)
    annotation (Line(points={{34,64},{34,54}},         color={0,127,255}));
  connect(collectorUnit3.port_b2, collectorUnit2.port_a2)
    annotation (Line(points={{46,64},{46,54}}, color={0,127,255}));
  connect(n3_Batterie.ColdWater, collectorUnit3.port_b3) annotation (Line(
        points={{125.617,64},{125.617,68},{30,68}}, color={0,127,255}));
  connect(collectorUnit3.port_a3, n3_Batterie.HotWater) annotation (Line(points={{50.4,80},
          {50,80},{50,64},{122.217,64}},           color={0,127,255}));
  connect(collectorUnit3.port_b1, n2_Batterie.HotWater) annotation (Line(points={{34,84},
          {34,84},{34,98},{125.229,98}},         color={0,127,255}));
  connect(collectorUnit3.port_a2, n2_Batterie.ColdWater) annotation (Line(
        points={{46,84},{46,84},{46,97.9161},{122.217,97.9161}}, color={0,127,
          255}));
  connect(heliosEnvelope.TLabo, n2_Batterie.TRoom) annotation (Line(points={{91.0556,
          179.333},{91.0556,185.733},{150.971,185.733},{150.971,122.239}},
        color={0,0,127}));
  connect(heliosEnvelope.TLabo, n1_Batterie.TRoom) annotation (Line(points={{91.0556,
          179.333},{91.0556,185.734},{151.172,185.734},{151.172,52.7692}},
        color={0,0,127}));
  connect(heliosEnvelope.TDesktop, n3_Batterie.TRoom) annotation (Line(points={{91.0556,
          164.867},{91.0556,172.433},{150.971,172.433},{150.971,88.2387}},
        color={0,0,127}));
  connect(heliosEnvelope.TLabo, s2_Batterie.TRoom) annotation (Line(points={{91.0556,
          179.333},{91.0556,195.734},{-32.7692,195.734},{-32.7692,57.12}},
        color={0,0,127}));
  connect(heliosEnvelope.TLabo, s1_batterie.TRoom) annotation (Line(points={{91.0556,
          179.333},{91.0556,196},{-32.9677,196},{-32.9677,91.5111}}, color={0,0,
          127}));
  connect(weather.weaBus, heliosEnvelope.weaBus) annotation (Line(
      points={{-100,-11},{-100,153.667},{31.7778,153.667}},
      color={255,204,51},
      thickness=0.5));

  connect(heliosEnvelope.TDesktop, hydraunicRadiatorCircuit1.Tair) annotation (
      Line(
      points={{91.0556,164.867},{174,164.867},{174,-48},{113.2,-48},{113.2,-40}},
      color={0,0,127},
      pattern=LinePattern.Dash));

  connect(heliosEnvelope.TDesktop, hydraunicRadiatorCircuit.Tair) annotation (
      Line(
      points={{91.0556,164.867},{174,164.867},{174,-48},{87.2,-48},{87.2,-40}},
      color={0,0,127},
      pattern=LinePattern.Dash));

  connect(boilerRoom.Tair, ave.y) annotation (Line(
      points={{-5.52,-105.6},{4,-105.6},{4,-105},{19.5,-105}},
      color={0,0,127},
      pattern=LinePattern.Dash));
  connect(heliosEnvelope.TDesktop, ave.u) annotation (Line(
      points={{91.0556,164.867},{170,164.867},{170,-105},{31,-105}},
      color={0,0,127},
      pattern=LinePattern.Dash));
  connect(prescribedTemperature.T, ave.y) annotation (Line(
      points={{-49.2,-114},{-48,-114},{-48,-118},{20,-118},{20,-110},{19.5,-110},
          {19.5,-105}},
      color={0,0,127},
      pattern=LinePattern.Dash));
  connect(prescribedTemperature.port, solarPanel.heatPort) annotation (Line(
        points={{-58,-114},{-68,-114},{-68,-72.2},{-80,-72.2}}, color={191,0,0}));
  connect(heliosEnvelope.WestConv, hydraunicRadiatorCircuit.heatPortCon[1])
    annotation (Line(points={{-12.2222,161.6},{-12.2222,27.8},{96,27.8},{96,
          -22.5}},
        color={191,0,0}));
  connect(hydraunicRadiatorCircuit.heatPortRad[1], heliosEnvelope.WestRad)
    annotation (Line(points={{100,-22.5},{100,-22.5},{100,28},{-11.6111,28},{
          -11.6111,138.267}},
                     color={191,0,0}));
  connect(hydraunicRadiatorCircuit.heatPortRad[2], heliosEnvelope.SouthRad)
    annotation (Line(points={{100,-21.5},{100,28},{50.7222,28},{50.7222,105.133}},
        color={191,0,0}));
  connect(hydraunicRadiatorCircuit.heatPortCon[2], heliosEnvelope.SouthConv)
    annotation (Line(points={{96,-21.5},{96,-21.5},{96,28},{14.6667,28},{
          14.6667,105.133}},
                     color={191,0,0}));
  connect(heliosEnvelope.EastConve, hydraunicRadiatorCircuit1.heatPortCon[1])
    annotation (Line(points={{79.4444,160.667},{108,160.667},{108,-6},{122,-6},
          {122,-22.5}},color={191,0,0}));
  connect(heliosEnvelope.EastRadia, hydraunicRadiatorCircuit1.heatPortRad[1])
    annotation (Line(points={{80.0556,138.267},{108,138.267},{108,-6},{126,-6},
          {126,-22.5}},color={191,0,0}));
  connect(heliosEnvelope.SouthConv, hydraunicRadiatorCircuit1.heatPortCon[2])
    annotation (Line(points={{14.6667,105.133},{14.6667,27.5665},{122,27.5665},
          {122,-21.5}},color={191,0,0}));
  connect(heliosEnvelope.SouthRad, hydraunicRadiatorCircuit1.heatPortRad[2])
    annotation (Line(points={{50.7222,105.133},{50.7222,27.5665},{126,27.5665},
          {126,-21.5}},color={191,0,0}));
  connect(colWest.port_b1, hydraunicRadiatorCircuit1.port_a) annotation (Line(
        points={{110,-70},{120,-70},{120,-42}}, color={0,127,255}));
  connect(colWest.port_a2, hydraunicRadiatorCircuit1.port_b) annotation (Line(
        points={{110,-82},{128,-82},{128,-42}}, color={0,127,255}));
  connect(hydraunicRadiatorCircuit.HeatCurve, multiplex3_1.u1[1]) annotation (
      Line(
      points={{109,-35},{110.5,-35},{110.5,-58.9},{80.6,-58.9}},
      color={0,0,127},
      pattern=LinePattern.Dash));
  connect(hydraunicRadiatorCircuit1.HeatCurve, multiplex3_1.u2[1]) annotation (
      Line(
      points={{135,-35},{139.5,-35},{139.5,-61},{80.6,-61}},
      color={0,0,127},
      pattern=LinePattern.Dash));
  connect(cTA_Controler.TCurve, multiplex3_1.u3[1]) annotation (Line(
      points={{16.6,-22},{20,-22},{20,-63.1},{80.6,-63.1}},
      color={0,0,127},
      pattern=LinePattern.Dash));
  connect(multiplex3_1.y, controlerGenerale.TConsigne) annotation (Line(
      points={{73.7,-61},{-2,-61},{-2,-64},{-40,-64},{-40,-51},{-77.6,-51}},
      color={0,0,127},
      pattern=LinePattern.Dash));
  connect(solarPanel.TBottom, controlerGenerale.Tbb) annotation (Line(
      points={{-91,-88},{-96,-88},{-96,-40},{-77,-40}},
      color={0,0,127},
      pattern=LinePattern.Dash));
  connect(mixingCircuit_Tset.port_a1, powerSensor.port_b1)
    annotation (Line(points={{34,-16},{34,-20}}, color={0,127,255}));
  connect(mixingCircuit_Tset.port_b2, powerSensor.port_a2)
    annotation (Line(points={{46,-16},{46,-20}},          color={0,127,255}));
  connect(collectorUnit.port_a1, mixingCircuit_Tset.port_b1)
    annotation (Line(points={{34,8},{34,4}},        color={0,127,255}));
  connect(collectorUnit.port_b2, mixingCircuit_Tset.port_a2)
    annotation (Line(points={{46,8},{46,4}},        color={0,127,255}));
  connect(cTA_Controler.TCurve, mixingCircuit_Tset.TMixedSet) annotation (Line(
      points={{16.6,-22},{20,-22},{20,-6},{30,-6}},
      color={0,0,127},
      pattern=LinePattern.Dash));
  connect(multiplex3_1.y, boilerRoom.TWatSupply) annotation (Line(
      points={{73.7,-61},{32.85,-61},{32.85,-113.66},{-5.52,-113.66}},
      color={0,0,127},
      pattern=LinePattern.Dash));
  connect(heliosEnvelope.S2_South, s2_Batterie.SouthS2_Out) annotation (Line(
        points={{-1.22222,114.467},{-1.22222,63.2335},{-38.8,63.2335},{-38.8,60}},
        color={0,140,72},
      pattern=LinePattern.Dash));
  connect(heliosEnvelope.S2_Eas, s2_Batterie.EastS2_Out) annotation (Line(
        points={{59.8889,154.133},{66.9445,154.133},{66.9445,60},{-47.0462,60}},
        color={0,140,72},
      pattern=LinePattern.Dash));
  connect(heliosEnvelope.S1_South, s1_batterie.SouthS1_Out) annotation (Line(
        points={{15.8889,114},{-58,114},{-58,94},{-57.5355,94}}, color={0,140,72},
      pattern=LinePattern.Dash));

  connect(heliosEnvelope.S1_Eas, s1_batterie.EastS1_Out) annotation (Line(
        points={{60.5,162.067},{-52.75,162.067},{-52.75,94},{-53.5097,94}},
        color={0,140,72},
      pattern=LinePattern.Dash));
  connect(heliosEnvelope.S1_West, s1_batterie.WestS1_Out) annotation (Line(
        points={{7.94444,156.933},{7.94444,157.467},{-50.1032,157.467},{
          -50.1032,94}},
                color={0,140,72},
      pattern=LinePattern.Dash));
  connect(heliosEnvelope.N2_Eas, n2_Batterie.EastN2_Out) annotation (Line(
        points={{59.2778,177.467},{132.639,177.467},{132.639,123.916},{133.486,
          123.916}},
        color={0,140,72},
      pattern=LinePattern.Dash));
  connect(n2_Batterie.WestN2_Out, heliosEnvelope.N2_West) annotation (Line(
        points={{138.343,124},{138,124},{138,177.467},{7.33333,177.467}}, color={0,140,72},
      pattern=LinePattern.Dash));

  connect(n3_Batterie.SouthN3_Out, heliosEnvelope.SouIn) annotation (Line(
        points={{133.583,90},{134,90},{134,104.2},{-11.6111,104.2}}, color={0,140,72},
      pattern=LinePattern.Dash));

  connect(n3_Batterie.EastN3_Out, heliosEnvelope.EasIn) annotation (Line(points={{140.091,
          90},{140,90},{140,177.933},{79.4444,177.933}},          color={0,140,72},
      pattern=LinePattern.Dash));

  connect(n3_Batterie.WestN3_Out, heliosEnvelope.WesIn) annotation (Line(points={{145.531,
          90},{146,90},{146,177.933},{-11.6111,177.933}},          color={0,140,72},
      pattern=LinePattern.Dash));

  connect(n1_Batterie.EastN1_Out, heliosEnvelope.N1_West) annotation (Line(
        points={{126.786,56},{68,56},{68,165.333},{7.94444,165.333}}, color={0,140,72},
      pattern=LinePattern.Dash));

  connect(heliosEnvelope.N1_Eas, n1_Batterie.WestN1_Out) annotation (Line(
        points={{59.8889,170.467},{59.8889,66.234},{130.89,66.234},{130.89,56}},
        color={0,140,72},
      pattern=LinePattern.Dash));
  connect(heliosEnvelope.N2_eas, jun1.port_1) annotation (Line(points={{60.5,146.2},
          {101.25,146.2},{101.25,128},{118,128}}, color={0,140,72},
      pattern=LinePattern.Dash));
  connect(jun1.port_2, n2_Batterie.ExtractAir) annotation (Line(points={{126,128},
          {130,128},{130,123.916},{129.211,123.916}}, color={0,127,255}));
  connect(heliosEnvelope.N2_west, jun1.port_3) annotation (Line(points={{6.72222,
          124.733},{12.3611,124.733},{12.3611,132},{122,132}}, color={0,140,72},
      pattern=LinePattern.Dash));

  connect(jun3.port_1, jun2.port_2)
    annotation (Line(points={{104,92},{104,92},{102,92}}, color={0,127,255}));
  connect(jun3.port_2, n3_Batterie.ExtractAir) annotation (Line(points={{112,92},
          {130,92},{130,89.9161},{129.211,89.9161}}, color={0,140,72},
      pattern=LinePattern.Dash));
  connect(heliosEnvelope.SouOut, jun2.port_1) annotation (Line(points={{80.6667,
          103.733},{81.3333,103.733},{81.3333,92},{94,92}}, color={0,140,72},
      pattern=LinePattern.Dash));
  connect(heliosEnvelope.EasOut, jun2.port_3) annotation (Line(points={{79.4444,
          125.2},{79.4444,124.6},{98,124.6},{98,96}}, color={0,140,72},
      pattern=LinePattern.Dash));
  connect(heliosEnvelope.WesOut, jun3.port_3) annotation (Line(points={{
          -12.8333,123.8},{107.583,123.8},{107.583,96},{108,96}},
                                                         color={0,140,72},
      pattern=LinePattern.Dash));
  connect(jun4.port_2, n1_Batterie.ExtractAir) annotation (Line(points={{104,58},
          {138,58},{138,56},{138.159,56}}, color={0,140,72},
      pattern=LinePattern.Dash));
  connect(heliosEnvelope.N1_west, jun4.port_1) annotation (Line(points={{6.72222,
          135.933},{51.3611,135.933},{51.3611,58},{96,58}}, color={0,140,72},
      pattern=LinePattern.Dash));
  connect(heliosEnvelope.N1_eas, jun4.port_3) annotation (Line(points={{60.5,
          138.733},{60.5,74.367},{100,74.367},{100,62}},
                                                color={0,140,72},
      pattern=LinePattern.Dash));
  connect(jun5.port_1, jun6.port_2)
    annotation (Line(points={{-38,100},{-34,100}}, color={0,127,255}));
  connect(jun5.port_2, s1_batterie.ExtractAir) annotation (Line(points={{-46,100},
          {-46,93.8963},{-45.4581,93.8963}}, color={0,127,255}));
  connect(heliosEnvelope.S1_west, jun6.port_1) annotation (Line(points={{6.11111,
          143.867},{6.11111,121.934},{-26,121.934},{-26,100}}, color={0,140,72},
      pattern=LinePattern.Dash));

  connect(heliosEnvelope.S1_south, jun6.port_3) annotation (Line(points={{72.1111,
          114},{72,114},{72,104},{-30,104}}, color={0,140,72},
      pattern=LinePattern.Dash));
  connect(heliosEnvelope.S1_eas, jun5.port_3) annotation (Line(points={{60.5,130.8},
          {9.25,130.8},{9.25,104},{-42,104}}, color={0,140,72},
      pattern=LinePattern.Dash));
  connect(s2_Batterie.ExtractAir, jun7.port_2) annotation (Line(points={{
          -52.3385,60},{-52,60},{-52,64},{-26,64}},
                                           color={0,140,72},
      pattern=LinePattern.Dash));
  connect(heliosEnvelope.S2_south, jun7.port_1) annotation (Line(points={{50.7222,
          114.467},{16.3611,114.467},{16.3611,64},{-18,64}}, color={0,140,72},
      pattern=LinePattern.Dash));
  connect(heliosEnvelope.S2_eas, jun7.port_3) annotation (Line(points={{60.5,
          123.333},{19.25,123.333},{19.25,68},{-22,68}},
                                                color={0,140,72},
      pattern=LinePattern.Dash));
  connect(booleanToReal.y, pumpCTA.u)
    annotation (Line(points={{16.4,-54},{29.2,-54}}, color={0,0,127}));
  connect(OnOff.y, booleanToReal.u) annotation (Line(points={{-9.4,-24},{-2,-24},
          {-2,-54},{7.2,-54}}, color={255,0,255}));
  connect(controlerGenerale.y1, boilerRoom.u) annotation (Line(points={{-61.9,-53.7},
          {-4.95,-53.7},{-4.95,-92.6},{-5.04,-92.6}}, color={255,0,255}));
  connect(T8.y, boilerRoom.T8B) annotation (Line(points={{-105.4,-45},{-101.7,
          -45},{-101.7,-94.42},{-30.48,-94.42}}, color={0,0,127}));
  connect(val1.port_1, pumpReturn.port_b1) annotation (Line(points={{-34,-82},{
          -20,-82},{-2,-82}}, color={0,127,255}));
  connect(jun8.port_2, val.port_1)
    annotation (Line(points={{-34,-70},{-26,-70}}, color={0,127,255}));
  connect(jun8.port_3, val1.port_3) annotation (Line(points={{-38,-74},{-38,-76},
          {-38,-78}}, color={0,127,255}));
  connect(controlerGenerale.V3V2, val1.y) annotation (Line(points={{-55,-39.6},
          {-55,-89.8},{-38,-89.8},{-38,-86.8}}, color={0,0,127}));
  connect(solarPanel.port_a, val3.port_a) annotation (Line(points={{-90,-82},{
          -98,-82},{-98,-70},{-64,-70}}, color={0,127,255}));
  connect(val3.port_b, jun8.port_1)
    annotation (Line(points={{-56,-70},{-42,-70}}, color={0,127,255}));
  connect(val1.port_2, val4.port_a)
    annotation (Line(points={{-42,-82},{-46,-82}}, color={0,127,255}));
  connect(val4.port_b, solarPanel.port_b)
    annotation (Line(points={{-54,-82},{-70,-82}}, color={0,127,255}));
  connect(val.port_3, val2.port_a) annotation (Line(points={{-22,-74},{-22,-75},
          {-22,-76}}, color={0,127,255}));
  connect(val2.port_b, boilerRoom.port_b) annotation (Line(points={{-22,-84},{
          -22,-90},{-20.4,-90}}, color={0,127,255}));
  connect(boilerRoom.port_a, val5.port_a) annotation (Line(points={{-15.6,-90},
          {-10,-90},{-10,-84}}, color={0,127,255}));
  connect(jun.port_3, val5.port_b) annotation (Line(points={{-10,-74},{-10,-75},
          {-10,-76}}, color={0,127,255}));
  connect(controlerGenerale.BOI, val5.y) annotation (Line(points={{-55,-44.6},{
          -16.5,-44.6},{-16.5,-80},{-14.8,-80}}, color={0,0,127}));
  connect(controlerGenerale.BOI, val2.y) annotation (Line(points={{-55,-44.6},{
          -55,-44.3},{-26.8,-44.3},{-26.8,-80}}, color={0,0,127}));
  connect(T7.y, controlerGenerale.T7) annotation (Line(points={{-83.4,-33},{
          -80.7,-33},{-80.7,-34.1},{-77.1,-34.1}}, color={0,0,127}));
  connect(T8.y, controlerGenerale.T8) annotation (Line(points={{-105.4,-45},{
          -80.7,-45},{-80.7,-46},{-77,-46}}, color={0,0,127}));
  connect(controlerGenerale.SUN, val4.y) annotation (Line(points={{-55,-49},{
          -55,-49.5},{-50,-49.5},{-50,-77.2}}, color={0,0,127}));
  connect(val3.y, controlerGenerale.SUN) annotation (Line(points={{-60,-65.2},{
          -50,-65.2},{-50,-49},{-55,-49}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},
            {180,200}})),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},{180,200}})));
end Helios;
