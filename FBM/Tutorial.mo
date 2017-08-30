within FBM;
package Tutorial "Models for website tutorial"
   extends Modelica.Icons.ExamplesPackage;
  model FirstTuto "Simple boiler consumption calibration"
    extends Modelica.Icons.Example;
    package MediumA = Buildings.Media.Air
      "Medium model for air";
    package MediumW = Buildings.Media.Water "Medium model";
    parameter Modelica.SIunits.Temperature TRet_Min= 273.15+60;
    parameter Modelica.SIunits.Temperature TSup_nominal=273.15 + 45;
    parameter Modelica.SIunits.Temperature TSupBoi= 273.15+80;
    parameter Modelica.SIunits.MassFlowRate Mass_flow = 0.25;
    parameter Real eff=0.84;
    parameter Modelica.SIunits.Power QFLOW=5500 "Nominal heating power";
    parameter Modelica.SIunits.Volume VWat = 1.5E-6*QFLOW
      "Water volume of boiler"
      annotation(Dialog(tab = "Dynamics", enable = not (energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState)));
    parameter Modelica.SIunits.Mass mDry =   1.5E-3*QFLOW
      "Mass of boiler that will be lumped to water heat capacity"
      annotation(Dialog(tab = "Dynamics", enable = not (energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState)));
    parameter Modelica.SIunits.PressureDifference dpBoiler = 6000;
    parameter Real kTank=0.4;
    parameter Real kPipe=0.4;
    ElementaryBlocs.MixingCircuit_Tset mixingCircuit_Tset(
      redeclare package Medium = Medium,
      m_flow_nominal=Mass_flow,
      reverseAction=true,
      KvReturn=82)           annotation (Placement(transformation(
          extent={{-9,9},{9,-9}},
          rotation=90,
          origin={-29,-29})));
    Buildings.Fluid.Boilers.BoilerPolynomial boiler(
      effCur=Buildings.Fluid.Types.EfficiencyCurves.Constant,
      redeclare package Medium = Medium,
      T_nominal=TSupBoi,
      m_flow_nominal=Mass_flow,
      fue=datFue,
      a={eff},
      Q_flow_nominal=QFLOW,
      VWat=VWat,
      mDry=mDry,
      dp_nominal=dpBoiler)
      annotation (Placement(transformation(extent={{-24,-70},{-36,-58}})));
    Modelica.Blocks.Sources.Constant TRet_Boiler(k=TRet_Min)
      "Minimum return temperature "
      annotation (Placement(transformation(extent={{-8,-32},{-14,-26}})));
    ElementaryBlocs.PumpSupply pumpSupply(
      redeclare package Medium = Medium,
      KvReturn=1,
      realInput=true,
      booleanInput=false,
      includePipes=true,
      m_flow_nominal=Mass_flow,
      dp(displayUnit="Pa") = 2000,
      addPowerToMedium=true,
      InsuHeatCondu=kPipe,
      measureSupplyT=true,
      InsuPipeThickness=0.04)                          annotation (Placement(
          transformation(
          extent={{-8,-9},{8,9}},
          rotation=270,
          origin={-29,-48})));
    Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 RWest(
      TRad_nominal(displayUnit="K"),
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      TAir_nominal(displayUnit="degC") = 292.15,
      nEle=5,
      T_a_nominal(displayUnit="degC") = 318.15,
      T_b_nominal(displayUnit="degC") = 308.15,
      redeclare package Medium = Medium,
      dp_nominal=4000,
      Q_flow_nominal=2500) "Radiator"
      annotation (Placement(transformation(extent={{82,18},{94,30}})));
    Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage valSW(
      CvData=Buildings.Fluid.Types.CvTypes.Kv,
      redeclare package Medium = Medium,
      Kv=20,
      dpFixed_nominal=4000,
      m_flow_nominal=Mass_flow)
             annotation (Placement(transformation(
          extent={{-6,-6},{6,6}},
          rotation=0,
          origin={74,24})));
    Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 RSouthWest(
      TRad_nominal(displayUnit="K"),
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      TAir_nominal(displayUnit="degC") = 292.15,
      nEle=5,
      T_a_nominal(displayUnit="degC") = 318.15,
      T_b_nominal(displayUnit="degC") = 308.15,
      redeclare package Medium = Medium,
      dp_nominal=2000,
      Q_flow_nominal=1500) "Radiator"
      annotation (Placement(transformation(extent={{82,4},{94,16}})));
    Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage valWest(
      CvData=Buildings.Fluid.Types.CvTypes.Kv,
      redeclare package Medium = Medium,
      Kv=20,
      dpFixed_nominal=4000,
      m_flow_nominal=Mass_flow)
             annotation (Placement(transformation(
          extent={{-6,-5},{6,5}},
          rotation=0,
          origin={74,11})));
    Buildings.Fluid.FixedResistances.Junction splValWestIn(
      dp_nominal={0,0,0},
      redeclare package Medium = Medium,
      m_flow_nominal=Mass_flow*{1,-1,-1})
                    "Flow splitter" annotation (Placement(transformation(
          extent={{5,5},{-5,-5}},
          rotation=270,
          origin={63,11})));
    Buildings.Fluid.FixedResistances.Junction splVal21(
      dp_nominal={0,0,0},
      redeclare package Medium = Medium,
      m_flow_nominal=Mass_flow*{1,-1,1})
                    "Flow splitter" annotation (Placement(transformation(
          extent={{-6,-6},{6,6}},
          rotation=270,
          origin={100,10})));
    ElementaryBlocs.PumpSupply_m_flow pumpSupply_m_flow(redeclare package
        Medium =
          Medium,
      KvReturn=1,
      m_flow_nominal=Mass_flow,
      addPowerToMedium=false)                           annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={82,-8})));
    ElementaryBlocs.MixingCircuit_Tset mixingCircuit_Tset1(redeclare package
        Medium = Medium, KvReturn=40,
      m_flow_nominal=Mass_flow,
      includePipes=true,
      InsuPipeThickness=0.02,
      dp(displayUnit="Pa") = 3000)                         annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={82,-54})));
    Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 REast(
      TRad_nominal(displayUnit="K"),
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      TAir_nominal(displayUnit="degC") = 292.15,
      T_a_nominal(displayUnit="degC") = 318.15,
      T_b_nominal(displayUnit="degC") = 308.15,
      redeclare package Medium = Medium,
      dp_nominal=400,
      nEle=5,
      Q_flow_nominal=2500) "Radiator"
      annotation (Placement(transformation(extent={{140,16},{152,28}})));
    Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage valE(
      CvData=Buildings.Fluid.Types.CvTypes.Kv,
      redeclare package Medium = Medium,
      Kv=20,
      dpFixed_nominal=4000,
      m_flow_nominal=Mass_flow)     annotation (Placement(transformation(
          extent={{-6,-6},{6,6}},
          rotation=0,
          origin={132,22})));
    Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2 RSouthEast(
      TRad_nominal(displayUnit="K"),
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      TAir_nominal(displayUnit="degC") = 292.15,
      nEle=5,
      T_a_nominal(displayUnit="degC") = 318.15,
      T_b_nominal(displayUnit="degC") = 308.15,
      redeclare package Medium = Medium,
      dp_nominal=2000,
      Q_flow_nominal=1500)                      "Radiator"
      annotation (Placement(transformation(extent={{140,2},{152,14}})));
    Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage valWest1(
      CvData=Buildings.Fluid.Types.CvTypes.Kv,
      redeclare package Medium = Medium,
      Kv=20,
      dpFixed_nominal=4000,
      m_flow_nominal=Mass_flow)
             annotation (Placement(transformation(
          extent={{-6,-5},{6,5}},
          rotation=0,
          origin={132,9})));
    Buildings.Fluid.FixedResistances.Junction splValWestIn1(
      dp_nominal={0,0,0},
      redeclare package Medium = Medium,
      m_flow_nominal=Mass_flow*{1,-1,-1})
                    "Flow splitter" annotation (Placement(transformation(
          extent={{5,5},{-5,-5}},
          rotation=270,
          origin={121,9})));
    Buildings.Fluid.FixedResistances.Junction splVal1(
      dp_nominal={0,0,0},
      redeclare package Medium = Medium,
      m_flow_nominal=Mass_flow*{1,-1,1})
                    "Flow splitter" annotation (Placement(transformation(
          extent={{-6,-6},{6,6}},
          rotation=270,
          origin={158,8})));
    ElementaryBlocs.PumpSupply_m_flow pumpSupply_m_flow1(redeclare package
        Medium =
          Medium,
      KvReturn=1,
      m_flow_nominal=Mass_flow)                          annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={140,-10})));
    ElementaryBlocs.MixingCircuit_Tset mixingCircuit_Tset2(KvReturn=40,
        redeclare package Medium = Medium,
      m_flow_nominal=Mass_flow,
      includePipes=true,
      InsuPipeThickness=0.02,
      dp(displayUnit="Pa") = 3000)                         annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={140,-56})));
    Controls.ControlHeating.Ctrl_Heating ctrl_Heating(TRoo_nominal=273.15 + 19,
      corFac_val=0,
      TSupNom=318.15)
      annotation (Placement(transformation(extent={{52,-64},{64,-50}})));
    Controls.ControlHeating.Ctrl_Heating ctrl_Heating1(TRoo_nominal=273.15 + 19,
      corFac_val=0,
      TSupNom=318.15)
      annotation (Placement(transformation(extent={{112,-68},{124,-54}})));
    Buildings.Fluid.Storage.StratifiedEnhanced tan(
      redeclare package Medium = Medium,
      hTan=2,
      m_flow_nominal=Mass_flow,
      VTan=3,
      kIns=kTank,
      nSeg=5,
      dIns=0.10)
      annotation (Placement(transformation(extent={{-34,-16},{-20,-2}})));
    ElementaryBlocs.PumpSupply_m_flow returnPump(
      redeclare package Medium = Medium,
      KvReturn=1,
      m_flow_nominal=Mass_flow,
      includePipes=true,
      InsuPipeThickness=0.02,
      Pipelength=10,
      measureSupplyT=true,
      dp=300000000)
      annotation (Placement(transformation(extent={{24,0},{10,-14}})));
    ElementaryBlocs.BalancingValve balancingValve(redeclare package Medium =
          Medium,
      KvReturn=40,
      m_flow_nominal=Mass_flow,
      InsuPipeThickness=0.04,
      dp=400000000)
      annotation (Placement(transformation(extent={{-10,-9},{10,9}},
          rotation=270,
          origin={31,-32})));
    ElementaryBlocs.CollectorUnit collectorUnit(redeclare package Medium = Medium,
        m_flow_nominal=Mass_flow)
      annotation (Placement(transformation(extent={{72,-86},{92,-66}})));
    Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam="modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos")
      annotation (Placement(transformation(extent={{-44,30},{-30,44}})));
    Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
          transformation(extent={{-24,22},{-8,50}}),      iconTransformation(
            extent={{-298,194},{-278,214}})));
    HouseModel_FirstTuto houseModel_FirstTuto
      annotation (Placement(transformation(extent={{92,60},{136,90}})));
    Modelica.Blocks.Logical.Hysteresis hysTOut(uLow=273.15 + 16, uHigh=273.15 + 17)
      "Hysteresis for on/off based on outside temperature"
      annotation (Placement(transformation(extent={{-10,6},{0,16}})));
    Modelica.Blocks.Logical.Not not2
      annotation (Placement(transformation(extent={{4,6},{14,16}})));
    Modelica.Blocks.Logical.And and1
      annotation (Placement(transformation(extent={{18,10},{28,20}})));
    Modelica.Blocks.Logical.Hysteresis hysPum(uLow=273.15 + 18, uHigh=273.15 + 20)
      "Pump hysteresis"
      annotation (Placement(transformation(extent={{-10,20},{0,30}})));
    Modelica.Blocks.Logical.Not not1 "Negate output of hysteresis"
      annotation (Placement(transformation(extent={{4,20},{14,30}})));
    Buildings.Utilities.Math.Average ave(nin=3)
      "Compute average of room temperatures"
      annotation (Placement(transformation(extent={{146,68},{156,78}})));
    Modelica.Blocks.Math.BooleanToReal booToReaRad(realTrue=Mass_flow)
      "Radiator pump signal"
      annotation (Placement(transformation(extent={{32,12},{40,20}})));
    replaceable package Medium = MediumW constrainedby
      Modelica.Media.Interfaces.PartialMedium;
    Buildings.Fluid.Storage.ExpansionVessel exp(redeclare package Medium =
          Medium, V_start=1)
      annotation (Placement(transformation(extent={{-12,-66},{-4,-58}})));
    Buildings.Controls.Continuous.LimPID conPID1(
      Ti=1200,
      k=0.005,
      controllerType=Modelica.Blocks.Types.SimpleController.PID,
      Td=60)   annotation (Placement(transformation(extent={{86,42},{80,48}})));
    Buildings.Controls.Continuous.LimPID conPID2(
      Ti=1200,
      k=0.005,
      controllerType=Modelica.Blocks.Types.SimpleController.PID,
      Td=60)   annotation (Placement(transformation(extent={{86,32},{80,38}})));
    Buildings.Controls.Continuous.LimPID conPID3(
      Ti=1200,
      k=0.005,
      controllerType=Modelica.Blocks.Types.SimpleController.PID,
      Td=60)   annotation (Placement(transformation(extent={{120,44},{126,50}})));
    Buildings.Controls.Continuous.LimPID conPID4(
      Ti=1200,
      k=0.005,
      controllerType=Modelica.Blocks.Types.SimpleController.PID,
      Td=60)   annotation (Placement(transformation(extent={{120,34},{126,40}})));
    Modelica.Blocks.Sources.Constant TRet_Boiler1(k=273.15 + 19)
      "Minimum return temperature "
      annotation (Placement(transformation(extent={{3,-3},{-3,3}},
          rotation=90,
          origin={103,53})));
    Buildings.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature
      annotation (Placement(transformation(extent={{-46,-10},{-38,-2}})));
    Modelica.Blocks.Sources.RealExpression Production(each y=boiler.mFue_flow)
      annotation (Placement(transformation(extent={{-128,40},{-114,56}})));
    Modelica.Blocks.Interaction.Show.RealValue RealVal3(significantDigits=5)
      annotation (Placement(transformation(extent={{-90,40},{-58,54}})));
    Modelica.Blocks.Continuous.Integrator integrator
      annotation (Placement(transformation(extent={{-108,44},{-100,52}})));
    Modelica.Blocks.Sources.RealExpression ConsumWest(each y=powerSensorWest.Diff_Energy)
      annotation (Placement(transformation(extent={{-126,104},{-112,120}})));
    Modelica.Blocks.Interaction.Show.RealValue RealVal1(significantDigits=5)
      annotation (Placement(transformation(extent={{-102,106},{-70,120}})));
    Modelica.Blocks.Sources.RealExpression ConsumEast(each y=powerSensorEast.Diff_Energy)
      annotation (Placement(transformation(extent={{-126,88},{-112,104}})));
    Modelica.Blocks.Interaction.Show.RealValue RealVal2(significantDigits=5)
      annotation (Placement(transformation(extent={{-102,90},{-70,104}})));
    Modelica.Blocks.Sources.RealExpression ConsumTotal(each y=powerSensorEast.Diff_Energy
           + powerSensorWest.Diff_Energy + integrator1.y)
      annotation (Placement(transformation(extent={{-126,70},{-112,86}})));
    Modelica.Blocks.Interaction.Show.RealValue RealVal4(significantDigits=5)
      annotation (Placement(transformation(extent={{-98,72},{-66,86}})));
    Modelica.Blocks.Sources.RealExpression Global_effi(each y=ConsumTotal.y/(
          0.0000001 + integrator.y))
      annotation (Placement(transformation(extent={{-126,56},{-112,72}})));
    Modelica.Blocks.Interaction.Show.RealValue RealVal5(significantDigits=5)
      annotation (Placement(transformation(extent={{-102,58},{-70,72}})));
    ElementaryBlocs.PowerSensor powerSensor(redeclare package Medium = Medium,
        m_flow_nominal=Mass_flow,
      Measured_Supply=true)
      annotation (Placement(transformation(extent={{-14,-16},{0,0}})));
    ElementaryBlocs.PowerSensor powerSensorWest(redeclare package Medium =
          Medium, m_flow_nominal=Mass_flow) annotation (Placement(
          transformation(
          extent={{-8.5,-9.5},{8.5,9.5}},
          rotation=90,
          origin={82.5,-31.5})));
    ElementaryBlocs.PowerSensor powerSensorEast(redeclare package Medium =
          Medium, m_flow_nominal=Mass_flow) annotation (Placement(
          transformation(
          extent={{-8.5,-9.5},{8.5,9.5}},
          rotation=90,
          origin={140.5,-33.5})));
    Modelica.Blocks.Sources.RealExpression BoilerRoom_efficiency(each y=(
          powerSensor.Diff_Energy + integrator1.y)/(integrator.y + 0.01))
      annotation (Placement(transformation(extent={{-128,16},{-114,32}})));
    Modelica.Blocks.Interaction.Show.RealValue RealVal6(significantDigits=5)
      annotation (Placement(transformation(extent={{-94,18},{-62,32}})));
    parameter Buildings.Fluid.Data.Fuels.WoodAirDriedLowerHeatingValue datFue(
      mCO2=0.004,
      h=17E6,
      d=1000)
      annotation (Placement(transformation(extent={{-42,104},{-30,116}})));
    Modelica.Blocks.Sources.RealExpression Electrical(each y=pumpSupply.power)
      annotation (Placement(transformation(extent={{-128,28},{-114,44}})));
    Modelica.Blocks.Continuous.Integrator integrator1(
                                                     k=1/(3600*1000000))
      annotation (Placement(transformation(extent={{-108,32},{-100,40}})));
    Modelica.Blocks.Interaction.Show.RealValue RealVal7(significantDigits=5)
      annotation (Placement(transformation(extent={{-92,28},{-60,42}})));
    Modelica.Blocks.Logical.GreaterThreshold greThr(threshold=TSup_nominal + 5)
      "Check for temperature at the bottom of the tank"
      annotation (Placement(transformation(extent={{-86,-52},{-74,-40}})));
    Modelica.Blocks.Math.BooleanToReal booToReaPum "Signal converter for pump"
      annotation (Placement(transformation(extent={{-4,-52},{-12,-44}})));
    Modelica.Blocks.Logical.Greater lesThr
      "Check for temperature at the top of the tank"
      annotation (Placement(transformation(extent={{-120,-74},{-110,-64}})));
    Modelica.Blocks.MathBoolean.Or pumOnSig(nu=3) "Signal for pump being on"
      annotation (Placement(transformation(extent={{-58,-90},{-48,-80}})));
    Modelica.Blocks.Math.BooleanToReal booToReaBoi1
                                                   "Signal converter for boiler"
      annotation (Placement(transformation(extent={{-50,-78},{-40,-68}})));
    Modelica.Blocks.Logical.LessThreshold lesThrTRoo(threshold=16 + 273.15)
      "Test to block boiler if room air temperature is sufficiently high"
      annotation (Placement(transformation(extent={{-134,-62},{-122,-50}})));
    Modelica.Blocks.Logical.And and4
      "Logical test to enable pump and subsequently the boiler"
      annotation (Placement(transformation(extent={{-6,-6},{6,6}},
          rotation=90,
          origin={-114,-48})));
    Modelica.StateGraph.InitialStep off "Pump and furnace off"
      annotation (Placement(transformation(extent={{-130,-36},{-118,-24}})));
    Modelica.StateGraph.TransitionWithSignal T5 "Transition to pump on"
      annotation (Placement(transformation(extent={{-120,-36},{-108,-24}})));
    Modelica.StateGraph.StepWithSignal pumOn "Pump on"
      annotation (Placement(transformation(extent={{-108,-36},{-96,-24}})));
    Modelica.StateGraph.Transition T6(enableTimer=true, waitTime=600)
      "Transition to boiler on"
      annotation (Placement(transformation(extent={{-98,-36},{-86,-24}})));
    Modelica.StateGraph.StepWithSignal boiOn "Boiler on"
      annotation (Placement(transformation(extent={{-86,-36},{-74,-24}})));
    Modelica.StateGraph.TransitionWithSignal T7
      "Transition that switches boiler off"
      annotation (Placement(transformation(extent={{-76,-36},{-64,-24}})));
    Modelica.StateGraph.StepWithSignal pumOn2 "Pump on"
      annotation (Placement(transformation(extent={{-66,-36},{-54,-24}})));
    Modelica.StateGraph.Transition T8(enableTimer=true, waitTime=60)
      "Transition to boiler on"
      annotation (Placement(transformation(extent={{-56,-36},{-44,-24}})));
    inner Modelica.StateGraph.StateGraphRoot stateGraphRoot
      "Root of the state graph"
      annotation (Placement(transformation(extent={{-148,-22},{-134,-8}})));
    Modelica.Blocks.Sources.Constant dTThr(k=1) "Threshold to switch boiler off"
      annotation (Placement(transformation(extent={{-152,-82},{-140,-70}})));
    Modelica.Blocks.Math.Add add1(k2=-1)
      annotation (Placement(transformation(extent={{-136,-78},{-126,-68}})));
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor tanTemBot
      "Tank temperature"
      annotation (Placement(transformation(extent={{-52,-18},{-62,-8}})));
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor tanTemTop
      "Tank temperature"
      annotation (Placement(transformation(extent={{-66,-10},{-76,0}})));
    Buildings.Utilities.Math.Max max(nin=2)
      annotation (Placement(transformation(extent={{0,-98},{-8,-90}})));
  equation
    connect(pumpSupply.port_b1, boiler.port_a) annotation (Line(points={{-23.6,
            -56},{-23.6,-64},{-24,-64}}, color={0,127,255}));
    connect(mixingCircuit_Tset.port_b2, pumpSupply.port_b2) annotation (Line(
          points={{-34.4,-38},{-34.4,-40}},             color={0,127,255}));
    connect(mixingCircuit_Tset.port_a1, pumpSupply.port_a1) annotation (Line(
          points={{-23.6,-38},{-23.6,-40}},             color={0,127,255}));
    connect(splValWestIn.port_3,valWest. port_a)
      annotation (Line(points={{68,11},{68,12},{68,11}},
                                                 color={0,127,255}));
    connect(valSW.port_b,RWest. port_a) annotation (Line(points={{80,24},{80,24},{
            82,24}},    color={0,127,255}));
    connect(valWest.port_b,RSouthWest. port_a)
      annotation (Line(points={{80,11},{80,10},{82,10}},    color={0,127,255}));
    connect(RSouthWest.port_b,splVal21. port_3)
      annotation (Line(points={{94,10},{94,10}},            color={0,127,255}));
    connect(splValWestIn.port_2,valSW. port_a) annotation (Line(points={{63,16},
            {64,16},{64,24},{68,24}},
                                    color={0,127,255}));
    connect(RWest.port_b,splVal21. port_1) annotation (Line(points={{94,24},{
            100,24},{100,16}},
                            color={0,127,255}));
    connect(pumpSupply_m_flow.port_b1, splValWestIn.port_1) annotation (Line(
          points={{76,2},{68,2},{68,6},{63,6}},   color={0,127,255}));
    connect(pumpSupply_m_flow.port_a2, splVal21.port_2) annotation (Line(points={{88,2},{
            96,2},{96,4},{100,4}},          color={0,127,255}));
    connect(splValWestIn1.port_3, valWest1.port_a) annotation (Line(points={{126,9},
            {126,10},{126,9}},          color={0,127,255}));
    connect(valE.port_b, REast.port_a)
      annotation (Line(points={{138,22},{138,22},{140,22}}, color={0,127,255}));
    connect(valWest1.port_b, RSouthEast.port_a)
      annotation (Line(points={{138,9},{138,8},{140,8}}, color={0,127,255}));
    connect(RSouthEast.port_b, splVal1.port_3)
      annotation (Line(points={{152,8},{152,8}},         color={0,127,255}));
    connect(splValWestIn1.port_2, valE.port_a) annotation (Line(points={{121,14},
            {120,14},{120,22},{126,22}},color={0,127,255}));
    connect(REast.port_b, splVal1.port_1)
      annotation (Line(points={{152,22},{158,22},{158,14}}, color={0,127,255}));
    connect(pumpSupply_m_flow1.port_b1, splValWestIn1.port_1) annotation (Line(
          points={{134,0},{126,0},{126,4},{121,4}},     color={0,127,255}));
    connect(pumpSupply_m_flow1.port_a2, splVal1.port_2) annotation (Line(points={{146,0},
            {154,0},{154,2},{158,2}},            color={0,127,255}));
    connect(tan.port_b, mixingCircuit_Tset.port_b1) annotation (Line(points={{-20,-9},
            {-20,-9},{-20,-20},{-23.6,-20}},       color={0,127,255}));
    connect(mixingCircuit_Tset.port_a2, tan.port_a) annotation (Line(points={{-34.4,
            -20},{-34.4,-8},{-34,-8},{-34,-9}},    color={0,127,255}));
    connect(mixingCircuit_Tset1.port_a1, collectorUnit.port_b3)
      annotation (Line(points={{76,-64},{76,-66}},          color={0,127,255}));
    connect(collectorUnit.port_a3, mixingCircuit_Tset1.port_b2)
      annotation (Line(points={{88,-86.4},{88,-64}}, color={0,127,255}));
    connect(weaDat.weaBus,weaBus)  annotation (Line(
        points={{-30,37},{-16,37},{-16,36}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(weaBus, ctrl_Heating.weaBus) annotation (Line(
        points={{-16,36},{50,36},{50,-52.66},{47.44,-52.66}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(weaBus, ctrl_Heating1.weaBus) annotation (Line(
        points={{-16,36},{18,36},{50,36},{50,-56.66},{107.44,-56.66}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(weaBus, houseModel_FirstTuto.weaBus) annotation (Line(
        points={{-16,36},{-16,36},{-16,86.8125},{96.7826,86.8125}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(not2.y,and1. u2) annotation (Line(
        points={{14.5,11},{17,11}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(hysPum.y,not1. u) annotation (Line(
        points={{0.5,25},{3,25}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(not1.y,and1. u1) annotation (Line(
        points={{14.5,25},{16,25},{16,15},{17,15}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(weaBus.TDryBul, hysTOut.u) annotation (Line(
        points={{-16,36},{-16,36},{-16,12},{-16,11},{-11,11}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(houseModel_FirstTuto.TAir, ave.u) annotation (Line(points={{136.957,
            72.5625},{144.5,72.5625},{144.5,73},{145,73}},
                                               color={0,0,127}));
    connect(ave.y, hysPum.u) annotation (Line(points={{156.5,73},{156,73},{156,
            74},{158,74},{158,92},{-16,92},{-16,26},{-14,26},{-12,26},{-12,25},
            {-11,25}},                            color={0,0,127}));
    connect(and1.y, booToReaRad.u) annotation (Line(points={{28.5,15},{30,15},{30,
            16},{31.2,16}},
                          color={255,0,255}));
    connect(booToReaRad.y, pumpSupply_m_flow.u) annotation (Line(points={{40.4,16},
            {44,16},{44,-8},{71.2,-8}},  color={0,0,127}));
    connect(booToReaRad.y, pumpSupply_m_flow1.u) annotation (Line(points={{40.4,16},
            {44,16},{44,-10},{129.2,-10}},color={0,0,127}));
    connect(houseModel_FirstTuto.radWest, RWest.heatPortRad) annotation (Line(
          points={{91.8087,80.8125},{91.8087,83.1},{89.2,83.1},{89.2,28.32}},
                                                                     color={191,0,
            0}));
    connect(RSouthWest.heatPortCon, houseModel_FirstTuto.convSouth) annotation (
        Line(points={{86.8,14.32},{86.8,19.16},{115.148,19.16},{115.148,60.1875}},
                                                                            color=
           {191,0,0}));
    connect(RSouthWest.heatPortRad, houseModel_FirstTuto.radNorth) annotation (
        Line(points={{89.2,14.32},{89.2,19.16},{126.435,19.16},{126.435,60}},
                                                                      color={191,0,
            0}));
    connect(REast.heatPortRad, houseModel_FirstTuto.radEast) annotation (Line(
          points={{147.2,26.32},{147.2,82.16},{136,82.16},{136,80.8125}},
                                                                       color={191,
            0,0}));
    connect(REast.heatPortCon, houseModel_FirstTuto.convEast) annotation (Line(
          points={{144.8,26.32},{144.8,75.16},{136,75.16},{136,75}}, color={191,0,
            0}));
    connect(RSouthEast.heatPortCon, houseModel_FirstTuto.convSouth) annotation (
        Line(points={{144.8,12.32},{144.8,16.16},{115.148,16.16},{115.148,
            60.1875}},
          color={191,0,0}));
    connect(RSouthEast.heatPortRad, houseModel_FirstTuto.radNorth) annotation (
        Line(points={{147.2,12.32},{147.2,16.16},{126.435,16.16},{126.435,60}},
                                                                        color={191,
            0,0}));
    connect(houseModel_FirstTuto.TAir[1], ctrl_Heating.TRoo_in1) annotation (Line(
          points={{136.957,71.9375},{136.957,54},{52,54},{52,-54.2}},
                                                              color={0,0,127}));
    connect(houseModel_FirstTuto.TAir[2], ctrl_Heating1.TRoo_in1) annotation (
        Line(points={{136.957,72.5625},{136.957,52.7},{112,52.7},{112,-58.2}},
                                                                    color={0,0,127}));
    connect(boiler.port_a, exp.port_a) annotation (Line(points={{-24,-64},{-24,
            -66},{-8,-66}},            color={0,127,255}));
    connect(RWest.heatPortCon, houseModel_FirstTuto.convWest) annotation (Line(
          points={{86.8,28.32},{86.8,75.16},{91.8087,75.16},{91.8087,74.0625}},
                                                                     color={191,
            0,0}));
    connect(conPID1.u_s, TRet_Boiler1.y) annotation (Line(points={{86.6,45},{102.3,
            45},{102.3,49.7},{103,49.7}}, color={0,0,127}));
    connect(conPID2.u_s, TRet_Boiler1.y) annotation (Line(points={{86.6,35},{102.3,
            35},{102.3,49.7},{103,49.7}}, color={0,0,127}));
    connect(TRet_Boiler1.y, conPID3.u_s) annotation (Line(points={{103,49.7},{103,
            49.7},{103,47},{119.4,47}}, color={0,0,127}));
    connect(TRet_Boiler1.y, conPID4.u_s) annotation (Line(points={{103,49.7},{103,
            49.7},{103,37},{119.4,37}}, color={0,0,127}));
    connect(houseModel_FirstTuto.TAir[1], conPID1.u_m) annotation (Line(points={{136.957,
            71.9375},{141.5,71.9375},{141.5,41.4},{83,41.4}}, color={0,0,127}));
    connect(conPID1.y, valSW.y) annotation (Line(points={{79.7,45},{79.7,44.5},{74,
            44.5},{74,31.2}}, color={0,0,127}));
    connect(conPID2.y, valWest.y) annotation (Line(points={{79.7,35},{79.7,34.5},
            {74,34.5},{74,17}},color={0,0,127}));
    connect(conPID2.u_m, houseModel_FirstTuto.TAir[3]) annotation (Line(points={{83,31.4},
            {142.5,31.4},{142.5,73.1875},{136.957,73.1875}},   color={0,0,127}));
    connect(conPID3.y, valE.y) annotation (Line(points={{126.3,47},{126.3,46.5},{132,
            46.5},{132,29.2}}, color={0,0,127}));
    connect(houseModel_FirstTuto.TAir[2], conPID3.u_m) annotation (Line(points={{136.957,
            72.5625},{136.957,42.7},{123,42.7},{123,43.4}},
                                                     color={0,0,127}));
    connect(conPID4.y, valWest1.y) annotation (Line(points={{126.3,37},{126.3,36.5},
            {132,36.5},{132,15}}, color={0,0,127}));
    connect(houseModel_FirstTuto.TAir[3], conPID4.u_m) annotation (Line(points={{136.957,
            73.1875},{136.957,33.7},{123,33.7},{123,33.4}},
                                                        color={0,0,127}));
    connect(booToReaRad.y, returnPump.u) annotation (Line(points={{40.4,16},{44,
            16},{44,-14.56},{17,-14.56}}, color={0,0,127}));
    connect(hysTOut.y, not2.u)
      annotation (Line(points={{0.5,11},{0.5,11},{3,11}}, color={255,0,255}));
    connect(ctrl_Heating.THeaCur, mixingCircuit_Tset1.TMixedSet) annotation (Line(
          points={{64,-54.2},{64,-54.2},{64,-54},{72,-54}}, color={0,0,127}));
    connect(ctrl_Heating1.THeaCur, mixingCircuit_Tset2.TMixedSet) annotation (
        Line(points={{124,-58.2},{126,-58.2},{126,-58},{128,-58},{128,-56},{130,
            -56}},                                                 color={0,0,127}));
    connect(prescribedTemperature.port, tan.heaPorTop) annotation (Line(points={{-38,-6},
            {-34,-6},{-34,-3.82},{-25.6,-3.82}},     color={191,0,0}));
    connect(prescribedTemperature.port, tan.heaPorBot) annotation (Line(points={{-38,-6},
            {-34,-6},{-34,-14.18},{-25.6,-14.18}},     color={191,0,0}));
    connect(prescribedTemperature.port, tan.heaPorSid) annotation (Line(points={{-38,-6},
            {-23.08,-6},{-23.08,-9}},     color={191,0,0}));
    connect(prescribedTemperature.port, pumpSupply.heatPort) annotation (Line(
          points={{-38,-6},{-40,-6},{-40,-48},{-38,-48}}, color={191,0,0}));
    connect(prescribedTemperature.port, returnPump.heatPort) annotation (Line(
          points={{-38,-6},{-40,-6},{-40,0},{17,0}}, color={191,0,0}));
    connect(ave.y, prescribedTemperature.T) annotation (Line(points={{156.5,73},{158,
            73},{158,92},{-48,92},{-48,-6},{-46.8,-6}}, color={0,0,127}));
    connect(RealVal3.numberPort, integrator.y) annotation (Line(points={{-92.4,
            47},{-96,47},{-96,48},{-99.6,48}}, color={0,0,127}));
    connect(Production.y, integrator.u) annotation (Line(points={{-113.3,48},{
            -110,48},{-110,46},{-110,48},{-108.8,48}}, color={0,0,127}));
    connect(pumpSupply.port_a2, boiler.port_b) annotation (Line(points={{-34.4,
            -56},{-36,-56},{-36,-64}}, color={0,127,255}));
    connect(ConsumTotal.y, RealVal4.numberPort) annotation (Line(points={{
            -111.3,78},{-106,78},{-106,79},{-100.4,79}}, color={0,0,127}));
    connect(Global_effi.y, RealVal5.numberPort) annotation (Line(points={{-111.3,
            64},{-108,64},{-108,65},{-104.4,65}}, color={0,0,127}));
    connect(powerSensor.port_b1, returnPump.port_a2) annotation (Line(points={{
            0,-3.2},{6,-3.2},{6,-2.8},{10,-2.8}}, color={0,127,255}));
    connect(powerSensor.port_a2, returnPump.port_b1) annotation (Line(points={{
            1.77636e-15,-12.8},{5,-12.8},{5,-11.2},{10,-11.2}}, color={0,127,
            255}));
    connect(tan.port_b, powerSensor.port_b2) annotation (Line(points={{-20,-9},
            {-18,-9},{-18,-12.8},{-14,-12.8}}, color={0,127,255}));
    connect(pumpSupply_m_flow.port_a1, powerSensorWest.port_b1) annotation (
        Line(points={{76,-18},{76,-23},{76.8,-23}}, color={0,127,255}));
    connect(pumpSupply_m_flow.port_b2, powerSensorWest.port_a2) annotation (
        Line(points={{88,-18},{88,-18},{88,-23},{88.2,-23}}, color={0,127,255}));
    connect(powerSensorWest.port_a1, mixingCircuit_Tset1.port_b1) annotation (
        Line(points={{76.8,-40},{76,-40},{76,-44}}, color={0,127,255}));
    connect(powerSensorWest.port_b2, mixingCircuit_Tset1.port_a2) annotation (
        Line(points={{88.2,-40},{88,-40},{88,-44}}, color={0,127,255}));
    connect(powerSensorEast.port_a1, mixingCircuit_Tset2.port_b1) annotation (
        Line(points={{134.8,-42},{134,-42},{134,-46}}, color={0,127,255}));
    connect(powerSensorEast.port_b2, mixingCircuit_Tset2.port_a2) annotation (
        Line(points={{146.2,-42},{146,-42},{146,-46}}, color={0,127,255}));
    connect(pumpSupply_m_flow1.port_a1, powerSensorEast.port_b1) annotation (
        Line(points={{134,-20},{134,-25},{134.8,-25}}, color={0,127,255}));
    connect(pumpSupply_m_flow1.port_b2, powerSensorEast.port_a2) annotation (
        Line(points={{146,-20},{146,-25},{146.2,-25}}, color={0,127,255}));
    connect(ConsumWest.y, RealVal1.numberPort) annotation (Line(points={{-111.3,
            112},{-104.4,112},{-104.4,113}}, color={0,0,127}));
    connect(ConsumEast.y, RealVal2.numberPort) annotation (Line(points={{-111.3,
            96},{-104.4,96},{-104.4,97}}, color={0,0,127}));
    connect(BoilerRoom_efficiency.y, RealVal6.numberPort) annotation (Line(
          points={{-113.3,24},{-96.4,24},{-96.4,25}}, color={0,0,127}));
    connect(collectorUnit.port_b1, mixingCircuit_Tset2.port_a1) annotation (
        Line(points={{92,-70},{134,-70},{134,-66}}, color={0,127,255}));
    connect(collectorUnit.port_a2, mixingCircuit_Tset2.port_b2) annotation (
        Line(points={{92,-82},{146,-82},{146,-66}}, color={0,127,255}));
    connect(returnPump.port_b2, balancingValve.port_a1) annotation (Line(points=
           {{24,-2.8},{36.4,-2.8},{36.4,-22}}, color={0,127,255}));
    connect(balancingValve.port_b1, collectorUnit.port_a1) annotation (Line(
          points={{36.4,-42},{36,-42},{36,-70},{72,-70}},
                                                        color={0,127,255}));
    connect(balancingValve.port_a2, collectorUnit.port_b2) annotation (Line(
          points={{25.6,-42},{26,-42},{26,-82},{72,-82}},
                                                        color={0,127,255}));
    connect(returnPump.port_a1, balancingValve.port_b2) annotation (Line(points=
           {{24,-11.2},{25.6,-11.2},{25.6,-22}}, color={0,127,255}));
    connect(prescribedTemperature.port, mixingCircuit_Tset1.heatPort)
      annotation (Line(points={{-38,-6},{-40,-6},{-40,-54},{92,-54}}, color={
            191,0,0}));
    connect(prescribedTemperature.port, mixingCircuit_Tset2.heatPort)
      annotation (Line(points={{-38,-6},{-40,-6},{-40,-56},{150,-56}}, color={
            191,0,0}));
    connect(prescribedTemperature.port, boiler.heatPort) annotation (Line(
          points={{-38,-6},{-40,-6},{-40,-59.68},{-30,-59.68}}, color={191,0,0}));
    connect(Electrical.y, integrator1.u)
      annotation (Line(points={{-113.3,36},{-108.8,36}}, color={0,0,127}));
    connect(integrator1.y, RealVal7.numberPort) annotation (Line(points={{-99.6,
            36},{-98,36},{-98,35},{-94.4,35}}, color={0,0,127}));
    connect(mixingCircuit_Tset.TMixedSet, TRet_Boiler.y) annotation (Line(
          points={{-20,-29},{-14.3,-29}},                     color={0,0,127}));
    connect(mixingCircuit_Tset.port_a2, powerSensor.port_a1) annotation (Line(
          points={{-34.4,-20},{-34,-20},{-34,-3.2},{-14,-3.2}}, color={0,127,255}));
    connect(booToReaPum.u,pumOnSig. y) annotation (Line(
        points={{-3.2,-48},{0,-48},{0,-85},{-47.25,-85}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(lesThr.y,and4. u2) annotation (Line(
        points={{-109.5,-69},{-110,-69},{-110,-55.2},{-109.2,-55.2}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(lesThrTRoo.y,and4. u1) annotation (Line(
        points={{-121.4,-56},{-114,-56},{-114,-55.2}},
        color={255,0,255},
        smooth=Smooth.None));
    connect(and4.y,T5. condition) annotation (Line(points={{-114,-41.4},{-114,
            -37.2}},
                  color={255,0,255}));
    connect(greThr.y,T7. condition) annotation (Line(points={{-73.4,-46},{-73.4,
            -46},{-70,-46},{-70,-37.2}},
                                  color={255,0,255}));
    connect(boiOn.active, booToReaBoi1.u) annotation (Line(points={{-80,-36.6},{-80,
            -73},{-51,-73}},    color={255,0,255}));
    connect(pumOn2.active,pumOnSig. u[1]) annotation (Line(points={{-60,-36.6},
            {-60,-82.6667},{-58,-82.6667}},
                             color={255,0,255}));
    connect(boiOn.active,pumOnSig. u[2]) annotation (Line(points={{-80,-36.6},{-80,
            -36.6},{-80,-60},{-80,-85},{-58,-85}},
                                            color={255,0,255}));
    connect(pumOn.active,pumOnSig. u[3]) annotation (Line(points={{-102,-36.6},
            {-102,-87.3333},{-58,-87.3333}},
                             color={255,0,255}));
    connect(off.outPort[1],T5. inPort)
      annotation (Line(points={{-117.7,-30},{-116.4,-30}},       color={0,0,0}));
    connect(T5.outPort,pumOn. inPort[1])
      annotation (Line(points={{-113.1,-30},{-108.6,-30}},    color={0,0,0}));
    connect(pumOn.outPort[1],T6. inPort) annotation (Line(points={{-95.7,-30},{
            -94.4,-30}},               color={0,0,0}));
    connect(T6.outPort,boiOn. inPort[1])
      annotation (Line(points={{-91.1,-30},{-91.1,-30},{-86.6,-30}},
                                                              color={0,0,0}));
    connect(boiOn.outPort[1],T7. inPort)
      annotation (Line(points={{-73.7,-30},{-73.7,-30},{-74,-30},{-72.4,-30}},
                                                              color={0,0,0}));
    connect(T7.outPort,pumOn2. inPort[1])
      annotation (Line(points={{-69.1,-30},{-69.1,-30},{-66.6,-30}},
                                                              color={0,0,0}));
    connect(pumOn2.outPort[1],T8. inPort)
      annotation (Line(points={{-53.7,-30},{-53.7,-30},{-52.4,-30}},
                                                              color={0,0,0}));
    connect(T8.outPort,off. inPort[1]) annotation (Line(points={{-49.1,-30},{-46,-30},
            {-46,-20},{-132,-20},{-132,-30},{-130.6,-30}},
                                                  color={0,0,0}));
    connect(ave.y, lesThrTRoo.u) annotation (Line(points={{156.5,73},{168,73},{
            168,126},{-152,126},{-152,-56},{-135.2,-56}},
                                       color={0,0,127}));
    connect(booToReaBoi1.y, boiler.y) annotation (Line(points={{-39.5,-73},{
            -39.5,-72},{-16,-72},{-16,-59.2},{-22.8,-59.2}},
                                         color={0,0,127}));
    connect(booToReaPum.y, pumpSupply.u) annotation (Line(points={{-12.4,-48},{-12.4,
            -48},{-19.28,-48}},             color={0,0,127}));
    connect(dTThr.y,add1. u2) annotation (Line(
        points={{-139.4,-76},{-137,-76}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(tan.heaPorVol[1],tanTemTop. port) annotation (Line(
        points={{-27,-9.336},{-36,-9.336},{-36,-10},{-50,-10},{-50,-4},{-58,-4},{-58,
            -5},{-66,-5}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(tanTemBot.port, tan.heaPorVol[tan.nSeg]) annotation (Line(
        points={{-52,-13},{-40,-13},{-40,-9},{-27,-9}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(tanTemTop.T, add1.u1) annotation (Line(points={{-76,-5},{-152,-5},{
            -152,-70},{-137,-70}},
                                color={0,0,127}));
    connect(tanTemBot.T, greThr.u) annotation (Line(points={{-62,-13},{-90,-13},{-90,
            -22},{-90,-46},{-87.2,-46}},
                                color={0,0,127}));
    connect(ctrl_Heating.THeaCur, max.u[1]) annotation (Line(points={{64,-54.2},{66,
            -54.2},{66,-94.4},{0.8,-94.4}},      color={0,0,127}));
    connect(ctrl_Heating1.THeaCur, max.u[2]) annotation (Line(points={{124,-58.2},
            {126,-58.2},{126,-93.6},{0.8,-93.6}},         color={0,0,127}));
    connect(max.y, lesThr.u1) annotation (Line(points={{-8.4,-94},{-124,-94},{-124,
            -69},{-121,-69}},      color={0,0,127}));
    connect(add1.y, lesThr.u2)
      annotation (Line(points={{-125.5,-73},{-121,-73}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
              -100},{160,140}})),                                  Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-160,-100},{160,
              140}}),
          graphics={
          Rectangle(
            extent={{-160,4},{6,-100}},
            lineColor={28,108,200},
            lineThickness=1),
          Rectangle(
            extent={{48,56},{162,-100}},
            lineColor={0,255,0},
            lineThickness=1),
          Rectangle(
            extent={{74,96},{160,58}},
            lineColor={255,128,0},
            lineThickness=1),
          Text(
            extent={{96,118},{138,88}},
            lineColor={255,128,0},
            lineThickness=1,
            textString="Building"),
          Text(
            extent={{80,-84},{176,-98}},
            lineColor={0,140,72},
            lineThickness=1,
            textString="Hydraunic heating 
systems"),Text(
            extent={{-158,-88},{-96,-100}},
            lineColor={28,108,200},
            lineThickness=1,
            textString="Boiler room"),
          Rectangle(
            extent={{-136,120},{-54,20}},
            lineColor={238,46,47},
            lineThickness=1),
          Text(
            extent={{-114,22},{-70,6}},
            lineColor={238,46,47},
            lineThickness=1,
            textString="Indicators"),
          Line(
            points={{108,56},{108,-100}},
            color={0,255,0},
            thickness=1,
            pattern=LinePattern.Dot),
          Text(
            extent={{52,56},{76,40}},
            lineColor={0,255,0},
            pattern=LinePattern.Dot,
            lineThickness=1,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="West"),
          Text(
            extent={{140,62},{160,34}},
            lineColor={0,255,0},
            pattern=LinePattern.Dot,
            lineThickness=1,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="East")}),       experiment(StopTime=604800, Interval=600));
  end FirstTuto;

  model HouseModel_FirstTuto
    package MediumA = Buildings.Media.Air
      "Medium model for air";
    Buildings.BoundaryConditions.SolarIrradiation.DiffusePerez
                                                     HDifTil[4](
      each outSkyCon=true,
      each outGroCon=true,
      each til=1.5707963267949,
      each lat=0.72954762733363,
      azi={4.7123889803847,1.5707963267949,3.1415926535898,0})
      "Calculates diffuse solar radiation on titled surface for both directions"
      annotation (Placement(transformation(extent={{-58,30},{-38,50}})));
    Buildings.BoundaryConditions.SolarIrradiation.DirectTiltedSurface
                                                            HDirTil[4](
      each til=1.5707963267949,
      each lat=0.72954762733363,
      azi={4.7123889803847,1.5707963267949,3.1415926535898,0})
      "Calculates direct solar radiation on titled surface for both directions"
      annotation (Placement(transformation(extent={{-60,62},{-40,82}})));
    Buildings.ThermalZones.ReducedOrder.SolarGain.CorrectionGDoublePane
                                    corGDouPan(UWin=2.1, n=4)
      "Correction factor for solar transmission"
      annotation (Placement(transformation(extent={{16,56},{36,76}})));
    Buildings.ThermalZones.ReducedOrder.RC.FourElements West(
      alphaExt=2.7,
      alphaWin=2.7,
      gWin=1,
      ratioWinConRad=0.09,
      nExt=1,
      CExt={5259932.23},
      alphaRad=5,
      alphaInt=2.12,
      nInt=1,
      RInt={0.000668895639141},
      RWin=0.01642857143,
      RExtRem=0.1265217391,
      alphaFloor=2.7,
      nFloor=1,
      RFloorRem=0.1265217391,
      CFloor={5259932.23},
      alphaRoof=2.7,
      nRoof=1,
      RRoofRem=0.1265217391,
      CRoof={5259932.23},
      extWallRC(thermCapExt(each der_T(fixed=true))),
      intWallRC(thermCapInt(each der_T(fixed=true))),
      floorRC(thermCapExt(each der_T(fixed=true))),
      roofRC(thermCapExt(each der_T(fixed=true))),
      VAir=144,
      redeclare package Medium = MediumA,
      RExt={0.00301421908725},
      RFloor={0.00301421908725},
      RRoof={0.00301421908725},
      AInt=50,
      CInt={6891363.86},
      AFloor=48,
      ARoof=48,
      AWin={12,3,4,0},
      ATransparent={12,3,4,0},
      AExt={12,3,12,18},
      nOrientations=4)
                      "Thermal zone"
      annotation (Placement(transformation(extent={{54,8},{102,44}})));
    Buildings.ThermalZones.ReducedOrder.EquivalentAirTemperature.VDI6007WithWindow
                                               eqAirTemp(
      wfGro=0,
      withLongwave=true,
      aExt=0.7,
      alphaWallOut=20,
      alphaRad=5,
      alphaWinOut=20,
      n=4,
      wfWall={0.25,0.25,0.25,0.25},
      wfWin={0.25,0.25,0.25,0.25},
      TGro=283.15) "Computes equivalent air temperature"
      annotation (Placement(transformation(extent={{-14,-4},{6,16}})));
    Modelica.Blocks.Math.Add solRad[4]
      "Sums up solar radiation of both directions"
      annotation (Placement(transformation(extent={{-28,16},{-18,26}})));
    Buildings.HeatTransfer.Sources.PrescribedTemperature preTem
      "Prescribed temperature for exterior walls outdoor surface temperature"
      annotation (Placement(transformation(extent={{18,4},{30,16}})));
    Buildings.HeatTransfer.Sources.PrescribedTemperature preTem1
      "Prescribed temperature for windows outdoor surface temperature"
      annotation (Placement(transformation(extent={{18,24},{30,36}})));
    Modelica.Thermal.HeatTransfer.Components.Convection theConWin
      "Outdoor convective heat transfer of windows"
      annotation (Placement(transformation(extent={{48,26},{38,36}})));
    Modelica.Thermal.HeatTransfer.Components.Convection theConWall
      "Outdoor convective heat transfer of walls"
      annotation (Placement(transformation(extent={{46,16},{36,6}})));
    Modelica.Blocks.Sources.Constant const[4](each k=0)
      "Sets sunblind signal to zero (open)"
      annotation (Placement(transformation(extent={{-10,24},{-4,30}})));
    Modelica.Blocks.Sources.Constant alphaWall(k=25*11.5)
      "Outdoor coefficient of heat transfer for walls"
      annotation (Placement(transformation(extent={{-4,-4},{4,4}},rotation=90,
      origin={40,-6})));
    Modelica.Blocks.Sources.Constant alphaWin(k=20*14)
      "Outdoor coefficient of heat transfer for windows"
      annotation (Placement(transformation(extent={{4,-4},{-4,4}},
      rotation=90,origin={42,48})));
    Buildings.ThermalZones.ReducedOrder.EquivalentAirTemperature.VDI6007
                                     eqAirTempVDI(
      aExt=0.7,
      n=1,
      wfWall={1},
      wfWin={0},
      wfGro=0,
      alphaWallOut=20,
      alphaRad=5,
      TGro=283.15) "Computes equivalent air temperature for roof"
      annotation (Placement(transformation(extent={{40,84},{60,104}})));
    Buildings.HeatTransfer.Sources.PrescribedTemperature preTemRoof
      "Prescribed temperature for roof outdoor surface temperature"
      annotation (Placement(transformation(extent={{-6,-6},{6,6}},rotation=-90,
      origin={77,74})));
    Modelica.Thermal.HeatTransfer.Components.Convection theConRoof
      "Outdoor convective heat transfer of roof"
      annotation (Placement(transformation(extent={{5,-5},{-5,5}},rotation=-90,
      origin={77,57})));
    Modelica.Blocks.Sources.Constant alphaRoof(k=25*11.5)
      "Outdoor coefficient of heat transfer for roof"
      annotation (Placement(transformation(extent={{4,-4},{-4,4}},origin={96,57})));
    Modelica.Blocks.Sources.Constant const1(k=0)
      "Sets sunblind signal to zero (open)"
      annotation (Placement(transformation(extent={{74,106},{68,112}})));
    Buildings.BoundaryConditions.WeatherData.Bus
                                       weaBus "Weather data bus"
      annotation (Placement(transformation(extent={{-92,56},{-58,88}}),
      iconTransformation(extent={{-60,96},{-40,116}})));
    Buildings.BoundaryConditions.SolarIrradiation.DiffusePerez
                                                     HDifTil1[4](
      each outSkyCon=true,
      each outGroCon=true,
      each til=1.5707963267949,
      each lat=0.72954762733363,
      azi={1.5707963267949,4.7123889803847,3.1415926535898,0})
      "Calculates diffuse solar radiation on titled surface for both directions"
      annotation (Placement(transformation(extent={{184,22},{204,42}})));
    Buildings.BoundaryConditions.SolarIrradiation.DirectTiltedSurface
                                                            HDirTil1[4](
      each til=1.5707963267949,
      each lat=0.72954762733363,
      azi={1.5707963267949,4.7123889803847,3.1415926535898,0})
      "Calculates direct solar radiation on titled surface for both directions"
      annotation (Placement(transformation(extent={{184,54},{204,74}})));
    Buildings.ThermalZones.ReducedOrder.SolarGain.CorrectionGDoublePane
                                    corGDouPan1(
                                               UWin=2.1, n=4)
      "Correction factor for solar transmission"
      annotation (Placement(transformation(extent={{258,48},{278,68}})));
    Buildings.ThermalZones.ReducedOrder.RC.FourElements East(
      alphaExt=2.7,
      alphaWin=2.7,
      gWin=1,
      ratioWinConRad=0.09,
      nExt=1,
      CExt={5259932.23},
      alphaRad=5,
      alphaInt=2.12,
      nInt=1,
      RInt={0.000668895639141},
      RWin=0.01642857143,
      RExtRem=0.1265217391,
      alphaFloor=2.7,
      nFloor=1,
      RFloorRem=0.1265217391,
      CFloor={5259932.23},
      alphaRoof=2.7,
      nRoof=1,
      RRoofRem=0.1265217391,
      CRoof={5259932.23},
      extWallRC(thermCapExt(each der_T(fixed=true))),
      intWallRC(thermCapInt(each der_T(fixed=true))),
      floorRC(thermCapExt(each der_T(fixed=true))),
      roofRC(thermCapExt(each der_T(fixed=true))),
      VAir=144,
      AFloor=64,
      ARoof=64,
      redeclare package Medium = MediumA,
      RExt={0.00301421908725},
      RFloor={0.00301421908725},
      RRoof={0.00301421908725},
      nOrientations=4,
      AWin={12,4,12,0},
      ATransparent={12,4,12,0},
      AExt={12,3,12,18},
      AInt=60,
      CInt={6891363.86})
                      "Thermal zone"
      annotation (Placement(transformation(extent={{296,0},{344,36}})));
    Buildings.ThermalZones.ReducedOrder.EquivalentAirTemperature.VDI6007WithWindow
                                               eqAirTemp1(
      wfGro=0,
      withLongwave=true,
      aExt=0.7,
      alphaWallOut=20,
      alphaRad=5,
      alphaWinOut=20,
      n=4,
      wfWall={0.25,0.25,0.25,0.25},
      wfWin={0.25,0.25,0.25,0.25},
      TGro=283.15) "Computes equivalent air temperature"
      annotation (Placement(transformation(extent={{228,-12},{248,8}})));
    Modelica.Blocks.Math.Add solRad1[4]
      "Sums up solar radiation of both directions"
      annotation (Placement(transformation(extent={{214,8},{224,18}})));
    Buildings.HeatTransfer.Sources.PrescribedTemperature preTem2
      "Prescribed temperature for exterior walls outdoor surface temperature"
      annotation (Placement(transformation(extent={{260,-4},{272,8}})));
    Buildings.HeatTransfer.Sources.PrescribedTemperature preTem3
      "Prescribed temperature for windows outdoor surface temperature"
      annotation (Placement(transformation(extent={{260,16},{272,28}})));
    Modelica.Thermal.HeatTransfer.Components.Convection theConWin1
      "Outdoor convective heat transfer of windows"
      annotation (Placement(transformation(extent={{290,18},{280,28}})));
    Modelica.Thermal.HeatTransfer.Components.Convection theConWall1
      "Outdoor convective heat transfer of walls"
      annotation (Placement(transformation(extent={{288,8},{278,-2}})));
    Modelica.Blocks.Sources.Constant const2[4](
                                              each k=0)
      "Sets sunblind signal to zero (open)"
      annotation (Placement(transformation(extent={{232,16},{238,22}})));
    Modelica.Blocks.Sources.Constant alphaWall1(
                                               k=25*11.5)
      "Outdoor coefficient of heat transfer for walls"
      annotation (Placement(transformation(extent={{-4,-4},{4,4}},rotation=90,
      origin={282,-14})));
    Modelica.Blocks.Sources.Constant alphaWin1(
                                              k=20*14)
      "Outdoor coefficient of heat transfer for windows"
      annotation (Placement(transformation(extent={{4,-4},{-4,4}},
      rotation=90,origin={284,40})));
    Buildings.HeatTransfer.Sources.PrescribedTemperature preTemFloor1
      "Prescribed temperature for floor plate outdoor surface temperature"
      annotation (Placement(transformation(extent={{-6,-6},{6,6}},
      rotation=90,origin={319,-114})));
    Modelica.Blocks.Sources.Constant TSoil1(k=281.15)
      "Outdoor surface temperature for floor plate"
      annotation (Placement(transformation(extent={{-4,-4},{4,4}},
      rotation=90, origin={320,-128})));
    Buildings.ThermalZones.ReducedOrder.EquivalentAirTemperature.VDI6007
                                     eqAirTempVDI1(
      aExt=0.7,
      n=1,
      wfWall={1},
      wfWin={0},
      wfGro=0,
      alphaWallOut=20,
      alphaRad=5,
      TGro=283.15) "Computes equivalent air temperature for roof"
      annotation (Placement(transformation(extent={{282,76},{302,96}})));
    Buildings.HeatTransfer.Sources.PrescribedTemperature preTemRoof1
      "Prescribed temperature for roof outdoor surface temperature"
      annotation (Placement(transformation(extent={{-6,-6},{6,6}},rotation=-90,
      origin={319,66})));
    Modelica.Thermal.HeatTransfer.Components.Convection theConRoof1
      "Outdoor convective heat transfer of roof"
      annotation (Placement(transformation(extent={{5,-5},{-5,5}},rotation=-90,
      origin={319,49})));
    Modelica.Blocks.Sources.Constant alphaRoof1(
                                               k=25*11.5)
      "Outdoor coefficient of heat transfer for roof"
      annotation (Placement(transformation(extent={{4,-4},{-4,4}},origin={338,49})));
    Modelica.Blocks.Sources.Constant const3(k=0)
      "Sets sunblind signal to zero (open)"
      annotation (Placement(transformation(extent={{316,98},{310,104}})));
    Buildings.BoundaryConditions.SolarIrradiation.DiffusePerez
                                                     HDifTil2[
                                                             2](
      each outSkyCon=true,
      each outGroCon=true,
      each til=1.5707963267949,
      each lat=0.72954762733363,
      azi={0,3.1415926535898})
      "Calculates diffuse solar radiation on titled surface for both directions"
      annotation (Placement(transformation(extent={{84,-122},{104,-102}})));
    Buildings.BoundaryConditions.SolarIrradiation.DirectTiltedSurface
                                                            HDirTil2[
                                                                    2](
      each til=1.5707963267949,
      each lat=0.72954762733363,
      azi={0,3.1415926535898})
      "Calculates direct solar radiation on titled surface for both directions"
      annotation (Placement(transformation(extent={{84,-90},{104,-70}})));
    Buildings.ThermalZones.ReducedOrder.SolarGain.CorrectionGDoublePane
                                    corGDouPan2(
                                               UWin=2.1, n=2)
      "Correction factor for solar transmission"
      annotation (Placement(transformation(extent={{158,-96},{178,-76}})));
    Buildings.ThermalZones.ReducedOrder.RC.FourElements South(
      alphaExt=2.7,
      alphaWin=2.7,
      gWin=1,
      ratioWinConRad=0.09,
      nExt=1,
      alphaRad=5,
      alphaInt=2.12,
      nInt=1,
      RInt={0.000668895639141},
      RWin=0.01642857143,
      RExtRem=0.1265217391,
      alphaFloor=2.7,
      nFloor=1,
      RFloorRem=0.1265217391,
      CFloor={5259932.23},
      alphaRoof=2.7,
      nRoof=1,
      RRoofRem=0.1265217391,
      CRoof={5259932.23},
      nOrientations=2,
      extWallRC(thermCapExt(each der_T(fixed=true))),
      intWallRC(thermCapInt(each der_T(fixed=true))),
      floorRC(thermCapExt(each der_T(fixed=true))),
      roofRC(thermCapExt(each der_T(fixed=true))),
      VAir=144,
      AFloor=64,
      ARoof=64,
      redeclare package Medium = MediumA,
      RExt={0.00301421908725},
      RFloor={0.00301421908725},
      RRoof={0.00301421908725},
      AWin={0,2*4},
      ATransparent={0,2*4},
      AExt={24,16},
      CExt={2659932.23},
      AInt=50,
      CInt={6591363.86})
                      "Thermal zone"
      annotation (Placement(transformation(extent={{196,-144},{244,-108}})));
    Buildings.ThermalZones.ReducedOrder.EquivalentAirTemperature.VDI6007WithWindow
                                               eqAirTemp2(
      wfGro=0,
      withLongwave=true,
      aExt=0.7,
      alphaWallOut=20,
      alphaRad=5,
      alphaWinOut=20,
      n=2,
      wfWall={0.3043478260869566,0.6956521739130435},
      wfWin={0.5,0.5},
      TGro=283.15) "Computes equivalent air temperature"
      annotation (Placement(transformation(extent={{128,-156},{148,-136}})));
    Modelica.Blocks.Math.Add solRad2[
                                    2]
      "Sums up solar radiation of both directions"
      annotation (Placement(transformation(extent={{114,-136},{124,-126}})));
    Buildings.HeatTransfer.Sources.PrescribedTemperature preTem4
      "Prescribed temperature for exterior walls outdoor surface temperature"
      annotation (Placement(transformation(extent={{160,-148},{172,-136}})));
    Buildings.HeatTransfer.Sources.PrescribedTemperature preTem5
      "Prescribed temperature for windows outdoor surface temperature"
      annotation (Placement(transformation(extent={{160,-128},{172,-116}})));
    Modelica.Thermal.HeatTransfer.Components.Convection theConWin2
      "Outdoor convective heat transfer of windows"
      annotation (Placement(transformation(extent={{190,-126},{180,-116}})));
    Modelica.Thermal.HeatTransfer.Components.Convection theConWall2
      "Outdoor convective heat transfer of walls"
      annotation (Placement(transformation(extent={{188,-136},{178,-146}})));
    Modelica.Blocks.Sources.Constant const4[
                                           2](each k=0)
      "Sets sunblind signal to zero (open)"
      annotation (Placement(transformation(extent={{132,-128},{138,-122}})));
    Modelica.Blocks.Sources.Constant alphaWall2(
                                               k=25*11.5)
      "Outdoor coefficient of heat transfer for walls"
      annotation (Placement(transformation(extent={{-4,-4},{4,4}},rotation=90,
      origin={182,-158})));
    Modelica.Blocks.Sources.Constant alphaWin2(
                                              k=20*14)
      "Outdoor coefficient of heat transfer for windows"
      annotation (Placement(transformation(extent={{4,-4},{-4,4}},
      rotation=90,origin={184,-104})));
    Buildings.ThermalZones.ReducedOrder.EquivalentAirTemperature.VDI6007
                                     eqAirTempVDI2(
      aExt=0.7,
      n=1,
      wfWall={1},
      wfWin={0},
      wfGro=0,
      alphaWallOut=20,
      alphaRad=5,
      TGro=283.15) "Computes equivalent air temperature for roof"
      annotation (Placement(transformation(extent={{182,-68},{202,-48}})));
    Buildings.HeatTransfer.Sources.PrescribedTemperature preTemRoof2
      "Prescribed temperature for roof outdoor surface temperature"
      annotation (Placement(transformation(extent={{-6,-6},{6,6}},rotation=-90,
      origin={219,-78})));
    Modelica.Thermal.HeatTransfer.Components.Convection theConRoof2
      "Outdoor convective heat transfer of roof"
      annotation (Placement(transformation(extent={{5,-5},{-5,5}},rotation=-90,
      origin={219,-95})));
    Modelica.Blocks.Sources.Constant alphaRoof2(
                                               k=25*11.5)
      "Outdoor coefficient of heat transfer for roof"
      annotation (Placement(transformation(extent={{4,-4},{-4,4}},origin={238,-95})));
    Modelica.Blocks.Sources.Constant const5(k=0)
      "Sets sunblind signal to zero (open)"
      annotation (Placement(transformation(extent={{212,-46},{206,-40}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a convSouth
      annotation (Placement(transformation(extent={{132,-188},{152,-168}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a radNorth
      annotation (Placement(transformation(extent={{250,-190},{270,-170}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a radEast
      annotation (Placement(transformation(extent={{350,32},{370,52}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a convEast
      annotation (Placement(transformation(extent={{350,-30},{370,-10}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a radWest
      annotation (Placement(transformation(extent={{-112,32},{-92,52}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a convWest
      annotation (Placement(transformation(extent={{-112,-40},{-92,-20}})));
    Modelica.Blocks.Routing.Multiplex3 multiplex3_1
      annotation (Placement(transformation(extent={{326,-56},{346,-36}})));
    Modelica.Blocks.Interfaces.RealOutput TAir[3]
      "West/East/South air temperature"
      annotation (Placement(transformation(extent={{360,-56},{380,-36}})));
    FBM.Tutorial.JointWall South_West(
      n=1,
      alphaJoi=2.12,
      AJoi=6,
      RJoi={0.000595693407511},
      RJoiRem=0.000595693407511,
      CJoi={1228895.6},
      T_start=292.15)
      annotation (Placement(transformation(extent={{112,-50},{132,-30}})));
    FBM.Tutorial.JointWall South_East(
      n=1,
      alphaJoi=2.12,
      AJoi=6,
      RJoi={0.000595693407511},
      RJoiRem=0.000595693407511,
      CJoi={1228895.6},
      T_start=292.15)
      annotation (Placement(transformation(extent={{260,-54},{280,-34}})));
  equation
    connect(eqAirTemp.TEqAirWin,preTem1. T)
      annotation (Line(points={{7,9.8},{10,9.8},{10,30},{16.8,30}},
      color={0,0,127}));
    connect(eqAirTemp.TEqAir,preTem. T)
      annotation (Line(points={{7,6},{14,6},{14,10},{16.8,10}},
      color={0,0,127}));
    connect(weaBus.TDryBul,eqAirTemp. TDryBul)
      annotation (Line(points={{-75,72},{-75,8},{-28,8},{-28,0},{-16,0}},
      color={255,204,51},
      thickness=0.5), Text(string="%first",index=-1,extent={{-6,3},{-6,3}}));
    connect(const.y,eqAirTemp. sunblind)
      annotation (Line(points={{-3.7,27},{-2,27},{-2,18},{-4,18}},
      color={0,0,127}));
    connect(HDifTil.HSkyDifTil,corGDouPan. HSkyDifTil)
      annotation (Line(points={{-37,46},{-18,46},{4,46},{4,68},{10,68},{10,67.8},{
            14,67.8},{14,68}},
      color={0,0,127}));
    connect(HDirTil.H,corGDouPan. HDirTil)
      annotation (Line(points={{-39,72},{-12,72},{14,72}},
                                                         color={0,0,127}));
    connect(HDifTil.H,solRad. u2)
      annotation (Line(points={{-37,40},{-34,40},{-34,18},{-29,18}},
      color={0,0,127}));
    connect(HDifTil.HGroDifTil,corGDouPan. HGroDifTil)
      annotation (Line(points={{-37,34},{6,34},{6,64},{14,64}},
      color={0,0,127}));
    connect(solRad.y,eqAirTemp. HSol)
      annotation (Line(points={{-17.5,21},{-16,21},{-16,12}},
      color={0,0,127}));
    connect(weaBus,HDifTil [1].weaBus)
      annotation (Line(points={{-75,72},{-64,72},{-64,40},{-58,40}},
      color={255,204,51},thickness=0.5));
    connect(weaBus,HDifTil [2].weaBus)
      annotation (Line(points={{-75,72},{-64,72},{-64,40},{-58,40}},
      color={255,204,51},thickness=0.5));
     connect(weaBus,HDifTil [3].weaBus)
      annotation (Line(points={{-75,72},{-64,72},{-64,40},{-58,40}},
      color={255,204,51},thickness=0.5));
    connect(weaBus,HDirTil [1].weaBus)
      annotation (Line(
      points={{-75,72},{-68,72},{-60,72}},
      color={255,204,51},
      thickness=0.5));
    connect(weaBus,HDirTil [2].weaBus)
      annotation (Line(
      points={{-75,72},{-68,72},{-60,72}},
      color={255,204,51},
      thickness=0.5));
     connect(weaBus,HDirTil [3].weaBus)
      annotation (Line(
      points={{-75,72},{-68,72},{-60,72}},
      color={255,204,51},
      thickness=0.5));
    connect(theConWin.solid, West.window) annotation (Line(points={{48,31},{50,31},
            {50,30},{54,30}}, color={191,0,0}));
    connect(preTem1.port,theConWin. fluid)
      annotation (Line(points={{30,30},{38,30},{38,31}}, color={191,0,0}));
    connect(West.extWall, theConWall.solid) annotation (Line(points={{54,22},{50,22},
            {50,11},{46,11}}, color={191,0,0}));
    connect(theConWall.fluid,preTem. port)
      annotation (Line(points={{36,11},{34,11},{34,10},{30,10}},
                                                             color={191,0,0}));
    connect(alphaWall.y,theConWall. Gc)
      annotation (Line(points={{40,-1.6},{40,6},{41,6}},    color={0,0,127}));
    connect(alphaWin.y,theConWin. Gc)
      annotation (Line(points={{42,43.6},{42,36},{43,36}}, color={0,0,127}));
    connect(weaBus.TBlaSky,eqAirTemp. TBlaSky)
      annotation (Line(
      points={{-75,72},{-74,72},{-74,6},{-16,6}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
    connect(preTemRoof.port,theConRoof. fluid)
      annotation (Line(points={{77,68},{77,68},{77,62}}, color={191,0,0}));
    connect(theConRoof.solid, West.roof)
      annotation (Line(points={{77,52},{76.9,52},{76.9,44}}, color={191,0,0}));
    connect(eqAirTempVDI.TEqAir,preTemRoof. T)
      annotation (Line(
      points={{61,94},{77,94},{77,81.2}}, color={0,0,127}));
    connect(theConRoof.Gc,alphaRoof. y)
      annotation (Line(points={{82,57},{88,57},{91.6,57}},color={0,0,127}));
    connect(eqAirTempVDI.TDryBul,eqAirTemp. TDryBul)
      annotation (Line(points={{38,88},{-86,88},{-86,8},{-28,8},{-28,0},{-16,0}},
      color={0,0,127}));
    connect(eqAirTempVDI.TBlaSky,eqAirTemp. TBlaSky)
      annotation (Line(points={{38,94},{-24,94},{-88,94},{-88,2},{-48,2},{-48,12},
            {-22,12},{-22,6},{-16,6}},
      color={0,0,127}));
    connect(eqAirTempVDI.HSol[1],weaBus. HGloHor)
      annotation (Line(points={{38,100},{-90,100},{-90,72},{-75,72}},
      color={0,0,127}),Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
    connect(HDirTil.inc,corGDouPan. inc)
      annotation (Line(points={{-39,68},{-39,70},{-12,70},{-12,60},{14,60}},
      color={0,0,127}));
    connect(const1.y,eqAirTempVDI. sunblind[1])
      annotation (Line(points={{67.7,109},{66,109},{66,108},{50,108},{50,106}},
                                        color={0,0,127}));
    connect(corGDouPan.solarRadWinTrans, West.solRad) annotation (Line(points={{37,
            66},{50,66},{50,41},{53,41}}, color={0,0,127}));
    connect(eqAirTemp1.TEqAirWin, preTem3.T) annotation (Line(points={{249,1.8},{252,
            1.8},{252,22},{258.8,22}}, color={0,0,127}));
    connect(eqAirTemp1.TEqAir, preTem2.T) annotation (Line(points={{249,-2},{256,-2},
            {256,2},{258.8,2}}, color={0,0,127}));
    connect(weaBus.TDryBul, eqAirTemp1.TDryBul) annotation (Line(
        points={{-75,72},{-75,122},{180,122},{180,-2},{204,-2},{204,-8},{226,-8}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(const2.y, eqAirTemp1.sunblind) annotation (Line(points={{238.3,19},{240,
            19},{240,10},{238,10}}, color={0,0,127}));
    connect(HDifTil1.HSkyDifTil, corGDouPan1.HSkyDifTil) annotation (Line(points={
            {205,38},{224,38},{246,38},{246,60},{252,60},{252,59.8},{256,59.8},{256,
            60}}, color={0,0,127}));
    connect(HDirTil1.H, corGDouPan1.HDirTil)
      annotation (Line(points={{205,64},{256,64}}, color={0,0,127}));
    connect(HDifTil1.H, solRad1.u2) annotation (Line(points={{205,32},{208,32},{208,
            10},{213,10}}, color={0,0,127}));
    connect(HDifTil1.HGroDifTil, corGDouPan1.HGroDifTil) annotation (Line(points={
            {205,26},{248,26},{248,56},{256,56}}, color={0,0,127}));
    connect(solRad1.y, eqAirTemp1.HSol)
      annotation (Line(points={{224.5,13},{226,13},{226,4}}, color={0,0,127}));
    connect(theConWin1.solid, East.window) annotation (Line(points={{290,23},{292,
            23},{292,22},{296,22}}, color={191,0,0}));
    connect(preTem3.port, theConWin1.fluid)
      annotation (Line(points={{272,22},{280,22},{280,23}}, color={191,0,0}));
    connect(East.extWall, theConWall1.solid) annotation (Line(points={{296,14},{292,
            14},{292,3},{288,3}}, color={191,0,0}));
    connect(theConWall1.fluid, preTem2.port) annotation (Line(points={{278,3},{276,
            3},{276,2},{272,2}}, color={191,0,0}));
    connect(alphaWall1.y, theConWall1.Gc)
      annotation (Line(points={{282,-9.6},{282,-2},{283,-2}}, color={0,0,127}));
    connect(alphaWin1.y, theConWin1.Gc)
      annotation (Line(points={{284,35.6},{284,28},{285,28}}, color={0,0,127}));
    connect(weaBus.TBlaSky, eqAirTemp1.TBlaSky) annotation (Line(
        points={{-75,72},{-74,72},{-74,122},{180,122},{180,-2},{226,-2}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(preTemFloor1.port, East.floor)
      annotation (Line(points={{319,-108},{320,-108},{320,0}},
                                                           color={191,0,0}));
    connect(TSoil1.y, preTemFloor1.T) annotation (Line(points={{320,-123.6},{319,-123.6},
            {319,-121.2}},color={0,0,127}));
    connect(preTemRoof1.port, theConRoof1.fluid)
      annotation (Line(points={{319,60},{319,60},{319,54}}, color={191,0,0}));
    connect(theConRoof1.solid, East.roof) annotation (Line(points={{319,44},{318.9,
            44},{318.9,36}}, color={191,0,0}));
    connect(eqAirTempVDI1.TEqAir, preTemRoof1.T)
      annotation (Line(points={{303,86},{319,86},{319,73.2}}, color={0,0,127}));
    connect(theConRoof1.Gc, alphaRoof1.y)
      annotation (Line(points={{324,49},{330,49},{333.6,49}}, color={0,0,127}));
    connect(eqAirTempVDI1.TDryBul, eqAirTemp1.TDryBul) annotation (Line(points={{280,
            80},{156,80},{156,0},{214,0},{214,-8},{226,-8}}, color={0,0,127}));
    connect(eqAirTempVDI1.TBlaSky, eqAirTemp1.TBlaSky) annotation (Line(points={{280,
            86},{218,86},{154,86},{154,-6},{194,-6},{194,4},{220,4},{220,-2},{226,
            -2}}, color={0,0,127}));
    connect(eqAirTempVDI1.HSol[1], weaBus.HGloHor) annotation (Line(points={{280,92},
            {144,92},{144,118},{-74,118},{-74,72},{-75,72}},
                                         color={0,0,127}), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(HDirTil1.inc, corGDouPan1.inc) annotation (Line(points={{205,60},{224,
            60},{242,60},{242,52},{256,52}}, color={0,0,127}));
    connect(const3.y, eqAirTempVDI1.sunblind[1]) annotation (Line(points={{309.7,101},
            {292,101},{292,100},{292,98}},          color={0,0,127}));
    connect(corGDouPan1.solarRadWinTrans, East.solRad) annotation (Line(points={{279,
            58},{292,58},{292,33},{295,33}}, color={0,0,127}));
    connect(eqAirTemp2.TEqAirWin, preTem5.T) annotation (Line(points={{149,-142.2},
            {152,-142.2},{152,-122},{158.8,-122}}, color={0,0,127}));
    connect(eqAirTemp2.TEqAir, preTem4.T) annotation (Line(points={{149,-146},{156,
            -146},{156,-142},{158.8,-142}}, color={0,0,127}));
    connect(weaBus.TDryBul, eqAirTemp2.TDryBul) annotation (Line(
        points={{-75,72},{-75,-144},{114,-144},{114,-152},{126,-152}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(const4.y, eqAirTemp2.sunblind) annotation (Line(points={{138.3,-125},{
            140,-125},{140,-134},{138,-134}}, color={0,0,127}));
    connect(HDifTil2.HSkyDifTil, corGDouPan2.HSkyDifTil) annotation (Line(points={
            {105,-106},{124,-106},{146,-106},{146,-84},{152,-84},{152,-84.2},{156,
            -84.2},{156,-84}}, color={0,0,127}));
    connect(HDirTil2.H, corGDouPan2.HDirTil)
      annotation (Line(points={{105,-80},{156,-80}}, color={0,0,127}));
    connect(HDirTil2.H, solRad2.u1) annotation (Line(points={{105,-80},{110,-80},{
            110,-128},{113,-128}}, color={0,0,127}));
    connect(HDifTil2.H, solRad2.u2) annotation (Line(points={{105,-112},{108,-112},
            {108,-134},{113,-134}}, color={0,0,127}));
    connect(HDifTil2.HGroDifTil, corGDouPan2.HGroDifTil) annotation (Line(points={
            {105,-118},{148,-118},{148,-88},{156,-88}}, color={0,0,127}));
    connect(solRad2.y, eqAirTemp2.HSol) annotation (Line(points={{124.5,-131},{126,
            -131},{126,-140}}, color={0,0,127}));
    connect(theConWin2.solid, South.window) annotation (Line(points={{190,-121},{192,
            -121},{192,-122},{196,-122}}, color={191,0,0}));
    connect(preTem5.port, theConWin2.fluid) annotation (Line(points={{172,-122},{180,
            -122},{180,-121}}, color={191,0,0}));
    connect(South.extWall, theConWall2.solid) annotation (Line(points={{196,-130},
            {192,-130},{192,-141},{188,-141}}, color={191,0,0}));
    connect(theConWall2.fluid, preTem4.port) annotation (Line(points={{178,-141},{
            176,-141},{176,-142},{172,-142}}, color={191,0,0}));
    connect(alphaWall2.y, theConWall2.Gc) annotation (Line(points={{182,-153.6},{182,
            -146},{183,-146}}, color={0,0,127}));
    connect(alphaWin2.y, theConWin2.Gc) annotation (Line(points={{184,-108.4},{184,
            -116},{185,-116}}, color={0,0,127}));
    connect(weaBus.TBlaSky, eqAirTemp2.TBlaSky) annotation (Line(
        points={{-75,72},{-76,72},{-76,-146},{126,-146}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(preTemRoof2.port, theConRoof2.fluid)
      annotation (Line(points={{219,-84},{219,-84},{219,-90}}, color={191,0,0}));
    connect(theConRoof2.solid, South.roof) annotation (Line(points={{219,-100},{218.9,
            -100},{218.9,-108}}, color={191,0,0}));
    connect(eqAirTempVDI2.TEqAir, preTemRoof2.T) annotation (Line(points={{203,-58},
            {219,-58},{219,-70.8}}, color={0,0,127}));
    connect(theConRoof2.Gc, alphaRoof2.y) annotation (Line(points={{224,-95},{230,
            -95},{233.6,-95}}, color={0,0,127}));
    connect(eqAirTempVDI2.TDryBul, eqAirTemp2.TDryBul) annotation (Line(points={{180,
            -64},{56,-64},{56,-144},{114,-144},{114,-152},{126,-152}}, color={0,0,
            127}));
    connect(eqAirTempVDI2.TBlaSky, eqAirTemp2.TBlaSky) annotation (Line(points={{180,
            -58},{118,-58},{54,-58},{54,-150},{94,-150},{94,-140},{120,-140},{120,
            -146},{126,-146}}, color={0,0,127}));
    connect(eqAirTempVDI2.HSol[1], weaBus.HGloHor) annotation (Line(points={{180,-52},
            {180,-52},{-76,-52},{-76,72},{-75,72}},
                                        color={0,0,127}), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(HDirTil2.inc, corGDouPan2.inc) annotation (Line(points={{105,-84},{124,
            -84},{142,-84},{142,-92},{156,-92}}, color={0,0,127}));
    connect(const5.y, eqAirTempVDI2.sunblind[1]) annotation (Line(points={{205.7,-43},
            {192,-43},{192,-44},{192,-44},{192,-46}}, color={0,0,127}));
    connect(corGDouPan2.solarRadWinTrans, South.solRad) annotation (Line(points={{
            179,-86},{192,-86},{192,-111},{195,-111}}, color={0,0,127}));
    connect(weaBus,HDifTil1 [1].weaBus)
      annotation (Line(points={{-75,72},{-74,72},{-74,122},{180,122},{180,52},{180,
            32},{184,32}},
      color={255,204,51},thickness=0.5));
    connect(weaBus,HDifTil1 [2].weaBus)
      annotation (Line(points={{-75,72},{-74,72},{-74,122},{180,122},{180,32},{184,
            32}},
      color={255,204,51},thickness=0.5));
    connect(weaBus,HDifTil1 [3].weaBus)
      annotation (Line(points={{-75,72},{-74,72},{-74,122},{180,122},{180,32},{184,
            32}},
      color={255,204,51},thickness=0.5));
    connect(weaBus,HDirTil1 [1].weaBus)
      annotation (Line(
      points={{-75,72},{-74,72},{-74,122},{180,122},{180,64},{184,64}},
      color={255,204,51},
      thickness=0.5));
    connect(weaBus,HDirTil1 [2].weaBus)
      annotation (Line(
      points={{-75,72},{-74,72},{-74,80},{-74,122},{180,122},{180,68},{180,64},{184,
            64}},
      color={255,204,51},
      thickness=0.5));
    connect(weaBus,HDirTil1 [3].weaBus)
      annotation (Line(
      points={{-75,72},{-74,72},{-74,80},{-74,122},{180,122},{180,68},{180,64},{184,
            64}},
      color={255,204,51},
      thickness=0.5));
       connect(weaBus,HDifTil2 [1].weaBus)
      annotation (Line(points={{-75,72},{-76,72},{-76,-112},{84,-112}},
      color={255,204,51},thickness=0.5));
    connect(weaBus,HDifTil2 [2].weaBus)
      annotation (Line(points={{-75,72},{-76,72},{-76,-112},{84,-112}},
      color={255,204,51},thickness=0.5));
    connect(weaBus,HDirTil2 [1].weaBus)
      annotation (Line(
      points={{-75,72},{-76,72},{-76,-80},{84,-80}},
      color={255,204,51},
      thickness=0.5));
    connect(weaBus,HDirTil2 [2].weaBus)
      annotation (Line(
      points={{-75,72},{-76,72},{-76,-80},{84,-80}},
      color={255,204,51},
      thickness=0.5));
    connect(South.intGainsConv, convSouth) annotation (Line(points={{244,-122},{250,
            -122},{250,-178},{142,-178}}, color={191,0,0}));
    connect(South.intGainsRad, radNorth) annotation (Line(points={{244,-118},{260,
            -118},{260,-180}}, color={191,0,0}));
    connect(East.intGainsRad, radEast) annotation (Line(points={{344,26},{354,26},
            {354,42},{360,42}}, color={191,0,0}));
    connect(East.intGainsConv, convEast) annotation (Line(points={{344,22},{350,22},
            {350,-20},{360,-20}}, color={191,0,0}));
    connect(convWest, West.intGainsConv) annotation (Line(points={{-102,-30},{108,
            -30},{108,30},{102,30}}, color={191,0,0}));
    connect(radWest, West.intGainsRad) annotation (Line(points={{-102,42},{-96,42},
            {-96,132},{122,132},{122,34},{102,34}}, color={191,0,0}));
    connect(preTemFloor1.port, West.floor) annotation (Line(points={{319,-108},{294,
            -108},{294,-104},{294,-26},{78,-26},{78,8}}, color={191,0,0}));
    connect(preTemFloor1.port, South.floor) annotation (Line(points={{319,-108},{294,
            -108},{294,-160},{220,-160},{220,-144}}, color={191,0,0}));
    connect(HDirTil.H, solRad.u1) annotation (Line(points={{-39,72},{-34,72},{-34,
            24},{-29,24}}, color={0,0,127}));
    connect(HDirTil1.H, solRad1.u1) annotation (Line(points={{205,64},{208,64},{208,
            16},{213,16}}, color={0,0,127}));
    connect(West.TAir, multiplex3_1.u1[1]) annotation (Line(points={{103,42},{
            122,42},{140,42},{140,-38},{324,-38},{324,-39}}, color={0,0,127}));
    connect(East.TAir, multiplex3_1.u2[1])
      annotation (Line(points={{345,34},{324,34},{324,-46}}, color={0,0,127}));
    connect(South.TAir, multiplex3_1.u3[1]) annotation (Line(points={{245,-110},
            {276,-110},{276,-53},{324,-53}}, color={0,0,127}));
    connect(multiplex3_1.y, TAir)
      annotation (Line(points={{347,-46},{370,-46}}, color={0,0,127}));
    connect(weaBus, HDifTil[4].weaBus) annotation (Line(
        points={{-75,72},{-64,72},{-64,40},{-58,40}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(weaBus, HDirTil[4].weaBus) annotation (Line(
        points={{-75,72},{-67.5,72},{-60,72}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(weaBus, HDirTil1[4].weaBus) annotation (Line(
        points={{-75,72},{-74,72},{-74,122},{180,122},{180,64},{184,64}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(weaBus, HDifTil1[4].weaBus) annotation (Line(
        points={{-75,72},{-74,72},{-74,122},{180,122},{180,32},{184,32},{184,32}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(West.intGainsConv, South_West.AirPort_A) annotation (Line(points={{
            102,30},{108,30},{108,-40.9091},{112.2,-40.9091}}, color={191,0,0}));
    connect(South_West.AirPort_A1, South.intGainsConv) annotation (Line(points=
            {{132,-40.9091},{194,-40.9091},{194,-40},{254,-40},{254,-122},{244,
            -122}}, color={191,0,0}));
    connect(South.intGainsConv, South_East.AirPort_A) annotation (Line(points={
            {244,-122},{254,-122},{254,-44.9091},{260.2,-44.9091}}, color={191,
            0,0}));
    connect(East.intGainsConv, South_East.AirPort_A1) annotation (Line(points={
            {344,22},{342,22},{342,-28},{280,-28},{280,-44.9091}}, color={191,0,
            0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-180},
              {360,140}}), graphics={
          Rectangle(
            extent={{-100,140},{360,-180}},
            lineColor={28,108,200},
            fillColor={255,255,170},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{16,104},{88,-102}},
            lineColor={28,108,200},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{216,104},{288,-102}},
            lineColor={28,108,200},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-30,64},{30,-64}},
            lineColor={28,108,200},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid,
            origin={152,-72},
            rotation=90),
          Rectangle(
            extent={{14,50},{18,-32}},
            lineColor={28,108,200},
            fillColor={85,255,255},
            fillPattern=FillPattern.Solid,
            lineThickness=0.5),
          Rectangle(
            extent={{286,46},{290,-36}},
            lineColor={28,108,200},
            fillColor={85,255,255},
            fillPattern=FillPattern.Solid,
            lineThickness=0.5),
          Rectangle(
            extent={{-1.5,18.5},{1.5,-18.5}},
            lineColor={28,108,200},
            fillColor={85,255,255},
            fillPattern=FillPattern.Solid,
            origin={47.5,-101.5},
            rotation=90,
            lineThickness=0.5),
          Rectangle(
            extent={{-1.5,18.5},{1.5,-18.5}},
            lineColor={28,108,200},
            fillColor={85,255,255},
            fillPattern=FillPattern.Solid,
            origin={119.5,-101.5},
            rotation=90,
            lineThickness=0.5),
          Rectangle(
            extent={{-1.5,18.5},{1.5,-18.5}},
            lineColor={28,108,200},
            fillColor={85,255,255},
            fillPattern=FillPattern.Solid,
            origin={187.5,-101.5},
            rotation=90,
            lineThickness=0.5),
          Rectangle(
            extent={{-1.5,18.5},{1.5,-18.5}},
            lineColor={28,108,200},
            fillColor={85,255,255},
            fillPattern=FillPattern.Solid,
            origin={253.5,-101.5},
            rotation=90,
            lineThickness=0.5),
          Rectangle(
            extent={{86,72},{90,-10}},
            lineColor={28,108,200},
            fillColor={85,255,255},
            fillPattern=FillPattern.Solid,
            lineThickness=0.5),
          Rectangle(
            extent={{214,72},{218,-10}},
            lineColor={28,108,200},
            fillColor={85,255,255},
            fillPattern=FillPattern.Solid,
            lineThickness=0.5),
          Text(
            extent={{72,-98},{82,-100}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={255,170,85},
            fillPattern=FillPattern.Solid,
            textString="3"),
          Text(
            extent={{14,-52},{24,-54}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={255,170,85},
            fillPattern=FillPattern.Solid,
            textString="1"),
          Text(
            extent={{78,-16},{88,-18}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={255,170,85},
            fillPattern=FillPattern.Solid,
            textString="2"),
          Text(
            extent={{48,98},{58,96}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={255,170,85},
            fillPattern=FillPattern.Solid,
            textString="4"),
          Text(
            extent={{278,-42},{288,-44}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={255,170,85},
            fillPattern=FillPattern.Solid,
            textString="1"),
          Text(
            extent={{246,100},{256,98}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={255,170,85},
            fillPattern=FillPattern.Solid,
            textString="4"),
          Text(
            extent={{216,-14},{226,-16}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={255,170,85},
            fillPattern=FillPattern.Solid,
            textString="2"),
          Text(
            extent={{274,-96},{284,-98}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={255,170,85},
            fillPattern=FillPattern.Solid,
            textString="3"),
          Text(
            extent={{146,-48},{156,-50}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={255,170,85},
            fillPattern=FillPattern.Solid,
            textString="1"),
          Text(
            extent={{146,-96},{156,-98}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={255,170,85},
            fillPattern=FillPattern.Solid,
            textString="2"),
          Text(
            extent={{96,-82},{106,-84}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={255,170,85},
            fillPattern=FillPattern.Solid,
            textString="3"),
          Text(
            extent={{204,-78},{214,-80}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={255,170,85},
            fillPattern=FillPattern.Solid,
            textString="4")}),                                     Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-100,-180},{360,140}})));
  end HouseModel_FirstTuto;

  model JointWall
    parameter Integer n(min = 1) "Number of RC-elements";
    parameter Modelica.SIunits.ThermalResistance RJoi[n](
      each min=Modelica.Constants.small)
      "Vector of resistors, from port_a to port_b"
      annotation(Dialog(group="Thermal mass"));
    parameter Modelica.SIunits.Area AJoi
      "Area of the wall"
      annotation(Dialog(group="Walls"));
   parameter Modelica.SIunits.CoefficientOfHeatTransfer alphaJoi
      "Convective coefficient of heat transfer of the wall"
      annotation(Dialog(group="walls"));
    parameter Modelica.SIunits.ThermalResistance RJoiRem(
      min=Modelica.Constants.small)
      "Resistance of remaining resistor RJoiRem between capacitor n and port_b"
       annotation(Dialog(group="Thermal mass"));
    parameter Modelica.SIunits.HeatCapacity CJoi[n](
      each min=Modelica.Constants.small)
      "Vector of heat capacities, from port_a to port_b"
      annotation(Dialog(group="Thermal mass"));
    parameter Modelica.SIunits.Temperature T_start
      "Initial temperature of capacities"
      annotation(Dialog(group="Thermal mass"));
    Modelica.Thermal.HeatTransfer.Components.HeatCapacitor thermCapJoi[n](
      final C=CJoi, each T(start=T_start)) "vector of thermal capacitors"
      annotation (Placement(transformation(extent={{-10,-12},{10,-32}})));
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor thermResJoi[n](
      final R=RJoi)
      "vector of thermal resistors connecting port_a and capacitors"
      annotation (Placement(transformation(extent={{-34,-10},{-14,10}})));
    Modelica.Thermal.HeatTransfer.Components.ThermalResistor thermResJoiRem(
      final R=RJoiRem)
      "single thermal resistor connecting least capacitor to port_b"
      annotation (Placement(transformation(extent={{10,-10},{30,10}})));
    Modelica.Thermal.HeatTransfer.Components.Convection convJoiWall_A
      "Convective heat transfer of jointure walls"
      annotation (Placement(transformation(extent={{-54,10},{-74,-10}})));
    Modelica.Thermal.HeatTransfer.Components.Convection convJoiWall_A1
      "Convective heat transfer of jointure walls"
      annotation (Placement(transformation(extent={{46,10},{66,-10}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a AirPort_A
      "Thermal connector to the air volume A1 (conv air port)"
      annotation (Placement(transformation(extent={{-108,-10},{-88,10}})));
    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a AirPort_A1
      "Thermal connector to the air volume A1 (conv air port)"
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    Modelica.Blocks.Sources.Constant alphaJoiWall(k=AJoi*alphaJoi)
      "Coefficient of convective heat transfer for interior walls"
      annotation (Placement(transformation(
      extent={{5,-5},{-5,5}},
      rotation=-90,
      origin={-64,-27})));
    Modelica.Blocks.Sources.Constant alphaJoiWall_1(k=AJoi*alphaJoi)
      "Coefficient of convective heat transfer for interior walls" annotation (
        Placement(transformation(
          extent={{5,-5},{-5,5}},
          rotation=-90,
          origin={56,-23})));
  equation
    connect(convJoiWall_A1.fluid, AirPort_A1)
      annotation (Line(points={{66,0},{100,0}}, color={191,0,0}));
    connect(convJoiWall_A.Gc, alphaJoiWall.y) annotation (Line(points={{-64,-10},{
            -64,-21.5}},             color={0,0,127}));
    connect(convJoiWall_A1.Gc, alphaJoiWall_1.y) annotation (Line(points={{56,-10},
            {56,-16},{56,-17.5}},   color={0,0,127}));
    connect(convJoiWall_A.fluid, AirPort_A)
      annotation (Line(points={{-74,0},{-88,0},{-98,0}},   color={191,0,0}));
     // Connecting inner elements thermResJoi[i]--thermCapJoi[i] to n groups
    for i in 1:n loop
      connect(thermResJoi[i].port_b,thermCapJoi[i].port)
      annotation (Line(points={{-14,0},{0,0},{0,-12}}, color={191,0,0}));
    end for;
    // Connecting groups between each other thermCapJoi[i] -- thermResJoi[i+1]
    for i in 1:n-1 loop
      connect(thermCapJoi[i].port,thermResJoi[i+1].port_a)
       annotation (Line(points={{0,-12},{0,-12},{-38,-12},{-38,0},{-34,0}},
                                                  color={191,0,0}));
    end for;
    // Connecting first RC element to port_a ,
    // last RC-element to RExtRem and RExtRem to port_b
    connect(thermCapJoi[n].port,thermResJoiRem.port_a)
    annotation (Line(points={{0,-12},{0,-12},{0,0},{10,0}}, color={191,0,0}));
    connect(thermResJoiRem.port_b, convJoiWall_A1.solid)
      annotation (Line(points={{30,0},{38,0},{46,0}}, color={191,0,0}));
    connect(convJoiWall_A.solid, thermResJoi[1].port_a)
      annotation (Line(points={{-54,0},{-34,0},{-34,0}}, color={191,0,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false,  extent=
    {{-100,-100},{100,120}}), graphics={  Rectangle(extent = {{-86, 60}, {-34, 26}},
     fillColor = {255, 213, 170},
     fillPattern = FillPattern.Solid, lineColor = {175, 175, 175}), Rectangle(
      extent = {{-28, 60}, {26, 26}}, fillColor = {255, 213, 170},
     fillPattern = FillPattern.Solid, lineColor = {175, 175, 175}), Rectangle(
      extent = {{32, 60}, {86, 26}}, fillColor = {255, 213, 170},
     fillPattern = FillPattern.Solid, lineColor = {175, 175, 175}), Rectangle(
      extent = {{0, 20}, {54, -14}}, fillColor = {255, 213, 170},
     fillPattern = FillPattern.Solid, lineColor = {175, 175, 175}), Rectangle(
      extent = {{-60, 20}, {-6, -14}}, fillColor = {255, 213, 170},
     fillPattern = FillPattern.Solid, lineColor = {175, 175, 175}), Rectangle(
      extent = {{-86, -20}, {-34, -54}}, fillColor = {255, 213, 170},
     fillPattern =  FillPattern.Solid, lineColor = {175, 175, 175}), Rectangle(
      extent = {{-28, -20}, {26, -54}}, fillColor = {255, 213, 170},
     fillPattern = FillPattern.Solid, lineColor = {175, 175, 175}), Rectangle(
      extent = {{32, -20}, {86, -54}}, fillColor = {255, 213, 170},
     fillPattern = FillPattern.Solid, lineColor = {175, 175, 175}), Rectangle(
      extent = {{-60, -60}, {-6, -94}}, fillColor = {255, 213, 170},
     fillPattern = FillPattern.Solid, lineColor = {175, 175, 175}), Rectangle(
      extent = {{0, -60}, {54, -94}}, fillColor = {255, 213, 170},
     fillPattern = FillPattern.Solid, lineColor = {175, 175, 175}), Rectangle(
      extent = {{-60, 100}, {-6, 66}}, fillColor = {255, 213, 170},
     fillPattern = FillPattern.Solid, lineColor = {175, 175, 175}), Rectangle(
      extent = {{0, 100}, {54, 66}}, fillColor = {255, 213, 170},
     fillPattern = FillPattern.Solid, lineColor = {175, 175, 175}), Rectangle(
      extent = {{60, -60}, {86, -92}}, fillColor = {255, 213, 170},
     fillPattern = FillPattern.Solid, lineColor = {175, 175, 175}), Rectangle(
      extent = {{60, 20}, {86, -14}}, fillColor = {255, 213, 170},
     fillPattern = FillPattern.Solid, lineColor = {175, 175, 175}), Rectangle(
      extent = {{60, 100}, {86, 66}}, fillColor = {255, 213, 170},
     fillPattern =  FillPattern.Solid, lineColor = {175, 175, 175}), Rectangle(
      extent = {{-86, -60}, {-66, -94}}, fillColor = {255, 213, 170},
     fillPattern = FillPattern.Solid, lineColor = {175, 175, 175}), Rectangle(
      extent = {{-86, 20}, {-66, -14}}, fillColor = {255, 213, 170},
     fillPattern = FillPattern.Solid, lineColor = {175, 175, 175}), Rectangle(
      extent = {{-86, 100}, {-66, 66}}, fillColor = {255, 213, 170},
     fillPattern = FillPattern.Solid, lineColor = {175, 175, 175}),
     Line(points = {{-90, 0}, {90, 0}}, color = {0, 0, 0}, thickness = 0.5,
     smooth = Smooth.None), Rectangle(extent = {{-74, 12}, {-26, -10}},
     lineColor = {0, 0, 0}, lineThickness = 0.5, fillColor = {255, 255, 255},
     fillPattern = FillPattern.Solid), Rectangle(extent = {{28, 12}, {76, -10}},
     lineColor = {0, 0, 0}, lineThickness =  0.5, fillColor = {255, 255, 255},
     fillPattern = FillPattern.Solid), Line(points = {{-1, 0}, {-1, -32}},
     color = {0, 0, 0}, thickness = 0.5, smooth = Smooth.None),
     Line(points = {{-18, -32}, {16, -32}}, pattern = LinePattern.None,
     thickness = 0.5, smooth = Smooth.None), Line(points = {{-18, -44}, {16, -44}},
     pattern = LinePattern.None, thickness = 0.5, smooth = Smooth.None),
     Text(extent = {{-90, 142}, {90, 104}}, lineColor = {0, 0, 255},
     textString = "%name"),
     Line(points = {{18, -32}, {-20, -32}}, color = {0, 0, 0}, thickness = 0.5,
     smooth = Smooth.None),
     Line(points = {{14, -44}, {-15, -44}}, color = {0, 0, 0}, thickness = 0.5,
     smooth = Smooth.None)}), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end JointWall;
end Tutorial;
