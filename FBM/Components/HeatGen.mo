within FBM.Components;
model HeatGen
    extends Buildings.Fluid.Interfaces.PartialTwoPortInterface;
parameter .FBM.HeatingDHWsystems.Types.HeatSource GeneratorType=
         FBM.HeatingDHWsystems.Types.HeatSource.CondensingBoiler "Type of generator";
protected
         parameter Boolean with_Boiler = GeneratorType == FBM.HeatingDHWsystems.Types.HeatSource.CondensingBoiler or
                                         GeneratorType == FBM.HeatingDHWsystems.Types.HeatSource.GasBoiler or
                                         GeneratorType == FBM.HeatingDHWsystems.Types.HeatSource.WoodBoiler annotation(Evaluate=true, HideResult=true);
         parameter Boolean with_Conden = GeneratorType == FBM.HeatingDHWsystems.Types.HeatSource.CondensingBoiler annotation(Evaluate=true, HideResult=true);
         parameter Boolean with_Gas = GeneratorType == FBM.HeatingDHWsystems.Types.HeatSource.GasBoiler annotation(Evaluate=true, HideResult=true);
         parameter Boolean with_Wood = GeneratorType == FBM.HeatingDHWsystems.Types.HeatSource.WoodBoiler annotation(Evaluate=true, HideResult=true);
         parameter Buildings.Fluid.Data.Fuels.Generic fue = Buildings.Fluid.Data.Fuels.NaturalGasHigherHeatingValue() "Fuel type";
         parameter Boolean with_Solar = GeneratorType == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField;
         parameter Modelica.SIunits.Temperature T_nominal = 363.15
                                                                  "Temperature used to compute nominal efficiency (only used if efficiency curve depends on temperature)";
         parameter Modelica.SIunits.Temperature TBoiRet_min = 273.15+60 "Boiler minimum return water temperature";
         parameter Buildings.Fluid.SolarCollectors.Data.GenericSolarCollector perPar = per "Performance data for solar pannel";
         parameter Real eta_nominal(fixed=false) "Boiler efficiency at nominal condition";
         parameter Real aQuaLin[6] = if size(a, 1) == 6 then a else fill(0, 6)
  "Auxiliary variable for efficiency curve because quadraticLinear requires exactly 6 elements";
  // --- parameters for boiler (need to be filled only if the heat source is a boiler)
public
  parameter Real a[:] = {0.9} "Coefficients for efficiency curve" annotation(Dialog(group = "Boiler"));
   Modelica.SIunits.Efficiency eta=
    if effCur ==Buildings.Fluid.Types.EfficiencyCurves.Constant then
      a[1]
    elseif effCur ==Buildings.Fluid.Types.EfficiencyCurves.Polynomial then
      Buildings.Utilities.Math.Functions.polynomial(a=a, x=Y_Boiler)
   elseif effCur ==Buildings.Fluid.Types.EfficiencyCurves.QuadraticLinear then
      Buildings.Utilities.Math.Functions.quadraticLinear(a=aQuaLin, x1=Y_Boiler, x2=y_Boiler)
   else
      0
  "Boiler efficiency" annotation(Dialog(group = "Boiler"));
  parameter Buildings.Fluid.Types.EfficiencyCurves effCur=Buildings.Fluid.Types.EfficiencyCurves.Constant
    "Curve used to compute the efficiency" annotation(Dialog(group = "Boiler"));
  parameter Modelica.SIunits.Power QNom(min=0)=3000 "Nominal power, can be seen as the max power of the emission system per zone" annotation(Dialog(group = "Boiler"));
   // --- parameters for solar pannel field (need to be filled only if the heat source is a solar collector)
  parameter Modelica.SIunits.Angle lat(displayUnit="degree")=0.759 "Latitude" annotation(Dialog(group = "Solar"));
  parameter Modelica.SIunits.Angle azi(displayUnit="degree")=0.0
    "Surface azimuth (0 for south-facing; -90 degree for east-facing; +90 degree for west facing" annotation(Dialog(group = "Solar"));
  parameter Modelica.SIunits.Angle til(displayUnit="degree")=0.52
    "Surface tilt (0 for horizontally mounted collector)" annotation(Dialog(group = "Solar"));
  parameter Real rho=0.2 "Ground reflectance" annotation(Dialog(group = "Solar"));
  parameter Integer nPar(min=2) = 2 "Number of parallele solar collector branch" annotation(Dialog(group = "Solar"));
  parameter Integer nSer= 1  "Number of solar collector in each branch" annotation(Dialog(group = "Solar"));
  parameter Boolean includePipes=true
    "Set to true to include pipes in the basecircuit"
    annotation(Dialog(group = "Solar"));
    parameter Buildings.Fluid.SolarCollectors.Data.GenericSolarCollector per
    "Performance data"  annotation(Dialog(group = "Solar"),choicesAllMatching=true,
    Placement(transformation(extent={{60,-80},{80,-60}})));
  Buildings.Fluid.Boilers.BoilerPolynomial boi(fue=fue,
  T_nominal=T_nominal,
    m_flow_nominal=m_flow_nominal,
    redeclare package Medium = Medium,
    a=a,
    effCur=effCur,
    Q_flow_nominal=QNom,
    dp_nominal=10000) if                    with_Boiler
    annotation (Placement(transformation(extent={{10,50},{-10,70}})));
  SolarPannelFieldWithTes solarPannelField(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    perColector=perPar,
    redeclare package MediumPrim = Medium,
    dpHexPrim(displayUnit="Pa") = 15000,
    dpHexSecon(displayUnit="Pa") = 15000,
    lat=lat,
    azi=azi,
    til=til,
    rho=rho,
    nPar=nPar,
    nSer=nSer,
    includePipes=includePipes) if         with_Solar
    annotation (Placement(transformation(extent={{-24,-60},{24,-10}})));
  Buildings.BoundaryConditions.WeatherData.Bus weaBus if with_Solar annotation (Placement(
        transformation(extent={{-58,-34},{-18,6}}), iconTransformation(extent={{
            -182,36},{-162,56}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a1
    annotation (Placement(transformation(extent={{-10,90},{10,110}})));
  Buildings.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear valBoi(
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    l={0.01,0.01},
    dpValve_nominal=6000,
    m_flow_nominal=m_flow_nominal,
    redeclare package Medium = Medium) if
                                      with_Wood
    "Three-way valve for boiler (used only for wood boiler)"
                        annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={24,22})));
  Buildings.Fluid.FixedResistances.Junction spl(
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    dp_nominal={0,0,-200},
    m_flow_nominal={m_flow_nominal,-m_flow_nominal,-m_flow_nominal},
    redeclare package Medium = Medium) if                               with_Wood
    "Splitter for the return temperature loop control (used only for wood boiler)"
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-22,22})));
  Modelica.Blocks.Sources.Constant TSetBoiRet(k=TBoiRet_min) if with_Wood
    "Temperature setpoint for boiler return"
    annotation (Placement(transformation(extent={{12,82},{22,92}})));
  Buildings.Controls.Continuous.LimPID conPIDBoi(
    Td=1,
    Ti=120,
    reverseAction=true,
    controllerType=Modelica.Blocks.Types.SimpleController.P,
    k=1) if  with_Wood              "Controller for valve in boiler loop"
    annotation (Placement(transformation(extent={{36,82},{46,92}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort temRet(redeclare package Medium =
        Medium, m_flow_nominal=m_flow_nominal) if  with_Wood   "Return water temperature"
                                          annotation (Placement(transformation(
        extent={{5,-6},{-5,6}},
        rotation=270,
        origin={24,45})));
  Buildings.Fluid.Movers.FlowControlled_m_flow pumBoi(
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    nominalValuesDefineDefaultPressureCurve=true) if
                                      with_Wood     "Pump for boiler"
                        annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-22,48})));
  Modelica.Blocks.Interfaces.RealInput Y_Boiler if
                                            with_Boiler
    "Control signal for heat source"
    annotation (Placement(transformation(extent={{126,40},{86,80}})));
  Modelica.Blocks.Interfaces.RealOutput y_Boiler if
                                             with_Boiler
    "Output for control command"
    annotation (Placement(transformation(extent={{-98,58},{-118,78}})));
  Modelica.Blocks.Interfaces.RealInput TBotTank if
                                            with_Solar
    "Bottom Tank Temperature for control command"
    annotation (Placement(transformation(extent={{126,-70},{86,-30}})));
initial equation
    if (with_Conden) then
    fue = Buildings.Fluid.Data.Fuels.NaturalGasHigherHeatingValue();
  elseif (with_Gas) then
    fue = Buildings.Fluid.Data.Fuels.NaturalGasLowerHeatingValue();
  else
    fue = Buildings.Fluid.Data.Fuels.WoodAirDriedLowerHeatingValue();
    T_nominal= 273.15 + 80;
  end if;
  if  effCur == Buildings.Fluid.Types.EfficiencyCurves.QuadraticLinear then
    assert(size(a, 1) == 6,
    "The boiler has the efficiency curve set to 'Buildings.Fluid.Types.EfficiencyCurves.QuadraticLinear',
    and hence the parameter 'a' must have exactly 6 elements.
    However, only " + String(size(a, 1)) + " elements were provided.");
  end if;
  if effCur ==Buildings.Fluid.Types.EfficiencyCurves.Constant then
    eta_nominal = a[1];
  elseif effCur ==Buildings.Fluid.Types.EfficiencyCurves.Polynomial then
    eta_nominal = Buildings.Utilities.Math.Functions.polynomial(
                                                          a=a, x=1);
  elseif effCur ==Buildings.Fluid.Types.EfficiencyCurves.QuadraticLinear then
    // For this efficiency curve, a must have 6 elements.
    eta_nominal = Buildings.Utilities.Math.Functions.quadraticLinear(
                                                               a=aQuaLin, x1=1, x2=T_nominal);
  else
     eta_nominal = 999;
  end if;
equation
  if with_Solar then
  connect(weaBus, solarPannelField.weaBus) annotation (Line(
      points={{-38,-14},{-18.24,-14},{-18.24,-14.5}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.Bezier), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(port_a, solarPannelField.port_a) annotation (Line(
      points={{-100,0},{-64,0},{-64,-35},{-24,-35}},
      color={0,127,255},
      smooth=Smooth.Bezier));
  connect(solarPannelField.port_b, port_b) annotation (Line(
      points={{24,-35},{62,-35},{62,0},{100,0}},
      color={0,127,255},
      smooth=Smooth.Bezier));
    connect(port_a1, solarPannelField.heatPort) annotation (Line(
        points={{0,100},{0,-10.5}},
        color={191,0,0},
        smooth=Smooth.Bezier));
  connect(solarPannelField.TbotTank, TBotTank) annotation (Line(
      points={{25.44,-50},{106,-50}},
      color={0,0,127},
      smooth=Smooth.Bezier));
  elseif with_Boiler then
  connect(port_a1, boi.heatPort) annotation (Line(
      points={{0,100},{0,67.2}},
      color={191,0,0},
      smooth=Smooth.Bezier));
    connect(boi.y, Y_Boiler) annotation (Line(
        points={{12,68},{50,68},{50,60},{106,60}},
        color={0,0,127},
        smooth=Smooth.Bezier));
    connect(y_Boiler, boi.T) annotation (Line(
        points={{-108,68},{-60,68},{-11,68}},
        color={0,0,127},
        smooth=Smooth.Bezier));
  end if;
if with_Wood then
  connect(conPIDBoi.y,valBoi. y) annotation (Line(
      points={{46.5,87},{92,87},{92,22},{36,22}},
      color={0,0,127},
      smooth=Smooth.Bezier));
    connect(spl.port_3, valBoi.port_3) annotation (Line(
        points={{-12,22},{-12,22},{14,22}},
        color={0,127,255},
        smooth=Smooth.Bezier));
    connect(port_a, spl.port_2) annotation (Line(
        points={{-100,0},{-23,0},{-23,12},{-22,12}},
        color={0,127,255},
        smooth=Smooth.Bezier));
  connect(valBoi.port_1, port_b) annotation (Line(
      points={{24,12},{24,12},{24,0},{100,0}},
      color={0,127,255},
      smooth=Smooth.Bezier));
  connect(TSetBoiRet.y,conPIDBoi. u_s)
    annotation (Line(points={{22.5,87},{26,87},{26,88},{32,88},{32,87},{35,87}},
                                                     color={0,0,127},
        smooth=Smooth.Bezier));
  connect(valBoi.port_2, temRet.port_a)
    annotation (Line(points={{24,32},{24,38},{24,40}}, color={0,127,255}));
  connect(boi.port_a, temRet.port_b) annotation (Line(
      points={{10,60},{24,60},{24,50}},
      color={0,127,255},
      smooth=Smooth.Bezier));
  connect(temRet.T, conPIDBoi.u_m) annotation (Line(
      points={{30.6,45},{42.3,45},{42.3,81},{41,81}},
      color={0,0,127},
      smooth=Smooth.Bezier));
  connect(pumBoi.port_b, spl.port_1) annotation (Line(
        points={{-22,38},{-22,35},{-22,32}},
        color={0,127,255},
        smooth=Smooth.Bezier));
  connect(pumBoi.port_a, boi.port_b) annotation (Line(
      points={{-22,58},{-22,58},{-22,60},{-10,60}},
      color={0,127,255},
      smooth=Smooth.Bezier));
  connect(Y_Boiler, pumBoi.m_flow_in) annotation (Line(
      points={{106,60},{52,60},{52,68},{-34,68},{-34,48.2}},
      color={0,0,127},
      smooth=Smooth.Bezier));
elseif with_Conden or with_Gas then
    connect(boi.port_b, port_a) annotation (Line(
      points={{-10,60},{-56,60},{-56,0},{-100,0}},
      color={0,127,255},
      smooth=Smooth.Bezier));
  connect(boi.port_a, port_b) annotation (Line(
      points={{10,60},{54,60},{54,0},{100,0}},
      color={0,127,255},
      smooth=Smooth.Bezier));
end if;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end HeatGen;
