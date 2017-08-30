within FBM.Components;
model HeatPumpW_W_Template
  extends Buildings.Fluid.Interfaces.PartialTwoPortInterface;
  parameter Modelica.SIunits.Power QNomHP(min=0)=3000 "Nominal power, can be seen as the max power of the emission system per zone" annotation(Dialog(group = "HeatPump"));
  parameter .FBM.HeatingDHWsystems.Types.HeatPump HeatPumpType=
         FBM.HeatingDHWsystems.Types.HeatPump.HP_AW "Type of generator" annotation(Dialog(group = "HeatPump"));
protected
         parameter Boolean with_WW = HeatPumpType == FBM.HeatingDHWsystems.Types.HeatPump.HP_WW annotation(Evaluate=true, HideResult=true);
         parameter Boolean with_AW = HeatPumpType == FBM.HeatingDHWsystems.Types.HeatPump.HP_AW annotation(Evaluate=true, HideResult=true);
         parameter  Real mAir = -1/cpA_default/dTEva_nominal;
public
  package MediumW = Buildings.Media.Water;
  package MediumA = Buildings.Media.Air;
    parameter Modelica.SIunits.Temperature TWatOut=15
    "Temperature of the water source";
  parameter Modelica.SIunits.TemperatureDifference dTEva_nominal=-5
    "Temperature difference evaporator inlet-outlet";
  parameter Modelica.SIunits.TemperatureDifference dTCon_nominal=10
    "Temperature difference condenser outlet-inlet";
  final parameter Modelica.SIunits.SpecificHeatCapacity cpA_default=
    MediumA.specificHeatCapacityCp(MediumA.setState_pTX(
      MediumA.p_default,
      MediumA.T_default,
      MediumA.X_default))
    "Specific heat capacity of MediumA at default medium state";
    final parameter Modelica.SIunits.SpecificHeatCapacity cpW_default=
    MediumW.specificHeatCapacityCp(MediumW.setState_pTX(
      MediumW.p_default,
      MediumW.T_default,
      MediumW.X_default))
    "Specific heat capacity of MediumW at default medium state";
  Modelica.Blocks.Interfaces.RealInput TSet
    annotation (Placement(transformation(extent={{124,32},{84,72}})));
  Buildings.Fluid.HeatPumps.Carnot_TCon heaPumAW(
    QCon_flow_nominal=QNomHP,
    dp1_nominal=6000,
    dp2_nominal=6000,
    redeclare package Medium1 = Medium,
    dTEva_nominal=dTEva_nominal,
    dTCon_nominal=dTCon_nominal,
    redeclare package Medium2 = MediumA,
    m1_flow_nominal=mAir) if        with_AW
    annotation (Placement(transformation(extent={{10,54},{-10,74}})));
  Buildings.BoundaryConditions.WeatherData.Bus weaBus if with_AW annotation (Placement(
        transformation(extent={{-100,-100},{-60,-60}}), iconTransformation(
          extent={{-88,-90},{-68,-70}})));
  Modelica.Blocks.Math.Gain mEva_flow(k=mAir) if                         with_AW
    "Evaporator mass flow rate"
    annotation (Placement(transformation(extent={{-38,84},{-48,94}})));
  Modelica.Blocks.Math.Add QEva_flow(k2=-1) if with_AW "Evaporator heat flow rate"
    annotation (Placement(transformation(extent={{-22,82},{-32,92}})));
  Buildings.Fluid.Sources.Outside out(nPorts=2, redeclare package Medium =
        MediumA)                                annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-68,36})));
  Buildings.Fluid.Movers.FlowControlled_m_flow fan(redeclare package Medium =
        MediumA, m_flow_nominal=mAir) if
                    with_AW
    annotation (Placement(transformation(extent={{-62,48},{-42,68}})));
  Buildings.Fluid.HeatPumps.Carnot_TCon heaPumWW(
    QCon_flow_nominal=QNomHP,
    dp1_nominal=6000,
    dp2_nominal=6000,
    redeclare package Medium1 = Medium,
    m1_flow_nominal=m_flow_nominal,
    dTEva_nominal=dTEva_nominal,
    dTCon_nominal=dTCon_nominal,
    redeclare package Medium2 = MediumW) if with_WW
    annotation (Placement(transformation(extent={{10,10},{-10,30}})));
  Buildings.Fluid.Sources.MassFlowSource_T sou(
    use_T_in=false,
    use_m_flow_in=true,
    redeclare package Medium = MediumW,
    nPorts=1,
    T=TWatOut) if
                with_WW
    annotation (Placement(transformation(extent={{-44,-12},{-24,8}})));
  Modelica.Blocks.Math.Gain mEva_flowW(k=-1/cpW_default/dTEva_nominal) if with_WW
    "Evaporator mass flow rate"
    annotation (Placement(transformation(extent={{-34,14},{-44,24}})));
  Modelica.Blocks.Math.Add QEva_flowW(
                                     k2=-1) if with_WW "Evaporator heat flow rate"
    annotation (Placement(transformation(extent={{-20,14},{-30,24}})));
  Buildings.Fluid.Sources.FixedBoundary sin(          redeclare package Medium =
        MediumW, nPorts=1) if with_WW
    annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        origin={30,14})));
equation
  connect(port_b, heaPumAW.port_a1) annotation (Line(
      points={{100,0},{56,0},{56,70},{10,70}},
      color={0,127,255},
      smooth=Smooth.Bezier));
  connect(heaPumAW.port_b1, port_a) annotation (Line(
      points={{-10,70},{-80,70},{-80,0},{-100,0}},
      color={0,127,255},
      smooth=Smooth.Bezier));
  connect(heaPumAW.TSet, TSet) annotation (Line(
      points={{12,73},{52,73},{52,52},{104,52}},
      color={0,0,127},
      smooth=Smooth.Bezier));
  connect(QEva_flow.y, mEva_flow.u) annotation (Line(
      points={{-32.5,87},{-32.5,89},{-37,89}},
      color={0,0,127},
      smooth=Smooth.Bezier));
  connect(heaPumAW.QCon_flow, QEva_flow.u1) annotation (Line(
      points={{-11,73},{-11,90},{-21,90}},
      color={0,0,127},
      smooth=Smooth.Bezier));
  connect(heaPumAW.QEva_flow, QEva_flow.u2) annotation (Line(
      points={{-11,55},{-11,56},{-16,56},{-16,84},{-21,84}},
      color={0,0,127},
      smooth=Smooth.Bezier));
      if (with_WW) then
    connect(QEva_flowW.u1, heaPumWW.QCon_flow) annotation (Line(
        points={{-19,22},{-14,22},{-14,29},{-11,29}},
        color={0,0,127},
        smooth=Smooth.Bezier));
    connect(QEva_flowW.u2, heaPumWW.QEva_flow) annotation (Line(
        points={{-19,16},{-14,16},{-14,11},{-11,11}},
        color={0,0,127},
        smooth=Smooth.Bezier));
  connect(QEva_flowW.y, mEva_flowW.u)
    annotation (Line(points={{-30.5,19},{-33,19}}, color={0,0,127}));
  connect(mEva_flowW.y, sou.m_flow_in) annotation (Line(
      points={{-44.5,19},{-54,19},{-54,6},{-44,6},{-44,6}},
      color={0,0,127},
      smooth=Smooth.Bezier));
    connect(heaPumWW.TSet, TSet) annotation (Line(
        points={{12,29},{52,29},{52,52},{104,52}},
        color={0,0,127},
        smooth=Smooth.Bezier));
    connect(heaPumWW.port_b1, port_a) annotation (Line(
        points={{-10,26},{-60,26},{-60,0},{-100,0}},
        color={0,127,255},
        smooth=Smooth.Bezier));
    connect(heaPumWW.port_a1, port_b) annotation (Line(
        points={{10,26},{54,26},{54,0},{100,0}},
        color={0,127,255},
        smooth=Smooth.Bezier));
    connect(sou.ports[1], heaPumWW.port_a2) annotation (Line(
        points={{-24,-2},{-18,-2},{-18,14},{-10,14}},
        color={0,127,255},
        smooth=Smooth.Bezier));
    connect(heaPumWW.port_b2, sin.ports[1]) annotation (Line(
        points={{10,14},{15,14},{20,14}},
        color={0,127,255},
        smooth=Smooth.Bezier));
      elseif with_AW then
         connect(weaBus, out.weaBus) annotation (Line(
      points={{-80,-80},{-74,-80},{-74,26},{-68.2,26}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.Bezier), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(out.ports[1], fan.port_a) annotation (Line(
      points={{-70,46},{-72,46},{-72,58},{-62,58}},
      color={0,127,255},
      smooth=Smooth.Bezier));
  connect(heaPumAW.port_a2, fan.port_b) annotation (Line(
      points={{-10,58},{-36,58},{-42,58}},
      color={0,127,255},
      smooth=Smooth.Bezier));
  connect(mEva_flow.y, fan.m_flow_in) annotation (Line(
      points={{-48.5,89},{-48.5,88.5},{-52.2,88.5},{-52.2,70}},
      color={0,0,127},
      smooth=Smooth.Bezier));
  connect(heaPumAW.port_b2, out.ports[2]) annotation (Line(
      points={{10,58},{48,58},{48,46},{-66,46}},
      color={0,127,255},
      smooth=Smooth.Bezier));
      end if;
   annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end HeatPumpW_W_Template;
