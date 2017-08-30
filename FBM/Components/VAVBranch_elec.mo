within FBM.Components;
model VAVBranch_elec "Supply branch of a VAV system"
  replaceable package MediumA = Modelica.Media.Interfaces.PartialMedium
    "Medium model for air" annotation (choicesAllMatching=true);
  replaceable package MediumW = Modelica.Media.Interfaces.PartialMedium
    "Medium model for water" annotation (choicesAllMatching=true);
  Buildings.Fluid.Actuators.Dampers.VAVBoxExponential vav(
    redeclare package Medium = MediumA,
    m_flow_nominal=m_flow_nominal,
    A=0.6,
    dp_nominal(displayUnit="Pa") = 220 + 20) "VAV box for room" annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={50,104})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium =
        MediumA)
    "Fluid connector a1 (positive design flow direction is from port_a1 to port_b1)"
    annotation (Placement(transformation(extent={{40,-36},{60,-16}}),
        iconTransformation(extent={{40,-36},{60,-16}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_b(redeclare package Medium =
        MediumA)
    "Fluid connector b (positive design flow direction is from port_a1 to port_b1)"
    annotation (Placement(transformation(extent={{40,190},{60,210}}),
        iconTransformation(extent={{40,190},{60,210}})));
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal
    "Mass flow rate of this thermal zone";
  parameter Modelica.SIunits.Volume VRoo "Room volume";
  FBM.Controls.ControlHeating.RoomVAV con
    "Room temperature controller"
    annotation (Placement(transformation(extent={{-64,-6},{-44,14}})));
  Buildings.Examples.VAVReheat.Controls.ControlBus controlBus annotation (
      Placement(transformation(extent={{-110,-50},{-90,-30}}),
        iconTransformation(extent={{-110,-38},{-90,-18}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort TSup(
    redeclare package Medium = MediumA,
    m_flow_nominal=m_flow_nominal,
    initType=Modelica.Blocks.Types.Init.InitialState) "Supply air temperature"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={50,74})));
  Modelica.Blocks.Interfaces.RealOutput yDam "Signal for VAV damper"
    annotation (Placement(transformation(extent={{200,-10},{220,10}})));
  Modelica.Blocks.Interfaces.RealInput TRoo(unit="K", displayUnit="degC")
    "Measured room temperature"
    annotation (Placement(transformation(extent={{-140,80},{-100,120}})));
  Modelica.Electrical.Analog.Basic.HeatingResistor
                        heatingResistor(
    R_ref=100,
    alpha=1e-3,   i(start=0),
    T_ref=293.15)
                annotation (Placement(transformation(extent={{-10,10},{10,-10}},
          rotation=-90,
        origin={18,42})));
  Modelica.Electrical.Analog.Basic.Ground G
  annotation (Placement(transformation(extent={{-30,8},{-10,28}})));
  Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent annotation (
      Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={-20,44})));
  Buildings.Fluid.Delays.DelayFirstOrder del(
    nPorts=2,
    redeclare package Medium = MediumA,
    m_flow_nominal=m_flow_nominal,
    tau=5) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={36,52})));
equation
  connect(con.controlBus, controlBus) annotation (Line(
      points={{-61,11.4},{-62,11.4},{-62,12},{-80,12},{-80,-40},{-100,-40}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(TSup.T, con.TSup) annotation (Line(
      points={{39,74},{-76,74},{-76,0},{-66,0}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dash));
  connect(con.yDam, vav.y) annotation (Line(
      points={{-43,-1},{32,-1},{32,104},{38,104}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dash));
  connect(TSup.port_b, vav.port_a) annotation (Line(
      points={{50,84},{50,94}},
      color={0,127,255},
      smooth=Smooth.None,
      thickness=0.5));
  connect(con.yDam, yDam) annotation (Line(
      points={{-43,-1},{188,-1},{188,5.55112e-016},{210,5.55112e-016}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dash));
  connect(con.TRoo, TRoo) annotation (Line(
      points={{-66,8},{-76,8},{-76,100},{-120,100}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dash));
  connect(port_b, vav.port_b) annotation (Line(
      points={{50,200},{50,200},{50,118},{50,114}},
      color={0,127,255},
      thickness=0.5));
  connect(G.p,heatingResistor. n) annotation (Line(
      points={{-20,28},{18,28},{18,32}},
      color={0,0,255}));
  connect(G.p, signalCurrent.n)
    annotation (Line(points={{-20,28},{-20,34}}, color={0,0,255}));
  connect(signalCurrent.p, heatingResistor.p)
    annotation (Line(points={{-20,54},{18,54},{18,52}}, color={0,0,255}));
  connect(con.yHea, signalCurrent.i)
    annotation (Line(points={{-43,8},{-43,44},{-27,44}}, color={0,0,127}));
  connect(del.ports[1], TSup.port_a)
    annotation (Line(points={{46,50},{50,50},{50,64}}, color={0,127,255}));
  connect(del.ports[2], port_a) annotation (Line(points={{46,54},{46,50},{50,50},
          {50,-26}}, color={0,127,255}));
  connect(heatingResistor.heatPort, del.heatPort)
    annotation (Line(points={{28,42},{32,42},{36,42}}, color={191,0,0}));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
            -100},{200,200}})), Icon(coordinateSystem(
          preserveAspectRatio=true, extent={{-100,-100},{200,200}}), graphics={
        Rectangle(
          extent={{-100,200},{200,-100}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-160.5,-16.1286},{139.5,-20.1286}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,127,255},
          origin={31.8714,60.5},
          rotation=90),
        Rectangle(
          extent={{36,42},{66,16}},
          fillPattern=FillPattern.Solid,
          fillColor={175,175,175},
          pattern=LinePattern.None),
        Rectangle(
          extent={{72,-20},{92,-40}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={192,192,192},
          origin={20,52},
          rotation=90),
        Rectangle(
          extent={{73,-10},{93,-22}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,127,255},
          origin={34,51},
          rotation=90),
        Polygon(
          points={{36,128},{64,144},{64,142},{36,126},{36,128}},
          pattern=LinePattern.None,
          smooth=Smooth.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Polygon(
          points={{36,36},{60,36},{60,34},{36,34},{36,36}},
          pattern=LinePattern.None,
          smooth=Smooth.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Polygon(
          points={{36,24},{60,24},{60,22},{36,22},{36,24}},
          pattern=LinePattern.None,
          smooth=Smooth.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Polygon(
          points={{46,30},{60,36},{60,34},{46,28},{46,30}},
          pattern=LinePattern.None,
          smooth=Smooth.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Polygon(
          points={{46,30},{60,24},{60,22},{46,28},{46,30}},
          pattern=LinePattern.None,
          smooth=Smooth.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Text(
          extent={{-78,198},{24,156}},
          lineColor={0,0,255},
          textString="%name"),
        Text(
          extent={{126,24},{194,-20}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="yDam"),
        Text(
          extent={{144,194},{184,168}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="p_rel"),
        Text(
          extent={{144,154},{192,122}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="TRooAir")}));
end VAVBranch_elec;
