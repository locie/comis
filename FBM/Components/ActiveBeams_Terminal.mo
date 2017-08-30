within FBM.Components;
model ActiveBeams_Terminal "Supply branch"
  replaceable package MediumA = Modelica.Media.Interfaces.PartialMedium
    "Medium model for air" annotation (choicesAllMatching=true);
  replaceable package MediumW = Modelica.Media.Interfaces.PartialMedium
    "Medium model for water" annotation (choicesAllMatching=true);
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
  FBM.Controls.ControlHeating.RoomActiveBeam2 con
    "Room temperature controller"
    annotation (Placement(transformation(extent={{0,8},{20,-12}})));
  Buildings.Examples.VAVReheat.Controls.ControlBus controlBus annotation (
      Placement(transformation(extent={{-110,-50},{-90,-30}}),
        iconTransformation(extent={{-110,-38},{-90,-18}})));
  Buildings.Fluid.Actuators.Valves.TwoWayLinear valHea(
    redeclare package Medium = MediumW,
    m_flow_nominal=m_flow_nominal*1000*15/4200/10,
    CvData=Buildings.Fluid.Types.CvTypes.OpPoint,
    from_dp=true,
    dpFixed_nominal=6000,
    dpValve_nominal=6000) "Valve at reaheat coil"
    annotation (Placement(transformation(extent={{128,20},{148,0}})));
  Modelica.Blocks.Interfaces.RealInput TRoo(unit="K", displayUnit="degC")
    "Measured room temperature"
    annotation (Placement(transformation(extent={{-140,80},{-100,120}})));
  Modelica.Fluid.Interfaces.FluidPort_a WatHea_In(redeclare package Medium =
        MediumW)
    annotation (Placement(transformation(extent={{190,70},{210,90}})));
  Modelica.Fluid.Interfaces.FluidPort_b WatHea_Out(redeclare package Medium =
        MediumW)
    annotation (Placement(transformation(extent={{190,0},{210,20}})));
  ActiveBeams.CoolingAndHeating beaCooHea(
    redeclare package MediumWat = MediumW,
    redeclare package MediumAir = MediumA,
    redeclare ActiveBeams.Data.Trox.DID632A_nozzleH_length6ft_cooling perCoo,
    redeclare ActiveBeams.Data.Trox.DID632A_nozzleH_length6ft_heating perHea)
                                          annotation (Placement(transformation(
        extent={{14,12},{-14,-12}},
        rotation=90,
        origin={56,42})));
  Buildings.Fluid.Actuators.Valves.TwoWayLinear valCoo(
    redeclare package Medium = MediumW,
    m_flow_nominal=m_flow_nominal*1000*15/4200/10,
    CvData=Buildings.Fluid.Types.CvTypes.OpPoint,
    from_dp=true,
    dpFixed_nominal=6000,
    dpValve_nominal=6000) "Valve at reaheat coil"
    annotation (Placement(transformation(extent={{98,38},{118,18}})));
  Modelica.Fluid.Interfaces.FluidPort_a WatCoo_In(redeclare package Medium =
        MediumW)
    annotation (Placement(transformation(extent={{190,50},{210,70}})));
  Modelica.Fluid.Interfaces.FluidPort_b WatCoo_Out(redeclare package Medium =
        MediumW)
    annotation (Placement(transformation(extent={{190,18},{210,38}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a AirConv
    "Connect to the convective air port of the zone"
    annotation (Placement(transformation(extent={{10,190},{30,210}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort TSup(
    redeclare package Medium = MediumA,
    m_flow_nominal=m_flow_nominal,
    initType=Modelica.Blocks.Types.Init.InitialState) "Supply air temperature"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={50,114})));
  Modelica.Blocks.Interfaces.RealOutput TempSup "Supply Temperature"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-12,210})));
  Modelica.Blocks.Interfaces.RealOutput yDam "Signal for VAV damper"
    annotation (Placement(transformation(extent={{200,-60},{220,-40}})));
equation
  connect(con.controlBus, controlBus) annotation (Line(
      points={{3,-9.4},{3,-40},{-100,-40}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(con.TRoo, TRoo) annotation (Line(
      points={{-2,-6},{-60,-6},{-60,100},{-120,100}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dash));
  connect(valHea.port_b, WatHea_Out) annotation (Line(
      points={{148,10},{148,10},{200,10}},
      color={0,127,255},
      thickness=0.5));
  connect(beaCooHea.watHea_b, valHea.port_a) annotation (Line(
      points={{56,28},{56,28},{56,26},{56,10},{128,10}},
      color={0,127,255},
      thickness=0.5));
  connect(beaCooHea.watHea_a, WatHea_In) annotation (Line(
      points={{56,56},{56,56},{56,80},{200,80}},
      color={0,127,255},
      thickness=0.5));
  connect(beaCooHea.watCoo_b, valCoo.port_a) annotation (Line(
      points={{62,28},{94,28},{98,28}},
      color={0,127,0},
      thickness=0.5));
  connect(con.yCoo, valCoo.y) annotation (Line(
      points={{21,6},{108,6},{108,16}},
      color={0,0,127},
      pattern=LinePattern.Dash));
  connect(WatCoo_In, beaCooHea.watCoo_a) annotation (Line(
      points={{200,60},{62,60},{62,56}},
      color={0,127,0},
      thickness=0.5));
  connect(valCoo.port_b, WatCoo_Out) annotation (Line(
      points={{118,28},{158,28},{200,28}},
      color={0,127,0},
      thickness=0.5));
  connect(AirConv, beaCooHea.heaPor) annotation (Line(points={{20,200},{20,200},
          {20,42},{44,42}}, color={191,0,0}));
  connect(port_b, TSup.port_b)
    annotation (Line(points={{50,200},{50,124}}, color={0,127,255}));
  connect(TSup.port_a, beaCooHea.air_b)
    annotation (Line(points={{50,104},{50,56}}, color={0,127,255}));
  connect(TempSup, TSup.T) annotation (Line(points={{-12,210},{-12,210},{-12,
          114},{39,114}}, color={0,0,127}));
  connect(con.yDam, yDam) annotation (Line(points={{21,3},{110.5,3},{110.5,-50},
          {210,-50}}, color={0,0,127}));
  connect(con.TSup, TSup.T) annotation (Line(points={{-2,2},{-12,2},{-12,114},{
          39,114}}, color={0,0,127}));
  connect(con.yHea, valHea.y)
    annotation (Line(points={{21,-6},{138,-6},{138,-2}}, color={0,0,127}));
  connect(beaCooHea.air_a, port_a)
    annotation (Line(points={{50,28},{50,28},{50,-26}}, color={0,127,255}));
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
end ActiveBeams_Terminal;
