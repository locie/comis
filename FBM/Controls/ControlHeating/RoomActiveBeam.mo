within FBM.Controls.ControlHeating;
block RoomActiveBeam "Controller for an Active Beam"
  extends Modelica.Blocks.Icons.Block;
  import Buildings.Examples.VAVReheat.Controls.OperationModes;

  Buildings.Controls.Continuous.LimPID conHea(
    yMax=1,
    xi_start=0.1,
    initType=Modelica.Blocks.Types.InitPID.InitialState,
    Td=60,
    yMin=0,
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=0.1,
    Ti=120) "Controller for heating"
    annotation (Placement(transformation(extent={{-18,30},{2,50}})));
  Buildings.Controls.Continuous.LimPID conCoo(
    yMax=1,
    reverseAction=true,
    Td=60,
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=0.1,
    Ti=120) "Controller for cooling (acts on damper)"
    annotation (Placement(transformation(extent={{-20,-50},{0,-30}})));
  Buildings.Examples.VAVReheat.Controls.ControlBus controlBus
    annotation (Placement(transformation(extent={{-80,64},{-60,84}})));
  Modelica.Blocks.Interfaces.RealInput TRoo(final quantity="ThermodynamicTemperature",
                                          final unit = "K", displayUnit = "degC", min=0)
    "Measured room temperature"
    annotation (Placement(transformation(extent={{-140,20},{-100,60}})));
  Modelica.Blocks.Interfaces.RealOutput yHeat "Signal for heating mode"
    annotation (Placement(transformation(extent={{100,30},{120,50}})));
  Modelica.Blocks.Routing.IntegerPassThrough mode
    annotation (Placement(transformation(extent={{-20,78},{0,98}})));
  Modelica.Blocks.Sources.RealExpression CoolingSelector(y=if (mode.y ==
        Integer(OperationModes.occupied) or mode.y == Integer(OperationModes.unoccupiedPreCool)
         or mode.y == Integer(OperationModes.safety)) then 1 else 0)
    "Supply air temperature setpoint for cooling"
    annotation (Placement(transformation(extent={{-20,-94},{0,-74}})));
  Modelica.Blocks.Interfaces.RealOutput yCoo "Signal for cooling mode"
    annotation (Placement(transformation(extent={{100,-50},{120,-30}})));
  Buildings.Utilities.Math.SmoothMin smoothMin(deltaX=0.1)
    annotation (Placement(transformation(extent={{24,-56},{44,-36}})));
  Modelica.Blocks.Sources.RealExpression HeatingSelector(y=if (mode.y ==
        Integer(OperationModes.occupied) or mode.y == Integer(OperationModes.unoccupiedPreCool)
         or mode.y == Integer(OperationModes.safety)) then 0 else 1)
    "Supply air temperature setpoint for heating"
    annotation (Placement(transformation(extent={{-20,56},{0,76}})));
  Buildings.Utilities.Math.SmoothMin smoothMin1(
                                               deltaX=0.1)
    annotation (Placement(transformation(extent={{22,46},{42,66}})));
equation
  connect(controlBus.TRooSetHea, conHea.u_s) annotation (Line(
      points={{-70,74},{-70,40},{-20,40}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(controlBus.TRooSetCoo, conCoo.u_s) annotation (Line(
      points={{-70,74},{-70,-40},{-22,-40}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(conHea.u_m, TRoo) annotation (Line(
      points={{-8,28},{-8,24},{-90,24},{-90,40},{-120,40}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(conCoo.u_m, TRoo) annotation (Line(
      points={{-10,-52},{-90,-52},{-90,40},{-120,40}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(controlBus.controlMode, mode.u) annotation (Line(
      points={{-70,74},{-46,74},{-46,88},{-22,88}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(conCoo.y, smoothMin.u1)
    annotation (Line(points={{1,-40},{11.5,-40},{22,-40}}, color={0,0,127}));
  connect(CoolingSelector.y, smoothMin.u2) annotation (Line(points={{1,-84},{12,
          -84},{12,-52},{22,-52}}, color={0,0,127}));
  connect(smoothMin.y, yCoo) annotation (Line(points={{45,-46},{74,-46},{74,-40},
          {110,-40}}, color={0,0,127}));
  connect(HeatingSelector.y, smoothMin1.u1) annotation (Line(points={{1,66},{10,
          66},{10,62},{20,62}}, color={0,0,127}));
  connect(conHea.y, smoothMin1.u2) annotation (Line(points={{3,40},{10,40},{10,
          50},{20,50}}, color={0,0,127}));
  connect(smoothMin1.y, yHeat) annotation (Line(points={{43,56},{72,56},{72,40},
          {110,40}}, color={0,0,127}));
  annotation ( Icon(graphics={
        Text(
          extent={{-92,48},{-44,24}},
          lineColor={0,0,127},
          textString="TRoo"),
        Text(
          extent={{-92,-30},{-44,-54}},
          lineColor={0,0,127},
          textString="TSup"),
        Text(
          extent={{42,52},{90,28}},
          lineColor={0,0,127},
          textString="yHea"),
        Text(
          extent={{46,-36},{94,-60}},
          lineColor={0,0,127},
          textString="yCoo")}));
end RoomActiveBeam;
