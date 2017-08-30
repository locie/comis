within FBM.Controls.ControlHeating;
block RoomActiveBeam2 "Controller for an Active Beam"

  extends Modelica.Blocks.Icons.Block;
  import Buildings.Examples.VAVReheat.Controls.OperationModes;
  parameter Real kPDamHea = 0.5
    "Proportional gain for VAV damper in heating mode";
  Buildings.Controls.Continuous.LimPID conHea(
    xi_start=0.1,
    initType=Modelica.Blocks.Types.InitPID.InitialState,
    Td=60,
    yMin=0,
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=0.1,
    Ti=120,
    yMax=0.99)
            "Controller for heating"
    annotation (Placement(transformation(extent={{-20,50},{0,70}})));
  Buildings.Controls.Continuous.LimPID conCoo(
    reverseAction=true,
    Td=60,
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=0.1,
    Ti=120,
    yMax=0.99)
            "Controller for cooling (acts on damper)"
    annotation (Placement(transformation(extent={{-20,-60},{0,-40}})));
  Buildings.Examples.VAVReheat.Controls.ControlBus controlBus
    annotation (Placement(transformation(extent={{-80,64},{-60,84}})));
  Modelica.Blocks.Interfaces.RealInput TRoo(final quantity="ThermodynamicTemperature",
                                          final unit = "K", displayUnit = "degC", min=0)
    "Measured room temperature"
    annotation (Placement(transformation(extent={{-140,20},{-100,60}})));
  Modelica.Blocks.Interfaces.RealOutput yHea "Signal for heating coil valve"
    annotation (Placement(transformation(extent={{100,30},{120,50}})));
  Modelica.Blocks.Interfaces.RealOutput yDam "Signal for VAV damper"
    annotation (Placement(transformation(extent={{100,-60},{120,-40}})));
  Modelica.Blocks.Interfaces.RealInput TSup(displayUnit="degC")
    "Measured supply air temperature after heating coil"
    annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));
  Buildings.Utilities.Math.SmoothMax smoothMax2(deltaX=0.1)
    annotation (Placement(transformation(extent={{12,-14},{32,6}})));
  Modelica.Blocks.Sources.Constant one(k=1)
    annotation (Placement(transformation(extent={{-72,-76},{-52,-56}})));
  Modelica.Blocks.Math.Add3 yDamHea(k2=kPDamHea, k3=-kPDamHea)
    "Outputs (unlimited) damper signal for heating."
    annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
  Modelica.Blocks.Math.Add add
    annotation (Placement(transformation(extent={{40,-40},{60,-20}})));
  Modelica.Blocks.Sources.Constant zero(k=0.1)
    annotation (Placement(transformation(extent={{-20,10},{0,30}})));
  Buildings.Utilities.Math.SmoothMin smoothMin(deltaX=0.1)
    annotation (Placement(transformation(extent={{76,-60},{96,-40}})));
  Modelica.Blocks.Sources.RealExpression CoolingSelector(y=if (mode.y ==
        Integer(OperationModes.occupied) or mode.y == Integer(OperationModes.unoccupiedPreCool)
         or mode.y == Integer(OperationModes.safety)) then 1 else 0)
    "Supply air temperature setpoint for cooling"
    annotation (Placement(transformation(extent={{2,-100},{22,-80}})));
  Modelica.Blocks.Interfaces.RealOutput yCoo "Signal for cooling mode"
    annotation (Placement(transformation(extent={{100,-90},{120,-70}})));
  Modelica.Blocks.Sources.RealExpression HeatingSelector(y=if (mode.y ==
        Integer(OperationModes.occupied) or mode.y == Integer(OperationModes.unoccupiedPreCool)
         or mode.y == Integer(OperationModes.safety)) then 0 else 1)
    "Supply air temperature setpoint for heating"
    annotation (Placement(transformation(extent={{4,78},{24,98}})));
  Modelica.Blocks.Routing.IntegerPassThrough mode
    annotation (Placement(transformation(extent={{-44,70},{-24,90}})));
  Modelica.Blocks.Math.Min min
    annotation (Placement(transformation(extent={{42,66},{62,86}})));
  Modelica.Blocks.Math.Min min1
    annotation (Placement(transformation(extent={{60,-88},{80,-68}})));
equation
  connect(controlBus.TRooSetHea, conHea.u_s) annotation (Line(
      points={{-70,74},{-70,60},{-22,60}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(controlBus.TRooSetCoo, conCoo.u_s) annotation (Line(
      points={{-70,74},{-70,-50},{-22,-50}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(conHea.u_m, TRoo) annotation (Line(
      points={{-10,48},{-10,40},{-120,40}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(conCoo.u_m, TRoo) annotation (Line(
      points={{-10,-62},{-10,-80},{-80,-80},{-80,40},{-120,40}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TRoo, yDamHea.u3) annotation (Line(
      points={{-120,40},{-80,40},{-80,-18},{-22,-18}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(yDamHea.u2, TSup) annotation (Line(
      points={{-22,-10},{-90,-10},{-90,-40},{-120,-40}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(one.y, yDamHea.u1) annotation (Line(
      points={{-51,-66},{-32,-66},{-32,-2},{-22,-2}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(conCoo.y, add.u2) annotation (Line(
      points={{1,-50},{28,-50},{28,-36},{38,-36}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(yDamHea.y, smoothMax2.u2) annotation (Line(
      points={{1,-10},{10,-10}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(smoothMax2.y, add.u1) annotation (Line(
      points={{33,-4},{34,-4},{34,-24},{38,-24}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(zero.y, smoothMax2.u1) annotation (Line(
      points={{1,20},{4,20},{4,2},{10,2}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(smoothMin.u1, add.y) annotation (Line(
      points={{74,-44},{68,-44},{68,-30},{61,-30}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(one.y, smoothMin.u2) annotation (Line(
      points={{-51,-66},{48,-66},{48,-56},{74,-56}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(smoothMin.y, yDam) annotation (Line(
      points={{97,-50},{110,-50}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(controlBus.controlMode, mode.u) annotation (Line(
      points={{-70,74},{-54,74},{-54,80},{-46,80}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(HeatingSelector.y, min.u1) annotation (Line(points={{25,88},{32,88},{
          32,82},{40,82}}, color={0,0,127}));
  connect(conHea.y, min.u2) annotation (Line(points={{1,60},{20,60},{20,70},{40,
          70}}, color={0,0,127}));
  connect(min.y, yHea) annotation (Line(points={{63,76},{86,76},{86,40},{110,40}},
        color={0,0,127}));
  connect(min1.y, yCoo)
    annotation (Line(points={{81,-78},{110,-78},{110,-80}}, color={0,0,127}));
  connect(conCoo.y, min1.u1) annotation (Line(points={{1,-50},{28,-50},{28,-72},
          {58,-72}}, color={0,0,127}));
  connect(CoolingSelector.y, min1.u2) annotation (Line(points={{23,-90},{40,-90},
          {40,-84},{58,-84}}, color={0,0,127}));
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
end RoomActiveBeam2;
