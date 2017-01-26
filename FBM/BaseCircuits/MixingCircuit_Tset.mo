within FBM.BaseCircuits;
model MixingCircuit_Tset
  "Mixing circuit with outlet temperature setpoint - assuming ideal mixing without pressure simulation"
  //Extensions
  extends Interfaces.PartialMixingCircuit(redeclare
      Buildings.Fluid.Actuators.Valves.ThreeWayLinear partialThreeWayValve);
  //Parameters
  parameter Modelica.SIunits.Mass mMix "Internal mass of the 3 way valve";
  Modelica.Blocks.Interfaces.RealInput TMixedSet
    "Setpoint for the supply temperature"                                              annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={0,104}),   iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={0,100})));
  Buildings.Controls.Continuous.LimPID
                             conVal(
    yMax=1,
    yMin=0,
    initType=Modelica.Blocks.Types.InitPID.InitialState,
    xi_start=1,
    Td=60,
    k=0.1,
    Ti=120,
    controllerType=Modelica.Blocks.Types.SimpleController.PI)
    "Controller for pump"
    annotation (Placement(transformation(extent={{34,76},{14,96}})));
equation
  connect(TMixedSet, conVal.u_s) annotation (Line(points={{0,104},{38,104},{38,86},
          {36,86}}, color={0,0,127}));
  connect(senTemSup.T, conVal.u_m)
    annotation (Line(points={{70,71},{24,71},{24,74}}, color={0,0,127}));
  connect(conVal.y, partialThreeWayValve.y)
    annotation (Line(points={{13,86},{6,86},{6,72},{0,72}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}})),           Icon(coordinateSystem(
          preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
        Polygon(
          points={{-20,70},{-20,50},{0,60},{-20,70}},
          lineColor={0,0,127},
          smooth=Smooth.None,
          fillColor={0,128,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{20,70},{20,50},{0,60},{20,70}},
          lineColor={0,0,127},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(
          points={{0,40},{0,60}},
          color={0,0,127},
          smooth=Smooth.None),
        Line(
          points={{0,100},{-6,80},{0,60}},
          color={0,255,128},
          smooth=Smooth.None),
        Polygon(
          points={{-10,10},{-10,-10},{10,0},{-10,10}},
          lineColor={0,0,127},
          smooth=Smooth.None,
          fillColor={0,128,255},
          fillPattern=FillPattern.Solid,
          origin={0,50},
          rotation=90),
        Line(
          points={{0,40},{0,0},{20,-40},{60,-60}},
          color={0,127,255},
          pattern=LinePattern.Dash)}));
end MixingCircuit_Tset;
