within FBM.Controls.Discrete;
model testTimer "should be delete"
  Timer_Off timer_Off(timing=600)
    annotation (Placement(transformation(extent={{-8,-10},{12,10}})));
  Modelica.Blocks.Sources.Pulse pulse(period=1200, startTime=200)
    annotation (Placement(transformation(extent={{-82,-10},{-62,10}})));
  Modelica.Blocks.Interaction.Show.RealValue realValue
    annotation (Placement(transformation(extent={{28,-10},{48,10}})));
equation
  connect(pulse.y, timer_Off.u)
    annotation (Line(points={{-61,0},{-8,0}}, color={0,0,127}));
  connect(timer_Off.y, realValue.numberPort)
    annotation (Line(points={{13,0},{26.5,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end testTimer;
