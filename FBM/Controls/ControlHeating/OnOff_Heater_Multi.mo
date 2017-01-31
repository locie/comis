within FBM.Controls.ControlHeating;
block OnOff_Heater_Multi
parameter Real bandwidth(start=0.1) "Bandwidth around reference signal";
public
  Modelica.Blocks.Logical.OnOffController onOffController(bandwidth=bandwidth)
    annotation (Placement(transformation(extent={{-44,-10},{-24,10}})));
  Modelica.Blocks.Math.BooleanToReal booleanToReal1
    annotation (Placement(transformation(extent={{-16,-10},{4,10}})));
public
  Modelica.Blocks.Interfaces.RealInput
                              reference
    "Connector of Real input signal used as reference signal" annotation (
      Placement(transformation(extent={{-130,90},{-90,50}})));
  Modelica.Blocks.Interfaces.RealInput
                              u
    "Connector of Real input signal used as measurement signal" annotation (
      Placement(transformation(extent={{-130,-30},{-90,-70}})));
  Modelica.Blocks.Interfaces.RealOutput y
    annotation (Placement(transformation(extent={{98,-10},{118,10}})));
equation
  connect(onOffController.y,booleanToReal1. u)
    annotation (Line(points={{-23,0},{-18,0}},     color={255,0,255}));
  connect(reference, onOffController.reference) annotation (Line(points={{-110,
          70},{-78,70},{-78,6},{-46,6}}, color={0,0,127}));
  connect(u, onOffController.u) annotation (Line(points={{-110,-50},{-78,-50},{
          -78,-6},{-46,-6}}, color={0,0,127}));
  connect(booleanToReal1.y, y)
    annotation (Line(points={{5,0},{108,0},{108,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end OnOff_Heater_Multi;
