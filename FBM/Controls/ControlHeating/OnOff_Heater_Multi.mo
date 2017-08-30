within FBM.Controls.ControlHeating;
block OnOff_Heater_Multi
parameter Real bandwidth(start=0.1) "Bandwidth around reference signal";
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
  Modelica.Blocks.Continuous.LimPID conBOI(
    Td=30,
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    initType=Modelica.Blocks.Types.InitPID.InitialState,
    k=0.01,
    Ti=600,
    yMax=1,
    yMin=0)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
equation
  connect(conBOI.y, y)
    annotation (Line(points={{11,0},{108,0}},         color={0,0,127}));
  connect(reference, conBOI.u_s) annotation (Line(points={{-110,70},{-62,70},{-62,
          0},{-12,0}}, color={0,0,127}));
  connect(u, conBOI.u_m)
    annotation (Line(points={{-110,-50},{0,-50},{0,-12}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end OnOff_Heater_Multi;
