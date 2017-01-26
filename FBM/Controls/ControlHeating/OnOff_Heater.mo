within FBM.Controls.ControlHeating;
block OnOff_Heater
parameter Real bandwidth(start=0.1) "Bandwidth around reference signal";
public
  Modelica.Blocks.Interfaces.RealInput
                              reference
    "Connector of Real input signal used as reference signal" annotation (
      Placement(transformation(extent={{-128,20},{-88,-20}})));
  Modelica.Blocks.Interfaces.RealInput
            u_m "Connector of measurement input signal" annotation (Placement(
        transformation(
        origin={10,-110},
        extent={{20,-20},{-20,20}},
        rotation=270)));
  Modelica.Blocks.Interfaces.RealOutput y
    annotation (Placement(transformation(extent={{98,-10},{118,10}})));
  Buildings.Controls.Continuous.LimPID conBOI(
    Td=30,
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    Ti=1200,
    k=0.001,
    initType=Modelica.Blocks.Types.InitPID.InitialState)
    annotation (Placement(transformation(extent={{14,-10},{34,10}})));
equation
  connect(u_m, conBOI.u_m)
    annotation (Line(points={{10,-110},{24,-110},{24,-12}}, color={0,0,127}));
  connect(conBOI.y, y)
    annotation (Line(points={{35,0},{108,0}}, color={0,0,127}));
  connect(reference, conBOI.u_s)
    annotation (Line(points={{-108,0},{-48,0},{12,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end OnOff_Heater;
