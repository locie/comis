within FBM.Controls.ControlHeating.Examples;
model RunningMeanTemperatureEN15251
  extends Modelica.Icons.Example;
  FBM.Controls.ControlHeating.RunningMeanTemperatureEN15251 runningMeanTemperature
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
        "modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos")
    annotation (Placement(transformation(extent={{-90,72},{-70,92}})));
equation
  connect(weaDat.weaBus, runningMeanTemperature.weaBus) annotation (
      Line(
      points={{-70,82},{-42,82},{-42,5},{-14.2,5}},
      color={255,204,51},
      thickness=0.5));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}})),
    experiment(StopTime=1e+007, __Dymola_NumberOfIntervals=50000),
    __Dymola_experimentSetupOutput,
    __Dymola_Commands(file=
          "modelica://FBM/Resources/Scripts/Dymola/Controls/ControlHeating/Examples/RunningMeanTemperatureEN15251.mos"
        "Simulate and Plot"));
end RunningMeanTemperatureEN15251;
