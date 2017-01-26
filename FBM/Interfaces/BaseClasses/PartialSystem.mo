within FBM.Interfaces.BaseClasses;
partial model PartialSystem "General partial for electricity-based systems"
  // --- Sensor
  Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (
      Placement(transformation(extent={{-90,68},{-50,108}}),
        iconTransformation(extent={{-190,80},{-170,100}})));
end PartialSystem;
