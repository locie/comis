within FBM.Interfaces.Examples;
model building
  extends FBM.Interfaces.Building(
    redeclare IDEAS.Buildings.Examples.BaseClasses.structure building,
    redeclare IDEAS.Templates.Ventilation.None ventilationSystem,
    redeclare Occupants.Standards.None occupant,
    redeclare IDEAS.Templates.Heating.None heatingSystem,
    redeclare BaseClasses.CausalInhomeFeeder inHomeGrid);
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics));
end building;
