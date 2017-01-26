within FBM.BaseCircuits;
model MixingCircuit_Linear "Mixing circuit with linear three way valve"
  extends Interfaces.PartialMixingCircuit(redeclare
      Buildings.Fluid.Actuators.Valves.ThreeWayLinear partialThreeWayValve);
equation
  connect(y, partialThreeWayValve.y)
    annotation (Line(points={{0,104},{0,72},{0,72}}, color={0,0,127}));
end MixingCircuit_Linear;
