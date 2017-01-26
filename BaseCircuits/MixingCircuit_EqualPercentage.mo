within FBM.BaseCircuits;
model MixingCircuit_EqualPercentage
  "Mixing circuit with equal percentage three way valve"
  extends Interfaces.PartialMixingCircuit(redeclare
      Buildings.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear partialThreeWayValve(final R=R,
        final delta0=delta0));
  parameter Real R=50 "Rangeability, R=50...100 typically";
  parameter Real delta0=0.01
    "Range of significant deviation from equal percentage law";
equation
  connect(y, partialThreeWayValve.y)
    annotation (Line(points={{0,104},{0,72},{0,72}}, color={0,0,127}));
end MixingCircuit_EqualPercentage;
