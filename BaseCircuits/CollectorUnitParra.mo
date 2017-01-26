within FBM.BaseCircuits;
model CollectorUnitParra "collector unit for parallele branch "
  parameter Integer nEle(min=2)=1
                                 "number of splitter in series";

    replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
    "Medium in the component"
    annotation (__Dymola_choicesAllMatching=true);
  //Extensions
  extends Buildings.Fluid.Interfaces.PartialFourPort(
    redeclare package Medium1 = Medium,
    redeclare package Medium2 = Medium,
    final allowFlowReversal1 = allowFlowReversal,
    final allowFlowReversal2 = allowFlowReversal);
  extends Buildings.Fluid.Interfaces.LumpedVolumeDeclarations;
  parameter Boolean allowFlowReversal=true
    "= true to allow flow reversal, false restricts to design direction (port_a -> port_b)"
    annotation(Dialog(tab="Assumptions"));
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal
    "Nominal mass flow rate"
    annotation(Dialog(group = "Nominal condition"));
  parameter Modelica.SIunits.MassFlowRate m_flow_small(min=0) = 1E-4*abs(m_flow_nominal)
    "Small mass flow rate for regularization of zero flow";
  Modelica.Fluid.Interfaces.FluidPort_a[ nEle] port_a3(
                     redeclare final package Medium = Medium,
                     m_flow(min=if allowFlowReversal1 then -Modelica.Constants.inf else 0),
                     h_outflow(start=Medium.h_default))
    "Fluid connector a1 (positive design flow direction is from port_a1 to port_b1)"
    annotation (Placement(transformation(extent={{50,-114},{70,-94}},
            rotation=0), iconTransformation(extent={{50,-114},{70,-94}})));
  Modelica.Fluid.Interfaces.FluidPort_b[nEle] port_b3(
                     redeclare final package Medium = Medium,
                     m_flow(max=if allowFlowReversal2 then +Modelica.Constants.inf else 0),
                     h_outflow(start=Medium.h_default))
    "Fluid connector b2 (positive design flow direction is from port_a2 to port_b2)"
    annotation (Placement(transformation(extent={{-50,90},{-70,110}},
                          rotation=0),
                iconTransformation(extent={{-50,90},{-70,110}})));

  Buildings.Fluid.FixedResistances.SplitterFixedResistanceDpM[nEle] spl_supply(
    redeclare package Medium = Medium,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=0)
    annotation (Placement(transformation(extent={{-70,70},{-50,50}})));
  Buildings.Fluid.FixedResistances.SplitterFixedResistanceDpM[ nEle] spl_return(
    redeclare package Medium = Medium,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=0)
    annotation (Placement(transformation(extent={{50,-50},{70,-70}})));
equation
  for i in 1:nEle-1 loop
  connect(spl_supply[i].port_2, spl_supply[i+1].port_1)
    annotation (Line(points={{-50,60},{-50,60},{-70,60}}, color={0,127,255}));
 connect(spl_return[i].port_2, spl_return[i+1].port_1)
    annotation (Line(points={{70,-60},{70,-60},{50,-60}}, color={0,127,255}));

  end for;


  for i in 1:nEle loop
    connect(port_b3[i], spl_supply[i].port_3)
    annotation (Line(points={{-60,100},{-60,85},{-60,70}}, color={0,127,255}));
    connect(spl_return[i].port_3, port_a3[i]) annotation (Line(points={{60,-50},
            {60,-104}},     color={0,127,255}));
     end for;


  connect(port_a1, spl_supply[1].port_1)
    annotation (Line(points={{-100,60},{-85,60},{-70,60}}, color={0,127,255}));
  connect(spl_supply[nEle].port_2, port_b1)
    annotation (Line(points={{-50,60},{26,60},{100,60}}, color={0,127,255}));
  connect(port_b2, spl_return[1].port_1) annotation (Line(points={{-100,-60},{-26,
          -60},{50,-60}}, color={0,127,255}));
  connect(spl_return[nEle].port_2, port_a2) annotation (Line(points={{70,-60},{100,
          -60}},           color={0,127,255}));

    annotation (Line(points={{60,-50},{60,-50},{60,-104}}, color={0,127,255}));
         annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));

end CollectorUnitParra;
