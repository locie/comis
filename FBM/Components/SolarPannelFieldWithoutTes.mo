within FBM.Components;
model SolarPannelFieldWithoutTes
  "Generic implementation of solar pannel field, look information page for more details"
  extends Buildings.Fluid.Interfaces.PartialTwoPortInterface(m_flow_nominal=4);
  extends FBM.Components.BaseClasses.SolarParameter;
  replaceable package MediumPrim =
      FBM.Media.Glycol20Water80 "Medium 1 in the component"
      annotation (choicesAllMatching = true,Dialog(enable = source == sou.SolarPannelField));
  Buildings.Fluid.SolarCollectors.EN12975[ nPar] solCol(
      redeclare each package Medium = MediumPrim,
    each lat=lat,
    each azi=azi,
    each til=til,
    each rho=rho,
    each nPanels=nSer,
    each per=perColector,
    p_start=300000)
    annotation (Placement(transformation(extent={{-36,30},{-16,50}})));
  Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
        transformation(extent={{-98,74},{-58,114}}), iconTransformation(extent={{-86,72},
            {-66,92}})));
  FBM.ElementaryBlocs.PumpSupply_m_flow pumpSupply(
    measurePower=false,
    m_flow_nominal=mPrim_flow_nominal,
    redeclare package Medium = MediumPrim,
    KvReturn=(3.6*mPrim_flow_nominal)*sqrt(1000/1),
    measureReturnT=true,
    InsuPipeThickness=InsuPipeThicknessSol,
    Pipelength=PipelengthSol,
    InsuHeatCondu=InsuHeatConduSol,
    includePipes=includePipesSol,
    realInput=true,
    booleanInput=false,
    p_start=300000,
    dp=80000)
    annotation (Placement(transformation(extent={{10,44},{-10,24}})));
  Buildings.Fluid.HeatExchangers.DryEffectivenessNTU hex(m1_flow_nominal=
        mPrim_flow_nominal,
    redeclare package Medium1 = MediumPrim,
    dp1_nominal=dpHexPrim,
    dp2_nominal=dpHexSecon,
    T_a1_nominal=T_a1_nominal,
    T_a2_nominal=T_a2_nominal,
    redeclare package Medium2 = Medium,
    m2_flow_nominal=m_flow_nominal,
    configuration=Buildings.Fluid.Types.HeatExchangerConfiguration.CounterFlow,
    Q_flow_nominal=m_flow_nominal*4180*(T_a1_nominal - T_a2_nominal))
    annotation (Placement(transformation(extent={{6,-6},{-6,6}},
        rotation=90,
        origin={22,34})));
  FBM.ElementaryBlocs.PumpSupply_m_flow pumpSupply1(
    measurePower=false,
    redeclare package Medium = Medium,
    dp(displayUnit="Pa") = 6000,
    m_flow_nominal=m_flow_nominal,
    KvReturn=(3.6*m_flow_nominal)*sqrt(1000/1),
    InsuPipeThickness=InsuPipeThicknessSol,
    Pipelength=PipelengthSol,
    InsuHeatCondu=InsuHeatConduSol,
    includePipes=includePipesSol,
    p_start=300000)
             annotation (Placement(transformation(
        extent={{10.5,9.5},{-10.5,-9.5}},
        rotation=0,
        origin={63.5,34.5})));
  Controls.ControlSolar.CTRL_Solar_SecPump cTRL_Solar_SecPump
    annotation (Placement(transformation(extent={{30,8},{44,22}})));
  Controls.ControlSolar.CTRL_Solar_Prim cTRL_Solar_Prim(
    lat=lat,
    azi=azi,
    til=til,
    rho=rho,
    HOn=HOn,
    HOff=HOff)
             annotation (Placement(transformation(extent={{-80,12},{-60,32}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort if
                                                                 includePipesSol
    annotation (Placement(transformation(extent={{-10,88},{10,108}})));
  Buildings.Fluid.Storage.ExpansionVessel exp(
                 V_start=1,
    redeclare package Medium = MediumPrim,
    p=300000)                                         "Expansion tank"
    annotation (Placement(transformation(
      extent={{-4,-5},{4,5}},
      origin={4,11})));
  Modelica.Blocks.Interfaces.RealInput TBoTank
    "temperature at the bottom of the tank"
    annotation (Placement(transformation(extent={{122,-88},{82,-48}})));
  Modelica.Blocks.Interfaces.RealOutput T "useless output"
    annotation (Placement(transformation(extent={{-98,-70},{-118,-50}})));
  Modelica.Blocks.Interfaces.RealInput u annotation (Placement(transformation(
        extent={{20,-20},{-20,20}},
        rotation=90,
        origin={-40,100})));
equation
  // --- connect the last pannel branch //
  for i in 1:nPar loop
  connect(weaBus, solCol[i].weaBus) annotation (Line(
      points={{-78,94},{-78,50},{-36,50},{-36,49.6}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(solCol[i].port_b, pumpSupply.port_a2) annotation (Line(
      points={{-16,40},{-14,40},{-10,40}},
      color={0,127,255},
      smooth=Smooth.Bezier));
  connect(pumpSupply.port_b1, solCol[i].port_a) annotation (Line(
      points={{-10,28},{-50,28},{-50,40},{-36,40}},
      color={0,127,255}));
  end for;
 if includePipesSol then
    connect(pumpSupply.heatPort, heatPort) annotation (Line(
        points={{0,44},{0,98}},
        color={191,0,0},
        smooth=Smooth.Bezier));
    connect(heatPort, pumpSupply1.heatPort)
      annotation (Line(points={{0,98},{63.5,98},{63.5,44}}, color={191,0,0}));
 end if;
  connect(pumpSupply.port_b2, hex.port_a1) annotation (Line(
      points={{10,40},{18.4,40}},
      color={0,127,255},
      smooth=Smooth.Bezier));
  connect(pumpSupply.port_a1, hex.port_b1) annotation (Line(
      points={{10,28},{18.4,28}},
      color={0,127,255},
      smooth=Smooth.Bezier));
  connect(hex.port_b2, pumpSupply1.port_a2) annotation (Line(points={{25.6,40},{
          46,40},{46,40.2},{53,40.2}},color={0,127,255}));
  connect(hex.port_a2, pumpSupply1.port_b1) annotation (Line(points={{25.6,28},{
          54,28},{54,28.8},{53,28.8}},color={0,127,255}));
  connect(pumpSupply.port_a1, exp.port_a) annotation (Line(points={{10,28},{10,
          28},{10,6},{4,6}},  color={0,127,255}));
  connect(pumpSupply.Tret, cTRL_Solar_SecPump.THotSol) annotation (Line(points={{7.6,
          44.4},{7.6,48},{14,48},{14,14},{29.3,14},{29.3,15}},       color={0,0,
          127}));
  connect(weaBus, cTRL_Solar_Prim.weaBus) annotation (Line(
      points={{-78,94},{-78,94},{-85.8,94},{-85.8,26.2}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(port_a, pumpSupply1.port_a1) annotation (Line(points={{-100,0},{76,0},
          {76,28.8},{74,28.8}}, color={0,127,255}));
  connect(pumpSupply1.port_b2, port_b) annotation (Line(points={{74,40.2},{88,
          40.2},{88,0},{100,0}}, color={0,127,255}));
  connect(cTRL_Solar_SecPump.TBoTank, TBoTank) annotation (Line(points={{29.58,
          9.4},{29.58,10},{22,10},{22,-68},{102,-68}}, color={0,0,127}));
  connect(T, pumpSupply.Tret) annotation (Line(points={{-108,-60},{14,-60},{14,
          44.4},{7.6,44.4}}, color={0,0,127}));
  connect(cTRL_Solar_Prim.y1, cTRL_Solar_SecPump.P0) annotation (Line(points={{-59,
          18},{-14,18},{-14,19.62},{29.3,19.62}}, color={255,0,255}));
  connect(cTRL_Solar_Prim.y, pumpSupply.u)
    annotation (Line(points={{-59,22},{0,22},{0,23.2}}, color={0,0,127}));
  connect(cTRL_Solar_SecPump.y, pumpSupply1.u) annotation (Line(points={{44.7,
          15},{64.35,15},{64.35,24.24},{63.5,24.24}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={175,175,175},
          lineThickness=1,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-94,96},{94,96},{94,90},{-92,90},{-92,82},{94,82},{94,74},{-92,
              74},{-92,66}},
          color={175,175,175},
          thickness=1),
        Line(
          points={{-94,66},{94,66},{94,60},{-92,60},{-92,52},{94,52},{94,44},{-92,
              44},{-92,36}},
          color={175,175,175},
          thickness=1),
        Line(
          points={{-92,36},{96,36},{96,30},{-90,30},{-90,22},{96,22},{96,14},{-90,
              14},{-90,6}},
          color={175,175,175},
          thickness=1),
        Line(
          points={{-92,6},{96,6},{96,0},{-90,0},{-90,-8},{96,-8},{96,-16},{-90,-16},
              {-90,-24}},
          color={175,175,175},
          thickness=1),
        Line(
          points={{-90,-24},{98,-24},{98,-30},{-88,-30},{-88,-38},{98,-38},{98,-46},
              {-88,-46},{-88,-54}},
          color={175,175,175},
          thickness=1),
        Line(
          points={{-90,-54},{98,-54},{98,-60},{-88,-60},{-88,-68},{98,-68},{98,-76},
              {-88,-76},{-88,-84}},
          color={175,175,175},
          thickness=1)}),                                            Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end SolarPannelFieldWithoutTes;
