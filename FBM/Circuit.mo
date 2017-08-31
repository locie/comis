within FBM;
package Circuit "Typical kind of distribution circuit"
  extends Modelica.Icons.VariantsPackage;
  model MixingCircuit
    "Mixing circuit controlled on the supply temperature. Constant mass flow, variable temperature"
    parameter Integer nZones(min=1)    "Number of conditioned thermal zones deserved by the system" annotation(Dialog(group= "Settings parameters"));
    parameter Boolean includePipes=false "Set to true to include pipes in the basecircuit" annotation(Dialog(group= "Settings parameters"));

    parameter Modelica.SIunits.MassFlowRate[nZones] m_flow_nominal( min=0)   "Nominal mass flow rates";

    extends FBM.Controls.ControlHeating.Interfaces.ControlPara;
    replaceable package Medium=Buildings.Media.Water;

    parameter Modelica.SIunits.Temperature[nZones] T_SetPoint_Day= {294.15 for i in 1:
        nZones} "Set point temperature for room by day" annotation(Dialog(group= "Heating controler parameters"));
    parameter Modelica.SIunits.Temperature[nZones] T_SetPoint_night= {291.15 for i in 1:
        nZones}
               "Set point temperature for room by night" annotation(Dialog(group= "Heating controler parameters"));

    parameter Real KvV3V = 20;


    ElementaryBlocs.MixingCircuit_Tset[nZones] mixingCircuit_Tset(redeclare
        package Medium = Medium, m_flow_nominal=m_flow_nominal,
      each KvReturn=KvV3V,
      Ti=600,
      Td=60,
      useBalancingValve=true,
      controllerType=Modelica.Blocks.Types.SimpleController.PI)
      annotation (Placement(transformation(extent={{-38,-18},{-4,18}})));
    ElementaryBlocs.PumpSupply_m_flow[nZones] pumpSupply_m_flow(redeclare
        package
        Medium = Medium, each includePipes=includePipes,
      m_flow_nominal=m_flow_nominal,
      each KvReturn=40)
      annotation (Placement(transformation(extent={{18,-18},{52,18}})));
  protected
    replaceable Controls.ControlHeating.Ctrl_Heating ctrl_Temp(
      heatingCurve(timeFilter=timeFilter),
      TSupNom=TSupNom,
      dTSupRetNom=dTSupRetNom,
      TSupMin=TSupMin,
      minSup=minSup,
      corFac_val=corFac_val,
      TRoo_nominal=TRoo_nominal,
      dTHeaterSet=dTHeaterSet,
      dTOutHeaBal=dTOutHeaBal,
      TOut_nominal=TOut_nominal) constrainedby
      Controls.ControlHeating.Ctrl_Heating(
      heatingCurve(timeFilter=timeFilter),
      TSupNom=TSupNom,
      dTSupRetNom=dTSupRetNom) "Controller the emission set point "
      annotation (Placement(transformation(extent={{-58,30},{-38,50}})));
  public
    Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
          transformation(extent={{-102,60},{-62,100}}), iconTransformation(extent={{-82,80},
              {-62,100}})));
    Modelica.Blocks.Sources.Constant[nZones] TRooNig(k={T_SetPoint_night[i] for i in
              1:nZones})
      "Room temperature set point at night"
      annotation (Placement(transformation(extent={{-58,70},{-48,80}})));
   Modelica.Blocks.Sources.Constant[nZones] TRooSet(k={T_SetPoint_Day[i] for i in
              1:nZones})
      annotation (Placement(transformation(extent={{-58,86},{-48,96}})));
   Modelica.Blocks.Logical.Switch[nZones] swi "Switch to select set point"
      annotation (Placement(transformation(extent={{-38,74},{-26,86}})));
        Buildings.Controls.SetPoints.OccupancySchedule[ nZones]
                                         occSch "Occupancy schedule"
      annotation (Placement(transformation(extent={{-46,82},{-42,86}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal[nZones](realTrue=
          m_flow_nominal)
      annotation (Placement(transformation(extent={{18,74},{30,86}})));
    Modelica.Blocks.Logical.Hysteresis[nZones] heatingControl(each uLow=-1,
        each uHigh=1)   "onoff controller for the pumps of the emission circuits"
      annotation (Placement(transformation(extent={{0,74},{12,86}})));
    Modelica.Fluid.Interfaces.FluidPort_a[nZones] port_a2(redeclare package
        Medium = Medium)
      annotation (Placement(transformation(extent={{90,-70},{110,-50}})));
    Modelica.Fluid.Interfaces.FluidPort_b[nZones] port_b1(redeclare package
        Medium = Medium)
      annotation (Placement(transformation(extent={{90,50},{110,70}})));
    Modelica.Fluid.Interfaces.FluidPort_a port_a1(redeclare package Medium =
          Medium)
      annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
    Modelica.Fluid.Interfaces.FluidPort_b port_b2(redeclare package Medium =
          Medium)
      annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
    Modelica.Blocks.Sources.RealExpression TSet_max(y=max(swi.y))
      "maximum value of set point temperature" annotation (Placement(
          transformation(
          extent={{-9,-9},{9,9}},
          rotation=90,
          origin={-73,23})));
    Modelica.Blocks.Interfaces.RealOutput THeaterSet(final quantity="ThermodynamicTemperature",unit="K",displayUnit="degC", min=0,start=283.15)
      "Heat pump set temperature" annotation (Placement(transformation(extent={{-10,-10},
              {10,10}},
          rotation=90,
          origin={60,110}),       iconTransformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={20,110})));
    Modelica.Blocks.Interfaces.RealOutput TSupply[nZones](
      final quantity="ThermodynamicTemperature",
      unit="K",
      displayUnit="degC",
      min=0) "TSupply after mixing" annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-20,110}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-20,110})));
  protected
    Modelica.Blocks.Math.Add add[nZones](each k1=-1, each k2=+1)
      annotation (Placement(transformation(extent={{-16,74},{-4,86}})));
  public
    Buildings.Utilities.Math.Average ave(nin=nZones) if  includePipes
      "Compute average of room temperatures"
      annotation (Placement(transformation(extent={{-24,-64},{-12,-52}})));
    Buildings.HeatTransfer.Sources.PrescribedTemperature
      prescribedTemperature if includePipes annotation (Placement(transformation(
          extent={{-7,-7},{7,7}},
          rotation=90,
          origin={35,-39})));
    Modelica.Blocks.Interfaces.RealInput[nZones] TSensor(
      final quantity="ThermodynamicTemperature",
      unit="K",
      displayUnit="degC",
      min=0) "Sensor temperature" annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=270,
          origin={-40,-106})));
  equation
    connect(ctrl_Temp.weaBus, weaBus) annotation (Line(
        points={{-65.6,46.2},{-65.6,46.1},{-82,46.1},{-82,80}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(TRooNig.y,swi. u3) annotation (Line(points={{-47.5,75},{-39.2,75},{-39.2,
            75.2}}, color={0,0,127}));
    connect(occSch.occupied,swi. u2) annotation (Line(points={{-41.8,82.8},{-40,82.8},
            {-40,80},{-39.2,80}},         color={255,0,255}));
    connect(TRooSet.y,swi. u1) annotation (Line(points={{-47.5,91},{-47.5,90.5},{-39.2,
            90.5},{-39.2,84.8}}, color={0,0,127}));
    connect(heatingControl.y, booleanToReal.u) annotation (Line(points={{12.6,80},
            {12.6,80},{16.8,80}}, color={255,0,255}));

    for i in 1:nZones loop
      connect(ctrl_Temp.THeaCur, mixingCircuit_Tset[i].TMixedSet)
        annotation (Line(points={{-38,44},{-21,44},{-21,18}}, color={0,0,127}));
      connect(port_a1, mixingCircuit_Tset[i].port_a1) annotation (Line(points={{-100,
            60},{-86,60},{-86,10.8},{-38,10.8}}, color={0,127,255}));
      connect(port_b2, mixingCircuit_Tset[i].port_b2) annotation (Line(points={{-100,
            -60},{-92,-60},{-86,-60},{-86,-10.8},{-38,-10.8}}, color={0,127,255}));
            if includePipes then

          connect(pumpSupply_m_flow[i].heatPort, prescribedTemperature.port)
      annotation (Line(points={{35,-18},{35,-26},{35,-32}}, color={191,0,0}));


            end if;
    end for;

    if includePipes then
      connect(ave.y, prescribedTemperature.T) annotation (Line(points={{-11.4,-58},{
            35,-58},{35,-47.4}}, color={0,0,127}));
      connect(TSensor, ave.u) annotation (Line(points={{-40,-106},{-40,-106},{-40,-58},
            {-25.2,-58}}, color={0,0,127}));

    end if;
    connect(booleanToReal.y, pumpSupply_m_flow.u) annotation (Line(points={{30.6,80},
            {30.6,80},{35,80},{35,19.44}},
                                         color={0,0,127}));
    connect(mixingCircuit_Tset.port_b1, pumpSupply_m_flow.port_a1) annotation (
        Line(points={{-4,10.8},{8,10.8},{18,10.8}},          color={0,127,255}));
    connect(mixingCircuit_Tset.port_a2, pumpSupply_m_flow.port_b2) annotation (
        Line(points={{-4,-10.8},{8,-10.8},{18,-10.8}},           color={0,127,255}));
    connect(pumpSupply_m_flow.port_b1, port_b1) annotation (Line(points={{52,10.8},
            {76,10.8},{76,60},{100,60}}, color={0,127,255}));
    connect(pumpSupply_m_flow.port_a2, port_a2) annotation (Line(points={{52,-10.8},
            {76,-10.8},{76,-12},{76,-60},{100,-60}}, color={0,127,255}));
    connect(TSet_max.y, ctrl_Temp.TRoo_in1)
      annotation (Line(points={{-73,32.9},{-73,44},{-58,44}}, color={0,0,127}));
    connect(ctrl_Temp.THeaterSet, THeaterSet)
      annotation (Line(points={{-39,41},{60,41},{60,110}}, color={0,0,127}));
    connect(mixingCircuit_Tset.Tsup, TSupply) annotation (Line(points={{-8.08,18.72},
            {-8.08,59.36},{-20,59.36},{-20,110}}, color={0,0,127}));
    connect(add.y, heatingControl.u)
      annotation (Line(points={{-3.4,80},{-3.4,80},{-1.2,80}}, color={0,0,127}));
    connect(swi.y, add.u2) annotation (Line(points={{-25.4,80},{-22,80},{-22,76.4},
            {-17.2,76.4}}, color={0,0,127}));
    connect(TSensor, add.u1) annotation (Line(points={{-40,-106},{-20,-106},{
            -20,83.6},{-17.2,83.6}},
                           color={0,0,127}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                                     Line(
            points={{-102,60},{98,60}},
            color={255,85,85},
            thickness=0.5),      Line(
            points={{-102,-60},{98,-60}},
            color={0,127,255},
            thickness=0.5),
          Polygon(
            points={{-40,70},{-40,50},{-20,60},{-40,70}},
            lineColor={0,0,127},
            smooth=Smooth.None,
            fillColor={0,128,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{0,70},{0,50},{-20,60},{0,70}},
            lineColor={0,0,127},
            smooth=Smooth.None,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-10,10},{-10,-10},{10,0},{-10,10}},
            lineColor={0,0,127},
            smooth=Smooth.None,
            fillColor={0,128,255},
            fillPattern=FillPattern.Solid,
            origin={-20,50},
            rotation=90),
          Line(
            points={{-20,40},{-20,0},{0,-40},{40,-60}},
            color={0,127,255},
            pattern=LinePattern.Dash),
          Ellipse(extent={{40,80},{80,40}}, lineColor={0,0,127},
            fillColor={0,127,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{48,76},{48,44},{80,60},{48,76}},
            lineColor={0,0,127},
            smooth=Smooth.None,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}),                          Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end MixingCircuit;

  model DistributionCircuit
    "Distribution circuit controlled on the supply temperature. Constant mass flow, variable temperature"
    parameter Integer nZones(min=1)    "Number of conditioned thermal zones deserved by the system" annotation(Dialog(group= "Settings parameters"));
    parameter Boolean includePipes=false "Set to true to include pipes in the basecircuit" annotation(Dialog(group= "Settings parameters"));

    parameter Modelica.SIunits.MassFlowRate[nZones] m_flow_nominal( min=0)   "Nominal mass flow rates";

    extends FBM.Controls.ControlHeating.Interfaces.ControlPara;
    replaceable package Medium=Buildings.Media.Water;

    parameter Modelica.SIunits.Temperature[nZones] T_SetPoint_Day= {294.15 for i in 1:
        nZones} "Set point temperature for room by day" annotation(Dialog(group= "Heating controler parameters"));
    parameter Modelica.SIunits.Temperature[nZones] T_SetPoint_night= {291.15 for i in 1:
        nZones}
               "Set point temperature for room by night" annotation(Dialog(group= "Heating controler parameters"));

    parameter Real KvV3V = 20;

    ElementaryBlocs.MixingCircuit_Tset[nZones] mixingCircuit_Tset(redeclare
        package Medium = Medium, m_flow_nominal=m_flow_nominal,
      each KvReturn=KvV3V,
      Ti=600,
      controllerType=Modelica.Blocks.Types.SimpleController.PID,
      Td=60,
      reverseAction=true)
      annotation (Placement(transformation(extent={{-38,18},{-4,-18}})));
    ElementaryBlocs.PumpSupply_m_flow[nZones] pumpSupply_m_flow(redeclare
        package
        Medium = Medium, each includePipes=includePipes,
      m_flow_nominal=m_flow_nominal,
      each measureSupplyT=false,
      each KvReturn=40)
      annotation (Placement(transformation(extent={{18,-18},{52,18}})));
  protected
    replaceable Controls.ControlHeating.Ctrl_Heating ctrl_Temp(
      heatingCurve(timeFilter=timeFilter),
      TSupNom=TSupNom,
      dTSupRetNom=dTSupRetNom,
      TSupMin=TSupMin,
      minSup=minSup,
      corFac_val=corFac_val,
      TRoo_nominal=TRoo_nominal,
      dTHeaterSet=dTHeaterSet,
      dTOutHeaBal=dTOutHeaBal,
      TOut_nominal=TOut_nominal) constrainedby
      Controls.ControlHeating.Ctrl_Heating(
      heatingCurve(timeFilter=timeFilter),
      TSupNom=TSupNom,
      dTSupRetNom=dTSupRetNom) "Controller the emission set point "
      annotation (Placement(transformation(extent={{-58,30},{-38,50}})));
  public
    Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
          transformation(extent={{-102,60},{-62,100}}), iconTransformation(extent={{-82,80},
              {-62,100}})));
    Modelica.Blocks.Sources.Constant[nZones] TRooNig(k={T_SetPoint_night[i] for i in
              1:nZones})
      "Room temperature set point at night"
      annotation (Placement(transformation(extent={{-58,70},{-48,80}})));
   Modelica.Blocks.Sources.Constant[nZones] TRooSet(k={T_SetPoint_Day[i] for i in
              1:nZones})
      annotation (Placement(transformation(extent={{-58,86},{-48,96}})));
   Modelica.Blocks.Logical.Switch[nZones] swi "Switch to select set point"
      annotation (Placement(transformation(extent={{-38,74},{-26,86}})));
        Buildings.Controls.SetPoints.OccupancySchedule[ nZones]
                                         occSch "Occupancy schedule"
      annotation (Placement(transformation(extent={{-46,82},{-42,86}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal[nZones](realTrue=
          m_flow_nominal)
      annotation (Placement(transformation(extent={{18,74},{30,86}})));
    Modelica.Blocks.Logical.Hysteresis[nZones] heatingControl(each uLow=-1, each
        uHigh=1)        "onoff controller for the pumps of the emission circuits"
      annotation (Placement(transformation(extent={{0,74},{12,86}})));
    Modelica.Fluid.Interfaces.FluidPort_a[nZones] port_a2(redeclare package
        Medium = Medium)
      annotation (Placement(transformation(extent={{90,-70},{110,-50}})));
    Modelica.Fluid.Interfaces.FluidPort_b[nZones] port_b1(redeclare package
        Medium = Medium)
      annotation (Placement(transformation(extent={{90,50},{110,70}})));
    Modelica.Fluid.Interfaces.FluidPort_a port_a1(redeclare package Medium =
          Medium)
      annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
    Modelica.Fluid.Interfaces.FluidPort_b port_b2(redeclare package Medium =
          Medium)
      annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
    Modelica.Blocks.Sources.RealExpression TSet_max(y=max(swi.y))
      "maximum value of set point temperature" annotation (Placement(
          transformation(
          extent={{-9,-9},{9,9}},
          rotation=90,
          origin={-73,23})));
    Modelica.Blocks.Interfaces.RealOutput THeaterSet(final quantity="ThermodynamicTemperature",unit="K",displayUnit="degC", min=0,start=283.15)
      "Heat pump set temperature" annotation (Placement(transformation(extent={{-10,-10},
              {10,10}},
          rotation=90,
          origin={60,110}),       iconTransformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={20,110})));
    Modelica.Blocks.Interfaces.RealOutput TSupply[nZones](
      final quantity="ThermodynamicTemperature",
      unit="K",
      displayUnit="degC",
      min=0) "TSupply after mixing" annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-20,110}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-20,110})));
  protected
    Modelica.Blocks.Math.Add add[nZones](each k1=-1, each k2=+1)
      annotation (Placement(transformation(extent={{-16,74},{-4,86}})));
  public
    Buildings.Utilities.Math.Average ave(nin=nZones) if  includePipes
      "Compute average of room temperatures"
      annotation (Placement(transformation(extent={{-24,-64},{-12,-52}})));
    Buildings.HeatTransfer.Sources.PrescribedTemperature
      prescribedTemperature if includePipes annotation (Placement(transformation(
          extent={{-7,-7},{7,7}},
          rotation=90,
          origin={35,-39})));
    Modelica.Blocks.Interfaces.RealInput[nZones] TSensor(
      final quantity="ThermodynamicTemperature",
      unit="K",
      displayUnit="degC",
      min=0) "Sensor temperature" annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=270,
          origin={-40,-106})));
  equation
    connect(ctrl_Temp.weaBus, weaBus) annotation (Line(
        points={{-65.6,46.2},{-65.6,46.1},{-82,46.1},{-82,80}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(TRooNig.y,swi. u3) annotation (Line(points={{-47.5,75},{-39.2,75},{-39.2,
            75.2}}, color={0,0,127}));
    connect(occSch.occupied,swi. u2) annotation (Line(points={{-41.8,82.8},{-40,82.8},
            {-40,80},{-39.2,80}},         color={255,0,255}));
    connect(TRooSet.y,swi. u1) annotation (Line(points={{-47.5,91},{-47.5,90.5},{-39.2,
            90.5},{-39.2,84.8}}, color={0,0,127}));
    connect(heatingControl.y, booleanToReal.u) annotation (Line(points={{12.6,80},
            {12.6,80},{16.8,80}}, color={255,0,255}));

    for i in 1:nZones loop
      connect(ctrl_Temp.THeaCur, mixingCircuit_Tset[i].TMixedSet)
      annotation (Line(points={{-38,44},{-21,44},{-21,-18}},color={0,0,127}));
        connect(port_a1, mixingCircuit_Tset[i].port_b2) annotation (Line(points={{-100,
            60},{-94,60},{-88,60},{-88,10.8},{-38,10.8}}, color={0,127,255}));
    connect(port_b2, mixingCircuit_Tset[i].port_a1) annotation (Line(points={{-100,
            -60},{-88,-60},{-88,-10.8},{-38,-10.8}}, color={0,127,255}));

            if includePipes then

          connect(pumpSupply_m_flow[i].heatPort, prescribedTemperature.port)
      annotation (Line(points={{35,-18},{35,-26},{35,-32}}, color={191,0,0}));

            end if;
    end for;

    if includePipes then
      connect(ave.y, prescribedTemperature.T) annotation (Line(points={{-11.4,-58},{
            35,-58},{35,-47.4}}, color={0,0,127}));
      connect(TSensor, ave.u) annotation (Line(points={{-40,-106},{-40,-106},{-40,-58},
            {-25.2,-58}}, color={0,0,127}));

    end if;
    connect(booleanToReal.y, pumpSupply_m_flow.u) annotation (Line(points={{30.6,80},
            {30.6,80},{35,80},{35,19.44}},
                                         color={0,0,127}));
    connect(pumpSupply_m_flow.port_b1, port_b1) annotation (Line(points={{52,10.8},
            {76,10.8},{76,60},{100,60}}, color={0,127,255}));
    connect(pumpSupply_m_flow.port_a2, port_a2) annotation (Line(points={{52,-10.8},
            {76,-10.8},{76,-12},{76,-60},{100,-60}}, color={0,127,255}));
    connect(TSet_max.y, ctrl_Temp.TRoo_in1)
      annotation (Line(points={{-73,32.9},{-73,44},{-58,44}}, color={0,0,127}));
    connect(ctrl_Temp.THeaterSet, THeaterSet)
      annotation (Line(points={{-39,41},{60,41},{60,110}}, color={0,0,127}));
    connect(mixingCircuit_Tset.Tsup, TSupply) annotation (Line(points={{-8.08,-18.72},
            {-8.08,59.36},{-20,59.36},{-20,110}}, color={0,0,127}));
    connect(add.y, heatingControl.u)
      annotation (Line(points={{-3.4,80},{-3.4,80},{-1.2,80}}, color={0,0,127}));
    connect(swi.y, add.u2) annotation (Line(points={{-25.4,80},{-22,80},{-22,76.4},
            {-17.2,76.4}}, color={0,0,127}));
    connect(TSensor, add.u1) annotation (Line(points={{-40,-106},{-20,-106},{-20,83.6},
            {-17.2,83.6}}, color={0,0,127}));
    connect(mixingCircuit_Tset.port_b1, pumpSupply_m_flow.port_b2) annotation (
        Line(points={{-4,-10.8},{8,-10.8},{18,-10.8}}, color={0,127,255}));
    connect(mixingCircuit_Tset.port_a2, pumpSupply_m_flow.port_a1) annotation (
        Line(points={{-4,10.8},{8,10.8},{18,10.8}}, color={0,127,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                                     Line(
            points={{-102,60},{98,60}},
            color={255,85,85},
            thickness=0.5),      Line(
            points={{-102,-60},{98,-60}},
            color={0,127,255},
            thickness=0.5),
          Polygon(
            points={{-40,70},{-40,50},{-20,60},{-40,70}},
            lineColor={0,0,127},
            smooth=Smooth.None,
            fillColor={0,128,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{0,70},{0,50},{-20,60},{0,70}},
            lineColor={0,0,127},
            smooth=Smooth.None,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-10,10},{-10,-10},{10,0},{-10,10}},
            lineColor={0,0,127},
            smooth=Smooth.None,
            fillColor={0,128,255},
            fillPattern=FillPattern.Solid,
            origin={-20,50},
            rotation=90),
          Line(
            points={{-20,40},{-20,0},{0,-40},{40,-60}},
            color={0,127,255},
            pattern=LinePattern.Dash),
          Ellipse(extent={{40,80},{80,40}}, lineColor={0,0,127},
            fillColor={0,127,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{48,76},{48,44},{80,60},{48,76}},
            lineColor={0,0,127},
            smooth=Smooth.None,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}),                          Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end DistributionCircuit;

  model InjectionCircuit
    "Mixing circuit controlled on the supply temperature. Constant mass flow, variable temperature"
    parameter Integer nZones(min=1)    "Number of conditioned thermal zones deserved by the system" annotation(Dialog(group= "Settings parameters"));
    parameter Boolean includePipes=false "Set to true to include pipes in the basecircuit" annotation(Dialog(group= "Settings parameters"));

    parameter Modelica.SIunits.MassFlowRate[nZones] m_flow_nominal( min=0)   "Nominal mass flow rates";

    extends FBM.Controls.ControlHeating.Interfaces.ControlPara;
    replaceable package Medium=Buildings.Media.Water;

    parameter Modelica.SIunits.Temperature[nZones] T_SetPoint_Day= {294.15 for i in 1:
        nZones} "Set point temperature for room by day" annotation(Dialog(group= "Heating controler parameters"));
    parameter Modelica.SIunits.Temperature[nZones] T_SetPoint_night= {291.15 for i in 1:
        nZones}
               "Set point temperature for room by night" annotation(Dialog(group= "Heating controler parameters"));

    parameter Real KvV3V = 20;

    ElementaryBlocs.MixingCircuit_Tset[nZones] mixingCircuit_Tset(redeclare
        package Medium = Medium, m_flow_nominal=m_flow_nominal,
      each KvReturn=KvV3V,
      Ti=600,
      Td=60,
      useBalancingValve=true,
      controllerType=Modelica.Blocks.Types.SimpleController.PI)
      annotation (Placement(transformation(extent={{-18,-18},{16,18}})));
    ElementaryBlocs.PumpSupply_m_flow[nZones] pumpSupply_m_flow(redeclare
        package
        Medium = Medium, each includePipes=includePipes,
      m_flow_nominal=m_flow_nominal,
      each measureSupplyT=false,
      each KvReturn=40)
      annotation (Placement(transformation(extent={{38,-18},{72,18}})));
  protected
    replaceable Controls.ControlHeating.Ctrl_Heating ctrl_Temp(
      heatingCurve(timeFilter=timeFilter),
      TSupNom=TSupNom,
      dTSupRetNom=dTSupRetNom,
      TSupMin=TSupMin,
      minSup=minSup,
      corFac_val=corFac_val,
      TRoo_nominal=TRoo_nominal,
      dTHeaterSet=dTHeaterSet,
      dTOutHeaBal=dTOutHeaBal,
      TOut_nominal=TOut_nominal) constrainedby
      Controls.ControlHeating.Ctrl_Heating(
      heatingCurve(timeFilter=timeFilter),
      TSupNom=TSupNom,
      dTSupRetNom=dTSupRetNom) "Controller the emission set point "
      annotation (Placement(transformation(extent={{-58,40},{-38,60}})));
  public
    Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
          transformation(extent={{-102,60},{-62,100}}), iconTransformation(extent={{-82,80},
              {-62,100}})));
    Modelica.Blocks.Sources.Constant[nZones] TRooNig(k={T_SetPoint_night[i] for i in
              1:nZones})
      "Room temperature set point at night"
      annotation (Placement(transformation(extent={{-58,70},{-48,80}})));
   Modelica.Blocks.Sources.Constant[nZones] TRooSet(k={T_SetPoint_Day[i] for i in
              1:nZones})
      annotation (Placement(transformation(extent={{-58,86},{-48,96}})));
   Modelica.Blocks.Logical.Switch[nZones] swi "Switch to select set point"
      annotation (Placement(transformation(extent={{-38,74},{-26,86}})));
        Buildings.Controls.SetPoints.OccupancySchedule[ nZones]
                                         occSch "Occupancy schedule"
      annotation (Placement(transformation(extent={{-46,82},{-42,86}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal[nZones](realTrue=
          m_flow_nominal)
      annotation (Placement(transformation(extent={{18,74},{30,86}})));
    Modelica.Blocks.Logical.Hysteresis[nZones] heatingControl(each uLow=-1, each
        uHigh=1)        "onoff controller for the pumps of the emission circuits"
      annotation (Placement(transformation(extent={{0,74},{12,86}})));
    Modelica.Fluid.Interfaces.FluidPort_a[nZones] port_a2(redeclare package
        Medium = Medium)
      annotation (Placement(transformation(extent={{90,-70},{110,-50}})));
    Modelica.Fluid.Interfaces.FluidPort_b[nZones] port_b1(redeclare package
        Medium = Medium)
      annotation (Placement(transformation(extent={{90,50},{110,70}})));
    Modelica.Fluid.Interfaces.FluidPort_a port_a1(redeclare package Medium =
          Medium)
      annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
    Modelica.Fluid.Interfaces.FluidPort_b port_b2(redeclare package Medium =
          Medium)
      annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
    Modelica.Blocks.Sources.RealExpression TSet_max(y=max(swi.y))
      "maximum value of set point temperature" annotation (Placement(
          transformation(
          extent={{9,-9},{-9,9}},
          rotation=180,
          origin={-79,53})));
    Modelica.Blocks.Interfaces.RealOutput THeaterSet(final quantity="ThermodynamicTemperature",unit="K",displayUnit="degC", min=0,start=283.15)
      "Heat pump set temperature" annotation (Placement(transformation(extent={{-10,-10},
              {10,10}},
          rotation=90,
          origin={60,110}),       iconTransformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={20,110})));
    Modelica.Blocks.Interfaces.RealOutput TSupply[nZones](
      final quantity="ThermodynamicTemperature",
      unit="K",
      displayUnit="degC",
      min=0) "TSupply after mixing" annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-20,110}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-20,110})));
  protected
    Modelica.Blocks.Math.Add add[nZones](each k1=-1, each k2=+1)
      annotation (Placement(transformation(extent={{-16,74},{-4,86}})));
  public
    Buildings.Utilities.Math.Average ave(nin=nZones) if  includePipes
      "Compute average of room temperatures"
      annotation (Placement(transformation(extent={{-24,-64},{-12,-52}})));
    Buildings.HeatTransfer.Sources.PrescribedTemperature
      prescribedTemperature if includePipes annotation (Placement(transformation(
          extent={{-7,-7},{7,7}},
          rotation=90,
          origin={35,-39})));
    Modelica.Blocks.Interfaces.RealInput[nZones] TSensor(
      final quantity="ThermodynamicTemperature",
      unit="K",
      displayUnit="degC",
      min=0) "Sensor temperature" annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=270,
          origin={-40,-106})));
    ElementaryBlocs.PumpSupply_m_flow[nZones] pumpSupply_m_flow1(
                                                                redeclare
        package
        Medium = Medium,
      m_flow_nominal=m_flow_nominal,
      each measureSupplyT=false,
      each KvReturn=40)
      annotation (Placement(transformation(extent={{-82,-18},{-48,18}})));
    Modelica.Blocks.Math.Gain[nZones] gain(k=2)
      "Multiply the mass flow rate as this circuit has a smaller temperature difference"
      annotation (Placement(transformation(extent={{-5,-5},{5,5}},
          rotation=270,
          origin={47,57})));
    Buildings.Fluid.FixedResistances.Junction[nZones] spl1(
      redeclare package Medium = Medium,
      each dp_nominal={0,0,100},
      each energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      each m_flow_nominal={m_flow_nominal[1],m_flow_nominal[1],m_flow_nominal[1]})                    "Splitter/mixer"
      annotation (Placement(transformation(
          extent={{-7,-7},{7,7}},
          rotation=0,
          origin={-31,11})));
    Buildings.Fluid.FixedResistances.Junction[nZones] spl2(
      redeclare package Medium = Medium,
      each m_flow_nominal={m_flow_nominal[1], m_flow_nominal[1],m_flow_nominal[1]},
      each dp_nominal=0*{1,1,1},
      each energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState) "Splitter/mixer"
      annotation (Placement(transformation(
          extent={{-7,-7},{7,7}},
          rotation=180,
          origin={-31,-11})));
    Buildings.Fluid.FixedResistances.Junction[nZones] spl(
      redeclare package Medium = Medium,
      each m_flow_nominal={m_flow_nominal[1],2*m_flow_nominal[1],m_flow_nominal[1]},
      each dp_nominal={0,0,200},
      each energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState) "Splitter/mixer"
      annotation (Placement(transformation(
          extent={{-7,-7},{7,7}},
          rotation=0,
          origin={27,11})));
    Buildings.Fluid.FixedResistances.Junction[nZones] spl3(
      redeclare package Medium = Medium,
      each dp_nominal=0*{1,1,1},
      each m_flow_nominal={2*m_flow_nominal[1],m_flow_nominal[1],m_flow_nominal[1]},
      each energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState) "Splitter/mixer"
      annotation (Placement(transformation(
          extent={{-7,-7},{7,7}},
          rotation=180,
          origin={27,-11})));
    Buildings.Controls.Continuous.LimPID[nZones] conV3V(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      k=0.1,
      Ti=600)
      annotation (Placement(transformation(extent={{-28,50},{-18,60}})));
  equation
    connect(ctrl_Temp.weaBus, weaBus) annotation (Line(
        points={{-65.6,56.2},{-65.6,46.1},{-82,46.1},{-82,80}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(TRooNig.y,swi. u3) annotation (Line(points={{-47.5,75},{-39.2,75},{-39.2,
            75.2}}, color={0,0,127}));
    connect(occSch.occupied,swi. u2) annotation (Line(points={{-41.8,82.8},{-40,82.8},
            {-40,80},{-39.2,80}},         color={255,0,255}));
    connect(TRooSet.y,swi. u1) annotation (Line(points={{-47.5,91},{-47.5,90.5},{-39.2,
            90.5},{-39.2,84.8}}, color={0,0,127}));
    connect(heatingControl.y, booleanToReal.u) annotation (Line(points={{12.6,80},
            {12.6,80},{16.8,80}}, color={255,0,255}));

    for i in 1:nZones loop
          connect(port_a1, pumpSupply_m_flow1[i].port_a1) annotation (Line(points={{-100,
            60},{-92,60},{-92,10.8},{-82,10.8}}, color={0,127,255}));
    connect(port_b2, pumpSupply_m_flow1[i].port_b2) annotation (Line(points={{-100,
            -60},{-92,-60},{-92,-10.8},{-82,-10.8}}, color={0,127,255}));
  connect(ctrl_Temp.THeaCur, conV3V[i].u_s) annotation (Line(points={{-38,54},{-34,
            54},{-34,55},{-29,55}}, color={0,0,127}));

            if includePipes then

          connect(pumpSupply_m_flow[i].heatPort, prescribedTemperature.port)
      annotation (Line(points={{55,-18},{55,-32},{35,-32}}, color={191,0,0}));

            end if;
    end for;

    if includePipes then
      connect(ave.y, prescribedTemperature.T) annotation (Line(points={{-11.4,-58},{
            35,-58},{35,-47.4}}, color={0,0,127}));
      connect(TSensor, ave.u) annotation (Line(points={{-40,-106},{-40,-106},{-40,-58},
            {-25.2,-58}}, color={0,0,127}));

    end if;
    connect(pumpSupply_m_flow.port_b1, port_b1) annotation (Line(points={{72,10.8},
            {76,10.8},{76,60},{100,60}}, color={0,127,255}));
    connect(pumpSupply_m_flow.port_a2, port_a2) annotation (Line(points={{72,-10.8},
            {76,-10.8},{76,-12},{76,-60},{100,-60}}, color={0,127,255}));
    connect(TSet_max.y, ctrl_Temp.TRoo_in1)
      annotation (Line(points={{-69.1,53},{-69.1,54},{-58,54}},
                                                              color={0,0,127}));
    connect(ctrl_Temp.THeaterSet, THeaterSet)
      annotation (Line(points={{-39,51},{60,51},{60,110}}, color={0,0,127}));
    connect(mixingCircuit_Tset.Tsup, TSupply) annotation (Line(points={{11.92,18.72},
            {11.92,59.36},{-20,59.36},{-20,110}}, color={0,0,127}));
    connect(add.y, heatingControl.u)
      annotation (Line(points={{-3.4,80},{-3.4,80},{-1.2,80}}, color={0,0,127}));
    connect(swi.y, add.u2) annotation (Line(points={{-25.4,80},{-22,80},{-22,76.4},
            {-17.2,76.4}}, color={0,0,127}));
    connect(TSensor, add.u1) annotation (Line(points={{-40,-106},{-40,-106},{-40,83.6},
            {-17.2,83.6}}, color={0,0,127}));
    connect(booleanToReal.y, pumpSupply_m_flow1.u) annotation (Line(points={{30.6,
            80},{34,80},{34,19.44},{-65,19.44}}, color={0,0,127}));
    connect(booleanToReal.y, gain.u)
      annotation (Line(points={{30.6,80},{47,80},{47,63}}, color={0,0,127}));
    connect(gain.y, pumpSupply_m_flow.u) annotation (Line(points={{47,51.5},{47,19.44},
            {55,19.44}}, color={0,0,127}));
    connect(pumpSupply_m_flow1.port_b1, spl1.port_1) annotation (Line(points={{-48,
            10.8},{-44,10.8},{-44,11},{-38,11}}, color={0,127,255}));
    connect(spl1.port_2, mixingCircuit_Tset.port_a1) annotation (Line(points={{-24,
            11},{-22,11},{-22,10.8},{-18,10.8}}, color={0,127,255}));
    connect(pumpSupply_m_flow1.port_a2, spl2.port_2) annotation (Line(points={{-48,
            -10.8},{-44,-10.8},{-44,-11},{-38,-11}}, color={0,127,255}));
    connect(spl2.port_1, mixingCircuit_Tset.port_b2) annotation (Line(points={{-24,
            -11},{-22,-11},{-22,-10.8},{-18,-10.8}}, color={0,127,255}));
    connect(spl1.port_3, spl2.port_3)
      annotation (Line(points={{-31,4},{-31,4},{-31,-4}}, color={0,127,255}));
    connect(spl.port_3, spl3.port_3)
      annotation (Line(points={{27,4},{27,-4}}, color={0,127,255}));
    connect(mixingCircuit_Tset.port_b1, spl.port_1) annotation (Line(points={{16,10.8},
            {18,10.8},{18,11},{20,11}},         color={0,127,255}));
    connect(spl.port_2, pumpSupply_m_flow.port_a1) annotation (Line(points={{34,11},
            {38,11},{38,10.8},{38,10.8}}, color={0,127,255}));
    connect(mixingCircuit_Tset.port_a2, spl3.port_2) annotation (Line(points={{16,
            -10.8},{18,-10.8},{18,-11},{20,-11}}, color={0,127,255}));
    connect(spl3.port_1, pumpSupply_m_flow.port_b2) annotation (Line(points={{34,-11},
            {36,-11},{36,-10.8},{38,-10.8}}, color={0,127,255}));
    connect(conV3V.y, mixingCircuit_Tset.TMixedSet)
      annotation (Line(points={{-17.5,55},{-1,55},{-1,18}}, color={0,0,127}));
    connect(TSensor, conV3V.u_m) annotation (Line(points={{-40,-106},{-40,31},{
            -23,31},{-23,49}}, color={0,0,127}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                                     Line(
            points={{-102,60},{98,60}},
            color={255,85,85},
            thickness=0.5),      Line(
            points={{-102,-60},{98,-60}},
            color={0,127,255},
            thickness=0.5),
          Polygon(
            points={{-40,70},{-40,50},{-20,60},{-40,70}},
            lineColor={0,0,127},
            smooth=Smooth.None,
            fillColor={0,128,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{0,70},{0,50},{-20,60},{0,70}},
            lineColor={0,0,127},
            smooth=Smooth.None,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-10,10},{-10,-10},{10,0},{-10,10}},
            lineColor={0,0,127},
            smooth=Smooth.None,
            fillColor={0,128,255},
            fillPattern=FillPattern.Solid,
            origin={-20,50},
            rotation=90),
          Line(
            points={{-20,40},{-20,0},{0,-40},{40,-60}},
            color={0,127,255},
            pattern=LinePattern.Dash),
          Ellipse(extent={{40,80},{80,40}}, lineColor={0,0,127},
            fillColor={0,127,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{48,76},{48,44},{80,60},{48,76}},
            lineColor={0,0,127},
            smooth=Smooth.None,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}),                          Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end InjectionCircuit;

  model DischargeCircuit
    "Mixing circuit controlled on the supply mass flow rate. Variable mass flow, constante temperature"
    parameter Integer nZones(min=1)    "Number of conditioned thermal zones deserved by the system" annotation(Dialog(group= "Settings parameters"));
    parameter Boolean includePipes=false "Set to true to include pipes in the basecircuit" annotation(Dialog(group= "Settings parameters"));

    parameter Modelica.SIunits.MassFlowRate[nZones] m_flow_nominal( min=0)   "Nominal mass flow rates";

    extends FBM.Controls.ControlHeating.Interfaces.ControlPara;
    replaceable package Medium=Buildings.Media.Water;

    parameter Modelica.SIunits.Temperature[nZones] T_SetPoint_Day= {294.15 for i in 1:
        nZones} "Set point temperature for room by day" annotation(Dialog(group= "Heating controler parameters"));
    parameter Modelica.SIunits.Temperature[nZones] T_SetPoint_night= {291.15 for i in 1:
        nZones}
               "Set point temperature for room by night" annotation(Dialog(group= "Heating controler parameters"));

    parameter Real KvV3V = 20;

    FBM.ElementaryBlocs.MixingCircuit_Linear[nZones] mixingCircuit_FlowSet(
      redeclare package Medium = Medium,
      m_flow_nominal=m_flow_nominal,
      each KvReturn=KvV3V,
      Ti=600,
      Td=60,
      useBalancingValve=true,
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      dpValve_nominalSupply=6000,
      CvDataReturn=Buildings.Fluid.Types.CvTypes.Kv)
      annotation (Placement(transformation(extent={{46,-22},{12,14}})));
    ElementaryBlocs.PumpSupply_m_flow[nZones] pumpSupply_m_flow(redeclare
        package
        Medium = Medium, each includePipes=includePipes,
      m_flow_nominal=m_flow_nominal,
      each measureSupplyT=false,
      each KvReturn=40,
      measurePower=false)
      annotation (Placement(transformation(extent={{-54,-22},{-20,14}})));
    Modelica.Fluid.Interfaces.FluidPort_a[nZones] port_a2(redeclare package
        Medium = Medium)
      annotation (Placement(transformation(extent={{90,-70},{110,-50}})));
    Modelica.Fluid.Interfaces.FluidPort_b[nZones] port_b1(redeclare package
        Medium = Medium)
      annotation (Placement(transformation(extent={{90,50},{110,70}})));
    Modelica.Fluid.Interfaces.FluidPort_a port_a1(redeclare package Medium =
          Medium)
      annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
    Modelica.Fluid.Interfaces.FluidPort_b port_b2(redeclare package Medium =
          Medium)
      annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
    Modelica.Blocks.Interfaces.RealOutput TSupply[nZones](
      final quantity="ThermodynamicTemperature",
      unit="K",
      displayUnit="degC",
      min=0) "TSupply after mixing" annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-20,110}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-20,110})));
  public
    Buildings.Utilities.Math.Average ave(nin=nZones) if  includePipes
      "Compute average of room temperatures"
      annotation (Placement(transformation(extent={{-6,-6},{6,6}},
          rotation=90,
          origin={-40,-76})));
    Buildings.HeatTransfer.Sources.PrescribedTemperature
      prescribedTemperature if includePipes annotation (Placement(transformation(
          extent={{-7,-7},{7,7}},
          rotation=90,
          origin={-37,-47})));
    Modelica.Blocks.Interfaces.RealInput[nZones] TSensor(
      final quantity="ThermodynamicTemperature",
      unit="K",
      displayUnit="degC",
      min=0) "Sensor temperature" annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=270,
          origin={-40,-106})));
    Modelica.Blocks.Sources.Constant[nZones] TRooNig(k={T_SetPoint_night[i] for i in
              1:nZones})
      "Room temperature set point at night"
      annotation (Placement(transformation(extent={{-70,72},{-60,82}})));
   Modelica.Blocks.Sources.Constant[nZones] TRooSet(k={T_SetPoint_Day[i] for i in
              1:nZones})
      annotation (Placement(transformation(extent={{-78,86},{-68,96}})));
   Modelica.Blocks.Logical.Switch[nZones] swi "Switch to select set point"
      annotation (Placement(transformation(extent={{-56,82},{-44,94}})));
        Buildings.Controls.SetPoints.OccupancySchedule[ nZones]
                                         occSch "Occupancy schedule"
      annotation (Placement(transformation(extent={{-64,88},{-60,92}})));
    Buildings.Controls.Continuous.LimPID[ nZones] conPID(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      k=0.01,
      Ti=600)
      annotation (Placement(transformation(extent={{-36,82},{-24,94}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal[nZones](realTrue=
          m_flow_nominal)
      annotation (Placement(transformation(extent={{28,84},{40,96}})));
    Modelica.Blocks.Logical.Hysteresis[nZones] heatingControl(each uLow=-1,
        each uHigh=1)   "onoff controller for the pumps of the emission circuits"
      annotation (Placement(transformation(extent={{10,84},{22,96}})));
  protected
    Modelica.Blocks.Math.Add add[nZones](each k1=-1, each k2=+1)
      annotation (Placement(transformation(extent={{-6,84},{6,96}})));
  equation

    for i in 1:nZones loop
       connect(port_a1, pumpSupply_m_flow[i].port_a1) annotation (Line(points={{-100,
            60},{-80,60},{-80,6.8},{-54,6.8}}, color={0,127,255}));
    connect(port_b2, pumpSupply_m_flow[i].port_b2) annotation (Line(points={{-100,
            -60},{-80,-60},{-80,-14.8},{-54,-14.8}}, color={0,127,255}));

            if includePipes then

          connect(pumpSupply_m_flow[i].heatPort, prescribedTemperature.port)
      annotation (Line(points={{-37,-22},{-37,-40}},        color={191,0,0}));

            end if;
    end for;

    if includePipes then
      connect(ave.y, prescribedTemperature.T) annotation (Line(points={{-40,-69.4},
              {-37,-69.4},{-37,-55.4}},
                                 color={0,0,127}));
      connect(TSensor, ave.u) annotation (Line(points={{-40,-106},{-40,-106},{-40,
              -83.2}},    color={0,0,127}));

    end if;
    connect(mixingCircuit_FlowSet.Tsup, TSupply) annotation (Line(points={{
            16.08,14.72},{16.08,60},{-20,60},{-20,64},{-20,110}}, color={0,0,
            127}));
    connect(pumpSupply_m_flow.port_b1, mixingCircuit_FlowSet.port_b1)
      annotation (Line(points={{-20,6.8},{12,6.8}}, color={0,127,255}));
    connect(pumpSupply_m_flow.port_a2, mixingCircuit_FlowSet.port_a2)
      annotation (Line(points={{-20,-14.8},{12,-14.8}}, color={0,127,255}));
    connect(mixingCircuit_FlowSet.port_b2, port_a2) annotation (Line(points={{
            46,-14.8},{64,-14.8},{64,-60},{100,-60}}, color={0,127,255}));
    connect(TRooNig.y,swi. u3) annotation (Line(points={{-59.5,77},{-57.2,77},{
            -57.2,83.2}},
                    color={0,0,127}));
    connect(occSch.occupied,swi. u2) annotation (Line(points={{-59.8,88.8},{-58,
            88.8},{-58,88},{-57.2,88}},   color={255,0,255}));
    connect(TRooSet.y,swi. u1) annotation (Line(points={{-67.5,91},{-67.5,98.5},
            {-57.2,98.5},{-57.2,92.8}},
                                 color={0,0,127}));
    connect(swi.y, conPID.u_s)
      annotation (Line(points={{-43.4,88},{-37.2,88}}, color={0,0,127}));
    connect(conPID.y, mixingCircuit_FlowSet.y) annotation (Line(points={{-23.4,
            88},{29,88},{29,14.72}}, color={0,0,127}));
    connect(mixingCircuit_FlowSet.port_a1, port_b1) annotation (Line(points={{
            46,6.8},{66,6.8},{66,60},{100,60}}, color={0,127,255}));
    connect(TSensor, conPID.u_m) annotation (Line(points={{-40,-106},{-40,-106},
            {-40,80.8},{-30,80.8}}, color={0,0,127}));
    connect(heatingControl.y,booleanToReal. u) annotation (Line(points={{22.6,90},
            {22.6,90},{26.8,90}}, color={255,0,255}));
    connect(add.y,heatingControl. u)
      annotation (Line(points={{6.6,90},{6.6,90},{8.8,90}},    color={0,0,127}));
    connect(swi.y, add.u1) annotation (Line(points={{-43.4,88},{-42,88},{-42,96},
            {-7.2,96},{-7.2,93.6}}, color={0,0,127}));
    connect(TSensor, add.u2) annotation (Line(points={{-40,-106},{-40,-106},{
            -40,86.4},{-7.2,86.4}}, color={0,0,127}));
    connect(booleanToReal.y, pumpSupply_m_flow.u) annotation (Line(points={{
            40.6,90},{40,90},{40,38},{-37,38},{-37,15.44}}, color={0,0,127}));
           annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                                     Line(
            points={{-102,60},{98,60}},
            color={255,85,85},
            thickness=0.5),      Line(
            points={{-102,-60},{98,-60}},
            color={0,127,255},
            thickness=0.5),
          Polygon(
            points={{-40,70},{-40,50},{-20,60},{-40,70}},
            lineColor={0,0,127},
            smooth=Smooth.None,
            fillColor={0,128,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{0,70},{0,50},{-20,60},{0,70}},
            lineColor={0,0,127},
            smooth=Smooth.None,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-10,10},{-10,-10},{10,0},{-10,10}},
            lineColor={0,0,127},
            smooth=Smooth.None,
            fillColor={0,128,255},
            fillPattern=FillPattern.Solid,
            origin={-20,50},
            rotation=90),
          Line(
            points={{-20,40},{-20,0},{0,-40},{40,-60}},
            color={0,127,255},
            pattern=LinePattern.Dash),
          Ellipse(extent={{40,80},{80,40}}, lineColor={0,0,127},
            fillColor={0,127,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{48,76},{48,44},{80,60},{48,76}},
            lineColor={0,0,127},
            smooth=Smooth.None,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}),                          Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end DischargeCircuit;
end Circuit;
