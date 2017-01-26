within FBM.Components.BaseClasses;
record PartiWall =
    Buildings.HeatTransfer.Data.OpaqueConstructions.Generic (
      material={Buildings.HeatTransfer.Data.Solids.GypsumBoard(x=0.013),
                Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.10),
                Buildings.HeatTransfer.Data.Solids.GypsumBoard(x=0.013)},
                final nLay=3)
  "BA13 Insu BA13, partition wall for heat and sound isolation"
  annotation (
    defaultComponentPrefixes="parameter",
    defaultComponentName="datOpaCon");
