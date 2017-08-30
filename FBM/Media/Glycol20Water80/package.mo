within FBM.Media;
package Glycol20Water80 "Medium extend from Buildings.Media.Water but with characteristic constants of a mixed glycol 20% water 80%"
  extends Buildings.Media.Water(
  cp_const=3880,
  d_const=1033,
  lambda_const=0.505,
  T_min=Modelica.SIunits.Conversions.from_degC(-8));
end Glycol20Water80;
