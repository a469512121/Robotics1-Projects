function [gain2,gain3] = gravitational_offset(P03,P04)
Pcg2 = P03 + .4*(P04 - P03);
gain2 = Pcg2(1);
Pcg3 = P04;
gain3 = Pcg3(1) - P03(1);