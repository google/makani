function [x,xd,xdd] = SineWaveAndDerivatives(amp,frq,pha,angles);

x = amp*sin(frq*angles + pha);
xd = amp*frq*cos(frq*angles + pha);
xdd = -amp*frq*frq*sin(frq*angles + pha);