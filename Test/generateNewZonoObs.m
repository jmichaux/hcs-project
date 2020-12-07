function Obs_new = generateNewZonoObs(Obs, Nonslice)
cobs1_new = Obs.center;
gobs1_new = [Obs.Z(:,2:end) Nonslice.Z(:,2:end)];
Obs_new = zonotope([cobs1_new, gobs1_new]);

end