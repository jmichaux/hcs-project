function Obs_new = generateNewZonoObs(Obs, Nonslice)
nObs_new = Obs.nObs;
obs_zonotopesNew = cell(nObs_new, 1);

cobs1_new = Obs.zonotopes{1,1}.Z;
gobs1_new = [Obs.zonotopes{1,1}.O Nonslice.zonotopes{1,1}.O];
obs_zonotopesNew{1,1} = zonotope(cobs1_new, gobs1_new);

Obs_new.zonotopes = obs_zonotopesNew;
Obs_new.nObs = nObs_new;


end