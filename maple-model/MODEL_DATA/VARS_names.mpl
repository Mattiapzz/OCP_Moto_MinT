ode_vars := [u(t), v(t), Omega(t), phi(t), theta(t), h(t), delta(t), eta(t), 
s__f(t), x__f(t), y__f(t), z__f(t), x__r(t), y__r(t), z__r(t), delta__f(t), 
phi__f(t), omega__r(t), omega__f(t), phi__d(t), theta__d(t)];
uvars := [Ftr(t), Mbr(t), Mbf(t), tau(t)];
rdr_vars := [tau__phi(t), tau__theta(t)];
alg_kine_vars := [alpha__pin(t), alpha__crw(t)];
alg_vars := [Fzf(t), Fzr(t)];
tyre_vars := [Fxf(t), Fxr(t), Fyf(t), Fyr(t), Mzf(t), Mzr(t), Mxf(t), Mxr(t)]
;
tyre_kine_vars := [alpha__r(t), alpha__f(t), lambda__r(t), lambda__f(t)];
inertia_vars := [M__tot, XG, YG, ZG, IX, IY, IZ, CXZ];
