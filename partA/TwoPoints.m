%% W RPM
% time = simout.time;
% sig = simout.data;
% k = max(sig)
% val_t1 = k * 0.632;
% val_t2 = k * 0.283;
t1 = 1.05795600000000;
t2 = 1.02708900000000;
tau = 1.5 * (t1 - t2)
t0 = t1 - 1 - tau
Pu = t0 / tau

% K = 2.2370e+03 / 240 = 9.3208
% tau = 0.0463
% t0 = 0.0117

%% Ia A
% maxIa = max(simout1.data)
% settleIa = 1.19432500180277;
% del = settleIa - maxIa
% val_t1 = del * 0.632 + maxIa
% val_t2 = del * 0.283 + maxIa
% t1 = 0.08154000000000;
% t2 = 0.05067100000000;
% tau = 1.5 * (t1 - t2)
% t0 = t1 - tau
% Pu = t0 / tau

% del = -70.1308
% tau = 0.0463
% t0 =  0.0352
% Pu =  0.7610


