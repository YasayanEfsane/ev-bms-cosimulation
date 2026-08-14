function nextState = rk4Step(derivativeFunction, time, state, step)
%RK4STEP One classical fourth-order Runge-Kutta update for the unified state.

k1 = derivativeFunction(time, state);
k2 = derivativeFunction(time + 0.5*step, state + 0.5*step*k1);
k3 = derivativeFunction(time + 0.5*step, state + 0.5*step*k2);
k4 = derivativeFunction(time + step, state + step*k3);
nextState = state + (step/6)*(k1 + 2*k2 + 2*k3 + k4);
end
