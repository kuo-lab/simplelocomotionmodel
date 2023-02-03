function speederror = getspeedp(w, P, desiredspeed)
  [xc,tc,xs,ts,energies,indices] = onestep(set(w, 'P', P));
  speederror = gaitspeed(w, xc, tc, xs, ts, energies, indices) - desiredspeed;
end

