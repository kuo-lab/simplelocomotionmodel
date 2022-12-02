## Tinydemo of locomotion

using DynLoco
using Plots, Statistics
plotlyjs() # Use this backend to preserve fonts on export to SVG or PDF
default(grid=false, fontfamily="Helvetica") # no grid on plots

wstar4 = findgait(WalkRW2l(α=0.35,safety=true), target=:speed=>0.3, varying=:P)
step = onestep(wstar4)
simulatestep(wstar4)