Simple Locomotion Models
================

# Simple model of pendulum-like walking

![walking figure and pendulum model](img/walkingimage.svg)

## Perform short walks of a range of distances

Take walks of varying distances, and show how the optimal trajectory has
an inverted-U velocity profile, with peak speed that increases with
distance up to about 12 steps. The cost function is total work, plus a
linear cost of time with coefficient ctime.

``` julia
using DynLoco, Plots; #plotlyjs()

wstar4 = findgait(WalkRW2l(α=0.35,safety=true), target=:speed=>0.3, varying=:P)
ctime = 0.015 # cost of time, to encourage hurrying
tchange = 1.75 # boundary condition time to get up to speed (arbitrary, excluded from optimization) 
p = plot() 
walksteps = [1, 2, 3, 4, 5, 6, 7, 10, 15, 20] # take walks of this # of steps
results = Array{MultiStepResults,1}(undef,0) # store each optimization result here
for (i,nsteps) in enumerate(walksteps)
    result = optwalktime(wstar4, nsteps, ctime=ctime) # optimize with a cost of time
    plotvees!(result, tchange=tchange, usespline=false, color=i, speedtype=:shortwalks, rampuporder=1, markersize=2) # plot instantaneous body speed vs. time
    push!(results, result) # add this optimization to results array
end
Plots.display(p) # instantaneous speed vs. distance profiles
```

![Speed vs time for short walks; each trace is a different bout
distance](README_files/figure-gfm/cell-2-output-1.png)

## Compare three objectives: Energy-Time, min-COT, constant acceleration

Walk a fixed number of steps, starting and ending at rest. The
objectives are:

- **Energy-Time** minimizes total energy (positive work) plus
  proportional time cost
- **min-COT** minimizes cost of transport (energy per weight and
  distance traveled)
- **Constant accel** accelerates at a constant rate, to yield a
  triangular speed profile. Uses a minimum variance objective to produce
  a constant rate of velocity change.

Compare for a fixed number of steps.

``` julia
## Triangle walk, based on min var walk
wstar4s = findgait(WalkRW2l(α=0.35,safety=true), target=:speed=>0.5, varying=:P)
wstar4n = findgait(WalkRW2l(α=0.35, safety=true), target=:speed=>0.4, varying=:P)
nsteps = 10
ctime = 0.0195
tchange = 1.75
nominalmsr=optwalktime(wstar4n, nsteps, ctime = ctime, boundarywork=true) # to compare with our usual solution
minvarmsr=optwalkvar(wstar4n, nsteps, boundarywork=true)
A = 1.9*wstar4s.vm/(nsteps*onestep(wstar4s).tf)
v0 = 0.11#0.8*A*tchange#0.12
mintrimsr=optwalktriangle(wstar4n, nsteps, A = A, boundarywork=false,boundaryvels=(v0,v0))
p = plot(layout=(1,2))
plotvees!(p[1],nominalmsr, tchange=tchange, rampuporder=1, usespline = false, markershape=:circle,speedtype=:shortwalks)
plotvees!(p[1],minvarmsr, tchange=tchange, rampuporder=1, usespline = false,markershape=:circle, speedtype=:shortwalks)
plotvees!(p[1],mintrimsr, tchange=tchange, rampuporder=1, usespline = false,markershape=:circle, speedtype=:shortwalks, seriescolor=:auto)
plot!(p[2],[0:nsteps+1], [1/2*nominalmsr.vm0^2; nominalmsr.steps.Pwork; NaN],markershape=:circle,seriescolor=:auto)
plot!(p[2],[0:nsteps+1], [1/2*minvarmsr.vm0^2; minvarmsr.steps.Pwork; NaN],markershape=:circle,xticks=0:nsteps+1)
plot!(p[2],[0:nsteps+1], [1/2*mintrimsr.vm0^2; mintrimsr.steps.Pwork; NaN],markershape=:circle,xticks=0:nsteps+1,seriescolor=:auto)
plot!(p[2],xlabel="step", ylabel="push-off work", legend=false)
energytimework = 1/2*nominalmsr.vm0^2 + sum(nominalmsr.steps.Pwork)
mincotwork = 1/2*minvarmsr.vm0^2 + sum(minvarmsr.steps.Pwork)
trianglework = (1/2*mintrimsr.vm0^2 + sum(mintrimsr.steps.Pwork))/(1/2*nominalmsr.vm0^2 + sum(nominalmsr.steps.Pwork))
Plots.display(p)
```

![](README_files/figure-gfm/cell-3-output-1.png)

Quantify the three predictions.

``` julia
using Markdown
println("energy-time work = ", 1/2*nominalmsr.vm0^2 + sum(nominalmsr.steps.Pwork))
println("min-COT = ", 1/2*minvarmsr.vm0^2 + sum(minvarmsr.steps.Pwork))
println("triangle   = ", 1/2*mintrimsr.vm0^2 + sum(mintrimsr.steps.Pwork))
println("ratio = ",  (1/2*minvarmsr.vm0^2 + sum(minvarmsr.steps.Pwork))/(1/2*nominalmsr.vm0^2 + sum(nominalmsr.steps.Pwork)) )
println("ratio = ",  (1/2*mintrimsr.vm0^2 + sum(mintrimsr.steps.Pwork))/(1/2*nominalmsr.vm0^2 + sum(nominalmsr.steps.Pwork)) )
#println("ratio = ",  (sum(minvarmsr.steps.Pwork))/(sum(nominalmsr.steps.Pwork)) )

threecosts = [1/2*nominalmsr.vm0^2 + sum(nominalmsr.steps.Pwork), 1/2*minvarmsr.vm0^2 + sum(minvarmsr.steps.Pwork), 1/2*mintrimsr.vm0^2 + sum(mintrimsr.steps.Pwork)]
bar(threecosts,xticks=((1,2,3),("Energy-Time", "Steady min-COT", "Steady accel")),legend=false)
```

    energy-time work = 0.21077394712113307


    min-COT = 0.23455721768541665
    triangle   = 0.27531432646148135
    ratio = 1.1128378098390652
    ratio = 1.3062066266816958

![](README_files/figure-gfm/cell-4-output-3.png)

``` julia
using Markdown
Markdown.parse("""
The energy-time work is $(threecosts[1]). 
The min-COT work is $(threecosts[2]).
The const accel work is $(threecosts[3]).
""")
```

The energy-time work is 0.21077394712113307. The min-COT work is
0.23455721768541665. The const accel work is 0.27531432646148135.

## Brachistokuo ramp

Optimal slope and walk with ramp, compared with flat walking in same
amount of time. We find six steps to be pretty good. Longer walks yield
a smaller differene because there’s more time spent at a steady speed.
Note: Currently solving for an asymmetric ramp profile, because pure
symmetry constraint is slightly overconstrained for latest Ipopt.

``` julia
wstar = findgait(WalkRW2l(α=0.35,safety=true), target=:speed=>0.4, varying=:P)
N = 6
walktime = N * onestep(wstar).tf *0.82 # meant to be a brisk walk
walkdistance = N * onestep(wstar).steplength
rampresult = optwalkslope(wstar, N, boundaryvels = (0., 0.), symmetric = false,
    totaltime = walktime)
p = multistepplot(rampresult; plotwork=true, label="ramp")
println("ramp total cost = ", rampresult.totalcost)
flatresult = optwalk(wstar, N, boundaryvels = (0., 0.),
    totaltime = rampresult.totaltime, δs = zeros(6))
println("flat total cost = ", flatresult.totalcost)
multistepplot!(flatresult; plotwork=true, label="flat")
# optionally, try a reversed ramp and see if it's higher cost still
#concaveresult = optwalk(wstar, 6, boundaryvels = (0.,0.), boundarywork=true,
#    totaltime = rampresult.totaltime, δ = -rampresult.δangles)
#multistepplot!(concaveresult; plotwork=true)
```

    ramp total cost = 0.1870530240274279
    flat total cost = 

    0.21330138986318115

![](README_files/figure-gfm/cell-6-output-3.png)

### Brachistokuo ramp: Compute the cost for different speeds

``` julia
walktimes = (0.8:0.05:1.2) * N*onestep(wstar).tf
rampresults = Array{MultiStepResults,1}(undef, length(walktimes))
flatresults = Array{MultiStepResults,1}(undef, length(walktimes))
for (i,walktime) in enumerate(walktimes)
    rampresults[i] = optwalk(wstar, N, boundaryvels = (0.,0.),
        totaltime = walktime, δs = rampresult.δangles)
    flatresults[i] = optwalk(wstar, N, boundaryvels = (0.,0.),
        totaltime = walktime, δs = zeros(N))
end
# plot totalcost vs average speed
plot(walkdistance ./ walktimes, [getfield.(rampresults, :totalcost), getfield.(flatresults, :totalcost)],
    xlabel="Average speed", ylabel="Total Work", labels=["Ramp" "Flat"])
```

![](README_files/figure-gfm/cell-7-output-1.png)

### Plot the ramp to scale

``` julia
plot(onestep(wstar).steplength .* (0:6),cumsum(tan.([0;rampresult.δangles]).*onestep(wstar).steplength),
    aspect_ratio=1)
sl = onestep(wstar).steplength
plot(sl .* [0; cumsum(cos.(rampresult.δangles))],[0; cumsum(sin.(rampresult.δangles))].*sl,
    aspect_ratio=1)
```

![](README_files/figure-gfm/cell-8-output-1.png)

## Short walks of varying slope

To be compared with Emily’s data

``` julia
wstar4s = findgait(WalkRW2l(α=0.4,safety=true), target=:speed=>0.4, varying=:P)
myslopes = 0:0.02:0.08
p = plot(layout=(2,1))
nsteps = 6
startv = wstar4.vm
for slope in myslopes
    # walk up a slope
    resultup = optwalktime(wstar4s, nsteps, ctime = ctime, δs=fill(slope, nsteps));
    plotvees!(p[1],resultup, tchange=1, title="Up", rampuporder=1)
    startv = [resultup.vm0;resultup.steps.vm]

    # Walk down a slope
    resultdown = optwalktime(wstar4s, nsteps, ctime = ctime, δs=fill(-slope, nsteps));
    plotvees!(p[2],resultdown, tchange=1, title="Down", rampuporder=1)
end
Plots.display(p)
```

![](README_files/figure-gfm/cell-9-output-1.png)

## Uneven terrain

## Walk over a single bump with fixed and with varying step lengths

``` julia
wstar4s = findgait(WalkRW2l(α=0.4,safety=true), target=:speed=>0.45, varying=:P)
nsteps = 15
δs = zeros(nsteps); δs[Int((nsteps+1)/2)] = 0.05 # one bump
nominalmsr=optwalk(wstar4s, nsteps, boundarywork=false, δs=δs)
# WalkRW2ls has varying step lengths according to preferred human
wstar4ls = findgait(WalkRW2ls(α=0.4,safety=true), target=:speed=>0.45, varying=:P, cstep=0.35, vmstar=wstar4s.vm)
wstar4lvs = findgait(WalkRW2lvs(α=0.4,safety=true), target=:speed=>0.45, varying=:P, c=1., vmstar=wstar4s.vm)
varyingmsr = optwalk(wstar4ls, nsteps, boundarywork=false,δs=δs)
varyingmsrv = optwalk(wstar4lvs, nsteps, boundarywork=false,δs=δs)
plotvees(nominalmsr,boundaryvels=nominalmsr.boundaryvels, speedtype=:shortwalks)
plotvees!(varyingmsr,boundaryvels=(nothing,nothing), speedtype=:shortwalks)
plotvees!(varyingmsrv,boundaryvels=(nothing,nothing), speedtype=:shortwalks)

plot(cumsum(nominalmsr.steps.tf), nominalmsr.steps.vm,label="normal")
plot!(cumsum(varyingmsr.steps.tf), varyingmsr.steps.vm, label="varying" )

# step timings, per step, regular and varying step lengths
plot(cumsum(nominalmsr.steps.tf),nominalmsr.steps.tf, label="normal")
plot!(cumsum(varyingmsr.steps.tf),varyingmsr.steps.tf, label="varying")
```

![](README_files/figure-gfm/cell-10-output-1.png)

## Walk over a bunch of terrains

``` julia
B = 0.05 # bump height, nondimensionalized to leg length L (nominal L = 1 m)
# (bump height affects amplitude of velocity change, but not the shape)

# A bunch of terrains
δs = ( # terrain defined a sequence of height or angle changes from previous step
    "U" =>    [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0] .* B, # Up
    "D" =>    [0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0] .* B, # Down
    "UD" =>   [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0] .* B, # Up-Down
    "DnUD" => [0, 0, 0, 0, 0, 0, -1, 0, 1, -1, 0, 0, 0, 0, 0] .* B, # Down & Up-Down
    "P" =>    [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, -1, -1, -1, 0, 0, 0, 0, 0, 0] .* B, # Pyramid
    "C1" =>   [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0] .* B, # Complex 1
    "C2" =>   [0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0] .* B, # Complex 2
)
p = plot(layout=(2,4), legend=false); plotnum = 1;
for (terrainname, terrainbumps) in δs
    results = optwalk(wstar4s, length(terrainbumps), δs=terrainbumps, boundarywork = false) # optimize push-offs (boundarywork=false means start from nominal walking)
    plotvees!(p[plotnum], results, boundaryvels=results.boundaryvels, speedtype=:midstance, 
        usespline=false,title=terrainname)
    plotnum = plotnum + 1
end
display(p)
```

![](README_files/figure-gfm/cell-11-output-1.png)

## Walk over a simple bump with no compensation

``` julia
upstep = δs[1][2] # first terrain, get the terrain array
nsteps = length(upstep)
nocompmsr = multistep(wstar4s, Ps=fill(wstar4s.P,nsteps),δangles=upstep,boundaryvels=(wstar4s.vm,wstar4s.vm))
nocompmsr.totalcost
multistepplot(nocompmsr)
```

![](README_files/figure-gfm/cell-12-output-1.png)

## Walk over a single bump with a reactive compensation

``` julia
nbump = Int((nsteps+1)/2)
reactmsr1 = multistep(wstar4s, Ps=fill(wstar4s.P,nbump),δangles=upstep[1:nbump],boundaryvels=(wstar4s.vm,wstar4s.vm))
reactmsr2 = optwalk(wstar4s, nsteps-nbump, totaltime = nominalmsr.totaltime - reactmsr1.totaltime - 2,boundaryvels=(reactmsr1.steps[end].vm,wstar4s.vm), boundarywork=(false,false))
multistepplot(reactmsr2)
```

![](README_files/figure-gfm/cell-13-output-1.png)

1.  Plot uneven terrain profile
2.  Draw a stick figure model (see Matlab walkrw2 animation)
3.  Generate automatic superposition figure

## Todo: Short walks

1.  Analyze ramp data (from Emily)
2.  Analyze different speed data

## Step-to-step transitions and energy cost

1.  Metabolic power for preferred walking
2.  Inverse dynamics grid data for COM power, work
3.  Edot = v^3.42 plot

## Todo: Mechanics story

1.  Some nice figures to show energy-time-slope tradeoff
2.  Step-to-step transition through prediction

## Todo: Photos & video of brachistokuo