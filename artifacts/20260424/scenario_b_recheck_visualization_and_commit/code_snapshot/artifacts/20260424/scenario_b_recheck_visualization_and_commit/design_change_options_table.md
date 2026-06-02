| symptom | tuning candidate | design candidate | note |
|---|---|---|---|
| many valid switches | increase `selector_switch_margin`, increase selected-track bonus | add temporal continuity term / hysteresis over multiple frames | only tuning if Yellow |
| same-class clutter false selects | tighten gate, raise confirm requirement | class-aware or shape-aware score, add spatial consistency penalty | Red if repeated while valid |
| reacquire too weak after continuity cut | relax hold / lost transition boundary | explicit reacquire state or target identity memory | consider only after production-like Red |
