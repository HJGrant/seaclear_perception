# Synthetic Results

production validation ではなく algorithm-level validation として実施した。

| scenario | switches | false spans | false frames | reacquire success / attempts | clutter false selects | hold transitions | lost transitions | escalate transitions |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `scenario_a` | 0 | 6 | 11 | 1/6 | 0 | 5 | 6 | 5 |
| `scenario_b` | 29 | 32 | 79 | 3/32 | 7 | 33 | 78 | 75 |
