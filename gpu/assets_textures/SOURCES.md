# Surface textures

Photographs of surfaces for the realistic view, 1024 x 1024, scaled down and recompressed from the
1K JPG sets of [ambientCG](https://ambientcg.com), under CC0 1.0 (no credit needed; given anyway).

| Files | ambientCG set |
|---|---|
| `brick_*.jpg` | Bricks059 |
| `asphalt_*.jpg` | Asphalt012 |
| `pavement_*.jpg` | PavingStones092 |
| `plaster_*.jpg` | Plaster001 |
| `concrete_*.jpg` | Concrete034 |
| `lawn_*.jpg` | Grass001 |
| `bark_*.jpg` | Bark007 |
| `roof_*.jpg` | Gravel022 |

`*_albedo.jpg` comes from each set's `Color` map, `*_normal.jpg` from `NormalGL`. The viewer finds a
kind's layer by the word in a material's name (`surfacePatternOf`). Without these files it draws its
procedural surfaces.
