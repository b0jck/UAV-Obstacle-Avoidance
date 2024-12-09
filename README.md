# UAV Trajectory Tracking and Obstacle Avoidance via Control Barrier Functions

This project involves Unmanned Aerial Vehicle (UAV) control to ensure Trajectory Tracking and Obstacle Avoidance using Control Barrier Functions.
More info on the mathematical model and control technique can be found in the [report](report_UAV.pdf).

The work done is generalizable to any scenario, with any given reference trajectory and set of obstacles (both static and dynamic). However, four scenarios were considered and are available for download in each folder of this repository.

The files in each folder include:
- A
**IDEA:**
Si runna UAV.m e si lascia il workspace intatto. Poi si apre la scena simulink e si clicca play, cosi tutte le posizione sono già state calcolate.
## sampler.slx
Questi blocchi (sampler) sono fatti per estrarre uno alla volta (1 per ogni istante di tempo) le posizioni di robot e ostacolo. L'uscita dei blocchi è un vettore (x(t), y(t), z(t)) che va dato in pasto a translation degli oggetti. Vanno copiati e aggiunti alla scena simulink di animazione.

## TODO:
- Aggiungere i sampler, per il momento anche solo del drone e provare a vedere se si muove nell'animazione.
  **IMPORTANTE:** Mettere lo stop time (tempo di simulazione in simulink) uguale a quello che abbiamo usato in Matlab (quello di tspan)
- Creare un'altro oggetto per simulare l'ostacolo. Per il momento una sfera va bene. Ci dovrebbe essere una primitiva per farlo facilmente
