# UAV-CBF

UAV obstacle avoidance via Control Barrier Functions

**IDEA:**
Si runna UAV.m e si lascia il workspace intatto. Poi si apre la scena simulink e si clicca play, cosi tutte le posizione sono già state calcolate.
## sampler.slx
Questi blocchi (sampler) sono fatti per estrarre uno alla volta (1 per ogni istante di tempo) le posizioni di robot e ostacolo. L'uscita dei blocchi è un vettore (x(t), y(t), z(t)) che va dato in pasto a translation degli oggetti. Vanno copiati e aggiunti alla scena simulink di animazione.

## TODO:
- Aggiungere i sampler, per il momento anche solo del drone e provare a vedere se si muove nell'animazione.
  **IMPORTANTE:** Mettere lo stop time (tempo di simulazione in simulink) uguale a quello che abbiamo usato in Matlab (quello di tspan)
- Creare un'altro oggetto per simulare l'ostacolo. Per il momento una sfera va bene. Ci dovrebbe essere una primitiva per farlo facilmente
