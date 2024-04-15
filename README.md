El objetivo principal del trabajo es introducirse al estudio de los sistemas dinámicos multiagente con su aplicación en el control distribuido de enjambres. Entre otras muchas aplicaciones, estos sistemas se pueden explotar para caracterizar campos escalares por ejemplo, para la búsqueda y confinamiento de fuentes de contaminación, para barrer grandes volúmenes en operaciones de búsqueda y rescate, o para coordinar robots en tareas de vigilancia.
La propuesta de este TFG se centra en confeccionar algoritmos fundamentales (ver bibliografía) en multiagentes para alguna de las aplicaciones arriba nombradas. En particular, se hará explotando las garantías y propiedades matemáticas de los algoritmos. Estas decisiones incluyen qué topología escoger para el grafo del enjambre, qué información local han de compartir o medir los agentes, cuál es su dinámica, etc. Finalmente, analizar los pros y contras a la hora de implementar el algoritmo para alcanzar el objetivo de la aplicación.
Sobre el aspecto técnico, este TFG requiere que el alumno se centre en dos componentes principales. Primero, sobre la teoría de grafos (fundamentalmente álgebra lineal) que describe las interacciones locales entre los agentes. Segundo, sobre ecuaciones diferenciales ya que los agentes son dinámicos y sus estados evolucionan en el tiempo. En particular, los agentes evolucionan al tomar decisiones principalmente basadas en los estados relativos con sus vecinos en el grafo. La combinación de estas dos componentes se aborda en la asignatura Sistemas dinámicos y realimentación de cuarto año en el grado de física y doble grado de física y matemáticas, aunque cursarla no es un requisito necesario para realizar el TFG. Se utilizará Python y/o Matlab para validaciones numéricas.


Bibliografía:
[1] W Yao, HG de Marina, Z Sun, M Cao. Distributed coordinated path following using guiding vector fields .  Proceedings IEEE International conference on Robotics and Automation (ICRA) 2021. https://arxiv.org/abs/2103.12372
[2]W. Yao, B. Lin, and M. Cao, Integrated path following and collision avoidance using a composite vector field, in 2019 IEEE 58th Conference on Decision and Control (CDC), IEEE, 2019, pp. 250–255.
[3] W. Yao, B. Lin, B. D. O. Anderson, and M. Cao, Guiding vector fields for following occluded paths, IEEE Transactions on Automatic Control (TAC), vol. 67, no. 8, 2022.
[4] HG de Marina. Maneuvering and robustness issues in undirected displacement-consensus-based formation control. IEEE Transactions on Automatic Control, 2021.
https://arxiv.org/abs/2008.03544
[5] HG de Marina, J Jiménez, W Yao. Leaderless collective motions in affine formation control. Conference on Decision and Control (CDC) 2021.
https://arxiv.org/abs/2104.03412
[6] Kahn, Arthur, et al. Global extremum seeking by Kriging with a multi-agent system. IFAC-PapersOnLine 48.28 (2015): 526-531.
https://hal.science/hal-01170131/document
[7] Cortés, J. Distributed Kriged Kalman filter for spatial estimation. IEEE Transactions on Automatic Control, 54(12), 2816-2827 (2009).
