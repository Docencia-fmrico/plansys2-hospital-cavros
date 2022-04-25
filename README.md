[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-c66648af7eb3fe8bc4f294546bfd86ef473780cde1dea487d3c4ff354943c9ae.svg)](https://classroom.github.com/online_ide?assignment_repo_id=7664414&assignment_repo_type=AssignmentRepo)
# PLANSYS2 HOSPITAL
## Introducción
En este proyecto se pide resolver un problema de planificación, empleando un entorno simulado de un hospital en gazebo 11, con el robot Tiago de PAL Robotics.

El problema a resolver consiste en dejar diversos objetos repartidos por el entorno, en la recepción del hospital, mediante el uso de plansys2.
## PDDL
En primer lugar, necesitamos plantear el problema para que el planificador lo resuelva correctamente. Para ello hemos decidido tomar la siguiente distribución de zonas:

![Imagen de zonas](https://github.com/Docencia-fmrico/plansys2-hospital-cavros/blob/main/media/world_zones.png?raw=true "World Zones")

Con esto como referencia, podemos empezar a definir el dominio. 

### Tipos
Para ello primero establecemos los tipos que vamos a usar, en este caso:

```
(:types
    room zone corridor - location
    robot
    gripper  
    object
)
```

- Localizaciones, para situar cada ubicación del mundo.
- Robot, que definirá al Tiago de la simulación.
- Gripper, como herramienta para realizar las acciones de pick & place.
- Object, que representará los distintos objetos a trasladar.

### Predicados
Los cuales los subdividimos en 3 grupos: los de localización que relacionan las ubicaciones entre sí; los del robot, que definen el estado del robot (posición, herramienta, ...etc.); y por último los del objeto, que establecen la localización del mismo.

```
(:predicates  
  ;; Locations
  (in ?l1 ?l2 - location)
  (connected ?l1 ?l2 - location)

  ;; Robot
  (robot_at ?r - robot ?l - location)
  (gripper_free ?g - gripper)
  (gripper_at ?g - gripper ?r - robot)
  (robot_carry ?r - robot ?g - gripper ?o - object)

  ;; Object
  (object_at ?o - object ?l - location)
)
```

### Acciones
Por último, para resolver el problema propuesto, requerimos de acciones que definan el movimiento del robot, y de acciones que permitan al robot interactuar con los objetos.

Dichas acciones están definidas como durative-actions, para poder emplearlas en el planificador de plansys2 y poder simular el efecto temporal de las mismas.

Las acciones empleadas son las siguientes:

- move, para desplazar al robot entre las diversas localizaciones.
- pick, para coger el objeto de una localización.
- place, para dejar el objeto en una localización.

## PlanSys2
Para conseguir que nuestro robot realice estas acciones empleamos plansys2 con ros2.

Por un lado tenemos las acciones de nuestro problema : move , pick y place implementadas como nodos que heredan ActionExecutorClient.

Las acciones pick y place tiene una implementación sencilla , en la que una variable de progreso simulará el tiempo que estas acciones tardan en ejecutarse ( ya que el robot no cogerá realmente ningun objeto del escenario). Cuando esta variable llega al 100% la acción se dará por terminado y se pasará a la siguiente.

La acción move crea un cliente de navegación. Cuando se activa este nodo, el cliente establece el goal al que quiere ir va devolviendo un feedback en relación a la distancia a la que se encuentra de dicho goal.

Para poder comandar a plansys2 la ejecución de un plan hemos creado un controlador : controller_node. En este definimos el estado inicial y el goal del problema.

Por tanto ejecutando nuestro escenario con tiago , la navegación nav2 con el mismo , nuestro hospital.launch.py que lanza plansys2 y finalmente el controlador veremos como poco a poco el plan se va ejecutando por el robot .


## Contribuidores
* Rubén Montilla Fernández
* Blanca Soria Rubio
* Victor de la Torre Rosa
* Cristian Sánchez Rodríguez 
