Nodo de ROS Hydro para guardar imagenes de profundidad y color del Turtlebot 1

==========================

Se suscribe a /camera/depth_registered/image_raw para las imagenes de profundidad
y a /camera/rgb/color/image_raw para las imagenes a color.

Las imagenes se guardan adonde se corra el nodo. Para compilarlo,
situarse en la carpeta raiz de src (que es el workspace del nodo) y 
correr "catkin_make" en la consola. 
Luego ir a la carpeta devel que se ha creado y correr "source setup.bash"
para decirle a ROS que ahí hay un nodo.
Finalmente, "rosrun obtener_depth obtener_depth"

