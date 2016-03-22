/* Este es un nodo de ROS Indigo para guardar imagenes de /camera/depth_registered/image_rect_raw
 * y de /camera/rgb/color/image_rect_raw de un Turtlebot1 para luego procesarlas con otro programa. 
 * Las de profundidad se guardan como unsigned int de 16 bits 
 * y 1 canal, las de color se guardan como unsigned int de 8 bits en 3 canales.
 * Se utiliza un suscriptor sincronizado para guardar el par de imagenes que estén más
 * cercanas en el tiempo.
 * LAS IMAGENES SE GUARDAN ADONDE SE EJECUTE EL NODO.
 * ---------------------------------------------------------
 * Creado por Fabricio Emder y Pablo Aguado en el 2016 */


/* This is a ROS Indigo node for saving Turtlebot images from the /camera/depth_registered/image_rect_raw
 * and /camera/rgb/color/image_rect_raw topics, for further processing outside of the program. 
 * Depth images are saved as 1 channel 16 bit unsigned int PNGs (CV_16UC1), and 
 * RGB images are saved as 3 channel 8 bit unsigned int PNGs (CV_8UC3).
 * A synchronized subscriber is used, for saving the pair of images that are most closer in time.
 * THE IMAGES ARE SAVED WHEREVER THE NODE IS RUN.
 * Created by Fabricio Emder and Pablo Aguado - 2016
 * */

 
Ingrese 'a' para guardar un par de imágenes o 'b' para guardar 300 imágenes
Enter 'a' to save a pair of images or 'b' to automatically save 300 images
 
Las imagenes se guardan adonde se corra el nodo. Para compilarlo,
situarse en la carpeta raiz de src (que es el workspace del nodo) y 
correr "catkin_make" en la consola. 
Luego ir a la carpeta devel que se ha creado y correr "source setup.bash"
para decirle a ROS que ahí hay un nodo.
Finalmente, "rosrun obtener_depth obtener_depth". Debe hacerse en la carpeta en que se quieran guardar 
las imágenes y desde la terminal en donde se haya corrido "source setup.bash".