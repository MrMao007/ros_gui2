from PIL import Image
 
file_path='/home/liu/catkin_ws/src/vwalls/map/3d.jpg'
write_path='/home/liu/catkin_ws/src/vwalls/map/3d.pgm'
Image.open(file_path).save(write_path)
