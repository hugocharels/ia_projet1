import cv2
from lle import World

world = World.from_file("cartes/corners")
img = world.get_image()
cv2.imwrite('world.png', img)


