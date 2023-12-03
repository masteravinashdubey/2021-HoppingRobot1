import pygame 
pygame.init()
while True:
	for event in pygame.event.get():
		if event.type == pygame.KEYDOWN:
			if event.key == pygame.K_LEFT:
				print("left key pressed")
