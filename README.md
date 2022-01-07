# No-Engine-Game-Engine-Space-Flier
This Is the culmination of the assignments in my CS 409 Interactive Entertainment Software class. It is a C++ game using OpenGL and no game engine

Controls, angle you ship with the arrow keys or roll with '<' and '>'. Hit the gas with the space bar or apply your space breaks with 'b'.
You can also nudge your ship in different directions with wasd keys.
Fly close to an asteroid and press 'k' to knock off crystals or press 't' to toggle debug mode.

All the asteroids, drones, crystals, and your ship are effected by the gravity of the black hole and can all collide with eachother. See the expected path you and your drones will drift in with a predicted path.
![Predicted path](https://user-images.githubusercontent.com/56166683/148467930-1bd6c69f-300b-408c-ac56-502b6765932c.png)

The asteroids are created using Perlin Noise. They collide with other objects based off of their specific generated shape.
![PerlinAsteroids](https://user-images.githubusercontent.com/56166683/148468126-dce6f9b7-37ab-41e2-930b-7d8519b60ca3.png)

Your drones will always attempt to fly in an escort formation around your ship unless their are drifting crystals. If there are crystals not being pursued then they will pursue them until caught. They will also attempt to avoid asteroids if they feel they are to close with a hard veer away from them.
![Chasing Crystals](https://user-images.githubusercontent.com/56166683/148468304-1a0beb7c-762d-418b-bde3-e41f92f1cf72.png)

In Debug mode you will see markers on the escort poitions around you ship with the target position ahead that the drones are flying towards to stay in formation. You will also see the same markers on crystals that are being pursued and a future position of the crystal that drones are flying towards.
![Debug Mode 2](https://user-images.githubusercontent.com/56166683/148468578-d6eea910-dd83-477b-87f7-82a65359ac0a.png)

Asteroids will also have lines coming out of their local axes with markers sitting on the asteroids surface along the worlds axes. These markers sit on the shape of the asteroid as it rotates like bouys bouncing on waves.
![Debug Mode 1](https://user-images.githubusercontent.com/56166683/148473585-b462f4c0-268e-418a-96d0-867789ef964a.png)
