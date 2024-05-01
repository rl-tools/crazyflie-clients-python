import sdl2
import sdl2.ext

def main():
    sdl2.ext.init()
    if sdl2.SDL_InitSubSystem(sdl2.SDL_INIT_JOYSTICK | sdl2.SDL_INIT_GAMECONTROLLER) != 0:
        print("Failed to initialize SDL subsystems:", sdl2.SDL_GetError())

    num_joysticks = sdl2.SDL_NumJoysticks()
    print(f"Number of joysticks connected: {num_joysticks}")
    for i in range(num_joysticks):
        if sdl2.SDL_IsGameController(i):
            controller = sdl2.SDL_GameControllerOpen(i)
            if controller:
                print(f"Game controller {i} connected.")
            else:
                print(f"Could not open game controller {i}.")

    running = True
    while running:
        events = sdl2.ext.get_events()
        for event in events:
            if event.type == sdl2.SDL_QUIT:
                running = False
            elif event.type == sdl2.SDL_CONTROLLERBUTTONDOWN:
                print(f"Button {event.cbutton.button} pressed on controller {event.cbutton.which}")
            elif event.type == sdl2.SDL_CONTROLLERBUTTONUP:
                print(f"Button {event.cbutton.button} released on controller {event.cbutton.which}")

    sdl2.ext.quit()

if __name__ == "__main__":
    main()