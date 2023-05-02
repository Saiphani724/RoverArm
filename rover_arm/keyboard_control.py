from pynput import keyboard
import pybullet as p

class KeyboardAction:
    def __init__(self):
        self.action = [0,0,0,0,0,0]
        self.df = 0.1
        self.df_max = 1
        self.da = 0.1
        self.da_max = 1
        self.dr = 0.1
        self.dr_max = 1

    def on_press(self, key):
        try:
            # X-axis
            if key.char == "d":
                if self.action[2] < self.da_max:
                    self.action[2] += self.da
            elif key.char == "a":
                if self.action[2] > -self.da_max:
                    self.action[2] += -self.da
            # Y-axis 
            if key.char == "w":
                if self.action[3] < self.da_max:
                    self.action[3] += self.da
            elif key.char == "s":
                if self.action[3] > -self.da_max:
                    self.action[3] += -self.da
            # Z-axis
            if key.char == "q":
                if self.action[4] < self.da_max:
                    self.action[4] += self.da
            if key.char == "e":
                if self.action[4] > -self.da_max:
                    self.action[4] += -self.da
            # Arm Fingers
            if key.char == "=":
                if self.action[5] < self.df_max:
                    self.action[5] += self.df
            elif key.char == "-":
                if self.action[5] > -1 :
                    self.action[5] -= self.df
            # print(key.char)

        except AttributeError:
            # Rover Control
            if key == key.up:
                if self.action[0] < self.dr_max:
                    self.action[0] += self.dr
            elif key == key.down:
                if self.action[0] > -self.dr_max:
                    self.action[0] += -self.dr
            if key == key.left:
                if self.action[1] < self.dr_max:
                    self.action[1] += self.dr
            elif key == key.right:
                if self.action[1] > -self.dr_max:
                    self.action[1] += -self.dr
                
        # print(self.action)

    def on_release(self, key):
        
        self.action[:-1] = [0,0,0,0,0]
        # print(self.action)

    def start_listening(self):
        listener = keyboard.Listener(
            on_press = self.on_press,
            on_release= self.on_release)
        listener.start()
    




