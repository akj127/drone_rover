class Controller:

    def __init__(self, k1, k2, k3):
        self.kd = k1
        self.ki = k2
        self.kp = k3

        self.x_integ = 0
        self.y_integ = 0

        self.prev_ex = 0
        self.prev_ey = 0

    def get_coordinates(self, ex, ey, t=0.1):
        self.x_integ += ex*(t)
        self.y_integ += ey*(t)
      
        edotx = (ex-self.prev_ex)/(t)
        edoty = (ey-self.prev_ey)/(t)
      
        out_x = self.kp*(ex)+self.ki*(self.x_integ)+self.kd*(edotx)
        out_y = self.kp*(ey)+self.ki*(self.y_integ)+self.kd*(edoty)
          
        self.prev_ex = ex
        self.prev_ey = ey

        return out_x, out_y
