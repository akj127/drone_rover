import numpy as np

class controller:

	def _init_(self,k1,k2,k3):
		self.kd = k1
	    	self.ki = k2
	    	self.kp = k3

	def get_coordinates(self,xd,yd,xr,yr,t):
		errorx = xd-xr
    		errory = yd-yr
	    	x_integ = 0
    		y_integ = 0
	    	ti = 0
        
    		for i in range(1,t):   
			ex = xr-xd
        		ey = yr-yd

    	  		x_integ += ex*(i-ti)
    	  		y_integ += ey*(i-ti)
          
    	  		edotx = (ex-errorx)/(i-ti)
    	  		edoty = (ey-errory)/(i-ti)
          
    	  		out_x = self.kp*(ex)+self.ki*(x_integ)+self.kd*(edotx)
    	  		out_y = self.kp*(ey)+self.ki*(y_integ)+self.kd*(edoty)
          
			errorx = ex
	      		errory = ey
      			ti = i
		print(out_x,out_y)
  

c = controller(1,2,3)
c.get_coordinates(4,5,6,7,10)

