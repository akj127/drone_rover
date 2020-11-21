class controller:

	def _init_(self,k1,k2,k3,xd,yd,xr,yr):
		self.kd = k1
	    	self.ki = k2
	    	self.kp = k3
	    	self.xd=  xd
	    	self.xr=  xr
	    	self.yd = yd
	    	self.yr=  yr
	    	self.x_integ=0
	    	self.y_integ=0
	    	self.ti=0
	    	self.prev_ex=0
	    	self.prev_ey=0

	def get_coordinates(self,t):
	
	 

		ex = self.xr-self.xd
        	ey = self.yr-self.yd

    	  	self.x_integ += ex*(t-ti)
    	  	self.y_integ += ey*(t-ti)
          
    	  	edotx = (ex-self.prev_ex)/(t-ti)
    	  	edoty = (ey-self.prev_ey)/(t-ti)
          
    	  	out_x = self.kp*(ex)+self.ki*(self.x_integ)+self.kd*(edotx)
    	  	out_y = self.kp*(ey)+self.ki*(self.y_integ)+self.kd*(edoty)
          
		self.prev_ex= ex
	      	self.prev_ex = ey
      		self.ti = t
			
		print(out_x,out_y)

		
