import Plot

class PlotHandler():
    def __init__(self, x_plot, y_plot, z_plot, throttle_plot):
        self.x_plot = x_plot
        self.y_plot = y_plot
        self.z_plot = z_plot
        self.throttle_plot = throttle_plot

        self.add_angle_plot_annotations(x_plot)
        self.add_angle_plot_annotations(y_plot)
        #self.add_angle_plot_annotations(z_plot)
        self.add_throttle_plot_annotations(throttle_plot)
        self.add_altitude_plot_annotations(z_plot)
        #self.add_acceleration_plot_annotations(z_plot)

        self.vertical_acceleration_graph = Plot.Graph(z_plot, "az", "red")
        self.vertical_velocity_graph = Plot.Graph(z_plot, "vz", "green")
        self.altitute_graph = Plot.Graph(z_plot, "Altitude", "blue")
        self.z_angle_v_graph = Plot.Graph(z_plot, "ZAV", "pink")
        self.z_target_graph = Plot.Graph(z_plot, "ZT", "orange")

        self.y_angle_graph = Plot.Graph(y_plot, "YAngle", "red")
        self.y_angle_v_graph = Plot.Graph(y_plot, "YAV", "green")
        self.y_target_graph = Plot.Graph(y_plot, "YT", "orange")
        self.y_pid_graph = Plot.Graph(y_plot, "YPID", "blue")

        self.x_angle_graph = Plot.Graph(x_plot, "XAngle", "red")
        self.x_angle_v_graph = Plot.Graph(x_plot, "XAV", "green")
        self.x_target_graph = Plot.Graph(x_plot, "XT", "orange")
        self.x_pid_graph = Plot.Graph(x_plot, "XPID", "blue")

        self.lf_graph = Plot.Graph(throttle_plot, "left front", "green")
        self.rf_graph = Plot.Graph(throttle_plot, "right front", "red")
        self.lr_graph = Plot.Graph(throttle_plot, "left rear", "blue")
        self.rr_graph = Plot.Graph(throttle_plot, "right rear", "orange")

        


    def add_angle_plot_annotations(self, plot):
        self.max_angle = 360
        angle1 = str(self.max_angle/4)+"°"
        angle2 = str(2*self.max_angle/4)+"°"
        angle3 = str(3*self.max_angle/4)+"°"
        Plot.GraphAnnotation(plot, 0.5, "0°", "0°")
        Plot.GraphAnnotation(plot, 0.5+0.125*1, angle1, angle1)
        Plot.GraphAnnotation(plot, 0.5+0.125*2, angle2, angle2)
        Plot.GraphAnnotation(plot, 0.5+0.125*3, angle3, angle3)
        Plot.GraphAnnotation(plot, 0.5-0.125*1, "-"+angle1, "-"+angle1)
        Plot.GraphAnnotation(plot, 0.5-0.125*2, "-"+angle2, "-"+angle2)
        Plot.GraphAnnotation(plot, 0.5-0.125*3, "-"+angle3, "-"+angle3)

    def add_throttle_plot_annotations(self, plot):
        Plot.GraphAnnotation(plot, 0.0, "0", "0")
        Plot.GraphAnnotation(plot, 0.25, "25", "25")
        Plot.GraphAnnotation(plot, 0.50, "50", "50")
        Plot.GraphAnnotation(plot, 0.75, "75", "75")

    def add_acceleration_plot_annotations(self, plot):
        self.max_acceleration = 2
        acc1 = str(self.max_acceleration/4)+"m/s²"
        acc2 = str(2*self.max_acceleration/4)+"m/s²"
        acc3 = str(3*self.max_acceleration/4)+"m/s²"
        Plot.GraphAnnotation(plot, 0.5, "0°", "0°")
        Plot.GraphAnnotation(plot, 0.5+0.125*1, acc1, acc1)
        Plot.GraphAnnotation(plot, 0.5+0.125*2, acc2, acc2)
        Plot.GraphAnnotation(plot, 0.5+0.125*3, acc3, acc3)
        Plot.GraphAnnotation(plot, 0.5-0.125*1, "-"+acc1, "-"+acc1)
        Plot.GraphAnnotation(plot, 0.5-0.125*2, "-"+acc2, "-"+acc2)
        Plot.GraphAnnotation(plot, 0.5-0.125*3, "-"+acc3, "-"+acc3)

    def add_altitude_plot_annotations(self, plot):
        self.max_altitude = 4
        self.max_velo_accel = 1
        altitude1 = str(self.max_altitude/4)+"m"
        altitude2 = str(2*self.max_altitude/4)+"m"
        altitude3 = str(3*self.max_altitude/4)+"m"
        velo_accel1 = str(self.max_velo_accel/4)+"m/s^x"
        velo_accel2 = str(2*self.max_velo_accel/4)+"m/s^x"
        velo_accel3 = str(3*self.max_velo_accel/4)+"m/s^x"
        Plot.GraphAnnotation(plot, 0.5, "0°", "0°")
        Plot.GraphAnnotation(plot, 0.5+0.125*1, altitude1, velo_accel1)
        Plot.GraphAnnotation(plot, 0.5+0.125*2, altitude2, velo_accel2)
        Plot.GraphAnnotation(plot, 0.5+0.125*3, altitude3, velo_accel3)
        Plot.GraphAnnotation(plot, 0.5-0.125*1, "-"+altitude1, "-"+velo_accel1)
        Plot.GraphAnnotation(plot, 0.5-0.125*2, "-"+altitude2, "-"+velo_accel2)
        Plot.GraphAnnotation(plot, 0.5-0.125*3, "-"+altitude3, "-"+velo_accel3)

    def redraw(self):
        self.x_plot.redraw()
        self.y_plot.redraw()
        self.z_plot.redraw()
        self.throttle_plot.redraw()

    def record_x_angle(self, angle):
        self.x_angle_graph.record_value(self.scale_angle(angle))
        
    def record_y_angle(self, angle):
        self.y_angle_graph.record_value(self.scale_angle(angle))

    def record_z_angle(self, angle):
        if False:
            self.z_angle_graph.record_value(self.scale_angle(angle))

    def record_x_av(self, av):
        self.x_angle_v_graph.record_value(self.scale_angle_v(av))
        
    def record_y_av(self, av):
        self.y_angle_v_graph.record_value(self.scale_angle_v(av))

    def record_z_av(self, av):
        self.z_angle_v_graph.record_value(self.scale_angle_v(av))

    def record_x_target(self, av):
        self.x_target_graph.record_value(self.scale_angle_v(av))
        
    def record_y_target(self, av):
        self.y_target_graph.record_value(self.scale_angle_v(av))

    def record_z_target(self, av):
        self.z_target_graph.record_value(self.scale_angle_v(av))

    def record_x_pid(self, pid):
        self.x_pid_graph.record_value(self.scale_pid(pid))

    def record_y_pid(self, pid):
        self.y_pid_graph.record_value(self.scale_pid(pid))
    
    def record_z_pid(self, pid):
        self.z_pid_graph.record_value(self.scale_pid(pid))

    def record_lf_throttle(self, value):
        self.lf_graph.record_value(self.scale_throttle(value))

    def record_rf_throttle(self, value):
        self.rf_graph.record_value(self.scale_throttle(value))

    def record_lr_throttle(self, value):
        self.lr_graph.record_value(self.scale_throttle(value))

    def record_rr_throttle(self, value):
        self.rr_graph.record_value(self.scale_throttle(value))

    def record_altitude(self, value):
        self.altitute_graph.record_value(self.scale_altitude(value))

    def record_vertical_velocity(self, value):
        self.vertical_velocity_graph.record_value(self.scale_velocity(value))

    def record_vertical_acceleration(self, value):
        self.vertical_acceleration_graph.record_value(self.scale_acceleration(value))
    
    def scale_angle_v(self, value):
        r = value*(180/3.14)/(self.max_angle) + 0.5
        if r > 1:
            r = 1
        if r < 0:
            r = 0
        return  r

    def scale_angle(self, value):
        r = value/(self.max_angle*2) + 0.5
        if r > 1:
            r = 1
        if r < 0:
            r = 0
        return  r

    def scale_pid(self, value):
        return value + 0.5

    def scale_throttle(self, value):
        return value

    def scale_altitude(self, value):
        # +-2 m per second is max
        r = value/(self.max_altitude * 2) + 0.5
        if r > 1:
            r = 1
        if r < 0:
            r = 0
        return  r

    def scale_velocity(self, value):
        # +-2 m per second is max
        r = value/(self.max_velo_accel * 2) + 0.5
        if r > 1:
            r = 1
        if r < 0:
            r = 0
        return  r

    def scale_acceleration(self, value):
        # +-2 m per second is max
        r = value/(self.max_velo_accel * 2) + 0.5
        if r > 1:
            r = 1
        if r < 0:
            r = 0
        return  r