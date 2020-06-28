import Plot

class PlotHandler():
    def __init__(self, x_plot, y_plot, z_plot, throttle_plot):
        self.x_plot = x_plot
        self.y_plot = y_plot
        self.z_plot = z_plot
        self.throttle_plot = throttle_plot

        self.add_angle_plot_annotations(x_plot)
        self.add_angle_plot_annotations(y_plot)
        self.add_angle_plot_annotations(z_plot)
        self.add_throttle_plot_annotations(throttle_plot)

        self.z_angle_graph = Plot.Graph(z_plot, "ZAngle", "red")

        self.y_angle_graph = Plot.Graph(y_plot, "YAngle", "red")
        self.y_pid_graph = Plot.Graph(y_plot, "YPID", "blue")

        self.x_angle_graph = Plot.Graph(x_plot, "XAngle", "red")
        self.x_pid_graph = Plot.Graph(x_plot, "XPID", "blue")

        self.lf_graph = Plot.Graph(throttle_plot, "left front", "green")
        self.rf_graph = Plot.Graph(throttle_plot, "right front", "red")
        self.lr_graph = Plot.Graph(throttle_plot, "left rear", "blue")
        self.rr_graph = Plot.Graph(throttle_plot, "right rear", "orange")

        


    def add_angle_plot_annotations(self, plot):
        Plot.GraphAnnotation(plot, 0.5, "0°", "0°")
        Plot.GraphAnnotation(plot, 0.5+0.125*1, "15°", "15°")
        Plot.GraphAnnotation(plot, 0.5+0.125*2, "30°", "30°")
        Plot.GraphAnnotation(plot, 0.5+0.125*3, "45°", "45°")
        Plot.GraphAnnotation(plot, 0.5-0.125*1, "-15°", "-15°")
        Plot.GraphAnnotation(plot, 0.5-0.125*2, "-30°", "-30°")
        Plot.GraphAnnotation(plot, 0.5-0.125*3, "-45°", "-45°")

    def add_throttle_plot_annotations(self, plot):
        Plot.GraphAnnotation(plot, 0.5, "0", "0")

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
        self.z_angle_graph.record_value(self.scale_angle(angle))

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

    def record_lr_throttle(self, pivalued):
        self.lr_graph.record_value(self.scale_throttle(value))

    def record_rr_throttle(self, value):
        self.rr_graph.record_value(self.scale_throttle(value))
        
    def scale_angle(self, value):
        return value/120 + 0.5

    def scale_pid(self, value):
        return value/120 + 0.5

    def scale_throttle(self, value):
        return value/120 + 0.5