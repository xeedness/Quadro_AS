import Plot

class PlotHandler():
    def __init__(self, plot):
        self.plot = plot
        self.z_angle_graph = Plot.Graph(plot, "ZAngle", "blue")
        self.y_angle_graph = Plot.Graph(plot, "YAngle", "red")
        self.x_angle_graph = Plot.Graph(plot, "XAngle", "green")
        self.annotation0 = Plot.GraphAnnotation(plot, 0.5, "0°", "0°")
        self.annotation15 = Plot.GraphAnnotation(plot, 0.5+0.125*1, "15°", "15°")
        self.annotation30 = Plot.GraphAnnotation(plot, 0.5+0.125*2, "30°", "30°")
        self.annotation45 = Plot.GraphAnnotation(plot, 0.5+0.125*3, "45°", "45°")
        self.annotationM15 = Plot.GraphAnnotation(plot, 0.5-0.125*1, "-15°", "-15°")
        self.annotationM30 = Plot.GraphAnnotation(plot, 0.5-0.125*2, "-30°", "-30°")
        self.annotationM45 = Plot.GraphAnnotation(plot, 0.5-0.125*3, "-45°", "-45°")

    def redraw(self):
        self.plot.redraw()

    def record_x_angle(self, angle):
        self.x_angle_graph.record_value(angle)
        
    def record_y_angle(self, angle):
        self.y_angle_graph.record_value(angle)

    def record_z_angle(self, angle):
        self.z_angle_graph.record_value(angle)