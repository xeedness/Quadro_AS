from tkinter import Tk, Canvas, Frame, BOTH

PLOT_POINTS = 200
HEIGHT = 768
WIDTH = 1280
ANNOTATION_COLOR = "#AAAAAA"

class Plot(Frame):
    def __init__(self, root):
        super().__init__()
        self.root = root
        self.initUI()

        

    def initUI(self):
        self.master.title("Lines")
        self.pack(fill=BOTH, expand=1)
        self.canvas = Canvas(self)
        self.canvas.pack(fill=BOTH, expand=1)
        self.root.update()
        self.graphs = []
        self.graph_names = []
        self.graph_annotations = []

    def get_point(self, x, y):
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()

        return (x*width, height - y*height)

    def add_graph(self, graph):
        self.graphs.append(graph)
        self.graph_names.append(self.canvas.create_text(100,100, anchor="se", fill=graph.color, text=graph.name))
        self.redraw()

    def add_graph_annotation(self, graph_annotation):
        self.graph_annotations.append(graph_annotation)

    def redraw(self):
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()

        for graph in self.graphs:
            graph.redraw()

        for graph_annotation in self.graph_annotations:
            graph_annotation.redraw()

        for i in range(0, len(self.graph_names)):
            graph_name = self.graph_names[i]
            self.canvas.coords(graph_name, width-10, height-10-i*20)

        




class Graph():
    def __init__(self, plot, name, color):
        super().__init__()
        self.color = color
        self.name = name
        self.plot = plot
        self.initData()

        self.plot.add_graph(self)
        
    def initData(self):
        self.last_index = 0
        self.values = []
        self.points = []
        self.lines = []

        for i in range(0, PLOT_POINTS):
            self.values.append(i/PLOT_POINTS)

        for i in range(0, PLOT_POINTS):
            self.points.append(self.plot.get_point(i/PLOT_POINTS, self.values[i]))

        for i in range(0, PLOT_POINTS-1):
            self.lines.append(self.plot.canvas.create_line(self.points[i], self.points[i+1], width=1, fill=self.color))

    def scale_value(self, value):
        return value/120 + 0.5

    def record_value(self, value): 
        new_index = (self.last_index+1) % PLOT_POINTS
        self.values[new_index] = scaled_value = self.scale_value(value)
        self.points[new_index] = self.plot.get_point(new_index/PLOT_POINTS, scaled_value)  
        
        if(new_index > 0):
            p1 = self.points[self.last_index]
            p2 = self.points[new_index]

            self.plot.canvas.coords(self.lines[self.last_index], p1[0], p1[1], p2[0], p2[1])

        self.last_index = new_index

    def redraw(self):
        for i in range(0, PLOT_POINTS):
            self.points[i] = self.plot.get_point(i/PLOT_POINTS, self.values[i])

        for i in range(0, PLOT_POINTS-1):
            self.plot.canvas.coords(self.lines[i], self.points[i][0], self.points[i][1], self.points[i+1][0], self.points[i+1][1])

class GraphAnnotation():
    def __init__(self, plot, y_pos, left_annotation_text, right_annotation_text):
        super().__init__()
        self.plot = plot
        self.y_pos = y_pos
        self.left_annotation_text = left_annotation_text
        self.right_annotation_text = right_annotation_text

        self.initUI()
        self.plot.add_graph_annotation(self)
        

    def initUI(self):
        p1 = self.plot.get_point(0, self.y_pos)
        p2 = self.plot.get_point(1, self.y_pos)
        self.line = self.plot.canvas.create_line(p1, p2, width=1, fill=ANNOTATION_COLOR)
        self.left_annotation = self.plot.canvas.create_text(p1[0]+5, p1[1], anchor="sw", fill=ANNOTATION_COLOR, text=self.left_annotation_text)
        self.right_annotation = self.plot.canvas.create_text(p2[0]-5, p2[1], anchor="se", fill=ANNOTATION_COLOR, text=self.right_annotation_text)

    def redraw(self):
        p1 = self.plot.get_point(0, self.y_pos)
        p2 = self.plot.get_point(1, self.y_pos)
        self.plot.canvas.coords(self.line, p1[0], p1[1], p2[0], p2[1])
        self.plot.canvas.coords(self.left_annotation, p1[0]+5, p1[1])
        self.plot.canvas.coords(self.right_annotation, p2[0]-5, p2[1])

        

        