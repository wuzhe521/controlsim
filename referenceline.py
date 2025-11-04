import numpy as np
import matplotlib.pyplot as plt

Safe_distance = 0.5

class Point:
    def __init__(self, x, y, dy, ddy):
        self.x = x
        self.y = y
        self.dy = dy
        self.ddy = ddy
        
        
class Line:
    def __init__(self, c0:float, c1:float, c2:float, c3:float):
        self.c0 = c0
        self.c1 = c1
        self.c2 = c2
        self.c3 = c3

class Ref_line:
    def __init__(self, name : str):
        self.name = name
        self.C0 = 0.0
        self.C1 = 0.0
        self.C2 = 0.0
        self.C3 = 0.0
        self.x = []
        self.y = []
        self.dy = []
        self.ddy = []
        
    def calc_reference_line(self, start_point:Point, end_point:Point, dx:float):
        c0 = start_point.y
        c1 = start_point.dy
        c2 = start_point.ddy
        