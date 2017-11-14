#!/usr/bin/env python

import csv
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline
import numpy as np

class Callibration:

    dist  = None
    ir_val = None
    spline_func = None
    poly_func = None



    def __init__(self):

        distance = []
        ir_reading = []

        with open('ir_values_ed.csv') as csvDataFile:
            csvReader = csv.reader(csvDataFile)
            for row in csvReader:
                distance.append(row[0])
                ir_reading.append(row[1])

        ir_reading.reverse()
        distance.reverse()
        # import pdb; pdb.set_trace()
        self.dist = np.array(distance[1:len(distance) -1]).astype(np.float)
        self.ir_val = np.array(ir_reading[1:len(ir_reading) -1]).astype(np.float)
        self.ir_val_inv = 1 / self.ir_val
        print(self.ir_val)
        print(self.dist)
        z = np.polyfit(self.ir_val_inv, self.dist, 2)
        print(z)
        self.poly_func = np.poly1d(z)


        self.spline_func = UnivariateSpline(self.ir_val, self.dist, s=30)



    def get_distance(self, graph_type, ir_value):

        if graph_type == 'poly':
            return self.poly_func(ir_value)

        elif graph_type == "spline":

            return self.spline_func(ir_value)

        else:

            #throw error
            print("not valid graph type")
            return null


    def plot_graph(self, graph_type):

        if graph_type == 'poly':

            poly_x = np.linspace(50, 600, 300)
            poly_y = self.poly_func(1/poly_x)

            plt.plot(self.ir_val, self.dist, "o", poly_x, poly_y)
            #plt.plot(poly_x, poly_y)
            plt.show()
            return


        elif graph_type == "spline":

            spline_x = np.linspace(0, 600, 600)
            spline_y = self.spline_func(spline_x)

            plt.plot(self.ir_val, self.dist, "o")
            plt.plot(spline_x, spline_y)
            plt.show()
            return


cal = Callibration()

print cal.get_distance("spline", 60)

cal.plot_graph("poly")