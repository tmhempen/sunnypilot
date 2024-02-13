#!/usr/bin/env python
import csv
import time
import cereal.messaging as messaging
from cereal.messaging import *
from openpilot.common.realtime import Ratekeeper
from openpilot.common.numpy_fast import interp, clip
from openpilot.common.params import Params
import argparse
import os

#from selfdrive.locationd.calibrationd import *
#from tools.sim.lib.simulated_sensors import SimulatedSensors
import random


class datastream:   # need to refactor into multiple classes in the future
    def __init__(self, csv_filepath):
        """Set default values for when datastream object is created. Incr / decr only set to 1%"""
        #self.axis_increment = 0.01    # 1% incr / decr to adjusting speed
        self.axis_values = {'gb':0.1, 'steer': 0.}   # initialize gb to a small value for later on proportional calculation
        self.axes_order = ['gb', 'steer']
        self.csv_filepath = csv_filepath  
        self.control_sock = messaging.pub_sock('testJoystick')  # for sending gb value
        self.time_list = []
        self.speed_list = []
        self.csv_data = self.read_csv_data()

        # subscribe to get car real-time speed
        self.sm = messaging.SubMaster(['gpsLocationExternal'])
       
        # start time before communicating first car speed
        self.start_time = time.time() 

        # communicate the first speed to get the car moving
        # dat = new_message('testJoystick')
        # dat.testJoystick.axes = [self.axis_values[a] for a in self.axes_order]  # put all values in axis_values into a list
        # dat.testJoystick.buttons = [False]
        # self.control_sock.send(dat.to_bytes())  # convert message to bytes and send it over messaging socket


    def read_csv_data(self):
        """
        Read CSV data into a list of dictionaries for index access
        """
        # need to make sure of the unit of speed in the csv, whether mph or m/s
        data = []
        with open(self.csv_filepath, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                data.append(row)
                self.time_list.append(float(row['time']))
                self.speed_list.append(float(row['speed'])) 
        return data
    
    def write_data(self, filename, data_list):
        """
        Write debug info into file for future reference
        """
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Time', 'Calculated G1', 'Vr0', 'Vr1', 'Vt1', 'Vt2', 'Difference target - actual'])
            writer.writerows(data_list)


    def get_target_speed(self, elapsed_time):
        """
        Return target speed based on the elapsed time
        """
        return self.interp(elapsed_time, self.time_list, self.speed_list)  # interp return type should be a float
    

    def interp(self, x, xp, fp):
        """
        Interpolate helper function
        """
        N = len(xp)

        def get_interp(xv):
            hi = 0
            while hi < N and xv > xp[hi]:
                hi += 1
            low = hi - 1
            return fp[-1] if hi == N and xv > xp[low] else (
                fp[0] if hi == 0 else
                (xv - xp[low]) * (fp[hi] - fp[low]) / (xp[hi] - xp[low]) + fp[low])

        return [get_interp(v) for v in x] if hasattr(x, '__iter__') else get_interp(x)


    def get_actual_speed(self):
        """
        Return actual car speed in MPH
        """
        self.sm.update()
        gle = self.sm['gpsLocationExternal']
        #print("get_actual_speed returned", gle.speed * 2.236)  # gle.speed should be m/s so convert it to mph
        # gle.speed is in m/s but the speeds in the csv file are MPH, conversion factor is 2.236
        return gle.speed * 2.236
    

    def get_simulated_real_speed(self, target_speed):
            """
            Return a pretend real speed for testing
            """
            rand_num = random.randint(-100, 101)
            rand_num /= 10000
            rand_num += 1
            print("Adjustment factor:", rand_num)
            adjusted_speed = target_speed * rand_num
            print("Adjusted speed:", adjusted_speed)
            return adjusted_speed


    def control_speed(self, start_time):
        """
        Run a while loop until the last time in the CSV file is reached
        """
        end_time = self.time_list[-1]
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        # define sleep time (pause between iterations)
        interval = 0.02  # loop will run every 20 ms
        t0 = 0
        # get car actual speed
        Vr0 = self.get_actual_speed()
        if Vr0 == 0:   # if car is not moving, set gb to move
            g0 = 0.1
        else:
            g0 = 0
        t1 = elapsed_time

        while True:    
            if elapsed_time >= end_time:   # infinitely loop
                start_time = current_time
                elapsed_time = 0
                t1 = elapsed_time

            # calculate equation vars
            t2 = t1 + interval  # next time (next iteration), interval is an approximation (assume calculatiosn and communication time is tiny)
            # calculate target V
            Vt2 = self.get_target_speed(t2)
            
            # get current actual speed of car
            actual_speed = self.get_actual_speed()
            if actual_speed is not None:
                Vr1 = actual_speed
            else:
                Vr1 = Vr0

            # solve formula for g1
            if Vt2 == Vr1:   # check if next speed = current actual speed
                g1 = 0
            elif Vr1 == Vr0:
                if Vt2 > Vr1:
                    g1 = 0.1  # default gb
                elif Vt2 < Vr1:
                    g1 = -0.1
                else:
                    g1 = 0
            else:
                if g0 == 0:
                    if Vr1 > Vr0:
                        g0 = 0.1  # default gb
                    else:
                        g0 = -0.1

                g1 = g0 * (t1 - t0) * (Vt2 - Vr1) / (Vr1 - Vr0) / interval
            
            # clip value 
            if g1 > 1:
                g1 = 1
            elif g1 < -1:
                g1 = -1

            # set gb to newly calculated gas brake value
            self.axis_values['gb'] = g1

            # print("AT TIME:", t1)
            # print("calculated g1:", self.axis_values['gb'])
            # print("Vr0:", Vr0)
            # print("Vr1:", Vr1)
            # print("Vt1:", Vt1)
            # print("Vt2:", Vt2)
            # print("target - actual:", Vt1 - Vr1)
            # print()

            # send control signals to Openpilot for gas brake
            dat = new_message('testJoystick')
            dat.testJoystick.axes = [self.axis_values[a] for a in self.axes_order]  # put all values in axis_values into a list
            dat.testJoystick.buttons = [False]
            self.control_sock.send(dat.to_bytes())  # convert message to bytes and send it over messaging socket


            time.sleep(interval)  # adjustment occurs every 20ms, adjust if needed

            # calculate values for next loop
            current_time = time.time()
            elapsed_time = current_time - start_time
            g0 = g1
            t0 = t1
            Vr0 = Vr1
    
            t1 = elapsed_time
            #print("current time:", t1, "current speed:", Vr1, "next target speed:", Vt2)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Publishes events from your joystick to control your car.\n' +
                                                 'openpilot must be offroad before starting joysticked.',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--csv_file', help='CSV file containing time (s) and speed (mph)')
    args = parser.parse_args()# subscribe to get real-time info from openpilot
    

    if not Params().get_bool("IsOffroad") and "ZMQ" not in os.environ:
        print("The car must be off before running datastream.")
        exit()

    # create the data stream object
    ds = datastream(csv_filepath = args.csv_file)
    
    while True:
        ds.control_speed(ds.start_time)