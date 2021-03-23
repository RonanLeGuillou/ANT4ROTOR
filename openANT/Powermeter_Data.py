# Copyright (c) 2020, Martin Schmoll <martin.schmoll@meduniwien.ac.at>
#
# This class represents a simple data container to share the received data with other modules in our software solution.
#
# The software comes as it is, free of charge and is intended exclusively for non commercial use.
# The author of this software does not take responsibilty for potential malfunctions. Feel free
# to use, modify and extend the software to your personal needs.
#
# Attention: Please keep in mind that there is a 200ms delay in the data!

class Powermeter_Data(object):

    def __init__(self):

        self.BatteryStatus = -1

        self.DataRate_F1 = 0
        self.DataRate_F2 = 0
        self.DataRate_F3 = 0

        self.OCA = 0.0

        self.Force_Left = 0.0
        self.Force_Right = 0.0
        self.Force_Total = 0.0

        self.Torque_Left = 0.0
        self.Torque_Right = 0.0
        self.Torque_Total = 0.0

        self.Power = 0.0

        self.CrankAngle = 0.0

        self.Cadence = 0.0

        self.Balance_Left = 0.0
        self.Balance_Right = 0.0

        self.Torque_Efficiency_Left = 0.0
        self.Torque_Efficiency_Right =  0.0

        self.Pedal_Smoothness_Left = 0.0
        self.Pedal_Smoothness_Right = 0.0