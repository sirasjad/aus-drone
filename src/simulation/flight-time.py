import matplotlib.pyplot as plt

# Formula: Time = Capacity * Discharge / Average Amp Draw (AAD)
# AAD = Weight * P / V
class FlightTime:
    def __init__(self):
        self.capacity = 3.7      # Ah
        self.discharge = 0.8     # 80%
        self.power = 170         # W/kg
        self.weight = 3.0        # Kg
        self.voltage = 22        # V
        self.timeRes = []
        self.weightRes = []

    def calculateTime(self):
        print("Time (m):\t Weight (kg):\t Average Amp Draw\t")

        while (self.weight > 2.0):
            avg_amp = (self.weight * self.power) / self.voltage
            time = (self.capacity * self.discharge) / avg_amp
            time = time * 60

            print("%s\t\t %s\t\t %s\t" % (round(time, 2), round(self.weight, 2), round(avg_amp, 2)))

            self.timeRes.append(time)
            self.weightRes.append(self.weight)
            self.weight = self.weight - 0.1

    def plotGraph(self):
        plt.title("Drone flight time")
        plt.ylabel("Time (min)");
        plt.xlabel("Weight (kg)");
        plt.ylim(7.6, 11)
        plt.xlim(2.1, 3)
        plt.plot(self.weightRes, self.timeRes)
        plt.show()

ft = FlightTime()
ft.calculateTime()
ft.plotGraph()