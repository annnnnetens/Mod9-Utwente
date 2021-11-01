class Biquad:
    def __init__(self, lis):
        self.a1 = lis[0]
        self.a2 = lis[1]
        self.b0 = lis[2]
        self.b1 = lis[3]
        self.b2 = lis[4]
        self.w1 = 0.0
        self.w2 = 0.0

    def filter(self, x):
        y = float(self.b0*x + self.w1)
        self.w1 = float(self.b1*x - self.a1*y + self.w2)
        self.w2 = float(self.b2*x - self.a2*y)
        return y

    