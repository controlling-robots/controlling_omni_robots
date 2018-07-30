from numpy import sqrt, sin, cos, pi


class CircleControlLaw:

    k = 2

    def __init__(self, R, v):
        """
        Constructor of Circle control law class.
        :param R:
        :param v:
        """
        self.R = R
        self.v = v

    def getControlCircle(self, x, y, alpha, center=(0, 0)):
        """
        Returns control actions for axis of x and y velocities
        for Circle trajectory.
        :param x: Current x-axis robot position.
        :param y: Current y-axis robot position.
        :param alpha: Current rotation robot relative base frame.
        :param center: Coordinates of center circle trajectory.
        :return: ux, uy, e - control for x, y; and error
        """
        x0, y0 = center[0], center[1]
        e = (x - x0) ** 2 - self.R ** 2 + (y - y0) ** 2
        denom = 2 * (x ** 2 - 2 * x * x0 + x0 ** 2 + y ** 2 - 2 * y * y0 + y0 ** 2)
        us_x = - self.v * (x * sin(alpha) - x0 * sin(alpha) - y * cos(alpha) + y0 * cos(alpha)) / denom
        us_y = - self.v * (x * cos(alpha) - x0 * cos(alpha) + y * sin(alpha) - y0 * sin(alpha)) / denom
        ue_x = -self.k * (2 * x - 2 * x0) * e
        ue_y = -self.k * (2 * y - 2 * y0) * e
        ux = us_x + ue_x
        uy = us_y + ue_y
        return ux, uy, e


class LineControlLaw:

    k = 2

    def __init__(self, v, beta, phi):
        """
        Constructor of Line control law class.
        :param v: Desired velocity.
        :param beta: Desired an angle of inclination trajectory
        to Y-axis of base frame.
        :param phi: Offset for Y-axis of base frame.
        """
        self.v = v
        self.beta = beta
        self.phi = phi

    def getControlLine(self, x, y, alpha):
        """
        Returns control actions for axis of x and y velocities
        for Line trajectory.
        :param x: Current x-axis robot position.
        :param y: Current y-axis robot position.
        :param alpha: Current rotation robot relative base frame.
        :return: ux, uy, e - control for x, y; and error
        """
        # beta, phi = +pi/2 + pi/8, 1
        e = self.phi - x * sin(self.beta) + y * cos(self.beta)
        denom = cos(self.beta)**2 + sin(self.beta)**2
        us_x = self.v * (cos(self.beta) * cos(alpha) / denom + sin(self.beta) * sin(alpha) / denom)
        us_y = -self.v * (cos(self.beta) * sin(alpha) / denom - cos(alpha) * sin(self.beta) / denom)
        ue_x = self.k * sin(self.beta) * e
        ue_y = -self.k * cos(self.beta) * e
        u_x = us_x + ue_x
        u_y = us_y + ue_y
        return u_x, u_y, e


class SinControlLaw:

    k = 2

    def __init__(self, A, v):
        """
        Constructor of Sin control law class.
        :param A: The amlitude of sinus curve.
        :param v: Desired velocity.
        """
        self.A = A
        self.v = v

    def getControlSin(self, x, y, alpha):
        """
        Returns control actions for axis of x and y velocities
        for Sin trajectory.
        :param x: Current x-axis robot position.
        :param y: Current y-axis robot position.
        :param alpha: Current rotation robot relative base frame.
        :return: ux, uy, e - control for x, y; and error
        """
        e = y - self.A * sin(self.k * x)
        denom = self.A**2 * self.k**2 * cos(self.k * x)**2 + 1
        us_x = self.v * (cos(alpha) / denom + (self.A * self.k * cos(self.k * x) * sin(alpha)) / denom)
        us_y = -self.v * (sin(alpha) / denom - (self.A * self.k * cos(self.k * x) * cos(alpha)) / denom)
        ue_x = self.A * self.k**2 * cos(self.k * x) * e
        ue_y = - self.k * (y - self.A * sin(self.k * x))
        u_x = us_x + ue_x
        u_y = us_y + ue_y
        return u_x, u_y, e
