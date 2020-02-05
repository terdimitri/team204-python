from math import sqrt, sin, cos, atan2, pi, tau, inf, isfinite, nan


class Projective2d:
    def __init__(self, r, theta, cartesian=False):
        if cartesian:
            self._r, self._theta = 0, 0
            self.x, self.y = r, theta
        else:
            self._r = r
            self._theta = theta
            self._reproc()

    def _reproc(self):
        """Change the argument and magnitude so that the argument is between
        0 and tau and the magnitude is positive."""
        if self._r < 0:
            self._theta += 180
        self._theta = self._theta % tau
        self._r = abs(self._r)

    @property
    def magnitude(self):
        return self._r
    @magnitude.setter
    def magnitude(self, r):
        self._r = r
        self._reproc()

    @property
    def argument(self):
        return self._theta
    @argument.setter
    def argument(self, theta):
        self._theta = theta
        self._reproc()

    @property
    def x(self):
        """The x component of the vector"""
        return self._r * cos(self._theta)
    @x.setter
    def x(self, new_x):
        old_y = self.y
        self._theta = atan2(old_y, new_x)
        self._r = sqrt(new_x**2 + old_y**2)
        self._reproc()

    @property
    def y(self):
        """The y component of the vector"""
        return self._r * sin(self._theta)
    @y.setter
    def y(self, new_y):
        old_x = self.x
        self._theta = atan2(new_y, old_x)
        self._r = sqrt(old_x**2 + new_y**2)
        self._reproc()

    def __rmul__(self, scalar):
        return Projective2d(self._r * scalar, self._theta)
    def __imul__(self, scalar):
        self.r *= scalar
        self._reproc()

    def __add__(self, other):
        out = Projective2d(0, 0)
        out.x, out.y = self.x + other.x, self.y + other.y
        return out
    def __iadd__(self, other):
        self.x += other.x
        self.y += other.y

    def __sub__(self, other):
        out = Projective2d(0, 0)
        out.x, out.y = self.x - other.x, self.y - other.y
        return out
    def __isub(self, other):
        self.x -= other.x
        self.y -= other.y

    def __repr__(self):
        return f'Projective2d({self._r}, {self._theta})'


class Matrix2d:
    def __init__(self, left_vector, right_vector):
        self.left_vector = left_vector
        self.right_vector = right_vector

    def __matmul__(self, other):
        if (isinstance(other, Matrix2d)):
            return  Matrix2d(self@other.left_vector, self@other.right_vector)
        else:
            return other.x * self.left_vector + other.y * self.right_vector

