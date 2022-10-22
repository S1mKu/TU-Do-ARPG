from cmath import sqrt
from hashlib import new
from operator import length_hint
from scipy import interpolate
import math
import numpy as np
import matplotlib.pyplot as plt

class Interpolation:

    #Weiß nicht ob das bei Aufrufen von außerhalb funktioniert mit globalen Variablen
    lengthOfCurve = 0
    curve = {}
    last_s = 0

    #Berechne Interpolation mit arclength annäherungsweise der Streckenlänge
    def interpolatePolygon(self, polygon):
        #Berechne Abstände der Punkte für die Arclength annäherungsweise
        u = [0]
        for i in range(1, len(polygon[0])):
            u.append(u[i-1] + sqrt((polygon[0][i] - polygon[0][i-1])**2 + (polygon[1][i] - polygon[1][i-1])**2).real)
        self.lengthOfCurve = u[len(u)-1]

        #s gibt smoothing an, erstmal ohne für performance, könnte man aber noch hochschrauben
        tck_ret, u = interpolate.splprep(polygon, s=1, u=u)
        self.curve = tck_ret
        # return tck_ret

    #Berechne Punkte auf der Kurve, auch mit Array von Punkten möglich
    def evaluateInterpolation(self, s):
        s = s % self.lengthOfCurve
        eval_ret = interpolate.splev(s, self.curve)
        return eval_ret

    #Berechne local curvature
    def getLocalCurvatureOfCurveSingle(self, s):
        s = s % self.lengthOfCurve
        deriv = interpolate.spalde(s, self.curve)

        #[x oder y][Grad der Ableitung]
        vx = deriv[0][1]
        vy = deriv[1][1]
        ax = deriv[0][2]
        ay = deriv[1][2]
        a = np.array([[vx, ax], [vy, ay]])
        kappa = np.linalg.det(a)
        return kappa

    def getLocalCurvatureOfCurveSingleStep(self, k , s_step):
        s = self.last_s + k * s_step
        s = s % self.lengthOfCurve
        deriv = interpolate.spalde(s, self.curve)

        #[x oder y][Grad der Ableitung]
        vx = deriv[0][1]
        vy = deriv[1][1]
        ax = deriv[0][2]
        ay = deriv[1][2]
        a = np.array([[vx, ax], [vy, ay]])
        kappa = np.linalg.det(a)
        return kappa

    def getLocalCurvatureOfCurveMultiple(self, s):
        for i in range(0, len(s)):
            s[i] = s[i] % self.lengthOfCurve
        deriv = interpolate.spalde(s, self.curve)
        kappa = []
        for i in range(0, len(deriv[0])):
            #[x oder y][Index des Punktes][Grad der Ableitung]
            vx = deriv[0][i][1]
            vy = deriv[1][i][1]
            ax = deriv[0][i][2]
            ay = deriv[1][i][2]
            a = np.array([[vx, ax], [vy, ay]])
            kappa.append(np.linalg.det(a))
        return kappa



    def getHeadingOfCurveMultiple(self, s):
        for i in range(0, len(s)):
            s[i] = s[i] % self.lengthOfCurve
        deriv = interpolate.spalde(s, self.curve)
        psi = []
        for i in range(0, len(deriv[0])):
            #[x oder y][Index des Punktes][Grad der Ableitung]
            psi.append(math.atan2(deriv[1][i][1], deriv[0][i][1])) 
        return psi


    def getHeadingOfCurveSingle(self, s):
        s = s % self.lengthOfCurve
        deriv = interpolate.spalde(s, self.curve)
        #[x oder y][Grad der Ableitung]
        psi = math.atan2(deriv[1][1], deriv[0][1])
        return psi

    def getNormalVectorOfHeadingSingle(self, s):
        s = s % self.lengthOfCurve
        deriv = interpolate.spalde(s, self.curve)
        #[x oder y][Grad der Ableitung]
        return [-deriv[1][1],deriv[0][1]]

    def getNormalVectorOfHeadingMultiple(self, s):
        for i in range(0, len(s)):
            s[i] = s[i] % self.lengthOfCurve
        deriv = interpolate.spalde(s, self.curve)
        normals = []
        for i in range(0, len(deriv[0])):
            #[x oder y][Index des Punktes][Grad der Ableitung]
            normals.append([-deriv[1][i][1],deriv[0][i][1]])  
        return normals


    # Approximiere Projektion auf Kurve auf Basis der letzten Position
    # Starte von letzter bekannten Projektion (müsste vom Aufrufer gespeichert werden, ansonsten nochmal hierdrin)
    # Suche nach neuer Minimaler Distanz mit Schritten entlang der Kurve
    # Sollte bei kleinen Schritten und genug hoher Frequenz gut genug approximieren
    # Idee ist dass die nächste Projektion auf die Kurve kurz nach der aktuellen Projketion auf der Kurve liegen muss
    # und die beiden Projektionen nah genug aneinander liegen dass dazwischen keine lokalen Mininma liegen
    def projectOntoCurve(self, lastS, currentPosition, stepSize):
        lastProjection = self.evaluateInterpolation(lastS)
        oldDistance = self.calculateDistance(currentPosition, lastProjection)
        currentS = lastS
        newDistance = oldDistance
        while newDistance <= oldDistance:
            oldDistance = newDistance
            currentS += stepSize
            currentS = currentS % self.lengthOfCurve # do laps
            newProjection = self.evaluateInterpolation(currentS)
            newDistance = self.calculateDistance(newProjection, currentPosition)
        currentS -= stepSize
        if currentS < 0:
            currentS = 0
        return currentS


    def calculateDistance(self, pos1, pos2):
        return sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2).real


    # Berechne/aproximiere die distanz eines punktes zu der kurve an der stelle s mit richtung des noralenvektors
    def calculateNormDistance(self, XY, S):
        XYS = self.evaluateInterpolation(S)
        dist = self.calculateDistance(XY, XYS)
        norm_vec = self.getNormalVectorOfHeadingSingle(S)
        # get vector from XYS->XY
        SXY_vec = np.subtract(np.array(XY), np.array(XYS))
        # https://stackoverflow.com/questions/49535295/how-to-check-if-vectors-are-facing-same-direction
        dot_product = np.dot(norm_vec, SXY_vec)
        if dot_product < 0:
            dist = -dist
        return dist

    def calculateNormDistance_alt(self, XY, S, s_step):
        XYS = self.evaluateInterpolation(S)
        dist = self.calculateDistance(XY, XYS)
        # https://stackoverflow.com/questions/1560492/how-to-tell-whether-a-point-is-to-the-right-or-left-side-of-a-line
        # To tell if a point is on the right or the left of a straight evaluate the crossproduct, if it is on left the 
        # 3rd component will be positive (right hand rule). 
        a = self.evaluateInterpolation(S)
        b = self.evaluateInterpolation(S + s_step) # make small step to approximate direction of curve
        # TODO could probably also be done with heading of curve but cant figure out currently how
        isLeft = ((b[0] - a[0])*(XY[1] - a[1]) - (b[1] - a[1])*(XY[0] - a[0])) > 0
        if not isLeft:
            dist = -dist
        return dist


    # Geht die gesamte kurve ab und sucht nach der kleinsten distanz zur kurve um das initiale s rauszufinden
    def getInitialS(self, currentPosition, stepSize):
        S = 0
        projection = self.evaluateInterpolation(S)
        distance = self.calculateDistance(currentPosition, projection)
        minS = S
        minDistance = distance
        while S < self.lengthOfCurve:
            S += stepSize
            projection = self.evaluateInterpolation(S)
            distance = self.calculateDistance(currentPosition, projection)
            if distance < minDistance:
                minS = S
                minDistance = distance
            lastS = S
        return minS



