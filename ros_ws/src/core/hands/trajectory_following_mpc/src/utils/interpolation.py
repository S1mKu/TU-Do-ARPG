from cmath import sqrt
from hashlib import new
from operator import length_hint
from scipy import interpolate
import math
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Polygon, Point32

class Interpolation:

    #Weiß nicht ob das bei Aufrufen von außerhalb funktioniert mit globalen Variablen
    lengthOfCurve = 0
    curve = {}
    v_length = 0
    velocity_f = {}
    last_s = 0

    def getBorderPolygons(self, sampling, min_width):
        poly_outer = Polygon()
        poly_inner = Polygon()
        for i in range(sampling):
            s = i* (self.lengthOfCurve / sampling)
            XY = self.evaluateInterpolation(s)
            norm = self.getNormalVectorOfHeadingSingle(s)
            roadside_inner_point = Point32()
            roadside_inner_point.x = XY[0]-(norm[0]* min_width/2)
            roadside_inner_point.y = XY[1]-(norm[1]* min_width/2)
            roadside_inner_point.z = 0.0
            poly_inner.points.append(roadside_inner_point)

            roadside_outer_point = Point32()
            roadside_outer_point.x = XY[0]+(norm[0]* min_width/2)
            roadside_outer_point.y = XY[1]+(norm[1]* min_width/2)
            roadside_outer_point.z = 0.0
            poly_outer.points.append(roadside_outer_point)

        return poly_outer, poly_inner

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

    def interpolateVelocities(self, s_array, velocity_array):
        f = interpolate.interp1d(s_array, velocity_array)
        self.v_length = s_array[len(s_array)-1]
        self.velocity_f = f

    def evaluateVelocity(self, s):
        s = s % self.v_length
        return self.velocity_f(s)

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



    # # Testet einmal alle Funktionen
    # # Polygonzug geht im Kreis mit Radius 1 um den Ursprung, gegen den Uhrzeigersinn von (1,0)
    # def testEverything(self):

    #     # testen mit 10, 100 oder 1000 Punkten
    #     #t = np.arange(0, 1.1, .01)
    #     #t = np.arange(0, 1.01, .01)
    #     t = np.arange(0, 1.001, .001)
    #     x = np.cos(2*np.pi*t)
    #     y = np.sin(2*np.pi*t)
    #     #tck, u = interpolate.splprep([x, y], s=0)
    #     polygon = [x,y]
    #     #print("polygon = ", polygon)
    #     tck = self.interpolatePolygon(polygon)
    #     unew = np.arange(0, 1.01, 0.01)
    #     out = self.evaluateInterpolation(tck, unew)
    #     #out = interpolate.splev(unew, tck)

    #     test = self.evaluateInterpolation(tck, [0, np.pi, 2*np.pi])

    #     testSingle = np.pi
    #     testMultiple = [0, np.pi/2, np.pi*3/2]
    #     #for i in range(77, 154):
    #         #print("testEval ", i, evaluateInterpolation(tck, i/100))
    #         #print("testDistance ", i/100, calculateDistance(evaluateInterpolation(tck, i/100), [0,1]))



    #     print("Testing everything with single = ", testSingle, " and Multiple = ", testMultiple,)
    #     print()

    #     #print("EvalutaionSingle0 = ", evaluateInterpolation(tck, 0))
    #     print("EvalutaionSingle = ", self.evaluateInterpolation(tck, testSingle))
    #     print("EvalutaionMultiple = ", self.evaluateInterpolation(tck, testMultiple))
    #     print()


    #     #print("derivTest = ", interpolate.spalde(0, tck)) 
    #     print("HeadingSingle = ", self.getHeadingOfCurveSingle(tck, testSingle))
    #     print("HeadingMultiple = ", self.getHeadingOfCurveMultiple(tck, testMultiple))
    #     print()

    #     print("NormalVectorOfHeadingSingle = ", self.getNormalVectorOfHeadingSingle(tck, testSingle))
    #     print("NormalVectorOfHeadingMultiple = ", self.getNormalVectorOfHeadingMultiple(tck, testMultiple))
    #     print()

    #     # Curvature wird negativ ausgeben?
    #     # Gibt nicht kappa sondern direkt 1/kappa also Radius des Kreises an?
    #     print("CurvatureSingle = ", getLocalCurvatureOfCurveSingle(tck, testSingle))
    #     print("CurvatureMultiple = ", getLocalCurvatureOfCurveMultiple(tck, testMultiple))
    #     print()

    #     testStepsize =  0.01
    #     testLastS = 0.7
    #     testCurrentPosition1 = [1, 1]
    #     testCurrentPosition2 = [0, 1]
    #     testCurrentPosition3 = [-1, 1]
    #     testCurrentPosition4 = [-1, 0]
    #     projection1 = projectOntoCurve(tck, testLastS, testCurrentPosition1, testStepsize)
    #     print("Projection1 onto ", testCurrentPosition1, " is ", evaluateInterpolation(tck, projection1), "with s=", projection1)
    #     print()
    #     projection2 = projectOntoCurve(tck, projection1, testCurrentPosition2, testStepsize)
    #     print("Projection2 onto ", testCurrentPosition2, " is ", evaluateInterpolation(tck, projection2), "with s=", projection2)
    #     print()
    #     projection3 = projectOntoCurve(tck, projection2, testCurrentPosition3, testStepsize)
    #     print("Projection3 onto ", testCurrentPosition3, " is ", evaluateInterpolation(tck, projection3), "with s=", projection3)
    #     print()
    #     projection4 = projectOntoCurve(tck, projection3, testCurrentPosition4, testStepsize)
    #     print("Projection4 onto ", testCurrentPosition4, " is ", evaluateInterpolation(tck, projection4), "with s=", projection4)





    #     plt.figure()
    #     plt.plot(x, y, 'x', out[0], out[1], np.sin(2*np.pi*unew), np.cos(2*np.pi*unew), x, y, 'b')
    #     plt.legend(['Linear', 'Cubic Spline', 'True'])
    #     plt.axis([-1.05, 1.05, -1.05, 1.05])
    #     plt.title('Spline of parametrically-defined curve')
    #     plt.show()

# if __name__ == '__main__':

    # interpolation = Interpolation()
    # interpolation.testEverything()


