import numpy as np
import math

def pt_to_pt_distance(pt1, pt2):
    distance = np.sqrt((pt2[0] - pt1[0]) ** 2 + (pt2[1] - pt1[1]) ** 2)
    return distance


def normalize_rad(rad: float):
    return (rad + np.pi) % (2 * np.pi) - np.pi


class pure_pursuit_test_claude():
    def __init__(self, path):
        self.path = path
        self.theta = 0
        self.alpha = 0
        self.theta_0 = 0.03
        self.alpha_0 = 0.03
        self.sol_pt1 = 0
        self.sol_pt2 = 0

    def sgn(self, x):
        if x < 0:
            return -1
        else:
            return 1

    # ... (other methods remain the same)
    def calculate_alpha_theta(self, currentPos, currentHeading, goalPt, sol_pt1, sol_pt2):
        # Calculate alpha (steering angle)
        goal_heading = np.arctan2(goalPt[1] - currentPos[1], goalPt[0] - currentPos[0])
        # print("goal_heading" + str(goal_heading))
        self.alpha = abs(normalize_rad(goal_heading - currentHeading))

        # Calculate theta (deflection angle)
        if len(self.path) > 1:
            path_heading = np.arctan2(sol_pt1[1] - sol_pt2[1], sol_pt1[0] - sol_pt2[0])
            # print("path_H" + str(path_heading))
            self.theta = abs(normalize_rad(path_heading - goal_heading))

        # print("theta " + str(self.theta))
        # print("alpha " + str(self.alpha))

    def step(self, currentPos, currentHeading, lookAheadDis, LFindex, vehicle_velocity_norm):
        # ... (existing step method code)


        currentX = currentPos[0]
        currentY = currentPos[1]

        # use for loop to search intersections
        lastFoundIndex = LFindex
        intersectFound = False
        startingIndex = lastFoundIndex
        for i in range(startingIndex, (len(self.path) + startingIndex)):
            # beginning of line-circle intersection code
            currentPoint = self.path[i % len(self.path)]
            nextPoint = self.path[(i+1) % len(self.path)]
            x1 = currentPoint[0] - currentX
            y1 = currentPoint[1] - currentY
            x2 = nextPoint[0] - currentX
            y2 = nextPoint[1] - currentY
            dx = x2 - x1
            dy = y2 - y1
            dr = math.sqrt(dx**2 + dy**2)
            D = x1 * y2 - x2 * y1
            discriminant = (lookAheadDis**2) * (dr**2) - D**2

            if discriminant >= 0:
                sol_x1 = (D * dy + self.sgn(dy) * dx * np.sqrt(discriminant)) / dr**2
                sol_x2 = (D * dy - self.sgn(dy) * dx * np.sqrt(discriminant)) / dr**2
                sol_y1 = (-D * dx + abs(dy) * np.sqrt(discriminant)) / dr**2
                sol_y2 = (-D * dx - abs(dy) * np.sqrt(discriminant)) / dr**2

                sol_pt1 = [sol_x1 + currentX, sol_y1 + currentY]
                sol_pt2 = [sol_x2 + currentX, sol_y2 + currentY]
                self.sol_pt1 = sol_pt1
                self.sol_pt2 = sol_pt2
                # end of line-circle intersection code

                minX = min(currentPoint[0], nextPoint[0])
                minY = min(currentPoint[1], nextPoint[1])
                maxX = max(currentPoint[0], nextPoint[0])
                maxY = max(currentPoint[1], nextPoint[1])

                # if one or both of the solutions are in range
                if ((minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY)) or (
                    (minX <= sol_pt2[0] <= maxX) and (minY <= sol_pt2[1] <= maxY)
                ):
                    foundIntersection = True

                    # if both solutions are in range, check which one is better
                    if ((minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY)) and (
                        (minX <= sol_pt2[0] <= maxX) and (minY <= sol_pt2[1] <= maxY)
                    ):
                        # make the decision by compare the distance between the intersections and the next point in path
                        if pt_to_pt_distance(sol_pt1, nextPoint) < pt_to_pt_distance(sol_pt2, nextPoint):
                            goalPt = sol_pt1
                        else:
                            goalPt = sol_pt2

                    # if not both solutions are in range, take the one that's in range
                    else:
                        # if solution pt1 is in range, set that as goal point
                        if (minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY):
                            goalPt = sol_pt1
                        else:
                            goalPt = sol_pt2

                    # only exit loop if the solution pt found is closer to the next pt in path than the current pos
                    if pt_to_pt_distance(goalPt, nextPoint) < pt_to_pt_distance([currentX, currentY], nextPoint):
                        # update lastFoundIndex and exit
                        lastFoundIndex = i
                        lastFoundIndex = lastFoundIndex % len(self.path)
                        break
                    else:
                        # in case for some reason the robot cannot find intersection in the next path segment, but we also don't want it to go backward
                        lastFoundIndex = i + 1
                        lastFoundIndex = lastFoundIndex % len(self.path)

                # if no solutions are in range
                else:
                    foundIntersection = False
                    # no new intersection found, potentially deviated from the path
                    # follow path[lastFoundIndex]
                    goalPt = [self.path[lastFoundIndex][0], self.path[lastFoundIndex][1]]

            # if determinant < 0
            else:
                foundIntersection = False
                # no new intersection found, potentially deviated from the path
                # follow path[lastFoundIndex]
                goalPt = [self.path[lastFoundIndex][0], self.path[lastFoundIndex][1]]

        # calculate the angle to the goal point
        target_heading = np.arctan2(goalPt[1] - currentPos[1], goalPt[0] - currentPos[0])
        turnError = normalize_rad(target_heading - currentHeading)

        # apply proportional control to the angle depending on the velocity
        # turnVel = Kp * turnError\
        turnVel =  (-18.0 / (np.sqrt(vehicle_velocity_norm)+0.0000001) * turnError / np.pi)
        # Calculate alpha and theta
        self.calculate_alpha_theta(currentPos, currentHeading, goalPt, self.sol_pt1, self.sol_pt2)

        return goalPt, lastFoundIndex, turnVel

    def adaptive_preview_distance(self, Ld_max, Ld_min,speed):
        # print("Ld_max: " + str(Ld_max))
        # print("Ld_min: " + str(Ld_min))

        if self.alpha <= self.alpha_0 and self.theta <= self.theta_0:
            # print("max" + str(Ld_max))
            return Ld_max
        else:
            Ld = (0.5 * (Ld_min + (Ld_max - Ld_min) * abs(np.cos(self.alpha / 2))) +
                  0.5 * (Ld_min + (Ld_max - Ld_min) * abs(np.cos(self.theta / 2))) +
                  0.8 * speed)

            # print("lookahead " + str(Ld))
            return np.clip(Ld, Ld_min, Ld_max)
