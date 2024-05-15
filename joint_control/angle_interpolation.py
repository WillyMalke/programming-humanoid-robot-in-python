'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        if 'LHipYawPitch' in target_joints:
            target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        joints = keyframes[0]
        times = keyframes[1]
        keys = keyframes[2]

        for i in range (len(joints)):
            interpolated_values = []

            # Iteriere über die Zeitpunkte
            for j in range(len(times[i])):
                # Berechne die Bezier-Interpolation für jeden Zeitpunkt t
                if j < len(times[i]) - 1:
                    if j == len(times[i]) - 2:  # Verhindere Extrapolation am Ende
                        p0 = keys[i][-4][0]
                        p1 = keys[i][-3][0]
                        p2 = keys[i][-2][0]
                        p3 = keys[i][-1][0]
                    else:
                        # Bezier-Interpolation
                        p0 = keys[i][j - 1][0]
                        p1 = keys[i][j][0]
                        p2 = keys[i][j + 1][0]
                        p3 = keys[i][j + 2][0]

                    # Berechne die Bezier-Interpolation gemäß der gegebenen Formel
                    t_relative = (times[i][j+1] - times[i][j]) / (times[i][j + 1] - times[i][j])
                    b_i = ((1 - t_relative) ** 3) * p0 + 3 * ((1 - t_relative) ** 2) * t_relative * p1 + \
                      3 * (1 - t_relative) * (t_relative ** 2) * p2 + (t_relative ** 3) * p3


                    # Füge den interpolierten Wert zur Liste hinzu
                    interpolated_values.append(b_i)

                    # Füge die interpolierten Werte für dieses Gelenk zur Ausgabeliste hinzu
                    target_joints[joints] = interpolated_values


            return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
