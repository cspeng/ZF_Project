#!/usr/bin/env python

import yaml
PKG= 'dashgo_tools'

class ros_py():
    yaml_file = 'NAVI_F5D0.yaml'
    params = {}

    def get_param(self, name, data):
        if name.startswith('~') or name.startwith('/'):
            name = name[1:]
        self.params[name] = data
        return data

    def test_params(self):
        with open(self.yaml_file, 'r') as f:
            doc = yaml.load(f)
        with open(self.yaml_file, 'w') as f:
            yaml.dump(doc, f, default_flow_style=False)
        return doc

    def dump_params(self):
        with open(self.yaml_file, 'w') as f:
            yaml.dump(self.params, f, default_flow_style=False)

    def get_params(self):
        """ fine toning parameters """
        rf_code = ['F4C020','F4C021', 'F4C022', 'F4C023', 'F4C024', 'F4C025']
        # settings on AGV tracking
        # goal: a 1 2 3 4 A, beacon: a b c d e f
        # base length: 150/30 or 5mm per unit
        self.base_length = rospy.get_param("~base_length", 560.0/5)
        self.rf_id = rospy.get_param("~rf_id", rf_code)
        self.lbat_trig = rospy.get_param("~lbat_trig", 1000)
        self.semi_trig = rospy.get_param("~semi_trig", 0.5)
        self.align_blob = rospy.get_param("~align_blob", 0.20)
        self.align_acute = rospy.get_param("~align_acute", 0.25) # (0.25, 0.20)
        self.align_drift = rospy.get_param("~align_drift", 1.2) # 1.2
        self.align_Adist = rospy.get_param("~align_Adist", 0.2) # 0.2
        self.align_Bdist = rospy.get_param("~align_Bdist", 0.2) # 0.2
        # setting of AGV locations
        self.goal_dist = rospy.get_param("~goal_dist", [0, 0.6 , 3.2, 1.7, 2.0, 8.7])
        self.rf_beacon = rospy.get_param("~rf_beacon", [0, 1.6, 1.3, 0.0, 0.0, 0.0])
        self.ZTEST_distance = rospy.get_param("~ZTEST_distance", 0.0) # 3.5
        self.ZPASS_atleast = rospy.get_param("~ZPASS_atleast", 1.2) # 1.2
        self.ZBA_autoback = rospy.get_param("~ZBA_autoback", 10) # 10
        self.ZBP_autoback = rospy.get_param("~ZBP_autoback", 0) # disable
        self.ZPP_angle = rospy.get_param("~ZPP_angle", -86.0) # -ve anticlockwsie
        self.ZPP_dist = rospy.get_param("~ZPP_dist", 0.6)
        self.ZPB_dist = rospy.get_param("~ZPB_dist", 1.9)

        return self.params

if __name__ == '__main__':
    """ main """
    rospy = ros_py()
    params = rospy.get_params()
    rospy.dump_params()
    print(rospy.test_params())

