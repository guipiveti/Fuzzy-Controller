import skfuzzy as fuzz
from skfuzzy import control as ctrl

import numpy as np

left = ctrl.Antecedent(np.arange(0, 101, 1), 'Left Sensor')
front_l = ctrl.Antecedent(np.arange(0, 101, 1), 'Front-L Sensor')
front_r = ctrl.Antecedent(np.arange(0, 101, 1), 'Front-R Sensor')
right = ctrl.Antecedent(np.arange(0, 101, 1), 'Right Sensor')
direction = ctrl.Consequent(np.arange(-100, 100, 1), 'Angle')

front_l['near'] = fuzz.trimf(front_l.universe, [0, 0, 30])
front_l['far'] = fuzz.trapmf(front_l.universe, [0, 30, 100, 100])
front_r['near'] = fuzz.trimf(front_r.universe, [0, 0, 30])
front_r['far'] = fuzz.trapmf(front_r.universe, [0, 30, 100, 100])
left['near'] = fuzz.trimf(left.universe, [0, 0, 30])
left['far'] = fuzz.trapmf(left.universe, [0, 30, 100, 100])
right['near'] = fuzz.trimf(right.universe, [0, 0, 30])
right['far'] = fuzz.trapmf(right.universe, [0, 30, 100, 100])

# front_l.view()
# front_r.view()
# left.view()
# right.view()

# direction['left'] = fuzz.trimf(direction.universe, [-100, -100, -50])
# direction['front'] = fuzz.trapmf(direction.universe, [0, 30, 100, 100])
# direction['right'] = fuzz.trapmf(direction.universe, [ 50, 100, 100, 100])
# direction.automf(names=["1", "2", "3"])
direction.automf(names=["Turn Left", "Straight", "Turn Right"])

# direction.view()

rule1 = ctrl.Rule(left['far'] & front_l['far'] & front_r['far'] & right['far'], direction['Straight'])
rule2 = ctrl.Rule(left['far'] & front_l['far'] & front_r['far'] & right['near'], direction['Turn Left'])
rule3 = ctrl.Rule(left['far'] & front_l['far'] & front_r['near'] & right['far'], direction['Turn Left'])
rule4 = ctrl.Rule(left['far'] & front_l['far'] & front_r['near'] & right['near'], direction['Turn Left'])

rule5 = ctrl.Rule(left['far'] & front_l['near'] & front_r['far'] & right['far'], direction['Turn Right'])
rule6 = ctrl.Rule(left['far'] & front_l['near'] & front_r['far'] & right['near'], direction['Turn Left'])
rule7 = ctrl.Rule(left['far'] & front_l['near'] & front_r['near'] & right['far'], direction['Turn Right'])
rule8 = ctrl.Rule(left['far'] & front_l['near'] & front_r['near'] & right['near'], direction['Turn Left'])

rule9 = ctrl.Rule(left['near'] & front_l['far'] & front_r['far'] & right['far'], direction['Turn Right'])
rule10 = ctrl.Rule(left['near'] & front_l['far'] & front_r['far'] & right['near'], direction['Straight'])
rule11 = ctrl.Rule(left['near'] & front_l['far'] & front_r['near'] & right['far'], direction['Turn Right'])
rule12 = ctrl.Rule(left['near'] & front_l['far'] & front_r['near'] & right['near'], direction['Turn Left'])

rule13 = ctrl.Rule(left['near'] & front_l['near'] & front_r['far'] & right['far'], direction['Turn Right'])
rule14 = ctrl.Rule(left['near'] & front_l['near'] & front_r['far'] & right['near'], direction['Turn Right'])
rule15 = ctrl.Rule(left['near'] & front_l['near'] & front_r['near'] & right['far'], direction['Turn Right'])
rule16 = ctrl.Rule(left['near'] & front_l['near'] & front_r['near'] & right['near'], direction['Turn Left'])


# rule1 = ctrl.Rule(front_l['far'] & front_r['far'], direction['Straight'])
# rule2 = ctrl.Rule(left['near'] & front_l['near'], direction['Turn Right'])
# rule3 = ctrl.Rule(right['near'] & front_r['near'], direction['Turn Left'])

# rule1.view()
# rule2.view()
# rule3.view()
# rule4.view()

# rule5.view()
# rule6.view()
# rule7.view()
# rule8.view()

# rule9.view()
# rule10.view()
# rule11.view()
# rule12.view()

# rule13.view()
# rule14.view()
# rule15.view()
# rule16.view()

# direction_ctrl = ctrl.ControlSystem(rules=[rule1, rule2, rule3])
# direction_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14, rule15, rule16])
direction_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14, rule15, rule16])
required_direction = ctrl.ControlSystemSimulation(direction_ctrl)

required_direction.input['Left Sensor'] = 6.5
required_direction.input['Front-L Sensor'] = 9.8
required_direction.input['Front-R Sensor'] = 9.8
required_direction.input['Right Sensor'] = 7

required_direction.compute()

print (required_direction.output['Angle'])
direction.view(sim=required_direction)