import skfuzzy.control as ctrl
import numpy as np
import skfuzzy as fuzzy

class OmegaControl():
    def __init__(self):
        # Initialize sparse universe
        universe = np.linspace(-1.0, 1.0, 20)

        # Create fuzzy variables
        theta_error = ctrl.Antecedent(universe, 'theta_error')
        out_omega = ctrl.Consequent(universe, 'out_omega')

        # Using Gaussian membership function
        theta_error['nb'] = fuzzy.gaussmf(theta_error.universe, -0.8, 0.3)
        theta_error['ns'] = fuzzy.gaussmf(theta_error.universe, -0.2, 0.1)
        theta_error['ze'] = fuzzy.gaussmf(theta_error.universe, 0.0, 0.1)
        theta_error['ps'] = fuzzy.gaussmf(theta_error.universe, 0.2, 0.1)
        theta_error['pb'] = fuzzy.gaussmf(theta_error.universe, 0.8, 0.3)

        out_omega['nb'] = fuzzy.gaussmf(out_omega.universe, -0.8, 0.2)
        out_omega['ns'] = fuzzy.gaussmf(out_omega.universe, -0.4, 0.2)
        out_omega['ze'] = fuzzy.gaussmf(out_omega.universe, 0.0, 0.2)
        out_omega['ps'] = fuzzy.gaussmf(out_omega.universe, 0.4, 0.2)
        out_omega['pb'] = fuzzy.gaussmf(out_omega.universe, 0.8, 0.2)

        rule_0 = ctrl.Rule(antecedent=(theta_error['nb']),
                            consequent=(out_omega['pb']),
                            label='rule_0')
        
        rule_1 = ctrl.Rule(antecedent=(theta_error['ns']),
                            consequent=(out_omega['ps']),
                            label='rule_1')
        
        rule_2 = ctrl.Rule(antecedent=(theta_error['ze']),
                            consequent=(out_omega['ze']),
                            label='rule_2')
        
        rule_3 = ctrl.Rule(antecedent=(theta_error['ps']),
                            consequent=(out_omega['ns']),
                            label='rule_3')
        
        rule_4 = ctrl.Rule(antecedent=(theta_error['pb']),
                            consequent=(out_omega['nb']),
                            label='rule_4')
        
        # Construct controller
        self.fuzzy_system = ctrl.ControlSystem(rules=[rule_0, rule_1, rule_2, rule_3, rule_4])

    def getController(self):
        return ctrl.ControlSystemSimulation(self.fuzzy_system, flush_after_run=1)