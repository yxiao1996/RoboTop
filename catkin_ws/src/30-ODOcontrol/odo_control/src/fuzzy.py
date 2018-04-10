import skfuzzy.control as ctrl
import numpy as np
import skfuzzy as fuzzy
class FuzzyControl():
    def __init__(self):
        # Initialize sparse universe
        universe = np.linspace(-2, 2, 50)

        # Create fuzzy variables
        debug = False
        gauss_mf = True
        lane_error = ctrl.Antecedent(universe, 'lane_error')
        head_error = ctrl.Antecedent(universe, 'head_error')
        out_v = ctrl.Consequent(np.linspace(0, 1, 50), 'out_v')
        out_omega = ctrl.Consequent(universe, 'out_omega')

        # Generate membership function automatically
        names = ['nb', 'ns', 'ze', 'ps', 'pb']
        #lane_error.automf(names=names)
        #head_error.automf(names=names)
        #out_v.automf(names=names)
        #out_omega.automf(names=names)

        if gauss_mf:
            # Use Gaussian membership function
            # Input membership
            lane_error['nb'] = fuzzy.gaussmf(lane_error.universe, -0.3, 0.1)
            lane_error['ns'] = fuzzy.gaussmf(lane_error.universe, -0.1, 0.1)
            lane_error['ze'] = fuzzy.gaussmf(lane_error.universe, 0.0, 0.1)
            lane_error['ps'] = fuzzy.gaussmf(lane_error.universe, 0.1, 0.1)
            lane_error['pb'] = fuzzy.gaussmf(lane_error.universe, 0.3, 0.1)
            
            head_error['nb'] = fuzzy.gaussmf(head_error.universe, -0.7, 0.2)
            head_error['ns'] = fuzzy.gaussmf(head_error.universe, -0.2, 0.1)
            head_error['ze'] = fuzzy.gaussmf(head_error.universe, 0.0, 0.2)
            head_error['ps'] = fuzzy.gaussmf(head_error.universe, 0.2, 0.1)
            head_error['pb'] = fuzzy.gaussmf(head_error.universe, 0.7, 0.2)

            # Output membership
            out_omega['nb'] = fuzzy.gaussmf(out_omega.universe, -0.7, 0.3)
            out_omega['ns'] = fuzzy.gaussmf(out_omega.universe, -0.4, 0.1)
            out_omega['ze'] = fuzzy.gaussmf(out_omega.universe, 0.0, 0.2)
            out_omega['ps'] = fuzzy.gaussmf(out_omega.universe, 0.4, 0.1)
            out_omega['pb'] = fuzzy.gaussmf(out_omega.universe, 0.7, 0.3)

            out_v['ps'] = fuzzy.gaussmf(out_v.universe, 0.3, 0.2)
            out_v['pb'] = fuzzy.gaussmf(out_v.universe, 0.6, 0.2)
            out_v['ze'] = fuzzy.gaussmf(out_v.universe, 0.0, 0.2)
        else:
            # Input membership
            lane_error['nb'] = fuzzy.zmf(lane_error.universe, -0.3, -0.1)
            lane_error['ns'] = fuzzy.trimf(lane_error.universe, [-0.2, -0.1, 0.0])
            lane_error['ze'] = fuzzy.trimf(lane_error.universe, [-0.1, 0.0, 0.1])
            lane_error['ps'] = fuzzy.trimf(lane_error.universe, [0.0, 0.1, 0.2])
            lane_error['pb'] = 1.0 - fuzzy.zmf(lane_error.universe, 0.1, 0.3)
            head_error['nb'] = fuzzy.zmf(head_error.universe, -0.7, -0.3)
            head_error['ns'] = fuzzy.trimf(head_error.universe, [-0.4, -0.2, -0.1])
            head_error['ze'] = fuzzy.trimf(head_error.universe, [-0.2, 0.0, 0.2])
            head_error['ps'] = fuzzy.trimf(head_error.universe, [0.1, 0.2, 0.4])
            head_error['pb'] = 1.0 - fuzzy.zmf(head_error.universe, 0.3, 0.7)

            # Output membership
            out_omega['nb'] = fuzzy.zmf(out_omega.universe, -0.7, -0.3)
            out_omega['ns'] = fuzzy.trimf(out_omega.universe, [-0.4, -0.2, -0.1])
            out_omega['ze'] = fuzzy.trimf(out_omega.universe, [-0.2, 0.0, 0.2])
            out_omega['ps'] = fuzzy.trimf(out_omega.universe, [0.1, 0.2, 0.4])
            out_omega['pb'] = 1.0 - fuzzy.zmf(out_omega.universe, 0.3, 0.7)
            out_v['ps'] = fuzzy.trimf(out_omega.universe, [0.4, 0.5, 0.6])
            out_v['pb'] = 1.0 - fuzzy.zmf(out_omega.universe, 0.5, 0.7)
            out_v['ze'] = fuzzy.zmf(out_v.universe, 0.3, 0.5)

        # Define rules
        # Rule 0: when perfectly in lane, run straight as high speed
        rule0 = ctrl.Rule(antecedent=((lane_error['ze'] & head_error['ze'])),
                            consequent=(out_v['pb']),
                            label='rule_0')

        rule0_slow = ctrl.Rule(antecedent=((lane_error['ze'] & head_error['ze'])),
                            consequent=(out_v['ps']),
                            label='rule_0')

        rule1 = ctrl.Rule(antecedent=((lane_error['ze'] & head_error['ze'])),
                            consequent=(out_omega['ze']),
                            label='rule_1')

        # Rule 1: when slight heading error, adjust head slightly
        rule2 = ctrl.Rule(antecedent=((lane_error['ns'] & head_error['ns']) |
                                        (lane_error['ps'] & head_error['ns']) |
                                        (lane_error['nb'] & head_error['ns']) |
                                        (lane_error['pb'] & head_error['ns']) |
                                        (lane_error['ze'] & head_error['ns']) |                  
                                        (lane_error['ns'] & head_error['ps']) |
                                        (lane_error['ps'] & head_error['ps']) |
                                        (lane_error['nb'] & head_error['ps']) |
                                        (lane_error['pb'] & head_error['ps']) |
                                        (lane_error['ze'] & head_error['ps']) |
                                        (lane_error['ns'] & head_error['ze']) |
                                        (lane_error['ps'] & head_error['ze']) |
                                        (lane_error['nb'] & head_error['ze']) |
                                        (lane_error['pb'] & head_error['ze'])),
                            consequent=(out_v['ps']),
                            label='rule_2')
        rule_v2 = ctrl.Rule(antecedent=((lane_error['ns'] & head_error['nb']) |
                                        (lane_error['ps'] & head_error['nb']) |
                                        (lane_error['nb'] & head_error['nb']) |
                                        (lane_error['pb'] & head_error['nb']) |
                                        (lane_error['ze'] & head_error['nb']) |
                                        (lane_error['ns'] & head_error['pb']) |
                                        (lane_error['ps'] & head_error['pb']) |
                                        (lane_error['nb'] & head_error['pb']) |
                                        (lane_error['pb'] & head_error['pb']) |
                                        (lane_error['ze'] & head_error['pb']) 
                                        ),
                        consequent=(out_v['ze']),
                        label='rule_v2')
        rule3 = ctrl.Rule(antecedent=((lane_error['ns'] & head_error['ns']) |
                                        (lane_error['ps'] & head_error['ns']) |
                                        (lane_error['nb'] & head_error['ns']) |
                                        (lane_error['pb'] & head_error['ns']) |
                                        (lane_error['ze'] & head_error['ns']) |
                                        (lane_error['ns'] & head_error['ze']) |
                                        (lane_error['nb'] & head_error['ze'])),
                            consequent=(out_omega['ps']),
                            label='rule_3')
        rule4 = ctrl.Rule(antecedent=((lane_error['ns'] & head_error['ps']) |
                                        (lane_error['ps'] & head_error['ps']) |
                                        (lane_error['nb'] & head_error['ps']) |
                                        (lane_error['pb'] & head_error['ps']) |
                                        (lane_error['ze'] & head_error['ps']) |
                                        (lane_error['ps'] & head_error['ze']) |
                                        (lane_error['pb'] & head_error['ze'])),
                            consequent=(out_omega['ns']),
                            label='rule_4')

        # Rule 2: when heading error is big, adjust heading seriously
        rule5 = ctrl.Rule(antecedent=((lane_error['ns'] & head_error['nb']) |
                                        (lane_error['ps'] & head_error['nb']) |
                                        (lane_error['nb'] & head_error['nb']) |
                                        (lane_error['pb'] & head_error['nb']) |
                                        (lane_error['ze'] & head_error['nb'])),
                            consequent=(out_omega['pb']),
                            label='rule_5')
        rule6 = ctrl.Rule(antecedent=((lane_error['ns'] & head_error['pb']) |
                                        (lane_error['ps'] & head_error['pb']) |
                                        (lane_error['nb'] & head_error['pb']) |
                                        (lane_error['pb'] & head_error['pb']) |
                                        (lane_error['ze'] & head_error['pb'])),
                            consequent=(out_omega['nb']),
                            label='rule_6')

        self.fuzzy_system = ctrl.ControlSystem(rules=[rule0, rule1, rule_v2, rule2, rule3, rule4, rule5, rule6])
        self.fuzzy_system_slow = ctrl.ControlSystem(rules=[rule0_slow, rule1, rule_v2, rule2, rule3, rule4, rule5, rule6])

        # Dubug fuzzy control
        if debug:
            sim = ctrl.ControlSystemSimulation(self.fuzzy_system, flush_after_run=1)
            upsampled = np.linspace(-2, 2, 21)
            x, y = np.meshgrid(upsampled, upsampled)
            z = np.zeros_like(x)

            for i in range(21):
                for j in range(21):
                    sim.input['lane_error'] = x[i, j]
                    sim.input['head_error'] = y[i, j]
                    sim.compute()
                    z[i, j] = sim.output['out_v']
            
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D

            fig = plt.figure(figsize=(8, 8))
            ax = fig.add_subplot(111, projection='3d')

            surf = ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap='viridis',
                                linewidth=0.4, antialiased=True)

            cset = ax.contourf(x, y, z, zdir='z', offset=-2.5, cmap='viridis', alpha=0.5)
            cset = ax.contourf(x, y, z, zdir='x', offset=3, cmap='viridis', alpha=0.5)
            cset = ax.contourf(x, y, z, zdir='y', offset=3, cmap='viridis', alpha=0.5)

            ax.view_init(30, 200)
            fig.show()
            tmp = raw_input()
        

    def get_ctl(self):
        return ctrl.ControlSystemSimulation(self.fuzzy_system, flush_after_run=1)

    def get_ctl_slow(self):
        return ctrl.ControlSystemSimulation(self.fuzzy_system_slow, flush_after_run=1)

class FuzzyControlSlow():
    def __init__(self):
        # Initialize sparse universe
        universe = np.linspace(-2, 2, 50)

        # Create fuzzy variables
        debug = False
        gauss_mf = True
        lane_error = ctrl.Antecedent(universe, 'lane_error')
        head_error = ctrl.Antecedent(universe, 'head_error')
        out_v = ctrl.Consequent(np.linspace(0, 1, 50), 'out_v')
        out_omega = ctrl.Consequent(universe, 'out_omega')

        # Generate membership function automatically
        names = ['nb', 'ns', 'ze', 'ps', 'pb']
        #lane_error.automf(names=names)
        #head_error.automf(names=names)
        #out_v.automf(names=names)
        #out_omega.automf(names=names)

        if gauss_mf:
            # Use Gaussian membership function
            # Input membership
            lane_error['nb'] = fuzzy.zmf(lane_error.universe, -0.3, -0.1)
            lane_error['ns'] = fuzzy.gaussmf(lane_error.universe, -0.1, 0.1)
            lane_error['ze'] = fuzzy.gaussmf(lane_error.universe, 0.0, 0.1)
            lane_error['ps'] = fuzzy.gaussmf(lane_error.universe, 0.1, 0.1)
            lane_error['pb'] = 1.0 - fuzzy.zmf(lane_error.universe, 0.1, 0.3)
            
            head_error['nb'] = fuzzy.zmf(head_error.universe, -0.7, -0.3)
            head_error['ns'] = fuzzy.gaussmf(head_error.universe, -0.2, 0.1)
            head_error['ze'] = fuzzy.gaussmf(head_error.universe, 0.0, 0.2)
            head_error['ps'] = fuzzy.gaussmf(head_error.universe, 0.2, 0.1)
            head_error['pb'] = 1.0 - fuzzy.zmf(head_error.universe, 0.3, 0.7)

            # Output membership
            out_omega['nb'] = fuzzy.zmf(out_omega.universe, -0.7, -0.3)
            out_omega['ns'] = fuzzy.gaussmf(out_omega.universe, -0.2, 0.1)
            out_omega['ze'] = fuzzy.gaussmf(out_omega.universe, 0.0, 0.2)
            out_omega['ps'] = fuzzy.gaussmf(out_omega.universe, 0.2, 0.1)
            out_omega['pb'] = 1.0 - fuzzy.zmf(out_omega.universe, 0.3, 0.7)

            out_v['ps'] = fuzzy.gaussmf(out_omega.universe, 0.2, 0.1)
            out_v['pb'] = 1.0 - fuzzy.zmf(out_omega.universe, 0.5, 0.7)
            out_v['ze'] = fuzzy.zmf(out_v.universe, 0.1, 0.3)
        else:
            # Input membership
            lane_error['nb'] = fuzzy.zmf(lane_error.universe, -0.3, -0.1)
            lane_error['ns'] = fuzzy.trimf(lane_error.universe, [-0.2, -0.1, 0.0])
            lane_error['ze'] = fuzzy.trimf(lane_error.universe, [-0.1, 0.0, 0.1])
            lane_error['ps'] = fuzzy.trimf(lane_error.universe, [0.0, 0.1, 0.2])
            lane_error['pb'] = 1.0 - fuzzy.zmf(lane_error.universe, 0.1, 0.3)
            head_error['nb'] = fuzzy.zmf(head_error.universe, -0.7, -0.3)
            head_error['ns'] = fuzzy.trimf(head_error.universe, [-0.4, -0.2, -0.1])
            head_error['ze'] = fuzzy.trimf(head_error.universe, [-0.2, 0.0, 0.2])
            head_error['ps'] = fuzzy.trimf(head_error.universe, [0.1, 0.2, 0.4])
            head_error['pb'] = 1.0 - fuzzy.zmf(head_error.universe, 0.3, 0.7)

            # Output membership
            out_omega['nb'] = fuzzy.zmf(out_omega.universe, -0.7, -0.3)
            out_omega['ns'] = fuzzy.trimf(out_omega.universe, [-0.4, -0.2, -0.1])
            out_omega['ze'] = fuzzy.trimf(out_omega.universe, [-0.2, 0.0, 0.2])
            out_omega['ps'] = fuzzy.trimf(out_omega.universe, [0.1, 0.2, 0.4])
            out_omega['pb'] = 1.0 - fuzzy.zmf(out_omega.universe, 0.3, 0.7)
            out_v['ps'] = fuzzy.trimf(out_omega.universe, [0.4, 0.5, 0.6])
            out_v['pb'] = 1.0 - fuzzy.zmf(out_omega.universe, 0.5, 0.7)
            out_v['ze'] = fuzzy.zmf(out_v.universe, 0.3, 0.5)

        # Define rules
        # Rule 0: when perfectly in lane, run straight as high speed
        rule0 = ctrl.Rule(antecedent=((lane_error['ze'] & head_error['ze'])),
                            consequent=(out_v['ps']),
                            label='rule_0')
        rule1 = ctrl.Rule(antecedent=((lane_error['ze'] & head_error['ze'])),
                            consequent=(out_omega['ze']),
                            label='rule_1')

        # Rule 1: when slight heading error, adjust head slightly
        rule2 = ctrl.Rule(antecedent=((lane_error['ns'] & head_error['ns']) |
                                        (lane_error['ps'] & head_error['ns']) |
                                        (lane_error['nb'] & head_error['ns']) |
                                        (lane_error['pb'] & head_error['ns']) |
                                        (lane_error['ze'] & head_error['ns']) |                  
                                        (lane_error['ns'] & head_error['ps']) |
                                        (lane_error['ps'] & head_error['ps']) |
                                        (lane_error['nb'] & head_error['ps']) |
                                        (lane_error['pb'] & head_error['ps']) |
                                        (lane_error['ze'] & head_error['ps']) |
                                        (lane_error['ns'] & head_error['ze']) |
                                        (lane_error['ps'] & head_error['ze']) |
                                        (lane_error['nb'] & head_error['ze']) |
                                        (lane_error['pb'] & head_error['ze'])),
                            consequent=(out_v['ps']),
                            label='rule_2')
        rule_v2 = ctrl.Rule(antecedent=((lane_error['ns'] & head_error['nb']) |
                                        (lane_error['ps'] & head_error['nb']) |
                                        (lane_error['nb'] & head_error['nb']) |
                                        (lane_error['pb'] & head_error['nb']) |
                                        (lane_error['ze'] & head_error['nb']) |
                                        (lane_error['ns'] & head_error['pb']) |
                                        (lane_error['ps'] & head_error['pb']) |
                                        (lane_error['nb'] & head_error['pb']) |
                                        (lane_error['pb'] & head_error['pb']) |
                                        (lane_error['ze'] & head_error['pb']) 
                                        ),
                        consequent=(out_v['ze']),
                        label='rule_v2')
        rule3 = ctrl.Rule(antecedent=((lane_error['ns'] & head_error['ns']) |
                                        (lane_error['ps'] & head_error['ns']) |
                                        (lane_error['nb'] & head_error['ns']) |
                                        (lane_error['pb'] & head_error['ns']) |
                                        (lane_error['ze'] & head_error['ns']) |
                                        (lane_error['ns'] & head_error['ze']) |
                                        (lane_error['nb'] & head_error['ze'])),
                            consequent=(out_omega['ps']),
                            label='rule_3')
        rule4 = ctrl.Rule(antecedent=((lane_error['ns'] & head_error['ps']) |
                                        (lane_error['ps'] & head_error['ps']) |
                                        (lane_error['nb'] & head_error['ps']) |
                                        (lane_error['pb'] & head_error['ps']) |
                                        (lane_error['ze'] & head_error['ps']) |
                                        (lane_error['ps'] & head_error['ze']) |
                                        (lane_error['pb'] & head_error['ze'])),
                            consequent=(out_omega['ns']),
                            label='rule_4')

        # Rule 2: when heading error is big, adjust heading seriously
        rule5 = ctrl.Rule(antecedent=((lane_error['ns'] & head_error['nb']) |
                                        (lane_error['ps'] & head_error['nb']) |
                                        (lane_error['nb'] & head_error['nb']) |
                                        (lane_error['pb'] & head_error['nb']) |
                                        (lane_error['ze'] & head_error['nb'])),
                            consequent=(out_omega['pb']),
                            label='rule_5')
        rule6 = ctrl.Rule(antecedent=((lane_error['ns'] & head_error['pb']) |
                                        (lane_error['ps'] & head_error['pb']) |
                                        (lane_error['nb'] & head_error['pb']) |
                                        (lane_error['pb'] & head_error['pb']) |
                                        (lane_error['ze'] & head_error['pb'])),
                            consequent=(out_omega['nb']),
                            label='rule_6')

        self.fuzzy_system = ctrl.ControlSystem(rules=[rule0, rule1, rule_v2, rule2, rule3, rule4, rule5, rule6])

        # Dubug fuzzy control
        if debug:
            sim = ctrl.ControlSystemSimulation(self.fuzzy_system, flush_after_run=1)
            upsampled = np.linspace(-2, 2, 21)
            x, y = np.meshgrid(upsampled, upsampled)
            z = np.zeros_like(x)

            for i in range(21):
                for j in range(21):
                    sim.input['lane_error'] = x[i, j]
                    sim.input['head_error'] = y[i, j]
                    sim.compute()
                    z[i, j] = sim.output['out_v']
            
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D

            fig = plt.figure(figsize=(8, 8))
            ax = fig.add_subplot(111, projection='3d')

            surf = ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap='viridis',
                                linewidth=0.4, antialiased=True)

            cset = ax.contourf(x, y, z, zdir='z', offset=-2.5, cmap='viridis', alpha=0.5)
            cset = ax.contourf(x, y, z, zdir='x', offset=3, cmap='viridis', alpha=0.5)
            cset = ax.contourf(x, y, z, zdir='y', offset=3, cmap='viridis', alpha=0.5)

            ax.view_init(30, 200)
            fig.show()
            tmp = raw_input()
        

    def get_ctl(self):
        return ctrl.ControlSystemSimulation(self.fuzzy_system, flush_after_run=1)

C = FuzzyControl()