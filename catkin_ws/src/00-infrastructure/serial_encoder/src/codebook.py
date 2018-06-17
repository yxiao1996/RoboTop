ONE_GUN = 0
TWO_GUN = 1

class CodeBook(object):
    # code book for generating move command
    def __init__(self, mode=TWO_GUN):
        self.moves = ['fetch_0', 'fetch_1', 'shoot_0', 'shoot_1']
        self.fetch_states = ['START', 'left', 'right', 'both', 'END']
        self.shoot_states = ['START', 'left', 'right', 'both', 'END']

        self.fetch_inputs = ['fetch_0', 'fetch_1']
        self.shoot_inputs = ['shoot_0', 'shoot_1']
        
        self.fetch_trans = {"fetch_0": {"START": "left", "left": "right", "right": "left"},
                            "fetch_1": {"left": "both", "right": "both", "both": "both"}}
        self.shoot_trans = {"shoot_0": {"START": "left", "left": "right", "right": "left"},
                            "shoot_1": {"left": "both", "right": "both", "both": "both"}}

        self.fetch_state = "START"
        self.shoot_state = "START"

        if mode == TWO_GUN:
            self.fetch_dict = {"left": {"button_0": 1.0, "button_1": 0.0},
                            "right": {"button_0": 0.0, "button_1": 1.0},
                            "both": {"button_0": 1.0, "button_1": 1.0}}
            self.shoot_dict = {"left": {"button_0": 1.0, "button_1": 0.0},
                            "right": {"button_0": 0.0, "button_1": 1.0},
                            "both": {"button_0": 1.0, "button_1": 1.0}}
        else:
            self.fetch_dict = {"left": {"button_0": 1.0, "button_1": 0.0},
                            "right": {"button_0": 0.0, "button_1": 1.0},
                            "both": {"button_0": 1.0, "button_1": 1.0}}
            self.shoot_dict = {"left": {"button_0": 1.0, "button_1": 0.0},
                            "right": {"button_0": 0.0, "button_1": 1.0},
                            "both": {"button_0": 1.0, "button_1": 1.0}}
    
    def lookup(self, input):
        if input in self.fetch_inputs:
            self.fetch_state = self.fetch_trans[input][self.fetch_state]
            if self.fetch_state == "START":
                button_1 = button_0 = 0.0
            else:
                button_0 = self.fetch_dict[self.fetch_state]["button_0"]
                button_1 = self.fetch_dict[self.fetch_state]["button_1"]
            return button_0, button_1
        if input in self.shoot_inputs:
            #self.shoot_state = self.shoot_trans[input][self.shoot_state]
            self.shoot_state = self.fetch_state
            if self.shoot_state == "START":
                button_1 = button_0 = 0.0
            else:
                button_0 = self.shoot_dict[self.shoot_state]["button_0"]
                button_1 = self.shoot_dict[self.shoot_state]["button_1"]
            return button_0, button_1
            
TEST = False
if TEST:
    c = CodeBook()
    print c.lookup('fetch_0'), c.fetch_state
    print c.lookup('fetch_0'), c.fetch_state
    print c.lookup('shoot_0'), c.shoot_state
    print c.lookup('fetch_0'), c.fetch_state
    print c.lookup('shoot_0'), c.shoot_state
    print c.lookup('fetch_0'), c.fetch_state
    print c.lookup('fetch_0'), c.fetch_state
    print c.lookup('shoot_0'), c.shoot_state
    print c.lookup('fetch_0'), c.fetch_state
    print c.lookup('shoot_0'), c.shoot_state
    print c.lookup('fetch_1'), c.fetch_state
    print c.lookup('shoot_1'), c.shoot_state
    print c.lookup('fetch_1'), c.fetch_state
    print c.lookup('shoot_1'), c.shoot_state
    print c.lookup('fetch_1'), c.fetch_state
    print c.lookup('shoot_1'), c.shoot_state