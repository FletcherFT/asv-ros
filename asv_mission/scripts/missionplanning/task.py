class Task:
    def __init__(self,children,energy,action):
        self.children=children
        self.is_root=not self.children
        self.energy=energy
        self.action=action