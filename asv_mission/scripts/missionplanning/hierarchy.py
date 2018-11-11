import task
#import pdb
class Hierarchy():
    def __init__(self,plan_msg):
        root = None
        self.home = None
        plan = plan_msg.plan
        for i in range(len(plan)):
            if plan[i].parent == -1:
                if root is None:
                    root=i
                else:
                    print("ERROR: There is more than one root task.")
                    return
            elif plan[i].action=="HOME":
                if self.home is None:
                    self.home = i
                else:
                    print("ERROR: There is more than one home task.")
                    return
                
        if root is None:
            print("ERROR: There is no root task.")
            return
        if not plan[root].children:
            print("Plan is empty, idling")

        self.plan = plan
        self._complete = []
        self._skipped = []
        self._checked = []
        self._todo = range(len(plan))
        self._root = root
        self._current = root

    def getTask(self):
        self._findLowest()
        return self.plan[self._current]

    def taskDone(self):
        self._complete.append(self._current)
        self._checked = self._complete+self._skipped
        self._todo = [x for x in range(len(self.plan)) if x not in self._checked]

    def skipTask(self):
        self._skipped.append(self._current)
        self._checked = self._complete+self._skipped
        self._todo = [x for x in range(len(self.plan)) if x not in self._checked]

    def _findLowest(self):
        while True:
            if self._current in self._checked:
                self._current = self.plan[self._current].parent
            children = self.plan[self._current].children
            # if there are children, see if any are incomplete
            if children:
                flag = False # flag to determine if a child of the current is incomplete
                for child in children:
                    if not child in self._checked:
                        self._current = child
                        flag = True
                        break
                # if there was an incomplete child, start the process again
                if flag:
                    continue
                # otherwise all children are complete of the current, so select the current for execution.
                else:
                    return
            # there are no children, and the current is incomplete, so return it
            else:
                return
            

