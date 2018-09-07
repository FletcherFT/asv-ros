import task
class Hierarchy():
    def __init__(self,plan_msg):
        root = None
        plan = plan_msg.plan
        for i in range(len(plan)):
            if plan[i].parent == -1:
                if root is None:
                    root=i
                else:
                    print("ERROR: There is more than one root task.")
                    return
        if root is None:
            print("ERROR: There is no root task.")
            return
        if not plan[root].children:
            print("Plan is empty, idling")

        self.plan = plan
        self._complete = []
        self._root = root
        self._current = root
        self._findLowest(True)

    def getTask(self):
        if self._current != self._root:
            self._complete.append(self._current)
        if self._findLowest()<0:
            return self.plan[self._root]
        else:
            return self.plan[self._current]

    def _findLowest(self,flag=False):
        # if first time, select first child of root (if there is one)
        if flag:
            if self.plan[self._root].children:
                self._current = self.plan[self._root].children[0]
            else:
                return
        go_up = False
        while True:
            # if we need to go up a layer, it means the task is done.
            if go_up:
                self._current = self.plan[self._current].parent
            # get the children of the current task
            layer = self.plan[self._current].children
            # if there's children, find one that hasn't been completed
            if layer:
                for i in layer:
                    if not i in self._complete:
                        self._current=i
                        go_up = False
                        break
#TODO UNTANGLE THIS
                    else:
                        if not self._current in self._complete:
                            return
                        else:
                            go_up=True
            # if there's no children and it's not on the list, this is our current primitive!
            else:
                if not self._current in self._complete:
                    return
                else:
                    go_up=True
        print("Cycle")