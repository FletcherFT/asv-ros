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
        if flag:
            if self.plan[self._root].children:
                self._current = self.plan[self._root].children[0]
            else:
                return -1
        go_up = False
        while True:
            if go_up:
                if self.plan[self._current].parent == self._root:
                    return -1
                self._current=self.plan[self._current].parent
            layer = self.plan[self._current].children
            if layer:
                for i in layer:
                    if not i in self._complete:
                        self._current=i
                        return
                go_up=True