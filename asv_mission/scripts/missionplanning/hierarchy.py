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
        self._flag = True

    def getTask(self):
        self._findLowest()
        # if self._flag:
        #     # this is the first time the getTask function has been called for this plan, don't append anything to self._complete
        #     self._findLowest()
        #     self._flag=False
        # else:
        #     self._complete.append(self._current)
        #     self._findLowest()
        return self.plan[self._current]

    def taskDone(self):
        self._complete.append(self._current)

    def _findLowest(self):
        while True:
            children = self.plan[self._current].children
            if children:
                # go through every child and find one that hasn't been completed
                flag=False
                for child in children:
                    if not child in self._complete:
                        # if there's a child that isn't completed, go down a level
                        self._current=child
                        flag=True
                        break
                if flag:
                    # if a child was found that hasn't been completed yet, select it and see if it has any children
                    continue
                else:
                    # otherwise, all the children are complete, therefore this current task is ready to be selected
                    return
            else:
                # this task has no children, so check if it has been completed
                if not self._current in self._complete:
                    return
                else:
                    # the task has been completed, so go up a level
                    self._current=self.plan[self._current].parent