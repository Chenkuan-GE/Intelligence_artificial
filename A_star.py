from Queue import PriorityQueue

class State(object):
  def __init__(self,value,parent,start=0,goal=0,solver = 0):
    self.children = []
    self.parent = parent
    self.value = value
    self.dist = 0
    if parent:
       self.path = parent.path[:]
