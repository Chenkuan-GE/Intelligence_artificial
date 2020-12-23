
import sys
import math
import numpy as np
import pdb
from operator import add
from scipy.spatial import distance



from visualiser import Visualiser
from problem_spec import ProblemSpec
from robot_config import write_robot_config_list_to_file, make_robot_config_from_ee1, make_robot_config_from_ee2
from tester import test_config_equality, test_self_collision, test_environment_bounds, test_obstacle_collision, test_angle_constraints, test_line_collision,test_bounding_box
from angle import Angle
import tester

"""
Template file for you to implement your solution to Assignment 2. Contains a class you can use to represent graph nodes,
and a method for finding a path in a graph made up of GraphNode objects.

COMP3702 2020 Assignment 2 Support Code
"""


class GraphNode:
    """
    Class representing a node in the state graph. You should create an instance of this class each time you generate
    a sample.
    """

    def __init__(self, spec, config):
        """
        Create a new graph node object for the given config.

        Neighbors should be added by appending to self.neighbors after creating each new GraphNode.

        :param spec: ProblemSpec object
        :param config: the RobotConfig object to be stored in this node
        """
        self.spec = spec
        self.config = config
        self.neighbors = []
        self.p_steps = {}
        self.parent = None

    def __eq__(self, other):
        return test_config_equality(self.config, other.config, self.spec)

    def __hash__(self):
        return hash(tuple(self.config.points))

    def get_successors(self):
        return self.neighbors

    def distance_to(self, other, spec):
        if (self == other):
            return 100000
        c1 = self.config
        c2 = other.config
        if not (c1.ee1_grappled and c2.ee1_grappled) or (c1.ee2_grappled and c2.ee2_grappled):
            return 100000
        else:
            distance = 0
            for i in range(spec.num_segments):
                distance = max(distance, max(abs(c1.lengths[i] - c2.lengths[i]), abs(c1.ee1_angles[i].in_radians() - c2.ee1_angles[i].in_radians())))
            return distance

    @staticmethod
    def add_connection(n1, n2):
        """
        Creates a neighbor connection between the 2 given GraphNode objects.

        :param n1: a GraphNode object
        :param n2: a GraphNode object
        """
        n1.neighbors.append(n2)
        n2.neighbors.append(n1)

def test_self_collision(config):
    if len(config.lengths) < 3:
        return True

    for i in range(len(config.lengths) - 1):
        p1 = config.points[i]
        q1 = config.points[i+1]
        for j in range(i+2, len(config.lengths)):
            p2 = config.points[j]
            q2 = config.points[j+1]

            if test_line_collision((p1,q1),(p2,q2)):
                return False
    return True

def test_obstacle_collision(config, spec, obstacles):
    for i in range(len(config.lengths)):
        p = config.points[i]
        q = config.points[i+1]
        for o in obstacles:
            if not test_bounding_box(p,q, (o.x1, o.y1), (o.x2, o.y2)):
                continue

            for e in o.edges:
                if test_line_collision((p,q), e):
                    return False
    return True

def test_angle_constranints(config, spec):
    for i in range(1, len(config.lengths)):
        a = config.ee1_angles[i]
        if not ((-11*math.pi/12)- spec.TOLERANCE < a < (11*math.pi/12)*spec.TOLERANCE):
            return False
    return True


def check_valid(config, spec):
    if len(config.points) <= 1:
        return True
    return (tester.test_angle_constraints(config, spec) and tester.test_self_collision(config) and tester.test_obstacle_collision(config, spec, spec.obstacles) and tester.test_environment_bounds(config))

def sample(spec):
    samples = []
    samples += sample_double_grappled(spec)
    samples += uniform_sample(spec)
    double_samples = []
    for s in samples:
        angles = s.ee1_angles.copy()
        lengths = s.lengths.copy()
        lengths.reverse()
        double_samples.append(make_robot_config_from_ee2(s.points[0][0], s.points[0][1], angles, lengths, ee1_grappled=s.ee2_grappled, ee2_grappled=s.ee1_grappled))
    samples += double_samples
    Visualiser(spec, samples)
    return samples



def uniform_sample(spec):
    samples = []
    sampled = 0
    while (sampled < 300):
        grapple1 = np.random.choice(range(spec.num_grapple_points))

        angles = []
        for i in range(spec.num_segments):
            angles.append(Angle(radians=np.random.uniform(-11/12*math.pi, 11/12 * math.pi)))
        lengths = np.random.uniform(spec.min_lengths[0], spec.max_lengths[0], spec.num_segments)
        lengths = lengths.tolist()
        config = make_robot_config_from_ee1(spec.grapple_points[grapple1][0], spec.grapple_points[grapple1][1], angles, lengths, ee1_grappled=True)

        if (check_valid(config, spec)):
            samples.append(config)
            sampled += 1
    return samples

def sample_double_grappled(spec):
    samples = []
    for i in range(spec.num_grapple_points):
        for j in range(i+1, spec.num_grapple_points):
            for k in range(10):
                c1 = make_robot_config_from_ee1(spec.grapple_points[i][0], spec.grapple_points[i][1],[],[],True,False)
                c2 = make_robot_config_from_ee2(spec.grapple_points[j][0], spec.grapple_points[j][1],[],[],False,True)
                s = extend(spec,c1,c2, [0], 0)
                if s is not False:
                    samples.append(s)
    return samples

def angle_from_p1_to_p2(p1, p2):
    p1 = np.asarray(p1)
    p2 = np.asarray(p2)
    v = p2 - p1
    if v[0] < 0:
        return Angle(radians = np.arctan(v[1]/v[0]) + math.pi)
    return Angle(radians=np.arctan(v[1]/v[0]))

def test_collision_between_two_parts(config1, config2):
    point = config1.points[0]
    lines1 = []
    for i in range(1, len(config1.points)):
        lines1.append((point, config1.points[i]))
        point = config1.points[i]
    point = config2.points[0]
    lines2 = []
    for i in range(1, len(config2.points)):
        lines2.append((point, config2.points[i]))
        point = config2.points[i]
    for l1 in lines1:
        for l2 in lines2:
            if test_line_collision(l1,l2):
                return False
    return True

def extend(spec, config1, config2, total_trials, extended):
    while (total_trials[0] < 1000):
        remained_length = (spec.num_segments - extended) * spec.max_lengths[0]
        trials = 0
        if extended < spec.num_segments - 2:
            # remained_length = (spec.num_segments - extended) * spec.max_lengths[0]
            d = distance.euclidean(config1.points[-1], config2.points[0])
            if d > remained_length:
                return False
            angle = angle_from_p1_to_p2(config1.points[-1], config2.points[0])
            success = False
            while trials < 100 and not success:
                if remained_length > 2*d:
                    seg_angle = Angle(radians=np.random.rand() * math.pi * 2)
                    seg_length = spec.min_lengths[0]
                else:
                    seg_angle = Angle(radians=np.random.normal(angle.in_radians(), math.pi / 6))
                    seg_length = np.random.uniform(spec.min_lengths[0], spec.max_lengths[0])
                success = True
                new_angles = config1.ee1_angles + [seg_angle - sum(config1.ee1_angles)]
                new_lengths = config1.lengths + [seg_length]
                new_config1 = make_robot_config_from_ee1(config1.points[0][0], config1.points[0][1], new_angles, new_lengths, ee1_grappled=True)
                if distance.euclidean(new_config1.points[-1],config2.points[-1]) > remained_length - spec.max_lengths[0]:
                    success = False
                elif not check_valid(new_config1, spec):
                    success = False
                trials += 1
                total_trials[0] += 1
            if not success:
                return False
            remained_length -= spec.max_lengths[0]
            angle = angle_from_p1_to_p2(config2.points[0], new_config1.points[-1])
            success = False
            while trials < 100 and not success:
                if remained_length > 2*d:
                    seg_angle = Angle(radians= np.random.rand() * math.pi * 2)
                    seg_length = spec.min_lengths[0]
                else:
                    seg_angle = Angle(radians=np.random.normal(angle.in_radians(), math.pi / 6))
                    seg_length = np.random.uniform((spec.min_lengths[0], spec.max_lengths[0]))
                success = True
                new_angles = config2.ee2_angles + [seg_angle - sum(config2.ee2_angles)]
                new_lengths = config2.lengths + [seg_length]
                new_config2 = make_robot_config_from_ee2(config2.points[-1][0], config1.points[-1][1], new_angles, new_lengths, ee2_grappled=True)
                if not check_valid(new_config2, spec):
                    success = False
                trials += 1
                total_trials[0] += 1
            if not success:
                return False
            result = extend(spec, new_config1, new_config2, total_trials, extended + 2)
            if result is not False:
                return result
        elif spec.num_segments - extended == 2:
            p1 = config1.points[-1]
            p2 = config2.points[0]
            d = distance.euclidean(p1, p2)
            if d > 2*spec.max_lengths[0]:
                return False
            elif d > 2*spec.min_lengths[0]:
                length1 = d/2
                length2 = d/2
                angle1 = angle_from_p1_to_p2(p1,p2) - sum(config1.ee1_angles)
                angle2 = Angle(radians=0)
                angle3 = angle_from_p1_to_p2(p2, config2.points[1]) - angle_from_p1_to_p2(p1, p2)
                new_lengths = config1.lengths + [length1,length2] +config2.lengths
                new_angles = config1.ee1_angles + [angle1,angle2,angle3] + [angle for angle in config2.ee1_angles[1:]]
                combined_config = make_robot_config_from_ee1(config1.points[0][0], config1.points[0][1], new_angles, new_lengths,ee1_grappled=True, ee2_grappled=True)
                if check_valid((combined_config, spec)):
                    return combined_config
                else:
                    return False
            else:
                return False
        elif spec.num_segments - extended == 1:
            p1 = config1.points[-1]
            p2 = config2.points[0]
            d = distance.euclidean(p1, p2)
            if d > spec.max_lengths[0]:
                return False
            elif d > spec.min_lengths[0]:
                angle1 = angle_from_p1_to_p2(p1, p2) - sum(config1.ee1_angles)
                angle2 = angle_from_p1_to_p2(p2, config2.points[1]) - angle_from_p1_to_p2(p1, p2)
                new_lengths = config1.lengths + [d] + config2.lengths
                new_angles = config1.ee1_angles + [angle1, angle2] + [angle for angle in config2.ee2_angles[1:]]
                combined_config = make_robot_config_from_ee1(config1.points[0][0], config1.points[0][1], new_angles, new_lengths, ee1_grappled=True, ee2_grappled=True)
                if check_valid((combined_config, spec)):
                    return combined_config
                else:
                    return False
            else:
                return False
        return False


def get_p_steps(c1, c2, spec, d):
    steps = [c1]
    step_amount = math.ceil(d / 0.001)
    length_step = []
    angle1_step = []
    angle2_step = []
    for i in range(spec.num_segments):
        length_diff = c2.lengths[i] - c1.lengths[i]
        if length_diff != 0:
            length_step.append(length_diff / step_amount)
        else:
            length_step.append(0)
        angle1_diff = c2.ee1_angles[i] - c1.ee1_angles[i]
        if angle1_diff != 0:
            angle1_step.append(angle1_diff / step_amount)
        else:
            angle1_step.append(0)
        angle2_diff = c2.ee2_angles[i] - c1.ee2_angles[i]
        if angle2_diff != 0:
            angle2_step.append(angle2_diff / step_amount)
        else:
            angle2_step.append(0)
    for i in range(step_amount - 1):
        if c1.ee1_grappled and c2.ee1_grappled:

            new_lengths = list(map(add, steps[i].lengths, length_step))
            new_angles = list(map(add, steps[i].ee1_angles, angle1_step))
            new_config = make_robot_config_from_ee1(c1.points[0][0], c1.points[0][1], new_angles, new_lengths, True, False)
        else:
            new_lengths = list(map(add, steps[i].lengths, length_step))
            new_angles = list(map(add, steps[i].ee2_angles, angle2_step))
            new_config = make_robot_config_from_ee1(c1.points[-1][0], c1.points[-1][1], new_angles, new_lengths, False, True)

        if check_valid(new_config, spec):
            steps.append(new_config)
        else:
            return False
    steps += [c2]
    return steps




def connect(nodes, spec):
    for i in range(len(nodes)):
        distances = []

        for j in range(len(nodes)):
            distances.append(nodes[i].distance_to(nodes[j], spec))
        indices = np.argsort(distances)

        for k in range(3):
            if (i < indices[k]):
                n1 = nodes[i]
                n2 = nodes[indices[k]]
                steps = get_p_steps(n1.config, n2.config, spec, distances[indices[k]])
                if steps is not False:
                    GraphNode.add_connection(n1, n2)
                    n1.p_steps[n2] = steps
                    reversed = steps.copy()
                    reversed.reverse()
                    n2.p_steps[n1] = reversed



def find_graph_path(spec, init_node):
    """
    This method performs a breadth first search of the state graph and return a list of configs which form a path
    through the state graph between the initial and the goal. Note that this path will not satisfy the primitive step
    requirement - you will need to interpolate between the configs in the returned list.

    You may use this method in your solver if you wish, or can implement your own graph search algorithm to improve
    performance.

    :param spec: ProblemSpec object
    :param init_node: GraphNode object for the initial configuration
    :return: List of configs forming a path through the graph from initial to goal
    """
    # search the graph
    init_container = [init_node]

    # here, each key is a graph node, each value is the list of configs visited on the path to the graph node
    init_visited = {init_node: [init_node.config]}

    while len(init_container) > 0:
        current = init_container.pop(0)

        if current is None:
            pdb.set_trace()
        if test_config_equality(current.config, spec.goal, spec):
            # found path to goal
            return init_visited[current]

        successors = current.get_successors()
        for suc in successors:
            if suc not in init_visited:
                init_container.append(suc)
                init_visited[suc] = init_visited[current] + [suc.config]
                suc.parent = current

    return None


def main():
    input_file = "testcases/3g2_m1.txt"
    output_file = "out.txt"

    #input_file = "testcases/3g1_m2.txt"

    spec = ProblemSpec(input_file)


    init_node = GraphNode(spec, spec.initial)
    goal_node = GraphNode(spec, spec.goal)

    steps = []

    #
    #
    # Code for your main method can go here.
    #
    # Your code should find a sequence of RobotConfig objects such that all configurations are collision free, the
    # distance between 2 successive configurations is less than 1 primitive step, the first configuration is the initial
    # state and the last configuration is the goal state.

    samples = sample(spec)
    nodes = []
    for s in samples:
        nodes.append(GraphNode(spec, s))
    nodes.append(init_node)
    nodes.append(goal_node)

    connect(nodes, spec)
    steps = []
    if find_graph_path(spec, init_node) is not None:
        current = goal_node
        while (current.parent is not None):
            steps = current.parent.p_steps[current] + steps
            current = current.parent
    else:
        print("Failed")



    #
    #


    write_robot_config_list_to_file(output_file, steps)

    #
    # You may uncomment this line to launch visualiser once a solution has been found. This may be useful for debugging.
    # *** Make sure this line is commented out when you submit to Gradescope ***
    #


if __name__ == '__main__':
    main()
