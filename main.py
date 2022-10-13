import math

AVERAGE_SPEED = 20  # km/hr


def haversine(start, destination):
    lat1 = math.radians(start.lat)
    long1 = math.radians(start.long)

    lat2 = math.radians(destination.lat)
    long2 = math.radians(destination.long)

    lat_diff = lat1 - lat2
    long_diff = long1 - long2

    a = (math.sin(lat_diff / 2) ** 2) + (
        math.cos(lat1) * math.cos(lat2) * (math.sin(long_diff / 2) ** 2)
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    R = 6371  # approx. radius of earth in kilometers
    return R * c


class Location:
    __slots__ = ["lat", "long"]

    def __init__(self, lat, long):
        self.lat = lat
        self.long = long


class Position:
    __slots__ = ["id", "location"]

    def __init__(self, id, location):
        self.id = id
        self.location = location

    def __str__(self):
        return str(self.id)


class Restaurant(Position):
    __slots__ = ["pt"]

    def __init__(self, id, location, pt):
        self.id = id
        self.location = location
        self.pt = pt


class Consumer(Position):
    def __init__(self, id, location):
        self.id = id
        self.location = location


class Order:
    __slots__ = ["consumer", "restaurant"]

    def __init__(self, restaurant, consumer):
        self.restaurant = restaurant
        self.consumer = consumer


class Node:
    """
    Vertex in the graph
    """

    __slots__ = ["id", "data"]

    def __init__(self, id, data):
        self.id = id
        self.data = data

    def __repr__(self):
        return self.id


class Edge:
    __slots__ = ["start", "end", "cost"]

    def __init__(self, start, end, cost=0):
        self.start = start
        self.end = end
        self.cost = cost

    def __str__(self):
        return f"{self.end.id}"

    def __repr__(self):
        return f"{self.end.id}"


class Graph:
    def __init__(self, edges, nodes, orders):
        self.edges = edges
        self.nodes = nodes
        self.orders = orders

        self.graph = {}
        self.paths = []

        for edge in self.edges:
            if edge.start.id in self.graph:
                self.graph[edge.start.id].append(edge.end)
            else:
                self.graph[edge.start.id] = [edge.end]

    def visit(self, start, path=[]):
        if start.id not in path:
            path = path + [start.id]
            if set(path) == set(list(self.graph.keys())):
                self.paths.append(path)
            for node in self.graph[start.id]:
                self.visit(node, path)

    def remove_invalid_paths(self):
        cons_rest = {order.consumer.id: order.restaurant.id for order in self.orders}
        accurate_paths = []
        for path in self.paths:
            for i, id in enumerate(path):
                invalid = False
                if isinstance(self.nodes[id].data, Consumer):
                    if cons_rest[id] not in path[: i + 1]:
                        invalid = True
                        break
            if not invalid:
                accurate_paths.append(path)
        self.paths = accurate_paths

    def find_min_cost_route(self):
        route_cost_map = []
        for path in self.paths:
            total_cost = 0
            for idx, id in enumerate(path):
                prev = None
                if idx > 0:
                    prev = self.nodes[path[idx - 1]]
                curr = self.nodes[id]
                if prev:
                    total_cost += (
                        haversine(prev.data.location, curr.data.location)
                        / AVERAGE_SPEED
                    )
                    if isinstance(curr.data, Restaurant):
                        if curr.data.pt > total_cost:
                            total_cost += (
                                curr.data.pt - total_cost
                            )  # considering preparation time in case of restaurant

            route_cost_map.append({"route": "->".join(path), "cost": total_cost})

        print(min(route_cost_map, key=lambda x: x.get("cost")))


def build_graph(start, orders):
    """
    Build undirected, unweighted graph from given orders and starting point of delivery person,
    under the assumption that delivery boy can travel from starting point only to any of the
    restaurant in the order.
    """

    edges = []
    nodes = {}

    start_node = Node(start.id, start)
    nodes.update({start.id: start_node})

    for order in orders:
        edges.append(Edge(start_node, Node(order.restaurant.id, order.restaurant)))

    for i in orders:
        nodes.update({i.restaurant.id: Node(i.restaurant.id, i.restaurant)})
        nodes.update({i.consumer.id: Node(i.consumer.id, i.consumer)})
        for j in orders:
            if i.restaurant.id != j.restaurant.id:
                edges.append(
                    Edge(
                        Node(i.restaurant.id, i.restaurant),
                        Node(j.restaurant.id, j.restaurant),
                    )
                )
            edges.append(
                Edge(
                    Node(i.restaurant.id, i.restaurant), Node(j.consumer.id, j.consumer)
                )
            )

            if i.consumer.id != j.consumer.id:
                edges.append(
                    Edge(
                        Node(i.consumer.id, i.consumer), Node(j.consumer.id, j.consumer)
                    )
                )
            edges.append(
                Edge(
                    Node(i.consumer.id, i.consumer), Node(j.restaurant.id, j.restaurant)
                )
            )

    graph = Graph(edges, nodes, orders)
    graph.visit(start_node)
    graph.remove_invalid_paths()
    graph.find_min_cost_route()


if __name__ == "__main__":
    """
    - This python script runs as-is.
    - Lat-long used are totally random.
    """

    start = Position("A", Location(41.507483, -99.436554))
    r1 = Restaurant(
        "R1", Location(38.504048, -98.315949), 1.5
    )  # 1.5 -> preparation time in hours
    r2 = Restaurant("R2", Location(-27.881403, 99.377138), 2)
    r3 = Restaurant("R3", Location(79.992667, 8.481794), 0.75)

    c1 = Consumer("C1", Location(-81.81566, -86.527669))
    c2 = Consumer("C2", Location(-6.490778, 102.238152))
    c3 = Consumer("C3", Location(-0.557782, 29.522093))

    o1 = Order(r1, c1)
    o2 = Order(r2, c2)
    o3 = Order(r3, c3)

    build_graph(start, [o1, o2])
    build_graph(start, [o1, o2, o3])
