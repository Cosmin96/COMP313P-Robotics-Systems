from heapq import heappush, heappop

INFINITY = float('INFINITY')

def dijkstra(graph, source):
	n = len(graph)
	Q = [(0, source)]
	
	distances = [INFINITY for i in range(n)]
	distances[source] = 0

	while len(Q) != 0:
		(cost, node) = heappop(Q)

		for vertex in range(n):
			if distances[vertex] > distances[node] + graph[node][vertex]:
				distances[vertex] = distances[node] + graph[node][vertex]
				heappush(Q, (distances[vertex], vertex))

	return distances

def main():
	graph = [
	[0, 3, INFINITY, INFINITY, 5],
	[3, 0, 3, 9, 1],
	[INFINITY, 3, 0, 2, 2],
	[INFINITY, 9, 2, 0, INFINITY],
	[5, 1, 2, INFINITY, 0]
	]

	print(dijkstra(graph, 0))

if __name__ == "__main__":
	main()