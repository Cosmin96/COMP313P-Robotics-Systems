from heapq import heappush, heappop

def insert(queue, n):
	heappush(queue, n)
	#queue.sort()

def getFirst(queue):
	return heappop(queue)

def main():
	queue = []
	elements = [6, 2, 4, 12, 10, 11, 1]
	for i in range(0, len(elements)):
		insert(queue, elements[i])
	
	print(queue)

	for i in range(0, len(queue)):
		print(getFirst(queue))
	print(queue)


if __name__ == "__main__":
	main()