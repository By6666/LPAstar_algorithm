# Lifelong RepairingiAstar algorithm

* Header file in the folder [include](./include).
 
* testing maps int the folder [inputs](./inputs).

* Code of reading map in the file [grid_input.cpp](./grid_input.cpp).

* Algorithm code file [LPAstar_algorithm.cpp](./LPAstar_algorithm.cpp).


Heuristic search methods promise to ﬁnd shortest paths for path-planning problems faster than uninformed search methods. Incremental search methods, on the other hand, promise to ﬁnd shortest paths for series of similar path-planning problems faster than is possible by solving each path-planning problem from scratch. Lifelong Planning A* (LPA*), an incremental version of A* that combines ideas from the artiﬁcial intelligence and the algorithms literature. It repeatedly ﬁnds shortest paths from a given start vertex to a given goal vertex while the edge costs of a graph change or vertices are added or deleted. Its ﬁrst search is the same as that of a version of A* that breaks ties in favor of vertices with smaller g-values but many of the subsequent searches are potentially faster because it reuses those parts of the previous search tree that are identical to the new one. 
