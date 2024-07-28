expense_8_puzzle.py :

This program solves the 8-puzzle problem using several different search algorithms, including Breadth First Search, 
Depth First Search, Uniform Cost Search, Iterative Deepening Search, Greedy Search, Depth Limited Search and A\* Search. 
The user can choose which algorithm to use by specifying the desired method as a command line argument.

Usage
To run the program, navigate to the project directory and run the following command:

python expense_8_puzzle.py <start-file> <goal-file> <method> <dump-flag>
Example:
python expense_8_puzzle.py start.txt goal.txt a* true

<method> can be bfs, dfs, ucs, ids, dls, a*, greedy
a* will be considered as default method,if no method is passed.
search trace is dumped for analysis in trace-<date>-<time>.txt,if dump flag is set as true.
Both <method>, <dump-flag> are optional parameters and the default values are a*, false respectively.
Note-Sample start file and goal file are attched to the folder. 
